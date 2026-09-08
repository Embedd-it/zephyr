/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Gerson Fernando Budke <nandojve@gmail.com>
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip USB High-Speed (USBHS) device controller driver.
 *
 */

#define DT_DRV_COMPAT microchip_usbhs_g2_udc

#include "udc_common.h"

#include <string.h>

#include <soc.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/irq.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_mchp_usbhs_g2, CONFIG_UDC_DRIVER_LOG_LEVEL);

/* Endpoint count macros. */
#define MAX_ENDPOINTS           10u
#define CLOCK_USABLE_TIMEOUT_US 10000u
#define BANK_KILL_TIMEOUT_US    1000u
#define DMA_SETTLE_TIMEOUT_US   1000u
#define EP_BITMAP_SHIFT         16u

/* The peripheral's DMA channels are hard-wired one-to-one to hardware
 * endpoints 1..USBHS_DEVDMA_NUMBER. EP0 and any endpoint beyond the last
 * channel have no DMA channel and always use the CPU-copy path.
 */
#define DMA_FIRST_EP 1u
#define DMA_LAST_EP  6u

/* Driver events for thread synchronization */
enum udc_mchp_usbhs_event_type {
	UDC_MCHP_USBHS_G2_EVENT_TYPE_SETUP = 0u,
	UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_NEW,
	UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_FINISHED,
	UDC_MCHP_USBHS_G2_EVENT_TYPE_OUT_PENDING,
};

struct udc_mchp_usbhs_config {
	/* Pipe/endpoint fifo RAM (32KB) base address */
	mem_addr_t fifo_base;

	/* Clock driver */
	const struct device *clock;

	/* Clock subsystem for this instance's own channel */
	clock_control_subsys_t clock_subsys;

	/* Register block */
	usbhs_registers_t *regs;

	/* Whether the High Speed mode is disabled */
	bool low_power;

	/* Function to enable the peripheral's interrupt */
	void (*irq_config_enable)(const struct device *dev);

	/* Function to disable the peripheral's interrupt */
	void (*irq_config_disable)(const struct device *dev);

	/* Function to start the peripheral's thread */
	void (*make_thread)(const struct device *dev);

	/* Number of configured endpoints (total distinct hardware endpoints) */
	size_t ep_num;

	/* Number of IN-capable endpoints, including EP0 IN. These occupy hardware endpoints
	 * 0..(num_of_in_eps - 1).
	 */
	size_t num_of_in_eps;

	/* Number of OUT-capable endpoints, including EP0 OUT. These occupy hardware endpoint 0 and
	 * num_of_in_eps..(ep_num - 1).
	 */
	size_t num_of_out_eps;

	/* Pointers to endpoint configuration structures */
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
};

struct udc_mchp_usbhs_data {
	/* Peripheral's thread data */
	struct k_thread thread_data;

	/* Thread event object */
	struct k_event events;

	/* Bitmap for endpoints where transfers have been queued */
	atomic_t xfer_new;

	/* Bitmap for endpoints where transfers have finished */
	atomic_t xfer_finished;

	/* Bitmap for endpoints whose last OUT transfer was truncated because the host sent more
	 * data than the buffer had room for.
	 */
	atomic_t out_truncated;

	/* Bitmap for endpoints whose last received isochronous OUT packet was flagged with a
	 * CRC error by hardware.
	 */
	atomic_t out_crc_error;

	/* Bitmap for endpoints where transfers output is waiting */
	atomic_t out_pending;

	/* Bitmap for endpoints where transfers setup is waiting */
	atomic_t setup_pending;

	/* Setup transaction data */
	uint8_t setup[8];

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	/* Size in bytes of the bank currently being read, 0 when the channel is idle */
	uint16_t dma_out_len[USBHS_DEVDMA_NUMBER];

	/* Size in bytes of the packet held by the bank, used to detect short packets */
	uint16_t dma_out_pkt[USBHS_DEVDMA_NUMBER];

	/* For DMA-driven IN transfers, indexed by DMA channel: number of TXINI (actual packet-sent)
	 * events still expected before the transmission is over.
	 */
	uint8_t dma_in_pkts_pending[USBHS_DEVDMA_NUMBER];
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */
};

static int udc_mchp_usbhs_process_pending_out(const struct device *dev, uint8_t ep_addr);
static int udc_mchp_usbhs_disable(const struct device *dev);

/*
 * Logical endpoint address to hardware endpoint index.
 */
static inline uint8_t udc_mchp_usbhs_ep_addr_to_hw_ep(uint8_t ep_addr)
{
	/* Direct mapping: USB endpoint number = HW endpoint index */
	return USB_EP_GET_IDX(ep_addr);
}

/*
 * Reverse mapping: hardware endpoint index to logical endpoint address. Used in ISR to get the
 * correct endpoint configuration.
 */
static inline uint8_t udc_mchp_usbhs_hw_ep_to_ep_addr(uint8_t hw_ep, bool is_in)
{
	if (hw_ep == 0u) {
		return is_in ? USB_EP_DIR_IN : 0u;
	}

	/* Direct mapping: HW EP index = USB endpoint number */
	return (is_in ? USB_EP_DIR_IN : 0u) | hw_ep;
}

/*
 * Endpoint address to bitmap index
 */
static inline int udc_mchp_usbhs_ep_to_bit(uint8_t ep)
{
	uint8_t hw_ep = udc_mchp_usbhs_ep_addr_to_hw_ep(ep);

	if (USB_EP_DIR_IS_IN(ep)) {
		return EP_BITMAP_SHIFT + hw_ep;
	}

	return hw_ep;
}

/*
 * Reverse mapping: bitmap index to endpoint address
 */
static inline uint8_t udc_mchp_usbhs_bit_to_ep(uint32_t *bitmap)
{
	unsigned int bit = 0u;
	uint8_t hw_ep = 0u;
	bool is_in = false;

	__ASSERT_NO_MSG(bitmap && *bitmap);

	bit = find_lsb_set(*bitmap) - 1u;
	*bitmap &= ~BIT(bit);

	if (bit >= EP_BITMAP_SHIFT) {
		hw_ep = bit - EP_BITMAP_SHIFT;
		is_in = true;
	} else {
		hw_ep = bit;
		is_in = false;
	}

	return udc_mchp_usbhs_hw_ep_to_ep_addr(hw_ep, is_in);
}

/*
 * Get the fifo data corresponding to the endpoint index
 */
static inline volatile uint8_t *
udc_mchp_usbhs_ep_fifo(const struct udc_mchp_usbhs_config *dev_config, uint8_t ep_idx)
{
	const mem_addr_t fifo_ep_offset = 0x8000u;

	return (volatile uint8_t *)(dev_config->fifo_base + (ep_idx * fifo_ep_offset));
}

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
static inline bool udc_mchp_usbhs_ep_has_dma(uint8_t ep_idx)
{
	/* DMA can only be used on non-control endpoints and up until a certain number */
	return ep_idx >= DMA_FIRST_EP && ep_idx <= DMA_LAST_EP;
}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

/*
 * Enable the peripheral clock
 */
static int udc_mchp_usbhs_enable_clock(const struct udc_mchp_usbhs_config *dev_config)
{
	int ret = 0;

	/* Enable 480 MHz UPLL */
	ret = clock_control_on(dev_config->clock,
			       (clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_UPLL, 0u));
	if (ret != 0) {
		LOG_ERR("Unable to enable UPLL clock: %i", ret);
		return ret;
	}

	LOG_DBG("USB PLL locked");

	/* If in low power mode use the slower USB CLK */
	if (dev_config->low_power) {
		ret = clock_control_on(
			dev_config->clock,
			(clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_USB_CLK, 0u));
		if (ret != 0) {
			LOG_ERR("Unable to enable USB CLK clock: %i", ret);
			return ret;
		}
		LOG_DBG("USB 48MHz clock enabled for Full-Speed mode");
	}

	return 0;
}

/*
 * Disable the peripheral clock
 */
static int udc_mchp_usbhs_disable_clock(const struct udc_mchp_usbhs_config *dev_config)
{
	int ret = 0;

	if (dev_config->low_power) {
		ret = clock_control_off(
			dev_config->clock,
			(clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_USB_CLK, 0u));
		if (ret != 0) {
			LOG_ERR("Unable to disable USB CLK clock: %i", ret);
			return ret;
		}
	}

	ret = clock_control_off(dev_config->clock,
				(clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_UPLL, 0u));
	if (ret != 0) {
		LOG_ERR("Unable to disable UPLL: %i", ret);
		return ret;
	}

	LOG_DBG("USB PLL disabled");

	return 0;
}

/*
 * Reset the endpoint
 */
static void udc_mchp_usbhs_ep_reset(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	unsigned int lock_key = irq_lock();

	regs->USBHS_DEVEPT |= BIT(USBHS_DEVEPT_EPRST0_Pos + ep_idx);
	regs->USBHS_DEVEPT &= ~BIT(USBHS_DEVEPT_EPRST0_Pos + ep_idx);
	barrier_dsync_fence_full();

	irq_unlock(lock_key);
}

/*
 *  Check if the endpoint is configured
 */
static bool udc_mchp_usbhs_ep_is_configured(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;

	return (regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_CFGOK_Msk) != 0u;
}

/*
 *  Prepare the endpoint for output operation
 */
static int udc_mchp_usbhs_prep_out(const struct device *dev, struct net_buf *buf,
				   struct udc_ep_config *ep_cfg)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	uint8_t ep_idx = udc_mchp_usbhs_ep_addr_to_hw_ep(ep_cfg->addr);
	uint32_t isr = 0u;
	uint32_t nbusybk = 0u;
	uint32_t dtseq = 0u;
#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);

	/* The DMA is armed one bank at a time from the RXOUTI interrupt. If a bank already contains
	 * data RXOUTI will fire immediately.
	 */
	if (udc_mchp_usbhs_ep_has_dma(ep_idx)) {
		LOG_DBG("Prepare OUT ep 0x%02x via DMA", ep_cfg->addr);
		atomic_clear_bit(&dev_data->out_pending, udc_mchp_usbhs_ep_to_bit(ep_cfg->addr));
		regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;

		return 0;
	}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	ARG_UNUSED(buf);

	isr = regs->USBHS_DEVEPTISR[ep_idx];
	nbusybk = (isr & USBHS_DEVEPTISR_NBUSYBK_Msk) >> USBHS_DEVEPTISR_NBUSYBK_Pos;
	dtseq = (isr & USBHS_DEVEPTISR_DTSEQ_Msk) >> USBHS_DEVEPTISR_DTSEQ_Pos;

	LOG_DBG("Prepare OUT ep 0x%02x (hw=%d) ISR=0x%08x IMR=0x%08x "
		"CFG=0x%08x NBUSYBK=%d DTSEQ=%d",
		ep_cfg->addr, ep_idx, isr, regs->USBHS_DEVEPTIMR[ep_idx],
		regs->USBHS_DEVEPTCFG[ep_idx], nbusybk, dtseq);

	/* Ensure RXOUTE interrupt is enabled. This is necessary because:
	 * - The interrupt may have been disabled by the ISR when deferring
	 * - For multi-packet transfers, we need to keep receiving
	 */
	regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;

	return 0;
}

/*
 *  Prepare the endpoint for input operation
 */
static int udc_mchp_usbhs_prep_in(const struct device *dev, struct net_buf *buf,
				  struct udc_ep_config *ep_cfg)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	uint8_t ep_idx = udc_mchp_usbhs_ep_addr_to_hw_ep(ep_cfg->addr);
	volatile uint8_t *fifo = NULL;
	uint32_t len = 0u;

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	usbhs_devdma_registers_t *dma = &regs->USBHS_DEVDMA[ep_idx - DMA_FIRST_EP];
	uint8_t ch = 0u;
	uint16_t mps = 0u;
	uint32_t mps_len = 0u;

	/* A zero-length buffer (ZLP-only) falls through to the CPU path below: it reuses the
	 * existing FIFOCON-clear-with-empty-bank ZLP sequence. This also covers the case of
	 * BUFF_LENGTH == 0.
	 */
	if (udc_mchp_usbhs_ep_has_dma(ep_idx) && buf->len > 0u) {
		dma = &regs->USBHS_DEVDMA[ep_idx - DMA_FIRST_EP];
		ch = ep_idx - DMA_FIRST_EP;
		mps = udc_mps_ep_size(ep_cfg);
		mps_len = MIN(buf->len, mps);

		LOG_DBG("Prepare IN ep 0x%02x length %u via DMA", ep_cfg->addr, buf->len);

		/* DMA loads banks well ahead of actual transmission, so completion must be detected
		 * by tracking how many packets have passed by a TXINI event.
		 */
		dev_data->dma_in_pkts_pending[ch] = (buf->len + mps - 1u) / mps;

		sys_cache_data_flush_range(buf->data, buf->len);

		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_TXINIC_Msk;
		regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_TXINES_Msk;

		dma->USBHS_DEVDMAADDRESS =
			USBHS_DEVDMAADDRESS_BUFF_ADD((uint32_t)(uintptr_t)buf->data);
		barrier_dsync_fence_full();

		dma->USBHS_DEVDMACONTROL = USBHS_DEVDMACONTROL_BUFF_LENGTH(buf->len) |
					   USBHS_DEVDMACONTROL_END_B_EN_Msk |
					   USBHS_DEVDMACONTROL_CHANN_ENB_Msk;

		/* Wait for at least the first packet's worth of bytes to be transferred before
		 * continuing with the transaction
		 */
		if (!WAIT_FOR(((dma->USBHS_DEVDMASTATUS & USBHS_DEVDMASTATUS_BUFF_COUNT_Msk) >>
			       USBHS_DEVDMASTATUS_BUFF_COUNT_Pos) <= (buf->len - mps_len),
			      DMA_SETTLE_TIMEOUT_US, k_busy_wait(1u))) {
			LOG_ERR("DMA IN ep 0x%02x: first packet not loaded within timeout",
				ep_cfg->addr);
			return -ETIMEDOUT;
		}

		/* Clear FIFOCON to trigger the IN transaction. Always, as DMA is not applicable for
		 * EP 0
		 */
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;

		return 0;
	}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	fifo = udc_mchp_usbhs_ep_fifo(dev_config, ep_idx);
	len = MIN(buf->len, udc_mps_ep_size(ep_cfg));
	LOG_DBG("Prepare IN ep 0x%02x length %u (total %u)", ep_cfg->addr, len, buf->len);

	for (size_t idx = 0u; idx < len; idx++) {
		fifo[idx] = buf->data[idx];
	}
	barrier_dsync_fence_full();

	/* Clear the TXINI interrupt flag and enable the interrupt.
	 * The interrupt was disabled by the ISR when the previous transfer
	 * completed, so we must re-enable it for every new transfer.
	 */
	regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_TXINIC_Msk;
	regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_TXINES_Msk;

	if (ep_idx != 0u) {
		/* Clear FIFOCON to trigger the IN transaction */
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
	}

	return 0;
}

/*
 * Handle IN operation
 */
static int udc_mchp_usbhs_handle_evt_din(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	struct net_buf *buf = NULL;

	buf = udc_buf_get(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buffer for ep 0x%02x", ep_cfg->addr);
		return -ENOBUFS;
	}

	udc_ep_set_busy(ep_cfg, false);

	return udc_submit_ep_event(dev, buf, 0);
}

/*
 * Handle OUT operation
 */
static int udc_mchp_usbhs_handle_evt_dout(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	int bit = udc_mchp_usbhs_ep_to_bit(ep_cfg->addr);
	struct net_buf *buf = NULL;
	int ret = 0;

	buf = udc_buf_get(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buffer for OUT ep 0x%02x", ep_cfg->addr);
		return -ENODATA;
	}

	if (atomic_test_and_clear_bit(&dev_data->out_truncated, bit)) {
		ret = -ENOSPC;
	} else if (atomic_test_and_clear_bit(&dev_data->out_crc_error, bit)) {
		ret = -EILSEQ;
	}

	udc_ep_set_busy(ep_cfg, false);

	return udc_submit_ep_event(dev, buf, ret);
}

/*
 * Push another transfer
 */
static void udc_mchp_usbhs_handle_xfer_next(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	struct net_buf *buf = NULL;
	struct udc_buf_info *bi = NULL;
	int ret = 0;

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		return;
	}

	if (ep_cfg->addr == USB_CONTROL_EP_OUT) {
		bi = udc_get_buf_info(buf);

		if (bi->setup != 0u) {
			return;
		}
	}

	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		ret = udc_mchp_usbhs_prep_out(dev, buf, ep_cfg);
	} else {
		ret = udc_mchp_usbhs_prep_in(dev, buf, ep_cfg);
	}

	if (ret != 0) {
		buf = udc_buf_get(ep_cfg);
		udc_submit_ep_event(dev, buf, -ECONNREFUSED);
	} else {
		udc_ep_set_busy(ep_cfg, true);
	}
}

/*
 * Driver thread
 */
static ALWAYS_INLINE void udc_mchp_usbhs_thread_handler(const struct device *dev)
{
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_ep_config *ep_cfg = NULL;
	uint32_t evt = 0u;
	uint32_t eps = 0u;
	uint8_t ep = 0u;
	int ret = 0;

	evt = k_event_wait(&dev_data->events, UINT32_MAX, false, K_FOREVER);
	udc_lock_internal(dev, K_FOREVER);

	if ((evt & BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_FINISHED)) != 0u) {
		k_event_clear(&dev_data->events, BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_FINISHED));

		eps = atomic_clear(&dev_data->xfer_finished);

		while (eps != 0u) {
			ep = udc_mchp_usbhs_bit_to_ep(&eps);
			ep_cfg = udc_get_ep_cfg(dev, ep);
			LOG_DBG("Finished event ep 0x%02x", ep);

			if (USB_EP_DIR_IS_IN(ep)) {
				ret = udc_mchp_usbhs_handle_evt_din(dev, ep_cfg);
			} else {
				ret = udc_mchp_usbhs_handle_evt_dout(dev, ep_cfg);
			}

			if (ret != 0) {
				udc_submit_event(dev, UDC_EVT_ERROR, ret);
			}

			if (!udc_ep_is_busy(ep_cfg)) {
				udc_mchp_usbhs_handle_xfer_next(dev, ep_cfg);
			} else {
				LOG_DBG("Endpoint 0x%02x busy", ep);
			}
		}
	}

	if ((evt & BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_NEW)) != 0u) {
		k_event_clear(&dev_data->events, BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_NEW));

		eps = atomic_clear(&dev_data->xfer_new);

		while (eps != 0u) {
			ep = udc_mchp_usbhs_bit_to_ep(&eps);
			ep_cfg = udc_get_ep_cfg(dev, ep);
			LOG_INF("New transfer ep 0x%02x in the queue", ep);

			if (!udc_ep_is_busy(ep_cfg)) {
				udc_mchp_usbhs_handle_xfer_next(dev, ep_cfg);
			} else {
				LOG_DBG("Endpoint 0x%02x busy", ep);
			}
		}
	}

	if ((evt & BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_SETUP)) != 0u) {
		k_event_clear(&dev_data->events, BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_SETUP));
		atomic_clear(&dev_data->setup_pending);

		udc_setup_received(dev, dev_data->setup);

		/* Re enable RXSTPES to accept the next SETUP packet */
		regs->USBHS_DEVEPTIER[0] = USBHS_DEVEPTIER_CTRL_RXSTPES_Msk;
	}

	/* Process pending OUT data. This must be checked AFTER SETUP processing
	 * because SETUP allocates the buffer that pending OUT data needs.
	 */
	if ((evt & BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_OUT_PENDING)) != 0u) {
		k_event_clear(&dev_data->events, BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_OUT_PENDING));

		eps = atomic_clear(&dev_data->out_pending);

		while (eps != 0u) {
			ep = udc_mchp_usbhs_bit_to_ep(&eps);
			LOG_INF("Pending OUT data for ep 0x%02x", ep);
			ret = udc_mchp_usbhs_process_pending_out(dev, ep);
			if (ret != 0 && ret != -ENOBUFS) {
				udc_submit_event(dev, UDC_EVT_ERROR, ret);
			}
		}
	}

	udc_unlock_internal(dev);
}

/*
 * Handle setup interrupt request
 */
static void udc_mchp_usbhs_handle_setup_isr(const struct device *dev)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	volatile uint8_t *fifo = udc_mchp_usbhs_ep_fifo(dev_config, 0u);
	uint32_t byte_count = 0u;

	byte_count =
		(regs->USBHS_DEVEPTISR[0] & USBHS_DEVEPTISR_BYCT_Msk) >> USBHS_DEVEPTISR_BYCT_Pos;

	if (byte_count == 8u) {
		for (size_t idx = 0u; idx < 8u; idx++) {
			dev_data->setup[idx] = fifo[idx];
		}
		atomic_set(&dev_data->setup_pending, 1u);
		/* Mask RXSTP until the thread actually reads the setup data */
		regs->USBHS_DEVEPTIDR[0] = USBHS_DEVEPTIDR_CTRL_RXSTPEC_Msk;
	} else {
		LOG_ERR("Wrong byte count %u for setup packet", byte_count);
		/* Still proceed and handle the interrupt*/
	}

	regs->USBHS_DEVEPTICR[0] = USBHS_DEVEPTICR_CTRL_RXSTPIC_Msk;
}

/*
 * Process deferred OUT data in thread context.
 * Called when ISR received data but no buffer was available at that time. The ISR disabled RXOUTE
 * and set out_pending flag. This function retries processing now that buffers may be available.
 */
static int udc_mchp_usbhs_process_pending_out(const struct device *dev, uint8_t ep_addr)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	uint8_t ep_idx = udc_mchp_usbhs_ep_addr_to_hw_ep(ep_addr);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	volatile uint8_t *fifo = udc_mchp_usbhs_ep_fifo(dev_config, ep_idx);
	struct net_buf *buf = NULL;
	uint32_t byte_count = 0u;
	uint32_t size = 0u;
	bool is_ctrl = (ep_idx == 0u);
	bool is_iso = (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_ISO;
	uint8_t *data = NULL;
	unsigned int lock_key = irq_lock();

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	/* Enabling RXOUTE will fire an ISR that will arm the DMA for the bank that is waiting */
	if (udc_mchp_usbhs_ep_has_dma(ep_idx)) {
		if (udc_buf_peek(ep_cfg) == NULL) {
			atomic_set_bit(&dev_data->out_pending, udc_mchp_usbhs_ep_to_bit(ep_addr));
			irq_unlock(lock_key);
			return -ENOBUFS;
		}

		regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;
		irq_unlock(lock_key);
		return 0;
	}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	/* Check if data is available:
	 * - Control EP0: Check RXOUTI (FIFOCON is always 0)
	 * - Bulk/Int/Iso: Check FIFOCON
	 */
	if (is_ctrl) {
		if ((regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_RXOUTI_Msk) == 0u) {
			LOG_DBG("EP0 OUT no data (RXOUTI=0)");
			/* Re-enable RXOUTE since no data to process */
			regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;
			irq_unlock(lock_key);
			return 0;
		}
	} else {
		if ((regs->USBHS_DEVEPTIMR[ep_idx] & USBHS_DEVEPTIMR_FIFOCON_Msk) == 0u) {
			LOG_DBG("OUT ep 0x%02x no data in bank (FIFOCON=0)", ep_addr);
			irq_unlock(lock_key);
			return 0;
		}
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		/* No buffer available. Leave data in FIFO. Host will get NAK for subsequent
		 * packets. Keep pending bit set to retry when buffer available.
		 */
		if (is_ctrl) {
			/* For EP0, keep RXOUTE disabled, keep RXOUTI set */
			LOG_WRN("EP0 OUT no buffer - NAK until ready");
		} else {
			LOG_WRN("OUT ep 0x%02x no buffer - NAK until ready", ep_addr);
		}
		atomic_set_bit(&dev_data->out_pending, udc_mchp_usbhs_ep_to_bit(ep_addr));
		irq_unlock(lock_key);
		return -ENOBUFS;
	}

	/* Isochronous OUT has no protocol-level retry, so CRCERRI is the only signal that the bytes
	 * about to be read out of the bank are corrupt. This behaviour is specific to ISO
	 * transactions.
	 */
	if (is_iso && (regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_ISO_CRCERRI_Msk) != 0u) {
		LOG_WRN("OUT ep 0x%02x: CRC error, packet corrupted", ep_addr);
		atomic_set_bit(&dev_data->out_crc_error, udc_mchp_usbhs_ep_to_bit(ep_addr));
		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_ISO_CRCERRIC_Msk;
	}

	/* Read byte count */
	byte_count = (regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_BYCT_Msk) >>
		     USBHS_DEVEPTISR_BYCT_Pos;

	LOG_DBG("Thread OUT ep 0x%02x byte_count %u room %u", ep_addr, byte_count,
		net_buf_tailroom(buf));

	size = MIN(byte_count, net_buf_tailroom(buf));
	if (byte_count > size) {
		LOG_ERR("OUT ep 0x%02x overflow: %u bytes received, only %u fit", ep_addr,
			byte_count, size);
		atomic_set_bit(&dev_data->out_truncated, udc_mchp_usbhs_ep_to_bit(ep_addr));
	}

	/* Read data from FIFO */
	data = net_buf_add(buf, size);

	for (size_t idx = 0u; idx < size; idx++) {
		data[idx] = fifo[idx];
	}

	/* Release the bank and re-enable interrupts.
	 * ISR left RXOUTI set when deferring, clear it now.
	 */
	regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_RXOUTIC_Msk;
	if (!is_ctrl) {
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
	}
	regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;

	if (net_buf_tailroom(buf) == 0u || size < udc_mps_ep_size(ep_cfg)) {
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_RXOUTEC_Msk;
		/* Transfer complete - buffer full or short packet received */
		atomic_set_bit(&dev_data->xfer_finished, udc_mchp_usbhs_ep_to_bit(ep_addr));
		k_event_post(&dev_data->events, BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_FINISHED));
	}

	irq_unlock(lock_key);

	return 0;
}

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
/*
 * Start the DMA for the OUT bank that raised RXOUTI.
 */
static void udc_mchp_usbhs_out_dma_start(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	uint8_t ep_addr = udc_mchp_usbhs_hw_ep_to_ep_addr(ep_idx, false);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	uint8_t ch = ep_idx - DMA_FIRST_EP;
	usbhs_devdma_registers_t *dma = &regs->USBHS_DEVDMA[ch];
	int bit = udc_mchp_usbhs_ep_to_bit(ep_addr);
	bool is_iso = (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_ISO;
	struct net_buf *buf = NULL;
	uint32_t byte_count = 0u;
	uint32_t size = 0u;
	uint8_t *dst = NULL;

	/* The channel is already reading this bank */
	if (dev_data->dma_out_len[ch] != 0u) {
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_RXOUTEC_Msk;
		return;
	}

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		/* No buffer available. Disable RXOUTE until a transfer is enqueued. */
		LOG_DBG("ISR OUT ep 0x%02x no buffer - defer", ep_addr);
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_RXOUTEC_Msk;
		atomic_set_bit(&dev_data->out_pending, bit);
		return;
	}

	/* Isochronous OUT has no protocol-level retry, so CRCERRI is the only signal that the bytes
	 * about to be read out of the bank are corrupt. This behaviour is specific to ISO
	 * transactions.
	 */
	if (is_iso && (regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_ISO_CRCERRI_Msk) != 0u) {
		LOG_WRN("OUT ep 0x%02x: CRC error, packet corrupted", ep_addr);
		atomic_set_bit(&dev_data->out_crc_error, bit);
		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_ISO_CRCERRIC_Msk;
	}

	byte_count = (regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_BYCT_Msk) >>
		     USBHS_DEVEPTISR_BYCT_Pos;
	size = MIN(byte_count, net_buf_tailroom(buf));
	if (byte_count > size) {
		LOG_ERR("OUT ep 0x%02x overflow: %u bytes received, only %u fit", ep_addr,
			byte_count, size);
		atomic_set_bit(&dev_data->out_truncated, bit);
	}

	/* BUFF_LENGTH == 0 means 32 KB to the DMA: a zero-length packet (or no room left) is
	 * released here and ends the transfer.
	 */
	if (size == 0u) {
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_RXOUTEC_Msk;
		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_RXOUTIC_Msk;
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
		atomic_set_bit(&dev_data->xfer_finished, bit);
		return;
	}

	/* No further RXOUTI handling until the DMA is done with this bank */
	regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_RXOUTEC_Msk;

	dev_data->dma_out_len[ch] = size;
	dev_data->dma_out_pkt[ch] = byte_count;

	dst = net_buf_tail(buf);

	/* Cache line(s) covering the destination must be clean before the DMA works with them */
	sys_cache_data_invd_range(dst, size);

	dma->USBHS_DEVDMAADDRESS = USBHS_DEVDMAADDRESS_BUFF_ADD((uint32_t)(uintptr_t)dst);
	barrier_dsync_fence_full();

	/* Run and stop at end of buffer, interrupting when BUFF_COUNT reaches 0 */
	dma->USBHS_DEVDMACONTROL = USBHS_DEVDMACONTROL_BUFF_LENGTH(size) |
				   USBHS_DEVDMACONTROL_END_BUFFIT_Msk |
				   USBHS_DEVDMACONTROL_CHANN_ENB_Msk;
}

/*
 * Complete the DMA read of an OUT bank.
 * Accounts the received bytes, releases the bank and either finishes the transfer or re-enables
 * RXOUTE for the next bank.
 */
static void udc_mchp_usbhs_out_dma_done(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	uint8_t ep_addr = udc_mchp_usbhs_hw_ep_to_ep_addr(ep_idx, false);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	uint8_t ch = ep_idx - DMA_FIRST_EP;
	usbhs_devdma_registers_t *dma = &regs->USBHS_DEVDMA[ch];
	int bit = udc_mchp_usbhs_ep_to_bit(ep_addr);
	struct net_buf *buf = NULL;
	uint32_t status = 0u;
	uint32_t remaining = 0u;
	uint32_t received = 0u;
	uint8_t *dst = NULL;

	/* Reading DEVDMASTATUS clears the interrupt */
	status = dma->USBHS_DEVDMASTATUS;

	if (dev_data->dma_out_len[ch] == 0u) {
		/* Spurious, or the transfer was cancelled meanwhile */
		return;
	}

	if ((status & (USBHS_DEVDMASTATUS_CHANN_ENB_Msk | USBHS_DEVDMASTATUS_CHANN_ACT_Msk)) !=
	    0u) {
		/* Not done yet, wait for the end of buffer interrupt */
		return;
	}

	buf = udc_buf_peek(ep_cfg);

	remaining =
		(status & USBHS_DEVDMASTATUS_BUFF_COUNT_Msk) >> USBHS_DEVDMASTATUS_BUFF_COUNT_Pos;
	received = dev_data->dma_out_len[ch] - MIN(remaining, dev_data->dma_out_len[ch]);

	if (buf != NULL) {
		dst = net_buf_tail(buf);
		sys_cache_data_invd_range(dst, received);
		net_buf_add(buf, received);
	}

	/* Release the bank; RXOUTI and FIFOCON now reflect the next bank */
	regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_RXOUTIC_Msk;
	regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;

	dma->USBHS_DEVDMACONTROL = 0u;
	dev_data->dma_out_len[ch] = 0u;

	if (buf == NULL || net_buf_tailroom(buf) == 0u ||
	    dev_data->dma_out_pkt[ch] < udc_mps_ep_size(ep_cfg) ||
	    atomic_test_bit(&dev_data->out_truncated, bit)) {
		/* Transfer complete - buffer full or short packet received */
		atomic_set_bit(&dev_data->xfer_finished, bit);
		return;
	}

	/* If the next bank is already full RXOUTI is set and the ISR fires immediately */
	regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;
}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

/*
 * OUT ISR handler - optimized for minimal thread wake-ups.
 *
 * DMA-capable endpoints take a different path: `udc_mchp_usbhs_prep_out` arms DMA with a whole
 * buffer, so the ISR just reads back how much of it the channel actually received and finishes the
 * transfer.
 *
 * All other endpoints (including EP0) are processed in ISR by the CPU when a buffer is available.
 * Uses NBUSYBK to process multiple packets per ISR for dual-banked endpoints.
 *
 * For CONTROL endpoints (EP0) - single bank:
 *   - Read data, clear RXOUTI
 *
 * For BULK/INT/ISO endpoints - dual bank with FIFOCON:
 *   - Loop while NBUSYBK != 0 to drain all available packets
 *   - Clear RXOUTI, read data, clear FIFOCON for each packet
 */
static void udc_mchp_usbhs_handle_out_isr(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	uint8_t ep_addr = udc_mchp_usbhs_hw_ep_to_ep_addr(ep_idx, false);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	volatile uint8_t *fifo = udc_mchp_usbhs_ep_fifo(dev_config, ep_idx);
	struct net_buf *buf = NULL;
	uint32_t byte_count = 0u;
	uint32_t size = 0u;
	uint8_t *data = NULL;
	bool is_ctrl = (ep_idx == 0u);
	bool is_iso = (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_ISO;

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	/* The DMA path is handled with normal interrupts, filtered by which endpoints are
	 * configured to use DMA.
	 */
	if (udc_mchp_usbhs_ep_has_dma(ep_idx)) {
		udc_mchp_usbhs_out_dma_start(dev, ep_idx);
		return;
	}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	do {
		buf = udc_buf_peek(ep_cfg);
		if (buf == NULL) {
			/* No buffer available. Disable RXOUTE to stop
			 * interrupts. Will be re-enabled when buffer queued.
			 */
			LOG_DBG("ISR OUT ep 0x%02x no buffer - defer", ep_addr);
			regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_RXOUTEC_Msk;
			atomic_set_bit(&dev_data->out_pending, udc_mchp_usbhs_ep_to_bit(ep_addr));
			return;
		}

		/* Isochronous OUT has no protocol-level retry, so CRCERRI is the only signal that
		 * the bytes about to be read out of the bank are corrupt. This behaviour is
		 * specific to ISO transactions.
		 */
		if (is_iso &&
		    (regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_ISO_CRCERRI_Msk) != 0u) {
			LOG_WRN("OUT ep 0x%02x: CRC error, packet corrupted", ep_addr);
			atomic_set_bit(&dev_data->out_crc_error, udc_mchp_usbhs_ep_to_bit(ep_addr));
			regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_ISO_CRCERRIC_Msk;
		}

		byte_count = (regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_BYCT_Msk) >>
			     USBHS_DEVEPTISR_BYCT_Pos;

		LOG_DBG("ISR OUT ep 0x%02x byte_count %u", ep_addr, byte_count);

		size = MIN(byte_count, net_buf_tailroom(buf));
		if (byte_count > size) {
			LOG_ERR("OUT ep 0x%02x overflow: %u bytes received, only %u fit", ep_addr,
				byte_count, size);
			atomic_set_bit(&dev_data->out_truncated, udc_mchp_usbhs_ep_to_bit(ep_addr));
		}
		data = net_buf_add(buf, size);

		for (size_t idx = 0u; idx < size; idx++) {
			data[idx] = fifo[idx];
		}

		/* Release the bank:
		 * - Control EP0: Clear RXOUTI only (single bank, no FIFOCON)
		 * - Bulk/Int/Iso: Clear RXOUTI first, then FIFOCON
		 */
		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_RXOUTIC_Msk;
		if (!is_ctrl) {
			regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
		}

		if (net_buf_tailroom(buf) == 0u || size < udc_mps_ep_size(ep_cfg)) {
			/* Transfer complete (buffer full or short/ZLP).
			 * Disable RXOUTE until new buffer queued.
			 */
			regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_RXOUTEC_Msk;
			atomic_set_bit(&dev_data->xfer_finished, udc_mchp_usbhs_ep_to_bit(ep_addr));
			return;
		}

		/* For dual-banked endpoints, check if another packet is ready. NBUSYBK indicates
		 * number of busy banks (packets waiting). Continue processing while data available
		 * and buffer has room.
		 */
	} while (!is_ctrl && ((regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_NBUSYBK_Msk) >>
			      USBHS_DEVEPTISR_NBUSYBK_Pos) != 0u);
}

/*
 * Handle interrupt IN request
 */
static void udc_mchp_usbhs_handle_in_isr(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	uint8_t ep_addr = udc_mchp_usbhs_hw_ep_to_ep_addr(ep_idx, true);
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, ep_addr);
	volatile uint8_t *fifo = udc_mchp_usbhs_ep_fifo(dev_config, ep_idx);
	struct net_buf *buf = NULL;
	uint32_t len = 0u;
#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	uint8_t ch = 0u;
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	buf = udc_buf_peek(ep_cfg);
	if (buf == NULL) {
		LOG_ERR("No buffer for ep 0x%02x", ep_addr);
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_TXINEC_Msk;
		return;
	}

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	/* The DMA path is handled with normal interrupts, filtered by which endpoints are
	 * connfigured to use DMA.
	 */
	if (udc_mchp_usbhs_ep_has_dma(ep_idx)) {
		ch = ep_idx - DMA_FIRST_EP;

		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_TXINIC_Msk;

		if (dev_data->dma_in_pkts_pending[ch] > 0u) {
			dev_data->dma_in_pkts_pending[ch]--;
		}

		if (dev_data->dma_in_pkts_pending[ch] > 0u) {
			/* DMA is still loading/transmitting further banks for this transfer.
			 * Release the current bank so the next one can be sent
			 */
			regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
			return;
		}

		net_buf_pull(buf, buf->len);

		if (udc_ep_buf_has_zlp(buf)) {
			regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
			udc_ep_buf_clear_zlp(buf);
			return;
		}

		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_TXINEC_Msk;
		atomic_set_bit(&dev_data->xfer_finished, udc_mchp_usbhs_ep_to_bit(ep_addr));
		return;
	}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	len = MIN(buf->len, udc_mps_ep_size(ep_cfg));
	LOG_DBG("ISR IN ep 0x%02x sent %u", ep_addr, len);

	net_buf_pull(buf, len);

	if (buf->len != 0u) {
		len = MIN(buf->len, udc_mps_ep_size(ep_cfg));
		for (size_t idx = 0; idx < len; idx++) {
			fifo[idx] = buf->data[idx];
		}
		barrier_dsync_fence_full();

		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_TXINIC_Msk;
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
	} else {
		if (udc_ep_buf_has_zlp(buf)) {
			regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_TXINIC_Msk;
			regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
			udc_ep_buf_clear_zlp(buf);
			return;
		}

		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_TXINEC_Msk;
		atomic_set_bit(&dev_data->xfer_finished, udc_mchp_usbhs_ep_to_bit(ep_addr));
	}
}

/*
 * Handle interrupt endpoint request
 */
static void udc_mchp_usbhs_handle_ep_isr(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	uint32_t sr = regs->USBHS_DEVEPTISR[ep_idx] & regs->USBHS_DEVEPTIMR[ep_idx];

	if (ep_idx == 0u) {
		if ((sr & USBHS_DEVEPTISR_CTRL_RXSTPI_Msk) != 0u) {
			udc_mchp_usbhs_handle_setup_isr(dev);
			return;
		}
	}

	if ((sr & USBHS_DEVEPTISR_RXOUTI_Msk) != 0u) {
		udc_mchp_usbhs_handle_out_isr(dev, ep_idx);
	}

	if ((sr & USBHS_DEVEPTISR_TXINI_Msk) != 0u) {
		udc_mchp_usbhs_handle_in_isr(dev, ep_idx);
	}
}

/*
 * Interrupt handler
 */
static void udc_mchp_usbhs_isr_handler(const struct device *dev)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	usbhs_registers_t *const regs = dev_config->regs;
	uint32_t sr = regs->USBHS_DEVISR & regs->USBHS_DEVIMR;
	uint32_t pending_events = 0u;

	if ((sr & USBHS_DEVISR_WAKEUP_Msk) != 0u) {
		regs->USBHS_DEVICR = USBHS_DEVICR_WAKEUPC_Msk;
		/* Disable WAKEUP, enable SUSP to detect next suspend */
		regs->USBHS_DEVIDR = USBHS_DEVIDR_WAKEUPEC_Msk;
		regs->USBHS_DEVIER = USBHS_DEVIER_SUSPES_Msk;
		LOG_DBG("USB Wakeup ISR");
		if (udc_is_suspended(dev)) {
			LOG_DBG("USB Wakeup");
			udc_set_suspended(dev, false);
			udc_submit_event(dev, UDC_EVT_RESUME, 0);
		}
	}

	if ((sr & USBHS_DEVISR_EORSM_Msk) != 0u) {
		LOG_DBG("USB End Of Resume ISR");
		regs->USBHS_DEVICR = USBHS_DEVICR_EORSMC_Msk;
		if (udc_is_suspended(dev)) {
			LOG_DBG("USB Resume");
			udc_set_suspended(dev, false);
			udc_submit_event(dev, UDC_EVT_RESUME, 0);
		}
	}

	if ((sr & USBHS_DEVISR_EORST_Msk) != 0u) {
		regs->USBHS_DEVICR = USBHS_DEVICR_EORSTC_Msk;
		LOG_DBG("USB Reset");

		/* Per datasheet §38.6.2.4: EPTYPE/EPSIZE/EPBK/EPDIR/ALLOC survive a bus reset and
		 * don't need reconfiguring. DEVEPTIMR (interrupt enables) and stale DEVEPTISR
		 * status bits do not get cleared automatically. An explicit EPRSTx pulse is the
		 * datasheet-recommended cleanup before reusing EP0.
		 */
		udc_mchp_usbhs_ep_reset(dev, 0u);

		regs->USBHS_DEVEPTIER[0] =
			USBHS_DEVEPTIER_CTRL_RXSTPES_Msk | USBHS_DEVEPTIER_RXOUTES_Msk;

		udc_submit_event(dev, UDC_EVT_RESET, 0);
	}

	if ((sr & USBHS_DEVISR_SUSP_Msk) != 0u) {
		LOG_DBG("USB Suspend ISR");
		regs->USBHS_DEVICR = USBHS_DEVICR_SUSPC_Msk;
		/* Disable SUSP, enable WAKEUP to detect resume */
		regs->USBHS_DEVIDR = USBHS_DEVIDR_SUSPEC_Msk;
		regs->USBHS_DEVIER = USBHS_DEVIER_WAKEUPES_Msk;
		if (!udc_is_suspended(dev)) {
			LOG_DBG("USB Suspend");
			udc_set_suspended(dev, true);
			udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
		}
	}

	if ((sr & USBHS_DEVISR_SOF_Msk) != 0u) {
		regs->USBHS_DEVICR = USBHS_DEVICR_SOFC_Msk;
		udc_submit_sof_event(dev);
	}

	for (uint8_t ep_idx = 0u; ep_idx < dev_config->ep_num; ep_idx++) {
		if ((sr & BIT(USBHS_DEVISR_PEP_0_Pos + ep_idx)) != 0u) {
			udc_mchp_usbhs_handle_ep_isr(dev, ep_idx);
		}
	}

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	/* DMA completion interrupt for OUT endpoints */
	for (uint8_t ch = 0u; ch < USBHS_DEVDMA_NUMBER; ch++) {
		if ((sr & BIT(USBHS_DEVISR_DMA_1_Pos + ch)) != 0u) {
			udc_mchp_usbhs_out_dma_done(dev, ch + DMA_FIRST_EP);
		}
	}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	/*
	 * Batched thread wake-up: check all pending flags and wake thread
	 * once if any events need processing. This reduces context switches.
	 */
	if (atomic_get(&dev_data->setup_pending) != 0u) {
		pending_events |= BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_SETUP);
	}
	if (atomic_get(&dev_data->xfer_finished) != 0u) {
		pending_events |= BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_FINISHED);
	}
	if (atomic_get(&dev_data->out_pending) != 0u) {
		pending_events |= BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_OUT_PENDING);
	}
	if (atomic_get(&dev_data->xfer_new) != 0u) {
		pending_events |= BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_NEW);
	}

	if (pending_events != 0u) {
		k_event_post(&dev_data->events, pending_events);
	}
}

static int udc_mchp_usbhs_ep_enqueue(const struct device *dev, struct udc_ep_config *ep_cfg,
				     struct net_buf *buf)
{
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);

	LOG_DBG("%s enqueue 0x%02x %p", dev->name, ep_cfg->addr, (void *)buf);
	udc_buf_put(ep_cfg, buf);

	if (ep_cfg->stat.halted != 0u) {
		LOG_WRN("ep 0x%02x is halted, not starting transfer", ep_cfg->addr);
	} else {
		atomic_set_bit(&dev_data->xfer_new, udc_mchp_usbhs_ep_to_bit(ep_cfg->addr));
		k_event_post(&dev_data->events, BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_NEW));
	}

	return 0;
}

static int udc_mchp_usbhs_ep_dequeue(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	usbhs_registers_t *const regs = dev_config->regs;
	uint8_t ep_idx = udc_mchp_usbhs_ep_addr_to_hw_ep(ep_cfg->addr);
	int bit = udc_mchp_usbhs_ep_to_bit(ep_cfg->addr);
#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	uint8_t ch = 0u;
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */
	unsigned int lock_key = irq_lock();

	if (USB_EP_DIR_IS_IN(ep_cfg->addr)) {
		/* Disable the USBHS_DEVEPTISRx.TXINI interrupt. */
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_TXINEC_Msk;

		if ((regs->USBHS_DEVEPTISR[ep_idx] & USBHS_DEVEPTISR_NBUSYBK_Msk) == 0u) {
			/* If the transmission was already in progress reset the endpoint */
			udc_mchp_usbhs_ep_reset(dev, ep_idx);
		} else {
			/* A bank is busy: kill the last written bank instead of resetting the
			 * endpoint.
			 */
			regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_KILLBKS_Msk;

			/* Wait for the end of the procedure: hardware clears KILLBK when the kill
			 * completes.
			 */
			if (!WAIT_FOR((regs->USBHS_DEVEPTIMR[ep_idx] &
				       USBHS_DEVEPTIMR_KILLBK_Msk) == 0u,
				      BANK_KILL_TIMEOUT_US, k_busy_wait(1u))) {
				LOG_ERR("EP 0x%02x: KILLBK did not complete within timeout",
					ep_idx);
				irq_unlock(lock_key);
				return -ETIMEDOUT;
			}
		}
	} else {
		regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_RXOUTEC_Msk;
		/* Release any bank the endpoint might currently hold for the cancelled transfer.
		 * Otherwise a later prep_out() re-arm (DMA or CPU) would treat this leftover data
		 * as belonging to whatever buffer comes next.
		 */
		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_RXOUTIC_Msk;
		if (ep_idx != 0u) {
			regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
		}
		/* Clear out_pending, out_truncated and out_crc_error to prevent thread from
		 * processing stale deferred data for this cancelled transfer.
		 */
		atomic_clear_bit(&dev_data->out_pending, bit);
		atomic_clear_bit(&dev_data->out_truncated, bit);
		atomic_clear_bit(&dev_data->out_crc_error, bit);
	}

#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	if (udc_mchp_usbhs_ep_has_dma(ep_idx)) {
		ch = ep_idx - DMA_FIRST_EP;

		regs->USBHS_DEVDMA[ch].USBHS_DEVDMACONTROL = 0u;
		dev_data->dma_out_len[ch] = 0u;
		dev_data->dma_out_pkt[ch] = 0u;
	}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	/* Clear xfer_finished to prevent thread from processing
	 * stale completion for this cancelled transfer.
	 */
	atomic_clear_bit(&dev_data->xfer_finished, bit);

	udc_ep_cancel_queued(dev, ep_cfg);
	udc_ep_set_busy(ep_cfg, false);

	irq_unlock(lock_key);

	return 0;
}

/*
 * USB 2.0 Specification Maximum Packet Sizes (Section 5.5-5.8)
 *
 * +---------------+------------------------+------------------------+
 * | Transfer Type | High-Speed (480 Mbps)  | Full-Speed (12 Mbps)   |
 * +---------------+------------------------+------------------------+
 * | Control       | 64 bytes (fixed)       | 8, 16, 32, or 64 bytes |
 * | Bulk          | 512 bytes (fixed)      | 8, 16, 32, or 64 bytes |
 * | Interrupt     | 1-1024 bytes           | 1-64 bytes             |
 * | Isochronous   | 1-1024 bytes           | 1-1023 bytes           |
 * +---------------+------------------------+------------------------+
 *
 * Notes:
 * - Control EP0 max is always 64 bytes for HS, typically 64 for FS
 * - Bulk endpoints have fixed max sizes per speed mode
 * - Interrupt/Isochronous allow variable sizes up to the maximum
 * - Hardware (USBHS) supports up to 1024 bytes for EP1-EP9
 */
static uint16_t udc_mchp_usbhs_get_max_mps(uint8_t ep_type, bool is_hs)
{
	switch (ep_type) {
	case USB_EP_TYPE_CONTROL:
		return 64u;
	case USB_EP_TYPE_BULK:
		return is_hs ? 512u : 64u;
	case USB_EP_TYPE_INTERRUPT:
		return is_hs ? 1024u : 64u;
	case USB_EP_TYPE_ISO:
		return is_hs ? 1024u : 1023u;
	default:
		return 0u;
	}
}

static int udc_mchp_usbhs_ep_enable(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_data *udc_data = dev->data;
#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */
	uint8_t ep_idx = udc_mchp_usbhs_ep_addr_to_hw_ep(ep_cfg->addr);
	uint8_t ep_type = ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK;
	uint16_t max_mps = 0u;
	uint32_t regval = 0u;
	uint8_t log2ceil_mps = 0u;
	uint8_t eptype = 0u;
	unsigned int lock_key = 0u;
	bool is_in_block = false;

	LOG_INF("Enable ep 0x%02x (hw_idx=%d, type=%d, mps=%d)", ep_cfg->addr, ep_idx, ep_type,
		ep_cfg->mps);

	if (ep_idx >= dev_config->ep_num) {
		LOG_ERR("Not enough configured endpoints");
		return -EINVAL;
	}

	/* This peripheral has one configuration register per hardware endpoint,
	 * and that index is the USB endpoint number on the wire - each number
	 * is registered (in driver_init()) as either IN-only or OUT-only, never
	 * both (except for EP0).
	 */
	if (ep_idx != 0) {
		is_in_block = ep_idx < dev_config->num_of_in_eps;

		if (USB_EP_DIR_IS_IN(ep_cfg->addr) != is_in_block) {
			LOG_ERR("EP 0x%02x direction does not match its hardware endpoint's fixed "
				"IN/OUT block",
				ep_cfg->addr);
			return -EINVAL;
		}
	}

	/* Validate MPS against USB 2.0 specification limits.
	 * The maximum packet size depends on transfer type and bus speed.
	 */
	max_mps = udc_mchp_usbhs_get_max_mps(ep_type, udc_data->caps.hs);
	if (ep_cfg->mps > max_mps) {
		LOG_ERR("EP 0x%02x mps %d exceeds USB spec max %d for type %d (%s)", ep_cfg->addr,
			ep_cfg->mps, max_mps, ep_type, udc_data->caps.hs ? "HS" : "FS");
		return -EINVAL;
	}

	switch (ep_type) {
	case USB_EP_TYPE_CONTROL:
		eptype = USBHS_DEVEPTCFG_EPTYPE_CTRL_Val;
		break;
	case USB_EP_TYPE_ISO:
		eptype = USBHS_DEVEPTCFG_EPTYPE_ISO_Val;
		break;
	case USB_EP_TYPE_BULK:
		eptype = USBHS_DEVEPTCFG_EPTYPE_BLK_Val;
		break;
	case USB_EP_TYPE_INTERRUPT:
		eptype = USBHS_DEVEPTCFG_EPTYPE_INTRPT_Val;
		break;
	default:
		return -EINVAL;
	}

	udc_mchp_usbhs_ep_reset(dev, ep_idx);

	regval = USBHS_DEVEPTCFG_EPTYPE(eptype);

	/* Map the MPS to hardware buffer size. Only power-of-2 sizes from
	 * 8 to 1024 bytes are supported. Calculate log2 ceiling and subtract 3
	 * to get the EPSIZE field value (0=8B, 1=16B, ... 7=1024B).
	 * Use 64 bytes minimum for Full-Speed USB compliance.
	 */
	log2ceil_mps = 32u - __builtin_clz((MAX(ep_cfg->mps, 64u) << 1u) - 1u) - 1u;
	regval |= USBHS_DEVEPTCFG_EPSIZE(log2ceil_mps - 3u);

	if (USB_EP_DIR_IS_IN(ep_cfg->addr) &&
	    (ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) != USB_EP_TYPE_CONTROL) {
		regval |= USBHS_DEVEPTCFG_EPDIR_IN;
	}

	/* Configure endpoint banks:
	 * - Control endpoints: single-bank (per datasheet recommendation)
	 * - Bulk/Interrupt/ISO endpoints: dual-bank (ping-pong mode)
	 *
	 * AUTOSW (automatic bank switching) is enabled for non-control endpoints
	 * and requires multi-bank configuration to function correctly.
	 */
	if ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_CONTROL) {
		regval |= USBHS_DEVEPTCFG_EPBK_1_BANK;
	} else {
		regval |= USBHS_DEVEPTCFG_EPBK_2_BANK;
	}

	if ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) != USB_EP_TYPE_CONTROL) {
		regval |= USBHS_DEVEPTCFG_AUTOSW_Msk;
	}

	/* ALLOC must be written in the same operation as the other configuration
	 * bits. If written separately (read-modify-write), the hardware may reset
	 * configuration bits like AUTOSW when ALLOC is not set.
	 */
	regval |= USBHS_DEVEPTCFG_ALLOC_Msk;

	LOG_DBG("EP%d config: writing 0x%08x", ep_idx, regval);

	regs->USBHS_DEVEPTCFG[ep_idx] = regval;

	if (!udc_mchp_usbhs_ep_is_configured(dev, ep_idx)) {
		LOG_ERR("Endpoint %d configuration failed (CFG=0x%08x)", ep_idx,
			regs->USBHS_DEVEPTCFG[ep_idx]);
		return -EINVAL;
	}

	LOG_DBG("EP%d after enable: CFG=0x%08x ISR=0x%08x", ep_idx, regs->USBHS_DEVEPTCFG[ep_idx],
		regs->USBHS_DEVEPTISR[ep_idx]);

	lock_key = irq_lock();

	regs->USBHS_DEVEPT |= BIT(USBHS_DEVEPT_EPEN0_Pos + ep_idx);
	regs->USBHS_DEVIER = BIT(USBHS_DEVIER_PEP_0_Pos + ep_idx);

	irq_unlock(lock_key);

	if (ep_idx == 0u) {
		regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_CTRL_RXSTPES_Msk;
		regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;
#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	} else if (udc_mchp_usbhs_ep_has_dma(ep_idx) && USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		/* RXOUTI starts the DMA read of each bank, the DMA channel end of buffer interrupt
		 * releases it.
		 */
		dev_data->dma_out_len[ep_idx - DMA_FIRST_EP] = 0u;
		dev_data->dma_out_pkt[ep_idx - DMA_FIRST_EP] = 0u;
		regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;
		regs->USBHS_DEVIER = BIT(USBHS_DEVIER_DMA_1_Pos + (ep_idx - DMA_FIRST_EP));
		if (ep_type == USB_EP_TYPE_ISO) {
			/* Enable the CRCERRI interrupt for ISO transactions */
			regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_ISO_CRCERRES_Msk;
		}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */
	} else if (USB_EP_DIR_IS_IN(ep_cfg->addr)) {
		/* IN completion (DMA or CPU) is always driven by TXINI: DMA only changes how bytes
		 * get into the bank, not when the bank is actually sent to the host.
		 */
		regs->USBHS_DEVEPTICR[ep_idx] = USBHS_DEVEPTICR_TXINIC_Msk;
		regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_TXINES_Msk;
	} else {
		/* OUT endpoint: Enable RXOUTE to receive OUT packets. Per point 8: RXOUTE must be
		 * enabled at EP enable time. Banks are empty after configuration, FIFOCON=0 (no
		 * data).
		 */
		regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RXOUTES_Msk;
		if (ep_type == USB_EP_TYPE_ISO) {
			/* Enable the CRCERRI interrupt for ISO transactions */
			regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_ISO_CRCERRES_Msk;
		}
	}

	return 0;
}

static int udc_mchp_usbhs_ep_disable(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	uint8_t ep_idx = udc_mchp_usbhs_ep_addr_to_hw_ep(ep_cfg->addr);
	unsigned int lock_key = 0u;
#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	uint8_t ch = 0u;
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */

	if (ep_idx >= dev_config->ep_num) {
		LOG_ERR("Not enough configured endpoints");
		return -EINVAL;
	}

	LOG_DBG("Disable ep 0x%02x", ep_cfg->addr);

	lock_key = irq_lock();

	regs->USBHS_DEVIDR = BIT(USBHS_DEVIDR_PEP_0_Pos + ep_idx);
#if defined(CONFIG_UDC_MCHP_USBHS_G2_DMA)
	if (udc_mchp_usbhs_ep_has_dma(ep_idx)) {
		ch = ep_idx - DMA_FIRST_EP;

		regs->USBHS_DEVIDR = BIT(USBHS_DEVIDR_DMA_1_Pos + ch);
		regs->USBHS_DEVDMA[ch].USBHS_DEVDMACONTROL = 0u;
		dev_data->dma_out_len[ch] = 0u;
		dev_data->dma_out_pkt[ch] = 0u;
	}
#endif /* CONFIG_UDC_MCHP_USBHS_G2_DMA */
	regs->USBHS_DEVEPT &= ~BIT(USBHS_DEVEPT_EPEN0_Pos + ep_idx);
	regs->USBHS_DEVEPTCFG[ep_idx] &= ~USBHS_DEVEPTCFG_ALLOC_Msk;

	irq_unlock(lock_key);

	return 0;
}

static int udc_mchp_usbhs_ep_set_halt(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	uint8_t ep_idx = udc_mchp_usbhs_ep_addr_to_hw_ep(ep_cfg->addr);

	if (ep_idx >= dev_config->ep_num) {
		LOG_ERR("Not enough configured endpoints");
		return -EINVAL;
	}

	LOG_INF("Set halt ep 0x%02x", ep_cfg->addr);

	regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_CTRL_STALLRQS_Msk;

	if (USB_EP_GET_IDX(ep_cfg->addr) != 0u) {
		ep_cfg->stat.halted = true;
	}

	return 0;
}

static int udc_mchp_usbhs_ep_clear_halt(const struct device *dev, struct udc_ep_config *ep_cfg)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	uint8_t ep_idx = udc_mchp_usbhs_ep_addr_to_hw_ep(ep_cfg->addr);

	if (ep_idx >= dev_config->ep_num) {
		LOG_ERR("Not enough configured endpoints");
		return -EINVAL;
	}

	LOG_INF("Clear halt ep 0x%02x", ep_cfg->addr);

	/* Control endpoint do not need a specific halt recovery procedure */
	if (ep_idx == 0u) {
		return 0;
	}

	/* Clear the STALL request. */
	regs->USBHS_DEVEPTIDR[ep_idx] = USBHS_DEVEPTIDR_CTRL_STALLRQC_Msk;
	/* Reset the data toggle counter. */
	regs->USBHS_DEVEPTIER[ep_idx] = USBHS_DEVEPTIER_RSTDTS_Msk;

	ep_cfg->stat.halted = false;

	/* Resume queued transfers if any */
	if (!udc_ep_is_busy(ep_cfg) && udc_buf_peek(ep_cfg) != NULL) {
		atomic_set_bit(&dev_data->xfer_new, udc_mchp_usbhs_ep_to_bit(ep_cfg->addr));
		k_event_post(&dev_data->events, BIT(UDC_MCHP_USBHS_G2_EVENT_TYPE_XFER_NEW));
	}

	return 0;
}

static int udc_mchp_usbhs_set_address(const struct device *dev, uint8_t addr)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;

	LOG_INF("Set address %u for %s", addr, dev->name);

	regs->USBHS_DEVCTRL &= ~(USBHS_DEVCTRL_UADD_Msk | USBHS_DEVCTRL_ADDEN_Msk);
	if (addr != 0u) {
		regs->USBHS_DEVCTRL |= USBHS_DEVCTRL_UADD(addr) | USBHS_DEVCTRL_ADDEN_Msk;
	}

	return 0;
}

static int udc_mchp_usbhs_host_wakeup(const struct device *dev)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;

	LOG_DBG("Remote wakeup from %s", dev->name);

	regs->USBHS_DEVCTRL |= USBHS_DEVCTRL_RMWKUP_Msk;

	return 0;
}

static void setup_test_mode(usbhs_registers_t *const regs)
{
	/* Set SPDCONF to forced High-Speed for test modes */
	regs->USBHS_DEVCTRL = (regs->USBHS_DEVCTRL & ~USBHS_DEVCTRL_SPDCONF_Msk) |
			      USBHS_DEVCTRL_SPDCONF_HIGH_SPEED;
}

static int udc_mchp_usbhs_test_mode(const struct device *dev, uint8_t mode, bool dryrun)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	volatile uint8_t *fifo = NULL;
	unsigned int lock_key = 0u;
	/* USB 2.0 spec defined test packet for Test_Packet mode */
	static const uint8_t test_packet[] = {
		0x00,
		/* JKJKJKJK * 9 */
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		/* JJKKJJKK * 8 */
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		/* JJJJKKKK * 8 */
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		0xEE,
		/* JJJJJJJKKKKKKK * 8 */
		0xFE,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		/* JJJJJJJK * 8 */
		0x7F,
		0xBF,
		0xDF,
		0xEF,
		0xF7,
		0xFB,
		0xFD,
		/* JKKKKKKK * 8, {JKKKKKKK * 10}(not used) */
		0xFC,
		0x7E,
		0xBF,
		0xDF,
		0xEF,
		0xF7,
		0xFB,
		0xFD,
		0x7E,
	};

	if (mode == 0u || mode > USB_SFS_TEST_MODE_FORCE_ENABLE) {
		return -EINVAL;
	}

	if (dryrun) {
		if (mode == USB_SFS_TEST_MODE_FORCE_ENABLE) {
			return -ENOTSUP;
		}
		LOG_DBG("Test Mode %u supported", mode);
		return 0;
	}

	LOG_DBG("Enable Test Mode %u", mode);

	switch (mode) {
	case USB_SFS_TEST_MODE_J:
		setup_test_mode(regs);

		regs->USBHS_DEVCTRL |= USBHS_DEVCTRL_TSTJ_Msk;
		break;
	case USB_SFS_TEST_MODE_K:
		setup_test_mode(regs);

		regs->USBHS_DEVCTRL |= USBHS_DEVCTRL_TSTK_Msk;
		break;
	case USB_SFS_TEST_MODE_SE0_NAK:
		setup_test_mode(regs);

		/* SE0_NAK: just enable test mode, no additional bits needed */
		break;
	case USB_SFS_TEST_MODE_PACKET:
		setup_test_mode(regs);

		/* Test_Packet mode sequence per datasheet:
		 * 1. TSTPCKT = 1
		 * 2. Store data in endpoint bank
		 * 3. Clear FIFOCON to start transmission
		 */
		fifo = udc_mchp_usbhs_ep_fifo(dev_config, 0);
		lock_key = irq_lock();

		/* Disable and reset EP0 */
		regs->USBHS_DEVEPT &= ~USBHS_DEVEPT_EPEN0_Msk;
		regs->USBHS_DEVEPT |= USBHS_DEVEPT_EPRST0_Msk;
		regs->USBHS_DEVEPT &= ~USBHS_DEVEPT_EPRST0_Msk;

		/* Reconfigure EP0 as bulk IN, 64 bytes, single bank */
		regs->USBHS_DEVEPTCFG[0] = USBHS_DEVEPTCFG_EPTYPE_BLK |
					   USBHS_DEVEPTCFG_EPSIZE_64_BYTE |
					   USBHS_DEVEPTCFG_EPDIR_IN | USBHS_DEVEPTCFG_EPBK_1_BANK |
					   USBHS_DEVEPTCFG_ALLOC_Msk;

		/* Enable EP0 */
		regs->USBHS_DEVEPT |= USBHS_DEVEPT_EPEN0_Msk;

		irq_unlock(lock_key);

		/* Step 1: Enable test packet mode */
		regs->USBHS_DEVCTRL |= USBHS_DEVCTRL_TSTPCKT_Msk;

		/* Step 2: Copy test packet to FIFO */
		for (size_t idx = 0u; idx < sizeof(test_packet); idx++) {
			fifo[idx] = test_packet[idx];
		}

		/* Step 3: Clear FIFOCON to start repeated transmission */
		regs->USBHS_DEVEPTIDR[0] = USBHS_DEVEPTIDR_FIFOCONC_Msk;
		break;
	case USB_SFS_TEST_MODE_FORCE_ENABLE:
		/* Force Enable is typically not supported in device mode */
		return -ENOTSUP;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum udc_bus_speed udc_mchp_usbhs_device_speed(const struct device *dev)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	uint32_t speed = 0u;

	speed = (regs->USBHS_SR & USBHS_SR_SPEED_Msk) >> USBHS_SR_SPEED_Pos;

	if (speed == USBHS_SR_SPEED_HIGH_SPEED_Val) {
		return UDC_BUS_SPEED_HS;
	}

	return UDC_BUS_SPEED_FS;
}

static int udc_mchp_usbhs_enable(const struct device *dev)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	int ret = 0;
	int enable_ret = 0;

	LOG_DBG("Enable device %s", dev->name);

	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Failed to enable USBHS clock");
		return ret;
	}

	/* Hardware reset of the USBHS block */
	regs->USBHS_CTRL &= ~USBHS_CTRL_USBE_Msk;
	barrier_dsync_fence_full();

	regs->USBHS_CTRL = USBHS_CTRL_UIMOD_Msk | USBHS_CTRL_USBE_Msk;
	barrier_dsync_fence_full();

	if (dev_config->low_power) {
		regs->USBHS_DEVCTRL = USBHS_DEVCTRL_DETACH_Msk | USBHS_DEVCTRL_SPDCONF_LOW_POWER;
	} else {
		regs->USBHS_DEVCTRL = USBHS_DEVCTRL_DETACH_Msk | USBHS_DEVCTRL_SPDCONF_NORMAL;
	}

	ret = udc_mchp_usbhs_enable_clock(dev_config);
	if (ret != 0) {
		enable_ret = ret;

		LOG_ERR("Failed to turn on peripheral clock: %i", ret);

		ret = udc_mchp_usbhs_disable(dev);
		if (ret != 0) {
			LOG_ERR("Failed to fully clean up after enable failure: %i", ret);
		}

		return enable_ret;
	}

	if (!WAIT_FOR(((regs->USBHS_SR & USBHS_SR_CLKUSABLE_Msk) != 0u), CLOCK_USABLE_TIMEOUT_US,
		      k_busy_wait(1u))) {
		LOG_ERR("Timed out waiting for peripheral clock");

		ret = udc_mchp_usbhs_disable(dev);
		if (ret != 0) {
			LOG_ERR("Failed to fully clean up after enable failure: %i", ret);
		}

		return -ETIMEDOUT;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 64u, 0u);
	if (ret != 0) {
		enable_ret = ret;

		LOG_ERR("Failed to enable control OUT endpoint");

		ret = udc_mchp_usbhs_disable(dev);
		if (ret != 0) {
			LOG_ERR("Failed to fully clean up after enable failure: %i", ret);
		}
		return enable_ret;
	}

	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 64u, 0u);
	if (ret != 0) {
		enable_ret = ret;

		LOG_ERR("Failed to enable control IN endpoint");

		ret = udc_mchp_usbhs_disable(dev);
		if (ret != 0) {
			LOG_ERR("Failed to fully clean up after enable failure: %i", ret);
		}
		return enable_ret;
	}

	regs->USBHS_DEVIER =
		USBHS_DEVIER_EORSMES_Msk | USBHS_DEVIER_EORSTES_Msk | USBHS_DEVIER_SUSPES_Msk;

#if defined(CONFIG_UDC_ENABLE_SOF)
	regs->USBHS_DEVIER = USBHS_DEVIER_SOFES_Msk;
#endif /* CONFIG_UDC_ENABLE_SOF */

	dev_config->irq_config_enable(dev);

	/* Reattach pads */
	regs->USBHS_DEVCTRL &= ~USBHS_DEVCTRL_DETACH_Msk;

	return 0;
}

static int udc_mchp_usbhs_disable(const struct device *dev)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	int ret = 0;
	int first_err = 0;

	LOG_DBG("Disable device %s", dev->name);

	dev_config->irq_config_disable(dev);

	regs->USBHS_DEVCTRL |= USBHS_DEVCTRL_DETACH_Msk;

	ret = udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("Failed to disable control OUT endpoint: %i", ret);
		first_err = first_err ? first_err : ret;
	}

	ret = udc_ep_disable_internal(dev, USB_CONTROL_EP_IN);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("Failed to disable control IN endpoint: %i", ret);
		first_err = first_err ? first_err : ret;
	}

	ret = udc_mchp_usbhs_disable_clock(dev_config);
	if (ret != 0) {
		LOG_ERR("Failed to disable USB clock: %i", ret);
		first_err = first_err ? first_err : ret;
	}

	regs->USBHS_CTRL = USBHS_CTRL_UIMOD_Msk | USBHS_CTRL_FRZCLK_Msk;

	ret = clock_control_off(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Failed to turn off peripheral clock: %i", ret);
		first_err = first_err ? first_err : ret;
	}

	return first_err;
}

static int udc_mchp_usbhs_init(const struct device *dev)
{
	LOG_INF("Init device %s", dev->name);

	return 0;
}

static int udc_mchp_usbhs_shutdown(const struct device *dev)
{
	LOG_DBG("Shutdown device %s", dev->name);

	return 0;
}

static int udc_mchp_usbhs_driver_init(const struct device *dev)
{
	const struct udc_mchp_usbhs_config *dev_config = dev->config;
	struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);
	struct udc_data *udc_data = dev->data;
	uint16_t mps = 0u;
	int ret = 0;

	/* Per the USB 2.0 Specification (Section 5.6.3, Table 5-11), the maximum packet size for
	 * isochronous endpoints is 1024 bytes in High-Speed and 1023 bytes in Full-Speed.
	 */
	if (dev_config->low_power) {
		mps = 1023u;
	} else {
		mps = 1024u;
	}

	k_mutex_init(&udc_data->mutex);
	k_event_init(&dev_data->events);
	atomic_clear(&dev_data->xfer_new);
	atomic_clear(&dev_data->xfer_finished);
	atomic_clear(&dev_data->out_pending);
	atomic_clear(&dev_data->setup_pending);
	atomic_clear(&dev_data->out_crc_error);

	udc_data->caps.rwup = true;
	udc_data->caps.mps0 = UDC_MPS0_64;
	if (!dev_config->low_power) {
		udc_data->caps.hs = true;
	}

	/* Register EP0 OUT (control) */
	dev_config->ep_cfg_out[0].caps.out = 1u;
	dev_config->ep_cfg_out[0].caps.control = 1u;
	dev_config->ep_cfg_out[0].caps.mps = 64u;
	dev_config->ep_cfg_out[0].addr = USB_EP_DIR_OUT;
	ret = udc_register_ep(dev, &dev_config->ep_cfg_out[0]);
	if (ret != 0) {
		LOG_ERR("Failed to register EP0 OUT");
		return ret;
	}

	/* Register EP0 IN (control) */
	dev_config->ep_cfg_in[0].caps.in = 1u;
	dev_config->ep_cfg_in[0].caps.control = 1u;
	dev_config->ep_cfg_in[0].caps.mps = 64u;
	dev_config->ep_cfg_in[0].addr = USB_EP_DIR_IN;
	ret = udc_register_ep(dev, &dev_config->ep_cfg_in[0]);
	if (ret != 0) {
		LOG_ERR("Failed to register EP0 IN");
		return ret;
	}

	/* This peripheral has a single configuration register per hardware endpoint index, and that
	 * index IS the USB endpoint number on the wire (there is no address remapping) - so a given
	 * endpoint number can only be used in one direction at any given time. The remaining
	 * endpoints are therefore split into a fixed IN block followed by a fixed OUT
	 * block: 1..(num_of_in_eps - 1) are IN-only, and the rest, up to ep_num - 1, are OUT-only.
	 */
	for (size_t ep_idx = 1u; ep_idx < dev_config->num_of_in_eps; ep_idx++) {
		dev_config->ep_cfg_in[ep_idx].caps.in = 1u;
		dev_config->ep_cfg_in[ep_idx].caps.bulk = 1u;
		dev_config->ep_cfg_in[ep_idx].caps.interrupt = 1u;
		dev_config->ep_cfg_in[ep_idx].caps.iso = 1u;
		dev_config->ep_cfg_in[ep_idx].caps.mps = mps;
		dev_config->ep_cfg_in[ep_idx].addr = USB_EP_DIR_IN | ((uint8_t)ep_idx);
		ret = udc_register_ep(dev, &dev_config->ep_cfg_in[ep_idx]);
		if (ret != 0) {
			LOG_ERR("Failed to register IN endpoint 0x%02x",
				dev_config->ep_cfg_in[ep_idx].addr);
			return ret;
		}
	}

	for (size_t ep_idx = 1u; ep_idx < dev_config->num_of_out_eps; ep_idx++) {
		uint8_t hw_ep = (uint8_t)(dev_config->num_of_in_eps - 1u + ep_idx);

		dev_config->ep_cfg_out[ep_idx].caps.out = 1u;
		dev_config->ep_cfg_out[ep_idx].caps.bulk = 1u;
		dev_config->ep_cfg_out[ep_idx].caps.interrupt = 1u;
		dev_config->ep_cfg_out[ep_idx].caps.iso = 1u;
		dev_config->ep_cfg_out[ep_idx].caps.mps = mps;
		dev_config->ep_cfg_out[ep_idx].addr = USB_EP_DIR_OUT | hw_ep;
		ret = udc_register_ep(dev, &dev_config->ep_cfg_out[ep_idx]);
		if (ret != 0) {
			LOG_ERR("Failed to register OUT endpoint 0x%02x",
				dev_config->ep_cfg_out[ep_idx].addr);
			return ret;
		}
	}

	dev_config->make_thread(dev);

	return 0;
}

static void udc_mchp_usbhs_lock(const struct device *dev)
{
	udc_lock_internal(dev, K_FOREVER);
}

static void udc_mchp_usbhs_unlock(const struct device *dev)
{
	udc_unlock_internal(dev);
}

static const struct udc_api udc_mchp_usbhs_driver_api = {
	.lock = udc_mchp_usbhs_lock,
	.unlock = udc_mchp_usbhs_unlock,
	.device_speed = udc_mchp_usbhs_device_speed,
	.init = udc_mchp_usbhs_init,
	.enable = udc_mchp_usbhs_enable,
	.disable = udc_mchp_usbhs_disable,
	.shutdown = udc_mchp_usbhs_shutdown,
	.set_address = udc_mchp_usbhs_set_address,
	.host_wakeup = udc_mchp_usbhs_host_wakeup,
	.test_mode = udc_mchp_usbhs_test_mode,
	.ep_enable = udc_mchp_usbhs_ep_enable,
	.ep_disable = udc_mchp_usbhs_ep_disable,
	.ep_set_halt = udc_mchp_usbhs_ep_set_halt,
	.ep_clear_halt = udc_mchp_usbhs_ep_clear_halt,
	.ep_enqueue = udc_mchp_usbhs_ep_enqueue,
	.ep_dequeue = udc_mchp_usbhs_ep_dequeue,
	.ep_try_config = NULL,
};

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	static void irq_config_enable_##n(const struct device *dev)                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), udc_mchp_usbhs_isr_handler, \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static void irq_config_disable_##n(const struct device *dev)                               \
	{                                                                                          \
		irq_disable(DT_INST_IRQN(n));                                                      \
	}                                                                                          \
                                                                                                   \
	K_THREAD_STACK_DEFINE(udc_mchp_usbhs_stack_##n, CONFIG_UDC_MCHP_USBHS_G2_STACK_SIZE);      \
                                                                                                   \
	static void udc_mchp_usbhs_thread_##n(void *dev, void *arg1, void *arg2)                   \
	{                                                                                          \
		while (true) {                                                                     \
			udc_mchp_usbhs_thread_handler(dev);                                        \
		}                                                                                  \
	}                                                                                          \
                                                                                                   \
	static void udc_mchp_usbhs_make_thread_##n(const struct device *dev)                       \
	{                                                                                          \
		struct udc_mchp_usbhs_data *dev_data = udc_get_private(dev);                       \
                                                                                                   \
		k_thread_create(&dev_data->thread_data, udc_mchp_usbhs_stack_##n,                  \
				K_THREAD_STACK_SIZEOF(udc_mchp_usbhs_stack_##n),                   \
				udc_mchp_usbhs_thread_##n, (void *)dev, NULL, NULL,                \
				K_PRIO_COOP(CONFIG_UDC_MCHP_USBHS_G2_THREAD_PRIORITY),             \
				K_ESSENTIAL, K_NO_WAIT);                                           \
		k_thread_name_set(&dev_data->thread_data, dev->name);                              \
	}                                                                                          \
                                                                                                   \
	BUILD_ASSERT(DT_INST_PROP(n, num_in_endpoints) + DT_INST_PROP(n, num_out_endpoints) - 1 <= \
			     MAX_ENDPOINTS,                                                        \
		     "Too many endpoints configured in devicetree");                               \
                                                                                                   \
	static struct udc_ep_config ep_cfg_in_##n[DT_INST_PROP(n, num_in_endpoints)];              \
	static struct udc_ep_config ep_cfg_out_##n[DT_INST_PROP(n, num_out_endpoints)];            \
                                                                                                   \
	static const struct udc_mchp_usbhs_config mchp_usbhs_##n##_config = {                      \
		.fifo_base = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(n, fifo),                        \
		.regs = (usbhs_registers_t *)DT_INST_REG_ADDR(n),                                  \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys =                                                                    \
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_IDX(n, 0, subsystem),       \
		.low_power = DT_INST_ENUM_IDX_OR(n, maximum_speed, 2) <= 1u,                       \
		.irq_config_enable = irq_config_enable_##n,                                        \
		.irq_config_disable = irq_config_disable_##n,                                      \
		.make_thread = udc_mchp_usbhs_make_thread_##n,                                     \
		.ep_num = DT_INST_PROP(n, num_in_endpoints) + DT_INST_PROP(n, num_out_endpoints) - \
			  1,                                                                       \
		.num_of_in_eps = DT_INST_PROP(n, num_in_endpoints),                                \
		.num_of_out_eps = DT_INST_PROP(n, num_out_endpoints),                              \
		.ep_cfg_in = ep_cfg_in_##n,                                                        \
		.ep_cfg_out = ep_cfg_out_##n,                                                      \
	};                                                                                         \
                                                                                                   \
	static struct udc_mchp_usbhs_data mchp_usbhs_##n##_data = {};                              \
                                                                                                   \
	static struct udc_data mchp_usbhs_##n##_udc_data = {                                       \
		.mutex = Z_MUTEX_INITIALIZER(mchp_usbhs_##n##_udc_data.mutex),                     \
		.priv = &mchp_usbhs_##n##_data,                                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, udc_mchp_usbhs_driver_init, NULL, &mchp_usbhs_##n##_udc_data,     \
			      &mchp_usbhs_##n##_config, POST_KERNEL,                               \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_mchp_usbhs_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
