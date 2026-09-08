/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip USB High-Speed (USBHS) host controller driver.
 *
 * The datasheet reports 10 pipes, but the USBHS_HSTPIP register only has bits to enable the
 * first 9; limit the driver.
 *
 * Once assigned to an endpoint, pipes cannot be repurposed. This is because the hardware data
 * toggle (HSTPIPISR.DTSEQ) is only preserved while the pipe stays enabled (see pipe_done()), so a
 * pipe must keep serving the same (address, endpoint) pair across separate transfers to avoid a
 * host/device toggle desync.
 */

#define DT_DRV_COMPAT microchip_usbhs_g2_uhc

#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>
#include <zephyr/cache.h>

#include "uhc_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_mchp_usbhs_g2, CONFIG_UHC_DRIVER_LOG_LEVEL);

/* The peripheral's DMA channels are hard-wired one-to-one to hardware
 * endpoints 1..USBHS_DEVDMA_NUMBER. EP0 and any endpoint beyond the last
 * channel have no DMA channel and always use the CPU-copy path.
 */
#define DMA_FIRST_EP 1u
#define DMA_LAST_EP  6u

#define NUM_MAX_PIPES            9u
#define CONTROL_PIPE             0u
#define CLOCK_USABLE_TIMEOUT_US  10000u
#define RESET_TIMEOUT_US         100000u
#define LOW_SPEED_ACK_TIMEOUT_US 10000u
#define DMA_TIMEOUT_US           1000u

/* IRQ events */
#define EVENT_CONNECTION    BIT(0u)
#define EVENT_DISCONNECTION BIT(1u)
#define EVENT_WAKE_UP       BIT(2u)
#define EVENT_PIPE          BIT(3u)
#define EVENT_XFER_ENQUEUED BIT(4u)

/* Pipe events */
#define PIPE_FLAG_SETUP_TRANSMITTED BIT(0u)
#define PIPE_FLAG_STALL             BIT(1u)
#define PIPE_FLAG_DATATGL_ERROR     BIT(2u)
#define PIPE_FLAG_DATAPID_ERROR     BIT(3u)
#define PIPE_FLAG_PID_ERROR         BIT(4u)
#define PIPE_FLAG_TIMEOUT_ERROR     BIT(5u)
#define PIPE_FLAG_CRC16_ERROR       BIT(6u)
#define PIPE_FLAG_ERRORS                                                                           \
	(PIPE_FLAG_DATATGL_ERROR | PIPE_FLAG_DATAPID_ERROR | PIPE_FLAG_PID_ERROR |                 \
	 PIPE_FLAG_TIMEOUT_ERROR | PIPE_FLAG_CRC16_ERROR)
#define PIPE_FLAG_IN_PACKET     BIT(7u)
#define PIPE_FLAG_OUT_PACKET    BIT(8u)
#define PIPE_FLAG_ISO_UNDERFLOW BIT(9u)
#define PIPE_FLAG_ISO_OVERFLOW  BIT(10u)
#define PIPE_FLAG_ISO_CRC_ERROR BIT(11u)

/*
 * The state of a transfer in progress
 */
enum uhc_mchp_usbhs_xfer_state {
	UHC_MCHP_USBHS_XFER_STATE_SETUP = 0,
	UHC_MCHP_USBHS_XFER_STATE_DATA,
	UHC_MCHP_USBHS_XFER_STATE_STATUS,
};

/*
 * Structure representing an hardware pipe
 */
struct uhc_mchp_usbhs_pipe {
	/* Events pending for this pipe */
	atomic_t flags;

	/* Transfer state */
	enum uhc_mchp_usbhs_xfer_state xfer_state;

	/* Currently handled transfer */
	struct uhc_transfer *xfer;

	/* Whether the pipe is already bound to a device's endpoint */
	bool bound;

	/* Device address */
	uint8_t bound_addr;

	/* Endpoint address */
	uint8_t bound_ep;

#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
	/* Whether the DMA reception has been armed */
	bool dma_reception_armed;
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */
};

struct uhc_mchp_usbhs_data {
	/* Driver mutex */
	struct k_mutex mutex;

	/* Deferred status register interrupt flags */
	struct k_event events;

	/* Hardware pipe descriptors */
	struct uhc_mchp_usbhs_pipe pipes[NUM_MAX_PIPES];

	/* Peripheral's thread data */
	struct k_thread thread_data;
};

struct uhc_mchp_usbhs_config {
	/* Register block */
	usbhs_registers_t *regs;

	/* Pipe/endpoint fifo RAM (32KB) base address */
	mem_addr_t fifo_base;

	/* Clock driver */
	const struct device *clock;

	/* Clock subsystem for this instance's own channel */
	clock_control_subsys_t clock_subsys;

	/* Whether the High Speed mode is disabled */
	bool low_power;

	/* Function to enable the peripheral's interrupt */
	void (*irq_config_enable)(const struct device *dev);

	/* Function to disable the peripheral's interrupt */
	void (*irq_config_disable)(const struct device *dev);

	/* Function to start the peripheral's thread */
	void (*make_thread)(const struct device *dev);
};

static int uhc_mchp_usbhs_disable(const struct device *dev);

/*
 * Pipe index to HSTPIP reset mask mapping.
 */
static const uint32_t usbhs_hstpip_prst_masks[NUM_MAX_PIPES] = {
	USBHS_HSTPIP_PRST0_Msk, USBHS_HSTPIP_PRST1_Msk, USBHS_HSTPIP_PRST2_Msk,
	USBHS_HSTPIP_PRST3_Msk, USBHS_HSTPIP_PRST4_Msk, USBHS_HSTPIP_PRST5_Msk,
	USBHS_HSTPIP_PRST6_Msk, USBHS_HSTPIP_PRST7_Msk, USBHS_HSTPIP_PRST8_Msk,
};

/*
 * Pipe index to HSTPIP enable mask mapping.
 */
static const uint32_t usbhs_hstpip_pen_masks[NUM_MAX_PIPES] = {
	USBHS_HSTPIP_PEN0_Msk, USBHS_HSTPIP_PEN1_Msk, USBHS_HSTPIP_PEN2_Msk,
	USBHS_HSTPIP_PEN3_Msk, USBHS_HSTPIP_PEN4_Msk, USBHS_HSTPIP_PEN5_Msk,
	USBHS_HSTPIP_PEN6_Msk, USBHS_HSTPIP_PEN7_Msk, USBHS_HSTPIP_PEN8_Msk,
};

/*
 * Signal that a pipe operation is concluded
 */
static inline void uhc_mchp_usbhs_pipe_done(const struct device *dev, size_t pipe_idx, int err)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];

	uhc_xfer_return(dev, pipe->xfer, err);

	/* Freeze, but do not disable, the pipe: leaving it enabled preserves
	 * HSTPIPISR.DTSEQ so a future transfer to the same (address, endpoint)
	 * continues with the correct data toggle.
	 */
	regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_PFREEZES_Msk;
	pipe->xfer = NULL;

	LOG_DBG("Pipe %zu is now idle (still bound to addr %u ep 0x%02x)", pipe_idx,
		pipe->bound_addr, pipe->bound_ep);
}

/*
 * Fetch the pipe index that is handling the provided transfer (if any).
 */
static inline int uhc_mchp_usbhs_get_pipe_for_transfer(struct uhc_mchp_usbhs_data *dev_data,
						       struct uhc_transfer *xfer)
{
	for (size_t pipe_idx = 0u; pipe_idx < NUM_MAX_PIPES; pipe_idx++) {
		if (dev_data->pipes[pipe_idx].xfer == xfer) {
			return pipe_idx;
		}
	}

	return -1;
}

/*
 * Set the pipe type: setup, token or data
 */
static inline void uhc_mchp_usbhs_set_pipe_type(usbhs_registers_t *const regs, size_t pipe_idx,
						uint32_t type)
{
	uint32_t hstpipcfg = regs->USBHS_HSTPIPCFG[pipe_idx];

	hstpipcfg &= ~USBHS_HSTPIPCFG_PTOKEN_Msk;
	hstpipcfg |= type;
	regs->USBHS_HSTPIPCFG[pipe_idx] = hstpipcfg;
}

/*
 * Enable a pipe.
 */
static inline void uhc_mchp_usbhs_enable_pipe(usbhs_registers_t *regs, size_t pipe_idx)
{
	/* Reset pipe */
	regs->USBHS_HSTPIP |= usbhs_hstpip_prst_masks[pipe_idx];
	regs->USBHS_HSTPIP &= ~usbhs_hstpip_prst_masks[pipe_idx];

	/* Enable pipe */
	regs->USBHS_HSTPIP |= usbhs_hstpip_pen_masks[pipe_idx];
}

/*
 * Get the configured max transfer size for a given pipe.
 */
static inline uint16_t uhc_mchp_usbhs_get_pipe_max_transfer_size(usbhs_registers_t *regs,
								 size_t pipe_idx)
{
	const uint16_t max_transfer_sizes[] = {8u, 16u, 32u, 64u, 128u, 256u, 512u, 1024u};
	uint8_t psize = (regs->USBHS_HSTPIPCFG[pipe_idx] & USBHS_HSTPIPCFG_PSIZE_Msk) >>
			USBHS_HSTPIPCFG_PSIZE_Pos;

	return max_transfer_sizes[psize];
}

/*
 * Check if the connected device is high speed.
 */
static inline bool uhc_mchp_usbhs_is_device_high_speed(usbhs_registers_t *regs)
{
	return ((regs->USBHS_SR & USBHS_SR_SPEED_Msk) >> USBHS_SR_SPEED_Pos) ==
	       USBHS_SR_SPEED_HIGH_SPEED_Val;
}

/*
 * Check if the connected device is low speed.
 */
static inline bool uhc_mchp_usbhs_is_device_low_speed(usbhs_registers_t *regs)
{
	return ((regs->USBHS_SR & USBHS_SR_SPEED_Msk) >> USBHS_SR_SPEED_Pos) ==
	       USBHS_SR_SPEED_LOW_SPEED_Val;
}

/*
 * Get the fifo data corresponding to the endpoint index.
 */
static inline volatile uint8_t *
uhc_mchp_usbhs_pipe_fifo(const struct uhc_mchp_usbhs_config *dev_config, uint8_t pipe_idx)
{
	const mem_addr_t fifo_pipe_offset = 0x8000u;

	return (volatile uint8_t *)(dev_config->fifo_base + (pipe_idx * fifo_pipe_offset));
}

#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
/*
 * Check if the pipe in question has DMA capabilities
 */
static inline bool uhc_mchp_usbhs_pipe_has_dma(uint8_t pipe_idx)
{
	/* DMA can only be used on non-control endpoints and up until a certain number */
	return pipe_idx >= DMA_FIRST_EP && pipe_idx <= DMA_LAST_EP;
}

/*
 * Arm DMA for reception of data
 */
static void uhc_mchp_usbhs_arm_dma_reception(usbhs_registers_t *const regs,
					     struct uhc_mchp_usbhs_pipe *pipe, size_t pipe_idx,
					     struct net_buf *buf)
{
	uint32_t tailroom = 0;
	uint8_t *dst = NULL;

	if (uhc_mchp_usbhs_pipe_has_dma(pipe_idx)) {
		usbhs_hstdma_registers_t *dma = &regs->USBHS_HSTDMA[pipe_idx - DMA_FIRST_EP];

		if ((regs->USBHS_HSTPIPIMR[pipe_idx] & USBHS_HSTPIPIMR_FIFOCON_Msk) != 0u) {
			/* A packet is already sitting in the bank from before we could arm; it will
			 * be drained normally by the CPU once RXINI arrives.
			 */
			pipe->dma_reception_armed = false;
		} else {
			tailroom = net_buf_tailroom(buf);
			dst = net_buf_tail(buf);

			/* Invalidate cache before starting DMA read */
			sys_cache_data_invd_range(dst, tailroom);

			dma->USBHS_HSTDMAADDRESS =
				USBHS_HSTDMAADDRESS_BUFF_ADD((uint32_t)(uintptr_t)dst);
			dma->USBHS_HSTDMACONTROL = USBHS_HSTDMACONTROL_BUFF_LENGTH(tailroom) |
						   USBHS_HSTDMACONTROL_END_B_EN_Msk |
						   USBHS_HSTDMACONTROL_END_TR_EN_Msk |
						   USBHS_HSTDMACONTROL_END_TR_IT_Msk |
						   USBHS_HSTDMACONTROL_CHANN_ENB_Msk;

			pipe->dma_reception_armed = true;
		}
	} else {
		pipe->dma_reception_armed = false;
	}
}
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */

/*
 * Handle an error in ISO reception; the pipe should be made ready for future data, but nothing is
 * added to the buffer.
 */
static int uhc_mchp_usbhs_handle_iso_in_gap(const struct device *dev, size_t pipe_idx)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */

	/* No valid data arrived this frame, do not add anything to the net_buf */

	regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;

#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
	uhc_mchp_usbhs_arm_dma_reception(regs, pipe, pipe_idx, pipe->xfer->buf);
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */

	uhc_mchp_usbhs_set_pipe_type(regs, pipe_idx, USBHS_HSTPIPCFG_PTOKEN_IN);
	regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_RXINIC_Msk;
	regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_RXINES_Msk;
	regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;

	return 0;
}

/*
 * Cancel a transfer
 */
static inline void uhc_mchp_usbhs_cancel_xfer(const struct device *dev, size_t pipe_idx, int err)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];

#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
	if (uhc_mchp_usbhs_pipe_has_dma(pipe_idx) && pipe->dma_reception_armed) {
		regs->USBHS_HSTDMA[pipe_idx - DMA_FIRST_EP].USBHS_HSTDMACONTROL = 0;
		pipe->dma_reception_armed = false;
	}
#endif

	uhc_xfer_return(dev, pipe->xfer, err);

	/* Freeze Pipe */
	regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_PFREEZES_Msk;
	pipe->xfer = NULL;
}

/*
 * Disable a pipe.
 */
static inline void uhc_mchp_usbhs_disable_pipe(usbhs_registers_t *regs, size_t pipe_idx)
{
	/* Reset pipe */
	regs->USBHS_HSTPIP |= usbhs_hstpip_prst_masks[pipe_idx];
	regs->USBHS_HSTPIP &= ~usbhs_hstpip_prst_masks[pipe_idx];

	/* Disable pipe */
	regs->USBHS_HSTPIP &= ~usbhs_hstpip_pen_masks[pipe_idx];

#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
	if (uhc_mchp_usbhs_pipe_has_dma(pipe_idx)) {
		regs->USBHS_HSTDMA[pipe_idx - DMA_FIRST_EP].USBHS_HSTDMACONTROL = 0;
	}
#endif
}

/*
 * Send some data over the pipe.
 */
static inline void uhc_mchp_usbhs_transmit(const struct uhc_mchp_usbhs_config *dev_config,
					   uint8_t pipe_idx, uint8_t *data, size_t length)
{
	volatile uint8_t *pipe_data = uhc_mchp_usbhs_pipe_fifo(dev_config, pipe_idx);
	usbhs_registers_t *const regs = dev_config->regs;
	uint16_t max_transfer_size = uhc_mchp_usbhs_get_pipe_max_transfer_size(regs, pipe_idx);
	size_t count = MIN(length, max_transfer_size);

	/* The fifo memory is in a different device region, so the setup data write must be guarded
	 */
	barrier_dsync_fence_full();
	for (size_t i = 0u; i < count; i++) {
		pipe_data[i] = data[i];
	}
	barrier_dsync_fence_full();

	/* Clear Setup Ready interrupt */
	regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_CTRL_TXSTPIC_Msk;
	/* Enable setup ready interrupt */
	regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_CTRL_TXSTPES_Msk;

	/* Clear FIFOCON and Unfreeze pipe */
	regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;
	regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;
}

/*
 * Send some data over the pipe, taken from a `struct net_buf`.
 */
static int uhc_mchp_usbhs_transmit_from_buf(const struct uhc_mchp_usbhs_config *dev_config,
					    uint8_t pipe_idx, struct net_buf *buf,
					    uint16_t max_packet_size)
{
	volatile uint8_t *pipe_data = uhc_mchp_usbhs_pipe_fifo(dev_config, pipe_idx);
	usbhs_registers_t *const regs = dev_config->regs;
	uint16_t max_transfer_size = uhc_mchp_usbhs_get_pipe_max_transfer_size(regs, pipe_idx);
	/* max_transfer_size is the allocated FIFO bucket, which may be rounded
	 * up past the endpoint's real wMaxPacketSize (see
	 * pipe_max_packet_size_to_psize()). Cap to the real size too, so a
	 * single OUT packet never exceeds what the device actually declared.
	 */
	size_t count = MIN(buf->len, MIN(max_transfer_size, (size_t)max_packet_size));
#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
	usbhs_hstdma_registers_t *dma = &regs->USBHS_HSTDMA[pipe_idx - DMA_FIRST_EP];

	/* If the pipe is enabled for DMA and the transmission is not empty */
	if (uhc_mchp_usbhs_pipe_has_dma(pipe_idx) && buf->len > 0u) {
		/* Flush cache before starting DMA read */
		sys_cache_data_flush_range(buf->data, count);

		dma->USBHS_HSTDMAADDRESS =
			USBHS_HSTDMAADDRESS_BUFF_ADD((uint32_t)(uintptr_t)buf->data);

		/* END_B_EN: stop the transfer when BUFF_COUNT reaches 0
		 * CHANN_ENB: enable the DMA channel
		 */
		dma->USBHS_HSTDMACONTROL = USBHS_HSTDMACONTROL_BUFF_LENGTH(count) |
					   USBHS_HSTDMACONTROL_END_B_EN_Msk |
					   USBHS_HSTDMACONTROL_CHANN_ENB_Msk;

		/* Wait for the packet's bytes to be processed */
		if (!WAIT_FOR(((dma->USBHS_HSTDMASTATUS & USBHS_HSTDMASTATUS_BUFF_COUNT_Msk) >>
			       USBHS_HSTDMASTATUS_BUFF_COUNT_Pos) == 0u,
			      DMA_TIMEOUT_US, k_busy_wait(1u))) {
			LOG_ERR("DMA OUT pipe %u: packet not loaded within timeout", pipe_idx);
			return -ETIMEDOUT;
		}

		net_buf_pull(buf, count);

		/* Clear Tx Out Ready Interrupt */
		regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_TXOUTIC_Msk;
		/* Enable Pipe out ready interrupt */
		regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_TXOUTES_Msk;

		/* Clear FIFOCON and Unfreeze pipe */
		regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;
		regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;
		return 0;
	}
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */

	/* The fifo memory is in a different device region, so the setup data write must be guarded
	 */
	barrier_dsync_fence_full();
	for (size_t i = 0u; i < count; i++) {
		pipe_data[i] = net_buf_pull_u8(buf);
	}
	barrier_dsync_fence_full();

	/* Clear Tx Out Ready Interrupt */
	regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_TXOUTIC_Msk;
	/* Enable Pipe out ready interrupt */
	regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_TXOUTES_Msk;

	/* Clear FIFOCON and Unfreeze pipe */
	regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;
	regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;

	return 0;
}

/*
 * Get some received data from the pipe.
 */
static size_t uhc_mchp_usbhs_unload_endpoint_fifo(const struct device *dev, uint8_t pipe_idx,
						  struct net_buf *buf)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	volatile uint8_t *pipe_data = uhc_mchp_usbhs_pipe_fifo(dev_config, pipe_idx);
	size_t count = 0u;
#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];
	usbhs_hstdma_registers_t *dma = &regs->USBHS_HSTDMA[pipe_idx - DMA_FIRST_EP];
	uint32_t tailroom = 0;
	uint32_t remaining = 0u;
	uint32_t received = 0u;
	uint8_t *dst = NULL;

	/* If the pipe is enabled for DMA and was armed */
	if (uhc_mchp_usbhs_pipe_has_dma(pipe_idx) && pipe->dma_reception_armed) {
		tailroom = net_buf_tailroom(buf);
		remaining = (dma->USBHS_HSTDMASTATUS & USBHS_HSTDMASTATUS_BUFF_COUNT_Msk) >>
			    USBHS_HSTDMASTATUS_BUFF_COUNT_Pos;
		received = tailroom - remaining;
		dst = net_buf_tail(buf);

		/* Invalidate cache again for the CPU to see the data written by the DMA */
		sys_cache_data_invd_range(dst, received);
		net_buf_add(buf, received);

		/* Stop the channel */
		dma->USBHS_HSTDMACONTROL = 0;
		pipe->dma_reception_armed = false;

		return received;
	}
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */

	count = (USBHS_HSTPIPISR_PBYCT_Msk & regs->USBHS_HSTPIPISR[pipe_idx]) >>
		USBHS_HSTPIPISR_PBYCT_Pos;

	if (count > buf->size - buf->len) {
		count = buf->size - buf->len;
	}

	/* The fifo memory is in a different device region, so the setup data write must be guarded
	 */
	barrier_dsync_fence_full();
	for (size_t i = 0u; i < count; i++) {
		net_buf_add_u8(buf, pipe_data[i]);
	}
	barrier_dsync_fence_full();

	return count;
}

/*
 * Get the HSTPIPCFG.PSIZE code for the smallest hardware bucket that can hold
 * the provided max packet size (rounded up if it isn't itself a power of two).
 */
static int uhc_mchp_usbhs_pipe_max_packet_size_to_psize(uint16_t max_packet_size, uint8_t *psize)
{
	/* Interrupt/Isochronous endpoints are not required to have a power-of-2
	 * wMaxPacketSize (USB 2.0 5.7/5.9/9.6.6), unlike Bulk. Round up to the
	 * smallest hardware bucket that fits; callers that need the exact,
	 * un-rounded packet size must not rely on the allocated bucket size.
	 */
	const uint16_t bucket_sizes[] = {8u, 16u, 32u, 64u, 128u, 256u, 512u, 1024u};

	for (size_t i = 0u; i < ARRAY_SIZE(bucket_sizes); i++) {
		if (max_packet_size <= bucket_sizes[i]) {
			*psize = (uint8_t)i;
			return 0;
		}
	}

	return -EINVAL;
}

/*
 * Update the maximum packet size to be transmitted on a pipe.
 */
static int uhc_mchp_usbhs_pipe_update_size(usbhs_registers_t *const regs, uint8_t pipe_idx,
					   uint16_t max_packet_size)
{
	uint8_t psize = 0u;
	int ret = 0;

	ret = uhc_mchp_usbhs_pipe_max_packet_size_to_psize(max_packet_size, &psize);
	if (ret != 0) {
		LOG_ERR("Invalid endpoint size: %i", max_packet_size);
		return ret;
	}

	regs->USBHS_HSTPIPCFG[pipe_idx] &= ~USBHS_HSTPIPCFG_PSIZE_Msk;
	regs->USBHS_HSTPIPCFG[pipe_idx] |= USBHS_HSTPIPCFG_PSIZE(psize);

	return 0;
}

/*
 * Update the address a pipe is targeting.
 */
static void uhc_mchp_usbhs_set_pipe_address(usbhs_registers_t *const regs, size_t pipe_idx,
					    uint8_t address)
{
	const uint32_t masks[NUM_MAX_PIPES] = {
		USBHS_HSTADDR1_HSTADDRP0_Msk, USBHS_HSTADDR1_HSTADDRP1_Msk,
		USBHS_HSTADDR1_HSTADDRP2_Msk, USBHS_HSTADDR1_HSTADDRP3_Msk,
		USBHS_HSTADDR2_HSTADDRP4_Msk, USBHS_HSTADDR2_HSTADDRP5_Msk,
		USBHS_HSTADDR2_HSTADDRP6_Msk, USBHS_HSTADDR2_HSTADDRP7_Msk,
		USBHS_HSTADDR3_HSTADDRP8_Msk,
	};
	const uint32_t positions[NUM_MAX_PIPES] = {
		USBHS_HSTADDR1_HSTADDRP0_Pos, USBHS_HSTADDR1_HSTADDRP1_Pos,
		USBHS_HSTADDR1_HSTADDRP2_Pos, USBHS_HSTADDR1_HSTADDRP3_Pos,
		USBHS_HSTADDR2_HSTADDRP4_Pos, USBHS_HSTADDR2_HSTADDRP5_Pos,
		USBHS_HSTADDR2_HSTADDRP6_Pos, USBHS_HSTADDR2_HSTADDRP7_Pos,
		USBHS_HSTADDR3_HSTADDRP8_Pos,
	};

	if (pipe_idx < 4u) {
		regs->USBHS_HSTADDR1 &= ~masks[pipe_idx];
		regs->USBHS_HSTADDR1 |= (address << positions[pipe_idx]) & masks[pipe_idx];
	} else if (pipe_idx < 8u) {
		regs->USBHS_HSTADDR2 &= ~masks[pipe_idx];
		regs->USBHS_HSTADDR2 |= (address << positions[pipe_idx]) & masks[pipe_idx];
	} else {
		regs->USBHS_HSTADDR3 &= ~masks[pipe_idx];
		regs->USBHS_HSTADDR3 |= (address << positions[pipe_idx]) & masks[pipe_idx];
	}
}

/*
 * Send an OUT packet or prepare reception for an IN packet.
 */
static int uhc_mchp_usbhs_data_packet(const struct device *dev, size_t pipe_idx,
				      struct uhc_transfer *xfer)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */
	int ret = 0;

	if (USB_EP_DIR_IS_OUT(xfer->ep)) {
		ret = uhc_mchp_usbhs_transmit_from_buf(dev_config, pipe_idx, xfer->buf, xfer->mps);
	} else {
#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
		uhc_mchp_usbhs_arm_dma_reception(regs, pipe, pipe_idx, xfer->buf);
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */

		if (xfer->type == USB_EP_TYPE_ISO) {
			/* Isochronous has no NAK/retry: each (micro)frame's IN attempt is
			 * independent and final, whether or not the device answered. INMODE=1 makes
			 * the pipe generate IN requests every frame indefinitely, stopping only
			 * when software explicitly freezes it.
			 */
			regs->USBHS_HSTPIPINRQ[pipe_idx] = USBHS_HSTPIPINRQ_INMODE_Msk;
		} else {
			/* Perform 1 IN requests before freezing the pipe. */
			regs->USBHS_HSTPIPINRQ[pipe_idx] = 0;
		}

		/* Clear RX IN Interrupt */
		regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_RXINIC_Msk;
		/* Enable Rx IN Interrupt */
		regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_RXINES_Msk;

		/* Clear FIFOCON and Unfreeze pipe */
		regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;
		regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;
	}

	return ret;
}

/*
 * Sent a setup packet. While parametrized, only really makes sense for pipe 0.
 */
static int uhc_mchp_usbhs_transmit_setup_packet(const struct device *dev, size_t pipe_idx)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];
	int ret = 0;

	uhc_mchp_usbhs_set_pipe_type(regs, pipe_idx, USBHS_HSTPIPCFG_PTOKEN_SETUP);
	uhc_mchp_usbhs_set_pipe_address(regs, pipe_idx, pipe->xfer->udev->addr);
	ret = uhc_mchp_usbhs_pipe_update_size(regs, pipe_idx, pipe->xfer->mps);
	if (ret != 0) {
		return ret;
	}

	uhc_mchp_usbhs_transmit(dev_config, pipe_idx, pipe->xfer->setup_pkt,
				sizeof(pipe->xfer->setup_pkt));

	pipe->xfer_state = UHC_MCHP_USBHS_XFER_STATE_SETUP;

	return 0;
}

/*
 * Prepare a pipe for operation.
 */
static int uhc_mchp_usbhs_pipe_setup(const struct device *dev, uint8_t device_addr,
				     uint8_t pipe_type, uint8_t ep_addr, uint16_t max_packet_size,
				     uint16_t interval)
{
	const uint32_t enable_irq_masks[NUM_MAX_PIPES] = {
		USBHS_HSTIER_PEP_0_Msk, USBHS_HSTIER_PEP_1_Msk, USBHS_HSTIER_PEP_2_Msk,
		USBHS_HSTIER_PEP_3_Msk, USBHS_HSTIER_PEP_4_Msk, USBHS_HSTIER_PEP_5_Msk,
		USBHS_HSTIER_PEP_6_Msk, USBHS_HSTIER_PEP_7_Msk, USBHS_HSTIER_PEP_8_Msk,
	};
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	size_t pipe_idx = 0u;
	uint32_t hstpipcfg = 0u;
	bool found = false;
	int ret = 0;

	/* Control pipes are always on hardware pipe 0 */
	if (pipe_type == USB_EP_TYPE_CONTROL) {
		pipe_idx = CONTROL_PIPE;

		uhc_mchp_usbhs_enable_pipe(regs, pipe_idx);

		/* Configure Pipe */
		hstpipcfg = USBHS_HSTPIPCFG_PBK_1_BANK | USBHS_HSTPIPCFG_PTOKEN_SETUP |
			    USBHS_HSTPIPCFG_PTYPE_CTRL | USBHS_HSTPIPCFG_PEPNUM(0) |
			    USBHS_HSTPIPCFG_ALLOC_Msk;

		/* Enable high-speed ping */
		if (uhc_mchp_usbhs_is_device_high_speed(regs)) {
			hstpipcfg |= USBHS_HSTPIPCFG_CTRL_BULK_PINGEN_Msk;
		}
	} else {
		/* Look for a pipe already reserved for this exact (address, endpoint) pair, so its
		 * data toggle (HSTPIPISR.DTSEQ) continues correctly instead of being abandoned for
		 * an unrelated pipe.
		 */
		for (pipe_idx = 1u; pipe_idx < NUM_MAX_PIPES; pipe_idx++) {
			struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];

			if (pipe->bound && pipe->bound_addr == device_addr &&
			    pipe->bound_ep == ep_addr) {
				if (pipe->xfer != NULL) {
					/* This endpoint's own pipe is still busy with
					 * another transfer.
					 */
					return -EBUSY;
				}

				LOG_DBG("Pipe %zu reused for addr %u ep 0x%02x", pipe_idx,
					device_addr, ep_addr);
				found = true;
				break;
			}
		}

		/* Otherwise, claim a pipe that has never been bound. A pipe already bound to :
		 * *different* endpoint is never repurposed.
		 */
		if (!found) {
			for (pipe_idx = 1u; pipe_idx < NUM_MAX_PIPES; pipe_idx++) {
				if (!dev_data->pipes[pipe_idx].bound) {
					LOG_DBG("Pipe %zu selected for allocation", pipe_idx);
					found = true;
					break;
				}
			}
		}

		if (!found) {
			LOG_ERR("Not enough pipes");
			return -EIO;
		}

		dev_data->pipes[pipe_idx].bound = true;
		dev_data->pipes[pipe_idx].bound_addr = device_addr;
		dev_data->pipes[pipe_idx].bound_ep = ep_addr;

		uhc_mchp_usbhs_enable_pipe(regs, pipe_idx);

		/* Configure Pipe */
		hstpipcfg = USBHS_HSTPIPCFG_PEPNUM(USB_EP_GET_IDX(ep_addr)) |
			    USBHS_HSTPIPCFG_INTFRQ(interval) | USBHS_HSTPIPCFG_ALLOC_Msk;

		/* Isochronous transfers use two banks for  latency resilience */
		if (pipe_type == USB_EP_TYPE_ISO) {
			hstpipcfg |= USBHS_HSTPIPCFG_PBK_2_BANK | USBHS_HSTPIPCFG_AUTOSW_Msk;
		} else {
			hstpipcfg |= USBHS_HSTPIPCFG_PBK_1_BANK;
		}

		if (pipe_type == USB_EP_TYPE_ISO) {
			hstpipcfg |= USBHS_HSTPIPCFG_PTYPE_ISO;
		} else if (pipe_type == USB_EP_TYPE_BULK) {
			hstpipcfg |= USBHS_HSTPIPCFG_PTYPE_BLK;
		} else if (pipe_type == USB_EP_TYPE_INTERRUPT) {
			hstpipcfg |= USBHS_HSTPIPCFG_PTYPE_INTRPT;
		}

		if (USB_EP_DIR_IS_IN(ep_addr)) {
			hstpipcfg |= USBHS_HSTPIPCFG_PTOKEN_IN;
		} else {
			hstpipcfg |= USBHS_HSTPIPCFG_PTOKEN_OUT;

			/* Enable PINGEN for HS BULK-OUT Pipe */
			if (pipe_type == USB_EP_TYPE_BULK &&
			    uhc_mchp_usbhs_is_device_high_speed(regs)) {
				hstpipcfg |= USBHS_HSTPIPCFG_CTRL_BULK_PINGEN_Msk;
			}
		}
	}

	regs->USBHS_HSTPIPCFG[pipe_idx] = hstpipcfg;

	ret = uhc_mchp_usbhs_pipe_update_size(regs, pipe_idx, max_packet_size);
	if (ret != 0) {
		uhc_mchp_usbhs_disable_pipe(regs, pipe_idx);
		return ret;
	}

	if ((regs->USBHS_HSTPIPISR[pipe_idx] & USBHS_HSTPIPISR_CFGOK_Msk) == 0u) {
		LOG_ERR("Unable to allocate pipe memory");
		uhc_mchp_usbhs_disable_pipe(regs, pipe_idx);

		return -ENOMEM;
	}

	/* Configure pipe address */
	uhc_mchp_usbhs_set_pipe_address(regs, pipe_idx, device_addr);

	if (pipe_type == USB_EP_TYPE_ISO) {
		/* Clear interrupts  */
		regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_ISO_UNDERFIC_Msk |
						  USBHS_HSTPIPICR_OVERFIC_Msk |
						  USBHS_HSTPIPICR_ISO_CRCERRIC_Msk;
		/* Enable interrupts  */
		regs->USBHS_HSTPIPIER[pipe_idx] =
			USBHS_HSTPIPIER_ISO_UNDERFIES_Msk | USBHS_HSTPIPIER_OVERFIES_Msk |
			USBHS_HSTPIPIER_ISO_CRCERRES_Msk | USBHS_HSTPIPIER_PERRES_Msk;
	} else {
		/* Clear stall and error interrupts  */
		regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPIDR_CTRL_RXSTALLDEC_Msk;
		/* Enable stall and error interrupts  */
		regs->USBHS_HSTPIPIER[pipe_idx] =
			USBHS_HSTPIPIER_CTRL_RXSTALLDES_Msk | USBHS_HSTPIPIER_PERRES_Msk;
	}

	/* Enable Pipe Interrupt */
	regs->USBHS_HSTIER = enable_irq_masks[pipe_idx];

	return pipe_idx;
}

/*
 * Enable the UTMI clock.
 */
static int uhc_mchp_usbhs_enable_clock(const struct uhc_mchp_usbhs_config *dev_config)
{
	int ret = 0;

	/* The clock peripheral is in a different device region; add memory barriers to
	 * ensure its configuration is synced with other USBHS register operations.
	 */
	barrier_dsync_fence_full();

	/* Enable 480 MHz UPLL */
	ret = clock_control_on(dev_config->clock,
			       (clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_UPLL, 0));
	if (ret != 0) {
		LOG_ERR("Unable to enable UPLL clock: %i", ret);
		return ret;
	}

	LOG_DBG("USB PLL locked");

	/* If in low power mode use the slower USB CLK */
	if (dev_config->low_power) {
		ret = clock_control_on(
			dev_config->clock,
			(clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_USB_CLK, 0));
		if (ret != 0) {
			LOG_ERR("Unable to enable USB CLK clock: %i", ret);
			return ret;
		}
		LOG_DBG("USB 48MHz clock enabled for Full-Speed mode");
	}

	return 0;
}

/*
 * Disable the UTMI clock.
 */
static int uhc_mchp_usbhs_disable_clock(const struct uhc_mchp_usbhs_config *dev_config)
{
	int ret = 0;

	if (dev_config->low_power) {
		ret = clock_control_off(
			dev_config->clock,
			(clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_USB_CLK, 0));
		if (ret != 0) {
			LOG_ERR("Unable to disable USB CLK clock: %i", ret);
			return ret;
		}
	}

	ret = clock_control_off(dev_config->clock,
				(clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_UPLL, 0));
	if (ret != 0) {
		LOG_ERR("Unable to disable UPLL: %i", ret);
		return ret;
	}

	LOG_DBG("USB PLL disabled");

	return 0;
}

static int uhc_mchp_usbhs_lock(const struct device *dev)
{
	return uhc_lock_internal(dev, K_FOREVER);
}

static int uhc_mchp_usbhs_unlock(const struct device *dev)
{
	return uhc_unlock_internal(dev);
}

static int uhc_mchp_usbhs_sof_enable(const struct device *dev)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;

	if ((regs->USBHS_HSTCTRL & USBHS_HSTCTRL_SOFE_Msk) != 0u) {
		LOG_DBG("SoF was already enabled");
		return -EALREADY;
	}

	regs->USBHS_HSTCTRL |= USBHS_HSTCTRL_SOFE_Msk;

	LOG_DBG("SOF enable");

	return 0;
}

static int uhc_mchp_usbhs_bus_suspend(const struct device *dev)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	/* Freeze all pipes */
	for (size_t pipe_idx = 0u; pipe_idx < NUM_MAX_PIPES; pipe_idx++) {
		if (dev_data->pipes[pipe_idx].xfer != NULL) {
			regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_PFREEZES_Msk;
		}
	}

	regs->USBHS_HSTCTRL &= ~USBHS_HSTCTRL_SOFE_Msk;
	uhc_submit_event(dev, UHC_EVT_SUSPENDED, 0);

	LOG_DBG("Bus suspend");

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

static int uhc_mchp_usbhs_bus_reset(const struct device *dev)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	regs->USBHS_HSTCTRL |= USBHS_HSTCTRL_RESET_Msk;

	if (!WAIT_FOR((regs->USBHS_HSTCTRL & USBHS_HSTCTRL_RESET_Msk) == 0u, RESET_TIMEOUT_US,
		      k_msleep(1))) {
		LOG_ERR("Timed out waiting for reset");
		ret = -ETIMEDOUT;
		goto cleanup;
	}

	/* A reset disables and deallocates all pipes; setup the control one */
	ret = uhc_mchp_usbhs_pipe_setup(dev, 0, USB_EP_TYPE_CONTROL, 0, 8, 0);
	if (ret != 0) {
		LOG_ERR("Unable to setup pipe");
		goto cleanup;
	}

	LOG_DBG("Bus reset");

cleanup:
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int uhc_mchp_usbhs_bus_resume(const struct device *dev)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	regs->USBHS_HSTCTRL |= USBHS_HSTCTRL_SOFE_Msk;
	regs->USBHS_HSTCTRL |= USBHS_HSTCTRL_RESUME_Msk;

	for (size_t pipe_idx = 0u; pipe_idx < NUM_MAX_PIPES; pipe_idx++) {
		if (dev_data->pipes[pipe_idx].xfer != NULL) {
			regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;
		}
	}

	uhc_submit_event(dev, UHC_EVT_RESUMED, 0);

	LOG_DBG("Bus resume");

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

static int uhc_mchp_usbhs_enqueue(const struct device *dev, struct uhc_transfer *const xfer)
{
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	ret = uhc_xfer_append(dev, xfer);

	k_mutex_unlock(&dev_data->mutex);

	k_event_post(&dev_data->events, EVENT_XFER_ENQUEUED);

	LOG_DBG("Enqueue");

	return ret;
}

static int uhc_mchp_usbhs_dequeue(const struct device *dev, struct uhc_transfer *const xfer)
{
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	struct uhc_data *data = dev->data;
	struct uhc_transfer *tmp;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	LOG_DBG("Dequeue");

	SYS_DLIST_FOR_EACH_CONTAINER(&data->ctrl_xfers, tmp, node) {
		if (xfer == tmp) {
			tmp->err = -ECONNRESET;
			break;
		}
	}

	k_mutex_unlock(&dev_data->mutex);

	k_event_post(&dev_data->events, EVENT_XFER_ENQUEUED);

	return 0;
}

static int uhc_mchp_usbhs_init(const struct device *dev)
{
	LOG_DBG("Init device");

	return 0;
}

static int uhc_mchp_usbhs_enable(const struct device *dev)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	int enable_ret = 0;
	int ret = 0;

	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Failed to enable USBHS clock");
		return ret;
	}

	/* Hardware reset of the USBHS block */
	regs->USBHS_CTRL &= ~USBHS_CTRL_USBE_Msk;
	regs->USBHS_CTRL = USBHS_CTRL_USBE_Msk | USBHS_CTRL_VBUSHWC_Msk;

	if (dev_config->low_power) {
		regs->USBHS_HSTCTRL = USBHS_HSTCTRL_SPDCONF_LOW_POWER;
	} else {
		regs->USBHS_HSTCTRL = USBHS_HSTCTRL_SPDCONF_NORMAL;
	}

	ret = uhc_mchp_usbhs_enable_clock(dev_config);
	if (ret != 0) {
		enable_ret = ret;

		LOG_ERR("Failed to turn on peripheral clock: %i", ret);

		ret = uhc_mchp_usbhs_disable(dev);
		if (ret != 0) {
			LOG_ERR("Failed to fully clean up after enable failure: %i", ret);
		}

		return enable_ret;
	}

	/* Wait for the UTMI clock to be usable */
	if (!WAIT_FOR(((regs->USBHS_SR & USBHS_SR_CLKUSABLE_Msk) != 0), CLOCK_USABLE_TIMEOUT_US,
		      k_busy_wait(1))) {
		LOG_ERR("Timed out waiting UTMI clock");

		ret = uhc_mchp_usbhs_disable(dev);
		if (ret != 0) {
			LOG_ERR("Failed to fully clean up after enable failure: %i", ret);
		}

		return -ETIMEDOUT;
	}

	regs->USBHS_HSTIER = USBHS_HSTIER_DCONNIES_Msk;
	regs->USBHS_SFR |= USBHS_SFR_VBUSRQS_Msk;

	dev_config->irq_config_enable(dev);

	return 0;
}

static int uhc_mchp_usbhs_disable(const struct device *dev)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	int ret = 0;
	int first_err = 0;

	LOG_DBG("Disable device %s", dev->name);

	dev_config->irq_config_disable(dev);

	ret = uhc_mchp_usbhs_disable_clock(dev_config);
	if (ret != 0) {
		LOG_ERR("Failed to disable USB clock: %i", ret);
		first_err = first_err ? first_err : ret;
	}

	/* Turn off the peripheral */
	regs->USBHS_CTRL = USBHS_CTRL_FRZCLK_Msk | USBHS_CTRL_VBUSHWC_Msk;

	ret = clock_control_off(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Failed to turn off peripheral clock: %i", ret);
		first_err = first_err ? first_err : ret;
	}

	return first_err;
}

static int uhc_mchp_usbhs_shutdown(const struct device *dev)
{
	LOG_DBG("Shutdown device");

	return 0;
}

static int uhc_mchp_usbhs_driver_init(const struct device *dev)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;

	dev_config->make_thread(dev);

	LOG_DBG("Driver initialized");

	return 0;
}

static DEVICE_API(uhc, uhc_mchp_usbhs_driver_api) = {
	.lock = uhc_mchp_usbhs_lock,
	.unlock = uhc_mchp_usbhs_unlock,
	.init = uhc_mchp_usbhs_init,
	.enable = uhc_mchp_usbhs_enable,
	.disable = uhc_mchp_usbhs_disable,
	.shutdown = uhc_mchp_usbhs_shutdown,

	.bus_reset = uhc_mchp_usbhs_bus_reset,
	.sof_enable = uhc_mchp_usbhs_sof_enable,
	.bus_suspend = uhc_mchp_usbhs_bus_suspend,
	.bus_resume = uhc_mchp_usbhs_bus_resume,

	.ep_enqueue = uhc_mchp_usbhs_enqueue,
	.ep_dequeue = uhc_mchp_usbhs_dequeue,
};

/*
 * Check if transfers are available and should be scheduled.
 */
static int uhc_mchp_usbhs_schedule_xfer(const struct device *dev)
{
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_data *uhc_data = dev->data;
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	struct uhc_mchp_usbhs_pipe *ctrl_pipe = &dev_data->pipes[CONTROL_PIPE];
	struct uhc_transfer *xfer = NULL;
	struct uhc_transfer *xfer_next = NULL;
	int pipe_idx = 0;
	int ret = 0;

	SYS_DLIST_FOR_EACH_CONTAINER_SAFE(&uhc_data->ctrl_xfers, xfer, xfer_next, node) {
		pipe_idx = uhc_mchp_usbhs_get_pipe_for_transfer(dev_data, xfer);

		/* The transfer was dequeued; cancel it */
		if (xfer->err == -ECONNRESET) {
			if (pipe_idx < 0) {
				LOG_ERR("Transfer unaccounted for");
			} else {
				uhc_mchp_usbhs_disable_pipe(regs, pipe_idx);
				dev_data->pipes[pipe_idx].xfer = NULL;
			}
			LOG_DBG("Transfer removed");
			uhc_xfer_return(dev, xfer, xfer->err);
			continue;
		}

		if (xfer->type == USB_EP_TYPE_CONTROL) {
			if (ctrl_pipe->xfer == NULL) {
				ctrl_pipe->xfer = xfer;

				LOG_HEXDUMP_DBG(xfer->setup_pkt, sizeof(xfer->setup_pkt),
						"Next control transfer");
				if (USB_EP_DIR_IS_OUT(xfer->ep)) {
					if (xfer->buf != NULL) {
						LOG_HEXDUMP_DBG(xfer->buf->data, xfer->buf->len,
								"Out data");
					}
				}
				ret = uhc_mchp_usbhs_transmit_setup_packet(dev, CONTROL_PIPE);
				if (ret != 0) {
					LOG_ERR("Unable to transmit setup packet: %i", ret);
					ctrl_pipe->xfer = NULL;
					uhc_xfer_return(dev, xfer, ret);
					continue;
				}
			}
		} else if (pipe_idx < 0) {
			pipe_idx = uhc_mchp_usbhs_pipe_setup(dev, xfer->udev->addr, xfer->type,
							     xfer->ep, xfer->mps, xfer->interval);
			if (pipe_idx == -EBUSY) {
				/* This endpoint already has a pipe reserved, but it's still busy
				 * with another transfer. Wait until it becomes available
				 */
				continue;
			} else if (pipe_idx < 0) {
				LOG_ERR("Unable to setup pipe: %i", pipe_idx);
				uhc_xfer_return(dev, xfer, pipe_idx);
				continue;
			}

			ret = uhc_mchp_usbhs_data_packet(dev, pipe_idx, xfer);
			if (ret != 0) {
				uhc_mchp_usbhs_disable_pipe(regs, pipe_idx);
				LOG_ERR("Unable to transmit data packet: %i", ret);
				uhc_xfer_return(dev, xfer, ret);
				continue;
			}
			LOG_DBG("New transfer started");

			dev_data->pipes[pipe_idx].xfer_state = UHC_MCHP_USBHS_XFER_STATE_DATA;
			dev_data->pipes[pipe_idx].xfer = xfer;
		}
	}

	return ret;
}

/*
 * Handle a pipe's setup stage.
 */
static int uhc_mchp_usbhs_handle_setup_stage(const struct device *dev, size_t pipe_idx,
					     uint32_t flags)
{
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];
	int ret = 0;

	if ((flags & PIPE_FLAG_SETUP_TRANSMITTED) != 0u) {
		LOG_DBG("Setup transmitted on pipe %zu", pipe_idx);

		/* Freeze Pipe */
		regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_PFREEZES_Msk;

		if (pipe->xfer->buf != NULL) {
			LOG_DBG("Data stage on pipe %zu", pipe_idx);

			if (USB_EP_DIR_IS_IN(pipe->xfer->ep)) {
#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
				uhc_mchp_usbhs_arm_dma_reception(regs, pipe, pipe_idx,
								 pipe->xfer->buf);
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */

				uhc_mchp_usbhs_set_pipe_type(regs, pipe_idx,
							     USBHS_HSTPIPCFG_PTOKEN_IN);

				/* Clear IN Rx Interrupt */
				regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_RXINIC_Msk;
				/* Enable IN Rx Interrupt */
				regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_RXINES_Msk;

				/* Clear FIFOCON and Unfreeze pipe */
				regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;
				regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;
			} else {
				uhc_mchp_usbhs_set_pipe_type(regs, pipe_idx,
							     USBHS_HSTPIPCFG_PTOKEN_OUT);

				ret = uhc_mchp_usbhs_transmit_from_buf(
					dev_config, pipe_idx, pipe->xfer->buf, pipe->xfer->mps);
			}

			pipe->xfer_state = UHC_MCHP_USBHS_XFER_STATE_DATA;
		} else {
			/* This means there is no data stage. We can proceed to the
			 * handshake stage, which means sending an IN token to the device
			 */
			LOG_DBG("Direct status stage on pipe %zu", pipe_idx);

			uhc_mchp_usbhs_set_pipe_type(regs, pipe_idx, USBHS_HSTPIPCFG_PTOKEN_IN);

			/* Clear IN Rx Interrupt */
			regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_RXINIC_Msk;
			/* Enable IN Rx Interrupt */
			regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_RXINES_Msk;

			/* Clear FIFOCON and Unfreeze pipe */
			regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;
			regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;

			pipe->xfer_state = UHC_MCHP_USBHS_XFER_STATE_STATUS;
		}
	}

	return ret;
}

/*
 * Handle a pipe's setup stage.
 */
static int uhc_mchp_usbhs_handle_data_stage(const struct device *dev, size_t pipe_idx,
					    uint32_t flags)
{
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];
	uint16_t max_transfer_size = uhc_mchp_usbhs_get_pipe_max_transfer_size(regs, pipe_idx);
	size_t count = 0u;
	size_t requested_len = 0u;
	int ret = 0;

	if ((flags & PIPE_FLAG_IN_PACKET) != 0u) {
		LOG_DBG("Packet received on pipe %zu", pipe_idx);
		/* In case of a low USB speed and a high CPU frequency,
		 * the ACK may lag behind the processor; wait for it
		 */
		if (uhc_mchp_usbhs_is_device_low_speed(regs)) {
			if (!WAIT_FOR((USBHS_HSTPIPIMR_PFREEZE_Msk &
				       regs->USBHS_HSTPIPIMR[pipe_idx]) == 0u,
				      LOW_SPEED_ACK_TIMEOUT_US, k_busy_wait(1))) {
				LOG_ERR("Data stage timed out waiting for low speed ACK");

				return -ETIMEDOUT;
			}
		}
	} else if ((flags & PIPE_FLAG_OUT_PACKET) != 0u) {
		LOG_DBG("Packet sent on pipe %zu", pipe_idx);
	} else {
		return 0;
	}

	/* Data was received or sent */
	if (USB_EP_DIR_IS_IN(pipe->xfer->ep)) {
		/* Fetch the received packet */
		count = uhc_mchp_usbhs_unload_endpoint_fifo(dev, pipe_idx, pipe->xfer->buf);
		/* The actual length is store differently depending if this is a control transfer */
		requested_len = pipe->xfer->buf->size;

		if (pipe_idx == CONTROL_PIPE) {
			requested_len = (size_t)pipe->xfer->setup_pkt[6] |
					((size_t)pipe->xfer->setup_pkt[7] << 8);
		}

		if ((count < max_transfer_size) || (pipe->xfer->buf->len >= requested_len)) {
			/* Only control transfers need a status stage */
			if (pipe_idx == CONTROL_PIPE) {
				/* Configure pipe for OUT token */
				uhc_mchp_usbhs_set_pipe_type(regs, pipe_idx,
							     USBHS_HSTPIPCFG_PTOKEN_OUT);

				/* Clear Tx Out Ready Interrupt */
				regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_TXOUTIC_Msk;
				/* Enable Pipe out ready interrupt */
				regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_TXOUTES_Msk;

				/* Clear FIFOCON and Unfreeze pipe */
				regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;
				regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;

				pipe->xfer_state = UHC_MCHP_USBHS_XFER_STATE_STATUS;
			} else {
				/* Done */
				uhc_mchp_usbhs_pipe_done(dev, pipe_idx, 0);
			}
		} else {
			/* Release the bank just unloaded before checking for the next reception, or
			 * FIFOCON still reflects the packet we just consumed.
			 */
			regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;

#if defined(CONFIG_UHC_MCHP_USBHS_G2_DMA)
			uhc_mchp_usbhs_arm_dma_reception(regs, pipe, pipe_idx, pipe->xfer->buf);
#endif /* CONFIG_UHC_MCHP_USBHS_G2_DMA */

			/* The transfer is not done yet; issue another IN token */
			uhc_mchp_usbhs_set_pipe_type(regs, pipe_idx, USBHS_HSTPIPCFG_PTOKEN_IN);

			/* Clear IN Rx Interrupt */
			regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_RXINIC_Msk;
			/* Enable IN Rx Interrupt */
			regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_RXINES_Msk;

			/* Unfreeze pipe */
			regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;
		}
	} else {
		if (pipe->xfer->buf->len > 0u) {
			ret = uhc_mchp_usbhs_transmit_from_buf(dev_config, pipe_idx,
							       pipe->xfer->buf, pipe->xfer->mps);
		} else if (pipe_idx == CONTROL_PIPE) {
			/* Only control transfers need a status stage */
			uhc_mchp_usbhs_set_pipe_type(regs, pipe_idx, USBHS_HSTPIPCFG_PTOKEN_IN);

			/* Clear IN Rx Interrupt */
			regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_RXINIC_Msk;
			/* Enable IN Rx Interrupt */
			regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_RXINES_Msk;

			/* Clear FIFOCON and Unfreeze pipe */
			regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_FIFOCONC_Msk;
			regs->USBHS_HSTPIPIDR[pipe_idx] = USBHS_HSTPIPIDR_PFREEZEC_Msk;

			pipe->xfer_state = UHC_MCHP_USBHS_XFER_STATE_STATUS;
		} else {
			/* Done */
			uhc_mchp_usbhs_pipe_done(dev, pipe_idx, 0);
		}
	}

	return ret;
}

/*
 * Handle a pipe's status stage.
 */
static int uhc_mchp_usbhs_handle_status_stage(const struct device *dev, size_t pipe_idx,
					      uint32_t flags)
{
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];

	if ((flags & PIPE_FLAG_IN_PACKET) != 0u) {
		LOG_DBG("Packet received on pipe %zu", pipe_idx);
		/* In case of a low USB speed and a high CPU frequency,
		 * the ACK may lag behind the processor; wait for it
		 */
		if (uhc_mchp_usbhs_is_device_low_speed(regs)) {
			if (!WAIT_FOR((USBHS_HSTPIPIMR_PFREEZE_Msk &
				       regs->USBHS_HSTPIPIMR[pipe_idx]) == 0u,
				      LOW_SPEED_ACK_TIMEOUT_US, k_busy_wait(1))) {
				LOG_ERR("Status stage timed out waiting for low speed ACK");

				return -ETIMEDOUT;
			}
		}
		/* Clear IN Rx Interrupt */
		regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_RXINIC_Msk;
	} else if ((flags & PIPE_FLAG_OUT_PACKET) != 0u) {
		LOG_DBG("Packet sent on pipe %zu", pipe_idx);
	} else {
		return 0;
	}

	LOG_DBG("Transfer done on pipe %zu", pipe_idx);

	/* See section 9.2.6.3 of the USB 2.0 specification; a set address request
	 * is allows up to 2ms of recovery time to the device.
	 */
	if (pipe_idx == CONTROL_PIPE &&
	    pipe->xfer->setup_pkt[0] == (USB_REQTYPE_DIR_TO_DEVICE << 7) &&
	    pipe->xfer->setup_pkt[1] == USB_SREQ_SET_ADDRESS) {
		k_msleep(2);
	}

	if (pipe->xfer->buf != NULL) {
		LOG_DBG("In data, %i of %i", pipe->xfer->buf->len, pipe->xfer->buf->size);
		LOG_HEXDUMP_DBG(pipe->xfer->buf->data, pipe->xfer->buf->len, "In data");
	}

	uhc_mchp_usbhs_pipe_done(dev, pipe_idx, 0);

	return 0;
}

/*
 * Handle a pipe's state.
 */
static int uhc_mchp_usbhs_handle_state(const struct device *dev, size_t pipe_idx)
{
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];
	uint32_t flags = atomic_clear(&pipe->flags);

	if (pipe->xfer == NULL) {
		return 0;
	} else if ((flags & PIPE_FLAG_STALL) != 0u) {
		LOG_WRN("Stall on pipe %zu (xfer_state=%d HSTPIPISR=0x%08X HSTPIPCFG=0x%08X)",
			pipe_idx, pipe->xfer_state, regs->USBHS_HSTPIPISR[pipe_idx],
			regs->USBHS_HSTPIPCFG[pipe_idx]);
		/* Reset DATA Toggle */
		regs->USBHS_HSTPIPIER[pipe_idx] = USBHS_HSTPIPIER_RSTDTS_Msk;

		uhc_mchp_usbhs_cancel_xfer(dev, pipe_idx, -EPIPE);
		return 0;
	} else if ((flags & PIPE_FLAG_ERRORS) != 0u) {
		LOG_WRN("Error on pipe %zu (0x%lX)", pipe_idx, (flags & PIPE_FLAG_ERRORS) >> 2u);

		uhc_mchp_usbhs_cancel_xfer(dev, pipe_idx, -EIO);
		return 0;
	} else if ((flags & PIPE_FLAG_ISO_CRC_ERROR) != 0u) {
		LOG_WRN("ISO CRC error on pipe %zu", pipe_idx);

		if (pipe->xfer->type == USB_EP_TYPE_ISO && USB_EP_DIR_IS_IN(pipe->xfer->ep) &&
		    pipe->xfer_state == UHC_MCHP_USBHS_XFER_STATE_DATA) {
			return uhc_mchp_usbhs_handle_iso_in_gap(dev, pipe_idx);
		}

		uhc_mchp_usbhs_cancel_xfer(dev, pipe_idx, -EIO);
		return 0;
	} else if ((flags & (PIPE_FLAG_ISO_UNDERFLOW | PIPE_FLAG_ISO_OVERFLOW)) != 0u &&
		   (flags & (PIPE_FLAG_IN_PACKET | PIPE_FLAG_OUT_PACKET)) == 0u) {
		/* Only if an error has occurred but no packet is reported to be transmitted or
		 * received, because the error may arise after a successful transmission before we
		 * are able to handle it in the case of ISO
		 */
		LOG_WRN("ISO frame error on pipe %zu", pipe_idx);

		if (pipe->xfer->type == USB_EP_TYPE_ISO && USB_EP_DIR_IS_IN(pipe->xfer->ep) &&
		    pipe->xfer_state == UHC_MCHP_USBHS_XFER_STATE_DATA) {
			return uhc_mchp_usbhs_handle_iso_in_gap(dev, pipe_idx);
		}

		uhc_mchp_usbhs_cancel_xfer(dev, pipe_idx, -EIO);
		return 0;
	}

	switch (pipe->xfer_state) {
	case UHC_MCHP_USBHS_XFER_STATE_SETUP:
		return uhc_mchp_usbhs_handle_setup_stage(dev, pipe_idx, flags);
	case UHC_MCHP_USBHS_XFER_STATE_DATA:
		return uhc_mchp_usbhs_handle_data_stage(dev, pipe_idx, flags);
	case UHC_MCHP_USBHS_XFER_STATE_STATUS:
		return uhc_mchp_usbhs_handle_status_stage(dev, pipe_idx, flags);
	}

	return 0;
}

/*
 * The driver thread.
 */
static void uhc_mchp_usbhs_thread_handler(const struct device *dev)
{
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	uint32_t events = 0u;

	LOG_DBG("UHC thread started");

	for (;;) {
		events = k_event_wait_safe(&dev_data->events, 0xFFFFFFFF, false, K_FOREVER);

		k_mutex_lock(&dev_data->mutex, K_FOREVER);

		/* Connection event */
		if ((events & EVENT_CONNECTION) != 0u) {
			LOG_INF("Device connected");

			/* Check device speed */
			switch ((regs->USBHS_SR & USBHS_SR_SPEED_Msk) >> USBHS_SR_SPEED_Pos) {
			case USBHS_SR_SPEED_HIGH_SPEED_Val:
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_HS, 0);
				LOG_DBG("High speed device");
				break;
			case USBHS_SR_SPEED_FULL_SPEED_Val:
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_FS, 0);
				LOG_DBG("Full speed device");
				break;
			case USBHS_SR_SPEED_LOW_SPEED_Val:
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_LS, 0);
				LOG_DBG("Low speed device");
				break;
			default:
				LOG_ERR("Invalid speed value detected");
				break;
			}

			/* Enable disconnection interrupt */
			regs->USBHS_HSTIER = USBHS_HSTIER_DDISCIES_Msk;
		}

		if ((events & EVENT_DISCONNECTION) != 0u) {
			LOG_INF("Device disconnected");

			uhc_submit_event(dev, UHC_EVT_DEV_REMOVED, 0);

			/* Release every idle pipe's endpoint binding */
			for (size_t pipe_idx = 1u; pipe_idx < NUM_MAX_PIPES; pipe_idx++) {
				struct uhc_mchp_usbhs_pipe *pipe = &dev_data->pipes[pipe_idx];

				if (pipe->bound && pipe->xfer == NULL) {
					uhc_mchp_usbhs_disable_pipe(regs, pipe_idx);
					pipe->bound = false;
				}
			}

			/* Stop reset signal, in case of disconnection during reset */
			regs->USBHS_HSTCTRL &= ~USBHS_HSTCTRL_RESET_Msk;
			/* Disable SOF */
			regs->USBHS_HSTCTRL &= ~USBHS_HSTCTRL_SOFE_Msk;
			/* Clear Device Disconnection Interrupt */
			regs->USBHS_HSTICR = USBHS_HSTICR_DDISCIC_Msk;
			/* Disable wakeup/resumes interrupts,
			 * in case of disconnection during suspend mode
			 */
			regs->USBHS_HSTIDR = (USBHS_HSTIDR_HWUPIEC_Msk | USBHS_HSTIDR_RSMEDIEC_Msk |
					      USBHS_HSTIDR_RXRSMIEC_Msk);
			/* Clear Wake-up Interrupt */
			regs->USBHS_HSTICR = USBHS_HSTICR_HWUPIC_Msk;
			/* Enable Host Wakeup Interrupt */
			regs->USBHS_HSTIER = USBHS_HSTIER_HWUPIES_Msk;

			if ((events & EVENT_CONNECTION) != 0u) {
				/* Both connection and disconnection interrupts are set
				 * and to sort this out the connection flag is cleared
				 * at cost of loss the connection interrupt.
				 */
				regs->USBHS_HSTICR = USBHS_HSTICR_DCONNIC_Msk;
			}

			regs->USBHS_SFR |= USBHS_SFR_VBUSRQS_Msk;
		}

		if ((events & EVENT_WAKE_UP) != 0u) {
			LOG_INF("Wake up");
			/* Disable Host Wakeup Interrupt */
			regs->USBHS_HSTIDR = USBHS_HSTIDR_HWUPIEC_Msk;
			/* Clear Wake-up Interrupt */
			regs->USBHS_HSTICR = USBHS_HSTICR_HWUPIC_Msk;
		}

		if ((events & EVENT_XFER_ENQUEUED) != 0u) {
			LOG_DBG("New transfer enqueued");
			uhc_mchp_usbhs_schedule_xfer(dev);
		}

		if ((events & EVENT_PIPE) != 0u) {
			LOG_DBG("Pipe event");

			for (size_t pipe_idx = 0u; pipe_idx < NUM_MAX_PIPES; pipe_idx++) {
				int ret = uhc_mchp_usbhs_handle_state(dev, pipe_idx);

				if (ret != 0) {
					uhc_mchp_usbhs_pipe_done(dev, pipe_idx, ret);
				}
			}

			uhc_mchp_usbhs_schedule_xfer(dev);
		}

		k_mutex_unlock(&dev_data->mutex);
	}
}

static void uhc_mchp_usbhs_isr_handler(const struct device *dev)
{
	struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);
	const struct uhc_mchp_usbhs_config *dev_config = dev->config;
	usbhs_registers_t *const regs = dev_config->regs;
	uint32_t status_register = 0u;
	uint32_t pipe_status_register = 0u;
	uint32_t events = 0u;
	uint32_t pipe_flags = 0u;

	/* Store the current interrupt flags */
	status_register = regs->USBHS_HSTISR & regs->USBHS_HSTIMR;

	/* Global interrupts */
	if ((status_register & USBHS_HSTISR_DCONNI_Msk) != 0u) {
		regs->USBHS_HSTICR = USBHS_HSTICR_DCONNIC_Msk;
		events |= EVENT_CONNECTION;
	}
	if ((status_register & USBHS_HSTISR_DDISCI_Msk) != 0u) {
		regs->USBHS_HSTICR = USBHS_HSTICR_DDISCIC_Msk;
		events |= EVENT_DISCONNECTION;
	}
	if ((status_register & USBHS_HSTISR_HWUPI_Msk) != 0u) {
		regs->USBHS_HSTICR = USBHS_HSTICR_HWUPIC_Msk;
		events |= EVENT_WAKE_UP;
	}

	/* Check pipe events */
	for (size_t pipe_idx = 0u; pipe_idx < NUM_MAX_PIPES; pipe_idx++) {
		pipe_flags = 0u;

		if ((status_register & BIT(USBHS_HSTISR_PEP_0_Pos + pipe_idx)) != 0u) {
			events |= EVENT_PIPE;

			/* Store the control pipe interrupt flags */
			pipe_status_register = regs->USBHS_HSTPIPISR[pipe_idx];

			if ((pipe_status_register & USBHS_HSTPIPISR_PERRI_Msk) != 0u) {
				/* Ack the error */
				if ((regs->USBHS_HSTPIPERR[pipe_idx] &
				     USBHS_HSTPIPERR_DATATGL_Msk) != 0u) {
					pipe_flags |= PIPE_FLAG_DATATGL_ERROR;
				}
				if ((regs->USBHS_HSTPIPERR[pipe_idx] &
				     USBHS_HSTPIPERR_DATAPID_Msk) != 0u) {
					pipe_flags |= PIPE_FLAG_DATAPID_ERROR;
				}
				if ((regs->USBHS_HSTPIPERR[pipe_idx] & USBHS_HSTPIPERR_PID_Msk) !=
				    0u) {
					pipe_flags |= PIPE_FLAG_PID_ERROR;
				}
				if ((regs->USBHS_HSTPIPERR[pipe_idx] &
				     USBHS_HSTPIPERR_TIMEOUT_Msk) != 0u) {
					pipe_flags |= PIPE_FLAG_TIMEOUT_ERROR;
				}
				if ((regs->USBHS_HSTPIPERR[pipe_idx] & USBHS_HSTPIPERR_CRC16_Msk) !=
				    0u) {
					pipe_flags |= PIPE_FLAG_CRC16_ERROR;
				}

				regs->USBHS_HSTPIPERR[pipe_idx] = 0;
			}
			if ((pipe_status_register & USBHS_HSTPIPISR_RXINI_Msk) != 0u) {
				/* Clear IN Rx Interrupt */
				regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_RXINIC_Msk;
				pipe_flags |= PIPE_FLAG_IN_PACKET;
			}
			if ((pipe_status_register & USBHS_HSTPIPISR_TXOUTI_Msk) != 0u) {
				/* Clear Tx Interrupt */
				regs->USBHS_HSTPIPICR[pipe_idx] = USBHS_HSTPIPICR_TXOUTIC_Msk;
				pipe_flags |= PIPE_FLAG_OUT_PACKET;
			}

			/* Special handling of isochronous transfers */
			if ((regs->USBHS_HSTPIPCFG[pipe_idx] & USBHS_HSTPIPCFG_PTYPE_Msk) ==
			    USBHS_HSTPIPCFG_PTYPE_ISO) {
				if ((pipe_status_register & USBHS_HSTPIPISR_ISO_UNDERFI_Msk) !=
				    0u) {
					/* Clear underflow interrupt */
					regs->USBHS_HSTPIPICR[pipe_idx] =
						USBHS_HSTPIPICR_ISO_UNDERFIC_Msk;
					pipe_flags |= PIPE_FLAG_ISO_UNDERFLOW;
				}

				if ((pipe_status_register & USBHS_HSTPIPISR_OVERFI_Msk) != 0u) {
					/* Clear overflow interrupt */
					regs->USBHS_HSTPIPICR[pipe_idx] =
						USBHS_HSTPIPICR_OVERFIC_Msk;
					pipe_flags |= PIPE_FLAG_ISO_OVERFLOW;
				}
				if ((pipe_status_register & USBHS_HSTPIPISR_ISO_CRCERRI_Msk) !=
				    0u) {
					/* Clear overflow interrupt */
					regs->USBHS_HSTPIPICR[pipe_idx] =
						USBHS_HSTPIPICR_ISO_CRCERRIC_Msk;
					pipe_flags |= PIPE_FLAG_ISO_CRC_ERROR;
				}
			} else {
				if ((pipe_status_register & USBHS_HSTPIPISR_CTRL_TXSTPI_Msk) !=
				    0u) {
					regs->USBHS_HSTPIPICR[pipe_idx] =
						USBHS_HSTPIPICR_CTRL_TXSTPIC_Msk;
					pipe_flags |= PIPE_FLAG_SETUP_TRANSMITTED;
				}
				if ((pipe_status_register & USBHS_HSTPIPISR_CTRL_RXSTALLDI_Msk) !=
				    0u) {
					regs->USBHS_HSTPIPICR[pipe_idx] =
						USBHS_HSTPIPICR_CTRL_RXSTALLDIC_Msk;
					pipe_flags |= PIPE_FLAG_STALL;
				}
			}

			atomic_or(&dev_data->pipes[pipe_idx].flags, pipe_flags);
		}
	}

	k_event_post(&dev_data->events, events);
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	static void irq_config_enable_##n(const struct device *dev)                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uhc_mchp_usbhs_isr_handler, \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static void irq_config_disable_##n(const struct device *dev)                               \
	{                                                                                          \
		irq_disable(DT_INST_IRQN(n));                                                      \
	}                                                                                          \
                                                                                                   \
	static K_KERNEL_STACK_DEFINE(uhc_mchp_usbhs_stack_##n,                                     \
				     CONFIG_UHC_MCHP_USBHS_G2_THREAD_STACK_SIZE);                  \
                                                                                                   \
	static void uhc_mchp_usbhs_thread_##n(void *dev, void *arg1, void *arg2)                   \
	{                                                                                          \
		ARG_UNUSED(arg1);                                                                  \
		ARG_UNUSED(arg2);                                                                  \
		for (;;) {                                                                         \
			uhc_mchp_usbhs_thread_handler(dev);                                        \
		}                                                                                  \
	}                                                                                          \
                                                                                                   \
	static void uhc_mchp_usbhs_make_thread_##n(const struct device *dev)                       \
	{                                                                                          \
		struct uhc_mchp_usbhs_data *dev_data = uhc_get_private(dev);                       \
                                                                                                   \
		k_thread_create(&dev_data->thread_data, uhc_mchp_usbhs_stack_##n,                  \
				K_THREAD_STACK_SIZEOF(uhc_mchp_usbhs_stack_##n),                   \
				uhc_mchp_usbhs_thread_##n, (void *)dev, NULL, NULL,                \
				K_PRIO_COOP(CONFIG_UHC_MCHP_USBHS_G2_THREAD_PRIORITY),             \
				K_ESSENTIAL, K_NO_WAIT);                                           \
		k_thread_name_set(&dev_data->thread_data, dev->name);                              \
	}                                                                                          \
                                                                                                   \
	static const struct uhc_mchp_usbhs_config mchp_usbhs_##n##_config = {                      \
		.regs = (usbhs_registers_t *)DT_INST_REG_ADDR(n),                                  \
		.fifo_base = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(n, fifo),                        \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys =                                                                    \
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_IDX(n, 0, subsystem),       \
		.low_power = DT_INST_ENUM_IDX_OR(n, maximum_speed, 2) <= 1u,                       \
		.irq_config_enable = irq_config_enable_##n,                                        \
		.irq_config_disable = irq_config_disable_##n,                                      \
		.make_thread = uhc_mchp_usbhs_make_thread_##n,                                     \
	};                                                                                         \
                                                                                                   \
	static struct uhc_mchp_usbhs_data mchp_usbhs_##n##_data = {                                \
		.events = Z_EVENT_INITIALIZER(mchp_usbhs_##n##_data.events),                       \
		.mutex = Z_MUTEX_INITIALIZER(mchp_usbhs_##n##_data.mutex),                         \
	};                                                                                         \
                                                                                                   \
	static struct uhc_data mchp_usbhs_##n##_uhc_data = {                                       \
		.mutex = Z_MUTEX_INITIALIZER(mchp_usbhs_##n##_uhc_data.mutex),                     \
		.priv = &mchp_usbhs_##n##_data,                                                    \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, uhc_mchp_usbhs_driver_init, NULL, &mchp_usbhs_##n##_uhc_data,     \
			      &mchp_usbhs_##n##_config, POST_KERNEL,                               \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uhc_mchp_usbhs_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
