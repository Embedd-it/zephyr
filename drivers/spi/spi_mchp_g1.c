/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file spi_mchp_g1.c
 * @brief Microchip SPI controller driver.
 */

#define DT_DRV_COMPAT microchip_spi_g1

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_mchp_g1, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"
#include <errno.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include "spi_rtio.h"
#include <zephyr/spinlock.h>

#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#define SPI_MCHP_CHIP_SELECT_COUNT 4

/* Number of bytes in transfer before using DMA if available */
#define SPI_MCHP_DMA_THRESHOLD 32

#ifndef SPI_RDR_RD__EXTR
#define SPI_RDR_RD__EXTR(reg) (((reg) & SPI_RDR_RD_Msk) >> SPI_RDR_RD_Pos)
#endif

/* Device constant configuration parameters */
struct spi_mchp_config {
	spi_registers_t *regs;
	const struct device *clock;
	clock_control_subsys_t clock_cfg;
	const struct pinctrl_dev_config *pcfg;
	bool loopback;
	void (*irq_config_func)(const struct device *dev);

#if defined(CONFIG_SPI_MCHP_G1_DMA)
	const struct device *dma_dev;
	const uint32_t dma_tx_channel;
	const uint32_t dma_tx_perid;
	const uint32_t dma_rx_channel;
	const uint32_t dma_rx_perid;
#endif /* CONFIG_SPI_MCHP_G1_DMA */
};

/* Device run time data */
struct spi_mchp_data {
	struct spi_context ctx;
	struct k_spinlock lock;
	bool is_host;

#if defined(CONFIG_SPI_RTIO)
	struct spi_rtio *rtio_ctx;
#endif

#if defined(CONFIG_SPI_MCHP_G1_DMA)
	struct k_sem dma_done;
#endif /* CONFIG_SPI_MCHP_G1_DMA */

#if defined(CONFIG_SPI_MCHP_G1_INTERRUPT)
	int32_t txrx_balance;
#endif
	unsigned int data_bits;
};

static inline k_spinlock_key_t spi_mchp_spin_lock(const struct device *dev)
{
	struct spi_mchp_data *dev_data = dev->data;

	return k_spin_lock(&dev_data->lock);
}

static inline void spi_mchp_spin_unlock(const struct device *dev, k_spinlock_key_t key)
{
	struct spi_mchp_data *dev_data = dev->data;

	k_spin_unlock(&dev_data->lock, key);
}

static int spi_mchp_target_to_mr_pcs(int target)
{
	int pcs[SPI_MCHP_CHIP_SELECT_COUNT] = {0x0, 0x1, 0x3, 0x7};

	/*
	 * PCS = xxx0    NPCS[3:0] = 1110
	 * PCS = xx01    NPCS[3:0] = 1101
	 * PCS = x011    NPCS[3:0] = 1011
	 * PCS = 0111    NPCS[3:0] = 0111
	 */

	return pcs[target];
}

static int spi_mchp_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_mchp_config *dev_config = dev->config;
	struct spi_mchp_data *dev_data = dev->data;
	spi_registers_t *const regs = dev_config->regs;
	uint32_t spi_mr = 0u, spi_csr = 0u;
	uint16_t spi_csr_idx = spi_cs_is_gpio(config) ? 0 : config->peripheral;
	int div, ret;
	uint32_t clkin_freq;

	dev_data->is_host = SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_CONTROLLER;

	if (spi_context_configured(&dev_data->ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (dev_data->is_host) {
		if (spi_csr_idx > (SPI_MCHP_CHIP_SELECT_COUNT - 1)) {
			LOG_ERR("Target %d is greater than %d", spi_csr_idx,
				SPI_MCHP_CHIP_SELECT_COUNT - 1);
			return -EINVAL;
		}
		/* Set host mode, disable mode fault detection, set fixed
		 * peripheral select mode.
		 */
		spi_mr |= (SPI_MR_MSTR_Msk | SPI_MR_MODFDIS_Msk);
		spi_mr |= SPI_MR_PCS(spi_mchp_target_to_mr_pcs(spi_csr_idx));
		if (dev_config->loopback) {
			spi_mr |= SPI_MR_LLB_Msk;
		}
	} else {
		spi_mr |= SPI_MR_MODFDIS_Msk;
		spi_mr &= ~(SPI_MR_MSTR_Msk);
	}

	if ((config->operation & SPI_MODE_CPOL) != 0u) {
		spi_csr |= SPI_CSR_CPOL_Msk;
	}

	if ((config->operation & SPI_MODE_CPHA) == 0u) {
		spi_csr |= SPI_CSR_NCPHA_Msk;
	}

	if ((SPI_WORD_SIZE_GET(config->operation) < 8) ||
	    (SPI_WORD_SIZE_GET(config->operation) > 16)) {
		return -ENOTSUP;
	}

	/* SPI_CA70 representation */
	dev_data->data_bits = SPI_WORD_SIZE_GET(config->operation) - 8;

	spi_csr |= SPI_CSR_BITS(dev_data->data_bits);

	/* Use the requested or next highest possible frequency */
	ret = clock_control_get_rate(dev_config->clock, dev_config->clock_cfg, &clkin_freq);
	if (ret != 0) {
		return ret;
	}

	div = clkin_freq / config->frequency;
	div = CLAMP(div, 1, UINT8_MAX);
	spi_csr |= SPI_CSR_SCBR(div);

	regs->SPI_CR = SPI_CR_SPIDIS_Msk; /* Disable SPI */
	regs->SPI_MR = spi_mr;
	regs->SPI_CSR[spi_csr_idx] = spi_csr;
	regs->SPI_CR = SPI_CR_SPIEN_Msk; /* Enable SPI */

	dev_data->ctx.config = config;

	return 0;
}

/* Avoid flooding mmio, emit instruction that takes several cpu cycles to retire
 */
#define spi_mchp_busy_wait() arch_nop()

static int spi_mchp_finish(const struct device *dev)
{
	const struct spi_mchp_config *dev_config = dev->config;
	spi_registers_t *const regs = dev_config->regs;

	if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_TXEMPTY_Msk) != 0), CONFIG_SPI_MCHP_G1_CHECK_TIME,
		      NULL)) {
		return -ETIMEDOUT;
	}

	if (regs->SPI_SR & SPI_SR_RDRF_Msk) {
		(void)regs->SPI_RDR;
	}

	return 0;
}

#define SPI_MCHP_G1_FAST_TX(buffer_sz, buffer_t)                                                   \
	/* Fast path that transmits a buf */                                                       \
	static int spi_mchp_fast_tx##buffer_sz(spi_registers_t *const regs,                        \
					       const buffer_t *tx_buf, const uint32_t tx_buf_len)  \
	{                                                                                          \
		const buffer_t *p = tx_buf;                                                        \
		const buffer_t *pend = (buffer_t *)((uintptr_t)tx_buf + tx_buf_len);               \
		buffer_t ch;                                                                       \
                                                                                                   \
		while (p != pend) {                                                                \
			ch = *p++;                                                                 \
                                                                                                   \
			if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_TDRE_Msk) != 0),                     \
				      CONFIG_SPI_MCHP_G1_CHECK_TIME, NULL)) {                      \
				return -ETIMEDOUT;                                                 \
			}                                                                          \
                                                                                                   \
			regs->SPI_TDR = SPI_TDR_TD(ch);                                            \
		}                                                                                  \
                                                                                                   \
		return 0;                                                                          \
	}

#define SPI_MCHP_G1_FAST_RX(buffer_sz, buffer_t)                                                   \
	/* Fast path that reads into a buf */                                                      \
	static int spi_mchp_fast_rx##buffer_sz(spi_registers_t *const regs, buffer_t *rx_buf,      \
					       const uint32_t rx_buf_len)                          \
	{                                                                                          \
		buffer_t *rx = rx_buf;                                                             \
		int len = rx_buf_len;                                                              \
                                                                                                   \
		if (len <= 0) {                                                                    \
			return -EINVAL;                                                            \
		}                                                                                  \
                                                                                                   \
		/* Write the first byte */                                                         \
		regs->SPI_TDR = SPI_TDR_TD(0);                                                     \
		len -= sizeof(buffer_t);                                                           \
                                                                                                   \
		while (len) {                                                                      \
			if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_TDRE_Msk) != 0),                     \
				      CONFIG_SPI_MCHP_G1_CHECK_TIME, NULL)) {                      \
				return -ETIMEDOUT;                                                 \
			}                                                                          \
                                                                                                   \
			/* Read byte N+0 from the receive register */                              \
			if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_RDRF_Msk) != 0),                     \
				      CONFIG_SPI_MCHP_G1_CHECK_TIME, NULL)) {                      \
				return -ETIMEDOUT;                                                 \
			}                                                                          \
                                                                                                   \
			*rx = (buffer_t)SPI_RDR_RD__EXTR(regs->SPI_RDR);                           \
			rx++;                                                                      \
                                                                                                   \
			/* Load byte N+1 into the transmit register */                             \
			regs->SPI_TDR = SPI_TDR_TD(0);                                             \
			len -= sizeof(buffer_t);                                                   \
		}                                                                                  \
                                                                                                   \
		/* Read the final incoming byte */                                                 \
		if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_RDRF_Msk) != 0),                             \
			      CONFIG_SPI_MCHP_G1_CHECK_TIME, NULL)) {                              \
			return -ETIMEDOUT;                                                         \
		}                                                                                  \
                                                                                                   \
		*rx = (buffer_t)SPI_RDR_RD__EXTR(regs->SPI_RDR);                                   \
                                                                                                   \
		return 0;                                                                          \
	}

#define SPI_MCHP_G1_FAST_TXRX(buffer_sz, buffer_t)                                                 \
	/* Fast path that writes and reads bufs of the same length */                              \
	static int spi_mchp_fast_txrx##buffer_sz(spi_registers_t *const regs,                      \
						 const buffer_t *tx_buf, const buffer_t *rx_buf,   \
						 const uint32_t len)                               \
	{                                                                                          \
		const buffer_t *tx = tx_buf;                                                       \
		const buffer_t *txend = tx_buf + len;                                              \
		buffer_t *rx = (buffer_t *)rx_buf;                                                 \
                                                                                                   \
		if (len == 0) {                                                                    \
			return -EINVAL;                                                            \
		}                                                                                  \
                                                                                                   \
		while (tx != txend) {                                                              \
			if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_TDRE_Msk) != 0),                     \
				      CONFIG_SPI_MCHP_G1_CHECK_TIME, NULL)) {                      \
				return -ETIMEDOUT;                                                 \
			}                                                                          \
                                                                                                   \
			regs->SPI_TDR = SPI_TDR_TD(*tx++);                                         \
                                                                                                   \
			/* Read byte N+0 from the receive register */                              \
			if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_RDRF_Msk) != 0),                     \
				      CONFIG_SPI_MCHP_G1_CHECK_TIME, NULL)) {                      \
				return -ETIMEDOUT;                                                 \
			}                                                                          \
                                                                                                   \
			*rx++ = (buffer_t)SPI_RDR_RD__EXTR(regs->SPI_RDR);                         \
		}                                                                                  \
                                                                                                   \
		return 0;                                                                          \
	}

#define SPI_MCHP_FAST_FUNCS(buffer_sz, buffer_t)                                                   \
	SPI_MCHP_G1_FAST_TX(buffer_sz, buffer_t)                                                   \
	SPI_MCHP_G1_FAST_RX(buffer_sz, buffer_t)                                                   \
	SPI_MCHP_G1_FAST_TXRX(buffer_sz, buffer_t)

SPI_MCHP_FAST_FUNCS(8, uint8_t)
SPI_MCHP_FAST_FUNCS(16, uint16_t)

static int spi_mchp_fast_tx(const struct device *dev, const void *tx_buf, const uint32_t tx_buf_len)
{
	struct spi_mchp_data *dev_data = dev->data;
	const struct spi_mchp_config *dev_config = dev->config;
	spi_registers_t *const regs = dev_config->regs;

	return dev_data->data_bits ? spi_mchp_fast_tx16(regs, tx_buf, tx_buf_len)
				   : spi_mchp_fast_tx8(regs, tx_buf, tx_buf_len);
}

static int spi_mchp_fast_rx(const struct device *dev, void *rx_buf, const uint32_t rx_buf_len)
{
	struct spi_mchp_data *dev_data = dev->data;
	const struct spi_mchp_config *dev_config = dev->config;
	spi_registers_t *const regs = dev_config->regs;

	return dev_data->data_bits ? spi_mchp_fast_rx16(regs, rx_buf, rx_buf_len)
				   : spi_mchp_fast_rx8(regs, rx_buf, rx_buf_len);
}

static int spi_mchp_fast_txrx(const struct device *dev, const void *tx_buf, const void *rx_buf,
			      const uint32_t len)
{
	struct spi_mchp_data *dev_data = dev->data;
	const struct spi_mchp_config *dev_config = dev->config;
	spi_registers_t *const regs = dev_config->regs;

	return dev_data->data_bits ? spi_mchp_fast_txrx16(regs, tx_buf, rx_buf, len)
				   : spi_mchp_fast_txrx8(regs, tx_buf, rx_buf, len);
}

#if defined(CONFIG_SPI_MCHP_G1_DMA)

static __aligned(4) uint32_t tx_dummy;
static __aligned(4) uint32_t rx_dummy;

#if defined(CONFIG_SPI_RTIO)
static void spi_mchp_iodev_complete(const struct device *dev, int status);
#endif

static void spi_mchp_dma_callback(const struct device *dma_dev, void *user_data, uint32_t channel,
				  int status)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);
	ARG_UNUSED(status);

	const struct device *dev = user_data;
	struct spi_mchp_data *dev_data = dev->data;

#if defined(CONFIG_SPI_RTIO)
	struct spi_rtio *rtio_ctx = dev_data->rtio_ctx;

	if (rtio_ctx->txn_head != NULL) {
		spi_mchp_iodev_complete(dev, status);
		return;
	}
#endif
	k_sem_give(&dev_data->dma_done);
}

/* DMA transceive path */
static int spi_mchp_dma_txrx(const struct device *dev, const uint8_t *tx_buf, const uint8_t *rx_buf,
			     const uint32_t len)
{
	const struct spi_mchp_config *dev_config = dev->config;
	struct spi_mchp_data *dev_data = dev->data;
	spi_registers_t *const regs = dev_config->regs;
#if defined(CONFIG_SPI_RTIO)
	struct spi_rtio *rtio_ctx = dev_data->rtio_ctx;
	bool blocking = rtio_ctx->txn_head == NULL;
#else
	bool blocking = true;
#endif

	int ret = 0;

	__ASSERT_NO_MSG(rx_buf != NULL || tx_buf != NULL);

	struct dma_config rx_dma_cfg = {
		.source_data_size = 1,
		.dest_data_size = 1,
		.block_count = 1,
		.dma_slot = dev_config->dma_rx_perid,
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.complete_callback_en = true,
		.dma_callback = spi_mchp_dma_callback,
		.user_data = (void *)dev,
	};

	uint32_t dest_address, dest_addr_adjust;

	if (rx_buf != NULL) {
		dest_address = (uint32_t)rx_buf;
		dest_addr_adjust = DMA_ADDR_ADJ_INCREMENT;
		(void)sys_cache_data_invd_range((void *)dest_address, len);
	} else {
		dest_address = (uint32_t)&rx_dummy;
		dest_addr_adjust = DMA_ADDR_ADJ_NO_CHANGE;
	}

	struct dma_block_config rx_block_cfg = {
		.dest_addr_adj = dest_addr_adjust,
		.block_size = len,
		.source_address = (uint32_t)&regs->SPI_RDR,
		.dest_address = dest_address,
	};

	rx_dma_cfg.head_block = &rx_block_cfg;

	struct dma_config tx_dma_cfg = {
		.source_data_size = 1,
		.dest_data_size = 1,
		.block_count = 1,
		.dma_slot = dev_config->dma_tx_perid,
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.source_burst_length = 1,
		.dest_burst_length = 1,
		.complete_callback_en = true,
		.dma_callback = NULL,
		.user_data = (void *)dev,
	};

	uint32_t source_address, source_addr_adjust;

	if (tx_buf != NULL) {
		source_address = (uint32_t)tx_buf;
		source_addr_adjust = DMA_ADDR_ADJ_INCREMENT;
		(void)sys_cache_data_flush_range((void *)source_address, len);
	} else {
		source_address = (uint32_t)&tx_dummy;
		source_addr_adjust = DMA_ADDR_ADJ_NO_CHANGE;
	}

	struct dma_block_config tx_block_cfg = {
		.source_addr_adj = source_addr_adjust,
		.block_size = len,
		.source_address = source_address,
		.dest_address = (uint32_t)&regs->SPI_TDR,
	};

	tx_dma_cfg.head_block = &tx_block_cfg;

	ret = dma_config(dev_config->dma_dev, dev_config->dma_rx_channel, &rx_dma_cfg);
	if (ret != 0) {
		LOG_ERR("failed to configure SPI DMA RX");
		goto out;
	}

	ret = dma_config(dev_config->dma_dev, dev_config->dma_tx_channel, &tx_dma_cfg);
	if (ret != 0) {
		LOG_ERR("failed to configure SPI DMA TX");
		goto out;
	}

	/* Clocking begins on tx, so start rx first */
	ret = dma_start(dev_config->dma_dev, dev_config->dma_rx_channel);
	if (ret != 0) {
		LOG_ERR("failed to start SPI DMA RX");
		goto out;
	}

	ret = dma_start(dev_config->dma_dev, dev_config->dma_tx_channel);
	if (ret != 0) {
		LOG_ERR("failed to start SPI DMA TX");
		dma_stop(dev_config->dma_dev, dev_config->dma_rx_channel);
	}

	/* Move up a level or wrap in branch when blocking */
	if (blocking != 0) {
		k_sem_take(&dev_data->dma_done, K_FOREVER);
		if (ret == 0) {
			ret = spi_mchp_finish(dev);
		}
	} else {
		ret = -EWOULDBLOCK;
	}

out:
	return ret;
}

#endif /* CONFIG_SPI_MCHP_G1_DMA */

static inline int spi_mchp_rx(const struct device *dev, spi_registers_t *const regs,
			      uint8_t *rx_buf, uint32_t rx_buf_len)
{
	k_spinlock_key_t key;
	int ret;

#if defined(CONFIG_SPI_MCHP_G1_DMA)
	const struct spi_mchp_config *dev_config = dev->config;

	if ((rx_buf_len < SPI_MCHP_DMA_THRESHOLD || dev_config->dma_dev == NULL) &&
	    !IS_ENABLED(CONFIG_SPI_RTIO)) {
		key = spi_mchp_spin_lock(dev);
		ret = spi_mchp_fast_rx(dev, rx_buf, rx_buf_len);
		if (ret == 0) {
			ret = spi_mchp_finish(dev);
		}
		spi_mchp_spin_unlock(dev, key);
	} else {
		/* RTIO Transfers should always fall here */
		return spi_mchp_dma_txrx(dev, NULL, rx_buf, rx_buf_len);
	}
#else
	key = spi_mchp_spin_lock(dev);
	ret = spi_mchp_fast_rx(dev, rx_buf, rx_buf_len);
	if (ret == 0) {
		ret = spi_mchp_finish(dev);
	}
	spi_mchp_spin_unlock(dev, key);
#endif

	return ret;
}

static inline int spi_mchp_tx(const struct device *dev, spi_registers_t *const regs,
			      const uint8_t *tx_buf, uint32_t tx_buf_len)
{
	k_spinlock_key_t key;
	int ret;

#if defined(CONFIG_SPI_MCHP_G1_DMA)
	const struct spi_mchp_config *dev_config = dev->config;

	if ((tx_buf_len < SPI_MCHP_DMA_THRESHOLD || dev_config->dma_dev == NULL) &&
	    !IS_ENABLED(CONFIG_SPI_RTIO)) {
		key = spi_mchp_spin_lock(dev);
		ret = spi_mchp_fast_tx(dev, tx_buf, tx_buf_len);
		if (ret == 0) {
			ret = spi_mchp_finish(dev);
		}
		spi_mchp_spin_unlock(dev, key);
	} else {
		/* RTIO Transfers should always fall here */
		return spi_mchp_dma_txrx(dev, tx_buf, NULL, tx_buf_len);
	}
#else
	key = spi_mchp_spin_lock(dev);
	ret = spi_mchp_fast_tx(dev, tx_buf, tx_buf_len);
	if (ret == 0) {
		ret = spi_mchp_finish(dev);
	}
	spi_mchp_spin_unlock(dev, key);
#endif

	return 0;
}

static inline int spi_mchp_txrx(const struct device *dev, spi_registers_t *const regs,
				const uint8_t *tx_buf, const uint8_t *rx_buf, uint32_t buf_len)
{
	k_spinlock_key_t key;
	int ret;

#if defined(CONFIG_SPI_MCHP_G1_DMA)
	const struct spi_mchp_config *dev_config = dev->config;

	if ((buf_len < SPI_MCHP_DMA_THRESHOLD || dev_config->dma_dev == NULL) &&
	    !IS_ENABLED(CONFIG_SPI_RTIO)) {
		key = spi_mchp_spin_lock(dev);
		ret = spi_mchp_fast_txrx(dev, tx_buf, rx_buf, buf_len);
		if (ret == 0) {
			ret = spi_mchp_finish(dev);
		}
		spi_mchp_spin_unlock(dev, key);
	} else {
		/* RTIO Transfers should always fall here */
		return spi_mchp_dma_txrx(dev, tx_buf, rx_buf, buf_len);
	}
#else
	key = spi_mchp_spin_lock(dev);
	ret = spi_mchp_fast_txrx(dev, tx_buf, rx_buf, buf_len);
	if (ret == 0) {
		ret = spi_mchp_finish(dev);
	}
	spi_mchp_spin_unlock(dev, key);
#endif

	return 0;
}

#if !defined(CONFIG_SPI_RTIO)
static void spi_mchp_prepare(const struct device *dev)
{
	const struct spi_mchp_config *dev_config = dev->config;
	spi_registers_t *const regs = dev_config->regs;
#if defined(CONFIG_SPI_MCHP_G1_INTERRUPT)
	struct spi_mchp_data *dev_data = dev->data;

	dev_data->txrx_balance = 0;
#endif

	/* Cleaning old RX data from previous TX only transfer */
	if (regs->SPI_SR & SPI_SR_RDRF_Msk) {
		(void)regs->SPI_RDR;
	}
}
#endif

#if (!defined(CONFIG_SPI_RTIO)) && (!defined(CONFIG_SPI_MCHP_G1_INTERRUPT))

/* Fast path where every overlapping tx and rx buffer is the same length */
static void spi_mchp_fast_transceive(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs)
{
	const struct spi_mchp_config *dev_config = dev->config;
	spi_registers_t *const regs = dev_config->regs;
	size_t tx_count = 0;
	size_t rx_count = 0;
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;

	if (tx_bufs != 0) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs != 0) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	while (tx_count != 0 && rx_count != 0) {
		if (tx->buf == NULL) {
			spi_mchp_rx(dev, regs, rx->buf, rx->len);
		} else if (rx->buf == NULL) {
			spi_mchp_tx(dev, regs, tx->buf, tx->len);
		} else if (rx->len == tx->len) {
			spi_mchp_txrx(dev, regs, tx->buf, rx->buf, rx->len);
		} else {
			__ASSERT_NO_MSG("Invalid fast transceive configuration");
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	for (; tx_count != 0; tx_count--) {
		spi_mchp_tx(dev, regs, tx->buf, tx->len);
		tx++;
	}

	for (; rx_count != 0; rx_count--) {
		spi_mchp_rx(dev, regs, rx->buf, rx->len);
		rx++;
	}
}

static bool spi_mchp_transfer_ongoing(struct spi_mchp_data *dev_data)
{
	return spi_context_tx_on(&dev_data->ctx) || spi_context_rx_on(&dev_data->ctx);
}

#define SPI_MCHP_G1_SHIFT_HOST(buffer_sz, buffer_t)                                                \
	static int spi_mchp_shift_host##buffer_sz(spi_registers_t *const regs,                     \
						  struct spi_mchp_data *dev_data)                  \
	{                                                                                          \
		buffer_t tx, rx;                                                                   \
                                                                                                   \
		if (spi_context_tx_buf_on(&dev_data->ctx)) {                                       \
			tx = *(buffer_t *)(dev_data->ctx.tx_buf);                                  \
		} else {                                                                           \
			tx = 0u;                                                                   \
		}                                                                                  \
                                                                                                   \
		if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_TDRE_Msk) != 0),                             \
			      CONFIG_SPI_MCHP_G1_CHECK_TIME, NULL)) {                              \
			return -ETIMEDOUT;                                                         \
		}                                                                                  \
                                                                                                   \
		regs->SPI_TDR = SPI_TDR_TD(tx);                                                    \
		spi_context_update_tx(&dev_data->ctx, sizeof(buffer_t), 1);                        \
                                                                                                   \
		if (!WAIT_FOR(((regs->SPI_SR & SPI_SR_RDRF_Msk) != 0),                             \
			      CONFIG_SPI_MCHP_G1_CHECK_TIME, NULL)) {                              \
			return -ETIMEDOUT;                                                         \
		}                                                                                  \
                                                                                                   \
		rx = (buffer_t)SPI_RDR_RD__EXTR(regs->SPI_RDR);                                    \
                                                                                                   \
		if (spi_context_rx_buf_on(&dev_data->ctx)) {                                       \
			*dev_data->ctx.rx_buf = rx;                                                \
		}                                                                                  \
		spi_context_update_rx(&dev_data->ctx, sizeof(buffer_t), 1);                        \
                                                                                                   \
		return 0;                                                                          \
	}

SPI_MCHP_G1_SHIFT_HOST(8, uint8_t)
SPI_MCHP_G1_SHIFT_HOST(16, uint16_t)

/* Returns true if the request is suitable for the fast
 * path. Specifically, the bufs are a sequence of:
 *
 * - Zero or more RX and TX buf pairs where each is the same length.
 * - Zero or more trailing RX only bufs
 * - Zero or more trailing TX only bufs
 */
static bool spi_mchp_is_regular(const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;
	size_t tx_count = 0;
	size_t rx_count = 0;

	if (tx_bufs != NULL) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs != NULL) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	if (tx == NULL || rx == NULL) {
		return true;
	}

	while (tx_count != 0 && rx_count != 0) {
		if (tx->len != rx->len) {
			return false;
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	return true;
}

#elif defined(CONFIG_SPI_RTIO)

static void spi_mchp_iodev_complete(const struct device *dev, int status);

static void spi_mchp_iodev_start(const struct device *dev)
{
	const struct spi_mchp_config *dev_config = dev->config;
	struct spi_mchp_data *dev_data = dev->data;
	struct spi_rtio *rtio_ctx = dev_data->rtio_ctx;
	struct rtio_sqe *sqe = &rtio_ctx->txn_curr->sqe;
	int ret = 0;

	switch (sqe->op) {
	case RTIO_OP_RX:
		ret = spi_mchp_rx(dev, dev_config->regs, sqe->rx.buf, sqe->rx.buf_len);
		break;
	case RTIO_OP_TX:
		ret = spi_mchp_tx(dev, dev_config->regs, sqe->tx.buf, sqe->tx.buf_len);
		break;
	case RTIO_OP_TINY_TX:
		ret = spi_mchp_tx(dev, dev_config->regs, sqe->tiny_tx.buf, sqe->tiny_tx.buf_len);
		break;
	case RTIO_OP_TXRX:
		ret = spi_mchp_txrx(dev, dev_config->regs, sqe->txrx.tx_buf, sqe->txrx.rx_buf,
				    sqe->txrx.buf_len);
		break;
	default:
		LOG_ERR("Invalid op code %d for submission %p", sqe->op, (void *)sqe);
		spi_mchp_iodev_complete(dev, -EINVAL);
		return;
	}

	/** Completion of the RTIO transfer should come through the DMA
	 * callback when successful, otherwise complete it here as an error.
	 */
	if (ret != 0 && ret != -EWOULDBLOCK) {
		spi_mchp_iodev_complete(dev, ret);
	}
}

static inline void spi_mchp_iodev_prepare_start(const struct device *dev)
{
	struct spi_mchp_data *dev_data = dev->data;
	struct spi_rtio *rtio_ctx = dev_data->rtio_ctx;
	struct spi_dt_spec *spi_dt_spec = rtio_ctx->txn_curr->sqe.iodev->data;
	struct spi_config *spi_config = &spi_dt_spec->config;
	int ret;

	ret = spi_mchp_configure(dev, spi_config);
	__ASSERT(!ret, "%d", ret);

	spi_context_cs_control(&dev_data->ctx, true);
}

static void spi_mchp_iodev_complete(const struct device *dev, int status)
{
	struct spi_mchp_data *dev_data = dev->data;
	struct spi_rtio *rtio_ctx = dev_data->rtio_ctx;

	if ((status >= 0) && (rtio_ctx->txn_curr->sqe.flags & RTIO_SQE_TRANSACTION)) {
		rtio_ctx->txn_curr = rtio_txn_next(rtio_ctx->txn_curr);
		spi_mchp_iodev_start(dev);
	} else {
		/** De-assert CS-line to space from next transaction */
		spi_context_cs_control(&dev_data->ctx, false);

		if (spi_rtio_complete(rtio_ctx, status)) {
			spi_mchp_iodev_prepare_start(dev);
			spi_mchp_iodev_start(dev);
		}
	}
}

static void spi_mchp_iodev_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct spi_mchp_data *dev_data = dev->data;
	struct spi_rtio *rtio_ctx = dev_data->rtio_ctx;

	if (spi_rtio_submit(rtio_ctx, iodev_sqe)) {
		spi_mchp_iodev_prepare_start(dev);
		spi_mchp_iodev_start(dev);
	}
}
#endif

static int spi_mchp_transceive(const struct device *dev, const struct spi_config *config,
			       const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
			       spi_callback_t spi_callback, void *userdata, bool asynchronous)
{
	struct spi_mchp_data *dev_data = dev->data;
	int ret = 0;

	if (!IS_ENABLED(CONFIG_SPI_MCHP_G1_INTERRUPT) && asynchronous) {
		return -ENOTSUP;
	}

	spi_context_lock(&dev_data->ctx, asynchronous, spi_callback, userdata, config);

	do {
#if defined(CONFIG_SPI_RTIO)
		struct spi_rtio *rtio_ctx = dev_data->rtio_ctx;

		ret = spi_rtio_transceive(rtio_ctx, config, tx_bufs, rx_bufs);
#else
		const struct spi_mchp_config *dev_config = dev->config;

		ret = spi_mchp_configure(dev, config);
		if (ret != 0) {
			break;
		}

		spi_mchp_prepare(dev);

#if defined(CONFIG_SPI_MCHP_G1_INTERRUPT)
		if (dev_data->is_host) {
			spi_context_cs_control(&dev_data->ctx, true);
			spi_context_buffers_setup(&dev_data->ctx, tx_bufs, rx_bufs,
						  dev_data->data_bits ? 2 : 1);
			dev_config->regs->SPI_IER = (SPI_IER_TXEMPTY_Msk | SPI_IER_RDRF_Msk);
		} else {
			spi_context_buffers_setup(&dev_data->ctx, tx_bufs, rx_bufs,
						  dev_data->data_bits ? 2 : 1);
			dev_config->regs->SPI_IER = (SPI_IER_TDRE_Msk | SPI_IER_TXEMPTY_Msk |
						     SPI_IER_RDRF_Msk | SPI_IER_NSSR_Msk);
		}

		ret = spi_context_wait_for_completion(&dev_data->ctx);
#else
		if (dev_data->is_host) {
			spi_context_cs_control(&dev_data->ctx, true);
		}

		if (spi_mchp_is_regular(tx_bufs, rx_bufs)) {
			spi_mchp_fast_transceive(dev, config, tx_bufs, rx_bufs);
		} else {
			if (dev_data->data_bits) {
				spi_context_buffers_setup(&dev_data->ctx, tx_bufs, rx_bufs, 2);
				do {
					spi_mchp_shift_host16(dev_config->regs, dev_data);
				} while (spi_mchp_transfer_ongoing(dev_data));
			} else {
				spi_context_buffers_setup(&dev_data->ctx, tx_bufs, rx_bufs, 1);
				do {
					spi_mchp_shift_host8(dev_config->regs, dev_data);
				} while (spi_mchp_transfer_ongoing(dev_data));
			}
		}

		if (dev_data->is_host) {
			spi_context_cs_control(&dev_data->ctx, false);
		}
#endif
#endif
	} while (0);

	spi_context_release(&dev_data->ctx, ret);

	return ret;
}

static int spi_mchp_transceive_sync(const struct device *dev, const struct spi_config *config,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs)
{
	return spi_mchp_transceive(dev, config, tx_bufs, rx_bufs, NULL, NULL, false);
}

#if defined(CONFIG_SPI_ASYNC)
static int spi_mchp_transceive_async(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs, spi_callback_t cb,
				     void *userdata)
{
	if (cb == NULL) {
		return -EINVAL;
	}
	return spi_mchp_transceive(dev, config, tx_bufs, rx_bufs, cb, userdata, true);
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_mchp_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_mchp_data *dev_data = dev->data;

	spi_context_unlock_unconditionally(&dev_data->ctx);

	return 0;
}

static int spi_mchp_init(const struct device *dev)
{
	int ret;
	const struct spi_mchp_config *dev_config = dev->config;
	struct spi_mchp_data *dev_data = dev->data;

	/* Enable SPI clock in PMC */
	ret = clock_control_on(dev_config->clock, dev_config->clock_cfg);
	if (ret < 0) {
		return -ENODEV;
	}

	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = spi_context_cs_configure_all(&dev_data->ctx);
	if (ret < 0) {
		return ret;
	}

#if defined(CONFIG_SPI_MCHP_G1_DMA) && !defined(CONFIG_SPI_MCHP_G1_INTERRUPT)
	k_sem_init(&dev_data->dma_done, 0, K_SEM_MAX_LIMIT);
#elif !defined(CONFIG_SPI_MCHP_G1_DMA) && defined(CONFIG_SPI_MCHP_G1_INTERRUPT)
	dev_config->irq_config_func(dev);
#endif

#if defined(CONFIG_SPI_RTIO)
	spi_rtio_init(dev_data->rtio_ctx, dev);
#endif

	spi_context_unlock_unconditionally(&dev_data->ctx);

	/* The device will be configured and enabled when transceive
	 * is called.
	 */

	return 0;
}

#if defined(CONFIG_SPI_MCHP_G1_INTERRUPT)
static void spi_mchp_host_isr(const struct device *dev)
{
	const struct spi_mchp_config *dev_config = dev->config;
	struct spi_mchp_data *dev_data = dev->data;
	uint16_t rx_data, tx_data;
	uint32_t status_reg = dev_config->regs->SPI_SR;
	uint8_t dfs = dev_data->data_bits ? 2 : 1;

	if ((status_reg & SPI_SR_TXEMPTY_Msk) && !spi_context_rx_on(&dev_data->ctx) &&
	    !spi_context_tx_on(&dev_data->ctx)) {
		/* transfer completed */
		dev_config->regs->SPI_IDR = (SPI_IDR_TDRE_Msk | SPI_IDR_TXEMPTY_Msk |
					     SPI_IDR_RDRF_Msk | SPI_IDR_NSSR_Msk);
		spi_context_complete(&dev_data->ctx, dev, 0);
		if (dev_data->is_host) {
			spi_context_cs_control(&dev_data->ctx, false);
		}
	} else {
		if (status_reg & SPI_SR_RDRF_Msk) {
			rx_data = SPI_RDR_RD__EXTR(dev_config->regs->SPI_RDR);
			dev_data->txrx_balance--;
			if (spi_context_rx_on(&dev_data->ctx)) {
				if (dev_data->ctx.rx_buf != NULL) {
					if (dev_data->data_bits) {
						*((uint16_t *)dev_data->ctx.rx_buf) = rx_data;
					} else {
						*((uint8_t *)dev_data->ctx.rx_buf) = rx_data;
					}
				}
				spi_context_update_rx(&dev_data->ctx, dfs, 1);
			}
		}
		if ((dev_data->txrx_balance <= 0) && (status_reg & SPI_SR_TDRE_Msk)) {
			tx_data = 0;
			if (spi_context_tx_on(&dev_data->ctx)) {
				if (dev_data->ctx.tx_buf != NULL) {
					if (dev_data->data_bits) {
						tx_data = *((uint16_t *)(dev_data->ctx.tx_buf));
					} else {
						tx_data = *((uint8_t *)(dev_data->ctx.tx_buf));
					}
				}
				spi_context_update_tx(&dev_data->ctx, dfs, 1);
				dev_config->regs->SPI_TDR = SPI_TDR_TD(tx_data);
				dev_data->txrx_balance++;
			} else if (spi_context_rx_on(&dev_data->ctx)) {
				dev_config->regs->SPI_TDR = SPI_TDR_TD(tx_data);
				dev_data->txrx_balance++;
			}
		}
	}
}

static void spi_mchp_target_isr(const struct device *dev)
{
	const struct spi_mchp_config *dev_config = dev->config;
	struct spi_mchp_data *dev_data = dev->data;
	uint16_t rx_data, tx_data;
	uint32_t status_reg = dev_config->regs->SPI_SR;
	uint8_t dfs = dev_data->data_bits ? 2 : 1;

	if (!spi_context_rx_on(&dev_data->ctx) && !spi_context_tx_on(&dev_data->ctx)) {
		if (status_reg & (SPI_SR_TXEMPTY_Msk)) {
			/* transfer completed */
			dev_config->regs->SPI_IDR = (SPI_IDR_TDRE_Msk | SPI_IDR_TXEMPTY_Msk |
						     SPI_IDR_RDRF_Msk | SPI_IDR_NSSR_Msk);
			spi_context_complete(&dev_data->ctx, dev, 0);
			return;
		} else if (status_reg & (SPI_IDR_TDRE_Msk)) {
			dev_config->regs->SPI_IDR = SPI_IDR_TDRE_Msk;
		}
	}
	if (status_reg & SPI_SR_RDRF_Msk) {
		rx_data = SPI_RDR_RD__EXTR(dev_config->regs->SPI_RDR);
		if (spi_context_rx_on(&dev_data->ctx)) {
			if (dev_data->ctx.rx_buf != NULL) {
				if (dev_data->data_bits) {
					*((uint16_t *)dev_data->ctx.rx_buf) = rx_data;
				} else {
					*((uint8_t *)dev_data->ctx.rx_buf) = rx_data;
				}
			}
			spi_context_update_rx(&dev_data->ctx, dfs, 1);
		}
	}
	if ((dev_data->txrx_balance <= 1) && (status_reg & SPI_SR_TDRE_Msk)) {
		tx_data = 0;
		if (spi_context_tx_on(&dev_data->ctx)) {
			if (dev_data->ctx.tx_buf != NULL) {
				if (dev_data->data_bits) {
					tx_data = *((uint16_t *)(dev_data->ctx.tx_buf));
				} else {
					tx_data = *((uint8_t *)(dev_data->ctx.tx_buf));
				}
			}
			spi_context_update_tx(&dev_data->ctx, dfs, 1);
			dev_config->regs->SPI_TDR = SPI_TDR_TD(tx_data);
		}
	}
}

static void spi_mchp_isr(const struct device *dev)
{
	struct spi_mchp_data *dev_data = dev->data;

	if (dev_data->is_host) {
		spi_mchp_host_isr(dev);
	} else {
		spi_mchp_target_isr(dev);
	}
}
#else
static void spi_mchp_isr(const struct device *dev)
{
	const struct spi_mchp_config *dev_config = dev->config;

	/* This line should not be reached, disable all sources of SPI
	 * interrupts
	 */
	dev_config->regs->SPI_IDR = SPI_IDR_Msk;
}
#endif

static DEVICE_API(spi, spi_mchp_driver_api) = {
	.transceive = spi_mchp_transceive_sync,
#if defined(CONFIG_SPI_ASYNC)
	.transceive_async = spi_mchp_transceive_async,
#endif
#if defined(CONFIG_SPI_RTIO)
	.iodev_submit = spi_mchp_iodev_submit,
#endif
	.release = spi_mchp_release,
};

#define SPI_DMA_INIT(n)                                                                            \
	.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                                \
	.dma_tx_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),                               \
	.dma_tx_perid = DT_INST_DMAS_CELL_BY_NAME(n, tx, perid),                                   \
	.dma_rx_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),                               \
	.dma_rx_perid = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),

#if defined(CONFIG_SPI_MCHP_G1_DMA)
#define CA70_SPI_USE_DMA(n) DT_INST_DMAS_HAS_NAME(n, tx)
#else
#define CA70_SPI_USE_DMA(n) 0
#endif

#define SPI_MCHP_G1_DEFINE_CONFIG(n)                                                               \
	static void mchp_spi_##n##_irq_config(const struct device *dev)                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), spi_mchp_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	};                                                                                         \
	static const struct spi_mchp_config mchp_spi_##n##_config = {                              \
		.regs = (spi_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_cfg = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.loopback = DT_INST_PROP(n, loopback),                                             \
		.irq_config_func = mchp_spi_##n##_irq_config,                                      \
		COND_CODE_1(CA70_SPI_USE_DMA(n), (SPI_DMA_INIT(n)), ()) };

#define CA70_SPI_RTIO_DEFINE(n)                                                                    \
	SPI_RTIO_DEFINE(spi_mchp_rtio_##n, CONFIG_MCHP_SPI_RTIO_SQ_SIZE,                           \
			CONFIG_MCHP_SPI_RTIO_SQ_SIZE)

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	SPI_MCHP_G1_DEFINE_CONFIG(n);                                                              \
	COND_CODE_1(CONFIG_SPI_RTIO, (CA70_SPI_RTIO_DEFINE(n)), ());                                 \
	static struct spi_mchp_data mchp_spi_##n##_data = {                                        \
		SPI_CONTEXT_INIT_LOCK(mchp_spi_##n##_data, ctx),                                   \
		SPI_CONTEXT_INIT_SYNC(mchp_spi_##n##_data, ctx),                                   \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)                               \
			IF_ENABLED(CONFIG_SPI_RTIO, (.rtio_ctx = &spi_mchp_rtio_##n)) };              \
	SPI_DEVICE_DT_INST_DEFINE(n, &spi_mchp_init, NULL, &mchp_spi_##n##_data,                   \
				  &mchp_spi_##n##_config, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,   \
				  &spi_mchp_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
