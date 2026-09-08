/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_i2sc_g1_i2s

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2s_mchp_i2sc_g1, CONFIG_I2S_LOG_LEVEL);

#include <errno.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

#if !defined(ABS)
#define ABS(a, b) ((a) > (b) ? (a) - (b) : (b) - (a))
#endif

struct i2s_mchp_i2sc_config {
	i2sc_registers_t *regs;
	void (*irq_config)(const struct device *dev);
	const struct device *clock;
	clock_control_subsys_t clock_cfg;
	const struct pinctrl_dev_config *pcfg;
	uint8_t irq_id;
	bool loopback;
	uint32_t mclk_ratio;
	const struct device *dma_dev;
	const uint32_t dma_tx_channel;
	const uint32_t dma_tx_perid;
	const uint32_t dma_rx_channel;
	const uint32_t dma_rx_perid;
};

struct i2s_mchp_i2sc_block {
	void *mem_block;
	size_t size;
};

struct i2s_mchp_i2sc_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	uint32_t dma_perid;
	struct dma_config dma_config;
	bool draining;
	enum i2s_state state;
	bool i2s_configured;
	bool dma_configured;
	struct i2s_config cfg;
	struct k_msgq queue;
	size_t blocks_nr;
	struct i2s_mchp_i2sc_block next_block;
	struct i2s_mchp_i2sc_block block;
	struct i2s_mchp_i2sc_block *blocks;
};

struct i2s_mchp_i2sc_data {
	const struct device *dev;
	uint32_t cr_reg;
	struct k_spinlock lock;
	struct i2s_mchp_i2sc_stream rx_stream;
	struct i2s_mchp_i2sc_stream tx_stream;
	struct i2s_mchp_i2sc_block rx_blocks[CONFIG_I2S_MCHP_I2SC_G1_RX_BLOCK_COUNT];
	struct i2s_mchp_i2sc_block tx_blocks[CONFIG_I2S_MCHP_I2SC_G1_TX_BLOCK_COUNT];
};

static inline struct i2s_mchp_i2sc_stream *
i2s_mchp_i2sc_get_stream(struct i2s_mchp_i2sc_data *const dev_data, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_stream *stream = NULL;

	if (dir == I2S_DIR_RX) {
		stream = &dev_data->rx_stream;
	} else if (dir == I2S_DIR_TX) {
		stream = &dev_data->tx_stream;
	}

	return stream;
}

static inline int i2s_mchp_i2sc_dma_config(struct i2s_mchp_i2sc_stream *stream)
{
	int ret = dma_config(stream->dma_dev, stream->dma_channel, &stream->dma_config);

	if (ret == 0) {
		stream->dma_configured = true;
	}

	return ret;
}

static inline void i2s_mchp_i2sc_dma_stop(struct i2s_mchp_i2sc_stream *stream)
{
	if (stream->dma_configured) {
		(void)dma_stop(stream->dma_dev, stream->dma_channel);
	}
}

static inline int i2s_mchp_i2sc_dma_start(struct i2s_mchp_i2sc_stream *stream)
{
	return dma_start(stream->dma_dev, stream->dma_channel);
}

static inline uint32_t i2s_mchp_i2sc_word_size_to_bytes(uint32_t word_size)
{
	return (word_size <= 8) ? 1 : (word_size <= 16) ? 2 : 4;
}

/* Use compact mode whenever possible: since it packs two samples into the HW buffer
 * at once, it avoids the RX/TX channel flip that could otherwise occur when only
 * a single sample is picked up. It also reduces the number of DMA transfers.
 */
static inline uint32_t i2s_mchp_i2sc_word_size_to_dma_bytes(uint32_t word_size)
{
	return (word_size == 8) ? 2 : (word_size == 16) ? 4 : 4;
}

static inline uint32_t i2s_mchp_i2sc_get_freq(const struct i2s_config *i2s_cfg)
{
	return (i2s_mchp_i2sc_word_size_to_bytes(i2s_cfg->word_size) * 8 * i2s_cfg->frame_clk_freq *
		i2s_cfg->channels);
}

/* Divider is in 'field' format */
static inline uint32_t i2s_mchp_i2sc_ck_calc_div(uint32_t clkin_freq, uint32_t clkout_freq)
{
	uint32_t ck_div = (((clkin_freq / 2) / clkout_freq) - 1) * 2;

	return ck_div;
}

/* Divider is in 'field' format */
static inline uint32_t i2s_mchp_i2sc_ck_calc_freq(uint32_t clkin_freq, uint32_t divider)
{
	return (clkin_freq / 2) / ((divider / 2) + 1);
}

/* Divider is in 'field' format */
static inline uint32_t i2s_mchp_i2sc_mck_calc_freq(uint32_t clkin_freq, uint32_t divider)
{
	return clkin_freq / (divider + 1);
}

/* Divider is in 'field' format */
static inline uint32_t i2s_mchp_i2sc_mck_calc_div(uint32_t clkin_freq, uint32_t clkout_freq)
{
	uint32_t mck_div = (clkin_freq / clkout_freq) - 1;
	uint32_t mck_freq_lo = i2s_mchp_i2sc_mck_calc_freq(clkin_freq, mck_div);
	uint32_t mck_freq_hi = i2s_mchp_i2sc_mck_calc_freq(clkin_freq, mck_div + 1);

	/* Choose divider with smaller error */
	if (ABS(clkout_freq, mck_freq_hi) < ABS(clkout_freq, mck_freq_lo)) {
		mck_div = mck_div + 1;
	}

	return mck_div;
}

static void i2s_mchp_i2sc_calculate_dividers(uint32_t clkin_freq, uint32_t clkout_freq,
					     uint32_t ratio, uint32_t *ck_div, uint32_t *mck_div)
{
	uint32_t ck_div_hi, ck_div_lo;
	uint32_t mck_freq_hi, mck_freq_lo;
	uint32_t new_freq_hi, new_freq_lo;
	uint32_t mck_div_hi, mck_div_lo;
	uint32_t real_mck_freq_hi, real_mck_freq_lo;
	uint64_t rel_error_hi, rel_error_lo;

	ck_div_hi = i2s_mchp_i2sc_ck_calc_div(clkin_freq, clkout_freq);

	/* Next divider is due nature of formula +2 */
	ck_div_lo = ck_div_hi + 2;
	new_freq_hi = i2s_mchp_i2sc_ck_calc_freq(clkin_freq, ck_div_hi);
	new_freq_lo = i2s_mchp_i2sc_ck_calc_freq(clkin_freq, ck_div_lo);

	mck_freq_hi = new_freq_hi * ratio;
	mck_freq_lo = new_freq_lo * ratio;

	mck_div_hi = i2s_mchp_i2sc_mck_calc_div(clkin_freq, mck_freq_hi);
	mck_div_lo = i2s_mchp_i2sc_mck_calc_div(clkin_freq, mck_freq_lo);

	real_mck_freq_hi = i2s_mchp_i2sc_mck_calc_freq(clkin_freq, mck_div_hi);
	real_mck_freq_lo = i2s_mchp_i2sc_mck_calc_freq(clkin_freq, mck_div_lo);

	rel_error_hi = ABS(mck_freq_hi, real_mck_freq_hi);
	rel_error_hi *= mck_freq_lo;

	rel_error_lo = ABS(mck_freq_lo, real_mck_freq_lo);
	rel_error_lo *= mck_freq_hi;

	/* Prioritize CK/MCK ratio precision over sampling frequency */
	if (rel_error_hi < rel_error_lo) {
		*ck_div = ck_div_hi;
		*mck_div = mck_div_hi;
	} else {
		*ck_div = ck_div_lo;
		*mck_div = mck_div_lo;
	}
}

static void i2s_mchp_i2sc_stream_set_state(struct i2s_mchp_i2sc_stream *stream,
					   enum i2s_state state)
{
	stream->state = state;
}

static void i2s_mchp_i2sc_stream_clean_queue(struct i2s_mchp_i2sc_stream *stream)
{
	int ret = 0;
	struct i2s_mchp_i2sc_block block;

	do {
		ret = k_msgq_get(&stream->queue, &block, K_NO_WAIT);
		if ((ret == 0) && (block.mem_block != NULL)) {
			k_mem_slab_free(stream->cfg.mem_slab, block.mem_block);
		}
	} while (ret == 0);
}

static void i2s_mchp_i2sc_stream_clean_next_block(struct i2s_mchp_i2sc_stream *stream)
{
	if (stream->next_block.mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->next_block.mem_block);
		stream->next_block.mem_block = NULL;
	}
}

static void i2s_mchp_i2sc_stream_clean_blocks(struct i2s_mchp_i2sc_stream *stream)
{
	if (stream->block.mem_block != NULL) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->block.mem_block);
		stream->block.mem_block = NULL;
	}
	i2s_mchp_i2sc_stream_clean_next_block(stream);
}

static void i2s_mchp_i2sc_stop_rx(const struct device *dev)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;
	struct i2s_mchp_i2sc_stream *stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);

	i2s_mchp_i2sc_dma_stop(stream);

	regs->I2SC_CR = I2SC_CR_RXDIS_Msk;
	dev_data->cr_reg &= (~I2SC_CR_RXEN_Msk);
	if (dev_data->cr_reg == 0) {
		regs->I2SC_CR = I2SC_CR_CKDIS_Msk;
	}

	/* Empty remaining sample, otherwise next transfer will pick it */
	if (regs->I2SC_SR & I2SC_SR_RXRDY_Msk) {
		(void)regs->I2SC_RHR;
	}
}

static void i2s_mchp_i2sc_stop_tx(const struct device *dev)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;
	struct i2s_mchp_i2sc_stream *stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);

	i2s_mchp_i2sc_dma_stop(stream);

	regs->I2SC_CR = I2SC_CR_TXDIS_Msk;
	dev_data->cr_reg &= (~I2SC_CR_TXEN_Msk);
	if (dev_data->cr_reg == 0) {
		regs->I2SC_CR = I2SC_CR_CKDIS_Msk;
	}
}

static int i2s_mchp_i2sc_stream_restart_rx(const struct device *dev)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;
	struct i2s_mchp_i2sc_stream *stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
	struct dma_block_config dma_blk = {0};
	int ret;
	bool dma_started = false;

	do {
		/* Should not happen */
		if (stream->block.mem_block != NULL) {
			ret = -EFAULT;
			break;
		}

		/* Should not happen, next block should be already allocated */
		if (stream->next_block.mem_block == NULL) {
			ret = -EFAULT;
			break;
		}
		stream->block = stream->next_block;
		stream->next_block.mem_block = NULL;

		stream->dma_config.head_block = &dma_blk;
		memset(&dma_blk, 0, sizeof(dma_blk));
		dma_blk.block_size = stream->block.size;
		dma_blk.source_address = (uint32_t)(&(regs->I2SC_RHR));
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_blk.dest_address = (uint32_t)stream->block.mem_block;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		ret = i2s_mchp_i2sc_dma_config(stream);
		if (ret != 0) {
			break;
		}

		ret = i2s_mchp_i2sc_dma_start(stream);
		if (ret != 0) {
			break;
		}
		dma_started = true;

		ret = k_mem_slab_num_free_get(stream->cfg.mem_slab);
		if (ret <= 0) {
			ret = -ENOMEM;
			break;
		}

		/* Allocate next block after DMA started to minimize the audio delay */
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->next_block.mem_block,
				       K_NO_WAIT);
		if (ret != 0) {
			stream->next_block.mem_block = NULL;
			break;
		}
		stream->next_block.size = stream->cfg.block_size;
	} while (0);

	if ((ret != 0) && (stream->block.mem_block != NULL) && !dma_started) {
		k_mem_slab_free(stream->cfg.mem_slab, stream->block.mem_block);
		stream->block.mem_block = NULL;
	}

	return ret;
}

static int i2s_mchp_i2sc_stream_start_rx(const struct device *dev)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;
	struct i2s_mchp_i2sc_stream *stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
	struct dma_block_config dma_blk = {0};
	int ret;

	do {
		/* Block memory should be NULL */
		if (stream->block.mem_block != NULL || stream->next_block.mem_block != NULL) {
			ret = -EFAULT;
			break;
		}

		ret = k_mem_slab_num_free_get(stream->cfg.mem_slab);
		if (ret <= 0) {
			ret = -ENOMEM;
			break;
		}

		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->block.mem_block, K_NO_WAIT);
		if (ret != 0) {
			break;
		}

		ret = k_mem_slab_num_free_get(stream->cfg.mem_slab);
		if (ret <= 0) {
			ret = -ENOMEM;
			break;
		}
		ret = k_mem_slab_alloc(stream->cfg.mem_slab, &stream->next_block.mem_block,
				       K_NO_WAIT);
		if (ret != 0) {
			break;
		}

		stream->block.size = stream->cfg.block_size;
		stream->next_block.size = stream->cfg.block_size;

		stream->dma_config.head_block = &dma_blk;
		memset(&dma_blk, 0, sizeof(dma_blk));
		dma_blk.block_size = stream->block.size;
		dma_blk.source_address = (uint32_t)(&(regs->I2SC_RHR));
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_blk.dest_address = (uint32_t)stream->block.mem_block;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		ret = i2s_mchp_i2sc_dma_config(stream);
		if (ret != 0) {
			break;
		}
		ret = i2s_mchp_i2sc_dma_start(stream);
	} while (0);

	if (ret != 0) {
		i2s_mchp_i2sc_stream_clean_blocks(stream);
	}

	return ret;
}

static int i2s_mchp_i2sc_stream_start_tx(const struct device *dev)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;
	struct i2s_mchp_i2sc_stream *stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
	struct dma_block_config dma_blk = {0};
	int ret;

	do {
		/* Should not happen */
		if (stream->block.mem_block != NULL) {
			ret = -EFAULT;
			break;
		}
		ret = k_msgq_get(&stream->queue, &stream->block, K_NO_WAIT);
		if (ret != 0) {
			break;
		}
		(void)sys_cache_data_flush_range(stream->block.mem_block, stream->block.size);
		stream->dma_config.head_block = &dma_blk;
		memset(&dma_blk, 0, sizeof(dma_blk));
		dma_blk.block_size = stream->block.size;
		dma_blk.source_address = (uint32_t)stream->block.mem_block;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_blk.dest_address = (uint32_t)(&(regs->I2SC_THR));
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		ret = i2s_mchp_i2sc_dma_config(stream);
		if (ret != 0) {
			break;
		}
		ret = i2s_mchp_i2sc_dma_start(stream);
	} while (0);

	if (ret != 0) {
		i2s_mchp_i2sc_stream_clean_blocks(stream);
	}

	return ret;
}

static void i2s_mchp_i2sc_dma_rx_complete(const struct device *dma_dev, void *arg, uint32_t id,
					  int error_code)
{
	const struct device *dev = (const struct device *)(arg);
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
	k_spinlock_key_t key;
	int ret;

	key = k_spin_lock(&dev_data->lock);
	if (unlikely(error_code < 0)) {
		i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
	} else {
		(void)sys_cache_data_invd_range(stream->block.mem_block, stream->block.size);
		ret = k_msgq_put(&stream->queue, &stream->block, K_NO_WAIT);
		if (ret != 0) {
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
		} else {
			stream->block.mem_block = NULL;
		}
	}

	if (stream->state == I2S_STATE_RUNNING) {
		ret = i2s_mchp_i2sc_stream_restart_rx(arg);
		if (ret != 0) {
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
		}
	} else if (stream->state == I2S_STATE_STOPPING && stream->draining) {
		ret = i2s_mchp_i2sc_stream_restart_rx(arg);
		if (ret != 0) {
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
		}
	} else if (stream->state == I2S_STATE_STOPPING && !stream->draining) {
		i2s_mchp_i2sc_stop_rx(dev);
		i2s_mchp_i2sc_stream_clean_next_block(stream);
		i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
	}

	if (stream->state == I2S_STATE_ERROR) {
		i2s_mchp_i2sc_stop_rx(dev);
		i2s_mchp_i2sc_stream_clean_blocks(stream);
	}
	k_spin_unlock(&dev_data->lock, key);
}

static void i2s_mchp_i2sc_dma_tx_complete(const struct device *dma_dev, void *arg, uint32_t id,
					  int error_code)
{
	const struct device *dev = (const struct device *)(arg);
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
	k_spinlock_key_t key;
	int ret;

	key = k_spin_lock(&dev_data->lock);
	if (unlikely(error_code < 0)) {
		i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
	}

	if (likely(stream->block.mem_block != NULL)) {
		i2s_mchp_i2sc_stream_clean_blocks(stream);
	} else {
		i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
	}

	if (stream->state == I2S_STATE_RUNNING) {
		ret = i2s_mchp_i2sc_stream_start_tx(arg);
		if (ret != 0) {
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
		}
	} else if ((stream->state == I2S_STATE_STOPPING) && stream->draining) {
		ret = i2s_mchp_i2sc_stream_start_tx(arg);
		if (ret == -ENOMSG) {
			/* TX queue drained: release RX from the combined DIR_BOTH drain */
			dev_data->rx_stream.draining = false;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
			i2s_mchp_i2sc_stop_tx(dev);
		} else if (ret != 0) {
			/* TX drain aborted by an error: release RX too, otherwise it
			 * would wait forever for a completion signal that never comes
			 */
			dev_data->rx_stream.draining = false;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
		}
	} else if ((stream->state == I2S_STATE_STOPPING) && !stream->draining) {
		i2s_mchp_i2sc_stop_tx(dev);
		i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
	}

	if (stream->state == I2S_STATE_ERROR) {
		i2s_mchp_i2sc_stop_tx(dev);
		i2s_mchp_i2sc_stream_clean_blocks(stream);
		i2s_mchp_i2sc_stream_clean_queue(stream);
	}
	k_spin_unlock(&dev_data->lock, key);
}

static void i2s_mchp_i2sc_isr(const struct device *dev)
{
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;

	LOG_ERR("Unexpected I2S interrupt 0x%x : 0x%x", regs->I2SC_SR, regs->I2SC_MR);
	regs->I2SC_IDR = I2SC_IDR_Msk;
}

static bool i2s_mchp_i2sc_check_invalid_options(const struct device *dev,
						const struct i2s_config *i2s_cfg, enum i2s_dir dir,
						int *ret)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;

	if (i2s_cfg->frame_clk_freq == 0) {
		if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
			stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_NOT_READY);
		}
		if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
			stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_NOT_READY);
		}
		/* test_i2s_unconfigured_access requires 0 */
		*ret = 0;
		return true;
	}

	/* Perihperal supports only I2S */
	if ((i2s_cfg->format & I2S_FMT_DATA_FORMAT_MASK) != I2S_FMT_DATA_FORMAT_I2S) {
		*ret = -ENOTSUP;
		return true;
	}

	if (i2s_cfg->format & I2S_FMT_DATA_ORDER_LSB) {
		*ret = -EINVAL;
		return true;
	}

	if (i2s_cfg->channels != 2) {
		*ret = -EINVAL;
		return true;
	}

	if (i2s_cfg->options & I2S_OPT_PINGPONG) {
		*ret = -EINVAL;
		return true;
	}

	if (i2s_cfg->word_size != 8 && i2s_cfg->word_size != 16 && i2s_cfg->word_size != 18 &&
	    i2s_cfg->word_size != 20 && i2s_cfg->word_size != 24 && i2s_cfg->word_size != 32) {
		*ret = -EINVAL;
		return true;
	}

	return false;
}

static uint32_t i2s_mchp_i2sc_convert_word_size(const struct i2s_config *i2s_cfg)
{
	uint32_t reg_mr = 0;

	switch (i2s_cfg->word_size) {
	case 8:
		reg_mr |= I2SC_MR_DATALENGTH(I2SC_MR_DATALENGTH_8_BITS_COMPACT_Val);
		break;
	case 16:
		reg_mr |= I2SC_MR_DATALENGTH(I2SC_MR_DATALENGTH_16_BITS_COMPACT_Val);
		break;
	case 18:
		reg_mr |= I2SC_MR_DATALENGTH(I2SC_MR_DATALENGTH_18_BITS_Val);
		break;
	case 20:
		reg_mr |= I2SC_MR_DATALENGTH(I2SC_MR_DATALENGTH_20_BITS_Val);
		break;
	case 24:
		reg_mr |= I2SC_MR_DATALENGTH(I2SC_MR_DATALENGTH_24_BITS_Val);
		break;
	case 32:
		reg_mr |= I2SC_MR_DATALENGTH(I2SC_MR_DATALENGTH_32_BITS_Val);
		break;
	}

	return reg_mr;
}

static int i2s_mchp_i2sc_check_stream(struct i2s_mchp_i2sc_stream *stream)
{
	return (stream->state != I2S_STATE_NOT_READY && stream->state != I2S_STATE_READY) ? -EINVAL
											  : 0;
}

static int i2s_mchp_i2sc_check_complementary_stream(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;

	stream = i2s_mchp_i2sc_get_stream(dev_data, dir);
	if (stream == NULL) {
		return -EINVAL;
	}

	return i2s_mchp_i2sc_check_stream(stream);
}

static int i2s_mchp_i2sc_configure_stream(const struct device *dev,
					  const struct i2s_config *i2s_cfg, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;
	int ret;

	stream = i2s_mchp_i2sc_get_stream(dev_data, dir);
	if (stream == NULL) {
		return -EINVAL;
	}

	ret = i2s_mchp_i2sc_check_stream(stream);
	if (ret != 0) {
		return ret;
	}

	i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_NOT_READY);
	stream->i2s_configured = false;

	i2s_mchp_i2sc_stream_clean_blocks(stream);
	i2s_mchp_i2sc_stream_clean_queue(stream);

	/* Peripheral can be i2s_configured only when TX/RX is disabled */
	stream->cfg = *i2s_cfg;
	stream->dma_config.source_data_size =
		i2s_mchp_i2sc_word_size_to_dma_bytes(stream->cfg.word_size);
	stream->dma_config.dest_data_size =
		i2s_mchp_i2sc_word_size_to_dma_bytes(stream->cfg.word_size);

	return 0;
}

static int i2s_mchp_i2sc_configure(const struct device *dev, enum i2s_dir dir,
				   const struct i2s_config *i2s_cfg)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;
	struct i2s_mchp_i2sc_stream *stream;
	uint32_t reg_mr = 0, mck_div, ck_div, clkin_freq, sample_freq;
	k_spinlock_key_t key;
	int ret, opt_ret = 0;

	key = k_spin_lock(&dev_data->lock);

	do {
		/* Check state of the streams, transition to NOT_READY and unconfigured */
		if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
			ret = i2s_mchp_i2sc_check_complementary_stream(dev, I2S_DIR_TX);
			if (ret) {
				break;
			}
			ret = i2s_mchp_i2sc_configure_stream(dev, i2s_cfg, I2S_DIR_RX);
			if (ret != 0) {
				break;
			}
		}

		if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
			ret = i2s_mchp_i2sc_check_complementary_stream(dev, I2S_DIR_RX);
			if (ret) {
				break;
			}
			ret = i2s_mchp_i2sc_configure_stream(dev, i2s_cfg, I2S_DIR_TX);
			if (ret != 0) {
				break;
			}
		}

		/* Return on invalid options, stream is at this moment NOT_READY */
		if (i2s_mchp_i2sc_check_invalid_options(dev, i2s_cfg, dir, &opt_ret)) {
			ret = opt_ret;
			break;
		}

		/* Zephyr 'i2s_config' requires to always use 2 channels for I2S */
		reg_mr |= I2SC_MR_RXMONO(0) | I2SC_MR_TXMONO(0);

		/* Client/host selection */
		if ((i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET) == 0) {
			reg_mr |= I2SC_MR_MODE(I2SC_MR_MODE_MASTER_Val);
		} else {
			reg_mr |= I2SC_MR_MODE(I2SC_MR_MODE_SLAVE_Val);
		}

		/* Single DMA for all I2S channels of single direction */
		reg_mr |= I2SC_MR_RXDMA(0);
		reg_mr |= I2SC_MR_TXDMA(0);
		reg_mr |= I2SC_MR_TXSAME(1);
		reg_mr |= i2s_mchp_i2sc_convert_word_size(i2s_cfg);

		/* Convervative choice for 24/20/18 bit depth */
		reg_mr |= I2SC_MR_IWS(1);

		/* Enable loopback */
		if (((i2s_cfg->options & I2S_OPT_LOOPBACK) != 0) || dev_config->loopback) {
			reg_mr |= I2SC_MR_RXLOOP(1);
		}

		if ((i2s_cfg->options & I2S_OPT_BIT_CLK_TARGET) == 0) {
			sample_freq = i2s_mchp_i2sc_get_freq(i2s_cfg);

			/* Enabled MCLK, setup internal dividers */
			if (dev_config->mclk_ratio != 0) {
				ret = clock_control_get_rate(dev_config->clock,
							     dev_config->clock_cfg, &clkin_freq);
				if (ret != 0) {
					break;
				}
				i2s_mchp_i2sc_calculate_dividers(clkin_freq, sample_freq,
								 dev_config->mclk_ratio, &ck_div,
								 &mck_div);
				if (mck_div == 0) {
					/* Zero is invalid value and causes non functional clock */
					LOG_ERR("Requested ratio %d is out of divider range",
						dev_config->mclk_ratio);
					ret = -ENOTSUP;
					break;
				}
				reg_mr |= I2SC_MR_IMCKDIV(mck_div);
				reg_mr |= I2SC_MR_IMCKFS(ck_div);
				reg_mr |= I2SC_MR_IMCKMODE(1);
			} else {
				/* When MCLK is disabled the only available divider is in Clock
				 * Module
				 */
				if (dev_config->mclk_ratio == 0) {
					clock_control_set_rate(
						dev_config->clock, dev_config->clock_cfg,
						(clock_control_subsys_rate_t)sample_freq);
				}
			}
		}

		/* Reset peripheral to default (disabled) state,
		 * MR must be written only when TX/RX/CK is disabled
		 */
		regs->I2SC_CR = I2SC_CR_SWRST_Msk;
		regs->I2SC_MR = reg_mr;

		/* Transition stream to READY */
		if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
			stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
			stream->i2s_configured = true;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
		}
		if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
			stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
			stream->i2s_configured = true;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
		}
	} while (0);

	k_spin_unlock(&dev_data->lock, key);

	return ret;
}

static const struct i2s_config *i2s_mchp_i2sc_config_get(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;

	stream = i2s_mchp_i2sc_get_stream(dev_data, dir);
	return stream->i2s_configured ? &stream->cfg : NULL;
}

static int i2s_mchp_i2sc_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;
	struct i2s_mchp_i2sc_block block;
	k_timeout_t timeout_ms;
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&dev_data->lock);
	stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
	timeout_ms = K_MSEC(stream->cfg.timeout);

	if (stream->state == I2S_STATE_NOT_READY || !stream->i2s_configured) {
		ret = -EIO;
	} else if (stream->state == I2S_STATE_ERROR) {
		timeout_ms = K_NO_WAIT;
	}
	k_spin_unlock(&dev_data->lock, key);

	if (ret != 0) {
		return ret;
	}

	ret = k_msgq_get(&stream->queue, &block, timeout_ms);
	if (ret != 0) {
		return ret == -ENOMSG ? -EIO : ret;
	}

	*mem_block = block.mem_block;
	*size = block.size;

	return ret;
}

static int i2s_mchp_i2sc_write(const struct device *dev, void *mem_block, size_t size)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;
	struct i2s_mchp_i2sc_block block = {.mem_block = mem_block, .size = size};
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&dev_data->lock);
	stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
	if ((stream->state != I2S_STATE_RUNNING && stream->state != I2S_STATE_READY) ||
	    !stream->i2s_configured) {
		ret = -EIO;
	}

	if ((ret == 0) && (size > stream->cfg.block_size)) {
		ret = -EIO;
	}
	k_spin_unlock(&dev_data->lock, key);

	if (ret != 0) {
		return ret;
	}

	ret = k_msgq_put(&stream->queue, &block, K_NO_WAIT);
	if (ret == -ENOMSG) {
		return -EIO;
	}

	return ret;
}

static int i2s_mchp_i2sc_stream_start(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;
	struct i2s_mchp_i2sc_stream *stream;
	k_spinlock_key_t key;
	int rx_ret = 0, tx_ret = 0;

	key = k_spin_lock(&dev_data->lock);
	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
		if (stream->state == I2S_STATE_READY) {
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_RUNNING);
		} else {
			rx_ret = -EIO;
		}
		if ((rx_ret == 0) && i2s_mchp_i2sc_stream_start_rx(dev) != 0) {
			rx_ret = -EIO;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
		}
	}

	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
		if (stream->state == I2S_STATE_READY) {
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_RUNNING);
		} else {
			tx_ret = -EIO;
		}
		if ((tx_ret == 0) && i2s_mchp_i2sc_stream_start_tx(dev) != 0) {
			tx_ret = -EIO;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_ERROR);
		}
	}

	if (rx_ret == 0 && tx_ret == 0) {
		if (dir == I2S_DIR_RX) {
			dev_data->cr_reg |= I2SC_CR_RXEN_Msk;
			regs->I2SC_CR = I2SC_CR_RXEN_Msk | I2SC_CR_CKEN_Msk;
		} else if (dir == I2S_DIR_TX) {
			dev_data->cr_reg |= I2SC_CR_TXEN_Msk;
			regs->I2SC_CR = I2SC_CR_TXEN_Msk | I2SC_CR_CKEN_Msk;
		} else {
			dev_data->cr_reg |= I2SC_CR_RXEN_Msk | I2SC_CR_TXEN_Msk;
			regs->I2SC_CR = I2SC_CR_RXEN_Msk | I2SC_CR_TXEN_Msk | I2SC_CR_CKEN_Msk;
		}
	}
	k_spin_unlock(&dev_data->lock, key);

	return rx_ret != 0 ? rx_ret : tx_ret;
}

static int i2s_mchp_i2sc_stream_drop(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&dev_data->lock);
	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
		if (stream->state != I2S_STATE_NOT_READY) {
			i2s_mchp_i2sc_stop_rx(dev);
			i2s_mchp_i2sc_stream_clean_blocks(stream);
			i2s_mchp_i2sc_stream_clean_queue(stream);
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
		} else {
			ret = -EIO;
		}
	}
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
		if (stream->state != I2S_STATE_NOT_READY) {
			i2s_mchp_i2sc_stop_tx(dev);
			i2s_mchp_i2sc_stream_clean_blocks(stream);
			i2s_mchp_i2sc_stream_clean_queue(stream);
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
		} else {
			ret = -EIO;
		}
	}
	k_spin_unlock(&dev_data->lock, key);

	return ret;
}

static int i2s_mchp_i2sc_stream_stop(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&dev_data->lock);
	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
		if (stream->state == I2S_STATE_RUNNING) {
			stream->draining = false;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_STOPPING);
		} else {
			ret = -EIO;
		}
	}
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
		if (stream->state == I2S_STATE_RUNNING) {
			stream->draining = false;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_STOPPING);
		} else {
			ret = -EIO;
		}
	}
	k_spin_unlock(&dev_data->lock, key);

	return ret;
}

static int i2s_mchp_i2sc_stream_drain(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&dev_data->lock);
	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
		if (stream->state == I2S_STATE_RUNNING) {
			stream->draining = (IS_ENABLED(CONFIG_I2S_MCHP_I2SC_G1_COMBINED_DRAIN) &&
					    (dir == I2S_DIR_BOTH) &&
					    i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX)->state ==
						    I2S_STATE_RUNNING);
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_STOPPING);
		} else {
			ret = -EIO;
		}
	}
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
		if (stream->state == I2S_STATE_RUNNING) {
			stream->draining = true;
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_STOPPING);
		} else {
			ret = -EIO;
		}
	}
	k_spin_unlock(&dev_data->lock, key);

	return ret;
}

static int i2s_mchp_i2sc_stream_prepare(const struct device *dev, enum i2s_dir dir)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	struct i2s_mchp_i2sc_stream *stream;
	k_spinlock_key_t key;
	int ret = 0;

	key = k_spin_lock(&dev_data->lock);
	if (dir == I2S_DIR_RX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
		if (stream->state == I2S_STATE_ERROR) {
			i2s_mchp_i2sc_stream_clean_blocks(stream);
			i2s_mchp_i2sc_stream_clean_queue(stream);
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
		} else {
			ret = -EIO;
		}
	}
	if (dir == I2S_DIR_TX || dir == I2S_DIR_BOTH) {
		stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
		if (stream->state == I2S_STATE_ERROR) {
			i2s_mchp_i2sc_stream_clean_blocks(stream);
			i2s_mchp_i2sc_stream_clean_queue(stream);
			i2s_mchp_i2sc_stream_set_state(stream, I2S_STATE_READY);
		} else {
			ret = -EIO;
		}
	}
	k_spin_unlock(&dev_data->lock, key);

	return ret;
}

static int i2s_mchp_i2sc_trigger(const struct device *dev, enum i2s_dir dir,
				 enum i2s_trigger_cmd cmd)
{
	int ret = 0;

	switch (cmd) {
	case I2S_TRIGGER_START:
		ret = i2s_mchp_i2sc_stream_start(dev, dir);
		break;
	case I2S_TRIGGER_DROP:
		ret = i2s_mchp_i2sc_stream_drop(dev, dir);
		break;
	case I2S_TRIGGER_STOP:
		ret = i2s_mchp_i2sc_stream_stop(dev, dir);
		break;
	case I2S_TRIGGER_DRAIN:
		ret = i2s_mchp_i2sc_stream_drain(dev, dir);
		break;
	case I2S_TRIGGER_PREPARE:
		ret = i2s_mchp_i2sc_stream_prepare(dev, dir);
		break;
	default:
		ret = -EINVAL;
		break;
	};

	return ret;
}

static int i2s_mchp_i2sc_initialize(const struct device *dev)
{
	struct i2s_mchp_i2sc_data *const dev_data = dev->data;
	const struct i2s_mchp_i2sc_config *const dev_config = dev->config;
	i2sc_registers_t *regs = dev_config->regs;
	struct i2s_mchp_i2sc_stream *stream;
	int ret;

	if (!device_is_ready(dev_config->clock)) {
		return -ENODEV;
	}

	ret = clock_control_on(dev_config->clock, dev_config->clock_cfg);
	if (ret < 0) {
		return ret;
	}

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_RX);
	stream->blocks_nr = CONFIG_I2S_MCHP_I2SC_G1_RX_BLOCK_COUNT;
	stream->blocks = dev_data->rx_blocks;
	k_msgq_init(&stream->queue, (char *)stream->blocks, sizeof(struct i2s_mchp_i2sc_block),
		    stream->blocks_nr);

	stream->dma_channel = dev_config->dma_rx_channel;
	stream->dma_perid = dev_config->dma_rx_perid;
	stream->dma_config.channel_direction = PERIPHERAL_TO_MEMORY;
	stream->dma_config.source_burst_length = 1;
	stream->dma_config.dest_burst_length = 1;
	stream->dma_config.user_data = (void *)dev;
	stream->dma_config.dma_callback = i2s_mchp_i2sc_dma_rx_complete;
	stream->dma_config.block_count = 1;
	stream->dma_config.dma_slot = stream->dma_perid;
	stream->dma_config.complete_callback_en = true;
	stream->dma_dev = dev_config->dma_dev;
	stream->state = I2S_STATE_NOT_READY;

	stream = i2s_mchp_i2sc_get_stream(dev_data, I2S_DIR_TX);
	stream->blocks_nr = CONFIG_I2S_MCHP_I2SC_G1_TX_BLOCK_COUNT;
	stream->blocks = dev_data->tx_blocks;
	k_msgq_init(&stream->queue, (char *)stream->blocks, sizeof(struct i2s_mchp_i2sc_block),
		    stream->blocks_nr);

	stream->dma_channel = dev_config->dma_tx_channel;
	stream->dma_perid = dev_config->dma_tx_perid;
	stream->dma_config.channel_direction = MEMORY_TO_PERIPHERAL;
	stream->dma_config.source_burst_length = 1;
	stream->dma_config.dest_burst_length = 1;
	stream->dma_config.user_data = (void *)dev;
	stream->dma_config.dma_callback = i2s_mchp_i2sc_dma_tx_complete;
	stream->dma_config.block_count = 1;
	stream->dma_config.dma_slot = stream->dma_perid;
	stream->dma_config.complete_callback_en = true;
	stream->dma_dev = dev_config->dma_dev;
	stream->state = I2S_STATE_NOT_READY;

	/* Disable all interrupts */
	regs->I2SC_IDR = I2SC_IDR_Msk;
	regs->I2SC_CR = I2SC_CR_CKDIS_Msk;

	dev_config->irq_config(dev);

	return 0;
}

static DEVICE_API(i2s, i2s_mchp_i2sc_driver_api) = {
	.configure = i2s_mchp_i2sc_configure,
	.config_get = i2s_mchp_i2sc_config_get,
	.read = i2s_mchp_i2sc_read,
	.write = i2s_mchp_i2sc_write,
	.trigger = i2s_mchp_i2sc_trigger,
};

#define I2S_DMA_INIT(n)                                                                            \
	.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                                \
	.dma_tx_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),                               \
	.dma_tx_perid = DT_INST_DMAS_CELL_BY_NAME(n, tx, perid),                                   \
	.dma_rx_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),                               \
	.dma_rx_perid = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void mck_i2sc_##n##_irq_config(const struct device *dev)                            \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2s_mchp_i2sc_isr,          \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct i2s_mchp_i2sc_config mck_i2sc_##n##_config = {                         \
		.regs = (i2sc_registers_t *)DT_INST_REG_ADDR(n),                                   \
		.irq_config = mck_i2sc_##n##_irq_config,                                           \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_cfg = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),            \
		.irq_id = DT_INST_IRQN(n),                                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.loopback = DT_INST_PROP(n, loopback),                                             \
		.mclk_ratio = DT_INST_PROP_OR(n, mclk_ratio, 0),                                   \
		I2S_DMA_INIT(n)};                                                                  \
                                                                                                   \
	static struct i2s_mchp_i2sc_data mck_i2sc_##n##_data = {0};                                \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, i2s_mchp_i2sc_initialize, NULL, &mck_i2sc_##n##_data,             \
			      &mck_i2sc_##n##_config, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,       \
			      &i2s_mchp_i2sc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
