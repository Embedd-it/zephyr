/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file i2c_mchp_twihs_g1.c
 * @brief Microchip TWIHS (I2C) controller driver.
 *
 * Only I2C with 7 bit addressing is currently supported.
 */

#define DT_DRV_COMPAT microchip_twihs_g1_i2c

#include <errno.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/irq.h>

#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
#include <zephyr/drivers/dma.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mchp_twihs_g1, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

/** Number of bytes that triggers DMA transfer, cannot be less than 2 */
#define DMA_TRANSFER_THRESHOLD (2)
#if (DMA_TRANSFER_THRESHOLD < 2)
#error "DMA_TRANSFER_THRESHOLD must be set to at least to 2 bytes"
#endif

/** I2C bus speed [Hz] in Standard Mode */
#define BUS_SPEED_STANDARD_HZ 100000u
/** I2C bus speed [Hz] in Fast Mode */
#define BUS_SPEED_FAST_HZ     400000u
/** I2C bus speed [Hz] in High Speed Mode */
#define BUS_SPEED_HIGH_HZ     3400000u
/* Maximum value of Clock Divider (CKDIV) */
#define CKDIV_MAX             7

#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
struct i2c_mchp_twihs_dma {
	const struct device *dma_dev;
	uint8_t tx_dma_request;
	uint8_t tx_dma_channel;
	uint8_t rx_dma_request;
	uint8_t rx_dma_channel;
};
#endif /* CONFIG_I2C_MCHP_TWIHS_G1_DMA */

/* Device constant configuration parameters */
struct i2c_mchp_twihs_config {
	twihs_registers_t *regs;
	void (*irq_config)(const struct device *dev);
	uint32_t bitrate;
	const struct device *clock;
	clock_control_subsys_t clock_cfg;
	const struct pinctrl_dev_config *pcfg;
	uint8_t irq_id;
#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
	struct i2c_mchp_twihs_dma i2c_dma;
#endif
};

struct i2c_mchp_twihs_msg {
	/* Buffer containing data to read or write */
	uint8_t *buf;
	/* Length of the buffer */
	uint32_t len;
	/* Index of the next byte to be read/written from/to the buffer */
	uint32_t idx;
	/* Value of TWIHS_SR at the end of the message */
	uint32_t twihs_sr;
#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
	/* DMA callback error */
	uint32_t dma_err;
#endif
	/* Transfer flags as defined in the i2c.h file */
	uint8_t flags;
};

/* Device run time data */
struct i2c_mchp_twihs_data {
	const struct device *dev;
	struct k_sem lock;
	struct k_sem transfer_done;
	struct i2c_mchp_twihs_msg msg;
	uint32_t config;
	bool is_configured;
#if defined(CONFIG_I2C_TARGET)
	bool in_target_mode;
	bool in_transfer;
	struct i2c_target_config *target_config;
	uint8_t target_byte;
#endif
};

#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
static bool i2c_mchp_twihs_passes_dma_threshold(struct i2c_mchp_twihs_msg *msg)
{
	return msg->len > (DMA_TRANSFER_THRESHOLD);
}

static void i2c_mchp_twihs_dma_write_done(const struct device *dma_dev, void *arg, uint32_t id,
					  int error_code)
{
	struct i2c_mchp_twihs_data *dev_data = (struct i2c_mchp_twihs_data *)arg;
	const struct device *dev = dev_data->dev;
	const struct i2c_mchp_twihs_config *dev_config = dev->config;
	twihs_registers_t *const regs = dev_config->regs;

	if (unlikely(error_code < 0)) {
		dev_data->msg.dma_err = error_code;
		regs->TWIHS_IDR = TWIHS_IDR_Msk;
		(void)dma_stop(dev_config->i2c_dma.dma_dev, dev_config->i2c_dma.tx_dma_channel);
		k_sem_give(&dev_data->transfer_done);
		return;
	}

	/* Update message, last byte must be send without DMA */
	/* NOTE: Running in interrupt context, this function is non-preemptible.
	 * Therefore, the subsequent data manipulation does not need a critical
	 * section
	 */
	dev_data->msg.idx = dev_data->msg.len - 1;

	/* Triggers the interrupts */
	regs->TWIHS_IER = TWIHS_IER_TXRDY_Msk | TWIHS_IER_TXCOMP_Msk | TWIHS_IER_NACK_Msk;
}

static void i2c_mchp_twihs_dma_read_done(const struct device *dma_dev, void *arg, uint32_t id,
					 int error_code)
{
	struct i2c_mchp_twihs_data *dev_data = (struct i2c_mchp_twihs_data *)arg;
	const struct device *dev = dev_data->dev;
	const struct i2c_mchp_twihs_config *dev_config = dev->config;
	twihs_registers_t *const regs = dev_config->regs;

	if (unlikely(error_code < 0)) {
		dev_data->msg.dma_err = error_code;
		regs->TWIHS_IDR = TWIHS_IDR_Msk;
		(void)dma_stop(dev_config->i2c_dma.dma_dev, dev_config->i2c_dma.rx_dma_channel);
		k_sem_give(&dev_data->transfer_done);
		return;
	}

	(void)sys_cache_data_invd_range(dev_data->msg.buf, dev_data->msg.len - 2);

	/* Update message, last two bytes must be send without DMA */
	/* NOTE: Running in interrupt context, this function is non-preemptible.
	 * Therefore, the subsequent data manipulation does not need a critical
	 * section.
	 */
	dev_data->msg.idx = dev_data->msg.len - 2;

	/* Triggers the interrupts */
	regs->TWIHS_IER = TWIHS_IER_RXRDY_Msk | TWIHS_IER_TXCOMP_Msk | TWIHS_IER_NACK_Msk;
}

static int i2c_mchp_twihs_write_dma_fire(const struct device *dev, uint16_t addr)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	twihs_registers_t *const regs = dev_config->regs;

	/* Set target address */
	regs->TWIHS_MMR = TWIHS_MMR_DADR(addr);
	regs->TWIHS_IER = TWIHS_IER_NACK_Msk;
	return dma_start(dev_config->i2c_dma.dma_dev, dev_config->i2c_dma.tx_dma_channel);
}

static int i2c_mchp_twihs_read_dma_fire(const struct device *dev, uint16_t addr)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	twihs_registers_t *const regs = dev_config->regs;
	int ret;

	regs->TWIHS_MMR = TWIHS_MMR_MREAD_Msk | TWIHS_MMR_DADR(addr);
	/* Set target address */
	ret = dma_start(dev_config->i2c_dma.dma_dev, dev_config->i2c_dma.rx_dma_channel);
	if (!ret) {
		regs->TWIHS_IER = TWIHS_IER_NACK_Msk;
		regs->TWIHS_CR = TWIHS_CR_START_Msk;
	}

	return ret;
}

static int i2c_mchp_twihs_write_dma_config(const struct device *dev)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = 1;
	dma_cfg.source_burst_length = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.dest_burst_length = 1;
	dma_cfg.user_data = dev_data;
	dma_cfg.dma_callback = i2c_mchp_twihs_dma_write_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = dev_config->i2c_dma.tx_dma_request;

	(void)sys_cache_data_flush_range(dev_data->msg.buf, dev_data->msg.len);
	/* NOTE: Last byte must be sent in interrupt mode */
	dma_blk.block_size = dev_data->msg.len - 1;
	dma_blk.source_address = (uint32_t)dev_data->msg.buf;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	dma_blk.dest_address = (uint32_t)(&(regs->TWIHS_THR));
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	return dma_config(dev_config->i2c_dma.dma_dev, dev_config->i2c_dma.tx_dma_channel,
			  &dma_cfg);
}

static int i2c_mchp_twihs_read_dma_config(const struct device *dev)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;

	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.source_burst_length = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.dest_burst_length = 1;
	dma_cfg.user_data = dev_data;
	dma_cfg.dma_callback = i2c_mchp_twihs_dma_read_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = dev_config->i2c_dma.rx_dma_request;

	/* NOTE: Last two bytes must be sent in interrupt mode */
	dma_blk.block_size = dev_data->msg.len - 2;
	dma_blk.dest_address = (uint32_t)dev_data->msg.buf;
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	dma_blk.source_address = (uint32_t)(&(regs->TWIHS_RHR));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	return dma_config(dev_config->i2c_dma.dma_dev, dev_config->i2c_dma.rx_dma_channel,
			  &dma_cfg);
}
#endif

static int i2c_mchp_twihs_clk_set(const struct i2c_mchp_twihs_config *const dev_config,
				  uint32_t speed)
{
	twihs_registers_t *const regs = dev_config->regs;
	uint32_t ck_div = 0u;
	uint32_t cl_div;
	bool div_completed = false;
	uint32_t clkin_freq;
	int ret;

	/*  From the datasheet "TWIHS Clock Waveform Generator Register"
	 *  T_low = ( ( CLDIV × 2^CKDIV ) + 3 ) × T_MCK
	 */
	ret = clock_control_get_rate(dev_config->clock, dev_config->clock_cfg, &clkin_freq);
	if (ret != 0) {
		return ret;
	}

	while (!div_completed) {
		cl_div = ((clkin_freq / (speed * 2u)) - 3) / (1 << ck_div);

		if (cl_div <= 255u) {
			div_completed = true;
		} else {
			ck_div++;
		}
	}

	if (ck_div > CKDIV_MAX) {
		LOG_ERR("Failed to configure I2C clock");
		return -EIO;
	}

	/* Set I2C bus clock duty cycle to 50% */
	regs->TWIHS_CWGR =
		TWIHS_CWGR_CLDIV(cl_div) | TWIHS_CWGR_CHDIV(cl_div) | TWIHS_CWGR_CKDIV(ck_div);

	return 0;
}

static int i2c_mchp_twihs_get_config(const struct device *dev, uint32_t *config)
{
	struct i2c_mchp_twihs_data *const dev_data = dev->data;

	if (!dev_data->is_configured) {
		LOG_ERR("I2C is not configured");
		return -EIO;
	}
	*config = dev_data->config;

	return 0;
}

static int i2c_mchp_twihs_configure(const struct device *dev, uint32_t config)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;
	uint32_t bitrate;
	int ret;

	if (config & I2C_ADDR_10_BITS) {
		LOG_ERR("I2C 10-bit addressing is currently not supported");
		LOG_ERR("Please submit a patch");
		return -EIO;
	}

	/* Configure clock */
	switch (I2C_SPEED_GET(config)) {
	case I2C_SPEED_STANDARD:
		bitrate = BUS_SPEED_STANDARD_HZ;
		break;
	case I2C_SPEED_FAST:
		bitrate = BUS_SPEED_FAST_HZ;
		break;
	default:
		LOG_ERR("Unsupported I2C speed value");
		return -EIO;
	}

	k_sem_take(&dev_data->lock, K_FOREVER);

	/* Setup clock waveform */
	ret = i2c_mchp_twihs_clk_set(dev_config, bitrate);
	if (ret < 0) {
		goto unlock;
	}

	if ((config & I2C_MODE_CONTROLLER) == I2C_MODE_CONTROLLER) {
		/* Disable Client Mode */
		regs->TWIHS_CR = TWIHS_CR_SVDIS_Msk;
		/* Enable Host Mode */
		regs->TWIHS_CR = TWIHS_CR_MSEN_Msk;
	}

	dev_data->config = config;
	dev_data->is_configured = true;

	ret = 0;
unlock:
	k_sem_give(&dev_data->lock);

	return ret;
}

static void i2c_mchp_twihs_write_msg_start(twihs_registers_t *const regs,
					   struct i2c_mchp_twihs_msg *msg, uint8_t daddr)
{
	/* Set target address. */
	regs->TWIHS_MMR = TWIHS_MMR_DADR(daddr);

	if (msg->len != 0) {
		/* Write first data byte on I2C bus */
		regs->TWIHS_THR = msg->buf[msg->idx++];

		/* Enable Transmit Ready and Transmission Completed interrupts */
		regs->TWIHS_IER = TWIHS_IER_TXRDY_Msk | TWIHS_IER_TXCOMP_Msk | TWIHS_IER_NACK_Msk;
	} else {
		/* I2C scan */
		regs->TWIHS_CR = TWIHS_CR_QUICK_Msk;
		regs->TWIHS_IER = TWIHS_IER_TXCOMP_Msk | TWIHS_IER_NACK_Msk;
	}
}

static void i2c_mchp_twihs_read_msg_start(twihs_registers_t *const regs,
					  struct i2c_mchp_twihs_msg *msg, uint8_t daddr)
{
	uint32_t twihs_cr_stop;

	/* Set target address and number of internal address bytes */
	regs->TWIHS_MMR = TWIHS_MMR_MREAD_Msk | TWIHS_MMR_DADR(daddr);

	/* In single data byte read the START and STOP must both be set */
	twihs_cr_stop = (msg->len == 1u) ? TWIHS_CR_STOP_Msk : 0;

	/* Start the transfer by sending START condition */
	regs->TWIHS_CR = TWIHS_CR_START_Msk | twihs_cr_stop;

	/* Enable Receive Ready and Transmission Completed interrupts */
	regs->TWIHS_IER = TWIHS_IER_RXRDY_Msk | TWIHS_IER_TXCOMP_Msk | TWIHS_IER_NACK_Msk;
}

static int i2c_mchp_twihs_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				   uint16_t addr)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;
	int ret;

	__ASSERT_NO_MSG(msgs);
	if (!num_msgs) {
		return 0;
	}
	k_sem_take(&dev_data->lock, K_FOREVER);

	/* Clear pending interrupts, such as NACK. */
	(void)regs->TWIHS_SR;

	/* Set number of internal address bytes to 0, not used. */
	regs->TWIHS_IADR = 0;

	for (size_t i = 0; i < num_msgs; i++) {
		if ((msgs[i].buf == NULL) ||
		    ((msgs[i].len == 0) && (msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ)) {
			ret = -ENOTSUP;
			goto unlock;
		}
		dev_data->msg.buf = msgs[i].buf;
		dev_data->msg.len = msgs[i].len;
		dev_data->msg.idx = 0u;
		dev_data->msg.twihs_sr = 0u;
		dev_data->msg.flags = msgs[i].flags;

#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
		dev_data->msg.dma_err = 0u;
		/* Minimum size required to use DMA */
		if (i2c_mchp_twihs_passes_dma_threshold(&dev_data->msg)) {
			if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
				ret = i2c_mchp_twihs_read_dma_config(dev);
				if (ret != 0) {
					goto unlock;
				}
				ret = i2c_mchp_twihs_read_dma_fire(dev, addr);
				if (ret != 0) {
					goto unlock;
				}
			} else {
				ret = i2c_mchp_twihs_write_dma_config(dev);
				if (ret != 0) {
					goto unlock;
				}
				ret = i2c_mchp_twihs_write_dma_fire(dev, addr);
				if (ret != 0) {
					goto unlock;
				}
			}

			/* Wait for the transfer to complete */
			k_sem_take(&dev_data->transfer_done, K_FOREVER);

			if (dev_data->msg.twihs_sr != 0) {
				/* Something went wrong */
				ret = -EIO;
				goto unlock;
			}
			if (dev_data->msg.dma_err != 0) {
				ret = -EIO;
				goto unlock;
			}
		} else {
			/* Fallback to interrupt mode */
#endif
			if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
				i2c_mchp_twihs_read_msg_start(regs, &dev_data->msg, addr);
			} else {
				i2c_mchp_twihs_write_msg_start(regs, &dev_data->msg, addr);
			}

			/* Wait for the transfer to complete */
			k_sem_take(&dev_data->transfer_done, K_FOREVER);

			if (dev_data->msg.twihs_sr != 0) {
				/* Something went wrong */
				ret = -EIO;
				goto unlock;
			}
#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
		}
#endif
	}

	ret = 0;
unlock:
	regs->TWIHS_IDR = TWIHS_IDR_Msk;
	k_sem_give(&dev_data->lock);

	return ret;
}

#if defined(CONFIG_I2C_TARGET)
static void i2c_mchp_twihs_target_isr(const struct device *dev)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;
	uint32_t isr_status, status_reg;
	bool processed;

	do {
		/* Retrieve interrupt status */
		status_reg = regs->TWIHS_SR;
		isr_status = status_reg & regs->TWIHS_IMR;
		processed = false;

		/* ADDR MATCH - clear after STOP or NAK */
		if (isr_status & TWIHS_SR_SVACC_Msk) {
			if (status_reg & TWIHS_SR_SVREAD_Msk) {
				regs->TWIHS_IER = TWIHS_IER_TXRDY_Msk;
				if (dev_data->target_config->callbacks->read_requested != NULL) {
					dev_data->target_config->callbacks->read_requested(
						dev_data->target_config, &dev_data->target_byte);
				}
			} else {
				regs->TWIHS_IER = TWIHS_IER_RXRDY_Msk;
				if (dev_data->target_config->callbacks->write_requested != NULL) {
					dev_data->target_config->callbacks->write_requested(
						dev_data->target_config);
				}
			}
			dev_data->in_transfer = true;
			regs->TWIHS_IDR = TWIHS_IDR_SVACC_Msk;
			regs->TWIHS_IER = TWIHS_SR_NACK_Msk | TWIHS_SR_EOSACC_Msk;
			processed = true;
		}

		/* NACK - clear on read */
		if (status_reg & TWIHS_SR_NACK_Msk) {
			dev_data->in_transfer = false;
			regs->TWIHS_IDR =
				TWIHS_IER_RXRDY_Msk | TWIHS_IER_TXRDY_Msk | TWIHS_SR_NACK_Msk;
			processed = true;
		}

		/* STOP - clear on read */
		if (status_reg & TWIHS_SR_EOSACC_Msk) {
			dev_data->in_transfer = false;
			if (dev_data->target_config->callbacks->stop != NULL) {
				dev_data->target_config->callbacks->stop(dev_data->target_config);
			}
			regs->TWIHS_IDR = TWIHS_IER_RXRDY_Msk | TWIHS_IER_TXRDY_Msk;
			regs->TWIHS_IER = TWIHS_IER_SVACC_Msk;
			processed = true;
		}

		if (dev_data->in_transfer) {
			if (status_reg & TWIHS_SR_SVREAD_Msk) { /* TARGET -> HOST*/
				if (status_reg & TWIHS_SR_TXRDY_Msk) {
					regs->TWIHS_THR = dev_data->target_byte;
					if (dev_data->target_config->callbacks->read_processed !=
					    NULL) {
						dev_data->target_config->callbacks->read_processed(
							dev_data->target_config,
							&dev_data->target_byte);
					}
					processed = true;
				}
			} else { /* HOST -> TARGET */
				if (status_reg & TWIHS_SR_RXRDY_Msk) {
					dev_data->target_byte = regs->TWIHS_RHR;
					if (dev_data->target_config->callbacks->write_received !=
					    NULL) {
						dev_data->target_config->callbacks->write_received(
							dev_data->target_config,
							dev_data->target_byte);
					}
					processed = true;
				}
			}
		}
	} while (processed);
}
#endif

static void i2c_mchp_twihs_isr(const struct device *dev)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;
	struct i2c_mchp_twihs_msg *msg = &dev_data->msg;
	uint32_t isr_status, status_reg;

#if defined(CONFIG_I2C_TARGET)
	if (dev_data->in_target_mode) {
		i2c_mchp_twihs_target_isr(dev);
		return;
	}
#endif

	/* Retrieve interrupt status */
	status_reg = regs->TWIHS_SR;
	isr_status = status_reg & regs->TWIHS_IMR;

	/* Not Acknowledged */
	if (isr_status & TWIHS_SR_NACK_Msk) {
		msg->twihs_sr = isr_status;
		goto tx_comp;
	}

	/* Byte received */
	if (isr_status & TWIHS_SR_RXRDY_Msk) {
		if (likely(msg->idx < msg->len)) {
			/* The STOP flag must be set before reading the
			 * penultimate byte. Since the penultimate byte is
			 * already in the RHR, the STOP flag will affect the
			 * last byte - preventing logic disruption during driver
			 * debugging.
			 */
			if (msg->idx + 2 == msg->len) {
				/* Send STOP condition */
				regs->TWIHS_CR = TWIHS_CR_STOP_Msk;
			}
			msg->buf[msg->idx++] = regs->TWIHS_RHR;
		} else {
			/* Should not happen under normal circumstances;
			 * disabling to prevent interrupt flood.
			 */
			regs->TWIHS_IDR = TWIHS_IDR_RXRDY_Msk;
		}
	}

	/* Byte sent */
	if (isr_status & TWIHS_SR_TXRDY_Msk) {
		if (likely(msg->idx < msg->len)) {
			regs->TWIHS_THR = msg->buf[msg->idx++];
		} else if (msg->idx == msg->len) {
			if (msg->flags & I2C_MSG_STOP) {
				/* Send STOP condition */
				regs->TWIHS_CR = TWIHS_CR_STOP_Msk;
				/* Disable Transmit Ready interrupt */
				regs->TWIHS_IDR = TWIHS_IDR_TXRDY_Msk;
			} else {
				/* Transmission completed */
				goto tx_comp;
			}
		}
	}

	/* Transmission completed */
	if (isr_status & TWIHS_SR_TXCOMP_Msk) {
		goto tx_comp;
	}

	return;

tx_comp:
	/* Disable all enabled interrupts */
	regs->TWIHS_IDR = regs->TWIHS_IMR;

#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
	if (i2c_mchp_twihs_passes_dma_threshold(msg)) {
		(void)dma_stop(dev_config->i2c_dma.dma_dev, dev_config->i2c_dma.tx_dma_channel);
		(void)dma_stop(dev_config->i2c_dma.dma_dev, dev_config->i2c_dma.rx_dma_channel);
	}
#endif

	/* We are done */
	k_sem_give(&dev_data->transfer_done);
}

static int i2c_mchp_twihs_initialize(const struct device *dev)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;
	uint32_t bitrate_cfg;
	int ret;

	/* Configure interrupts */
	dev_config->irq_config(dev);

	/* Initialize semaphore */
	k_sem_init(&dev_data->lock, 1, 1);
	k_sem_init(&dev_data->transfer_done, 0, 1);

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Enable TWIHS clock in PMC */
	ret = clock_control_on(dev_config->clock, dev_config->clock_cfg);
	if (ret < 0) {
		return -ENODEV;
	}

	/* Reset the module */
	regs->TWIHS_CR = TWIHS_CR_SWRST_Msk;

	bitrate_cfg = i2c_map_dt_bitrate(dev_config->bitrate);

	ret = i2c_mchp_twihs_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to initialize %s device", dev->name);
		return ret;
	}

#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
	dev_data->dev = dev;
	if (!device_is_ready(dev_config->i2c_dma.dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}

	int dma_ch = dev_config->i2c_dma.tx_dma_channel;

	if (dma_request_channel(dev_config->i2c_dma.dma_dev, &dma_ch) !=
	    dev_config->i2c_dma.tx_dma_channel) {
		LOG_ERR("TX DMA channel %d unavailable", dev_config->i2c_dma.tx_dma_channel);
		return -EBUSY;
	}

	dma_ch = dev_config->i2c_dma.rx_dma_channel;
	if (dma_request_channel(dev_config->i2c_dma.dma_dev, &dma_ch) !=
	    dev_config->i2c_dma.rx_dma_channel) {
		LOG_ERR("TX DMA channel %d unavailable", dev_config->i2c_dma.rx_dma_channel);
		return -EBUSY;
	}
#endif

	/* Enable module's IRQ */
	irq_enable(dev_config->irq_id);

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

#if defined(CONFIG_I2C_TARGET)
static int i2c_mchp_twihs_target_register(const struct device *dev,
					  struct i2c_target_config *target_config)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;

	if (dev_data->in_target_mode) {
		LOG_ERR("Device is already in target mode");
		return -EBUSY;
	}

	if ((target_config == NULL) || (target_config->callbacks == NULL)) {
		LOG_ERR("Incorrect device configuration parameters");
		return -EINVAL;
	}

	k_sem_take(&dev_data->lock, K_FOREVER);
	/* Reset peripheral */
	regs->TWIHS_CR = TWIHS_CR_SWRST_Msk;
	/* Disable all interrupts */
	regs->TWIHS_IDR = TWIHS_IDR_Msk;
	dev_data->in_target_mode = true;
	dev_data->in_transfer = false;
	dev_data->target_config = target_config;
	/* Disable hHost, disable client */
	regs->TWIHS_CR = TWIHS_CR_MSDIS_Msk | TWIHS_CR_SVDIS_Msk;
	/* Clear pending flags */
	(void)regs->TWIHS_SR;
	/* Set the address the device is listen to, clock stretching is enabled
	 */
	regs->TWIHS_SMR = TWIHS_SMR_SADR(dev_data->target_config->address);
	/* Enable Client/Target Mode */
	regs->TWIHS_CR = TWIHS_CR_SVEN_Msk;
	/* Enable match addr interrupt */
	regs->TWIHS_IER = TWIHS_IER_SVACC_Msk;
	k_sem_give(&dev_data->lock);

	return 0;
}

static int i2c_mchp_twihs_target_unregister(const struct device *dev,
					    struct i2c_target_config *target_config)
{
	const struct i2c_mchp_twihs_config *const dev_config = dev->config;
	struct i2c_mchp_twihs_data *const dev_data = dev->data;
	twihs_registers_t *const regs = dev_config->regs;

	if (!dev_data->in_target_mode) {
		LOG_ERR("Device is not in target mode");
		return -EBUSY;
	}

	if (target_config == NULL) {
		LOG_ERR("Incorrect device configuration parameters");
		return -EINVAL;
	}

	if (!WAIT_FOR(regs->TWIHS_SR & (TWIHS_SR_TXCOMP_Msk | TWIHS_SR_NACK_Msk), 10000,
		      k_busy_wait(2000))) {
		return -EBUSY;
	}

	k_sem_take(&dev_data->lock, K_FOREVER);
	dev_data->in_target_mode = false;
	dev_data->in_transfer = false;

	/* Disable all interrupts */
	regs->TWIHS_IDR = TWIHS_IDR_Msk;
	/* Disable Client/Target Mode */
	regs->TWIHS_CR = TWIHS_CR_MSDIS_Msk | TWIHS_CR_SVDIS_Msk;
	/* Enable Host Mode */
	regs->TWIHS_CR = TWIHS_CR_MSEN_Msk;
	k_sem_give(&dev_data->lock);

	return 0;
}
#endif

static DEVICE_API(i2c, i2c_mchp_twihs_driver_api) = {
	.get_config = i2c_mchp_twihs_get_config,
	.configure = i2c_mchp_twihs_configure,
	.transfer = i2c_mchp_twihs_transfer,
#if defined(CONFIG_I2C_TARGET)
	.target_register = i2c_mchp_twihs_target_register,
	.target_unregister = i2c_mchp_twihs_target_unregister,
#endif
};

#if defined(CONFIG_I2C_MCHP_TWIHS_G1_DMA)
#define I2C_MCHP_TWIHS_G1_DMA_CHANNELS(n)                                                          \
	.i2c_dma.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                        \
	.i2c_dma.tx_dma_request = DT_INST_DMAS_CELL_BY_NAME(n, tx, perid),                         \
	.i2c_dma.tx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),                       \
	.i2c_dma.rx_dma_request = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),                         \
	.i2c_dma.rx_dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel)
#else
#define I2C_MCHP_TWIHS_G1_DMA_CHANNELS(n)
#endif

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void mchp_twihs_##n##_irq_config(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2c_mchp_twihs_isr,         \
			    DEVICE_DT_INST_GET(n), 0);                                             \
	}                                                                                          \
                                                                                                   \
	static const struct i2c_mchp_twihs_config mchp_twihs_##n##_config = {                      \
		.regs = (twihs_registers_t *)DT_INST_REG_ADDR(n),                                  \
		.irq_config = mchp_twihs_##n##_irq_config,                                         \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_cfg = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),            \
		.irq_id = DT_INST_IRQN(n),                                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.bitrate = DT_INST_PROP(n, clock_frequency),                                       \
		I2C_MCHP_TWIHS_G1_DMA_CHANNELS(n)};                                                \
                                                                                                   \
	static struct i2c_mchp_twihs_data mchp_twihs_##n##_data;                                   \
                                                                                                   \
	I2C_DEVICE_DT_INST_DEFINE(n, i2c_mchp_twihs_initialize, NULL, &mchp_twihs_##n##_data,      \
				  &mchp_twihs_##n##_config, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, \
				  &i2c_mchp_twihs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
