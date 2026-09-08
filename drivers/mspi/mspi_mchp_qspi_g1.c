/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip MCU family MSPI driver for QUADSPI peripheral.
 */

#define DT_DRV_COMPAT microchip_qspi_g1_mspi

#include <stdint.h>

#include <errno.h>
#include <zephyr/spinlock.h>
#include <zephyr/device.h>
#include <zephyr/cache.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mspi_mchp_qspi_g1, CONFIG_MSPI_LOG_LEVEL);

#if !defined(ABS)
#define ABS(a, b) ((a) > (b) ? (a) - (b) : (b) - (a))
#endif

#define MSPI_MCHP_QSPI_G1_WAIT_FOR_COMPLETION_MS                                                   \
	K_MSEC(CONFIG_MSPI_MCHP_QSPI_G1_WAIT_FOR_COMPLETION_MS)

struct mspi_mchp_qspi_config {
	qspi_registers_t *regs;
	void *mmap;
	size_t mmap_size;
	const struct device *clock;
	clock_control_subsys_t clock_cfg;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config_func)(const struct device *dev);
	struct mspi_cfg mspi_cfg;
#if defined(CONFIG_MSPI_DMA)
	const struct device *dma_dev;
	const uint32_t dma_channel;
#endif
};

struct mspi_mchp_qspi_data {
	struct k_mutex lock;
	const struct mspi_dev_id *dev_id;
	struct mspi_dev_cfg dev_cfg;
	bool in_transfer;
	struct k_sem transfer_done;
#if defined(CONFIG_MSPI_DMA)
	int dma_errno;
	void *invalidate_mem;
	size_t invalidate_size;
#endif
};

static int mspi_mchp_qspi_config(const struct mspi_dt_spec *spec)
{
	const struct device *dev = spec->bus;
	const struct mspi_mchp_qspi_config *dev_config = dev->config;
	qspi_registers_t *regs = dev_config->regs;
	int ret;

	if (spec->config.num_periph > 1) {
		LOG_ERR("Invalid MSPI peripheral number.");
		return -ENOTSUP;
	}

	if (spec->config.duplex != MSPI_HALF_DUPLEX) {
		LOG_ERR("Only HALF_DUPLEX is supported");
		return -ENOTSUP;
	}

	if (spec->config.op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("Only controller mode is supported");
		return -ENOTSUP;
	}

	ret = clock_control_on(dev_config->clock, dev_config->clock_cfg);
	if (ret != 0) {
		LOG_ERR("Clock enable failed");
		return ret;
	}

	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("MSPI pinctrl setup failed");
		return ret;
	}

	dev_config->irq_config_func(dev);

	/* 8 bits and memory mapped SPI */
	regs->QSPI_MR = QSPI_MR_NBBITS(8) | QSPI_MR_SMM(1);

	return 0;
}

static int mspi_mchp_qspi_check_iomode(enum mspi_io_mode iomode)
{
	return (iomode == MSPI_IO_MODE_SINGLE || iomode == MSPI_IO_MODE_DUAL_1_1_2 ||
		iomode == MSPI_IO_MODE_QUAD_1_1_4 || iomode == MSPI_IO_MODE_DUAL_1_2_2 ||
		iomode == MSPI_IO_MODE_QUAD_1_4_4 || iomode == MSPI_IO_MODE_DUAL ||
		iomode == MSPI_IO_MODE_QUAD)
		       ? 0
		       : -ENOTSUP;
}

static uint32_t mspi_mchp_qspi_convert_iomode(enum mspi_io_mode iomode)
{
	uint32_t reg = 0;

	/* Convert IO mode */
	switch (iomode) {
	case MSPI_IO_MODE_SINGLE:
		reg = QSPI_IFR_WIDTH(QSPI_IFR_WIDTH_SINGLE_BIT_SPI_Val);
		break;
	case MSPI_IO_MODE_DUAL_1_1_2:
		reg = QSPI_IFR_WIDTH(QSPI_IFR_WIDTH_DUAL_OUTPUT_Val);
		break;
	case MSPI_IO_MODE_QUAD_1_1_4:
		reg = QSPI_IFR_WIDTH(QSPI_IFR_WIDTH_QUAD_OUTPUT_Val);
		break;
	case MSPI_IO_MODE_DUAL_1_2_2:
		reg = QSPI_IFR_WIDTH(QSPI_IFR_WIDTH_DUAL_IO_Val);
		break;
	case MSPI_IO_MODE_QUAD_1_4_4:
		reg = QSPI_IFR_WIDTH(QSPI_IFR_WIDTH_QUAD_IO_Val);
		break;
	case MSPI_IO_MODE_DUAL:
		reg = QSPI_IFR_WIDTH(QSPI_IFR_WIDTH_DUAL_CMD_Val);
		break;
	case MSPI_IO_MODE_QUAD:
		reg = QSPI_IFR_WIDTH(QSPI_IFR_WIDTH_QUAD_CMD_Val);
		break;
	default:
		/* Valid value is verified in 'mspi_mchp_qspi_check_iomode' */
		break;
	}

	return reg;
}

static int mspi_mchp_qspi_check_cpp_mode(enum mspi_cpp_mode cpp_mode)
{
	return (cpp_mode == MSPI_CPP_MODE_0 || cpp_mode == MSPI_CPP_MODE_1 ||
		cpp_mode == MSPI_CPP_MODE_2 || cpp_mode == MSPI_CPP_MODE_3)
		       ? 0
		       : -ENOTSUP;
}

static uint32_t mspi_mchp_qspi_convert_cpp_mode(enum mspi_cpp_mode cpp_mode)
{
	uint32_t reg = 0;

	/* Convert IO mode */
	switch (cpp_mode) {
	case MSPI_CPP_MODE_0:
		reg = 0;
		break;
	case MSPI_CPP_MODE_1:
		reg = QSPI_SCR_CPHA_Msk;
		break;
	case MSPI_CPP_MODE_2:
		reg = QSPI_SCR_CPOL_Msk;
		break;
	case MSPI_CPP_MODE_3:
		reg = QSPI_SCR_CPHA_Msk | QSPI_SCR_CPOL_Msk;
		break;
	}

	return reg;
}

static int mspi_mchp_qspi_check_dummy_clock(uint32_t clock)
{
	return clock > (QSPI_IFR_NBDUM_Msk >> QSPI_IFR_NBDUM_Pos) ? -ENOTSUP : 0;
}

static int mspi_mchp_qspi_check_update_ce(struct mspi_dev_cfg *dst_cfg,
					  const struct mspi_dev_cfg *dev_cfg)
{
	if (dev_cfg->ce_num != 0) {
		LOG_ERR("ce_num is not supported");
		return -ENOTSUP;
	}
	dst_cfg->ce_num = dev_cfg->ce_num;

	return 0;
}

static int mspi_mchp_qspi_calc_divider(uint32_t clkin_freq, uint32_t clkout_freq, uint32_t *divider,
				       uint32_t *real_freq)
{
	uint32_t scbr_lo, scbr_hi, scbr_used;
	uint32_t real_freq_lo, real_freq_hi, real_freq_used;

	if ((clkout_freq == 0) || (clkout_freq > clkin_freq)) {
		return -EINVAL;
	}

	scbr_lo = (clkin_freq / clkout_freq) - 1;
	scbr_hi = scbr_lo + 1;

	real_freq_lo = clkin_freq / (scbr_lo + 1);
	real_freq_hi = clkin_freq / (scbr_hi + 1);

	if (ABS(clkout_freq, real_freq_hi) < ABS(clkout_freq, real_freq_lo)) {
		scbr_used = scbr_hi;
		real_freq_used = real_freq_hi;
	} else {
		scbr_used = scbr_lo;
		real_freq_used = real_freq_lo;
	}

	/* Final SCBR must fit into 8 bits */
	if (scbr_used > (QSPI_SCR_SCBR_Msk >> QSPI_SCR_SCBR_Pos)) {
		return -ENOTSUP;
	}

	*divider = scbr_used;
	if (real_freq != NULL) {
		*real_freq = real_freq_used;
	}

	return 0;
}

static int mspi_mchp_qspi_set_divider(qspi_registers_t *regs, uint32_t clkin_freq,
				      uint32_t clkout_freq)
{
	uint32_t divider, scr;
	int ret;

	ret = mspi_mchp_qspi_calc_divider(clkin_freq, clkout_freq, &divider, NULL);
	if (ret != 0) {
		return ret;
	}
	scr = regs->QSPI_SCR;
	scr &= ~(QSPI_SCR_SCBR_Msk);
	scr |= QSPI_SCR_SCBR(divider);
	regs->QSPI_SCR = scr;

	return 0;
}

static int mspi_mchp_qspi_check_update_freq(const struct mspi_mchp_qspi_config *dev_config,
					    uint32_t clkin_freq, struct mspi_dev_cfg *dst_cfg,
					    const struct mspi_dev_cfg *dev_cfg)
{
	uint32_t divider, real_freq;
	int ret;

	ret = mspi_mchp_qspi_calc_divider(clkin_freq, dev_cfg->freq, &divider, &real_freq);
	if (ret != 0) {
		return ret;
	}

	if (real_freq > dev_config->mspi_cfg.max_freq) {
		LOG_DBG("Requested freq %u Hz results in real freq %u Hz, "
			"which exceeds maximum allowed frequency %u Hz",
			dev_cfg->freq, real_freq, dev_config->mspi_cfg.max_freq);
		return -EINVAL;
	}

	dst_cfg->freq = dev_cfg->freq;

	return 0;
}

static int mspi_mchp_qspi_check_update_iomode(struct mspi_dev_cfg *dst_cfg,
					      const struct mspi_dev_cfg *dev_cfg)
{
	int ret;

	ret = mspi_mchp_qspi_check_iomode(dev_cfg->io_mode);
	if (ret != 0) {
		LOG_ERR("io_mode is not supported");
		return ret;
	}
	dst_cfg->io_mode = dev_cfg->io_mode;

	return 0;
}

static int mspi_mchp_qspi_check_update_data_rate(struct mspi_dev_cfg *dst_cfg,
						 const struct mspi_dev_cfg *dev_cfg)
{
	if (dev_cfg->data_rate != MSPI_DATA_RATE_SINGLE) {
		LOG_ERR("data_rate is not supported");
		return -ENOTSUP;
	}
	dst_cfg->data_rate = dev_cfg->data_rate;

	return 0;
}

static int mspi_mchp_qspi_check_update_cpp_mode(struct mspi_dev_cfg *dst_cfg,
						const struct mspi_dev_cfg *dev_cfg)
{
	int ret;

	ret = mspi_mchp_qspi_check_cpp_mode(dev_cfg->cpp);
	if (ret != 0) {
		LOG_ERR("cpp is not supported");
		return ret;
	}
	dst_cfg->cpp = dev_cfg->cpp;

	return 0;
}

static int mspi_mchp_qspi_check_update_endian(struct mspi_dev_cfg *dst_cfg,
					      const struct mspi_dev_cfg *dev_cfg)
{
	dst_cfg->endian = dev_cfg->endian;

	return 0;
}

static int mspi_mchp_qspi_check_update_ce_pol(struct mspi_dev_cfg *dst_cfg,
					      const struct mspi_dev_cfg *dev_cfg)
{
	if (dev_cfg->ce_polarity != MSPI_CE_ACTIVE_LOW) {
		LOG_ERR("ce_polarity is not supported");
		return -ENOTSUP;
	}
	dst_cfg->ce_polarity = dev_cfg->ce_polarity;

	return 0;
}

static int mspi_mchp_qspi_check_update_dqs(struct mspi_dev_cfg *dst_cfg,
					   const struct mspi_dev_cfg *dev_cfg)
{
	if (dev_cfg->dqs_enable) {
		LOG_ERR("dqs_enable is not supported");
		return -ENOTSUP;
	}
	dst_cfg->dqs_enable = dev_cfg->dqs_enable;

	return 0;
}

static int mspi_mchp_qspi_check_update_tx_dummy(struct mspi_dev_cfg *dst_cfg,
						const struct mspi_dev_cfg *dev_cfg)
{
	int ret;

	ret = mspi_mchp_qspi_check_dummy_clock(dev_cfg->tx_dummy);
	if (ret != 0) {
		return ret;
	}

	dst_cfg->tx_dummy = dev_cfg->tx_dummy;

	return 0;
}

static int mspi_mchp_qspi_check_update_rx_dummy(struct mspi_dev_cfg *dst_cfg,
						const struct mspi_dev_cfg *dev_cfg)
{
	int ret;

	ret = mspi_mchp_qspi_check_dummy_clock(dev_cfg->rx_dummy);
	if (ret != 0) {
		return ret;
	}

	dst_cfg->rx_dummy = dev_cfg->rx_dummy;

	return 0;
}

static int mspi_mchp_qspi_check_update_read_cmd(struct mspi_dev_cfg *dst_cfg,
						const struct mspi_dev_cfg *dev_cfg)
{
	dst_cfg->read_cmd = dev_cfg->read_cmd;

	return 0;
}

static int mspi_mchp_qspi_check_update_write_cmd(struct mspi_dev_cfg *dst_cfg,
						 const struct mspi_dev_cfg *dev_cfg)
{
	dst_cfg->write_cmd = dev_cfg->write_cmd;

	return 0;
}

static int mspi_mchp_qspi_check_update_cmd_length(struct mspi_dev_cfg *dst_cfg,
						  const struct mspi_dev_cfg *dev_cfg)
{
	if (dev_cfg->cmd_length > 1) {
		LOG_ERR("cmd_length is not supported");
		return -ENOTSUP;
	}
	dst_cfg->cmd_length = dev_cfg->cmd_length;

	return 0;
}

static int mspi_mchp_qspi_check_update_addr_length(struct mspi_dev_cfg *dst_cfg,
						   const struct mspi_dev_cfg *dev_cfg)
{
	if (dev_cfg->addr_length != 0 && dev_cfg->addr_length != 3 && dev_cfg->addr_length != 4) {
		LOG_ERR("addr_length is not supported");
		return -ENOTSUP;
	}
	dst_cfg->addr_length = dev_cfg->addr_length;

	return 0;
}

static int mspi_mchp_qspi_check_update_mem_bound(struct mspi_dev_cfg *dst_cfg,
						 const struct mspi_dev_cfg *dev_cfg)
{
	dst_cfg->mem_boundary = dev_cfg->mem_boundary;

	return 0;
}

static int mspi_mchp_qspi_check_update_break_time(struct mspi_dev_cfg *dst_cfg,
						  const struct mspi_dev_cfg *dev_cfg)
{
	dst_cfg->time_to_break = dev_cfg->time_to_break;

	return 0;
}

static int mspi_mchp_qspi_dev_config(const struct device *dev, const struct mspi_dev_id *dev_id,
				     const enum mspi_dev_cfg_mask param_mask,
				     const struct mspi_dev_cfg *dev_cfg)
{
	struct mspi_mchp_qspi_data *dev_data = dev->data;
	const struct mspi_mchp_qspi_config *dev_config = dev->config;
	struct mspi_dev_cfg tmp_cfg;
	qspi_registers_t *regs = dev_config->regs;
	uint32_t clkin_freq, scr;
	int ret = 0;

	ret = k_mutex_lock(&dev_data->lock, K_FOREVER);
	if (ret) {
		return ret;
	}

	/* Apply requested changes to temp, commit dev_data->dev_cfg */
	tmp_cfg = dev_data->dev_cfg;

	do {
		/* Try to setup divider */
		ret = clock_control_get_rate(dev_config->clock, dev_config->clock_cfg, &clkin_freq);
		if (ret != 0) {
			break;
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_CE_NUM) != 0) {
			ret = mspi_mchp_qspi_check_update_ce(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) != 0) {
			ret = mspi_mchp_qspi_check_update_freq(dev_config, clkin_freq, &tmp_cfg,
							       dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_IO_MODE) != 0) {
			ret = mspi_mchp_qspi_check_update_iomode(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) != 0) {
			ret = mspi_mchp_qspi_check_update_data_rate(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_CPP) != 0) {
			ret = mspi_mchp_qspi_check_update_cpp_mode(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_ENDIAN) != 0) {
			ret = mspi_mchp_qspi_check_update_endian(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_CE_POL) != 0) {
			ret = mspi_mchp_qspi_check_update_ce_pol(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_DQS) != 0) {
			ret = mspi_mchp_qspi_check_update_dqs(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_RX_DUMMY) != 0) {
			ret = mspi_mchp_qspi_check_update_rx_dummy(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_TX_DUMMY) != 0) {
			ret = mspi_mchp_qspi_check_update_tx_dummy(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_READ_CMD) != 0) {
			ret = mspi_mchp_qspi_check_update_read_cmd(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_WRITE_CMD) != 0) {
			ret = mspi_mchp_qspi_check_update_write_cmd(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_CMD_LEN) != 0) {
			ret = mspi_mchp_qspi_check_update_cmd_length(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_ADDR_LEN) != 0) {
			ret = mspi_mchp_qspi_check_update_addr_length(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_MEM_BOUND) != 0) {
			ret = mspi_mchp_qspi_check_update_mem_bound(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_BREAK_TIME) != 0) {
			ret = mspi_mchp_qspi_check_update_break_time(&tmp_cfg, dev_cfg);
			if (ret != 0) {
				break;
			}
		}
	} while (0);

	if (ret == 0) {
		dev_data->dev_cfg = tmp_cfg;

		if ((param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) != 0) {
			mspi_mchp_qspi_set_divider(regs, clkin_freq, dev_data->dev_cfg.freq);
		}

		if ((param_mask & MSPI_DEVICE_CONFIG_CPP) != 0) {
			scr = regs->QSPI_SCR;
			scr &= ~(QSPI_SCR_CPHA_Msk | QSPI_SCR_CPOL_Msk);
			scr |= mspi_mchp_qspi_convert_cpp_mode(dev_data->dev_cfg.cpp);
			regs->QSPI_SCR = scr;
		}

		regs->QSPI_CR = QSPI_CR_QSPIEN_Msk;

		dev_data->dev_id = dev_id;
	}

	k_mutex_unlock(&dev_data->lock);

	return ret;
}

static int mspi_mchp_qspi_get_channel_status(const struct device *dev, uint8_t ch)
{
	struct mspi_mchp_qspi_data *dev_data = dev->data;

	if (ch != 0) {
		return -ENOTSUP;
	}

	return dev_data->in_transfer ? -EBUSY : 0;
}

static int mspi_mchp_qspi_prepare_frame(const struct device *dev, const struct mspi_xfer *xfer,
					struct mspi_xfer_packet const *packet)
{
	const struct mspi_mchp_qspi_config *dev_config = dev->config;
	struct mspi_mchp_qspi_data *dev_data = dev->data;
	qspi_registers_t *regs = dev_config->regs;
	uint32_t ifr_reg = 0;

	if (xfer->addr_length != 0 && xfer->addr_length != 3 && xfer->addr_length != 4) {
		return -EINVAL;
	}

	if (xfer->cmd_length != 0 && xfer->cmd_length != 1) {
		return -EINVAL;
	}

	ifr_reg |= mspi_mchp_qspi_convert_iomode(dev_data->dev_cfg.io_mode);
	ifr_reg |= (xfer->addr_length != 0 ? QSPI_IFR_ADDREN_Msk : 0);
	ifr_reg |= QSPI_IFR_ADDRL(xfer->addr_length == 4);
	ifr_reg |= (xfer->cmd_length != 0 ? QSPI_IFR_INSTEN_Msk : 0);
	ifr_reg |= QSPI_IFR_DATAEN(packet->num_bytes != 0);

	if (packet->dir == MSPI_TX) {
		if (packet->num_bytes == 0) {
			ifr_reg |= QSPI_IFR_TFRTYP(QSPI_IFR_TFRTYP_TRSFR_WRITE_Val);
		} else {
			ifr_reg |= QSPI_IFR_TFRTYP(QSPI_IFR_TFRTYP_TRSFR_WRITE_MEMORY_Val);
		}
		ifr_reg |= QSPI_IFR_NBDUM(xfer->tx_dummy);
	} else {
		if (packet->num_bytes == 0) {
			ifr_reg |= QSPI_IFR_TFRTYP(QSPI_IFR_TFRTYP_TRSFR_READ_Val);
		} else {
			ifr_reg |= QSPI_IFR_TFRTYP(QSPI_IFR_TFRTYP_TRSFR_READ_MEMORY_Val);
		}
		ifr_reg |= QSPI_IFR_NBDUM(xfer->rx_dummy);
	}

	if (xfer->addr_length != 0 && packet->num_bytes == 0) {
		regs->QSPI_IAR = QSPI_IAR_ADDR(packet->address);
	}
	if (xfer->cmd_length != 0) {
		regs->QSPI_ICR = QSPI_ICR_INST(packet->cmd);
	}

	/* Disable interrupt, clear old flags, reset semaphore */
	regs->QSPI_IDR = QSPI_IDR_INSTRE_Msk;
	(void)regs->QSPI_SR;
	k_sem_reset(&dev_data->transfer_done);

	/* Enable transfer complete interrupt, except of DMA */
	if (packet->num_bytes == 0 || !IS_ENABLED(CONFIG_MSPI_DMA)) {
		regs->QSPI_IER = QSPI_IER_INSTRE_Msk;
	}

	__DSB();
	regs->QSPI_IFR = ifr_reg;

	__DSB();
	/* Read QSPI_IFR (dummy read) to synchronize APB and AHB */
	if (packet->num_bytes != 0) {
		(void)regs->QSPI_IFR;
	}

	return 0;
}

#if defined(CONFIG_MSPI_DMA)
static void mspi_mchp_qspi_dma_transfer_complete(const struct device *dma_dev, void *arg,
						 uint32_t id, int error_code)
{
	const struct device *dev = (const struct device *)arg;
	const struct mspi_mchp_qspi_config *dev_config = dev->config;
	struct mspi_mchp_qspi_data *dev_data = dev->data;
	qspi_registers_t *regs = dev_config->regs;

	/* Invalidate received data memory */
	if (dev_data->invalidate_mem != NULL) {
		sys_cache_data_invd_range(dev_data->invalidate_mem, dev_data->invalidate_size);
	}

	dev_data->dma_errno = error_code;
	/* Complete the transfer in all scenarios to release the CS line */
	regs->QSPI_IER = QSPI_IER_INSTRE_Msk;
	regs->QSPI_CR = QSPI_CR_LASTXFER_Msk;
}

static int mspi_mchp_qspi_dma_transfer(const struct device *dev, uint8_t *dest, uint8_t *src,
				       size_t num_bytes)
{
	const struct mspi_mchp_qspi_config *dev_config = dev->config;
	struct dma_block_config dma_blk = {0};
	struct dma_config dma_cfg = {0};
	int ret;

	dma_blk.block_size = num_bytes;
	dma_blk.source_address = (uint32_t)src;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	dma_blk.dest_address = (uint32_t)dest;
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = 1;
	dma_cfg.source_burst_length = 1;
	dma_cfg.dest_data_size = 1;
	dma_cfg.dest_burst_length = 1;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = mspi_mchp_qspi_dma_transfer_complete;
	dma_cfg.block_count = 1;
	dma_cfg.complete_callback_en = true;
	dma_cfg.head_block = &dma_blk;

	ret = dma_config(dev_config->dma_dev, dev_config->dma_channel, &dma_cfg);
	if (ret != 0) {
		return ret;
	}

	return dma_start(dev_config->dma_dev, dev_config->dma_channel);
}

static int mspi_mchp_qspi_dma_send(const struct device *dev, uint8_t *dest, uint8_t *src,
				   size_t num_bytes)
{
	struct mspi_mchp_qspi_data *dev_data = dev->data;

	dev_data->invalidate_mem = NULL;
	dev_data->invalidate_size = 0;
	sys_cache_data_flush_range(src, num_bytes);
	return mspi_mchp_qspi_dma_transfer(dev, dest, src, num_bytes);
}

static int mspi_mchp_qspi_dma_receive(const struct device *dev, uint8_t *dest, uint8_t *src,
				      size_t num_bytes)
{
	struct mspi_mchp_qspi_data *dev_data = dev->data;

	dev_data->invalidate_mem = dest;
	dev_data->invalidate_size = num_bytes;
	return mspi_mchp_qspi_dma_transfer(dev, dest, src, num_bytes);
}
#endif

static int mspi_mchp_qspi_transceive(const struct device *dev, const struct mspi_dev_id *dev_id,
				     const struct mspi_xfer *xfer)
{
	const struct mspi_mchp_qspi_config *dev_config = dev->config;
	struct mspi_mchp_qspi_data *dev_data = dev->data;
	qspi_registers_t *regs = dev_config->regs;
	struct mspi_xfer_packet const *packet;
	int ret = 0;

	if (xfer->async) {
		return -ENOTSUP;
	}

	ret = k_mutex_lock(&dev_data->lock, K_FOREVER);
	if (ret) {
		return ret;
	}

	dev_data->in_transfer = true;

	do {
		if (dev_id != dev_data->dev_id) {
			LOG_ERR("Controller is not configured for this device");
			ret = -EINVAL;
			break;
		}

		for (uint32_t i = 0; ret == 0 && i < xfer->num_packet; i++) {
			packet = &xfer->packets[i];

			/* Extra constraint for command retrieving data without address */
			if (packet->num_bytes != 0 && xfer->addr_length == 0 &&
			    packet->address != 0) {
				/* Command with data access and no address length expects to have
				 * address set to 0
				 */
				ret = -EINVAL;
				break;
			}

			/* Check address range */
			if ((packet->num_bytes != 0) &&
			    ((packet->address > dev_config->mmap_size) ||
			     (dev_config->mmap_size - packet->address) < packet->num_bytes)) {
				ret = -EINVAL;
				break;
			}

			ret = mspi_mchp_qspi_prepare_frame(dev, xfer, packet);
			if (ret != 0) {
				break;
			}

#if defined(CONFIG_MSPI_DMA)
			if (packet->num_bytes) {
				if (packet->dir == MSPI_TX) {
					ret = mspi_mchp_qspi_dma_send(
						dev,
						&((uint8_t *)dev_config->mmap)[packet->address],
						&packet->data_buf[0], packet->num_bytes);
				} else {
					ret = mspi_mchp_qspi_dma_receive(
						dev, &packet->data_buf[0],
						&((uint8_t *)dev_config->mmap)[packet->address],
						packet->num_bytes);
				}
				if (ret != 0) {
					/* Deassert CS */
					regs->QSPI_CR = QSPI_CR_LASTXFER_Msk;
					break;
				}
				ret = k_sem_take(&dev_data->transfer_done,
						 MSPI_MCHP_QSPI_G1_WAIT_FOR_COMPLETION_MS);
				if (ret != 0) {
					dma_stop(dev_config->dma_dev, dev_config->dma_channel);
					regs->QSPI_IDR = QSPI_IDR_INSTRE_Msk;
					/* Deassert CS */
					regs->QSPI_CR = QSPI_CR_LASTXFER_Msk;
					ret = ret == -EAGAIN ? -ETIMEDOUT : ret;
					break;
				} else if (dev_data->dma_errno < 0) {
					dma_stop(dev_config->dma_dev, dev_config->dma_channel);
					ret = -EFAULT;
					break;
				}
			} else {
				ret = k_sem_take(&dev_data->transfer_done,
						 MSPI_MCHP_QSPI_G1_WAIT_FOR_COMPLETION_MS);
				if (ret) {
					regs->QSPI_IDR = QSPI_IDR_INSTRE_Msk;
					ret = (ret == 0 ? -EFAULT
							: (ret == -EAGAIN ? -ETIMEDOUT : ret));
					break;
				}
			}
#else
			if (packet->dir == MSPI_TX) {
				for (uint32_t j = 0; j < packet->num_bytes; j++) {
					((uint8_t *)dev_config->mmap)[j + packet->address] =
						packet->data_buf[j];
				}
			} else {
				for (uint32_t j = 0; j < packet->num_bytes; j++) {
					packet->data_buf[j] =
						((uint8_t *)dev_config->mmap)[j + packet->address];
				}
			}
			__DSB();
			if (packet->num_bytes != 0) {
				regs->QSPI_CR = QSPI_CR_LASTXFER_Msk;
			}
			ret = k_sem_take(&dev_data->transfer_done,
					 MSPI_MCHP_QSPI_G1_WAIT_FOR_COMPLETION_MS);
			if (ret) {
				regs->QSPI_IDR = QSPI_IDR_INSTRE_Msk;
				ret = (ret == 0 ? -EFAULT : (ret == -EAGAIN ? -ETIMEDOUT : ret));
				break;
			}
#endif
		}
	} while (0);

	dev_data->in_transfer = false;
	k_mutex_unlock(&dev_data->lock);

	return ret;
}

static void mspi_mchp_qspi_isr(const struct device *dev)
{
	const struct mspi_mchp_qspi_config *dev_config = dev->config;
	struct mspi_mchp_qspi_data *dev_data = dev->data;
	qspi_registers_t *regs = dev_config->regs;
	uint32_t status = regs->QSPI_SR;

	if (status & QSPI_SR_INSTRE_Msk) {
		regs->QSPI_IDR = QSPI_IDR_INSTRE_Msk;
		k_sem_give(&dev_data->transfer_done);
	}
}

static int mspi_mchp_qspi_initialize(const struct device *dev)
{
	const struct mspi_mchp_qspi_config *dev_config = dev->config;
	struct mspi_mchp_qspi_data *dev_data = dev->data;
	const struct mspi_dt_spec spec = {
		.bus = dev,
		.config = dev_config->mspi_cfg,
	};

	k_mutex_init(&dev_data->lock);

	k_sem_init(&dev_data->transfer_done, 0, 1);

	return mspi_mchp_qspi_config(&spec);
}

static DEVICE_API(mspi, mspi_mchp_qspi_driver_api) = {
	.config = mspi_mchp_qspi_config,
	.dev_config = mspi_mchp_qspi_dev_config,
	.get_channel_status = mspi_mchp_qspi_get_channel_status,
	.transceive = mspi_mchp_qspi_transceive,
};

#if defined(CONFIG_MSPI_DMA)
#define DMA_INIT(n)                                                                                \
	.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, txrx)),                              \
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, txrx, channel),
#else
#define DMA_INIT(n)
#endif

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void mchp_qspi_##n##_irq_config(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), mspi_mchp_qspi_isr,         \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct mspi_mchp_qspi_config mchp_qspi_##n##_config = {                       \
		.regs = (qspi_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, regs),                     \
		.mmap = (void *)DT_INST_REG_ADDR_BY_NAME(n, mmap),                                 \
		.mmap_size = (size_t)DT_INST_REG_SIZE_BY_NAME(n, mmap),                            \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_cfg = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config_func = mchp_qspi_##n##_irq_config,                                     \
		.mspi_cfg =                                                                        \
			{                                                                          \
				.channel_num = 0,                                                  \
				.op_mode =                                                         \
					DT_INST_ENUM_IDX_OR(n, op_mode, MSPI_OP_MODE_CONTROLLER),  \
				.duplex = DT_INST_ENUM_IDX_OR(n, duplex, MSPI_HALF_DUPLEX),        \
				.num_periph = DT_INST_CHILD_NUM(n),                                \
				.max_freq = DT_INST_PROP(n, clock_frequency),                      \
			},                                                                         \
		DMA_INIT(n)};                                                                      \
	static struct mspi_mchp_qspi_data mchp_qspi_##n##_data = {};                               \
	DEVICE_DT_INST_DEFINE(n, &mspi_mchp_qspi_initialize, NULL, &mchp_qspi_##n##_data,          \
			      &mchp_qspi_##n##_config, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mspi_mchp_qspi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
