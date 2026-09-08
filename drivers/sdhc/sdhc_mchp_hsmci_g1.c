/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 Nikhef
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip High Speed MultiMedia Card Interface (HSMCI) SD/MMC host controller driver.
 *
 */

#define DT_DRV_COMPAT microchip_hsmci_g1_sdhc

#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
#include <zephyr/drivers/dma.h>
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_DMA */

LOG_MODULE_REGISTER(sdhc_mchp_hsmci_g1, CONFIG_SDHC_LOG_LEVEL);

#define HSMCI_DEFAULT_TIMEOUT_US 5000000u
#define HSMCI_POLL_TIMEOUT_US    100000u
#define MSMCI_MAX_DIVISOR        0x1FFu
#define HSMCI_SR_ERR                                                                               \
	(HSMCI_SR_RINDE_Msk | HSMCI_SR_RDIRE_Msk | HSMCI_SR_RCRCE_Msk | HSMCI_SR_RENDE_Msk |       \
	 HSMCI_SR_RTOE_Msk | HSMCI_SR_DCRCE_Msk | HSMCI_SR_DTOE_Msk | HSMCI_SR_CSTOE_Msk |         \
	 HSMCI_SR_OVRE_Msk | HSMCI_SR_UNRE_Msk)
#define HSMCI_SR_XFER_ERR                                                                          \
	(HSMCI_SR_UNRE_Msk | HSMCI_SR_OVRE_Msk | HSMCI_SR_DTOE_Msk | HSMCI_SR_DCRCE_Msk)
#define HSMCI_MINIMUM_FREQUENCY_DIVIDER 513u
#define HSMCI_MAXIMUM_FREQUENCY_DIVIDER 2u
#define HSMCI_POWER_DELAY               500u
#define HSMCI_MAX_CURRENT_330           4u
#define HSMCI_MAX_CLOCK_TICKS_TIMEOUT   15u

static const uint8_t resp2size[] = {
	[SD_RSP_TYPE_NONE] = HSMCI_CMDR_RSPTYP_NORESP,
	[SD_RSP_TYPE_R1] = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R1b] = HSMCI_CMDR_RSPTYP_R1B,
	[SD_RSP_TYPE_R2] = HSMCI_CMDR_RSPTYP_136_BIT,
	[SD_RSP_TYPE_R3] = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R4] = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R5] = 0u /* SDIO not supported */,
	[SD_RSP_TYPE_R5b] = 0u /* SDIO not supported */,
	[SD_RSP_TYPE_R6] = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R7] = HSMCI_CMDR_RSPTYP_48_BIT,
};

struct sdhc_mchp_hsmci_config {
	/* Register block */
	hsmci_registers_t *regs;

	/* Clock driver */
	const struct device *clock;

	/* Main clock subsystem. */
	clock_control_subsys_t clock_subsys;

	/* Pin configuration */
	const struct pinctrl_dev_config *pincfg;

	/* GPIO pin for carrier detect */
	struct gpio_dt_spec carrier_detect;

	/* Connects and enables the peripheral IRQ */
	void (*irq_config)(const struct device *dev);

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
	struct {
		const struct device *dev;
		uint32_t channel;
		uint32_t perid;
	} dma;
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_DMA */
};

struct sdhc_mchp_hsmci_data {
	/* Open drain bus mode */
	bool open_drain;

	/* Driver mutex */
	struct k_mutex mutex;

	/* Signaled by the ISR when an armed status flag fires */
	struct k_sem done;

	/* HSMCI_SR value captured by the ISR when it signals `done` */
	uint32_t irq_sr;

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
	/* Signaled by the DMA callback when a block transfer completes */
	struct k_sem dma_done;
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_DMA */
};

static int sdhc_mchp_hsmci_reset(const struct device *dev)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;
	uint32_t mr = regs->HSMCI_MR;
	uint32_t dtor = regs->HSMCI_DTOR;
	uint32_t sdcr = regs->HSMCI_SDCR;
	uint32_t cstor = regs->HSMCI_CSTOR;
	uint32_t cfg = regs->HSMCI_CFG;

	regs->HSMCI_CR = HSMCI_CR_SWRST_Msk;
	regs->HSMCI_MR = mr;
	regs->HSMCI_DTOR = dtor;
	regs->HSMCI_SDCR = sdcr;
	regs->HSMCI_CSTOR = cstor;
	regs->HSMCI_CFG = cfg;

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE)
	regs->HSMCI_CR = HSMCI_CR_PWSEN_Msk | HSMCI_CR_MCIEN_Msk;
#else
	regs->HSMCI_CR = HSMCI_CR_MCIEN_Msk;
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE */

	return 0;
}

static int sdhc_mchp_hsmci_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	uint32_t frequency = 0u;
	int ret = 0;

	ret = clock_control_get_rate(dev_config->clock, dev_config->clock_subsys, &frequency);
	if (ret != 0) {
		LOG_ERR("Failed to get clock rate: %d", ret);
		return ret;
	}

	memset(props, 0u, sizeof(*props));

	props->f_max = frequency / HSMCI_MAXIMUM_FREQUENCY_DIVIDER;
	/* High Speed MultiMedia Card Interface clock (MCCK or HSMCI_CK) is Host Clock (MAIN_CLK)
	 * divided by 2×CLKDIV+CLKODD+2.
	 * CLKDIV is 8 bits and CLKODD is 1, so the maximum divider is 2×255+1+2 = 513
	 */
	props->f_min = frequency / HSMCI_MINIMUM_FREQUENCY_DIVIDER;
	/* high-speed not working yet due to limitations of the SDHC sm */
	props->host_caps.high_spd_support = false;
	props->power_delay = HSMCI_POWER_DELAY;
	props->is_spi = false;
	props->max_current_330 = HSMCI_MAX_CURRENT_330;

	return 0;
}

static int sdhc_mchp_hsmci_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	struct sdhc_mchp_hsmci_data *dev_data = dev->data;
	hsmci_registers_t *const regs = dev_config->regs;
	uint32_t frequency = 0u;
	uint32_t div_val = 0u;
	int ret = 0;

	LOG_DBG("%s(clock=%d, bus_width=%d, timing=%d, mode=%d)", __func__, ios->clock,
		ios->bus_width, ios->timing, ios->bus_mode);

	if (ios->clock > 0u) {
		ret = clock_control_get_rate(dev_config->clock, dev_config->clock_subsys,
					     &frequency);
		if (ret != 0) {
			LOG_ERR("Failed to get clock rate, err=%d", ret);
			return ret;
		}

		if (ios->clock > frequency / 2u) {
			return -ENOTSUP;
		}

		div_val = DIV_ROUND_UP(frequency, ios->clock) - 2u;

		if (div_val > MSMCI_MAX_DIVISOR) {
			div_val = MSMCI_MAX_DIVISOR;
		}

		LOG_DBG("divider: %d (freq=%d)", div_val, frequency / (div_val + 2u));

		regs->HSMCI_MR &= ~(HSMCI_MR_CLKDIV_Msk | HSMCI_MR_CLKODD_Msk);
		regs->HSMCI_MR |= ((div_val & 1u) ? HSMCI_MR_CLKODD_Msk : 0u) |
				  HSMCI_MR_CLKDIV(div_val >> 1u);
	}

	if (ios->bus_width) {
		regs->HSMCI_SDCR &= ~HSMCI_SDCR_SDCBUS_Msk;

		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			regs->HSMCI_SDCR = HSMCI_SDCR_SDCBUS_1;
			break;
		case SDHC_BUS_WIDTH4BIT:
			regs->HSMCI_SDCR = HSMCI_SDCR_SDCBUS_4;
			break;
		default:
			return -ENOTSUP;
		}
	}

	dev_data->open_drain = (ios->bus_mode == SDHC_BUSMODE_OPENDRAIN);

	if (ios->timing) {
		switch (ios->timing) {
		case SDHC_TIMING_LEGACY:
			regs->HSMCI_CFG &= ~HSMCI_CFG_HSMODE_Msk;
			break;
		case SDHC_TIMING_HS:
			regs->HSMCI_CFG |= HSMCI_CFG_HSMODE_Msk;
			break;
		default:
			return -ENOTSUP;
		}
	}

	return 0;
}

static int sdhc_mchp_hsmci_init(const struct device *dev)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	struct sdhc_mchp_hsmci_data *dev_data = dev->data;
	int ret = 0;

	ret = k_mutex_init(&dev_data->mutex);
	if (ret != 0) {
		LOG_ERR("Unable to initialize mutex: %i", ret);
		return ret;
	}

	ret = k_sem_init(&dev_data->done, 0, 1);
	if (ret != 0) {
		LOG_ERR("Unable to initialize semaphore: %i", ret);
		return ret;
	}

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
	ret = k_sem_init(&dev_data->dma_done, 0, 1);
	if (ret != 0) {
		LOG_ERR("Unable to initialize DMA semaphore: %i", ret);
		return ret;
	}
	if (!device_is_ready(dev_config->dma.dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_DMA */

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Unable to configure pinctrl: %i", ret);
		return ret;
	}

	/* Enable module's clock */
	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Unable to enable peripheral clock: %i", ret);
		return ret;
	}

	/* init carrier detect (if set) */
	if (dev_config->carrier_detect.port != NULL) {
		if (!gpio_is_ready_dt(&dev_config->carrier_detect)) {
			LOG_ERR("GPIO port for carrier-detect pin is not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&dev_config->carrier_detect, GPIO_INPUT);
		if (ret != 0) {
			LOG_ERR("Couldn't configure carrier-detect pin: %i", ret);
			return ret;
		}
	}

	hsmci_registers_t *const regs = dev_config->regs;

	/* reset the device */
	regs->HSMCI_CR = HSMCI_CR_SWRST_Msk;
	/* mask all interrupt sources before the IRQ line is enabled below */
	regs->HSMCI_IDR = 0xFFFFFFFFu;
	regs->HSMCI_CR = HSMCI_CR_PWSDIS_Msk;
	regs->HSMCI_CR = HSMCI_CR_MCIEN_Msk;
#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE)
	regs->HSMCI_MR = HSMCI_MR_RDPROOF_Msk | HSMCI_MR_WRPROOF_Msk |
			 HSMCI_MR_PWSDIV(CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE_DIV);
	regs->HSMCI_CR = HSMCI_CR_PWSEN_Msk;
#else
	regs->HSMCI_MR = HSMCI_MR_RDPROOF_Msk | HSMCI_MR_WRPROOF_Msk;
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE */

	dev_config->irq_config(dev);

	return 0;
}

static int sdhc_mchp_hsmci_get_card_present(const struct device *dev)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;

	if (dev_config->carrier_detect.port == NULL) {
		return 1;
	}

	return gpio_pin_get_dt(&dev_config->carrier_detect);
}

static int sdhc_mchp_hsmci_card_busy(const struct device *dev)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;

	return (regs->HSMCI_SR & HSMCI_SR_NOTBUSY_Msk) == 0u;
}

static void sdhc_mchp_hsmci_isr(const struct device *dev)
{
	struct sdhc_mchp_hsmci_data *dev_data = dev->data;
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;

	dev_data->irq_sr = regs->HSMCI_SR;
	regs->HSMCI_IDR = regs->HSMCI_IMR;
	k_sem_give(&dev_data->done);
}

/*
 * Waits for any bit in irq_mask to assert in the SR register.
 * *sr_out receives the HSMCI_SR value captured by the ISR when it fired.
 */
static int sdhc_mchp_hsmci_wait_irq(const struct device *dev, uint32_t irq_mask,
				    uint32_t timeout_us, uint32_t *sr_out)
{
	struct sdhc_mchp_hsmci_data *dev_data = dev->data;
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;

	k_sem_reset(&dev_data->done);
	regs->HSMCI_IER = irq_mask;

	if (k_sem_take(&dev_data->done, K_USEC(timeout_us)) != 0) {
		regs->HSMCI_IDR = irq_mask;
		return -ETIMEDOUT;
	}

	if (sr_out != NULL) {
		*sr_out = dev_data->irq_sr;
	}

	return 0;
}

static int sdhc_mchp_hsmci_send_clocks(const struct device *dev)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;
	int ret = 0;

	regs->HSMCI_MR &= ~(HSMCI_MR_WRPROOF_Msk | HSMCI_MR_RDPROOF_Msk | HSMCI_MR_FBYTE_Msk);
	regs->HSMCI_ARGR = 0u;
	regs->HSMCI_CMDR =
		HSMCI_CMDR_RSPTYP_NORESP | HSMCI_CMDR_SPCMD_INIT | HSMCI_CMDR_OPDCMD_OPENDRAIN;

	ret = sdhc_mchp_hsmci_wait_irq(dev, HSMCI_SR_CMDRDY_Msk, HSMCI_POLL_TIMEOUT_US, NULL);

	regs->HSMCI_MR |= HSMCI_MR_WRPROOF_Msk | HSMCI_MR_RDPROOF_Msk;

	if (ret != 0) {
		LOG_ERR("Timeout waiting for CMDRDY after init clocks");
		return ret;
	}

	return 0;
}

static int sdhc_mchp_hsmci_send_cmd(const struct device *dev, struct sdhc_command *cmd,
				    uint32_t cmdr, struct sdhc_mchp_hsmci_data *dev_data)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;
	uint32_t sr = 0u;
	uint32_t err_mask = HSMCI_SR_ERR;
	uint8_t nrt = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK;
	int ret = 0;

	regs->HSMCI_ARGR = cmd->arg;

	cmdr |= HSMCI_CMDR_CMDNB(cmd->opcode) | HSMCI_CMDR_MAXLAT_64;
	if (dev_data->open_drain) {
		cmdr |= HSMCI_CMDR_OPDCMD_OPENDRAIN;
	}

	if (nrt > SD_RSP_TYPE_R7) {
		return -ENOTSUP;
	}

	cmdr |= resp2size[nrt];
	regs->HSMCI_CMDR = cmdr;

	/* special case, ignore CRC status if response is R3 to clear it */
	if (nrt == SD_RSP_TYPE_R3 || nrt == SD_RSP_TYPE_NONE) {
		err_mask &= ~HSMCI_SR_RCRCE_Msk;
	}

	ret = sdhc_mchp_hsmci_wait_irq(dev, HSMCI_SR_CMDRDY_Msk | err_mask, HSMCI_POLL_TIMEOUT_US,
				       &sr);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for CMDRDY (SR=%08x)", regs->HSMCI_SR);
		return ret;
	}

	sr &= err_mask;
	if (sr != 0u) {
		LOG_DBG("Status register error bits: %08x", sr);
		return -EIO;
	}

	if (nrt == SD_RSP_TYPE_R1b) {
		ret = sdhc_mchp_hsmci_wait_irq(dev, HSMCI_SR_NOTBUSY_Msk, HSMCI_POLL_TIMEOUT_US,
					       &sr);
		if (ret != 0) {
			LOG_ERR("Timeout waiting for card not-busy (R1b)");
			return ret;
		}
		if (sr & HSMCI_SR_DTIP_Msk) {
			LOG_ERR("DTIP unexpectedly set after NOTBUSY for R1b (SR=%08x)", sr);
			return -EIO;
		}
	}

	/* No response was received: RSPR still holds the previous command's response */
	if (nrt == SD_RSP_TYPE_NONE) {
		return 0;
	}

	/* RSPR is just a FIFO, index is of no consequence */
	cmd->response[3] = regs->HSMCI_RSPR[0];
	cmd->response[2] = regs->HSMCI_RSPR[0];
	cmd->response[1] = regs->HSMCI_RSPR[0];
	cmd->response[0] = regs->HSMCI_RSPR[0];

	return 0;
}

static int sdhc_mchp_hsmci_wait_write_end(const struct device *dev)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;
	uint32_t sr = 0u;
	int ret = 0;

	ret = sdhc_mchp_hsmci_wait_irq(dev, HSMCI_SR_NOTBUSY_Msk | HSMCI_SR_XFER_ERR,
				       HSMCI_POLL_TIMEOUT_US, &sr);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for write end (NOTBUSY)");
		return ret;
	}

	if ((sr & HSMCI_SR_XFER_ERR) != 0u) {
		LOG_DBG("PDC sr 0x%08x last transfer error", sr);
		return -EIO;
	}

	if (!(regs->HSMCI_SR & HSMCI_SR_FIFOEMPTY_Msk)) {
		return -EIO;
	}
	return 0;
}

static int sdhc_mchp_hsmci_wait_read_end(const struct device *dev)
{
	uint32_t sr = 0u;
	int ret = 0;

	ret = sdhc_mchp_hsmci_wait_irq(dev, HSMCI_SR_XFRDONE_Msk | HSMCI_SR_XFER_ERR,
				       HSMCI_POLL_TIMEOUT_US, &sr);
	if (ret != 0) {
		LOG_ERR("Timeout waiting for read end (XFRDONE)");
		return ret;
	}

	if ((sr & HSMCI_SR_XFER_ERR) != 0u) {
		return -EIO;
	}
	return 0;
}

static int sdhc_mchp_hsmci_write_timeout(const struct device *dev, int timeout_ms)
{
	/* timeout multiplier shift (actual value is 1 << mul_shift[*]) */
	uint8_t const multiplier_shift[] = {0, 4, 7, 8, 10, 12, 16, 20};
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;
	uint32_t frequency = 0u;
	uint32_t clock_ticks = 0u;
	uint32_t multiplier = 0u;
	uint32_t max_clock_ticks = 0u;
	int ret = 0;

	ret = clock_control_get_rate(dev_config->clock, dev_config->clock_subsys, &frequency);
	if (ret != 0) {
		LOG_ERR("Failed to get clock rate: %d", ret);
		return ret;
	}

	clock_ticks = (uint32_t)(((uint64_t)frequency * (uint64_t)timeout_ms) / 1000u);

	for (size_t i = 0u; i < sizeof(multiplier_shift) / sizeof(multiplier_shift[0u]); i++) {
		/* The `DTOMUL` field contains a multiplier expressed in steps as per the
		 * `multiplier_shift` array.
		 */
		multiplier = 1u << multiplier_shift[i];
		/* The `DTOCYC` field is 4 bits wide and specifies the maximum number of cycles
		 * (multiplied by `DTOMUL`) to wait for between two data block transfer. The maximum
		 * expressible timeout is thus 0xF (=15) times multiplier.
		 */
		max_clock_ticks = HSMCI_MAX_CLOCK_TICKS_TIMEOUT * multiplier;

		/* If the maximum expressible timeout with this multiplier is bigger than the number
		 * of clock cycles elapsed in timeout_ms, use it
		 */
		if (max_clock_ticks > clock_ticks) {
			regs->HSMCI_DTOR =
				HSMCI_DTOR_DTOMUL(i) |
				HSMCI_DTOR_DTOCYC((clock_ticks + multiplier - 1) / multiplier);
			return 0;
		}
	}

	/* No other choice but to wait for the maximum possible time; it's not enough, but it will
	 * have to do.
	 */
	regs->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_Msk | HSMCI_DTOR_DTOCYC_Msk;

	return 0;
}

#if !defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
static inline int sdhc_mchp_hsmci_wait_write_transfer_done(hsmci_registers_t *const regs)
{
	uint32_t sr = 0u;
	uint32_t wf_cycle_count = k_us_to_cyc_ceil32(HSMCI_POLL_TIMEOUT_US);
	uint32_t wf_start = k_cycle_get_32();

	do {
		sr = regs->HSMCI_SR;
		if ((sr & (HSMCI_SR_TXRDY_Msk | HSMCI_SR_XFER_ERR)) != 0u) {
			break;
		}
		k_busy_wait(1u);
	} while (wf_cycle_count > (k_cycle_get_32() - wf_start));

	if ((sr & HSMCI_SR_XFER_ERR) != 0u) {
		return -EIO;
	}

	if ((sr & HSMCI_SR_TXRDY_Msk) == 0u) {
		LOG_ERR("Timeout waiting for TXRDY");
		return -ETIMEDOUT;
	}

	return 0;
}

static inline int sdhc_mchp_hsmci_wait_read_transfer_done(hsmci_registers_t *const regs)
{
	uint32_t sr = 0u;
	uint32_t wf_cycle_count = k_us_to_cyc_ceil32(HSMCI_POLL_TIMEOUT_US);
	uint32_t wf_start = k_cycle_get_32();

	do {
		sr = regs->HSMCI_SR;
		if ((sr & (HSMCI_SR_RXRDY_Msk | HSMCI_SR_XFER_ERR)) != 0u) {
			break;
		}
		k_busy_wait(1u);
	} while (wf_cycle_count > (k_cycle_get_32() - wf_start));

	if ((sr & HSMCI_SR_XFER_ERR) != 0u) {
		return -EIO;
	}

	if ((sr & HSMCI_SR_RXRDY_Msk) == 0u) {
		LOG_ERR("Timeout waiting for RXRDY");
		return -ETIMEDOUT;
	}

	return 0;
}
#endif /* !CONFIG_SDHC_MCHP_HSMCI_G1_DMA */

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
static void sdhc_mchp_hsmci_dma_callback(const struct device *dma_dev, void *user_data,
					 uint32_t channel, int status)
{
	ARG_UNUSED(dma_dev);
	ARG_UNUSED(channel);
	const struct device *dev = user_data;
	struct sdhc_mchp_hsmci_data *dev_data = dev->data;

	if (status != 0) {
		LOG_ERR("DMA transfer error: %d", status);
	}
	k_sem_give(&dev_data->dma_done);
}

/* Moves a whole block transfer via the XDMAC controller instead of polling
 * TXRDY/RXRDY per byte/word. The DMA peripheral-side address is the
 * HSMCI_FIFO memory aperture (datasheet section 39.8.5/39.8.6), used as a fixed,
 * non-incrementing address - not HSMCI_TDR/HSMCI_RDR.
 */
static int sdhc_mchp_hsmci_do_dma_transfer(const struct device *dev, hsmci_registers_t *const regs,
					   bool byte_mode, bool is_write, void *data,
					   int transfer_count, uint32_t timeout_ms)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	struct sdhc_mchp_hsmci_data *dev_data = dev->data;
	uint32_t data_size = byte_mode ? 1u : 4u;
	struct dma_config dma_cfg = {0u};
	struct dma_block_config dma_blk = {0u};
	int ret = 0;

	dma_cfg.dma_slot = dev_config->dma.perid;
	dma_cfg.source_data_size = data_size;
	dma_cfg.dest_data_size = data_size;
	/* burst_length is counted in data units, not bytes: 1 unit per handshake,
	 * matching HSMCI_DMA_CHKSIZE_1_Val below (dma_mchp_xdmac_g1.c derives
	 * XDMAC_CC_CSIZE from this as log2(burst_length), which must equal
	 * CHKSIZE's encoding per the datasheet). Do not set this to data_size.
	 */
	dma_cfg.source_burst_length = 1u;
	dma_cfg.dest_burst_length = 1u;
	dma_cfg.block_count = 1u;
	dma_cfg.complete_callback_en = true;
	dma_cfg.dma_callback = sdhc_mchp_hsmci_dma_callback;
	dma_cfg.user_data = (void *)dev;
	dma_cfg.head_block = &dma_blk;
	dma_blk.block_size = transfer_count * data_size;

	if (is_write) {
		dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
		dma_blk.source_address = (uint32_t)data;
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_blk.dest_address = (uint32_t)&regs->HSMCI_FIFO[0];
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	} else {
		dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
		dma_blk.source_address = (uint32_t)&regs->HSMCI_FIFO[0];
		dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_blk.dest_address = (uint32_t)data;
		dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	}

	ret = dma_config(dev_config->dma.dev, dev_config->dma.channel, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel: %d", ret);
		return -EIO;
	}

	k_sem_reset(&dev_data->dma_done);

	ret = dma_start(dev_config->dma.dev, dev_config->dma.channel);
	if (ret != 0) {
		LOG_ERR("Failed to start DMA channel: %d", ret);
		return -EIO;
	}

	ret = k_sem_take(&dev_data->dma_done, K_MSEC(timeout_ms));
	/* disable the handshake enabled in sdhc_mchp_hsmci_request_inner() */
	regs->HSMCI_DMA = 0u;

	if (ret != 0) {
		LOG_ERR("Timeout waiting for DMA transfer to complete");
		dma_stop(dev_config->dma.dev, dev_config->dma.channel);
		return -ETIMEDOUT;
	}

	return is_write ? sdhc_mchp_hsmci_wait_write_end(dev) : sdhc_mchp_hsmci_wait_read_end(dev);
}
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_DMA */

#if !defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
static int sdhc_mchp_hsmci_do_manual_transfer(const struct device *dev,
					      hsmci_registers_t *const regs, bool byte_mode,
					      bool is_write, void *data, int transfer_count)
{
	int ret = 0;

	if (is_write) {
		if (byte_mode) {
			const uint8_t *ptr = data;

			while (transfer_count-- > 0) {
				ret = sdhc_mchp_hsmci_wait_write_transfer_done(regs);
				if (ret != 0) {
					return ret;
				}
				regs->HSMCI_TDR = *ptr;
				ptr++;
			}
		} else {
			const uint32_t *ptr = data;

			while (transfer_count-- > 0) {
				ret = sdhc_mchp_hsmci_wait_write_transfer_done(regs);
				if (ret != 0) {
					return ret;
				}
				regs->HSMCI_TDR = *ptr;
				ptr++;
			}
		}
		ret = sdhc_mchp_hsmci_wait_write_end(dev);
	} else {
		if (byte_mode) {
			uint8_t *ptr = data;

			while (transfer_count-- > 0) {
				ret = sdhc_mchp_hsmci_wait_read_transfer_done(regs);
				if (ret != 0) {
					return ret;
				}
				*ptr = regs->HSMCI_RDR;
				ptr++;
			}
		} else {
			uint32_t *ptr = data;

			while (transfer_count-- > 0) {
				ret = sdhc_mchp_hsmci_wait_read_transfer_done(regs);
				if (ret != 0) {
					return ret;
				}
				*ptr = regs->HSMCI_RDR;
				ptr++;
			}
		}
		ret = sdhc_mchp_hsmci_wait_read_end(dev);
	}
	return ret;
}
#endif /* !CONFIG_SDHC_MCHP_HSMCI_G1_DMA */

static int sdhc_mchp_hsmci_request_inner(const struct device *dev, struct sdhc_command *cmd,
					 struct sdhc_data *sd_data)
{
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	struct sdhc_mchp_hsmci_data *dev_data = dev->data;
	hsmci_registers_t *const regs = dev_config->regs;
	uint32_t sr = 0u;
	uint32_t size = 0u;
	uint32_t transfer_count = 0u;
	uint32_t cmdr = 0u;
	bool is_write = false;
	bool byte_mode = false;
	int ret = 0;

	LOG_DBG("%s(opcode=%d, arg=%08x, data=%08x, rsptype=%d)", __func__, cmd->opcode, cmd->arg,
		(uint32_t)sd_data, cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);

	if (cmd->opcode == SD_GO_IDLE_STATE) {
		/* send 74 clocks, as required by SD spec */
		ret = sdhc_mchp_hsmci_send_clocks(dev);
		if (ret != 0) {
			return ret;
		}
	}

	if (sd_data) {
		cmdr |= HSMCI_CMDR_TRCMD_START_DATA;

		ret = sdhc_mchp_hsmci_write_timeout(dev, cmd->timeout_ms);
		if (ret != 0) {
			return ret;
		}

		switch (cmd->opcode) {
		case SD_WRITE_SINGLE_BLOCK:
			cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
			cmdr |= HSMCI_CMDR_TRDIR_WRITE;
			is_write = true;
			break;
		case SD_WRITE_MULTIPLE_BLOCK:
			is_write = true;
			cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
			cmdr |= HSMCI_CMDR_TRDIR_WRITE;
			break;
		case SD_APP_SEND_SCR:
		case SD_SWITCH:
		case SD_READ_SINGLE_BLOCK:
			is_write = false;
			cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
			cmdr |= HSMCI_CMDR_TRDIR_READ;
			break;
		case SD_READ_MULTIPLE_BLOCK:
			is_write = false;
			cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
			cmdr |= HSMCI_CMDR_TRDIR_READ;
			break;
		case SD_APP_SEND_NUM_WRITTEN_BLK:
			is_write = false;
			break;
		default:
			return -ENOTSUP;
		}

		if ((sd_data->block_size & 0x3) == 0 && (((uint32_t)sd_data->data) & 0x3) == 0) {
			size = (sd_data->block_size + 3) >> 2;
			regs->HSMCI_MR &= ~HSMCI_MR_FBYTE_Msk;
			byte_mode = false;
		} else {
			size = sd_data->block_size;
			regs->HSMCI_MR |= HSMCI_MR_FBYTE_Msk;
			byte_mode = true;
		}

		regs->HSMCI_BLKR =
			HSMCI_BLKR_BLKLEN(sd_data->block_size) | HSMCI_BLKR_BCNT(sd_data->blocks);

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
		regs->HSMCI_DMA = HSMCI_DMA_DMAEN_Msk | HSMCI_DMA_CHKSIZE(HSMCI_DMA_CHKSIZE_1_Val);
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_DMA */

		transfer_count = size * sd_data->blocks;
	}

	ret = sdhc_mchp_hsmci_send_cmd(dev, cmd, cmdr, dev_data);

	if (sd_data) {
		if (ret == 0) {
#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
			ret = sdhc_mchp_hsmci_do_dma_transfer(dev, regs, byte_mode, is_write,
							      sd_data->data, transfer_count,
							      cmd->timeout_ms);
#else
			ret = sdhc_mchp_hsmci_do_manual_transfer(dev, regs, byte_mode, is_write,
								 sd_data->data, transfer_count);
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_DMA */
		}
	}

	sr = regs->HSMCI_SR;

	LOG_DBG("RSP0=%08x, RPS1=%08x, RPS2=%08x,RSP3=%08x, SR=%08x", cmd->response[0],
		cmd->response[1], cmd->response[2], cmd->response[3], sr);

	return ret;
}

static void sdhc_mchp_hsmci_abort(const struct device *dev)
{
	struct sdhc_command cmd = {
		.opcode = SD_STOP_TRANSMISSION,
		.arg = 0,
		.response_type = SD_RSP_TYPE_NONE,
	};
	sdhc_mchp_hsmci_request_inner(dev, &cmd, NULL);
}

static int sdhc_mchp_hsmci_request(const struct device *dev, struct sdhc_command *cmd,
				   struct sdhc_data *sd_data)
{
	struct sdhc_mchp_hsmci_data *dev_data = dev->data;
	uint32_t sr = 0u;
#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE)
	const struct sdhc_mchp_hsmci_config *dev_config = dev->config;
	hsmci_registers_t *const regs = dev_config->regs;
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE */
	int ret = 0;

	ret = k_mutex_lock(&dev_data->mutex, K_MSEC(cmd->timeout_ms));
	if (ret != 0) {
		LOG_ERR("Could not access card");
		return -EBUSY;
	}

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE)
	regs->HSMCI_CR = HSMCI_CR_PWSDIS_Msk;
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE */

	do {
		ret = sdhc_mchp_hsmci_request_inner(dev, cmd, sd_data);
		if (sd_data && (ret != 0 || sd_data->blocks > 1)) {
			sr = 0u;

			sdhc_mchp_hsmci_abort(dev);
			ret = sdhc_mchp_hsmci_wait_irq(dev, HSMCI_SR_NOTBUSY_Msk,
						       HSMCI_DEFAULT_TIMEOUT_US, &sr);
			if (ret != 0) {
				LOG_DBG("Card did not idle after CMD12");
			}
		}
	} while (ret != 0 && (cmd->retries-- > 0));

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE)
	regs->HSMCI_CR = HSMCI_CR_PWSEN_Msk;
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_PWRSAVE */

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static DEVICE_API(sdhc, sdhc_mchp_hsmci_driver_api) = {
	.reset = sdhc_mchp_hsmci_reset,
	.get_host_props = sdhc_mchp_hsmci_get_host_props,
	.set_io = sdhc_mchp_hsmci_set_io,
	.get_card_present = sdhc_mchp_hsmci_get_card_present,
	.request = sdhc_mchp_hsmci_request,
	.card_busy = sdhc_mchp_hsmci_card_busy,
};

#if defined(CONFIG_SDHC_MCHP_HSMCI_G1_DMA)
#define SDHC_MCHP_HSMCI_DMA_CONFIG_INIT(n)                                                         \
	.dma = {                                                                                   \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rxtx)),                          \
		.channel = DT_INST_DMAS_CELL_BY_NAME(n, rxtx, channel),                            \
		.perid = DT_INST_DMAS_CELL_BY_NAME(n, rxtx, perid),                                \
	},
#else
#define SDHC_MCHP_HSMCI_DMA_CONFIG_INIT(n)
#endif /* CONFIG_SDHC_MCHP_HSMCI_G1_DMA */

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void sdhc_mchp_hsmci_irq_config_##n(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), sdhc_mchp_hsmci_isr,        \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct sdhc_mchp_hsmci_config mchp_hsmci_##n##_config = {                     \
		.regs = (hsmci_registers_t *)DT_INST_REG_ADDR(n),                                  \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),         \
		.carrier_detect = GPIO_DT_SPEC_INST_GET_OR(n, cd_gpios, {0}),                      \
		.irq_config = sdhc_mchp_hsmci_irq_config_##n,                                      \
		SDHC_MCHP_HSMCI_DMA_CONFIG_INIT(n)};                                               \
                                                                                                   \
	static struct sdhc_mchp_hsmci_data mchp_hsmci_##n##_data = {};                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &sdhc_mchp_hsmci_init, NULL, &mchp_hsmci_##n##_data,              \
			      &mchp_hsmci_##n##_config, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,    \
			      &sdhc_mchp_hsmci_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
