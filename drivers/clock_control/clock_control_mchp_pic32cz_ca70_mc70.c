/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip Power Management Controller (PMC) clock controller driver.
 *
 */

#define DT_DRV_COMPAT microchip_pic32cz_ca70_mc70_pmc

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <soc.h>
#include <zephyr/drivers/clock_control/mchp_clock_pic32cz_ca70_mc70.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(clock_control_mchp_pic32cz_ca70_mc70, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

/* timeout values in microseconds */
#define TIMEOUT_RC_RDY        1000000u
#define TIMEOUT_XOSC_RDY      1000000u
#define TIMEOUT_PLLA_LOCK_RDY 1000000u
#define TIMEOUT_UPLL_LOCK_RDY 1000000u
#define TIMEOUT_PCK_RDY       1000000u
#define TIMEOUT_MCLK_RDY      1000000u
#define TIMEOUT_REG_SYNC      1000u

#define SUPC_CR_PASSWD  SUPC_CR_KEY(0xA5u)
#define CKGR_MOR_PASSWD CKGR_MOR_KEY(0x37u)

#define SLOWRC_CLOCK_RATE   32000u
#define SLOWXOSC_CLOCK_RATE 32768u
#define UPLL_CLOCK_RATE     480000000u
#define USB_CLK_RATE        48000000u
#define USB_CLK_DIVIDER     9u

#define HOST_CLOCK_CONTROLLER_SOURCE_NUM    4u
#define HOST_CLOCK_CONTROLLER_PRESCALER_NUM 8u
#define HOST_CLOCK_CONTROLLER_DIVIDER_NUM   4u
#define MAIN_CLOCK_DIVIDERS_NUM             4u

/* Peripheral clocks before 7 are always enabled */
#define MIN_PERIPHERAL_CLOCK_ID       7u
/* The first 32 peripheral clocks are controlled by PCxR0 registers */
#define MAX_PCXR0_PERIPHERAL_CLOCK_ID 32u
/* peripheral clocks from 32 to 63 are controlled by PCxR1 registers */
#define MAX_PCXR1_PERIPHERAL_CLOCK_ID 64u
/* peripheral clocks from 64 to 127 are controlled by the PCR register */
#define MAX_PCR_PERIPHERAL_CLOCK_ID   128u
#define PROGRAMMABLE_CLOCKS_NUM       8u

#if !defined(ABS)
#define ABS(a, b) ((a) > (b) ? (a) - (b) : (b) - (a))
#endif

/* Clock subsystem definition.
 *
 * Structure which can be used as a sys argument in the clock_control API.
 * Encode clock type and instance number.
 *
 * - 00..15 (16 bits): id
 * - 16..31 (16 bits): type
 */
union clock_control_mchp_subsys {
	uint32_t val;
	struct {
		uint32_t id: 16;
		uint32_t type: 16;
	} bits;
};

struct clock_control_mchp_config {
	/* Register block for PMC peripheral */
	pmc_registers_t *regs;

	/* Register block for SUPC peripheral (used to configure slow clock) */
	supc_registers_t *supc_regs;

	/* Register block for UTMI peripheral (used to configure PLL) */
	utmi_registers_t *utmi_regs;

	/* Matrix register block */
	matrix_registers_t *matrix_regs;

	/* Frequency of the slow crystal oscillator */
	uint32_t slow_xosc_freq;

	/* Main crystal oscillator frequency */
	uint32_t main_xosc_freq_mhz;

	/* Main clock oscillator selection */
	enum clock_mchp_generator_main_oscillator clock_gen_main_osc;

	/* Slow clock oscillator selection */
	enum clock_mchp_generator_slow_oscillator clock_gen_slow_osc;

	/* Source for the main clock controller */
	enum clock_mchp_host_clock_controller_source mckr_source;

	/* Prescaler for the processor clock */
	enum clock_mchp_processor_clock_prescaler mckr_prescaler;

	/* Divider for the main clock */
	enum clock_mchp_main_clock_divider mckr_main_clock_divider;

	/* Number of cycles to wait for startup of the main crystal oscillator */
	uint8_t main_xosc_startup_cycles;

	/* Number of cycles to wait for startup of the UPLL */
	uint8_t upll_startup_cycles;

	/* Divisor to apply to the UPLL */
	uint8_t upll_div;

	/* Frequency of the main RC oscillator (for the MOSCRCF register) */
	uint8_t main_rc_freq_mhz;

	/* Multiplier to apply to the PLLA */
	uint8_t plla_multiplier;

	/* Divider to apply to the PLLA */
	uint8_t plla_divider;

	/* Number of slow clock cycles before the LOCKA bit is set */
	uint8_t plla_lock_count;

	/* Whether the UPLL is enabled */
	bool upll_enabled: 1;

	/* Whether the main crystal oscillator is enabled */
	bool main_xosc_enabled: 1;

	/* Whether to use PCK7 instead of PCK6 for the TC0 peripheral */
	bool tc0_pck7: 1;

	/* Whether to use GCLK instead of the peripheral clock for the I2SC0 peripheral */
	bool i2sc0_gclk: 1;

	/* Whether to use GCLK instead of the peripheral clock for the I2SC1 peripheral */
	bool i2sc1_gclk: 1;
};

struct clock_control_mchp_data {
	/* Rate of the host clock */
	uint32_t host_clk_rate;

	/* Rate of the main clock */
	uint32_t main_clk_rate;

	/* Rate of the PLLA */
	uint32_t plla_clk_rate;

	/* Rate of the slow clock */
	uint32_t slow_clk_rate;

	/* Rate of the UPLL */
	uint32_t upll_clk_rate;

	/* Currently applied source for the main clock controller */
	enum clock_mchp_host_clock_controller_source mckr_source;

	/* Currently applied prescaler for the processor clock */
	enum clock_mchp_processor_clock_prescaler mckr_prescaler;

	/* Currently applied divider for the main clock */
	enum clock_mchp_main_clock_divider mckr_main_clock_divider;
};

static int pck_get_pcr(const struct device *dev, uint32_t id, uint32_t *regval);
static int gclk_init(const struct device *dev, uint32_t id, uint8_t source, uint32_t divider);
static int pck_init(const struct device *dev, uint32_t id, uint8_t source, uint32_t prescaler);
static int configure_plla(const struct device *dev, uint8_t multiplier, uint8_t divider,
			  uint8_t lock_count, uint32_t main_clk_rate, uint32_t *plla_clock_rate);
static void configure_mckr(const struct device *dev,
			   enum clock_mchp_host_clock_controller_source source,
			   enum clock_mchp_processor_clock_prescaler prescaler,
			   enum clock_mchp_main_clock_divider main_clock_divider);
/*
 * Calculate the frequency of the GCLK
 */
static inline uint32_t gclk_calc_freq(uint32_t clkin_freq, uint32_t divider)
{
	return clkin_freq / (divider + 1);
}

/*
 * enable UPLL
 */
static int clock_control_mchp_upll_on(const struct clock_control_mchp_config *dev_config)
{
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t uckr = 0u;

	if (!dev_config->main_xosc_enabled) {
		LOG_ERR("UPLL requires the main crystal oscillator");
		return -EINVAL;
	}

	if (dev_config->main_xosc_freq_mhz != 12u && dev_config->main_xosc_freq_mhz != 16u) {
		LOG_ERR("UPLL Requires a source XOSC of either 12MHz or 16MHz, found %uMHz instead",
			dev_config->main_xosc_freq_mhz);
		return -EINVAL;
	}

	uckr = regs->CKGR_UCKR & ~CKGR_UCKR_UPLLCOUNT_Msk;
	uckr |= CKGR_UCKR_UPLLEN_Msk;
	uckr |= CKGR_UCKR_UPLLCOUNT(dev_config->upll_startup_cycles);
	regs->CKGR_UCKR = uckr;

	if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_LOCKU_Msk) != 0u), TIMEOUT_UPLL_LOCK_RDY,
		      (void)NULL)) {
		LOG_ERR("%s: UPLL ready timed out", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Enable the USB clock
 */
static int clock_control_mchp_usb_clk_on(const struct clock_control_mchp_config *dev_config)
{
	pmc_registers_t *const regs = dev_config->regs;

	regs->PMC_USB = PMC_USB_USBDIV(USB_CLK_DIVIDER) | PMC_USB_USBS_Msk;
	regs->PMC_SCER = PMC_SCER_USBCLK_Msk;

	return 0;
}

static int clock_control_mchp_on(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};
	uint32_t pcr = 0u;

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_PERIPHERAL:
		if (subsys.bits.id < MIN_PERIPHERAL_CLOCK_ID) {
			return -ENODEV;
		} else if (subsys.bits.id < MAX_PCXR0_PERIPHERAL_CLOCK_ID) {
			regs->PMC_PCER0 = BIT(subsys.bits.id);
		} else if (subsys.bits.id < MAX_PCXR1_PERIPHERAL_CLOCK_ID) {
			regs->PMC_PCER1 = BIT(subsys.bits.id & 0x1Fu);
		} else if (subsys.bits.id < MAX_PCR_PERIPHERAL_CLOCK_ID) {
			/* Select the peripheral clock for reading */
			regs->PMC_PCR = PMC_PCR_PID(subsys.bits.id);

			/* Read the current configuration */
			pcr = regs->PMC_PCR;

			/* Enable the clock (and set write mode) */
			pcr |= PMC_PCR_EN_Msk | PMC_PCR_CMD_Msk;

			/* Write back */
			regs->PMC_PCR = pcr;
		} else {
			return -ENODEV;
		}
		break;
	case SUBSYS_TYPE_PROGRAMMABLE:
		if (subsys.bits.id < PROGRAMMABLE_CLOCKS_NUM) {
			regs->PMC_SCER = BIT(subsys.bits.id + PMC_SCER_PCK0_Pos);

			if (!WAIT_FOR(((regs->PMC_SR & BIT(subsys.bits.id + PMC_SR_PCKRDY0_Pos)) !=
				       0u),
				      TIMEOUT_PCK_RDY, (void)NULL)) {
				LOG_ERR("%s: PCK timed out", __func__);
				return -ENODEV;
			}
		} else {
			return -ENODEV;
		}
		break;
	case SUBSYS_TYPE_UPLL:
		return clock_control_mchp_upll_on(dev_config);
	case SUBSYS_TYPE_USB_CLK:
		return clock_control_mchp_usb_clk_on(dev_config);
	case SUBSYS_TYPE_GCLK:
		if (subsys.bits.id < MAX_PCR_PERIPHERAL_CLOCK_ID) {
			/* Select the peripheral clock for reading */
			regs->PMC_PCR = PMC_PCR_PID(subsys.bits.id);

			/* Read the current configuration */
			pcr = regs->PMC_PCR;

			/* Enable the clock (and set write mode) */
			pcr |= PMC_PCR_GCLKEN_Msk | PMC_PCR_CMD_Msk | PMC_PCR_EN_Msk;

			/* Write back */
			regs->PMC_PCR = pcr;
		} else {
			return -ENODEV;
		}
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

static int clock_control_mchp_off(const struct device *dev, clock_control_subsys_t sys)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};
	uint32_t pcr = 0u;

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_PERIPHERAL:
		if (subsys.bits.id < MIN_PERIPHERAL_CLOCK_ID) {
			return -ENODEV;
		} else if (subsys.bits.id < MAX_PCXR0_PERIPHERAL_CLOCK_ID) {
			regs->PMC_PCDR0 = BIT(subsys.bits.id);
		} else if (subsys.bits.id < MAX_PCXR1_PERIPHERAL_CLOCK_ID) {
			regs->PMC_PCDR1 = BIT(subsys.bits.id & 0x1Fu);
		} else if (subsys.bits.id < MAX_PCR_PERIPHERAL_CLOCK_ID) {
			/* Select the peripheral clock for reading */
			regs->PMC_PCR = PMC_PCR_PID(subsys.bits.id);

			/* Read the current configuration */
			pcr = regs->PMC_PCR;

			/* Disable the clock (and set write mode) */
			pcr &= ~PMC_PCR_EN_Msk;
			pcr |= PMC_PCR_CMD_Msk;

			/* Write back */
			regs->PMC_PCR = pcr;
		} else {
			return -ENODEV;
		}
		break;
	case SUBSYS_TYPE_PROGRAMMABLE:
		if (subsys.bits.id < PROGRAMMABLE_CLOCKS_NUM) {
			regs->PMC_SCDR = BIT(subsys.bits.id + PMC_SCDR_PCK0_Pos);
		} else {
			return -ENODEV;
		}
		break;
	case SUBSYS_TYPE_UPLL:
		regs->CKGR_UCKR &= ~CKGR_UCKR_UPLLEN_Msk;
		break;
	case SUBSYS_TYPE_USB_CLK:
		regs->PMC_SCDR = PMC_SCER_USBCLK_Msk;
		break;
	case SUBSYS_TYPE_GCLK:
		if (subsys.bits.id < MAX_PCR_PERIPHERAL_CLOCK_ID) {
			/* Select the peripheral clock for reading */
			regs->PMC_PCR = PMC_PCR_PID(subsys.bits.id);

			/* Read the current configuration */
			pcr = regs->PMC_PCR;

			/* Disable the clock (and set write mode) */
			pcr &= ~(PMC_PCR_GCLKEN_Msk | PMC_PCR_EN_Msk);
			pcr |= PMC_PCR_CMD_Msk;

			/* Write back */
			regs->PMC_PCR = pcr;
		} else {
			return -ENODEV;
		}
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

/*
 * Retrieve the rate of a GCLK
 */
static int clock_control_mchp_get_rate_gclk(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t *rate)
{
	struct clock_control_mchp_data *dev_data = dev->data;
	uint32_t divider = 0u;
	uint32_t regval = 0u;
	uint32_t mux = 0u;
	uint32_t clkin_freq = 0u;
	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};

	if (pck_get_pcr(dev, subsys.bits.id, &regval) != 0) {
		return -ENODEV;
	}

	mux = (((regval) & (PMC_PCR_GCLKCSS_Msk)) >> PMC_PCR_GCLKCSS_Pos);
	divider = (((regval) & (PMC_PCR_GCLKDIV_Msk)) >> PMC_PCR_GCLKDIV_Pos);

	switch (mux) {
	case PMC_PCR_GCLKCSS_SLOW_CLK_Val:
		clkin_freq = dev_data->slow_clk_rate;
		break;
	case PMC_PCR_GCLKCSS_MCK_CLK_Val:
		clkin_freq = dev_data->host_clk_rate;
		break;
	case PMC_PCR_GCLKCSS_PLLA_CLK_Val:
		clkin_freq = dev_data->plla_clk_rate;
		break;
	case PMC_PCR_GCLKCSS_UPLL_CLK_Val:
		clkin_freq = dev_data->upll_clk_rate;
		break;
	case PMC_PCR_GCLKCSS_MAIN_CLK_Val:
		clkin_freq = dev_data->main_clk_rate;
		break;
	default:
		return -ENODEV;
	}

	*rate = gclk_calc_freq(clkin_freq, divider);

	return 0;
}

static int clock_control_mchp_get_rate(const struct device *dev, clock_control_subsys_t sys,
				       uint32_t *rate)
{
	struct clock_control_mchp_data *dev_data = dev->data;
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};
	uint16_t prescaler = 0u;

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_PERIPHERAL:
		*rate = dev_data->host_clk_rate;
		break;
	case SUBSYS_TYPE_SLOW_CLK:
		*rate = dev_data->slow_clk_rate;
		break;
	case SUBSYS_TYPE_MAIN_CLK:
		*rate = dev_data->main_clk_rate;
		break;
	case SUBSYS_TYPE_MCKR:
		*rate = dev_data->host_clk_rate;
		break;
	case SUBSYS_TYPE_PROGRAMMABLE:
		switch ((regs->PMC_PCK[subsys.bits.id] & PMC_PCK_CSS_Msk) >> PMC_PCK_CSS_Pos) {
		case PMC_PCK_CSS_SLOW_CLK_Val:
			*rate = dev_data->slow_clk_rate;
			break;
		case PMC_PCK_CSS_MAIN_CLK_Val:
			*rate = dev_data->main_clk_rate;
			break;
		case PMC_PCK_CSS_PLLA_CLK_Val:
			*rate = dev_data->plla_clk_rate;
			break;
		case PMC_PCK_CSS_UPLL_CLK_Val:
			if ((regs->PMC_MCKR & PMC_MCKR_UPLLDIV2_Msk) != 0u) {
				*rate = UPLL_CLOCK_RATE / 2u;
			} else {
				*rate = UPLL_CLOCK_RATE;
			}
			break;
		case PMC_PCK_CSS_MCK_Val:
			*rate = dev_data->host_clk_rate;
			break;
		}

		/* Apply prescaler */
		prescaler = (regs->PMC_PCK[subsys.bits.id] & PMC_PCK_PRES_Msk) >> PMC_PCK_PRES_Pos;
		*rate /= (prescaler + 1u);

		break;
	case SUBSYS_TYPE_UPLL:
		*rate = dev_data->upll_clk_rate;
		break;
	case SUBSYS_TYPE_USB_CLK:
		*rate = USB_CLK_RATE;
		break;
	case SUBSYS_TYPE_GCLK:
		return clock_control_mchp_get_rate_gclk(dev, sys, rate);
	case SUBSYS_TYPE_PLLA:
		*rate = dev_data->plla_clk_rate;
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

#if defined(CONFIG_CLOCK_CONTROL_MCHP_SET_RATE)
/*
 * Calculate the divider to be used by set_rate in order to achieve a certain
 * frequency for a programmable clock
 */
static inline uint32_t pck_calc_divider(uint32_t clkin_freq, uint32_t clkout_freq)
{
	uint32_t divider = clkin_freq / clkout_freq;

	if (ABS((clkin_freq / divider), clkout_freq) >
	    ABS((clkin_freq / (divider + 1u)), clkout_freq)) {
		divider = divider + 1u;
	}

	return divider;
}

/*
 * Attempt to set the rate for a PCK
 */
static int clock_control_mchp_set_rate_pck(const struct device *dev, clock_control_subsys_t sys,
					   uint32_t rate)
{
	struct clock_control_mchp_data *dev_data = dev->data;
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t clkin_freq = 0u;
	uint32_t clkout_freq = rate;
	uint32_t divider = 0u;
	uint32_t regval = 0u;
	uint32_t mux = 0u;
	/* PMC_PCK_PRES is a 0-based field; the divider computed below is the 1-based
	 * divide-by-N value expected by pck_init().
	 */
	const uint32_t max = ((PMC_PCK_PRES_Msk) >> PMC_PCK_PRES_Pos) + 1u;

	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};

	if (clkout_freq == 0u) {
		return -EINVAL;
	}

	if (subsys.bits.id >= PROGRAMMABLE_CLOCKS_NUM) {
		return -EINVAL;
	}

	regval = regs->PMC_PCK[subsys.bits.id];

	mux = (((regval) & (PMC_PCK_CSS_Msk)) >> PMC_PCK_CSS_Pos);
	switch (mux) {
	case PMC_PCK_CSS_SLOW_CLK_Val:
		clkin_freq = dev_data->slow_clk_rate;
		break;
	case PMC_PCK_CSS_MCK_Val:
		clkin_freq = dev_data->host_clk_rate;
		break;
	case PMC_PCK_CSS_PLLA_CLK_Val:
		clkin_freq = dev_data->plla_clk_rate;
		break;
	case PMC_PCK_CSS_UPLL_CLK_Val:
		clkin_freq = dev_data->upll_clk_rate;
		break;
	case PMC_PCK_CSS_MAIN_CLK_Val:
		clkin_freq = dev_data->main_clk_rate;
		break;
	default:
		return -ENODEV;
	}

	/* if clkout_freq > clkin_freq then clkin_freq / clkout_freq truncates to 0 */
	if (clkout_freq > clkin_freq) {
		return -EINVAL;
	}

	divider = pck_calc_divider(clkin_freq, clkout_freq);
	if ((divider < 1u) || (divider > max)) {
		return -EINVAL;
	}
	pck_init(dev, subsys.bits.id, mux, divider);

	return 0;
}

/*
 * Attempt to set the rate for a GCLK
 */
static int clock_control_mchp_set_rate_gclk(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t rate)
{
	struct clock_control_mchp_data *dev_data = dev->data;
	uint32_t clkin_freq = 0u;
	uint32_t clkout_freq = rate;
	uint32_t divider = 0u;
	uint32_t regval = 0u;
	uint32_t mux = 0u;
	/* PMC_PCR_GCLKDIV is a 0-based field; the divider computed below is the 1-based
	 * divide-by-N value expected by gclk_init().
	 */
	const uint32_t max = ((PMC_PCR_GCLKDIV_Msk) >> PMC_PCR_GCLKDIV_Pos) + 1u;

	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};

	if (clkout_freq == 0u) {
		return -EINVAL;
	}

	if (pck_get_pcr(dev, subsys.bits.id, &regval) != 0) {
		return -ENODEV;
	}

	mux = (((regval) & (PMC_PCR_GCLKCSS_Msk)) >> PMC_PCR_GCLKCSS_Pos);
	switch (mux) {
	case PMC_PCR_GCLKCSS_SLOW_CLK_Val:
		clkin_freq = dev_data->slow_clk_rate;
		break;
	case PMC_PCR_GCLKCSS_MCK_CLK_Val:
		clkin_freq = dev_data->host_clk_rate;
		break;
	case PMC_PCR_GCLKCSS_PLLA_CLK_Val:
		clkin_freq = dev_data->plla_clk_rate;
		break;
	case PMC_PCR_GCLKCSS_UPLL_CLK_Val:
		clkin_freq = dev_data->upll_clk_rate;
		break;
	case PMC_PCR_GCLKCSS_MAIN_CLK_Val:
		clkin_freq = dev_data->main_clk_rate;
		break;
	default:
		return -ENODEV;
	}

	/* if clkout_freq > clkin_freq then clkin_freq / clkout_freq truncates to 0 */
	if (clkout_freq > clkin_freq) {
		return -EINVAL;
	}

	divider = pck_calc_divider(clkin_freq, clkout_freq);
	if ((divider < 1u) || (divider > max)) {
		return -EINVAL;
	}
	gclk_init(dev, subsys.bits.id, mux, divider);

	return 0;
}

/*
 * Attempt to set a rate for the PLLA
 */
static int clock_control_mchp_set_rate_plla(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t rate)
{
	struct clock_control_mchp_data *dev_data = dev->data;
	const struct clock_control_mchp_config *dev_config = dev->config;
	const uint32_t max_multiplier = ((CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos) + 1u;
	const uint32_t max_divider = ((CKGR_PLLAR_DIVA_Msk) >> CKGR_PLLAR_DIVA_Pos) + 1u;
	uint32_t clkout_freq = 0u;

	if (rate == 0u) {
		return -EINVAL;
	}

	/* iterate to find correct divider and multiplier */
	for (uint32_t divider = 1u; divider < max_divider; divider++) {
		for (uint32_t multiplier = 1u; multiplier < max_multiplier; multiplier++) {
			clkout_freq = (dev_data->main_clk_rate / divider) * (multiplier + 1u);

			if (clkout_freq == rate) {
				return configure_plla(
					dev, multiplier, divider, dev_config->plla_lock_count,
					dev_data->main_clk_rate, &dev_data->plla_clk_rate);
			}
		}
	}

	/* The required output frequency is not supported */
	return -ENOTSUP;
}

/*
 * Attempt to set a rate for the MCKR
 */
static int clock_control_mchp_set_rate_mckr(const struct device *dev, clock_control_subsys_t sys,
					    uint32_t rate)
{
	struct clock_control_mchp_data *dev_data = dev->data;
	const uint32_t clkin_freqs[HOST_CLOCK_CONTROLLER_SOURCE_NUM] = {
		[CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_SLOW_CK] = dev_data->slow_clk_rate,
		[CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_MAIN_CLK] = dev_data->main_clk_rate,
		[CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_PLLA_CK] = dev_data->plla_clk_rate,
		[CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_UPLL_CK] = dev_data->upll_clk_rate,
	};
	const uint32_t prescalers[HOST_CLOCK_CONTROLLER_PRESCALER_NUM] = {
		1u, 2u, 4u, 8u, 16u, 32u, 64u, 3u,
	};
	const uint32_t dividers[HOST_CLOCK_CONTROLLER_DIVIDER_NUM] = {1u, 2u, 4u, 3u};
	uint32_t clkout_freq = 0u;
	uint32_t clkin_freq = 0u;
	uint32_t prescaler = 0u;
	uint32_t divider = 0u;

	if (rate == 0u) {
		return -EINVAL;
	}

	for (size_t source_idx = 0u; source_idx < ARRAY_SIZE(clkin_freqs); source_idx++) {
		clkin_freq = clkin_freqs[source_idx];

		/* iterate to find correct divider and multiplier */
		for (size_t prescaler_idx = 0u; prescaler_idx < ARRAY_SIZE(prescalers);
		     prescaler_idx++) {
			prescaler = prescalers[prescaler_idx];

			for (size_t divider_idx = 0u; divider_idx < ARRAY_SIZE(dividers);
			     divider_idx++) {
				divider = dividers[divider_idx];

				clkout_freq = (clkin_freq / prescaler) / divider;

				if (clkout_freq == rate) {
					configure_mckr(dev, source_idx, prescaler_idx, divider_idx);
					return 0;
				}
			}
		}
	}

	/* The required output frequency is not supported */
	return -ENOTSUP;
}

static int clock_control_mchp_set_rate(const struct device *dev, clock_control_subsys_t sys,
				       clock_control_subsys_rate_t rate)
{
	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_PROGRAMMABLE:
		return clock_control_mchp_set_rate_pck(dev, sys, (uint32_t)rate);
	case SUBSYS_TYPE_GCLK:
		return clock_control_mchp_set_rate_gclk(dev, sys, (uint32_t)rate);
	case SUBSYS_TYPE_PLLA:
		return clock_control_mchp_set_rate_plla(dev, sys, (uint32_t)rate);
	case SUBSYS_TYPE_MCKR:
		return clock_control_mchp_set_rate_mckr(dev, sys, (uint32_t)rate);
	default:
		return -ENODEV;
	}

	return -ENODEV;
}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_SET_RATE */

static enum clock_control_status clock_control_mchp_get_status(const struct device *dev,
							       clock_control_subsys_t sys)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};
	enum clock_control_status status = CLOCK_CONTROL_STATUS_OFF;
	uint32_t pcr = 0u;

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_PERIPHERAL:
		if (subsys.bits.id < MIN_PERIPHERAL_CLOCK_ID) {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		} else if (subsys.bits.id < MAX_PCXR0_PERIPHERAL_CLOCK_ID) {
			if ((regs->PMC_PCSR0 & BIT(subsys.bits.id)) != 0u) {
				status = CLOCK_CONTROL_STATUS_ON;
			}
		} else if (subsys.bits.id < MAX_PCXR1_PERIPHERAL_CLOCK_ID) {
			if ((regs->PMC_PCSR1 & BIT(subsys.bits.id & 0x1Fu)) != 0u) {
				status = CLOCK_CONTROL_STATUS_ON;
			}
		} else if (subsys.bits.id < MAX_PCR_PERIPHERAL_CLOCK_ID) {
			/* Select the peripheral clock for reading */
			regs->PMC_PCR = PMC_PCR_PID(subsys.bits.id);

			/* Read the current configuration */
			pcr = regs->PMC_PCR;

			if ((pcr & PMC_PCR_EN_Msk) != 0u) {
				status = CLOCK_CONTROL_STATUS_ON;
			} else {
				status = CLOCK_CONTROL_STATUS_OFF;
			}
		} else {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}
		break;
	case SUBSYS_TYPE_PROGRAMMABLE:
		if (subsys.bits.id < PROGRAMMABLE_CLOCKS_NUM) {
			if ((regs->PMC_SCSR & BIT(subsys.bits.id + PMC_SCSR_PCK0_Pos)) != 0u) {
				status = CLOCK_CONTROL_STATUS_ON;
			}
		} else {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}
		break;
	case SUBSYS_TYPE_GCLK:
		if (subsys.bits.id < MAX_PCR_PERIPHERAL_CLOCK_ID) {
			/* Select the peripheral clock for reading */
			regs->PMC_PCR = PMC_PCR_PID(subsys.bits.id);

			/* Read the current configuration */
			pcr = regs->PMC_PCR;

			if ((pcr & PMC_PCR_GCLKEN_Msk) != 0u) {
				status = CLOCK_CONTROL_STATUS_ON;
			} else {
				status = CLOCK_CONTROL_STATUS_OFF;
			}
		} else {
			return CLOCK_CONTROL_STATUS_UNKNOWN;
		}
		break;
	case SUBSYS_TYPE_UPLL:
		if ((regs->CKGR_UCKR & CKGR_UCKR_UPLLEN_Msk) != 0u) {
			status = CLOCK_CONTROL_STATUS_ON;
		} else {
			status = CLOCK_CONTROL_STATUS_OFF;
		}
		break;
	case SUBSYS_TYPE_USB_CLK:
		if ((regs->PMC_SCSR & PMC_SCSR_USBCLK_Msk) != 0u) {
			status = CLOCK_CONTROL_STATUS_ON;
		} else {
			status = CLOCK_CONTROL_STATUS_OFF;
		}
		break;
	default:
		status = CLOCK_CONTROL_STATUS_UNKNOWN;
		break;
	}

	return status;
}

/*
 * Select the main RC oscillator as MAIN_CLK source
 */
static uint32_t configure_main_rc_as_main_clk_source(const struct device *dev)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t mor = 0u;

	/* Enable Fast RC oscillator but DO NOT switch to RC now */
	regs->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCRCEN_Msk;

	/* Wait for the Fast RC to stabilize after having enabled it */
	if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_MOSCRCS_Msk) != 0u), TIMEOUT_RC_RDY, (void)NULL)) {
		LOG_ERR("%s: Fast RC oscillator selection timed out", __func__);
	}

	mor = regs->CKGR_MOR;
	mor &= ~CKGR_MOR_MOSCRCF_Msk;
	mor |= CKGR_MOR_MOSCRCF(dev_config->main_rc_freq_mhz);
	mor |= CKGR_MOR_PASSWD;
	regs->CKGR_MOR = mor;

	/* Wait for the Fast RC to stabilize after having changed the frequency */
	if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_MOSCRCS_Msk) != 0u), TIMEOUT_RC_RDY, (void)NULL)) {
		LOG_ERR("%s: Fast RC oscillator selection timed out", __func__);
	}

	/* Select RC as source for mainck */
	mor = regs->CKGR_MOR;
	mor &= ~CKGR_MOR_MOSCSEL_Msk;
	mor |= CKGR_MOR_PASSWD;
	regs->CKGR_MOR = mor;

	return (dev_config->main_rc_freq_mhz + 1u) * 4000000u;
}

/*
 * Select the slow XOSC as SLOWCK source
 */
static void configure_slow_xosc_as_slowck_source(const struct device *dev)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	supc_registers_t *supc_regs = dev_config->supc_regs;

	supc_regs->SUPC_CR = SUPC_CR_PASSWD | SUPC_CR_XTALSEL_Msk;
	/* Wait for the crystal oscillator to be ready */
	if (!WAIT_FOR(((supc_regs->SUPC_SR & SUPC_SR_OSCSEL_Msk) != 0u), TIMEOUT_XOSC_RDY,
		      (void)NULL)) {
		LOG_ERR("%s: slow oscillator selection timed out", __func__);
	}
}

/*
 * Verify the clock rate of the main oscillator
 */
static uint32_t verify_clock_rate(const struct device *dev)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t clock_rate = 0u;
	uint32_t mainf = 0u;

	/* Set the measurement on the main crystal oscillator */
	regs->CKGR_MCFR = CKGR_MCFR_CCSS_Msk;

	/* Restart the measurement */
	regs->CKGR_MCFR = CKGR_MCFR_CCSS_Msk | CKGR_MCFR_RCMEAS_Msk;

	/* MAINFRDY is synchronized to SLOW_CLK and may still report the previous measurement, wait
	 * for it
	 */
	(void)WAIT_FOR(((regs->CKGR_MCFR & CKGR_MCFR_MAINFRDY_Msk) == 0u), TIMEOUT_REG_SYNC,
		       (void)NULL);

	/* Wait for the output to be ready */
	if (!WAIT_FOR(((regs->CKGR_MCFR & CKGR_MCFR_MAINFRDY_Msk) != 0u), TIMEOUT_XOSC_RDY,
		      (void)NULL)) {
		LOG_ERR("%s: main oscillator selection timed out", __func__);
		clock_rate = configure_main_rc_as_main_clk_source(dev);
	} else {
		/* Verify the resulting frequency; `MAINF` must not be 0 */
		mainf = (regs->CKGR_MCFR & CKGR_MCFR_MAINF_Msk) >> CKGR_MCFR_MAINF_Pos;

		if (mainf == 0u) {
			LOG_ERR("%s: main oscillator failure", __func__);
			clock_rate = configure_main_rc_as_main_clk_source(dev);
		} else {
			clock_rate = dev_config->main_xosc_freq_mhz * 1000000u;
		}
	}

	return clock_rate;
}

/*
 * Select the main oscillator that drives MAIN_CLK. Returns the resulting MAIN_CLK rate.
 */
static uint32_t configure_main_oscillator(const struct device *dev,
					  enum clock_mchp_generator_main_oscillator source)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t main_clk_rate = 0u;
	uint32_t mor = 0u;

	switch (source) {
	case CLOCK_MCHP_GENERATOR_MAIN_OSCILLATOR_RC:
		main_clk_rate = configure_main_rc_as_main_clk_source(dev);
		break;
	case CLOCK_MCHP_GENERATOR_MAIN_OSCILLATOR_XOSC:
		mor = regs->CKGR_MOR;

		mor |= CKGR_MOR_MOSCSEL_Msk;
		mor |= CKGR_MOR_PASSWD;
		regs->CKGR_MOR = mor;

		/* Wait for clock selection to be complete */
		if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_MOSCSELS_Msk) != 0u), TIMEOUT_REG_SYNC,
			      (void)NULL)) {
			LOG_ERR("%s: main oscillator selection timed out", __func__);
			main_clk_rate = configure_main_rc_as_main_clk_source(dev);
			break;
		}

		main_clk_rate = verify_clock_rate(dev);
		break;
	case CLOCK_MCHP_GENERATOR_MAIN_OSCILLATOR_XOSC_BYPASS:
		mor = regs->CKGR_MOR;

		/* Enable the bypass first, then disable the main crystal oscillator */
		mor |= CKGR_MOR_MOSCXTBY_Msk;
		mor |= CKGR_MOR_PASSWD;
		regs->CKGR_MOR = mor;

		mor = regs->CKGR_MOR;
		mor &= ~CKGR_MOR_MOSCXTEN_Msk;
		mor |= CKGR_MOR_PASSWD;
		regs->CKGR_MOR = mor;

		/* Select the external clock on XIN as source for mainck */
		mor = regs->CKGR_MOR;
		mor |= CKGR_MOR_MOSCSEL_Msk;
		mor |= CKGR_MOR_PASSWD;
		regs->CKGR_MOR = mor;

		/* Wait for clock selection to be complete */
		if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_MOSCSELS_Msk) != 0u), TIMEOUT_REG_SYNC,
			      (void)NULL)) {
			LOG_ERR("%s: main oscillator selection timed out", __func__);
			main_clk_rate = configure_main_rc_as_main_clk_source(dev);
			break;
		}

		main_clk_rate = verify_clock_rate(dev);
		break;
	default:
		LOG_ERR("Invalid source to mainck: %u", source);
		break;
	}

	return main_clk_rate;
}

/*
 * Select the oscillator that drives SLOW_CLK. Returns the resulting SLOW_CLK rate.
 */
static uint32_t configure_slow_oscillator(const struct device *dev,
					  enum clock_mchp_generator_slow_oscillator source)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	supc_registers_t *supc_regs = dev_config->supc_regs;
	uint32_t slow_clock_rate = 0u;
	uint32_t supc = 0u;

	switch (source) {
	case CLOCK_MCHP_GENERATOR_SLOW_OSCILLATOR_RC:
		slow_clock_rate = SLOWRC_CLOCK_RATE;
		break;
	case CLOCK_MCHP_GENERATOR_SLOW_OSCILLATOR_XOSC:
		supc = supc_regs->SUPC_MR & ~(SUPC_MR_OSCBYPASS_Msk | SUPC_MR_KEY_Msk);

		supc |= SUPC_MR_KEY_PASSWD;
		supc_regs->SUPC_MR = supc;

		configure_slow_xosc_as_slowck_source(dev);
		slow_clock_rate = SLOWXOSC_CLOCK_RATE;
		break;
	case CLOCK_MCHP_GENERATOR_SLOW_OSCILLATOR_XOSC_BYPASS:
		supc = supc_regs->SUPC_MR & ~(SUPC_MR_OSCBYPASS_Msk | SUPC_MR_KEY_Msk);

		supc |= SUPC_MR_OSCBYPASS_Msk;
		supc |= SUPC_MR_KEY_PASSWD;
		supc_regs->SUPC_MR = supc;

		configure_slow_xosc_as_slowck_source(dev);
		slow_clock_rate = dev_config->slow_xosc_freq;
		break;
	}

	return slow_clock_rate;
}

/*
 * Program the PLLA multiplier/divider/lock count. A divider or multiplier of 0 leaves the
 * PLLA unconfigured (matching its behavior after reset).
 */
static int configure_plla(const struct device *dev, uint8_t multiplier, uint8_t divider,
			  uint8_t lock_count, uint32_t main_clk_rate, uint32_t *plla_clock_rate)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t pllar = 0u;

	*plla_clock_rate = 0u;

	if ((divider == 0u) || (multiplier == 0u)) {
		return 0;
	}

	pllar = regs->CKGR_PLLAR &
		~(CKGR_PLLAR_MULA_Msk | CKGR_PLLAR_PLLACOUNT_Msk | CKGR_PLLAR_DIVA_Msk);

	/* Set `ONE` to allow pllar configuration */
	pllar |= CKGR_PLLAR_ONE_Msk;
	pllar |= CKGR_PLLAR_MULA(multiplier);
	pllar |= CKGR_PLLAR_PLLACOUNT(lock_count);
	pllar |= CKGR_PLLAR_DIVA(divider);
	regs->CKGR_PLLAR = pllar;

	*plla_clock_rate = (main_clk_rate * (multiplier + 1u)) / divider;

	/* Wait for the pll to be ready */
	if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_LOCKA_Msk) != 0u), TIMEOUT_PLLA_LOCK_RDY,
		      (void)NULL)) {
		LOG_ERR("%s: PLLA lock timed out", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Program the UPLL post-divider (PMC_MCKR.UPLLDIV2) and cache the resulting UPLL rate.
 */
static void configure_upll_div(const struct device *dev, bool div2_en)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	struct clock_control_mchp_data *dev_data = dev->data;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t mckr = 0u;

	mckr = regs->PMC_MCKR & ~PMC_MCKR_UPLLDIV2_Msk;
	mckr |= PMC_MCKR_UPLLDIV2(div2_en ? 1u : 0u);
	regs->PMC_MCKR = mckr;

	dev_data->upll_clk_rate = div2_en ? (UPLL_CLOCK_RATE / 2u) : UPLL_CLOCK_RATE;
}

/*
 * Select the provided MCKR source
 */
static void configure_mckr_source(const struct device *dev,
				  enum clock_mchp_host_clock_controller_source source)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t mckr = regs->PMC_MCKR & ~(PMC_MCKR_CSS_Msk);

	mckr |= PMC_MCKR_CSS(source);
	regs->PMC_MCKR = mckr;

	if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_MCKRDY_Msk) != 0u), TIMEOUT_MCLK_RDY, (void)NULL)) {
		LOG_ERR("%s: MCKRDY timed out", __func__);
	}
}

/*
 * Select the provided MCKR divider
 */
static void configure_mckr_divider(const struct device *dev, uint8_t divider)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t mckr = regs->PMC_MCKR & ~(PMC_MCKR_MDIV_Msk);

	mckr |= PMC_MCKR_MDIV(divider);
	regs->PMC_MCKR = mckr;

	if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_MCKRDY_Msk) != 0u), TIMEOUT_MCLK_RDY, (void)NULL)) {
		LOG_ERR("%s: MCKRDY timed out", __func__);
	}
}

/*
 * Select the provided MCKR prescaler
 */
static void configure_mckr_prescaler(const struct device *dev, uint8_t prescaler)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t mckr = regs->PMC_MCKR & ~(PMC_MCKR_PRES_Msk);

	mckr |= PMC_MCKR_PRES(prescaler);
	regs->PMC_MCKR = mckr;

	if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_MCKRDY_Msk) != 0u), TIMEOUT_MCLK_RDY, (void)NULL)) {
		LOG_ERR("%s: MCKRDY timed out", __func__);
	}
}

/*
 * Recompute the cached host_clk_rate from the currently applied MCKR source/prescaler/divider
 * and the currently cached source rates. Must be called after any of those change.
 */
static void recompute_host_clk_rate(const struct device *dev)
{
	struct clock_control_mchp_data *dev_data = dev->data;
	static const uint8_t actual_main_clock_dividers[MAIN_CLOCK_DIVIDERS_NUM] = {1u, 2u, 4u, 3u};
	static const uint8_t actual_prescalers[HOST_CLOCK_CONTROLLER_PRESCALER_NUM] = {
		1u, 2u, 4u, 8u, 16u, 32u, 64u, 3u,
	};
	uint32_t source_rate = 0u;

	switch (dev_data->mckr_source) {
	case CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_SLOW_CK:
		source_rate = dev_data->slow_clk_rate;
		break;
	case CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_MAIN_CLK:
		source_rate = dev_data->main_clk_rate;
		break;
	case CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_PLLA_CK:
		source_rate = dev_data->plla_clk_rate;
		break;
	case CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_UPLL_CK:
		source_rate = dev_data->upll_clk_rate;
		break;
	default:
		source_rate = 0u;
		break;
	}

	dev_data->host_clk_rate = (source_rate / actual_prescalers[dev_data->mckr_prescaler]) /
				  actual_main_clock_dividers[dev_data->mckr_main_clock_divider];
}

/*
 * Configure the Host Clock Controller (MCKR) source/prescaler/divider, and refresh the cached
 * host_clk_rate.
 */
static void configure_mckr(const struct device *dev,
			   enum clock_mchp_host_clock_controller_source source,
			   enum clock_mchp_processor_clock_prescaler prescaler,
			   enum clock_mchp_main_clock_divider main_clock_divider)
{
	struct clock_control_mchp_data *dev_data = dev->data;

	if ((source == CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_SLOW_CK) ||
	    (source == CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_MAIN_CLK)) {
		configure_mckr_source(dev, source);
		configure_mckr_prescaler(dev, prescaler);
		configure_mckr_divider(dev, main_clock_divider);
	} else {
		configure_mckr_prescaler(dev, prescaler);
		configure_mckr_divider(dev, main_clock_divider);
		configure_mckr_source(dev, source);
	}

	dev_data->mckr_source = source;
	dev_data->mckr_prescaler = prescaler;
	dev_data->mckr_main_clock_divider = main_clock_divider;

	recompute_host_clk_rate(dev);
}

/*
 * initialize programmable clock from device tree node.
 */
static int pck_init(const struct device *dev, uint32_t id, uint8_t source, uint32_t prescaler)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t pckn = 0u;

	if (id >= PROGRAMMABLE_CLOCKS_NUM) {
		return -EINVAL;
	}

	pckn = regs->PMC_PCK[id] & ~PMC_PCK_PRES_Msk & ~PMC_PCK_CSS_Msk;

	pckn |= PMC_PCK_PRES(prescaler - 1u);
	pckn |= PMC_PCK_CSS(source);
	regs->PMC_PCK[id] = pckn;

	return 0;
}

#define CLOCK_MCHP_ITERATE_PCK(child)                                                              \
	{                                                                                          \
		pck_init(dev, DT_REG_ADDR_BY_IDX(child, 0), DT_ENUM_IDX(child, source),            \
			 DT_PROP(child, prescaler));                                               \
	}

/*
 * Retrieve the PCR value for a certain programmable clock ID
 */
static int pck_get_pcr(const struct device *dev, uint32_t id, uint32_t *regval)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	int ret = -ENODEV;

	if (id < MAX_PCR_PERIPHERAL_CLOCK_ID) {
		regs->PMC_PCR = PMC_PCR_PID(id);
		*regval = regs->PMC_PCR;
		ret = 0;
	}

	return ret;
}

/*
 * Initialize generic clock from device tree node.
 */
static int gclk_init(const struct device *dev, uint32_t id, uint8_t source, uint32_t divider)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	uint32_t pcr = 0u;

	if (id >= MAX_PCR_PERIPHERAL_CLOCK_ID) {
		return -EINVAL;
	}

	/* Select the peripheral clock for reading */
	regs->PMC_PCR = PMC_PCR_PID(id);

	/* Read the current configuration */
	pcr = regs->PMC_PCR;

	pcr &= ~(PMC_PCR_GCLKCSS_Msk | PMC_PCR_GCLKDIV_Msk);
	pcr |= PMC_PCR_CMD_Msk | PMC_PCR_GCLKCSS(source) | PMC_PCR_GCLKDIV(divider - 1u) |
	       PMC_PCR_GCLKEN_Msk;

	/* Write back */
	regs->PMC_PCR = pcr;

	return 0;
}

#define CLOCK_MCHP_ITERATE_GCLK(child)                                                             \
	{                                                                                          \
		gclk_init(dev, DT_REG_ADDR_BY_IDX(child, 0), DT_ENUM_IDX(child, source),           \
			  DT_PROP(child, divider));                                                \
	}

#if defined(CONFIG_CLOCK_CONTROL_MCHP_CONFIG_RUNTIME)
static int clock_control_mchp_configure(const struct device *dev, clock_control_subsys_t sys,
					void *req_config)
{
	struct clock_control_mchp_data *dev_data = dev->data;
	union clock_control_mchp_subsys subsys = {.val = (uint32_t)sys};
	const enum clock_mchp_generator_slow_oscillator *slow_oscillator = NULL;
	const enum clock_mchp_generator_main_oscillator *main_oscillator = NULL;
	struct clock_mchp_subsys_plla_config *plla_cfg = NULL;
	struct clock_mchp_subsys_mckr_config *mckr_cfg = NULL;
	struct clock_mchp_subsys_upll_config *upll_cfg = NULL;
	struct clock_mchp_subsys_gclk_config *gclk_cfg = NULL;
	struct clock_mchp_subsys_pck_config *pck_cfg = NULL;
	int ret = 0;

	if (req_config == NULL) {
		return -EINVAL;
	}

	switch (subsys.bits.type) {
	case SUBSYS_TYPE_SLOW_CLK:
		slow_oscillator = req_config;

		dev_data->slow_clk_rate = configure_slow_oscillator(dev, *slow_oscillator);
		recompute_host_clk_rate(dev);
		break;
	case SUBSYS_TYPE_MAIN_CLK:
		main_oscillator = req_config;

		dev_data->main_clk_rate = configure_main_oscillator(dev, *main_oscillator);
		recompute_host_clk_rate(dev);
		break;
	case SUBSYS_TYPE_PLLA:
		plla_cfg = req_config;

		ret = configure_plla(dev, plla_cfg->multiplier, plla_cfg->divider,
				     plla_cfg->lock_count, dev_data->main_clk_rate,
				     &dev_data->plla_clk_rate);
		if (ret != 0) {
			return ret;
		}
		recompute_host_clk_rate(dev);
		break;
	case SUBSYS_TYPE_MCKR:
		mckr_cfg = req_config;

		configure_mckr(dev, mckr_cfg->source, mckr_cfg->prescaler, mckr_cfg->divider);
		break;
	case SUBSYS_TYPE_UPLL:
		upll_cfg = req_config;

		configure_upll_div(dev, upll_cfg->div2_en);
		recompute_host_clk_rate(dev);
		break;
	case SUBSYS_TYPE_GCLK:
		gclk_cfg = req_config;

		if ((gclk_cfg->divider < 1u) || (gclk_cfg->divider > 256u)) {
			return -EINVAL;
		}
		return gclk_init(dev, subsys.bits.id, (uint8_t)gclk_cfg->source, gclk_cfg->divider);
	case SUBSYS_TYPE_PROGRAMMABLE:
		pck_cfg = req_config;

		if ((pck_cfg->prescaler < 1u) || (pck_cfg->prescaler > 256u)) {
			return -EINVAL;
		}
		return pck_init(dev, subsys.bits.id, (uint8_t)pck_cfg->source, pck_cfg->prescaler);
	default:
		return -ENODEV;
	}

	return 0;
}
#endif /* CONFIG_CLOCK_CONTROL_MCHP_CONFIG_RUNTIME */

static DEVICE_API(clock_control, clock_control_mchp_driver_api) = {
	.on = clock_control_mchp_on,
	.off = clock_control_mchp_off,
	.get_status = clock_control_mchp_get_status,
#if defined(CONFIG_CLOCK_CONTROL_MCHP_GET_RATE)
	.get_rate = clock_control_mchp_get_rate,
#endif /* CONFIG_CLOCK_CONTROL_MCHP_GET_RATE */
#if defined(CONFIG_CLOCK_CONTROL_MCHP_SET_RATE)
	.set_rate = clock_control_mchp_set_rate,
#endif /* CONFIG_CLOCK_CONTROL_MCHP_SET_RATE */
#if defined(CONFIG_CLOCK_CONTROL_MCHP_CONFIG_RUNTIME)
	.configure = clock_control_mchp_configure,
#endif /* CONFIG_CLOCK_CONTROL_MCHP_CONFIG_RUNTIME */
};

static int clock_control_mchp_init(const struct device *dev)
{
	const struct clock_control_mchp_config *dev_config = dev->config;
	pmc_registers_t *const regs = dev_config->regs;
	utmi_registers_t *utmi_regs = dev_config->utmi_regs;
	matrix_registers_t *matrix_regs = dev_config->matrix_regs;
	struct clock_control_mchp_data *dev_data = dev->data;
	uint32_t cktrim = 0u;
	uint32_t mor = 0u;
	int ret = 0;

	/* Configure clock sources for TC0, I2SC0 and I2SC1 */
	matrix_regs->CCFG_PCCR = CCFG_PCCR_TC0CC(dev_config->tc0_pck7) |
				 CCFG_PCCR_I2SC0CC(dev_config->i2sc0_gclk) |
				 CCFG_PCCR_I2SC1CC(dev_config->i2sc1_gclk);

	/* Main Crystal Oscillator initialization */
	if (dev_config->main_xosc_enabled) {
		mor = regs->CKGR_MOR & ~CKGR_MOR_MOSCXTST_Msk;

		mor |= CKGR_MOR_MOSCXTST(dev_config->main_xosc_startup_cycles);
		mor |= CKGR_MOR_MOSCXTEN_Msk;
		mor |= CKGR_MOR_PASSWD;
		regs->CKGR_MOR = mor;

		if (!WAIT_FOR(((regs->PMC_SR & PMC_SR_MOSCXTS_Msk) != 0u), TIMEOUT_XOSC_RDY,
			      (void)NULL)) {
			LOG_ERR("%s: XOSC ready timed out", __func__);
		}
	}

	/* USB UTMI Oscillator configuration */
	cktrim = utmi_regs->UTMI_CKTRIM & ~UTMI_CKTRIM_FREQ_Msk;
	if (dev_config->main_xosc_freq_mhz == 12u) {
		cktrim |= UTMI_CKTRIM_FREQ_XTAL12;
	} else if (dev_config->main_xosc_freq_mhz == 16u) {
		cktrim |= UTMI_CKTRIM_FREQ_XTAL16;
	} else {
		/* This case will fail in `clock_control_mchp_upll_on` */
	}

	/* USB UTMI Oscillator configuration */
	utmi_regs->UTMI_CKTRIM = cktrim;

	/* UPLL post-divider configuration */
	configure_upll_div(dev, dev_config->upll_div != 0u);

	if (dev_config->upll_enabled) {
		clock_control_mchp_upll_on(dev_config);
	}

	/* Clock generator initialization */
	dev_data->slow_clk_rate = configure_slow_oscillator(dev, dev_config->clock_gen_slow_osc);
	dev_data->main_clk_rate = configure_main_oscillator(dev, dev_config->clock_gen_main_osc);

	/* PLLA initialization */
	ret = configure_plla(dev, dev_config->plla_multiplier, dev_config->plla_divider,
			     dev_config->plla_lock_count, dev_data->main_clk_rate,
			     &dev_data->plla_clk_rate);
	if (ret != 0) {
		return ret;
	}

	/* Host Clock Controller initialization */
	configure_mckr(dev, dev_config->mckr_source, dev_config->mckr_prescaler,
		       dev_config->mckr_main_clock_divider);

	DT_FOREACH_CHILD(DT_NODELABEL(pck), CLOCK_MCHP_ITERATE_PCK);
	DT_FOREACH_CHILD(DT_NODELABEL(gclk), CLOCK_MCHP_ITERATE_GCLK);

	return 0;
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	static struct clock_control_mchp_data mchp_clock_control_##n##_data = {};                  \
                                                                                                   \
	static const struct clock_control_mchp_config mchp_clock_control_##n##_config = {          \
		.regs = (pmc_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, pmc),                       \
		.supc_regs = (supc_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, supc),                \
		.utmi_regs = (utmi_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, utmi),                \
		.matrix_regs = (matrix_registers_t *)DT_INST_REG_ADDR_BY_NAME(n, matrix),          \
		.tc0_pck7 = DT_INST_PROP(n, tc0_pck7),                                             \
		.i2sc0_gclk = DT_INST_PROP(n, i2sc0_gclk),                                         \
		.i2sc1_gclk = DT_INST_PROP(n, i2sc1_gclk),                                         \
		.clock_gen_main_osc =                                                              \
			DT_ENUM_IDX(DT_INST_CHILD(n, clock_generator), main_oscillator),           \
		.clock_gen_slow_osc =                                                              \
			DT_ENUM_IDX(DT_INST_CHILD(n, clock_generator), slow_oscillator),           \
		.main_rc_freq_mhz =                                                                \
			DT_ENUM_IDX(DT_INST_CHILD(n, clock_generator), rc_frequency_mhz),          \
		.slow_xosc_freq = DT_PROP(DT_INST_CHILD(n, clock_generator), slow_xosc_frequency), \
		.main_xosc_enabled =                                                               \
			DT_PROP(DT_INST_CHILD(n, clock_generator), main_xosc_enabled),             \
		.main_xosc_startup_cycles =                                                        \
			DT_PROP(DT_INST_CHILD(n, clock_generator), main_xosc_startup_cycles),      \
		.main_xosc_freq_mhz =                                                              \
			DT_PROP(DT_INST_CHILD(n, clock_generator), main_xosc_frequency_mhz),       \
		.upll_enabled = DT_PROP(DT_INST_CHILD(n, clock_generator), upll_enabled),          \
		.upll_startup_cycles =                                                             \
			DT_PROP(DT_INST_CHILD(n, clock_generator), upll_startup_cycles),           \
		.upll_div = DT_ENUM_IDX(DT_INST_CHILD(n, clock_generator), upll_div),              \
		.plla_multiplier = DT_PROP(DT_INST_CHILD(n, clock_generator), plla_multiplier),    \
		.plla_lock_count = DT_PROP(DT_INST_CHILD(n, clock_generator), plla_lock_count),    \
		.plla_divider = DT_PROP(DT_INST_CHILD(n, clock_generator), plla_divider),          \
		.mckr_source = DT_ENUM_IDX(DT_INST_CHILD(n, mckr), source),                        \
		.mckr_prescaler = DT_ENUM_IDX(DT_INST_CHILD(n, mckr), prescaler),                  \
		.mckr_main_clock_divider =                                                         \
			DT_ENUM_IDX(DT_INST_CHILD(n, mckr), main_clock_divider),                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, clock_control_mchp_init, NULL, &mchp_clock_control_##n##_data,    \
			      &mchp_clock_control_##n##_config, PRE_KERNEL_1,                      \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_mchp_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
