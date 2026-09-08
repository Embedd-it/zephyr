/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_clock_pic32cz_ca70_mc70.h
 * @brief Clock control header file for Microchip pic32cz_ca70_mc70 family.
 *
 * This file provides clock driver interface definitions and structures
 * for the pic32cz_ca70_mc70 family, used with clock_control_configure().
 *
 * @defgroup clock_control_mchp_pic32cz_ca70_mc70 Clock Control (PIC32CZ CA70/MC70 specifics)
 * @ingroup clock_control_interface
 * @{
 */

#ifndef INCLUDE_ZEPHYR_DRIVERS_CLOCK_CONTROL_MCHP_CLOCK_PIC32CZ_CA70_MC70_H_
#define INCLUDE_ZEPHYR_DRIVERS_CLOCK_CONTROL_MCHP_CLOCK_PIC32CZ_CA70_MC70_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>

/** @brief Clock Generator main oscillator options */
enum clock_mchp_generator_main_oscillator {
	CLOCK_MCHP_GENERATOR_MAIN_OSCILLATOR_RC = 0,   /**< Fast RC oscillator */
	CLOCK_MCHP_GENERATOR_MAIN_OSCILLATOR_XOSC = 1, /**< External crystal oscillator */
	CLOCK_MCHP_GENERATOR_MAIN_OSCILLATOR_XOSC_BYPASS =
		2, /**< External clock, bypassing the XTAL */
};

/** @brief Clock Generator slow oscillator options */
enum clock_mchp_generator_slow_oscillator {
	CLOCK_MCHP_GENERATOR_SLOW_OSCILLATOR_RC = 0, /**< Internal 32 kHz RC oscillator */
	CLOCK_MCHP_GENERATOR_SLOW_OSCILLATOR_XOSC =
		1, /**< External 32.768 kHz crystal oscillator */
	CLOCK_MCHP_GENERATOR_SLOW_OSCILLATOR_XOSC_BYPASS =
		2, /**< External 32.768 kHz clock, bypassing the XTAL */
};

/** @brief PLLA configuration structure */
struct clock_mchp_subsys_plla_config {
	/** @brief PLLA multiplier. Actual multiplication factor applied is (multiplier + 1) */
	uint8_t multiplier;

	/** @brief PLLA divider. A value of 0 disables the PLLA */
	uint8_t divider;

	/** @brief Number of slow clock cycles before the LOCKA bit is set */
	uint8_t lock_count;
};

/** @brief UPLL configuration structure */
struct clock_mchp_subsys_upll_config {
	/** @brief Divide the UPLL output clock by 2 before it is used as a MCKR source */
	bool div2_en;
};

/** @brief Host Clock Controller source clocks */
enum clock_mchp_host_clock_controller_source {
	CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_SLOW_CK = 0,  /**< SLOW_CLK */
	CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_MAIN_CLK = 1, /**< MAIN_CLK */
	CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_PLLA_CK = 2,  /**< PLLA output clock */
	CLOCK_MCHP_HOST_CLOCK_CONTROLLER_SOURCE_UPLL_CK = 3,  /**< UPLL output clock */
};

/** @brief Processor Clock prescaler */
enum clock_mchp_processor_clock_prescaler {
	CLOCK_MCHP_PROCESSOR_CLOCK_PRESCALER_1 = 0,  /**< Divide by 1 */
	CLOCK_MCHP_PROCESSOR_CLOCK_PRESCALER_2 = 1,  /**< Divide by 2 */
	CLOCK_MCHP_PROCESSOR_CLOCK_PRESCALER_4 = 2,  /**< Divide by 4 */
	CLOCK_MCHP_PROCESSOR_CLOCK_PRESCALER_8 = 3,  /**< Divide by 8 */
	CLOCK_MCHP_PROCESSOR_CLOCK_PRESCALER_16 = 4, /**< Divide by 16 */
	CLOCK_MCHP_PROCESSOR_CLOCK_PRESCALER_32 = 5, /**< Divide by 32 */
	CLOCK_MCHP_PROCESSOR_CLOCK_PRESCALER_64 = 6, /**< Divide by 64 */
	CLOCK_MCHP_PROCESSOR_CLOCK_PRESCALER_3 = 7,  /**< Divide by 3 */
};

/** @brief Main Clock divider */
enum clock_mchp_main_clock_divider {
	CLOCK_MCHP_MAIN_CLOCK_DIVIDER_EQ_PCK = 0,   /**< Divide by 1 */
	CLOCK_MCHP_MAIN_CLOCK_DIVIDER_PCK_DIV2 = 1, /**< Divide by 2 */
	CLOCK_MCHP_MAIN_CLOCK_DIVIDER_PCK_DIV4 = 2, /**< Divide by 4 */
	CLOCK_MCHP_MAIN_CLOCK_DIVIDER_PCK_DIV3 = 3, /**< divide by 3 */
};

/** @brief Host Clock Controller (MCKR) configuration structure */
struct clock_mchp_subsys_mckr_config {
	/** @brief Source clock for the Host Clock Controller */
	enum clock_mchp_host_clock_controller_source source;

	/** @brief Prescaler applied to the selected source */
	enum clock_mchp_processor_clock_prescaler prescaler;

	/** @brief Divider applied after the prescaler */
	enum clock_mchp_main_clock_divider divider;
};

/** @brief Generic Clock (GCLK) source clocks. Matches the PMC_PCR.GCLKCSS field encoding */
enum clock_mchp_gclk_src {
	CLOCK_MCHP_GCLK_SRC_SLOWCK = 0,   /**< SLOW_CLK */
	CLOCK_MCHP_GCLK_SRC_MAIN_CLK = 1, /**< MAIN_CLK */
	CLOCK_MCHP_GCLK_SRC_PLLACK = 2,   /**< PLLA output clock */
	CLOCK_MCHP_GCLK_SRC_UPLLCK = 3,   /**< UPLL output clock */
	CLOCK_MCHP_GCLK_SRC_HOST_CLK = 4, /**< HOST_CLK (post main clock controller) */
	CLOCK_MCHP_GCLK_SRC_AUDIOCLK = 5, /**< AUDIOPLLCLK */
};

/** @brief Generic Clock (GCLK) configuration structure. Used for SUBSYS_TYPE_GCLK */
struct clock_mchp_subsys_gclk_config {
	/** @brief Generic clock source selection */
	enum clock_mchp_gclk_src source;

	/** @brief Generic clock division ratio (1-256) */
	uint16_t divider;
};

/** @brief Programmable Clock (PCK) source clocks. Matches the PMC_PCK.CSS field encoding */
enum clock_mchp_pck_src {
	CLOCK_MCHP_PCK_SRC_SLOWCK = 0,   /**< SLOW_CLK */
	CLOCK_MCHP_PCK_SRC_MAIN_CLK = 1, /**< MAIN_CLK */
	CLOCK_MCHP_PCK_SRC_PLLACK = 2,   /**< PLLA output clock */
	CLOCK_MCHP_PCK_SRC_UPLLCK = 3,   /**< UPLL output clock */
	CLOCK_MCHP_PCK_SRC_HOST_CLK = 4, /**< HOST_CLK (Host Clock Controller output) */
};

/** @brief Programmable Clock (PCK) configuration structure. Used for SUBSYS_TYPE_PROGRAMMABLE */
struct clock_mchp_subsys_pck_config {
	/** @brief Programmable clock source selection */
	enum clock_mchp_pck_src source;

	/** @brief Programmable clock prescaler (1-256) */
	uint16_t prescaler;
};

/** @} */

#endif /* INCLUDE_ZEPHYR_DRIVERS_CLOCK_CONTROL_MCHP_CLOCK_PIC32CZ_CA70_MC70_H_ */
