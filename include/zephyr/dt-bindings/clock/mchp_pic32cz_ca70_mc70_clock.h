/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file mchp_pic32cz_ca70_mc70_clock.h
 * @brief List clock subsystem IDs for pic32cz_ca70_mc70 family.
 *
 * Clock subsystem IDs. To be used in devicetree nodes, and as argument for clock API.
 */

#ifndef INCLUDE_ZEPHYR_DT_BINDINGS_CLOCK_MCHP_PIC32CZ_CA70_MC70_CLOCK_H_
#define INCLUDE_ZEPHYR_DT_BINDINGS_CLOCK_MCHP_PIC32CZ_CA70_MC70_CLOCK_H_

/* Clock subsystem Types */
/** @brief Subsystem type: Peripheral clock. */
#define SUBSYS_TYPE_PERIPHERAL   (0)
/** @brief Subsystem type: Generic clock for peripherals. */
#define SUBSYS_TYPE_GCLK         (1)
/** @brief Subsystem type: Programmable clock generator. */
#define SUBSYS_TYPE_PROGRAMMABLE (2)
/** @brief Subsystem type: Slow clock. */
#define SUBSYS_TYPE_SLOW_CLK     (3)
/** @brief Subsystem type: UPLL clock. */
#define SUBSYS_TYPE_UPLL         (4)
/** @brief Subsystem type: 48MHz USB clock. */
#define SUBSYS_TYPE_USB_CLK      (5)
/** @brief Subsystem type: Main Clock (MAINCK). */
#define SUBSYS_TYPE_MAIN_CLK     (6)
/** @brief Subsystem type: PLLA. */
#define SUBSYS_TYPE_PLLA         (7)
/** @brief Subsystem type: Host Clock Controller (MCKR). */
#define SUBSYS_TYPE_MCKR         (8)

/**
 * @name CCFG_PCCR Clock source selection bits
 *
 * Bits selecting the corresponding generic clock as source for TC0, I2SC0 and I2SC1
 * @{
 */

#define MCHP_CCFG_PCCR_TC0CC_GCLK   (1 << 20)
#define MCHP_CCFG_PCCR_I2SC0CC_GCLK (1 << 21)
#define MCHP_CCFG_PCCR_I2SC1CC_GCLK (1 << 22)

/** @} */

/**
 * @brief Derive a 32-bit clock subsystem identifier.
 *
 * Encodes the clock subsystem type, MCLK bus information,
 * GCLK peripheral channel, and instance number into a
 * single 32-bit identifier.
 *
 * Bit field layout:
 * - 00..15 (16 bits): inst
 *
 * - 16..31 (16 bits): type
 * following values
 * SUBSYS_TYPE_PERIPHERAL   (0)
 * SUBSYS_TYPE_GCK          (1)
 * SUBSYS_TYPE_PROGRAMMABLE (2)
 *
 * @param type clock subsystem type
 * @param inst instance number of the specified clock type
 *
 * @return Encoded clock subsystem identifier
 */
#define MCHP_CLOCK_DERIVE_ID(type, inst) (((type) << 16) | (inst))

/**
 * @name Peripheral Clock IDs
 * @{
 */
/** @brief RTC clock ID. */
#define CLOCK_MCHP_RTC          MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 2)
/** @brief WDT clock ID. */
#define CLOCK_MCHP_WDT          MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 4)
/** @brief EEFC clock ID. */
#define CLOCK_MCHP_EEFC         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 6)
/** @brief UART0 clock ID. */
#define CLOCK_MCHP_UART0        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 7)
/** @brief UART1 clock ID. */
#define CLOCK_MCHP_UART1        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 8)
/** @brief PIOA clock ID. */
#define CLOCK_MCHP_PIOA         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 10)
/** @brief PIOB clock ID. */
#define CLOCK_MCHP_PIOB         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 11)
/** @brief PIOC clock ID. */
#define CLOCK_MCHP_PIOC         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 12)
/** @brief USART0 clock ID. */
#define CLOCK_MCHP_USART0       MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 13)
/** @brief USART1 clock ID. */
#define CLOCK_MCHP_USART1       MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 14)
/** @brief USART2 clock ID. */
#define CLOCK_MCHP_USART2       MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 15)
/** @brief PIOD clock ID. */
#define CLOCK_MCHP_PIOD         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 16)
/** @brief PIOE clock ID. */
#define CLOCK_MCHP_PIOE         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 17)
/** @brief HSMCI clock ID. */
#define CLOCK_MCHP_HSMCI        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 18)
/** @brief TWIHS0 clock ID. */
#define CLOCK_MCHP_TWIHS0       MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 19)
/** @brief TWIHS1 clock ID. */
#define CLOCK_MCHP_TWIHS1       MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 20)
/** @brief SPI0 clock ID. */
#define CLOCK_MCHP_SPI0         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 21)
/** @brief TC0_CHANNEL0 clock ID. */
#define CLOCK_MCHP_TC0_CHANNEL0 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 23)
/** @brief TC0_CHANNEL1 clock ID. */
#define CLOCK_MCHP_TC0_CHANNEL1 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 24)
/** @brief TC0_CHANNEL2 clock ID. */
#define CLOCK_MCHP_TC0_CHANNEL2 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 25)
/** @brief TC1_CHANNEL0 clock ID. */
#define CLOCK_MCHP_TC1_CHANNEL0 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 26)
/** @brief TC1_CHANNEL1 clock ID. */
#define CLOCK_MCHP_TC1_CHANNEL1 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 27)
/** @brief TC1_CHANNEL2 clock ID. */
#define CLOCK_MCHP_TC1_CHANNEL2 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 28)
/** @brief AFEC0 clock ID. */
#define CLOCK_MCHP_AFEC0        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 29)
/** @brief DACC clock ID. */
#define CLOCK_MCHP_DACC         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 30)
/** @brief PWM0 clock ID. */
#define CLOCK_MCHP_PWM0         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 31)
/** @brief ICM clock ID. */
#define CLOCK_MCHP_ICM          MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 32)
/** @brief ACC clock ID. */
#define CLOCK_MCHP_ACC          MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 33)
/** @brief USBHS clock ID. */
#define CLOCK_MCHP_USBHS        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 34)
/** @brief MCAN0 clock ID. */
#define CLOCK_MCHP_MCAN0        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 35)
/** @brief MCAN1 clock ID. */
#define CLOCK_MCHP_MCAN1        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 37)
/** @brief GMAC clock ID. */
#define CLOCK_MCHP_GMAC         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 39)
/** @brief AFEC1 clock ID. */
#define CLOCK_MCHP_AFEC1        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 40)
/** @brief TWIHS2 clock ID. */
#define CLOCK_MCHP_TWIHS2       MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 41)
/** @brief SPI1 clock ID. */
#define CLOCK_MCHP_SPI1         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 42)
/** @brief QSPI clock ID. */
#define CLOCK_MCHP_QSPI         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 43)
/** @brief UART2 clock ID. */
#define CLOCK_MCHP_UART2        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 44)
/** @brief UART3 clock ID. */
#define CLOCK_MCHP_UART3        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 45)
/** @brief UART4 clock ID. */
#define CLOCK_MCHP_UART4        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 46)
/** @brief TC2_CHANNEL0 clock ID. */
#define CLOCK_MCHP_TC2_CHANNEL0 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 47)
/** @brief TC2_CHANNEL1 clock ID. */
#define CLOCK_MCHP_TC2_CHANNEL1 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 48)
/** @brief TC2_CHANNEL2 clock ID. */
#define CLOCK_MCHP_TC2_CHANNEL2 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 49)
/** @brief TC3_CHANNEL0 clock ID. */
#define CLOCK_MCHP_TC3_CHANNEL0 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 50)
/** @brief TC3_CHANNEL1 clock ID. */
#define CLOCK_MCHP_TC3_CHANNEL1 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 51)
/** @brief TC3_CHANNEL2 clock ID. */
#define CLOCK_MCHP_TC3_CHANNEL2 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 52)
/** @brief MLB clock ID. */
#define CLOCK_MCHP_MLB          MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 53)
/** @brief AES clock ID. */
#define CLOCK_MCHP_AES          MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 56)
/** @brief TRNG clock ID. */
#define CLOCK_MCHP_TRNG         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 57)
/** @brief XDMAC clock ID. */
#define CLOCK_MCHP_XDMAC        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 58)
/** @brief ISI clock ID. */
#define CLOCK_MCHP_ISI          MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 59)
/** @brief PWM1 clock ID. */
#define CLOCK_MCHP_PWM1         MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 60)
/** @brief I2SC0 clock ID. */
#define CLOCK_MCHP_I2SC0        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 69)
/** @brief I2SC1 clock ID. */
#define CLOCK_MCHP_I2SC1        MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 70)
/** @} */

/**
 * @name Programmable Clock indexes
 * @{
 */
/** @brief Programmable clock ID 0. */
#define CLOCK_MCHP_PCK0_IDX 0
/** @brief Programmable clock ID 1. */
#define CLOCK_MCHP_PCK1_IDX 1
/** @brief Programmable clock ID 2. */
#define CLOCK_MCHP_PCK2_IDX 2
/** @brief Programmable clock ID 3. */
#define CLOCK_MCHP_PCK3_IDX 3
/** @brief Programmable clock ID for UART/USART. */
#define CLOCK_MCHP_PCK4_IDX 4
/** @brief Programmable clock ID for MCAN. */
#define CLOCK_MCHP_PCK5_IDX 5
/** @brief Programmable clock ID for TC. */
#define CLOCK_MCHP_PCK6_IDX 6
/** @brief Programmable clock ID for TC. */
#define CLOCK_MCHP_PCK7_IDX 7
/** @} */

/**
 * @name Programmable Clock IDs
 * @{
 */
/** @brief Programmable clock ID 0. */
#define CLOCK_MCHP_PCK0 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PROGRAMMABLE, CLOCK_MCHP_PCK0_IDX)
/** @brief Programmable clock ID 1. */
#define CLOCK_MCHP_PCK1 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PROGRAMMABLE, CLOCK_MCHP_PCK1_IDX)
/** @brief Programmable clock ID 2. */
#define CLOCK_MCHP_PCK2 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PROGRAMMABLE, CLOCK_MCHP_PCK2_IDX)
/** @brief Programmable clock ID 3. */
#define CLOCK_MCHP_PCK3 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PROGRAMMABLE, CLOCK_MCHP_PCK3_IDX)
/** @brief Programmable clock ID for UART/USART. */
#define CLOCK_MCHP_PCK4 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PROGRAMMABLE, CLOCK_MCHP_PCK4_IDX)
/** @brief Programmable clock ID for MCAN. */
#define CLOCK_MCHP_PCK5 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PROGRAMMABLE, CLOCK_MCHP_PCK5_IDX)
/** @brief Programmable clock ID for TC. */
#define CLOCK_MCHP_PCK6 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PROGRAMMABLE, CLOCK_MCHP_PCK6_IDX)
/** @brief Programmable clock ID for TC. */
#define CLOCK_MCHP_PCK7 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PROGRAMMABLE, CLOCK_MCHP_PCK7_IDX)
/** @} */

/**
 * @name Generic Clock indexes
 * @{
 */
/** @brief Generic clock ID for I2SC0. */
#define CLOCK_MCHP_GCLK_I2SC0_IDX 69
/** @brief Generic clock ID for I2SC1. */
#define CLOCK_MCHP_GCLK_I2SC1_IDX 70
/** @} */

/**
 * @name Generic Clock IDs
 * @{
 */
/** @brief Generic clock ID for I2SC0. */
#define CLOCK_MCHP_GCLK_I2SC0 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_GCLK, CLOCK_MCHP_GCLK_I2SC0_IDX)
/** @brief Generic clock ID for I2SC1. */
#define CLOCK_MCHP_GCLK_I2SC1 MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_GCLK, CLOCK_MCHP_GCLK_I2SC1_IDX)
/** @} */

/**
 * @name System Clock IDs
 *
 * Singleton clock subsystems, used with clock_control_configure().
 * @{
 */
/** @brief Slow clock (SLOW_CLK) oscillator selection ID. */
#define CLOCK_MCHP_SLOW_CLK MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_SLOW_CLK, 0)
/** @brief Main clock (MAIN_CLK) oscillator selection ID. */
#define CLOCK_MCHP_MAIN_CLK MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_MAIN_CLK, 0)
/** @brief PLLA ID. */
#define CLOCK_MCHP_PLLA     MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PLLA, 0)
/** @brief Host Clock Controller (MCKR) ID. */
#define CLOCK_MCHP_MCKR     MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_MCKR, 0)
/** @brief UPLL ID. */
#define CLOCK_MCHP_UPLL     MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_UPLL, 0)
/** @brief USB CLK. */
#define CLOCK_MCHP_USB_CLK  MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_USB_CLK, 0)
/** @} */

#endif /* INCLUDE_ZEPHYR_DT_BINDINGS_CLOCK_MCHP_PIC32CZ_CA70_MC70_CLOCK_H_ */
