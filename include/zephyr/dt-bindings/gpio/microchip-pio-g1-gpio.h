/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Devicetree GPIO flag definitions for Microchip PIO (G1).
 *
 * Provides Microchip-specific GPIO DT flags used in addition to standard Zephyr
 * GPIO binding flags.
 */

#ifndef INCLUDE_ZEPHYR_DT_BINDINGS_GPIO_MICROCHIP_PIO_G1_GPIO_H_
#define INCLUDE_ZEPHYR_DT_BINDINGS_GPIO_MICROCHIP_PIO_G1_GPIO_H_

/**
 * @def MCHP_GPIO_GLITCH_FILTER
 * @brief Enable hardware glitch filtering for a GPIO pin.
 *
 * Zephyr-specific devicetree flag for Microchip SoCs. When set in GPIO DT flags, the
 * driver enables the SoC's glitch filter feature for the configured pin.
 */
#define MCHP_GPIO_GLITCH_FILTER           (1 << 8)
/**
 * @def MCHP_GPIO_DEBOUNCE_FILTER
 * @brief Enable hardware debounce filtering for a GPIO pin.
 *
 * Zephyr-specific devicetree flag for Microchip SoCs. When set in GPIO DT flags, the
 * driver enables the SoC's debounce filter feature for the configured pin.
 */
#define MCHP_GPIO_DEBOUNCE_FILTER         (1 << 9)
/**
 * @def MCHP_GPIO_SCHMITT_TRIGGER_DISABLE
 * @brief Disable the schmitt trigger for a GPIO pin
 *
 * Zephyr-specific devicetree flag for Microchip SoCs. When set in GPIO DT flags, the
 * driver disables the SoC's schmitt trigger feature for the configured pin.
 */
#define MCHP_GPIO_SCHMITT_TRIGGER_DISABLE (1 << 10)

#endif /* INCLUDE_ZEPHYR_DT_BINDINGS_GPIO_MICROCHIP_PIO_G1_GPIO_H_ */
