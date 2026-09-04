/*
 * SPDX-FileCopyrightText: Copyright (c) 2018 Justin Watson
 * SPDX-FileCopyrightText: Copyright (c) 2023 Gerson Fernando Budke
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip PIO GPIO driver.
 *
 */

#define DT_DRV_COMPAT microchip_pio_g1_gpio

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/dt-bindings/gpio/microchip-pio-g1-gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_mchp_pio_g1, CONFIG_GPIO_LOG_LEVEL);

struct gpio_mchp_pio_config {
	/* Common configuration data structure; needs to be first in order to be accessed */
	struct gpio_driver_config common;

	/* Register block */
	pio_registers_t *regs;

	/* Divider applied to the slow clock for debounce filtering */
	uint32_t slow_clock_divider;
};

struct gpio_mchp_pio_data {
	/* Common runtime data structure; needs to be first in order to be accessed */
	struct gpio_driver_data common;

	/* Reference to callback for the commn gpio library */
	sys_slist_t event_callbacks;

	/* Additional map to keep track of input configuration */
	uint32_t input_map;

	/* Additional maps to keep track of the initial output configuration */
	uint32_t init_high_map;
	uint32_t init_low_map;
};

static int gpio_mchp_pio_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_mchp_pio_data *dev_data = dev->data;
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	if ((flags & GPIO_PULL_UP) != 0u && (flags & GPIO_PULL_DOWN) != 0u) {
		/* A pull cannot be enabled if the opposite pull is enabled. */
		return -EINVAL;
	}

	if ((flags & MCHP_GPIO_GLITCH_FILTER) != 0u && (flags & MCHP_GPIO_DEBOUNCE_FILTER) != 0u) {
		/* Not supported at the same time */
		return -EINVAL;
	}

	if ((flags & GPIO_SINGLE_ENDED) != 0u) {
		if ((flags & GPIO_LINE_OPEN_DRAIN) != 0u) {
			/* Enable open-drain drive mode */
			regs->PIO_MDER = BIT(pin);
		} else {
			/* Open-drain is the only supported single-ended mode */
			return -ENOTSUP;
		}
	} else {
		/* Disable open-drain drive mode */
		regs->PIO_MDDR = BIT(pin);
	}

	/* Neither input nor output mode is selected or the pin is explicitly disabled */
	if (((flags & (GPIO_OUTPUT | GPIO_INPUT)) == 0u)) {
		/* Disable the interrupt. */
		regs->PIO_IDR = BIT(pin);
		/* Disable pull-up. */
		regs->PIO_PUDR = BIT(pin);
		/* Disable pull-down. */
		regs->PIO_PPDDR = BIT(pin);
		/* Disable output */
		regs->PIO_ODR = BIT(pin);
		/* Keep the pin under PIO control */
		regs->PIO_PER = BIT(pin);

		dev_data->input_map &= ~BIT(pin);

		return 0;
	}

	dev_data->init_high_map &= ~BIT(pin);
	dev_data->init_low_map &= ~BIT(pin);

	/* Setup the pin direction. */
	if ((flags & GPIO_OUTPUT) != 0u) {
		if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0u) {
			/* Set the pin. */
			regs->PIO_SODR = BIT(pin);
			dev_data->init_high_map |= BIT(pin);
		}
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0u) {
			/* Clear the pin. */
			regs->PIO_CODR = BIT(pin);
			dev_data->init_low_map |= BIT(pin);
		}
		/* Enable the output */
		regs->PIO_OER = BIT(pin);
		/* Enable direct control of output level via PIO_ODSR */
		regs->PIO_OWER = BIT(pin);
	} else {
		/* Disable the output */
		regs->PIO_ODR = BIT(pin);
	}

	if ((flags & GPIO_INPUT) != 0u) {
		dev_data->input_map |= BIT(pin);
	} else {
		dev_data->input_map &= ~BIT(pin);
	}

	/* A pull cannot be enabled if the opposite pull is still enabled: the
	 * hardware silently discards the PIO_PUER/PIO_PPDER write in that case.
	 */
	if ((flags & GPIO_PULL_UP) != 0u) {
		/* Disable pull-down first, then enable pull-up. */
		regs->PIO_PPDDR = BIT(pin);
		regs->PIO_PUER = BIT(pin);
	} else if ((flags & GPIO_PULL_DOWN) != 0u) {
		/* Disable pull-up first, then enable pull-down. */
		regs->PIO_PUDR = BIT(pin);
		regs->PIO_PPDER = BIT(pin);
	} else {
		regs->PIO_PUDR = BIT(pin);
		regs->PIO_PPDDR = BIT(pin);
	}

	if ((flags & MCHP_GPIO_GLITCH_FILTER) != 0u) {
		/* Apply glitch filter */
		regs->PIO_IFER = BIT(pin);
		regs->PIO_IFSCDR = BIT(pin);
	} else if ((flags & MCHP_GPIO_DEBOUNCE_FILTER) != 0u) {
		/* Apply debounce filter */
		regs->PIO_IFER = BIT(pin);
		regs->PIO_IFSCER = BIT(pin);
	} else {
		/* No filter */
		regs->PIO_IFDR = BIT(pin);
	}

	if ((flags & MCHP_GPIO_SCHMITT_TRIGGER_DISABLE) != 0u) {
		regs->PIO_SCHMITT |= BIT(pin);
	} else {
		regs->PIO_SCHMITT &= ~BIT(pin);
	}

	/* Enable the PIO to control the pin (instead of a peripheral). */
	regs->PIO_PER = BIT(pin);

	return 0;
}

static int gpio_mchp_pio_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	*value = regs->PIO_PDSR;

	return 0;
}

static int gpio_mchp_pio_port_set_masked_raw(const struct device *dev, uint32_t mask,
					     uint32_t value)
{
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	regs->PIO_ODSR = (regs->PIO_ODSR & ~mask) | (mask & value);

	return 0;
}

static int gpio_mchp_pio_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	regs->PIO_SODR = mask;

	return 0;
}

static int gpio_mchp_pio_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	regs->PIO_CODR = mask;

	return 0;
}

static int gpio_mchp_pio_port_toggle_bits(const struct device *dev, uint32_t mask)
{
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	regs->PIO_ODSR ^= mask;

	return 0;
}

static int gpio_mchp_pio_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
						 enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	/* Disable the interrupt. */
	regs->PIO_IDR = BIT(pin);
	/* Disable additional interrupt modes. */
	regs->PIO_AIMDR = BIT(pin);

	if (trig != GPIO_INT_TRIG_BOTH) {
		uint32_t rising_edge = 0u;

		/* Enable additional interrupt modes to support single
		 * edge/level detection.
		 */
		regs->PIO_AIMER = BIT(pin);

		if (mode == GPIO_INT_MODE_EDGE) {
			regs->PIO_ESR = BIT(pin);
		} else {
			regs->PIO_LSR = BIT(pin);
		}

		if (trig == GPIO_INT_TRIG_HIGH) {
			rising_edge = BIT(pin);
		} else {
			rising_edge = ~BIT(pin);
		}

		/* Set to high-level or rising edge. */
		regs->PIO_REHLSR = rising_edge & BIT(pin);
		/* Set to low-level or falling edge. */
		regs->PIO_FELLSR = ~rising_edge & BIT(pin);
	}

	if (mode != GPIO_INT_MODE_DISABLED) {
		/* Clear any pending interrupts */
		(void)regs->PIO_ISR;
		/* Enable the interrupt. */
		regs->PIO_IER = BIT(pin);
	}

	return 0;
}

static int gpio_mchp_pio_manage_callback(const struct device *dev, struct gpio_callback *callback,
					 bool set)
{
	struct gpio_mchp_pio_data *dev_data = dev->data;

	return gpio_manage_callback(&dev_data->event_callbacks, callback, set);
}

#if defined(CONFIG_GPIO_GET_CONFIG)
static int gpio_mchp_pio_pin_get_config(const struct device *dev, gpio_pin_t pin,
					gpio_flags_t *flags)
{
	struct gpio_mchp_pio_data *dev_data = dev->data;
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	*flags = 0u;

	if ((regs->PIO_MDSR & BIT(pin)) != 0u) {
		*flags |= GPIO_SINGLE_ENDED | GPIO_LINE_OPEN_DRAIN;
	}

	if ((regs->PIO_PSR & BIT(pin)) == 0u) {
		*flags |= GPIO_DISCONNECTED;
	}

	if ((regs->PIO_OSR & BIT(pin)) != 0u) {
		*flags |= GPIO_OUTPUT;
	}

	if ((dev_data->input_map & BIT(pin)) != 0u) {
		*flags |= GPIO_INPUT;
	}

	if ((dev_data->init_high_map & BIT(pin)) != 0u) {
		*flags |= GPIO_OUTPUT_INIT_HIGH;
	}
	if ((dev_data->init_low_map & BIT(pin)) != 0u) {
		*flags |= GPIO_OUTPUT_INIT_LOW;
	}

	if ((regs->PIO_PUSR & BIT(pin)) == 0u) {
		*flags |= GPIO_PULL_UP;
	} else if ((regs->PIO_PPDSR & BIT(pin)) == 0u) {
		*flags |= GPIO_PULL_DOWN;
	}

	if (((regs->PIO_IFSR & BIT(pin)) != 0u) && ((regs->PIO_IFSCSR & BIT(pin)) == 0u)) {
		*flags |= MCHP_GPIO_GLITCH_FILTER;
	} else if (((regs->PIO_IFSR & BIT(pin)) != 0u) && ((regs->PIO_IFSCSR & BIT(pin)) != 0u)) {
		*flags |= MCHP_GPIO_DEBOUNCE_FILTER;
	}

	if ((regs->PIO_SCHMITT & BIT(pin)) != 0u) {
		*flags |= MCHP_GPIO_SCHMITT_TRIGGER_DISABLE;
	}

	return 0;
}
#endif /* CONFIG_GPIO_GET_CONFIG */

#if defined(CONFIG_GPIO_GET_DIRECTION)
static int gpio_mchp_pio_port_get_direction(const struct device *dev, gpio_port_pins_t map,
					    gpio_port_pins_t *inputs, gpio_port_pins_t *outputs)
{
	struct gpio_mchp_pio_data *dev_data = dev->data;
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	for (size_t pin_index = 0u; pin_index < 32u; pin_index++) {
		if ((map & BIT(pin_index)) != 0u) {
			if ((regs->PIO_PSR & BIT(pin_index)) == 0u) {
				/* PIO is inactive */
				continue;
			}

			if ((regs->PIO_OSR & BIT(pin_index)) != 0u) {
				if (outputs != NULL) {
					*outputs |= 1u << pin_index;
				}
			}

			/* The input value can be read regardless of configuration, so we keep track
			 * of it with an additional map
			 */
			if ((dev_data->input_map & BIT(pin_index)) != 0u) {
				if (inputs != NULL) {
					*inputs |= 1u << pin_index;
				}
			}
		}
	}

	return 0;
}
#endif /* CONFIG_GPIO_GET_DIRECTION */

static DEVICE_API(gpio, gpio_mchp_pio_driver_api) = {
	.pin_configure = gpio_mchp_pio_pin_configure,
	.port_get_raw = gpio_mchp_pio_port_get_raw,
	.port_set_masked_raw = gpio_mchp_pio_port_set_masked_raw,
	.port_set_bits_raw = gpio_mchp_pio_port_set_bits_raw,
	.port_clear_bits_raw = gpio_mchp_pio_port_clear_bits_raw,
	.port_toggle_bits = gpio_mchp_pio_port_toggle_bits,
	.pin_interrupt_configure = gpio_mchp_pio_pin_interrupt_configure,
	.manage_callback = gpio_mchp_pio_manage_callback,
#if defined(CONFIG_GPIO_GET_CONFIG)
	.pin_get_config = gpio_mchp_pio_pin_get_config,
#endif /* CONFIG_GPIO_GET_CONFIG */
#if defined(CONFIG_GPIO_GET_DIRECTION)
	.port_get_direction = gpio_mchp_pio_port_get_direction,
#endif /* CONFIG_GPIO_GET_DIRECTION */
};

static int gpio_mchp_pio_init(const struct device *dev)
{
	const struct gpio_mchp_pio_config *dev_config = dev->config;
	pio_registers_t *const regs = dev_config->regs;

	regs->PIO_SCDR = dev_config->slow_clock_divider;

	return 0;
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	static const struct gpio_mchp_pio_config mchp_gpio_##n##_config = {                        \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_NGPIOS(32u),              \
			},                                                                         \
		.regs = (pio_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.slow_clock_divider = DT_INST_PROP(n, slow_clock_divider),                         \
	};                                                                                         \
                                                                                                   \
	static struct gpio_mchp_pio_data mchp_gpio_##n##_data = {};                                \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, gpio_mchp_pio_init, NULL, &mchp_gpio_##n##_data,                  \
			      &mchp_gpio_##n##_config, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,    \
			      &gpio_mchp_pio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
