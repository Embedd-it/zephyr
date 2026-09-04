/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip PIO pin controller (pinctrl) driver.
 *
 */

#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

#define NUM_PORTS 5

/*
 * Utility macro that expands to the PORT address if it exists.
 * This macro checks if a node label exists in the device tree and, if it does,
 * it expands to the register address of that node label. If the node label does
 * not exist, it expands to nothing.
 */
#define MCHP_PORT_ADDR_OR_NONE(nodelabel)                                                          \
	IF_ENABLED(DT_NODE_EXISTS(DT_NODELABEL(nodelabel)),            \
		(DT_REG_ADDR(DT_NODELABEL(nodelabel)),))

/*
 * Array of port addresses for the MCHP G1 PIO series.
 * This array contains the register addresses of the ports (PORTA, PORTB, PORTC, and PORTD)
 * for the MCHP G1 PIO series port peripheral. The addresses are obtained using the
 * MCHP_PORT_ADDR_OR_NONE macro, which ensures that only existing ports are included.
 * This can be updated for other devices using conditional compilation directives
 */
static const uint32_t mchp_port_addrs[NUM_PORTS] = {
	MCHP_PORT_ADDR_OR_NONE(pioa) MCHP_PORT_ADDR_OR_NONE(piob) MCHP_PORT_ADDR_OR_NONE(pioc)
		MCHP_PORT_ADDR_OR_NONE(piod) MCHP_PORT_ADDR_OR_NONE(pioe)};

/*
 * Set pinmux registers using odd/even logic
 * This function configures the pinmux registers for a given pin based on
 * the provided pin control information. It determines whether the pin number
 * is odd or even and sets the appropriate bits in the ABCDSR registers.
 */
static int pinctrl_mchp_pio_pinmux(const pinctrl_soc_pin_t *pin)
{
	uint8_t pin_num = MCHP_PINMUX_PIN_GET(pin->pinmux);
	uint8_t port_id = MCHP_PINMUX_PORT_GET(pin->pinmux);
	uint8_t pin_mux = MCHP_PINMUX_PERIPH_GET(pin->pinmux);
	uint8_t func = MCHP_PINMUX_FUNC_GET(pin->pinmux);

	if (port_id >= NUM_PORTS) {
		return -EINVAL;
	}

	pio_registers_t *const regs = (pio_registers_t *)mchp_port_addrs[port_id];

	switch (func) {
	/* Peripheral output */
	case MCHP_PINMUX_FUNC_periph: {
		switch (pin_mux) {
		case MCHP_PINMUX_PERIPH_a: {
			regs->PIO_ABCDSR[0u] &= ~BIT(pin_num);
			regs->PIO_ABCDSR[1u] &= ~BIT(pin_num);
			regs->PIO_PDR = BIT(pin_num);
			break;
		}
		case MCHP_PINMUX_PERIPH_b: {
			regs->PIO_ABCDSR[0u] |= BIT(pin_num);
			regs->PIO_ABCDSR[1u] &= ~BIT(pin_num);
			regs->PIO_PDR = BIT(pin_num);
			break;
		}
		case MCHP_PINMUX_PERIPH_c: {
			regs->PIO_ABCDSR[0u] &= ~BIT(pin_num);
			regs->PIO_ABCDSR[1u] |= BIT(pin_num);
			regs->PIO_PDR = BIT(pin_num);
			break;
		}
		case MCHP_PINMUX_PERIPH_d: {
			regs->PIO_ABCDSR[0u] |= BIT(pin_num);
			regs->PIO_ABCDSR[1u] |= BIT(pin_num);
			regs->PIO_PDR = BIT(pin_num);
			break;
		}
		default: {
			/* Invalid pin */
			return -EINVAL;
		}
		}
		break;
	}
	/* GPIO */
	case MCHP_PINMUX_FUNC_gpio: {
		regs->PIO_PER = BIT(pin_num);
		break;
	}
	/* Managed by the specific peripherals */
	case MCHP_PINMUX_FUNC_extra:
	case MCHP_PINMUX_FUNC_system:
	case MCHP_PINMUX_FUNC_wakeup: {
		break;
	}
	default: {
		return -EINVAL;
	}
	}

	return 0;
}

/**
 * Set all pin configuration registers by checking the flags.
 * This function configures various pin settings such as pull-up and pull-down based on the provided
 * pin control information.
 */
static int pinctrl_mchp_pio_set_flags(const pinctrl_soc_pin_t *pin)
{
	uint8_t pin_num = MCHP_PINMUX_PIN_GET(pin->pinmux);
	uint8_t port_id = MCHP_PINMUX_PORT_GET(pin->pinmux);
	pio_registers_t *const regs = (pio_registers_t *)mchp_port_addrs[port_id];

	if ((pin->pinflag & MCHP_PINCTRL_PULLUP) != 0u &&
	    (pin->pinflag & MCHP_PINCTRL_PULLDOWN) != 0u) {
		/* A pull cannot be enabled if the opposite pull is enabled. */
		return -EINVAL;
	}

	/* Disable interrupts on the pin(s) */
	regs->PIO_IDR = BIT(pin_num);

	/* Enable pull-up. */
	if ((pin->pinflag & MCHP_PINCTRL_PULLUP) != 0u) {
		regs->PIO_PPDDR = BIT(pin_num);
		regs->PIO_PUER = BIT(pin_num);
	} else if ((pin->pinflag & MCHP_PINCTRL_PULLDOWN) != 0u) {
		/* Enable pull-down. */
		regs->PIO_PUDR = BIT(pin_num);
		regs->PIO_PPDER = BIT(pin_num);
	} else {
		/* Neither */
		regs->PIO_PUDR = BIT(pin_num);
		regs->PIO_PPDDR = BIT(pin_num);
	}

#if defined(CONFIG_PIN_OPEN_DRAIN)
	if ((pin->pinflag & MCHP_PINCTRL_OPENDRAIN) != 0u) {
		regs->PIO_MDER = BIT(pin_num);
	} else {
		regs->PIO_MDDR = BIT(pin_num);
	}
#endif /* CONFIG_PIN_OPEN_DRAIN */

#if defined(CONFIG_PIN_DRIVE_STRENGTH)
	if ((pin->pinflag & MCHP_PINCTRL_DRIVESTRENGTH) != 0u) {
		regs->PIO_DRIVER |= BIT(pin_num);
	} else {
		regs->PIO_DRIVER &= ~BIT(pin_num);
	}
#endif /* CONFIG_PIN_DRIVE_STRENGTH */

	return 0;
}

/**
 * Configure a specific pin based on the provided pin configuration.
 * This helper function configures a pin by determining its port function and then
 * calling the appropriate functions to set the pinmux and other pin configurations.
 */
static int pinctrl_mchp_pio_configure_pin(pinctrl_soc_pin_t pin)
{
	int ret = 0;

	ret = pinctrl_mchp_pio_pinmux(&pin);
	if (ret != 0) {
		return ret;
	}

	ret = pinctrl_mchp_pio_set_flags(&pin);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);
	int ret = 0;

	for (size_t i = 0u; i < pin_cnt; i++) {
		ret = pinctrl_mchp_pio_configure_pin(pins[i]);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}
