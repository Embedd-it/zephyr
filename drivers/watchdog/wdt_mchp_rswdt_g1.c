/*
 * SPDX-FileCopyrightText: Copyright (C) 2017 Intel Deutschland GmbH
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip reset-source watchdog timer (RSWDT) driver.
 *
 */

#define DT_DRV_COMPAT microchip_rswdt_g1_wdt

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_mchp_rswdt_g1, CONFIG_WDT_LOG_LEVEL);

/*
 * Note:
 * - Once the watchdog disable bit is set, it cannot be cleared till next
 *   power reset, i.e, the watchdog cannot be started once stopped.
 * - The MCU boots with RSWDT disabled.
 */

#define PRESCALER     128u
#define WDT_MAX_VALUE 4095u

struct wdt_mchp_rswdt_config {
	/* Register block */
	rswdt_registers_t *regs;

	/* Clock driver */
	const struct device *clock;

	/* Interrupt configuration function */
	void (*irq_config)(const struct device *dev);
};

struct wdt_mchp_rswdt_data {
	/* Callback to be invoked on WDT trigger */
	wdt_callback_t cb;

	/* Watchdog mode, for MR register. It has to be cached because it's write-once */
	uint32_t mode;

	/* Whether a valid timeout has been set */
	bool timeout_valid;

	/* Whether the mode has been set */
	bool mode_set;
};

/*
 * Calculates the watchdog counter value (WDV) to be installed in the watchdog timer
 */
static int wdt_mchp_rswdt_convert_timeout(uint32_t timeout, uint32_t sclk)
{
	uint32_t max = 0u;
	uint32_t min = 0u;

	timeout = timeout * 1000u;
	min = (PRESCALER * 1000000u) / sclk;
	max = min * WDT_MAX_VALUE;

	if ((timeout < min) || (timeout > max)) {
		LOG_ERR("Invalid timeout value allowed range:%d ms to %d ms", min / 1000u,
			max / 1000u);
		return -EINVAL;
	}

	return RSWDT_MR_WDV(timeout / min);
}

static int wdt_mchp_rswdt_disable(const struct device *dev)
{
	struct wdt_mchp_rswdt_data *dev_data = dev->data;

	/* since Watchdog mode register is 'write-once', we can't disable if
	 * someone has already set the mode register
	 */
	if (dev_data->mode_set) {
		return -EPERM;
	}

	/* No timeout has been installed by this driver instance yet, so there is no
	 * watchdog actually running under Zephyr's control to disable (install_timeout()
	 * has no hardware effect until setup() is called).
	 */
	if (!dev_data->timeout_valid) {
		return -EFAULT;
	}

	/* Watchdog Mode register is 'write-once' only register and the RSWDT starts off disabled,
	 * so it can only ever be enabled (once). Therefore the disable operation isn't meaningful.
	 * Return no error only if the peripheral is already disabled.
	 * Set `mode_set` for logical purposes.
	 */
	dev_data->mode_set = true;

	return 0;
}

static int wdt_mchp_rswdt_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_mchp_rswdt_config *dev_config = dev->config;
	rswdt_registers_t *const regs = dev_config->regs;
	struct wdt_mchp_rswdt_data *dev_data = dev->data;

	if (!dev_data->timeout_valid) {
		LOG_ERR("No valid timeouts installed");
		return -EINVAL;
	}

	/* since Watchdog mode register is 'write-once', we can't set if
	 * someone has already set the mode register
	 */
	if (dev_data->mode_set) {
		return -EPERM;
	}

	if ((options & WDT_OPT_PAUSE_IN_SLEEP) == WDT_OPT_PAUSE_IN_SLEEP) {
		dev_data->mode |= RSWDT_MR_WDIDLEHLT_Msk;
	}

	if ((options & WDT_OPT_PAUSE_HALTED_BY_DBG) == WDT_OPT_PAUSE_HALTED_BY_DBG) {
		dev_data->mode |= RSWDT_MR_WDDBGHLT_Msk;
	}

	regs->RSWDT_MR = dev_data->mode;
	dev_data->mode_set = true;

	return 0;
}

static int wdt_mchp_rswdt_install_timeout(const struct device *dev,
					  const struct wdt_timeout_cfg *timeout_config)
{
	const struct wdt_mchp_rswdt_config *dev_config = dev->config;
	uint32_t wdt_mode = 0u;
	int32_t max_timeout_value = 0u;
	uint32_t rate = 0u;
	int ret = 0;

	struct wdt_mchp_rswdt_data *dev_data = dev->data;

	if (dev_data->timeout_valid) {
		LOG_ERR("No more timeouts can be installed");
		return -ENOMEM;
	}

	/* RSWDT does not support a time window */
	if (timeout_config->window.min > 0) {
		return -EINVAL;
	}

	ret = clock_control_get_rate(
		dev_config->clock,
		(clock_control_subsys_t)MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_SLOW_CLK, 0), &rate);
	if (ret != 0) {
		LOG_ERR("Unable to fetch the slow clock rate");
		return ret;
	}

	/* Convert time to cycles. */
	max_timeout_value = wdt_mchp_rswdt_convert_timeout(timeout_config->window.max, rate);
	if (max_timeout_value < 0) {
		return -EINVAL;
	}

	LOG_DBG("Configured timeout value: %i", max_timeout_value);

	if (timeout_config->callback) {
		wdt_mode |= RSWDT_MR_WDFIEN_Msk; /* WDT fault interrupt. */
		dev_data->cb = timeout_config->callback;
	}

	switch (timeout_config->flags) {
	case WDT_FLAG_RESET_SOC:
		wdt_mode |= RSWDT_MR_WDRSTEN_Msk; /* WDT reset enable */
		break;
	case WDT_FLAG_RESET_NONE:
		/* A Watchdog fault (underflow or error) asserts interrupt. */
		if (timeout_config->callback == NULL) {
			LOG_ERR("Invalid(NULL) ISR callback passed");
			return -EINVAL;
		}
		break;
	default:
		LOG_ERR("Unsupported watchdog config Flag");
		return -ENOTSUP;
	}

	dev_data->mode = wdt_mode | RSWDT_MR_WDV(max_timeout_value) | RSWDT_MR_ALLONES_Msk;

	dev_data->timeout_valid = true;

	return 0;
}

static int wdt_mchp_rswdt_feed(const struct device *dev, int channel_id)

{
	const struct wdt_mchp_rswdt_config *dev_config = dev->config;
	rswdt_registers_t *const regs = dev_config->regs;

	/* On watchdog restart the Watchdog counter is immediately
	 * reloaded/fed with the 12-bit watchdog counter
	 * value from WDT_MR and restarted
	 */
	regs->RSWDT_CR = RSWDT_CR_KEY_PASSWD | RSWDT_CR_WDRSTT_Msk;

	return 0;
}

static DEVICE_API(wdt, wdt_mchp_rswdt_driver_api) = {
	.setup = wdt_mchp_rswdt_setup,
	.disable = wdt_mchp_rswdt_disable,
	.install_timeout = wdt_mchp_rswdt_install_timeout,
	.feed = wdt_mchp_rswdt_feed,
};

static int wdt_mchp_rswdt_init(const struct device *dev)
{
	const struct wdt_mchp_rswdt_config *dev_config = dev->config;

	dev_config->irq_config(dev);

	return 0;
}

static void wdt_mchp_rswdt_isr(const struct device *dev)
{
	const struct wdt_mchp_rswdt_config *dev_config = dev->config;
	rswdt_registers_t *const regs = dev_config->regs;
	struct wdt_mchp_rswdt_data *dev_data = dev->data;

	/* Clear status bit to acknowledge interrupt by dummy read. */
	(void)regs->RSWDT_SR;

	if (dev_data->cb != NULL) {
		dev_data->cb(dev, 0);
	}
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), wdt_mchp_rswdt_isr,         \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct wdt_mchp_rswdt_config mchp_rswdt_##n##_config = {                      \
		.regs = (rswdt_registers_t *)DT_INST_REG_ADDR(n),                                  \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.irq_config = irq_config_##n,                                                      \
	};                                                                                         \
                                                                                                   \
	static struct wdt_mchp_rswdt_data mchp_rswdt_##n##_data = {0};                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, wdt_mchp_rswdt_init, NULL, &mchp_rswdt_##n##_data,                \
			      &mchp_rswdt_##n##_config, PRE_KERNEL_1,                              \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_mchp_rswdt_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
