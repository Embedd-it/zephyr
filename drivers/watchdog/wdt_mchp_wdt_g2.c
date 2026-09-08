/*
 * SPDX-FileCopyrightText: Copyright (C) 2017 Intel Deutschland GmbH
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip watchdog timer (WDT) driver.
 *
 */

#define DT_DRV_COMPAT microchip_wdt_g2_wdt

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/irq.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_mchp_wdt_g2, CONFIG_WDT_LOG_LEVEL);

/*
 * Note:
 * - Once the watchdog disable bit is set, it cannot be cleared till next
 *   power reset, i.e, the watchdog cannot be started once stopped.
 * - Since the MCU boots with WDT enabled, the CONFIG_WDT_DISABLE_AT_BOOT
 *   is set default at boot and watchdog module is disabled in the MCU for
 *   systems that don't need watchdog functionality.
 * - If the application needs to use the watchdog in the system, then
 *   CONFIG_WDT_DISABLE_AT_BOOT must be unset in the app's config file
 */

#define PRESCALER     128u
#define WDT_MAX_VALUE 4095u

struct wdt_mchp_config {
	/* Register block */
	wdt_registers_t *regs;

	/* Clock driver */
	const struct device *clock;

	/* Interrupt configuration function */
	void (*irq_config)(const struct device *dev);
};

struct wdt_mchp_data {
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
static int wdt_mchp_convert_timeout(uint32_t timeout, uint32_t sclk)
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

	return WDT_MR_WDV(timeout / min);
}

static void wdt_mchp_hw_disable(const struct device *dev)
{
	const struct wdt_mchp_config *dev_config = dev->config;
	wdt_registers_t *const regs = dev_config->regs;
	struct wdt_mchp_data *dev_data = dev->data;

	/* Watchdog Mode register is 'write-once' only register.
	 * Once disabled, it cannot be enabled until the device is reset
	 */
	regs->WDT_MR |= WDT_MR_WDDIS_Msk;
	dev_data->mode_set = true;
}

static int wdt_mchp_disable(const struct device *dev)
{
	struct wdt_mchp_data *dev_data = dev->data;

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

	wdt_mchp_hw_disable(dev);

	return 0;
}

static int wdt_mchp_setup(const struct device *dev, uint8_t options)
{
	const struct wdt_mchp_config *dev_config = dev->config;
	wdt_registers_t *const regs = dev_config->regs;
	struct wdt_mchp_data *dev_data = dev->data;

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
		dev_data->mode |= WDT_MR_WDIDLEHLT_Msk;
	}

	if ((options & WDT_OPT_PAUSE_HALTED_BY_DBG) == WDT_OPT_PAUSE_HALTED_BY_DBG) {
		dev_data->mode |= WDT_MR_WDDBGHLT_Msk;
	}

	regs->WDT_MR = dev_data->mode;
	dev_data->mode_set = true;

	return 0;
}

static int wdt_mchp_install_timeout(const struct device *dev,
				    const struct wdt_timeout_cfg *timeout_config)
{
	const struct wdt_mchp_config *dev_config = dev->config;
	uint32_t wdt_mode = 0u;
	int32_t min_timeout_value = 0u;
	int32_t max_timeout_value = 0u;
	uint32_t rate = 0u;
	int ret = 0;

	struct wdt_mchp_data *dev_data = dev->data;

	if (dev_data->timeout_valid) {
		LOG_ERR("No more timeouts can be installed");
		return -ENOMEM;
	}

	if (timeout_config->window.min >= timeout_config->window.max) {
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
	max_timeout_value = wdt_mchp_convert_timeout(timeout_config->window.max, rate);
	if (max_timeout_value < 0) {
		return -EINVAL;
	}

	min_timeout_value = wdt_mchp_convert_timeout(
		timeout_config->window.max - timeout_config->window.min, rate);
	if (min_timeout_value < 0) {
		return -EINVAL;
	}

	if (timeout_config->callback) {
		wdt_mode |= WDT_MR_WDFIEN_Msk; /* WDT fault interrupt. */
		dev_data->cb = timeout_config->callback;
	}

	switch (timeout_config->flags) {
	case WDT_FLAG_RESET_SOC:
		wdt_mode |= WDT_MR_WDRSTEN_Msk; /* WDT reset enable */
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

	dev_data->mode = wdt_mode | WDT_MR_WDV(max_timeout_value) | WDT_MR_WDD(min_timeout_value);

	dev_data->timeout_valid = true;

	return 0;
}

static int wdt_mchp_feed(const struct device *dev, int channel_id)

{
	const struct wdt_mchp_config *dev_config = dev->config;
	wdt_registers_t *const regs = dev_config->regs;

	/* On watchdog restart the Watchdog counter is immediately
	 * reloaded/fed with the 12-bit watchdog counter
	 * value from WDT_MR and restarted
	 */
	regs->WDT_CR = WDT_CR_KEY_PASSWD | WDT_CR_WDRSTT_Msk;

	return 0;
}

static DEVICE_API(wdt, wdt_mchp_driver_api) = {
	.setup = wdt_mchp_setup,
	.disable = wdt_mchp_disable,
	.install_timeout = wdt_mchp_install_timeout,
	.feed = wdt_mchp_feed,
};

static int wdt_mchp_init(const struct device *dev)
{
	const struct wdt_mchp_config *dev_config = dev->config;

#if defined(CONFIG_WDT_DISABLE_AT_BOOT)
	wdt_mchp_hw_disable(dev);
#endif /* CONFIG_WDT_DISABLE_AT_BOOT */

	dev_config->irq_config(dev);

	return 0;
}

static void wdt_mchp_isr(const struct device *dev)
{
	const struct wdt_mchp_config *dev_config = dev->config;
	wdt_registers_t *const regs = dev_config->regs;
	struct wdt_mchp_data *dev_data = dev->data;

	/* Clear status bit to acknowledge interrupt by dummy read. */
	(void)regs->WDT_SR;

	if (dev_data->cb != NULL) {
		dev_data->cb(dev, 0);
	}
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), wdt_mchp_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct wdt_mchp_config mchp_wdt_##n##_config = {                              \
		.regs = (wdt_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.irq_config = irq_config_##n,                                                      \
	};                                                                                         \
                                                                                                   \
	static struct wdt_mchp_data mchp_wdt_##n##_data = {0};                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, wdt_mchp_init, NULL, &mchp_wdt_##n##_data,                        \
			      &mchp_wdt_##n##_config, PRE_KERNEL_1,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_mchp_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
