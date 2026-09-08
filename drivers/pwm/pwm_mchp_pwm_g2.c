/*
 * SPDX-FileCopyrightText: Copyright (c) 2019 Aurelien Jarno
 * SPDX-FileCopyrightText: Copyright (c) 2021 Linaro Limited
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip PWM driver.
 *
 */

#define DT_DRV_COMPAT microchip_pwm_g2_pwm

#include <zephyr/device.h>
#include <errno.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pwm/pwm_utils.h>
#include <zephyr/spinlock.h>

#if defined(CONFIG_PWM_EVENT)
#include <zephyr/irq.h>
#endif /* CONFIG_PWM_EVENT */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_mchp_pwm_g2, CONFIG_PWM_LOG_LEVEL);

struct pwm_mchp_config {
	/* Register block */
	pwm_registers_t *regs;

	/* Clock driver */
	const struct device *clock;

	/* Main clock subsystem. */
	clock_control_subsys_t clock_subsys;

	/* Pin control configuration */
	const struct pinctrl_dev_config *pcfg;

	/* PWM prescaler */
	uint8_t prescaler;

	/* PWM divider */
	uint8_t divider;

#if defined(CONFIG_PWM_EVENT)
	/* Interrupt configuration function */
	void (*irq_config)(const struct device *dev);
#endif /* CONFIG_PWM_EVENT */
};

struct pwm_mchp_data {
#if defined(CONFIG_PWM_EVENT)
	/* Reference to callback for the commn PWM library */
	sys_slist_t event_callbacks;

	/* Interrupt-safe spinlock */
	struct k_spinlock lock;
#endif /* CONFIG_PWM_EVENT */
};

static int pwm_mchp_get_cycles_per_sec(const struct device *dev, uint32_t channel, uint64_t *cycles)
{
	const struct pwm_mchp_config *dev_config = dev->config;
	uint8_t prescaler = dev_config->prescaler;
	uint8_t divider = dev_config->divider;
	uint32_t rate = 0u;
	int ret = 0;

	ret = clock_control_get_rate(dev_config->clock, dev_config->clock_subsys, &rate);
	if (ret != 0) {
		LOG_ERR("Unable to fetch the clock rate");
		return ret;
	}

	if (divider == 0u) {
		LOG_ERR("Invalid divider");
		return -EINVAL;
	}

	*cycles = rate / ((1u << prescaler) * divider);

	return 0;
}

static int pwm_mchp_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
			       uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_mchp_config *dev_config = dev->config;
	pwm_registers_t *const regs = dev_config->regs;
	uint32_t cmr = 0u;

	if (channel >= PWM_CH_NUM_NUMBER) {
		return -EINVAL;
	}

	if (period_cycles == 0u) {
		return -ENOTSUP;
	}

	if (period_cycles > 0xFFFF) {
		return -ENOTSUP;
	}

	/* Select clock A */
	cmr = PWM_CMR_CPRE_CLKA;

	if ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_NORMAL) {
		cmr |= PWM_CMR_CPOL_Msk;
	}

	/* Disable the output if changing polarity (or clock) */
	if (regs->PWM_CH_NUM[channel].PWM_CMR != cmr) {
		regs->PWM_DIS = BIT(channel);

		regs->PWM_CH_NUM[channel].PWM_CMR = cmr;
		regs->PWM_CH_NUM[channel].PWM_CPRD = period_cycles;
		regs->PWM_CH_NUM[channel].PWM_CDTY = pulse_cycles;
	}
	/* Update period and pulse using the update registers, so that the
	 * change is triggered at the next PWM period.
	 */
	else {
		regs->PWM_CH_NUM[channel].PWM_CPRDUPD = period_cycles;
		regs->PWM_CH_NUM[channel].PWM_CDTYUPD = pulse_cycles;
	}

	/* Enable the output */
	regs->PWM_ENA = BIT(channel);

	return 0;
}

#if defined(CONFIG_PWM_EVENT)
static void update_interrupts(const struct device *dev)
{
	const struct pwm_mchp_config *dev_config = dev->config;
	struct pwm_mchp_data *dev_data = dev->data;
	struct pwm_event_callback *cb, *tmp;
	pwm_registers_t *const regs = dev_config->regs;
	uint32_t pwm_ier1 = 0u;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&dev_data->event_callbacks, cb, tmp, node) {
		if ((cb->event_mask & PWM_EVENT_TYPE_PERIOD) != 0u) {
			pwm_ier1 |= PWM_IER1_CHID0_Msk << cb->channel;
		}
		if ((cb->event_mask & PWM_EVENT_TYPE_FAULT) != 0u) {
			pwm_ier1 |= PWM_IER1_FCHID0_Msk << cb->channel;
		}
	}

	/* Disable all interrupts */
	regs->PWM_IDR1 = UINT32_MAX;

	/* Dummy read to clear status register */
	(void)regs->PWM_ISR1;

	/* Reenable interrupts */
	regs->PWM_IER1 = pwm_ier1;
}

static void pwm_mchp_isr(const struct device *dev)
{
	const struct pwm_mchp_config *dev_config = dev->config;
	struct pwm_mchp_data *dev_data = dev->data;
	pwm_registers_t *const regs = dev_config->regs;
	pwm_events_t events = 0u;
	uint32_t status = regs->PWM_ISR1;

	for (size_t i = 0u; i < PWM_CH_NUM_NUMBER; i++) {
		events = 0u;
		if ((status & (PWM_ISR1_CHID0_Msk << i)) != 0u) {
			events |= PWM_EVENT_TYPE_PERIOD;
		}
		if ((status & (PWM_ISR1_FCHID0_Msk << i)) != 0u) {
			events |= PWM_EVENT_TYPE_FAULT;
		}

		if (events > 0u) {
			pwm_fire_event_callbacks(&dev_data->event_callbacks, dev, i, events);
		}
	}
}

static int pwm_mchp_manage_event_callback(const struct device *dev,
					  struct pwm_event_callback *callback, bool set)
{
	struct pwm_mchp_data *dev_data = dev->data;
	int ret = 0;

	ret = pwm_manage_event_callback(&dev_data->event_callbacks, callback, set);
	if (ret != 0) {
		return ret;
	}

	K_SPINLOCK(&dev_data->lock) {
		update_interrupts(dev);
	}

	return 0;
}
#endif /* CONFIG_PWM_EVENT */

static int pwm_mchp_init(const struct device *dev)
{
	const struct pwm_mchp_config *dev_config = dev->config;
	pwm_registers_t *const regs = dev_config->regs;
	uint8_t prescaler = dev_config->prescaler;
	uint8_t divider = dev_config->divider;
	int ret = 0;

#if defined(CONFIG_PWM_EVENT)
	struct pwm_mchp_data *dev_data = dev->data;

	sys_slist_init(&dev_data->event_callbacks);
	dev_config->irq_config(dev);
#endif /* CONFIG_PWM_EVENT */

	/* Enable peripheral clock */
	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Unable to activate peripheral clock");
		return ret;
	}

	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	/* Configure the clock A that will be used by all 4 channels */
	regs->PWM_CLK = PWM_CLK_PREA(prescaler) | PWM_CLK_DIVA(divider);

	return 0;
}

static DEVICE_API(pwm, pwm_mchp_driver_api) = {
	.set_cycles = pwm_mchp_set_cycles,
	.get_cycles_per_sec = pwm_mchp_get_cycles_per_sec,
#if defined(CONFIG_PWM_EVENT)
	.manage_event_callback = pwm_mchp_manage_event_callback,
#endif /* CONFIG_PWM_EVENT */
};

#if defined(CONFIG_PWM_EVENT)
#define PWM_MCHP_INTERRUPT_DECLARE(n)                                                              \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), 0, pwm_mchp_isr, DEVICE_DT_INST_GET(n), 0);           \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}
#define PWM_MCHP_INTERRUPT_INIT(n) .irq_config = irq_config_##n,
#else
#define PWM_MCHP_INTERRUPT_DECLARE(n)
#define PWM_MCHP_INTERRUPT_INIT(n)
#endif /* CONFIG_PWM_EVENT */

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	PWM_MCHP_INTERRUPT_DECLARE(n)                                                              \
                                                                                                   \
	static const struct pwm_mchp_config mchp_pwm_##n##_config = {                              \
		.regs = (pwm_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),         \
		.prescaler = DT_INST_PROP(n, prescaler),                                           \
		.divider = DT_INST_PROP(n, divider),                                               \
		PWM_MCHP_INTERRUPT_INIT(n)};                                                       \
                                                                                                   \
	static struct pwm_mchp_data mchp_pwm_##n##_data = {};                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &pwm_mchp_init, NULL, &mchp_pwm_##n##_data,                       \
			      &mchp_pwm_##n##_config, POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,       \
			      &pwm_mchp_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
