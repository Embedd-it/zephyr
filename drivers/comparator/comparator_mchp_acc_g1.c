/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip Analog Comparator Controller (ACC) driver.
 *
 */

#define DT_DRV_COMPAT microchip_acc_g1_comparator

#include <soc.h>
#include <zephyr/drivers/comparator.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(comparator_mchp_acc_g1, CONFIG_COMPARATOR_LOG_LEVEL);

#define MASK_TIMEOUT_US 10000u

struct comparator_mchp_acc_data {
	/* Callback to be invoked on trigger */
	comparator_callback_t callback;

	/* User provided data to be passed to the callback */
	void *user_data;

	/* Driver locking mutex */
	struct k_mutex mutex;

	/* Configured trigger type */
	enum comparator_trigger trigger;

	/* Trigger interrupt flag */
	atomic_t trigger_pending;
};

struct comparator_mchp_acc_config {
	/* Register block */
	acc_registers_t *regs;

	/* Clock driver */
	const struct device *clock;

	/* Clock subsystem for this instance's own channel */
	clock_control_subsys_t clock_subsys;

	/* Pin control configuration */
	const struct pinctrl_dev_config *pcfg;

	/* Interrupt configuration function */
	void (*irq_config)(const struct device *dev);

	/* Negative input level */
	uint8_t selminus;

	/* Positive input level */
	uint8_t selplus;

	/* Whether to invert the comparator output before processing */
	bool inv;

	/* Input hysteresis */
	uint8_t hysteresis;

	/* Current selection */
	uint8_t current_selection;
};

/*
 * Read ACC_ISR. Its CE bit is cleared on read, so whichever caller's read happens to observe it
 * must latch the edge for all consumers.
 */
static inline uint32_t comparator_mchp_acc_read_isr(const struct device *dev)
{
	const struct comparator_mchp_acc_config *dev_config = dev->config;
	acc_registers_t *const regs = dev_config->regs;
	struct comparator_mchp_acc_data *dev_data = dev->data;
	uint32_t isr = regs->ACC_ISR;

	if ((isr & ACC_ISR_CE_Msk) != 0u && dev_data->callback == NULL) {
		atomic_set(&dev_data->trigger_pending, 1u);
	}

	return isr;
}

/*
 * When configuration changes, wait for the mask bit to be cleared.
 */
static inline int comparator_mchp_acc_wait_for_mask(const struct device *dev)
{
	if (!WAIT_FOR((comparator_mchp_acc_read_isr(dev) & ACC_ISR_MASK_Msk) == 0u, MASK_TIMEOUT_US,
		      k_busy_wait(1u))) {
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Update the interrupt configuration of the peripheral
 */
static void comparator_mchp_acc_update_interrupt_configuration(const struct device *dev)
{
	const struct comparator_mchp_acc_config *dev_config = dev->config;
	acc_registers_t *const regs = dev_config->regs;
	struct comparator_mchp_acc_data *dev_data = dev->data;

	switch (dev_data->trigger) {
	case COMPARATOR_TRIGGER_NONE:
		/* Disable the interrupt */
		regs->ACC_IDR = ACC_IDR_CE_Msk;
		break;
	default:
		/* Enable the interrupt */
		regs->ACC_IER = ACC_IER_CE_Msk;
		break;
	}
}

static int comparator_mchp_acc_get_output(const struct device *dev)
{
	struct comparator_mchp_acc_data *dev_data = dev->data;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	ret = (comparator_mchp_acc_read_isr(dev) & ACC_ISR_SCO_Msk) != 0u;

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int comparator_mchp_acc_set_trigger(const struct device *dev,
					   enum comparator_trigger trigger)
{
	const struct comparator_mchp_acc_config *dev_config = dev->config;
	acc_registers_t *const regs = dev_config->regs;
	struct comparator_mchp_acc_data *dev_data = dev->data;
	uint32_t mr = 0u;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	mr = regs->ACC_MR;
	mr &= ~ACC_MR_EDGETYP_Msk;

	dev_data->trigger = trigger;

	switch (trigger) {
	case COMPARATOR_TRIGGER_NONE:
		break;
	case COMPARATOR_TRIGGER_RISING_EDGE:
		mr |= ACC_MR_EDGETYP_RISING;
		regs->ACC_MR = mr;
		break;
	case COMPARATOR_TRIGGER_FALLING_EDGE:
		mr |= ACC_MR_EDGETYP_FALLING;
		regs->ACC_MR = mr;
		break;
	case COMPARATOR_TRIGGER_BOTH_EDGES:
		mr |= ACC_MR_EDGETYP_ANY;
		regs->ACC_MR = mr;
		break;
	}

	comparator_mchp_acc_update_interrupt_configuration(dev);

	ret = comparator_mchp_acc_wait_for_mask(dev);
	if (ret != 0) {
		goto error_cleanup;
	}

error_cleanup:
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int comparator_mchp_acc_set_trigger_callback(const struct device *dev,
						    comparator_callback_t callback, void *user_data)
{
	struct comparator_mchp_acc_data *dev_data = dev->data;
	unsigned int lock_key = 0u;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	lock_key = irq_lock();
	dev_data->callback = callback;
	dev_data->user_data = user_data;
	irq_unlock(lock_key);

	if (dev_data->trigger != COMPARATOR_TRIGGER_NONE) {
		/* If already pending invoke the callback */
		if (atomic_clear(&dev_data->trigger_pending) != 0u) {
			if (dev_data->callback != NULL) {
				dev_data->callback(dev, dev_data->user_data);
			}
		}
	}

	comparator_mchp_acc_update_interrupt_configuration(dev);

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

static int comparator_mchp_acc_trigger_is_pending(const struct device *dev)
{
	struct comparator_mchp_acc_data *dev_data = dev->data;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	ret = atomic_clear(&dev_data->trigger_pending) != 0u;

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int comparator_mchp_acc_init(const struct device *dev)
{
	const struct comparator_mchp_acc_config *dev_config = dev->config;
	acc_registers_t *const regs = dev_config->regs;
	int ret = 0;

	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Failed to enable peripheral clock");
		return ret;
	}

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		LOG_ERR("Unable to configure pins");
		return ret;
	}

	dev_config->irq_config(dev);

	/* Reset the peripheral */
	regs->ACC_CR = ACC_CR_SWRST_Msk;

	regs->ACC_ACR =
		ACC_ACR_HYST(dev_config->hysteresis) | ACC_ACR_ISEL(dev_config->current_selection);

	regs->ACC_MR = ACC_MR_INV(dev_config->inv) | ACC_MR_SELMINUS(dev_config->selminus) |
		       ACC_MR_SELPLUS(dev_config->selplus) | ACC_MR_ACEN_Msk;

	ret = comparator_mchp_acc_wait_for_mask(dev);
	if (ret != 0) {
		LOG_ERR("Timed out waiting for configuration to apply");
		regs->ACC_MR = 0u;
		clock_control_off(dev_config->clock, dev_config->clock_subsys);
		return ret;
	}

	return 0;
}

static DEVICE_API(comparator, comparator_mchp_acc_driver_api) = {
	.get_output = comparator_mchp_acc_get_output,
	.set_trigger = comparator_mchp_acc_set_trigger,
	.set_trigger_callback = comparator_mchp_acc_set_trigger_callback,
	.trigger_is_pending = comparator_mchp_acc_trigger_is_pending,
};

void comparator_mchp_acc_isr(const struct device *dev)
{
	struct comparator_mchp_acc_data *dev_data = dev->data;

	/* Acks CE; latches trigger_pending if no callback is registered. */
	comparator_mchp_acc_read_isr(dev);

	if (dev_data->callback != NULL) {
		/* If the callback is invoked clear the pending state */
		atomic_clear(&dev_data->trigger_pending);
		dev_data->callback(dev, dev_data->user_data);
	}
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq), DT_INST_IRQ(n, priority),               \
			    comparator_mchp_acc_isr, DEVICE_DT_INST_GET(n), 0);                    \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));                                         \
	}                                                                                          \
                                                                                                   \
	static const struct comparator_mchp_acc_config mchp_acc_##n##_config = {                   \
		.regs = (acc_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys =                                                                    \
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_IDX(n, 0, subsystem),       \
		.irq_config = irq_config_##n,                                                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.selminus = DT_INST_ENUM_IDX(n, selminus),                                         \
		.selplus = DT_INST_ENUM_IDX(n, selplus),                                           \
		.inv = DT_INST_PROP(n, invert_output),                                             \
		.hysteresis = DT_INST_ENUM_IDX(n, hysteresis),                                     \
		.current_selection = DT_INST_ENUM_IDX(n, current),                                 \
	};                                                                                         \
                                                                                                   \
	static struct comparator_mchp_acc_data mchp_acc_##n##_data = {                             \
		.mutex = Z_MUTEX_INITIALIZER(mchp_acc_##n##_data.mutex),                           \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, comparator_mchp_acc_init, NULL, &mchp_acc_##n##_data,             \
			      &mchp_acc_##n##_config, POST_KERNEL,                                 \
			      CONFIG_COMPARATOR_INIT_PRIORITY, &comparator_mchp_acc_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION);
