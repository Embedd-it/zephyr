/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file dac_mchp_dacc_g1.c
 * @brief Microchip DACC (DAC) controller driver.
 */

#define DT_DRV_COMPAT microchip_dacc_g1_dac

#include <errno.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dac_mchp_dacc_g1, CONFIG_DAC_LOG_LEVEL);

#define DAC_CHANNEL_NO 2

/* Device constant configuration parameters */
struct dac_mchp_dacc_config {
	dacc_registers_t *regs;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock;
	clock_control_subsys_t clock_cfg;
	void (*irq_config)(const struct device *dev);
	uint8_t irq_id;
	uint8_t prescaler;
};

struct dac_mchp_dacc_channel {
	struct k_sem done;
};

/* Device run time data */
struct dac_mchp_dacc_data {
	struct dac_mchp_dacc_channel dac_channels[DAC_CHANNEL_NO];
};

static void dac_mchp_dacc_isr(const struct device *dev)
{
	const struct dac_mchp_dacc_config *const dev_config = dev->config;
	struct dac_mchp_dacc_data *const dev_data = dev->data;
	dacc_registers_t *const regs = dev_config->regs;
	uint32_t int_stat;

	/* Retrieve interrupt status */
	int_stat = regs->DACC_ISR & regs->DACC_IMR;

	if ((int_stat & DACC_ISR_TXRDY0_Msk) != 0) {
		/* Disable Transmit Ready Interrupt */
		regs->DACC_IDR = DACC_IDR_TXRDY0_Msk;
		k_sem_give(&dev_data->dac_channels[0].done);
	}
	if ((int_stat & DACC_ISR_TXRDY1_Msk) != 0) {
		/* Disable Transmit Ready Interrupt */
		regs->DACC_IDR = DACC_IDR_TXRDY1_Msk;
		k_sem_give(&dev_data->dac_channels[1].done);
	}
}

static int dac_mchp_dacc_channel_setup(const struct device *dev,
				       const struct dac_channel_cfg *channel_cfg)
{
	const struct dac_mchp_dacc_config *const dev_config = dev->config;
	dacc_registers_t *const regs = dev_config->regs;

	if (channel_cfg->channel_id >= DAC_CHANNEL_NO) {
		return -EINVAL;
	}
	if (channel_cfg->resolution != 12) {
		return -ENOTSUP;
	}

	if (channel_cfg->internal) {
		return -ENOTSUP;
	}

	/* Enable Channel */
	regs->DACC_CHER = DACC_CHER_CH0_Msk << channel_cfg->channel_id;

	return 0;
}

static int dac_mchp_dacc_write_value(const struct device *dev, uint8_t channel, uint32_t value)
{
	struct dac_mchp_dacc_data *const dev_data = dev->data;
	const struct dac_mchp_dacc_config *const dev_config = dev->config;
	dacc_registers_t *const regs = dev_config->regs;

	if (channel >= DAC_CHANNEL_NO) {
		return -EINVAL;
	}

	if (regs->DACC_IMR & (DACC_IMR_TXRDY0_Msk << channel)) {
		/* Attempting to send data on channel that's already in use */
		return -EINVAL;
	}

	if (value >= BIT(12)) {
		LOG_ERR("value %d out of range", value);
		return -EINVAL;
	}

	k_sem_take(&dev_data->dac_channels[channel].done, K_FOREVER);

	/* Trigger conversion */
	regs->DACC_CDR[channel] = DACC_CDR_DATA0(value);

	/* Enable Transmit Ready Interrupt */
	regs->DACC_IER = DACC_IER_TXRDY0_Msk << channel;

	return 0;
}

static int dac_mchp_dacc_initialize(const struct device *dev)
{
	const struct dac_mchp_dacc_config *const dev_config = dev->config;
	struct dac_mchp_dacc_data *const dev_data = dev->data;
	dacc_registers_t *const regs = dev_config->regs;
	int ret;

	/* Configure interrupts */
	dev_config->irq_config(dev);

	/* Initialize semaphores */
	for (size_t i = 0; i < ARRAY_SIZE(dev_data->dac_channels); i++) {
		k_sem_init(&dev_data->dac_channels[i].done, 1, 1);
	}

	/* Enable DAC clock in PMC */
	ret = clock_control_on(dev_config->clock, dev_config->clock_cfg);
	if (ret < 0) {
		return -ENODEV;
	}

	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Set Mode Register */
	regs->DACC_MR = DACC_MR_PRESCALER(dev_config->prescaler);

	/* Enable module's IRQ */
	irq_enable(dev_config->irq_id);

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

static DEVICE_API(dac, dac_mchp_dacc_driver_api) = {
	.channel_setup = dac_mchp_dacc_channel_setup,
	.write_value = dac_mchp_dacc_write_value,
};

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void mchp_dacc_##n##_irq_config(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), dac_mchp_dacc_isr,          \
			    DEVICE_DT_INST_GET(n), 0);                                             \
	}                                                                                          \
                                                                                                   \
	static const struct dac_mchp_dacc_config mchp_dacc_##n##_config = {                        \
		.regs = (dacc_registers_t *)DT_INST_REG_ADDR(n),                                   \
		.irq_config = mchp_dacc_##n##_irq_config,                                          \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_cfg = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),            \
		.irq_id = DT_INST_IRQN(n),                                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n)};                                        \
                                                                                                   \
	static struct dac_mchp_dacc_data mchp_dacc_##n##_data;                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, dac_mchp_dacc_initialize, NULL, &mchp_dacc_##n##_data,            \
			      &mchp_dacc_##n##_config, POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,      \
			      &dac_mchp_dacc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
