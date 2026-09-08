/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip Reset Controller (RSTC) driver.
 *
 */

#define DT_DRV_COMPAT microchip_rstc_g2_reset

#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/reset/mchp_rstc_g2.h>

#define RESET_WAIT_TIME_US 10000000u

struct reset_mchp_rstc_config {
	/* Register block */
	rstc_registers_t *regs;

	/* Reset pulse duration */
	uint32_t erstl;

	/* Generate a user reset on low NRST */
	bool user_reset;
};

static int reset_mchp_rstc_status(const struct device *dev, uint32_t id, uint8_t *status)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);
	ARG_UNUSED(status);

	return -ENOTSUP;
}

static int reset_mchp_rstc_line_assert(const struct device *dev, uint32_t id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	return -ENOTSUP;
}

static int reset_mchp_rstc_line_deassert(const struct device *dev, uint32_t id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	return -ENOTSUP;
}

static int reset_mchp_rstc_line_toggle(const struct device *dev, uint32_t id)
{
	const struct reset_mchp_rstc_config *dev_config = dev->config;
	rstc_registers_t *const regs = dev_config->regs;

	switch (id) {
	case RSTC_G2_PROCESSOR_RESET:
		regs->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_PROCRST_Msk;
		break;
	case RSTC_G2_EXTERNAL_RESET:
		regs->RSTC_CR = RSTC_CR_KEY_PASSWD | RSTC_CR_EXTRST_Msk;
		break;
	default:
		return -EINVAL;
	}

	/* Wait for the reset event to be over */
	if (!WAIT_FOR(((regs->RSTC_SR & RSTC_SR_SRCMP_Msk) == 0), RESET_WAIT_TIME_US, (void)NULL)) {
		return -ETIMEDOUT;
	}

	return 0;
}

static DEVICE_API(reset, reset_mchp_rstc_driver_api) = {
	.status = reset_mchp_rstc_status,
	.line_assert = reset_mchp_rstc_line_assert,
	.line_deassert = reset_mchp_rstc_line_deassert,
	.line_toggle = reset_mchp_rstc_line_toggle,
};

static int reset_mchp_rstc_init(const struct device *dev)
{
	const struct reset_mchp_rstc_config *dev_config = dev->config;
	rstc_registers_t *const regs = dev_config->regs;

	regs->RSTC_MR = RSTC_MR_KEY_PASSWD | RSTC_MR_URSTEN(dev_config->user_reset) |
			RSTC_MR_ERSTL(dev_config->erstl);

	return 0;
}

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
	     "Only one Microchip RSTC g2 instance is supported.");

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	static const struct reset_mchp_rstc_config mchp_rstc_##n##_config = {                      \
		.regs = (rstc_registers_t *)DT_INST_REG_ADDR(n),                                   \
		.erstl = DT_INST_PROP(n, erstl),                                                   \
		.user_reset = DT_INST_PROP(n, user_reset),                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, reset_mchp_rstc_init, NULL, NULL, &mchp_rstc_##n##_config,        \
			      PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                    \
			      &reset_mchp_rstc_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
