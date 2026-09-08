/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 Vestas Wind Systems A/S
 * SPDX-FileCopyrightText: Copyright (c) 2021 Alexander Wachter
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip CAN controller driver.
 *
 */

#define DT_DRV_COMPAT microchip_can_g2_can

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include "can_mcan.h"

LOG_MODULE_REGISTER(can_mchp_can_g2, CONFIG_CAN_LOG_LEVEL);

#define PROGRAMMABLE_CLOCK ((clock_control_subsys_t)CLOCK_MCHP_PCK5)

struct can_mchp_config {
	/* Register block */
	mcan_registers_t *regs;

	/* Message RAM */
	mem_addr_t mram;

	/* IRQ setup function */
	void (*irq_config)(const struct device *dev);

	/* Clock driver */
	const struct device *clock;

	/* Main clock subsystem. */
	clock_control_subsys_t clock_subsys;

	/* Pin configuration */
	const struct pinctrl_dev_config *pcfg;

	/* DMA base address */
	mm_reg_t dma_base;
};

static int can_mchp_mcan_read_reg(const struct device *dev, uint16_t reg, uint32_t *val)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *dev_config = mcan_config->custom;

	return can_mcan_sys_read_reg((mm_reg_t)dev_config->regs, reg, val);
}

static int can_mchp_mcan_write_reg(const struct device *dev, uint16_t reg, uint32_t val)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *dev_config = mcan_config->custom;

	return can_mcan_sys_write_reg((mm_reg_t)dev_config->regs, reg, val);
}

static int can_mchp_mcan_read_mram(const struct device *dev, uint16_t offset, void *dst, size_t len)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *dev_config = mcan_config->custom;

	return can_mcan_sys_read_mram(dev_config->mram, offset, dst, len);
}

static int can_mchp_mcan_write_mram(const struct device *dev, uint16_t offset, const void *src,
				    size_t len)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *dev_config = mcan_config->custom;

	return can_mcan_sys_write_mram(dev_config->mram, offset, src, len);
}

static int can_mchp_mcan_clear_mram(const struct device *dev, uint16_t offset, size_t len)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_mchp_config *dev_config = mcan_config->custom;

	return can_mcan_sys_clear_mram(dev_config->mram, offset, len);
}

static int can_mchp_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_mcan_config *mcan_cfg = dev->config;
	const struct can_mchp_config *dev_config = mcan_cfg->custom;

	return clock_control_get_rate(dev_config->clock, PROGRAMMABLE_CLOCK, rate);
}

static int can_mchp_init(const struct device *dev)
{
	const struct can_mcan_config *mcan_cfg = dev->config;
	const struct can_mchp_config *dev_config = mcan_cfg->custom;
	uint32_t peripheral_clock_rate = 0u;
	uint32_t programmable_clock_rate = 0u;
	uint32_t mrba = 0u;
	int ret = 0;

	/* Enable CAN clock in PMC */
	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Unable to activate peripheral clock: %i", ret);
		return ret;
	}

	ret = clock_control_get_rate(dev_config->clock, dev_config->clock_subsys,
				     &peripheral_clock_rate);
	if (ret != 0) {
		LOG_ERR("Unable to get the peripheral clock rate: %i", ret);
		return ret;
	}

	ret = clock_control_on(dev_config->clock, PROGRAMMABLE_CLOCK);
	if (ret != 0) {
		LOG_ERR("Unable to activate programmable clock: %i", ret);
		return ret;
	}

	ret = clock_control_get_rate(dev_config->clock, PROGRAMMABLE_CLOCK,
				     &programmable_clock_rate);
	if (ret != 0) {
		LOG_ERR("Unable to get the programmable clock rate: %i", ret);
		return ret;
	}

	if (programmable_clock_rate > peripheral_clock_rate) {
		LOG_ERR("The peripheral clock must be faster than the CAN clock");
		return -EINVAL;
	}

	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return ret;
	}

	/* get actual message ram base address */
	mrba = dev_config->mram & 0xFFFF0000u;

	/* keep lower 16bit; update DMA Base Register */
	sys_write32((sys_read32(dev_config->dma_base) & 0x0000FFFFu) | mrba, dev_config->dma_base);

	ret = can_mcan_configure_mram(dev, mrba, dev_config->mram);
	if (ret != 0) {
		return ret;
	}

	ret = can_mcan_init(dev);
	if (ret != 0) {
		return ret;
	}

	dev_config->irq_config(dev);

	return ret;
}

static DEVICE_API(can, can_mchp_driver_api) = {
	.get_capabilities = can_mcan_get_capabilities,
	.start = can_mcan_start,
	.stop = can_mcan_stop,
	.set_mode = can_mcan_set_mode,
	.set_timing = can_mcan_set_timing,
	.send = can_mcan_send,
	.add_rx_filter = can_mcan_add_rx_filter,
	.remove_rx_filter = can_mcan_remove_rx_filter,
	.get_state = can_mcan_get_state,
#if defined(CONFIG_CAN_MANUAL_RECOVERY_MODE)
	.recover = can_mcan_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
	.get_core_clock = can_mchp_get_core_clock,
	.get_max_filters = can_mcan_get_max_filters,
	.timing_min = CAN_MCAN_TIMING_MIN_INITIALIZER,
	.timing_max = CAN_MCAN_TIMING_MAX_INITIALIZER,
#if defined(CONFIG_CAN_FD_MODE)
	.set_timing_data = can_mcan_set_timing_data,
	.timing_data_min = CAN_MCAN_TIMING_DATA_MIN_INITIALIZER,
	.timing_data_max = CAN_MCAN_TIMING_DATA_MAX_INITIALIZER,
#endif /* CONFIG_CAN_FD_MODE */
};

static const struct can_mcan_ops can_mcan_ops = {
	.read_reg = can_mchp_mcan_read_reg,
	.write_reg = can_mchp_mcan_write_reg,
	.read_mram = can_mchp_mcan_read_mram,
	.write_mram = can_mchp_mcan_write_mram,
	.clear_mram = can_mchp_mcan_clear_mram,
};

#define CAN_MCHP_HAS_DMA(n) DT_INST_REG_HAS_NAME(n, dma_base)
#define CAN_MCHP_HAS_MSG(n) DT_INST_REG_HAS_NAME(n, message_ram)

#define CAN_MCHP_MRAM_BUF(n) CAN_MCAN_DT_INST_MRAM_DEFINE(n, can_mchp_mram_##n);
#define CAN_MCHP_MRAM_DMA(n) .mram = (mem_addr_t)POINTER_TO_UINT(&can_mchp_mram_##n),
#define CAN_MCHP_MRAM_MSG(n) .mram = (mem_addr_t)CAN_MCAN_DT_INST_MRAM_ADDR(n),
#define CAN_MCHP_DMA_BASE(n) .dma_base = (mm_reg_t)DT_INST_REG_ADDR_BY_NAME(n, dma_base),

#define CAN_MCHP_MRAM_BUF_DEFINE(n) IF_ENABLED(CAN_MCHP_HAS_DMA(n), (CAN_MCHP_MRAM_BUF(n)))
#define CAN_MCHP_MRAM_DMA_INIT(n)   IF_ENABLED(CAN_MCHP_HAS_DMA(n), (CAN_MCHP_MRAM_DMA(n)))
#define CAN_MCHP_MRAM_MSG_INIT(n)   IF_ENABLED(CAN_MCHP_HAS_MSG(n), (CAN_MCHP_MRAM_MSG(n)))
#define CAN_MCHP_DMA_BASE_INIT(n)   IF_ENABLED(CAN_MCHP_HAS_DMA(n), (CAN_MCHP_DMA_BASE(n)))

#define CAN_MCHP_MEMORY_DEFINE(n)                                                                  \
	CAN_MCHP_MRAM_DMA_INIT(n)                                                                  \
	CAN_MCHP_MRAM_MSG_INIT(n)                                                                  \
	CAN_MCHP_DMA_BASE_INIT(n)

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	CAN_MCAN_DT_INST_BUILD_ASSERT_MRAM_CFG(n);                                                 \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, int0, irq),                                     \
			    DT_INST_IRQ_BY_NAME(n, int0, priority), can_mcan_line_0_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, int0, irq));                                     \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, int1, irq),                                     \
			    DT_INST_IRQ_BY_NAME(n, int1, priority), can_mcan_line_1_isr,           \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQ_BY_NAME(n, int1, irq));                                     \
	}                                                                                          \
                                                                                                   \
	CAN_MCAN_DT_INST_CALLBACKS_DEFINE(n, can_mchp_cbs_##n);                                    \
	CAN_MCHP_MRAM_BUF_DEFINE(n)                                                                \
                                                                                                   \
	static const struct can_mchp_config mchp_can_##n##_config = {                              \
		.regs = (mcan_registers_t *)CAN_MCAN_DT_INST_MCAN_ADDR(n),                         \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.irq_config = irq_config_##n,                                                      \
		CAN_MCHP_MEMORY_DEFINE(n)};                                                        \
                                                                                                   \
	static const struct can_mcan_config can_mcan_cfg_##n = CAN_MCAN_DT_CONFIG_INST_GET(        \
		n, &mchp_can_##n##_config, &can_mcan_ops, &can_mchp_cbs_##n);                      \
                                                                                                   \
	CAN_MCAN_DATA_DEFINE(can_mcan_data_##n, NULL);                                             \
                                                                                                   \
	CAN_DEVICE_DT_INST_DEFINE(n, can_mchp_init, NULL, &can_mcan_data_##n, &can_mcan_cfg_##n,   \
				  POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, &can_mchp_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
