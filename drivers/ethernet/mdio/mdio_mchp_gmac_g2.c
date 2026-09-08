/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 IP-Logix Inc.
 * SPDX-FileCopyrightText: Copyright 2023 NXP
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip MCU family Ethernet MDIO/GMAC driver.
 */

#define DT_DRV_COMPAT microchip_gmac_g2_mdio

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/mdio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mdio_mchp_gmac_g2, CONFIG_MDIO_LOG_LEVEL);

struct mdio_mchp_gmac_dev_data {
	struct k_sem sem;
};

struct mdio_mchp_gmac_dev_config {
	gmac_registers_t *const regs;
	const struct pinctrl_dev_config *pcfg;
	const struct device *clock;
	clock_control_subsys_t clock_cfg;
};

static int mdio_mchp_gmac_transfer(const struct device *dev, uint8_t prtad, uint8_t regad,
				   enum mdio_opcode op, bool c45, uint16_t data_in,
				   uint16_t *data_out)
{
	const struct mdio_mchp_gmac_dev_config *const dev_config = dev->config;
	struct mdio_mchp_gmac_dev_data *const dev_data = dev->data;
	int timeout = 50;

	k_sem_take(&dev_data->sem, K_FOREVER);

	/* Write mdio transaction */
	dev_config->regs->GMAC_MAN = (c45 ? 0U : GMAC_MAN_CLTTO_Msk) | GMAC_MAN_OP(op) |
				     GMAC_MAN_WTN(0x02) | GMAC_MAN_PHYA(prtad) |
				     GMAC_MAN_REGA(regad) | GMAC_MAN_DATA(data_in);

	/* Wait until done */
	while (!(dev_config->regs->GMAC_NSR & GMAC_NSR_IDLE_Msk)) {
		if (timeout-- == 0U) {
			LOG_ERR("transfer timedout %s", dev->name);
			k_sem_give(&dev_data->sem);

			return -ETIMEDOUT;
		}

		k_sleep(K_MSEC(5));
	}

	if (data_out) {
		*data_out = dev_config->regs->GMAC_MAN & GMAC_MAN_DATA_Msk;
	}

	k_sem_give(&dev_data->sem);

	return 0;
}

static int mdio_mchp_gmac_read(const struct device *dev, uint8_t prtad, uint8_t regad,
			       uint16_t *data)
{
	return mdio_mchp_gmac_transfer(dev, prtad, regad, MDIO_OP_C22_READ, false, 0, data);
}

static int mdio_mchp_gmac_write(const struct device *dev, uint8_t prtad, uint8_t regad,
				uint16_t data)
{
	return mdio_mchp_gmac_transfer(dev, prtad, regad, MDIO_OP_C22_WRITE, false, data, NULL);
}

static int mdio_mchp_gmac_read_c45(const struct device *dev, uint8_t prtad, uint8_t devad,
				   uint16_t regad, uint16_t *data)
{
	int ret;

	ret = mdio_mchp_gmac_transfer(dev, prtad, devad, MDIO_OP_C45_ADDRESS, true, regad, NULL);
	if (!ret) {
		ret = mdio_mchp_gmac_transfer(dev, prtad, devad, MDIO_OP_C45_READ, true, 0, data);
	}

	return ret;
}

static int mdio_mchp_gmac_write_c45(const struct device *dev, uint8_t prtad, uint8_t devad,
				    uint16_t regad, uint16_t data)
{
	int ret;

	ret = mdio_mchp_gmac_transfer(dev, prtad, devad, MDIO_OP_C45_ADDRESS, true, regad, NULL);
	if (!ret) {
		ret = mdio_mchp_gmac_transfer(dev, prtad, devad, MDIO_OP_C45_WRITE, true, data,
					      NULL);
	}

	return ret;
}

static int mdio_mchp_gmac_initialize(const struct device *dev)
{
	const struct mdio_mchp_gmac_dev_config *const dev_config = dev->config;
	struct mdio_mchp_gmac_dev_data *const dev_data = dev->data;
	int ret;

	k_sem_init(&dev_data->sem, 1, 1);

	/* Enable GMAC module's clock */
	ret = clock_control_on(dev_config->clock, dev_config->clock_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to enable GMAC/MDIO clock (%d)", ret);
		return -ENODEV;
	}

	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret >= 0) {
		/* Enable MDIO */
		dev_config->regs->GMAC_NCR |= GMAC_NCR_MPE_Msk;
	}

	return ret;
}

static DEVICE_API(mdio, mdio_mchp_gmac_driver_api) = {
	.read = mdio_mchp_gmac_read,
	.write = mdio_mchp_gmac_write,
	.read_c45 = mdio_mchp_gmac_read_c45,
	.write_c45 = mdio_mchp_gmac_write_c45,
};

#define MDIO_MCHP_GMAC_G2_CONFIG(n)                                                                \
	static const struct mdio_mchp_gmac_dev_config mchp_gmac_mdio_##n##_config = {              \
		.regs = (gmac_registers_t *)DT_REG_ADDR(DT_INST_PARENT(n)),                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.clock = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_INST_PARENT(n))),                         \
		.clock_cfg =                                                                       \
			(clock_control_subsys_t)DT_CLOCKS_CELL(DT_INST_PARENT(n), subsystem)};

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	MDIO_MCHP_GMAC_G2_CONFIG(n);                                                               \
	static struct mdio_mchp_gmac_dev_data mchp_gmac_mdio_##n##_data;                           \
	DEVICE_DT_INST_DEFINE(n, &mdio_mchp_gmac_initialize, NULL, &mchp_gmac_mdio_##n##_data,     \
			      &mchp_gmac_mdio_##n##_config, POST_KERNEL,                           \
			      CONFIG_MDIO_INIT_PRIORITY, &mdio_mchp_gmac_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
