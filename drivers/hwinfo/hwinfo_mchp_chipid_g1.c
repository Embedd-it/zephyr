/*
 * SPDX-FileCopyrightText: Copyright (c)-2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip hardware info (chip ID / reset cause) driver.
 *
 */

#include <soc.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hwinfo_mchp_chipid_g1, CONFIG_HWINFO_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_chipid_g1_hwinfo)
#define HWINFO_INST DT_COMPAT_GET_ANY_STATUS_OKAY(microchip_chipid_g1_hwinfo)

ssize_t z_impl_hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	chipid_registers_t *const regs = (chipid_registers_t *)DT_REG_ADDR_BY_IDX(HWINFO_INST, 0);
	uint32_t chipid = 0u;

	if (buffer == NULL || length == 0u) {
		return -EINVAL;
	}

	chipid = regs->CHIPID_CIDR;
	buffer[0] = (chipid >> 24u) & 0xFFu;

	if (length > 1u) {
		buffer[1] = (chipid >> 16u) & 0xFFu;
	} else {
		return 1u;
	}

	if (length > 2u) {
		buffer[2] = (chipid >> 8u) & 0xFFu;
	} else {
		return 2u;
	}

	if (length > 3u) {
		buffer[3] = chipid & 0xFFu;
	} else {
		return 3u;
	}

	if (length > 4u) {
		buffer[4] = regs->CHIPID_EXID & 0xFFu;
	} else {
		return 4u;
	}

	return 5u;
}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(microchip_chipid_g1_hwinfo) */

#if DT_HAS_COMPAT_STATUS_OKAY(microchip_rstc_g2_reset)
#define RSTC_INST DT_COMPAT_GET_ANY_STATUS_OKAY(microchip_rstc_g2_reset)

int z_impl_hwinfo_get_supported_reset_cause(uint32_t *supported)
{
	*supported = RESET_POR | RESET_PIN | RESET_WATCHDOG | RESET_SOFTWARE | RESET_USER |
		     RESET_LOW_POWER_WAKE;

	return 0;
}

int z_impl_hwinfo_get_reset_cause(uint32_t *cause)
{
	rstc_registers_t *const regs = (rstc_registers_t *)DT_REG_ADDR_BY_IDX(RSTC_INST, 0);
	uint8_t rcause = (regs->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos;

	if (cause == NULL) {
		LOG_ERR("Invalid argument: NULL pointer passed");
		return -EINVAL;
	}

	switch (rcause) {
	case RSTC_SR_RSTTYP_GENERAL_RST_Val:
		*cause = RESET_POR;
		break;
	case RSTC_SR_RSTTYP_BACKUP_RST_Val:
		*cause = RESET_LOW_POWER_WAKE;
		break;
	case RSTC_SR_RSTTYP_WDT_RST_Val:
		*cause = RESET_WATCHDOG;
		break;
	case RSTC_SR_RSTTYP_SOFT_RST_Val:
		*cause = RESET_SOFTWARE;
		break;
	case RSTC_SR_RSTTYP_USER_RST_Val:
		*cause = RESET_USER | RESET_PIN;
		break;
	default:
		*cause = 0u;
		break;
	}

	return 0;
}
#endif /* DT_HAS_COMPAT_STATUS_OKAY(microchip_rstc_g2_reset) */
