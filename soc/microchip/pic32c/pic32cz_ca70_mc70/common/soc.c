/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip PIC32CZ CA70/MC70 MCU family initialization code
 *
 * This file provides routines to initialize and support board-level hardware
 * for the Microchip PIC32CZ CA70/MC70 MCU family.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/cache.h>
#include <zephyr/arch/cache.h>
#include <zephyr/logging/log.h>
#include <soc.h>

#define LOG_LEVEL CONFIG_SOC_LOG_LEVEL
LOG_MODULE_REGISTER(soc);

void soc_early_init_hook(void)
{
	efc_registers_t *efc_regs = (efc_registers_t *)0x400E0C00;

	efc_regs->EEFC_FMR = EEFC_FMR_FWS(6) | EEFC_FMR_CLOE_Msk;
}

void soc_reset_hook(void)
{
	/*
	 * DTCM is enabled by default at reset, therefore we have to disable it first to get the
	 * caches into a state where then the sys_cache*-functions can enable them.
	 */
	SCB_InvalidateDCache();
	SCB_DisableDCache();

	/* Enable the caches. */
	sys_cache_instr_enable();
	sys_cache_data_enable();
}
