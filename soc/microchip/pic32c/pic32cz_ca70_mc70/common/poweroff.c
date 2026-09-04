/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 Bjarki Arge Andreasen
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/poweroff.h>
#include <soc.h>

/*
 * Poweroff will make the chip enter the backup low-power mode, which
 * achieves the lowest possible power consumption. Wakeup from this mode
 * requires enabling a wakeup source or input, or power cycling the device.
 */
void z_sys_poweroff(void)
{
	/* To enter backup mode, first set SLEEPDEEP */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	/* Then set VROFF */
	SUPC_REGS->SUPC_CR = SUPC_CR_KEY_PASSWD | SUPC_CR_VROFF_STOP_VREG;

	/* At this point the device is irrecoverably shut down (until a power cycle) */
	__WFE();
	__WFI();

	CODE_UNREACHABLE;
}
