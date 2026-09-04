/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOC_MICROCHIP_PIC32CZ_CA70_MC70_SOC_H_
#define SOC_MICROCHIP_PIC32CZ_CA70_MC70_SOC_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>

#if defined(CONFIG_SOC_PIC32CZ2051CA70064)
#include <pic32cz2051ca70064.h>
#elif defined(CONFIG_SOC_PIC32CZ2051CA70100)
#include <pic32cz2051ca70100.h>
#elif defined(CONFIG_SOC_PIC32CZ2051CA70144)
#include <pic32cz2051ca70144.h>
#elif defined(CONFIG_SOC_PIC32CZ1038CA70064)
#include <pic32cz1038ca70064.h>
#elif defined(CONFIG_SOC_PIC32CZ1038CA70100)
#include <pic32cz1038ca70100.h>
#elif defined(CONFIG_SOC_PIC32CZ1038CA70144)
#include <pic32cz1038ca70144.h>
#elif defined(CONFIG_SOC_PIC32CZ5125CA70064)
#include <pic32cz5125ca70064.h>
#elif defined(CONFIG_SOC_PIC32CZ5125CA70100)
#include <pic32cz5125ca70100.h>
#elif defined(CONFIG_SOC_PIC32CZ5125CA70144)
#include <pic32cz5125ca70144.h>
#else
#error "Library does not support the specified device."
#endif

#endif /* _ASMLANGUAGE */

#endif /* SOC_MICROCHIP_PIC32CZ_CA70_SOC_H_ */
