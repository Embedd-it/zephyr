/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "device_subsys.h"
#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>

static const struct device_subsys_data subsys_data[] = {
	{.subsys = (void *)CLOCK_MCHP_UART3},
	{.subsys = (void *)CLOCK_MCHP_USART1},
	{.subsys = (void *)CLOCK_MCHP_PCK5},
};

static const struct device_data devices[] = {{.dev = DEVICE_DT_GET(DT_NODELABEL(pmc)),
					      .subsys_data = subsys_data,
					      .subsys_cnt = ARRAY_SIZE(subsys_data)}};
