/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FAULT_DISK_H_INCLUDED
#define FAULT_DISK_H_INCLUDED

#include <stdbool.h>

#define FAULT_DISK_NAME "FAULT0"

void fault_disk_setup(void);

void fault_disk_reset(void);

void fault_disk_set_status_override(int status);

void fault_disk_set_read_error(bool fail);

void fault_disk_set_write_error(bool fail);

#endif /* FAULT_DISK_H_INCLUDED */
