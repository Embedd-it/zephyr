# SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
# SPDX-License-Identifier: Apache-2.0

board_runner_args(mplab_ipe "--tool" "PKOB4" "--part" "32CZ2051CA70144" "--erase" "--verify")
board_runner_args(jlink "--device=PIC32CZ2051CA70144" "--speed=4000")
board_runner_args(openocd "--config=${BOARD_DIR}/support/openocd_pic32cz_ca70_cult.cfg")

include(${ZEPHYR_BASE}/boards/common/mplab_ipe.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
