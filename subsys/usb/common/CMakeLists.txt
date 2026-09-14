# SPDX-FileCopyrightText: Copyright Nordic Semiconductor ASA
# SPDX-FileCopyrightText: Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_USBD_VIDEO_CLASS OR CONFIG_USBH_VIDEO_CLASS)
  zephyr_include_directories(.)
  zephyr_sources(uvc.c)
endif()

if(CONFIG_USBD_MSC_CLASS OR CONFIG_USBH_MSC_CLASS)
  zephyr_include_directories(.)
endif()
