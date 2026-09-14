/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/fs/fs.h>
#include <zephyr/ztest.h>

#include <ff.h>

/*
 * The test case simulates up to three units using RAM.
 *
 * `USBD_DEFINE_MSC_LUN(id, ...)`'s LUN order (and therefore its device-side bLUN / host-side
 * "USB0_<n>" index) is the alphabetical order of the resulting `usbd_msc_lun_##id` symbol
 * (linker SORT_BY_NAME on the `usbd_msc_lun` iterable section), not declaration order. "ram0"
 * and "ram1" sort as LUN 0 and LUN 1 respectively.
 */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(ramdisk0))
USBD_DEFINE_MSC_LUN(ram0, "RAM0", "Zephyr", "RAMDisk", "0.00");
static struct fs_mount_t fs_mnt0;
static FATFS fat_fs0;
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(ramdisk1))
USBD_DEFINE_MSC_LUN(ram1, "RAM1", "Zephyr", "RAMDisk", "0.00");
static struct fs_mount_t fs_mnt1;
static FATFS fat_fs1;
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(ramdisk0)) ||                                             \
	DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(ramdisk1))
static void setup_disk(char const *mnt_point, struct fs_mount_t *fs_mnt, FATFS *fat_fs)
{
	struct fs_mount_t *mp = fs_mnt;
	int result = 0;

	mp->type = FS_FATFS;
	mp->fs_data = fat_fs;
	mp->mnt_point = mnt_point;

	result = fs_mount(mp);
	zassert_ok(result, "Failed to mount filesystem: %i", result);
}
#endif

void ram_disk_setup(void)
{
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(ramdisk0))
	setup_disk("/RAM0:", &fs_mnt0, &fat_fs0);
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(ramdisk1))
	setup_disk("/RAM1:", &fs_mnt1, &fat_fs1);
#endif
}
