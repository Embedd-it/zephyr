/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USBH_VENDOR_H_
#define ZEPHYR_INCLUDE_USB_CLASS_USBH_VENDOR_H_

#include <zephyr/device.h>
#include <zephyr/syscall.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read data from input endpoint
 *
 * @param dev           Pointer to device
 * @param ep_id         Endpoint ID
 * @param buf           Pointer to output buffer
 * @param buf_len       Buffer length
 *
 */
typedef int (*usbh_vendor_bulk_read_t)(struct device const *dev, uint8_t ep_id, uint8_t *buf,
				       size_t buf_len);

/**
 * @brief Write data to output endpoint
 *
 * @param dev           Pointer to device
 * @param ep_id         Endpoint ID
 * @param buf           Pointer to input buffer
 * @param buf_len       Buffer length
 *
 */
typedef int (*usbh_vendor_bulk_write_t)(struct device const *dev, uint8_t ep_id, const uint8_t *buf,
					size_t buf_len);

__subsystem struct usbh_vendor_driver_api {
	/**
	 * @driver_ops_mandatory @copybrief usbh_vendor_bulk_read
	 */
	usbh_vendor_bulk_read_t bulk_read;
	/**
	 * @driver_ops_mandatory @copybrief usbh_vendor_bulk_write
	 */
	usbh_vendor_bulk_write_t bulk_write;
};

/**
 * @brief Read data from input endpoint
 *
 * @param dev           Pointer to device
 * @param ep_id         Endpoint ID
 * @param buf           Pointer to output buffer
 * @param buf_len       Buffer length
 *
 * @return positive number of received bytes, negative errno value on failure.
 */
__syscall int usbh_vendor_bulk_read(struct device const *dev, uint8_t ep_id, uint8_t *const buf,
				    const size_t buf_len);

static inline int z_impl_usbh_vendor_bulk_read(struct device const *dev, uint8_t ep_id,
					       uint8_t *const buf, const size_t buf_len)
{
	struct usbh_vendor_driver_api const *api = DEVICE_API_GET(usbh_vendor, dev);

	if (api->bulk_read == NULL) {
		return -ENOSYS;
	}
	return api->bulk_read(dev, ep_id, buf, buf_len);
}

/**
 * @brief Write data to output endpoint
 *
 * @param dev           Pointer to device
 * @param ep_id         Endpoint ID
 * @param buf           Pointer to input buffer
 * @param buf_len       Buffer length
 *
 * @return 0 on success, negative errno value on failure.
 */
__syscall int usbh_vendor_bulk_write(struct device const *dev, uint8_t ep_id,
				     const uint8_t *const buf, const size_t buf_len);

static inline int z_impl_usbh_vendor_bulk_write(struct device const *dev, uint8_t ep_id,
						const uint8_t *const buf, const size_t buf_len)
{
	struct usbh_vendor_driver_api const *api = DEVICE_API_GET(usbh_vendor, dev);

	if (api->bulk_write == NULL) {
		return -ENOSYS;
	}
	return api->bulk_write(dev, ep_id, buf, buf_len);
}

#include <zephyr/syscalls/usbh_vendor.h>

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_USB_CLASS_USBH_VENDOR_H_ */
