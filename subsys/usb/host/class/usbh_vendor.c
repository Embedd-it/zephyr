/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_usbh_vendor_device

#include <stdlib.h>

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/usbh_vendor.h>

#include "usbh_ch9.h"
#include "usbh_class.h"
#include "usbh_desc.h"
#include "usbh_device.h"

LOG_MODULE_REGISTER(usbh_vendor, CONFIG_USBH_VENDOR_LOG_LEVEL);

#define USBH_VENDOR_SEM_TIMEOUT_MS (1000)

struct usbh_vendor_drv_data {
	/* Mutex */
	struct k_mutex lock;
	/* Device was probed */
	bool probed;
	/* Pointer to udev */
	struct usb_device *udev;
	/* Read or write was completed */
	struct k_sem transfer_complete;
	/* Selected interface */
	uint16_t iface;
	/* Bitmap of input endpoints */
	uint16_t in_epts;
	/* Bitmap of output endpoints */
	uint16_t out_epts;
};

/**
 * @brief Transfer complete, release semaphore
 *
 * @param udev   Pointer to USB device
 * @param xfer   Pointer to transfer
 *
 * @return 0 on success, negative errno on failure
 */
static int usbh_vendor_bulk_transfer_cb(struct usb_device *const udev,
					struct uhc_transfer *const xfer)
{
	struct usbh_vendor_drv_data *drv_data = xfer->priv;

	k_sem_give(&drv_data->transfer_complete);

	return 0;
}

/**
 * @brief Read buffer from endpoint
 *
 * @param dev        Pointer to device
 * @param ep_id      Endpoint ID
 * @param buf        Application buffer to read data
 * @param buf_len    Buffer length
 *
 * @return positive number of received bytes, negative errno value on failure.
 */
int usbh_vendor_bulk_read_api(struct device const *dev, uint8_t ep_id, uint8_t *const buf,
			      const size_t buf_len)
{
	struct usbh_vendor_drv_data *drv_data;
	struct uhc_transfer *xfer = NULL;
	size_t copied_bytes;
	int ret = 0;

	if ((dev == NULL) || (buf_len == 0) || (buf == NULL)) {
		return -EINVAL;
	}
	drv_data = dev->data;

	ret = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (ret) {
		return ret;
	}

	do {
		if (!drv_data->probed) {
			LOG_DBG("Driver was not probed yet");
			ret = -EAGAIN;
			break;
		}

		ep_id = ep_id & 0xF;
		if (!(drv_data->in_epts & (1 << ep_id))) {
			ret = -ENOTSUP;
			break;
		}

		xfer = usbh_xfer_alloc(drv_data->udev, USB_EP_GET_ADDR(ep_id, USB_EP_DIR_IN),
				       usbh_vendor_bulk_transfer_cb, drv_data);
		if (xfer == NULL) {
			ret = -ENOMEM;
			break;
		}
		xfer->buf = usbh_xfer_buf_alloc(drv_data->udev, buf_len);
		if (xfer->buf == NULL) {
			ret = -ENOMEM;
			break;
		}

		k_sem_reset(&drv_data->transfer_complete);
		ret = usbh_xfer_enqueue(drv_data->udev, xfer);
		if (ret) {
			break;
		}

		if (k_sem_take(&drv_data->transfer_complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) !=
		    0) {
			(void)usbh_xfer_dequeue(drv_data->udev, xfer);
			ret = -ETIME;
			break;
		}

		if (xfer->err != 0) {
			LOG_ERR("Bulk read transfer failed, err %d", xfer->err);
			ret = -EIO;
			break;
		}

		copied_bytes = MIN(xfer->buf->len, buf_len);
		memcpy(buf, xfer->buf->data, copied_bytes);
		ret = copied_bytes;
	} while (0);

	if (xfer != NULL && xfer->buf != NULL) {
		usbh_xfer_buf_free(drv_data->udev, xfer->buf);
	}
	if (xfer != NULL) {
		usbh_xfer_free(drv_data->udev, xfer);
	}

	k_mutex_unlock(&drv_data->lock);

	return ret;
}

/**
 * @brief Write buffer to endpoint
 *
 * @param dev        Pointer to device
 * @param ep_id      Endpoint ID
 * @param buf        Application buffer to write data
 * @param buf_len    Buffer length
 *
 * @return 0 on success, negative errno on failure
 */
int usbh_vendor_bulk_write_api(struct device const *dev, uint8_t ep_id, const uint8_t *const buf,
			       const size_t buf_len)
{
	struct usbh_vendor_drv_data *drv_data;
	struct uhc_transfer *xfer = NULL;
	int ret = 0;

	if ((dev == NULL) || (buf_len == 0) || (buf == NULL)) {
		return -EINVAL;
	}
	drv_data = dev->data;

	ret = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (ret) {
		return ret;
	}

	do {
		if (!drv_data->probed) {
			LOG_DBG("Driver was not probed yet");
			ret = -EAGAIN;
			break;
		}

		ep_id = ep_id & 0xF;
		if (!(drv_data->out_epts & (1 << ep_id))) {
			ret = -ENOTSUP;
			break;
		}

		xfer = usbh_xfer_alloc(drv_data->udev, USB_EP_GET_ADDR(ep_id, USB_EP_DIR_OUT),
				       usbh_vendor_bulk_transfer_cb, drv_data);
		if (xfer == NULL) {
			ret = -ENOMEM;
			break;
		}
		xfer->buf = usbh_xfer_buf_alloc(drv_data->udev, buf_len);
		if (xfer->buf == NULL) {
			ret = -ENOMEM;
			break;
		}

		memcpy(xfer->buf->data, buf, buf_len);
		net_buf_add(xfer->buf, buf_len);

		k_sem_reset(&drv_data->transfer_complete);
		ret = usbh_xfer_enqueue(drv_data->udev, xfer);
		if (ret) {
			break;
		}

		if (k_sem_take(&drv_data->transfer_complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) !=
		    0) {
			(void)usbh_xfer_dequeue(drv_data->udev, xfer);
			ret = -ETIME;
			break;
		}

		if (xfer->err != 0) {
			LOG_ERR("Bulk write transfer failed, err %d", xfer->err);
			ret = -EIO;
			break;
		}

	} while (0);

	if (xfer != NULL && xfer->buf != NULL) {
		usbh_xfer_buf_free(drv_data->udev, xfer->buf);
	}
	if (xfer != NULL) {
		usbh_xfer_free(drv_data->udev, xfer);
	}

	k_mutex_unlock(&drv_data->lock);

	return ret;
}

/**
 * @brief Driver initialization
 *
 * @param c_data  Pointer to USB class
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_vendor_class_init(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct usbh_vendor_drv_data *const drv_data = dev->data;

	k_mutex_init(&drv_data->lock);
	k_sem_init(&drv_data->transfer_complete, 0, 1);
	drv_data->probed = false;

	return 0;
}

/**
 * @brief Parse configuration descriptors, build bitmap for valid in/out endpoints
 *
 * @param c_data        Pointer to USB class
 * @param udev          Pointer to USB device
 * @param iface         Interface number
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_vendor_parse_desc(struct usbh_class_data *const c_data,
				  struct usb_device *const udev, uint8_t iface)
{
	const struct device *dev = c_data->priv;
	struct usbh_vendor_drv_data *const drv_data = dev->data;
	const struct usb_desc_header *desc;
	struct usb_ep_descriptor *ep_desc = NULL;
	bool in_interface = false;
	struct usb_if_descriptor *if_desc;

	desc = udev->cfg_desc;
	if ((desc == NULL) || (desc->bDescriptorType != USB_DESC_CONFIGURATION)) {
		return -EFAULT;
	}

	desc = usbh_desc_get_next(desc);
	while (desc != NULL) {
		if (usbh_desc_is_valid(desc, sizeof(struct usb_if_descriptor),
				       USB_DESC_INTERFACE)) {
			if_desc = ((struct usb_if_descriptor *)desc);
			if ((if_desc->bInterfaceClass == USB_BCC_VENDOR) &&
			    (if_desc->bInterfaceSubClass == 0xFF) &&
			    (if_desc->bInterfaceProtocol == 0xFF) &&
			    (if_desc->bInterfaceNumber == iface)) {
				in_interface = true;
			} else {
				in_interface = false;
			}
		} else if (usbh_desc_is_valid(desc, sizeof(struct usb_ep_descriptor),
					      USB_DESC_ENDPOINT) &&
			   in_interface) {
			ep_desc = (struct usb_ep_descriptor *)desc;
			if ((ep_desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
			    USB_EP_TYPE_BULK) {
				if (USB_EP_DIR_IS_IN(ep_desc->bEndpointAddress)) {
					drv_data->in_epts |=
						(1 << USB_EP_GET_IDX(ep_desc->bEndpointAddress));
				} else {
					drv_data->out_epts |=
						(1 << USB_EP_GET_IDX(ep_desc->bEndpointAddress));
				}
			}
		}
		desc = usbh_desc_get_next(desc);
	}

	return 0;
}

/**
 * @brief Driver probe function, invoked on USB device attach
 *
 * @param c_data        Pointer to USB class
 * @param udev          Pointer to USB device
 * @param iface         Interface number
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_vendor_class_probe(struct usbh_class_data *const c_data,
				   struct usb_device *const udev, uint8_t iface)
{
	const struct device *dev = c_data->priv;
	struct usbh_vendor_drv_data *drv_data = (void *)dev->data;
	int result = 0;

	LOG_DBG("Vendor device was attached");

	if ((udev == NULL) || (udev->state != USB_STATE_CONFIGURED)) {
		LOG_ERR("USB device not properly configured");
		return -ENODEV;
	}

	if (drv_data == NULL) {
		LOG_ERR("No Vendor device instance is available");
		return -ENODEV;
	}

	result = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (result) {
		return result;
	}

	drv_data->udev = udev;
	drv_data->iface = iface;
	drv_data->in_epts = 0;
	drv_data->out_epts = 0;

	result = usbh_vendor_parse_desc(c_data, udev, iface);
	if (result == 0) {
		drv_data->probed = true;
	}

	k_mutex_unlock(&drv_data->lock);

	return result;
}

/**
 * @brief Driver remove function, invoked on USB device removal
 *
 * @param c_data        Pointer to USB class
 *
 * @return 0 on success, negative errno value on failure.
 */
static int usbh_vendor_class_removed(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct usbh_vendor_drv_data *drv_data = (void *)dev->data;
	int result = 0;

	result = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (result) {
		return result;
	}

	drv_data->udev = NULL;
	drv_data->probed = false;
	k_mutex_unlock(&drv_data->lock);

	LOG_DBG("Vendor device was removed");

	return result;
}

/**
 * @brief USB driver API table
 */
static __maybe_unused DEVICE_API(usbh_vendor, usbh_vendor_api) = {
	.bulk_read = usbh_vendor_bulk_read_api,
	.bulk_write = usbh_vendor_bulk_write_api,
};

/**
 * @brief USB class driver API table
 */
static __maybe_unused struct usbh_class_api usbh_vendor_class_api = {
	.init = usbh_vendor_class_init,
	.probe = usbh_vendor_class_probe,
	.removed = usbh_vendor_class_removed,
};

static __maybe_unused struct usbh_class_filter usbh_vendor_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_VENDOR,
		.sub = 0xFF,
		.proto = 0xFF,
	},
	{0},
};

#define USBH_VENDOR_DEVICE_DEFINE(n)                                                               \
	static struct usbh_vendor_drv_data usbh_vendor_drv_data##n;                                \
	COND_CODE_0(DT_INST_PROP(n, match_class),                                               \
	(static struct usbh_class_filter const usbh_vendor_vid_pid_filters_##n[] = {            \
		{                                                                               \
			.flags = USBH_CLASS_MATCH_VID_PID,                                      \
			.vid = (DT_INST_REG_ADDR(n) >> 16u) & 0xFFFFu,                          \
			.pid = DT_INST_REG_ADDR(n) & 0xFFFFu,                                   \
		},                                                                              \
		{0u}                                                                            \
	};), ())  \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, &usbh_vendor_drv_data##n, NULL, POST_KERNEL,          \
			      CONFIG_USBH_VENDOR_INIT_PRIORITY, &usbh_vendor_api);                 \
	USBH_DEFINE_CLASS(                                                                         \
		vendor_host_c_data_##n, &usbh_vendor_class_api, (void *)DEVICE_DT_INST_GET(n),     \
		COND_CODE_1(DT_INST_PROP(n, match_class),                                       \
				(usbh_vendor_filters), (usbh_vendor_vid_pid_filters_##n)));

DT_INST_FOREACH_STATUS_OKAY(USBH_VENDOR_DEVICE_DEFINE)
