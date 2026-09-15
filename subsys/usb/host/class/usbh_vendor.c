/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_usbh_vendor_device

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

struct usbh_vendor_drv_data;

struct usbh_vendor_ept_data {
	struct usbh_vendor_drv_data *drv_data;
	void *user_ctx;
	uint16_t ep_id;
	usbh_vendor_callback_t cb;
	bool enabled;
	struct uhc_transfer *xfer;
	struct k_sem complete;
	uint16_t mps;
};

struct usbh_vendor_drv_data {
	/* Mutex */
	struct k_mutex lock;
	/* Device was probed */
	bool probed;
	/* Pointer to udev */
	struct usb_device *udev;
	/* Selected interface */
	uint16_t iface;
	/* Bitmap of input endpoints */
	uint16_t bulk_in_epts;
	/* Bitmap of output endpoints */
	uint16_t bulk_out_epts;
	/* Bitmap of interrupt input endpoints */
	uint16_t intr_in_epts;
	/* Bitmap of interrupt output endpoints */
	uint16_t intr_out_epts;
	/* Interrupt data */
	struct usbh_vendor_ept_data ept_data[32];
};

struct usbh_vendor_ept_data *usbh_vendor_get_ept_data(struct usbh_vendor_drv_data *dev_data,
						      uint8_t ept_id, bool in)
{
	int index = (in != 0 ? 0 : 1) * 16 + (ept_id & 0xF);

	return &dev_data->ept_data[index];
}

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
	struct usbh_vendor_ept_data *ept_data = xfer->priv;

	k_sem_give(&ept_data->complete);

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
	struct usbh_vendor_ept_data *ept_data;
	struct usbh_vendor_drv_data *drv_data;
	struct uhc_transfer *xfer = NULL;
	bool dequeue_flow = false;
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
		if (!(drv_data->bulk_in_epts & (1 << ep_id))) {
			ret = -ENOTSUP;
			break;
		}

		ept_data = usbh_vendor_get_ept_data(drv_data, ep_id, true);
		xfer = usbh_xfer_alloc(drv_data->udev, USB_EP_GET_ADDR(ep_id, USB_EP_DIR_IN),
				       usbh_vendor_bulk_transfer_cb, ept_data);
		if (xfer == NULL) {
			ret = -ENOMEM;
			break;
		}
		xfer->buf = usbh_xfer_buf_alloc(drv_data->udev, buf_len);
		if (xfer->buf == NULL) {
			ret = -ENOMEM;
			break;
		}

		k_sem_reset(&ept_data->complete);
		ret = usbh_xfer_enqueue(drv_data->udev, xfer);
		if (ret) {
			break;
		}

		if (k_sem_take(&ept_data->complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) != 0) {
			(void)usbh_xfer_dequeue(drv_data->udev, xfer);
			if (k_sem_take(&ept_data->complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) !=
			    0) {
				dequeue_flow = true;
				ret = -ETIME;
				break;
			}
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

	if (!dequeue_flow) {
		if (xfer != NULL && xfer->buf != NULL) {
			usbh_xfer_buf_free(drv_data->udev, xfer->buf);
		}
		if (xfer != NULL) {
			usbh_xfer_free(drv_data->udev, xfer);
		}
	}

	k_mutex_unlock(&drv_data->lock);

	return ret;
}

static int usbh_vendor_interrupt_cb(struct usb_device *const udev, struct uhc_transfer *const xfer);

static int usbh_vendor_interrupt_prepare_message(struct usbh_vendor_drv_data *drv_data,
						 struct usbh_vendor_ept_data *ept_data)
{
	struct uhc_transfer *xfer = NULL;
	int ret;

	do {
		xfer = usbh_xfer_alloc(drv_data->udev,
				       USB_EP_GET_ADDR(ept_data->ep_id, USB_EP_DIR_IN),
				       usbh_vendor_interrupt_cb, ept_data);
		if (xfer == NULL) {
			ret = -ENOMEM;
			break;
		}
		xfer->buf = usbh_xfer_buf_alloc(drv_data->udev, xfer->mps);
		if (xfer->buf == NULL) {
			ret = -ENOMEM;
			break;
		}

		ept_data->xfer = xfer;
		k_sem_reset(&ept_data->complete);
		ret = usbh_xfer_enqueue(drv_data->udev, xfer);
		if (ret) {
			break;
		}
	} while (0);

	if ((ret != 0) && (xfer != NULL) && (xfer->buf != NULL)) {
		usbh_xfer_buf_free(drv_data->udev, xfer->buf);
	}

	if ((ret != 0) && (xfer != NULL)) {
		usbh_xfer_free(drv_data->udev, xfer);
		ept_data->xfer = NULL;
	}

	return ret;
}

/**
 * @brief Interrupt receive complete, release semaphore
 *
 * @param udev   Pointer to USB device
 * @param xfer   Pointer to transfer
 *
 * @return 0 on success, negative errno on failure
 */
static int usbh_vendor_interrupt_cb(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	struct usbh_vendor_ept_data *ept_data = xfer->priv;

	if ((xfer->buf != NULL) && (xfer->err == 0) && (ept_data->enabled)) {
		ept_data->cb(ept_data->user_ctx, ept_data->ep_id, xfer->buf->data, xfer->buf->len);
	}

	if (ept_data->enabled) {
		net_buf_reset(xfer->buf);
		usbh_xfer_enqueue(udev, xfer);
	} else {
		k_sem_give(&ept_data->complete);
	}

	return 0;
}

int usbh_vendor_interrupt_subscribe_api(struct device const *dev, uint8_t ep_id,
					usbh_vendor_callback_t cb, void *user_ctx)
{
	struct usbh_vendor_ept_data *ept_data;
	struct usbh_vendor_drv_data *drv_data;
	int ret = 0;

	if (dev == NULL || cb == NULL) {
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
		ept_data = usbh_vendor_get_ept_data(drv_data, ep_id, true);
		if (ept_data->enabled == true) {
			ret = -EALREADY;
			break;
		}

		if (!(drv_data->intr_in_epts & (1 << ep_id))) {
			ret = -ENOTSUP;
			break;
		}

		ept_data->user_ctx = user_ctx;
		ept_data->cb = cb;
		ept_data->enabled = true;

		ret = usbh_vendor_interrupt_prepare_message(drv_data, &drv_data->ept_data[ep_id]);
		if (ret != 0) {
			drv_data->ept_data[ep_id].enabled = false;
		}
	} while (0);

	k_mutex_unlock(&drv_data->lock);

	return ret;
}

int usbh_vendor_interrupt_unsubscribe_api(struct device const *dev, uint8_t ep_id)
{
	struct usbh_vendor_ept_data *ept_data;
	struct usbh_vendor_drv_data *drv_data;
	bool dequeue_flow = false;
	int ret = 0;

	if (dev == NULL) {
		return -EINVAL;
	}
	drv_data = dev->data;

	ret = k_mutex_lock(&drv_data->lock, K_FOREVER);
	if (ret) {
		return ret;
	}

	do {
		ep_id = ep_id & 0xF;
		ept_data = usbh_vendor_get_ept_data(drv_data, ep_id, true);
		if (ept_data->enabled != true) {
			ret = -EALREADY;
			break;
		}

		/* Stop reloading xfer */
		ept_data->enabled = false;
		/* Complete callback to release semaphore was not called, dequeue by force */
		if (k_sem_take(&ept_data->complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) != 0) {
			(void)usbh_xfer_dequeue(drv_data->udev, ept_data->xfer);
			if (k_sem_take(&ept_data->complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) !=
			    0) {
				dequeue_flow = true;
				ret = -ETIME;
				break;
			}
		}

		if (!dequeue_flow) {
			if ((ept_data->xfer != NULL) && (ept_data->xfer->buf != NULL)) {
				usbh_xfer_buf_free(drv_data->udev, ept_data->xfer->buf);
				ept_data->xfer->buf = NULL;
			}

			if (ept_data->xfer != NULL) {
				usbh_xfer_free(drv_data->udev, ept_data->xfer);
				ept_data->xfer = NULL;
			}
		}
	} while (0);

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
	struct usbh_vendor_ept_data *ept_data;
	struct usbh_vendor_drv_data *drv_data;
	struct uhc_transfer *xfer = NULL;
	bool dequeue_flow = false;
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
		if (!(drv_data->bulk_out_epts & (1 << ep_id))) {
			ret = -ENOTSUP;
			break;
		}

		ept_data = usbh_vendor_get_ept_data(drv_data, ep_id, false);
		xfer = usbh_xfer_alloc(drv_data->udev, USB_EP_GET_ADDR(ep_id, USB_EP_DIR_OUT),
				       usbh_vendor_bulk_transfer_cb, ept_data);
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

		k_sem_reset(&ept_data->complete);
		ret = usbh_xfer_enqueue(drv_data->udev, xfer);
		if (ret) {
			break;
		}

		if (k_sem_take(&ept_data->complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) != 0) {
			(void)usbh_xfer_dequeue(drv_data->udev, xfer);
			if (k_sem_take(&ept_data->complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) !=
			    0) {
				dequeue_flow = true;
				ret = -ETIME;
				break;
			}
		}

		if (xfer->err != 0) {
			LOG_ERR("Bulk write transfer failed, err %d", xfer->err);
			ret = -EIO;
			break;
		}

	} while (0);

	if (!dequeue_flow) {
		if (xfer != NULL && xfer->buf != NULL) {
			usbh_xfer_buf_free(drv_data->udev, xfer->buf);
		}
		if (xfer != NULL) {
			usbh_xfer_free(drv_data->udev, xfer);
		}
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

int usbh_vendor_interrupt_write_api(struct device const *dev, uint8_t ep_id,
				    const uint8_t *const buf, const size_t buf_len)
{
	struct usbh_vendor_ept_data *ept_data;
	struct usbh_vendor_drv_data *drv_data;
	struct uhc_transfer *xfer = NULL;
	bool dequeue_flow = false;
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
		if (!(drv_data->intr_out_epts & (1 << ep_id))) {
			ret = -ENOTSUP;
			break;
		}

		ept_data = usbh_vendor_get_ept_data(drv_data, ep_id, false);

		/* buffer length should not exceed endpoint buffer */
		if (buf_len > ept_data->mps) {
			ret = -EMSGSIZE;
			break;
		}

		xfer = usbh_xfer_alloc(drv_data->udev, USB_EP_GET_ADDR(ep_id, USB_EP_DIR_OUT),
				       usbh_vendor_bulk_transfer_cb, ept_data);
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

		k_sem_reset(&ept_data->complete);
		ret = usbh_xfer_enqueue(drv_data->udev, xfer);
		if (ret) {
			break;
		}

		if (k_sem_take(&ept_data->complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) != 0) {
			(void)usbh_xfer_dequeue(drv_data->udev, xfer);
			if (k_sem_take(&ept_data->complete, K_MSEC(USBH_VENDOR_SEM_TIMEOUT_MS)) !=
			    0) {
				dequeue_flow = true;
				ret = -ETIME;
				break;
			}
		}

		if (xfer->err != 0) {
			LOG_ERR("Bulk write transfer failed, err %d", xfer->err);
			ret = -EIO;
			break;
		}

	} while (0);

	if (!dequeue_flow) {
		if (xfer != NULL && xfer->buf != NULL) {
			usbh_xfer_buf_free(drv_data->udev, xfer->buf);
		}
		if (xfer != NULL) {
			usbh_xfer_free(drv_data->udev, xfer);
		}
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
	struct usbh_vendor_ept_data *ept_data;

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
			if (USB_EP_DIR_IS_IN(ep_desc->bEndpointAddress)) {
				ept_data = usbh_vendor_get_ept_data(
					drv_data, USB_EP_GET_IDX(ep_desc->bEndpointAddress), true);
				ept_data->mps = sys_le16_to_cpu(ep_desc->wMaxPacketSize) & 0x7FF;
				if ((ep_desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
				    USB_EP_TYPE_BULK) {
					drv_data->bulk_in_epts |=
						(1 << USB_EP_GET_IDX(ep_desc->bEndpointAddress));
				} else if ((ep_desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
					   USB_EP_TYPE_INTERRUPT) {
					drv_data->intr_in_epts |=
						(1 << USB_EP_GET_IDX(ep_desc->bEndpointAddress));
				}
			} else {
				ept_data = usbh_vendor_get_ept_data(
					drv_data, USB_EP_GET_IDX(ep_desc->bEndpointAddress), false);
				ept_data->mps = sys_le16_to_cpu(ep_desc->wMaxPacketSize) & 0x7FF;
				if ((ep_desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
				    USB_EP_TYPE_BULK) {
					drv_data->bulk_out_epts |=
						(1 << USB_EP_GET_IDX(ep_desc->bEndpointAddress));
				} else if ((ep_desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
					   USB_EP_TYPE_INTERRUPT) {
					drv_data->intr_out_epts |=
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
	struct usbh_vendor_ept_data *ept_data;
	const struct device *dev = c_data->priv;
	struct usbh_vendor_drv_data *drv_data = (void *)dev->data;
	int result = 0, i;

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
	drv_data->bulk_in_epts = 0;
	drv_data->bulk_out_epts = 0;
	drv_data->intr_in_epts = 0;
	drv_data->intr_out_epts = 0;

	memset(drv_data->ept_data, 0, sizeof(drv_data->ept_data));
	for (i = 0; i < 16; i++) {
		ept_data = usbh_vendor_get_ept_data(drv_data, i, true);
		ept_data->drv_data = drv_data;
		ept_data->ep_id = i;
		k_sem_init(&ept_data->complete, 0, 1);

		ept_data = usbh_vendor_get_ept_data(drv_data, i, false);
		ept_data->drv_data = drv_data;
		ept_data->ep_id = i;
		k_sem_init(&ept_data->complete, 0, 1);
	}

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
	.interrupt_write = usbh_vendor_interrupt_write_api,
	.interrupt_subscribe = usbh_vendor_interrupt_subscribe_api,
	.interrupt_unsubscribe = usbh_vendor_interrupt_unsubscribe_api,
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
