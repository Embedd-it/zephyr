/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/bos.h>
#include <zephyr/usb/class/usbh_vendor.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/drivers/usb/udc.h>

LOG_MODULE_REGISTER(test_main, LOG_LEVEL_INF);
USBH_CONTROLLER_DEFINE(test_uhs_ctx, DEVICE_DT_GET(DT_NODELABEL(zephyr_uhc0)));
struct usbh_context *const uhs_ctx = &test_uhs_ctx;

USBD_DEVICE_DEFINE(test_vendor_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x2fe3, 0xffff);
USBD_DESC_LANG_DEFINE(lang_desc);
USBD_DESC_MANUFACTURER_DEFINE(mfr_desc, "Zephyr Vendor Test");
USBD_DESC_PRODUCT_DEFINE(product_desc, "Zephyr Vendor");
USBD_CONFIGURATION_DEFINE(vendor_cfg, 0, 100, NULL);

static int test_usbd_vendor_init(struct usbd_class_data *const c_data)
{
	return 0;
}

#define VENDOR_TEST_IN_EPT_ADDR   (0x81)
#define VENDOR_TEST_OUT_EPT_ADDR  (0x1)
#define VENDOR_TEST_EPT_DATA_SIZE (64)

/* Interface descriptor */
struct usb_if_descriptor if_desc = {
	.bLength = sizeof(struct usb_if_descriptor),
	.bDescriptorType = USB_DESC_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2, /* 'in_ept', 'out_ept' */
	.bInterfaceClass = USB_BCC_VENDOR,
	.bInterfaceSubClass = 0xFF,
	.bInterfaceProtocol = 0xFF,
	.iInterface = 0,
};

/* Input endpoint */
struct usb_ep_descriptor in_ept = {
	.bLength = sizeof(struct usb_ep_descriptor),
	.bDescriptorType = USB_DESC_ENDPOINT,
	.bEndpointAddress = VENDOR_TEST_IN_EPT_ADDR,
	.bmAttributes = USB_EP_TYPE_BULK,
	.wMaxPacketSize = sys_cpu_to_le16(VENDOR_TEST_EPT_DATA_SIZE),
	.bInterval = 0,
};

/* Output endpoint */
struct usb_ep_descriptor out_ept = {
	.bLength = sizeof(struct usb_ep_descriptor),
	.bDescriptorType = USB_DESC_ENDPOINT,
	.bEndpointAddress = VENDOR_TEST_OUT_EPT_ADDR,
	.bmAttributes = USB_EP_TYPE_BULK,
	.wMaxPacketSize = sys_cpu_to_le16(VENDOR_TEST_EPT_DATA_SIZE),
	.bInterval = 0,
};

/* Descriptors structure */
struct test_vendor_data {
	struct usb_if_descriptor *if_desc;
	struct usb_ep_descriptor *in_ept;
	struct usb_ep_descriptor *out_ept;
	struct usb_desc_header *sentinel;
} __packed;

struct test_vendor_data test_vendor_data = {
	.if_desc = &if_desc,
	.in_ept = &in_ept,
	.out_ept = &out_ept,
	.sentinel = NULL,
};

/* Vendor-specific device shared buffer between input and output endpoints */
static uint8_t vendor_device_shared_buf[VENDOR_TEST_EPT_DATA_SIZE];

/* Prepare buffer for input endpoint */
static int test_usbd_vendor_submit_bulk_in(struct usbd_class_data *const c_data)
{
	struct net_buf *buf = NULL;
	int ret;

	buf = usbd_ep_buf_alloc(c_data, VENDOR_TEST_IN_EPT_ADDR, VENDOR_TEST_EPT_DATA_SIZE);
	if (buf == NULL) {
		return -ENOMEM;
	}

	net_buf_add_mem(buf, vendor_device_shared_buf,
			MIN(sizeof(vendor_device_shared_buf), net_buf_tailroom(buf)));
	ret = usbd_ep_enqueue(c_data, buf);
	return ret;
}

/* Prepare buffer for output endpoint */
static int test_usbd_vendor_submit_bulk_out(struct usbd_class_data *const c_data)
{
	struct net_buf *buf = NULL;
	int ret;

	buf = usbd_ep_buf_alloc(c_data, VENDOR_TEST_OUT_EPT_ADDR, VENDOR_TEST_EPT_DATA_SIZE);
	if (buf == NULL) {
		return -ENOMEM;
	}

	ret = usbd_ep_enqueue(c_data, buf);
	return ret;
}

/* Return descriptor structures */
static void *test_usbd_vendor_get_desc(struct usbd_class_data *const c_data,
				       const enum usbd_speed speed)
{
	return &test_vendor_data;
}

/* Request from host */
static int test_usbd_vendor_request(struct usbd_class_data *const c_data, struct net_buf *buf,
				    int err)
{
	struct udc_buf_info *bi = (struct udc_buf_info *)net_buf_user_data(buf);
	const uint8_t ep = bi->ep;

	if (ep == VENDOR_TEST_OUT_EPT_ADDR) {
		memcpy(vendor_device_shared_buf, buf->data,
		       MIN(sizeof(vendor_device_shared_buf), buf->len));
		test_usbd_vendor_submit_bulk_out(c_data);
	}

	if (ep == VENDOR_TEST_IN_EPT_ADDR) {
		test_usbd_vendor_submit_bulk_in(c_data);
	}

	return 0;
}

/* Device was enabled, prepare buffers */
static void test_usbd_vendor_enable(struct usbd_class_data *const c_data)
{
	(void)test_usbd_vendor_submit_bulk_in(c_data);
	(void)test_usbd_vendor_submit_bulk_out(c_data);
}

/* Necessary device callbacks */
static const struct usbd_class_api test_usbd_vendor_api = {
	.init = test_usbd_vendor_init,
	.get_desc = test_usbd_vendor_get_desc,
	.request = test_usbd_vendor_request,
	.enable = test_usbd_vendor_enable,
};

USBD_DEFINE_CLASS(vendor_c_data, &test_usbd_vendor_api, NULL, NULL);

/* Test data for read */
static char data_read_buf[VENDOR_TEST_EPT_DATA_SIZE] = "Lorem ipsum dolor sit amet";

/* Test data for write */
static char data_write_buf[VENDOR_TEST_EPT_DATA_SIZE] = "consectetur adipiscing elit";

ZTEST(vendor_host_test, vendor)
{
	const struct device *dev;
	int ret;

	dev = DEVICE_DT_GET(DT_NODELABEL(any_vendor_device));

	/* Sleep is necessary, so bus can probe the host driver */
	k_msleep(400);

	ret = memcmp(data_write_buf, data_read_buf, VENDOR_TEST_EPT_DATA_SIZE);
	zassert_not_equal(ret, 0, "Buffers are expected not be the same");

	ret = usbh_vendor_bulk_write(dev, 1, data_write_buf, VENDOR_TEST_EPT_DATA_SIZE);
	zassert_ok(ret, "Failed to write data to endpoint");

	/* First read still returns old data; the next read will return the updated data */
	ret = usbh_vendor_bulk_read(dev, 1, data_read_buf, VENDOR_TEST_EPT_DATA_SIZE);
	zassert_true(ret >= 0, "Failed to read data from endpoint");

	ret = memcmp(data_write_buf, data_read_buf, VENDOR_TEST_EPT_DATA_SIZE);
	zassert_not_equal(ret, 0, "Buffers are expected not be the same");

	/* The read now reports previously written data */
	ret = usbh_vendor_bulk_read(dev, 1, data_read_buf, VENDOR_TEST_EPT_DATA_SIZE);
	zassert_true(ret >= 0, "Failed to read data from endpoint");

	ret = memcmp(data_write_buf, data_read_buf, VENDOR_TEST_EPT_DATA_SIZE);
	zassert_equal(ret, 0, "Buffers are expected to be the same");

	ret = usbh_vendor_bulk_write(dev, 2, data_write_buf, VENDOR_TEST_EPT_DATA_SIZE);
	zassert_equal(ret, -ENOTSUP, "Write to endpoint 2 should fail");

	ret = usbh_vendor_bulk_read(dev, 2, data_read_buf, VENDOR_TEST_EPT_DATA_SIZE);
	zassert_equal(ret, -ENOTSUP, "Read from endpoint 2 should fail");
}

void *vendor_host_test_enable(void)
{
	int result;

	result = usbh_init(uhs_ctx);
	zassert_ok(result, "Failed to initialize USB host");

	result = usbh_enable(uhs_ctx);
	zassert_ok(result, "Failed to enable USB host");

	result = uhc_bus_reset(uhs_ctx->dev);
	zassert_ok(result, "Failed to signal bus reset");

	result = uhc_bus_resume(uhs_ctx->dev);
	zassert_ok(result, "Failed to signal bus resume");

	result = uhc_sof_enable(uhs_ctx->dev);
	zassert_ok(result, "Failed to enable SoF generator");

	LOG_INF("Host controller enabled");

	result = usbd_add_descriptor(&test_vendor_usbd, &lang_desc);
	zassert_ok(result);

	result = usbd_device_set_bcd_usb(&test_vendor_usbd, USBD_SPEED_FS, 0x0201);
	zassert_ok(result);

	result = usbd_add_descriptor(&test_vendor_usbd, &mfr_desc);
	zassert_ok(result);

	result = usbd_add_descriptor(&test_vendor_usbd, &product_desc);
	zassert_ok(result);

	result = usbd_add_configuration(&test_vendor_usbd, USBD_SPEED_FS, &vendor_cfg);
	zassert_ok(result);

	result = usbd_register_class(&test_vendor_usbd, "vendor_c_data", USBD_SPEED_FS, 1);
	zassert_ok(result);

	/* Triplet 0, 0, 0 means use class code from interface descriptor */
	result = usbd_device_set_code_triple(&test_vendor_usbd, USBD_SPEED_FS, 0, 0, 0);
	zassert_ok(result);

	result = usbd_init(&test_vendor_usbd);
	zassert_ok(result);

	result = usbd_enable(&test_vendor_usbd);
	zassert_ok(result);

	LOG_INF("Vendor device enabled");

	return NULL;
}

void vendor_host_test_shutdown(void *f)
{
	int result;

	result = usbd_disable(&test_vendor_usbd);
	zassert_ok(result, "Failed to disable device");

	result = usbd_shutdown(&test_vendor_usbd);
	zassert_ok(result, "Failed to shutdown device");

	result = usbh_disable(uhs_ctx);
	zassert_ok(result, "Failed to disable USB host");

	result = usbh_shutdown(uhs_ctx);
	zassert_ok(result, "Failed to shutdown host support");

	LOG_INF("Host controller disabled");
}

ZTEST_SUITE(vendor_host_test, NULL, vendor_host_test_enable, NULL, NULL, vendor_host_test_shutdown);
