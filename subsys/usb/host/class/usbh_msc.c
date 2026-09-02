/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/disk.h>

#include "msc.h"
#include "usbh_class.h"
#include "usbh_desc.h"
#include "usbh_ch9.h"
#include "usbh_device.h"

LOG_MODULE_REGISTER(usbh_msc, CONFIG_USBH_MSC_LOG_LEVEL);

/* Size of a Command Block Wrapper */
#define CBW_SIZE                     31u
/* Size of a Command Status Wrapper */
#define CSW_SIZE                     13u
/* Maximum size for a CBW */
#define CBW_COMMAND_BLOCK_MAX_LENGTH 16u
/* Timeout for any SCSI request */
#define SCSI_REQ_TIMEOUT             5000u
/* USB<n>_<lun> */
#define DISK_NAME_LENGTH                                                                           \
	(8u + (CONFIG_USBH_MSC_INSTANCES_COUNT / 10u) + 1u +                                       \
	 (CONFIG_USBH_MSC_MAX_SUPPORTED_LUN / 10u) + 1u)
/* Max sense data returned by a Request Sense command */
#define SCSI_MAX_SENSE_DATA                   18u
/* Code for the current sense data */
#define SCSI_SENSE_DATA_RESPONSE_CODE_CURRENT 0x70u

/* From SPC table 49 */
#define SCSI_SENSE_DATA_KEY_NOT_READY       0x02u
#define SCSI_SENSE_DATA_KEY_MEDIUM_ERROR    0x03u
#define SCSI_SENSE_DATA_KEY_ILLEGAL_REQUEST 0x05u
#define SCSI_SENSE_DATA_KEY_UNIT_ATTENTION  0x06u

/* From SPC table 50 */
#define SCSI_SENSE_DATA_ASC_INVALID_COMMAND_OPERATION_CODE 0x20u
#define SCSI_SENSE_DATA_ASC_NOT_READY_TO_READY             0x28u
#define SCSI_SENSE_DATA_ASC_MEDIUM_NOT_PRESENT             0x3Au

#define SCSI_READ_CAPACITY_10_DATA_LENGTH 8u
#define SCSI_MODE_SENSE_DATA_LENGTH       8u

/* Data direction of an SCSI transaction */
enum scsi_direction {
	SCSI_DIRECTION_DATA_OUT = 0u,
	SCSI_DIRECTION_DATA_IN = 1u,
};

/* SCSI Command Block Wrapper */
struct scsi_cbw {
	/* Target Logical UNit index */
	uint8_t lun;
	/* Transaction direction */
	enum scsi_direction direction;
	/* Size of the attached command block data */
	size_t command_block_length;
	/* Command block data */
	const uint8_t *command_block;
	/* Data phase transfer length */
	size_t data_transfer_length;
};

/* Status code of an SCSI transaction as returned by the Sense Data */
enum scsi_status {
	/* OK */
	SCSI_STATUS_COMMAND_PASSED = 0x00u,
	/* The device is reporting an error */
	SCSI_STATUS_COMMAND_FAILED = 0x01u,
	/* There was an issue during communication */
	SCSI_STATUS_PHASE_ERROR = 0x02u,
};

/* SCSI Command Status Wrapper */
struct scsi_csw {
	/* Tag for the corresponding CBW */
	uint32_t tag;
	/* The amount of unprocessed data (depending on the direction of the transaction), if any */
	uint32_t data_residue;
	/* Returned status code */
	enum scsi_status status;
};

/* See document SPC section 4.5.1, Sense data introduction */
struct scsi_sense_data {
	bool valid;
	uint8_t response_code;
	uint8_t sense_key;
	uint8_t additional_sense_code;
	uint8_t additional_sense_code_qualifier;
};

struct usbh_msc_config {
	/* Driver instance index, formatted into the drive name alongside the LUN */
	uint8_t driver_index;
};

struct usbh_msc_data;

/* Data structure for each logical unit */
struct lun_data {
	/* Pointer back to the original driver data */
	struct usbh_msc_data *dev_data;
	/* Logical unit index */
	uint8_t index;
	/* Mountpoint for the Disk Access API */
	char disk_name[DISK_NAME_LENGTH];
	/* Disk access structure */
	struct disk_info disk;
	/* Wether the drive is read-only or writeable */
	bool write_protect;
	/* Address of the last block on the drive */
	uint32_t last_logical_block_address;
	/* Block size */
	uint32_t block_length_in_bytes;
	/* Unit state; whether initialization was successful, if the medium is actually connected
	 */
	enum {
		UNIT_STATE_NOT_READY,
		UNIT_STATE_READY,
		UNIT_STATE_NO_MEDIUM,
		UNIT_STATE_ERROR,
	} unit_state;
};

struct usbh_msc_data {
	/* Connected usb device */
	struct usb_device *udev;
	/* Mutual exclusion lock */
	struct k_mutex lock;
	/* Sync semaphore for waiting on asynchronous operations */
	struct k_sem sync;
	/* Bulk IN endpoint address */
	const struct usb_ep_descriptor *in_bulk_ep;
	/* Bulk OUT endpoint address */
	const struct usb_ep_descriptor *out_bulk_ep;
	/* Index of the last logical unit */
	uint8_t max_logical_unit;
	/* Information about each logical unit on the drive */
	struct lun_data lun_data[CONFIG_USBH_MSC_MAX_SUPPORTED_LUN];
	/* Index of the target interface */
	uint8_t target_iface;
	/* Tag given back as is to identify the response */
	uint32_t tag;
	/* Number of recovery attempts in the current transaction sequence */
	uint16_t recovery_attempts;
	/* Whether the current transaction sequence is undergoing a reset recovery procedure */
	bool currently_recovering;
};

static int reset_recovery(struct usbh_msc_data *dev_data, bool check_for_readiness);
static int usbh_msc_remove(struct usbh_class_data *const c_data);

/*
 * Start a new transaction sequence anew, with recovery state cleared
 */
static void clear_recovery_state(struct usbh_msc_data *dev_data)
{
	dev_data->recovery_attempts = 0u;
	dev_data->currently_recovering = false;
}

/*
 * Synchronization callback to wait for completion of asynchronous transfers
 * Should be passed to `usbh_xfer_alloc_with_buf` or `usbh_xfer_alloc` before queuing the
 * transfer, to then block on `dev_data->sync` in order to wait for completion. This
 * function only gives way to the semaphore; it doesn't analyze or deallocate anything.
 */
static int sync_cb(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	struct usbh_msc_data *dev_data = xfer->priv;

	if (xfer->err != 0) {
		LOG_DBG("Request finished %p, err %d, sem %i", xfer, xfer->err,
			k_sem_count_get(&dev_data->sync));
	}

	/* If the transfer was cancelled we deallocate it here */
	if (xfer->err == -ECONNRESET) {
		LOG_INF("Transfer %p cancelled", (void *)xfer);
		usbh_xfer_free(udev, xfer);

		return 0;
	}

	k_sem_give(&dev_data->sync);

	return 0;
}

/*
 * Block on `dev_data->sync` waiting for the `sync_cb` callback
 * This function waits for the last transfer enqueued with `sync_cb` as completion to be
 * done. If it actually completes it returns the error code; in the event of a timeout it
 * makes sure the transfer is no longer pending.
 */
static int wait_for_sync(struct usbh_msc_data *dev_data, struct uhc_transfer *xfer)
{
	int ret = 0;

	if (k_sem_take(&dev_data->sync, K_MSEC(SCSI_REQ_TIMEOUT)) != 0) {
		LOG_ERR("Timeout");

		ret = usbh_xfer_dequeue(dev_data->udev, xfer);
		/* While the semaphore take timed out, the transfer was actually already
		 * done and the callback on its way. */
		if (ret == -EALREADY) {
			/* Take the semaphore again to be sure that the callback is done */
			if (k_sem_take(&dev_data->sync, K_MSEC(SCSI_REQ_TIMEOUT)) != 0) {
				/* Should not happen */
				LOG_ERR("Double timeout");
			}
		}
		/* Dequeue failed */
		else if (ret != 0) {
			LOG_ERR("Failed to cancel transfer");
		}
		/* Dequeue succeeded, do nothing */
		else {
		}

		/* The USB host driver may still need to work with the transfer, so we leave it to
		 * the callback to deallocate it.
		 */

		return -ETIMEDOUT;
	} else {
		/* The transfer was successful, store the ret and free it */
		ret = xfer->err;
		usbh_xfer_free(dev_data->udev, xfer);
	}

	return ret;
}

/*
 * Get the endpoint address for the provided direction.
 * Returns the corresponding endpoint address.
 */
static inline uint8_t get_endpoint_for_direction(struct usbh_msc_data *dev_data,
						 enum scsi_direction direction)
{
	if (direction == SCSI_DIRECTION_DATA_OUT) {
		return dev_data->out_bulk_ep->bEndpointAddress;
	} else {
		return dev_data->in_bulk_ep->bEndpointAddress;
	}
}

/*
 * Convert the unit state to an errno code.
 */
static inline int unit_state_to_errno(struct lun_data *lun_data)
{
	switch (lun_data->unit_state) {
	case UNIT_STATE_READY: {
		return 0;
	}
	case UNIT_STATE_NO_MEDIUM: {
		return -ENOMEDIUM;
	}
	default: {
		return -EIO;
	}
	}
}

/*
 * Data transfer phase of a SCSI transaction.
 */
static int scsi_transfer_data(struct usbh_msc_data *dev_data, size_t data_length, uint8_t *data,
			      enum scsi_direction direction)
{
	struct uhc_transfer *xfer = NULL;
	struct net_buf *buf = NULL;
	int ret = 0;
	/* Pick the target endpoint */
	uint8_t endpoint_address = get_endpoint_for_direction(dev_data, direction);

	if (data_length == 0 || data == NULL) {
		/* Nothing to do */
		return 0;
	}

	xfer = usbh_xfer_alloc_with_buf(dev_data->udev, endpoint_address, sync_cb, dev_data,
					data_length);
	if (xfer == NULL) {
		return -ENOMEM;
	}
	buf = xfer->buf;

	/* If the data goes to the device copy it into the buffer */
	if (direction == SCSI_DIRECTION_DATA_OUT) {
		net_buf_add_mem(buf, data, data_length);
	}

	ret = usbh_xfer_enqueue(dev_data->udev, xfer);
	if (ret != 0) {
		LOG_ERR("Unable to enqueue the transfer: %i", ret);
		usbh_xfer_free(dev_data->udev, xfer);
		goto error_cleanup;
	}

	/* Wait for completion, deallocation handled automatically */
	ret = wait_for_sync(dev_data, xfer);
	if (ret != 0) {
		goto error_cleanup;
	}

	if (direction == SCSI_DIRECTION_DATA_IN) {
		/* If the data goes into the host copy it into the buffer */
		memcpy(data, buf->data, buf->len);
	}

	/* Return the number of bytes transferred */
	ret = buf->len;

error_cleanup:
	/* Done with the buffer and transfer */
	usbh_xfer_buf_free(dev_data->udev, buf);
	return ret;
}

/*
 * Read a pending Command Status Wrapper.
 */
static int scsi_read_status(struct usbh_msc_data *dev_data, struct scsi_csw *csw)
{
	uint8_t buffer[CSW_SIZE] = {};
	int ret = 0;
	uint32_t signature = 0;

	ret = scsi_transfer_data(dev_data, sizeof(buffer), buffer, SCSI_DIRECTION_DATA_IN);
	/* Stall, clear in endpoint and retry once */
	if (ret == -EPIPE) {
		LOG_DBG("CSW stalled, clearing endpoint and retrying...");
		ret = usbh_req_clear_sfs_halt(dev_data->udev,
					      dev_data->in_bulk_ep->bEndpointAddress);
		if (ret != 0) {
			LOG_ERR("Could not restore input endpoint: %i", ret);
			return ret;
		}

		/* Retry */
		ret = scsi_transfer_data(dev_data, sizeof(buffer), buffer, SCSI_DIRECTION_DATA_IN);
	}

	/* Other error */
	if (ret < 0) {
		LOG_ERR("Error reading status: %i", ret);
		return ret;
	}
	/* Not enough data */
	else if (ret < CSW_SIZE) {
		LOG_ERR("CSW too short: %i bytes", ret);
		return -ENODATA;
	}

	signature = sys_get_le32(&buffer[0]);
	if (signature != CSW_SIGNATURE) {
		LOG_ERR("Invalid signature: 0x%08X", signature);
		return -EINVAL;
	}

	/* Extract tag and status */
	csw->tag = sys_get_le32(&buffer[4]);
	csw->data_residue = sys_get_le32(&buffer[8]);
	csw->status = buffer[12];

	return 0;
}

/*
 * Send an SCSI Command Block Wrapper.
 */
static int scsi_command(struct usbh_msc_data *dev_data, struct scsi_cbw cbw)
{
	struct uhc_transfer *xfer = NULL;
	struct net_buf *buf = NULL;
	int ret = 0;
	uint8_t flags = 0;

	xfer = usbh_xfer_alloc_with_buf(dev_data->udev, dev_data->out_bulk_ep->bEndpointAddress,
					sync_cb, dev_data, CBW_SIZE);
	if (xfer == NULL) {
		return -ENOMEM;
	}
	buf = xfer->buf;

	if (cbw.direction == SCSI_DIRECTION_DATA_IN) {
		flags |= CBW_FLAGS_DIRECTION_IN;
	}

	/* See USB MSC Bulk-Only Transport specification, section 5.1 */

	/* dCBWSignature */
	net_buf_add_le32(buf, CBW_SIGNATURE);
	/* dCBWTag */
	net_buf_add_le32(buf, dev_data->tag);
	/* dCBWDataTransferLength */
	net_buf_add_le32(buf, cbw.data_transfer_length);
	/* bmCBWFlags */
	net_buf_add_u8(buf, flags);
	/* bCBWLUN */
	net_buf_add_u8(buf, cbw.lun);
	/* bCBWCBLength */
	net_buf_add_u8(buf, cbw.command_block_length);
	/* CBWCB */
	net_buf_add_mem(buf, cbw.command_block, cbw.command_block_length);
	/* CBW padding */
	while (buf->len < CBW_SIZE) {
		net_buf_add_u8(buf, 0);
	}

	ret = usbh_xfer_enqueue(dev_data->udev, xfer);
	if (ret != 0) {
		LOG_ERR("Unable to enqueue the transfer: %i", ret);
		usbh_xfer_free(dev_data->udev, xfer);
		goto error_cleanup;
	}

	/* Wait for completion, deallocation handled automatically */
	ret = wait_for_sync(dev_data, xfer);

error_cleanup:
	usbh_xfer_buf_free(dev_data->udev, buf);
	return ret;
}

/*
 * Full SCSI transaction: CBW - Data - CSW.
 */
static int scsi_transaction(struct usbh_msc_data *dev_data, struct scsi_cbw cbw, uint8_t *data)
{
	struct scsi_csw csw = {};
	int ret = 0;
	size_t received_data = 0u;
	bool skip_recovery = false;
	bool phase_error = false;

	/* Refuse a logical unit index bigger than the last supported one on the drive or
	 * over the allocation limit
	 */
	if (cbw.lun > dev_data->max_logical_unit || cbw.lun >= CONFIG_USBH_MSC_MAX_SUPPORTED_LUN) {
		return -EINVAL;
	}

	/* Too much data */
	if (cbw.command_block_length > CBW_COMMAND_BLOCK_MAX_LENGTH) {
		return -EINVAL;
	}

	do {
		dev_data->recovery_attempts++;

		/* If a failure occurred attempt recovery */
		if (ret != 0) {
			LOG_WRN("Transaction failed, attempting %ith recovery",
				dev_data->recovery_attempts);

			if (dev_data->currently_recovering) {
				/* An ongoing recovery failed; no use trying again */
				phase_error = false;
				break;
			}

			ret = reset_recovery(dev_data, true);
			/* The recovery procedure negates the phase error */
			phase_error = false;

			if (ret != 0) {
				LOG_ERR("Unable to recover the USB communication: %i", ret);
				skip_recovery = true;
				break;
			} else if (dev_data->lun_data[cbw.lun].unit_state != UNIT_STATE_READY) {
				LOG_ERR("Unit not ready after reset");
				skip_recovery = true;
				ret = -EAGAIN;
				break;
			} else {
				LOG_WRN("Communication recovered, retrying...");
			}
		}

		/* Send the command */
		ret = scsi_command(dev_data, cbw);
		if (ret != 0) {
			LOG_ERR("Unable to send CBW: %i", ret);
			continue;
		}

		/* Data phase */
		if (cbw.data_transfer_length > 0 && data) {
			ret = scsi_transfer_data(dev_data, cbw.data_transfer_length, data,
						 cbw.direction);
			/* The device stalled the transaction; it's a valid response, we should just
			 * clear the endpoint and continue with the status to check what happened */
			if (ret == -EPIPE) {
				/* Pick the target endpoint */
				uint8_t endpoint_address =
					get_endpoint_for_direction(dev_data, cbw.direction);

				LOG_DBG("Data stalled, clearing endpoint");
				ret = usbh_req_clear_sfs_halt(dev_data->udev, endpoint_address);
			}

			/* Any other error */
			if (ret < 0) {
				LOG_ERR("Error while transferring data: %i", ret);
				continue;
			}

			received_data = ret;
		}

		/* Get the status */
		ret = scsi_read_status(dev_data, &csw);
		if (ret != 0) {
			LOG_ERR("Unable to get CSW: %i", ret);
			continue;
		}

		/* Wrong tag */
		if (csw.tag != dev_data->tag) {
			ret = -EIO;
			LOG_ERR("Mismatching CBW and CSW tags: 0x%08X vs 0x%08X", dev_data->tag,
				csw.tag);
			continue;
		}
		/* Increase the tag */
		dev_data->tag++;

		/* Check the status */
		switch (csw.status) {
		case SCSI_STATUS_COMMAND_FAILED: {
			LOG_ERR("SCSI command failed");
			/* This is the device reporting an issue: recovery won't help */
			skip_recovery = true;
			ret = -EFAULT;
			break;
		}
		case SCSI_STATUS_PHASE_ERROR: {
			/* Problem with the BOT communication protocol */
			LOG_ERR("SCSI phase error");
			ret = -EIO;
			phase_error = true;
			break;
		}
		case SCSI_STATUS_COMMAND_PASSED: {
			if (csw.data_residue > cbw.data_transfer_length) {
				LOG_ERR("Impossible data residue: %i over %i", csw.data_residue,
					cbw.data_transfer_length);
				ret = -EIO;
			} else {
				size_t reported_transferred =
					cbw.data_transfer_length - csw.data_residue;

				if (cbw.direction == SCSI_DIRECTION_DATA_IN) {
					if (received_data != reported_transferred) {
						LOG_WRN("CSW residue implies %zu received bytes "
							"but %zu "
							"were actually received. Assuming the "
							"smaller value",
							reported_transferred, received_data);
					}

					ret = MIN(received_data, reported_transferred);
				} else {
					ret = reported_transferred;
				}
			}
			break;
		}
		default: {
			LOG_WRN("Unknown status: 0x%02X", csw.status);
			ret = -EIO;
			break;
		}
		}
	} while (ret < 0 && dev_data->recovery_attempts < CONFIG_USBH_MSC_RECOVERY_ATTEMPTS &&
		 !skip_recovery && ret != -ENOMEM);

	if (ret >= 0 && dev_data->recovery_attempts > 0) {
		/* `recovery_attempts` is prehemptively incremented at the start of the loop, but is
		 * shared among certain transaction sequences; it should be decremented if the last
		 * attempt was successful to avoid building up unfairly towards following
		 * transactions
		 */
		dev_data->recovery_attempts--;
	}

	/* Pending phase error, possibly due to excessive attempts; recover without checking for
	 * readiness because that would eventually recurse here
	 */
	if (phase_error) {
		int error = reset_recovery(dev_data, false);

		if (error != 0) {
			LOG_ERR("Could not reset recovery: %i", error);
		}

		dev_data->lun_data[cbw.lun].unit_state = UNIT_STATE_NOT_READY;
	}

	return ret;
}

/*
 * Request SCSI sense data.
 */
static int scsi_request_sense(struct usbh_msc_data *dev_data, struct scsi_sense_data *sense_data,
			      uint8_t lun_index)
{
	int ret = 0;
	uint8_t buffer[SCSI_MAX_SENSE_DATA] = {0u};
	uint8_t const command_block[6u] = {
		SCSI_COMMAND_REQUEST_SENSE, 0u, 0u, 0u, SCSI_MAX_SENSE_DATA, 0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = sizeof(buffer),
	};
	/* We need data up to the 14th byte */
	const size_t required_sense_data = 14u;

	ret = scsi_transaction(dev_data, cbw, buffer);

	if (ret < 0) {
		LOG_ERR("Request sense failed: %i", ret);
		return ret;
	}
	/* Not enough data */
	else if (ret < required_sense_data) {
		LOG_ERR("Not enough sense data: %i", ret);
		return -EIO;
	}

	/* See SPC document, section 4.4 */
	sense_data->valid = (buffer[0u] & 0x80u) > 0u;
	sense_data->response_code = buffer[0u] & 0x7Fu;
	sense_data->sense_key = buffer[2u] & 0xFu;
	sense_data->additional_sense_code = buffer[12u];
	sense_data->additional_sense_code_qualifier = buffer[13u];

	return 0;
}

/*
 * Request sense data, converting it to a proper errno value.
 */
static int check_sense(struct usbh_msc_data *dev_data, uint8_t lun_index)
{
	struct scsi_sense_data sense_data = {};
	int ret = scsi_request_sense(dev_data, &sense_data, lun_index);

	if (ret != 0) {
		LOG_ERR("Failed to request sense data: %i", ret);
		return ret;
	}

	LOG_DBG("Sense response: 0x%02X 0x%02X 0x%02X 0x%02X", sense_data.response_code,
		sense_data.sense_key, sense_data.additional_sense_code,
		sense_data.additional_sense_code_qualifier);

	if (sense_data.response_code == SCSI_SENSE_DATA_RESPONSE_CODE_CURRENT) {
		/* The device is specifically reporting not being ready. Wait a bit
		 * and retry */
		if (sense_data.sense_key == SCSI_SENSE_DATA_KEY_UNIT_ATTENTION &&
		    sense_data.additional_sense_code == SCSI_SENSE_DATA_ASC_NOT_READY_TO_READY) {
			ret = -EAGAIN;
		}
		/* Medium error, may disappear on retry */
		else if (sense_data.sense_key == SCSI_SENSE_DATA_KEY_MEDIUM_ERROR) {
			ret = -EIO;
		} else if (sense_data.additional_sense_code ==
			   SCSI_SENSE_DATA_ASC_MEDIUM_NOT_PRESENT) {
			ret = -ENOMEDIUM;
		}
		/* The device is reporting some other condition, use a generic error */
		else {
			ret = -EIO;
		}
	}
	/* We are only interested in the current data */
	else {
		ret = -EINVAL;
	}

	return ret;
}

/*
 * Check if the device is ready.
 */
static int test_unit_ready(struct usbh_msc_data *dev_data, uint8_t lun_index)
{
	int ret = 0;
	size_t attempts = 0u;
	uint8_t const command_block[6u] = {
		SCSI_COMMAND_TEST_UNIT_READY, 0u, 0u, 0u, 0u, 0u,
	};
	struct scsi_cbw cbw = {
		.direction = SCSI_DIRECTION_DATA_OUT,
		.lun = lun_index,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = 0u,
	};

	/* Repeat the procedure for a number of attempts */
	for (attempts = 0; attempts < CONFIG_USBH_MSC_TEST_UNIT_READY_ATTEMPTS; attempts++) {
		ret = scsi_transaction(dev_data, cbw, NULL);
		if (ret == 0) {
			/* Done! */
			break;
		}
		/* Device is not ready, inspect the sense data */
		if (ret == -EFAULT) {
			ret = check_sense(dev_data, lun_index);
			/* The device is specifically reporting not being ready. Wait a bit
			 * and retry */
			if (ret == -EAGAIN) {
				LOG_DBG("Unit not ready yet, retrying...");
			}
			/* No media present, cannot be ready */
			else if (ret == -ENOMEDIUM) {
				LOG_DBG("Medium on %i not present yet", lun_index);
				break;
			} else {
				/* The device is reporting some other condition, but the approach is
				 * the same: wait a bit and retry. */
			}
		} else {
			LOG_ERR("Unable to test the unit for readiness: %i", ret);
			return ret;
		}

		k_msleep(100);
	}

	if (attempts >= CONFIG_USBH_MSC_TEST_UNIT_READY_ATTEMPTS) {
		LOG_ERR("Was not ready in time");
		return -ETIMEDOUT;
	}

	return ret;
}

/*
 * Read the capacity information of a logical unit, storing it in `dev_data`.
 */
static int read_capacity(struct usbh_msc_data *dev_data, uint8_t lun_index)
{
	int ret = 0;
	uint8_t response[SCSI_READ_CAPACITY_10_DATA_LENGTH] = {};
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_READ_CAPACITY_10, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = sizeof(response),
	};
	uint32_t last_logical_block_address = 0;
	uint32_t block_length_in_bytes = 0;

	ret = scsi_transaction(dev_data, cbw, response);

	/* Transaction refused */
	if (ret == -EFAULT) {
		return check_sense(dev_data, lun_index);
	}
	/* Other error */
	else if (ret < 0) {
		LOG_ERR("Unable to check the unit's capacity: %i", ret);
		return ret;
	}
	/* Not enough data */
	else if (ret < 8) {
		LOG_ERR("Not enough capacity data: %i", ret);
		return -EIO;
	}

	/* SCSI is an old protocol, uses big endian format */
	last_logical_block_address = sys_get_be32(&response[0]);
	/* Disk size too big */
	if (last_logical_block_address == 0xFFFFFFFF) {
		return -ENOTSUP;
	}

	block_length_in_bytes = sys_get_be32(&response[4]);
	/* Invalid response */
	if (block_length_in_bytes == 0) {
		return -EINVAL;
	}

	dev_data->lun_data[lun_index].last_logical_block_address = last_logical_block_address;
	dev_data->lun_data[lun_index].block_length_in_bytes = block_length_in_bytes;

	return 0;
}

/*
 * Read the mode information of a logical unit, storing it in `dev_data`.
 */
static int mode_sense_6(struct usbh_msc_data *dev_data, uint8_t lun_index)
{
	int ret = 0;
	uint8_t response[SCSI_MODE_SENSE_DATA_LENGTH] = {};
	uint8_t const command_block[6u] = {
		SCSI_COMMAND_MODE_SENSE_6,
		1u << 3u,                            /* Disable block descriptors */
		0x3Fu,                               /* All pages, current values */
		0u,                                  /* Subpage */
		SCSI_MODE_SENSE_DATA_LENGTH & 0xFFu, /* Data length */
		0u,
	};
	/* We need data up to the 3th byte */
	const size_t required_sense_data = 3u;

	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = sizeof(response),
	};

	ret = scsi_transaction(dev_data, cbw, response);

	/* Transaction refused */
	if (ret == -EFAULT) {
		return check_sense(dev_data, lun_index);
	}
	/* Other error */
	else if (ret < 0) {
		LOG_ERR("Unable to check the unit's mode: %i", ret);
		return ret;
	}
	/* Not enough data */
	else if (ret < required_sense_data) {
		LOG_ERR("Not enough mode data: %i", ret);
		return -EIO;
	}

	dev_data->lun_data[lun_index].write_protect = (response[2u] & (1u << 7u)) > 0;

	return 0;
}

/*
 * Read the mode information of a logical unit, storing it in `dev_data`.
 */
static int mode_sense_10(struct usbh_msc_data *dev_data, uint8_t lun_index)
{
	int ret = 0;
	uint8_t response[SCSI_MODE_SENSE_DATA_LENGTH] = {};
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_MODE_SENSE_10,
		1u << 3u, /* Disable block descriptors */
		0x3Fu,    /* All pages, current values */
		0u,       /* Subpage */
		0u,
		0u,
		0u,
		(SCSI_MODE_SENSE_DATA_LENGTH >> 8u) & 0xFFu, /* Data length */
		SCSI_MODE_SENSE_DATA_LENGTH & 0xFFu,
		0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = sizeof(response),
	};

	ret = scsi_transaction(dev_data, cbw, response);

	/* Transaction refused */
	if (ret == -EFAULT) {
		return check_sense(dev_data, lun_index);
	}
	/* Other error */
	else if (ret < 0) {
		LOG_ERR("Unable to check the unit's mode: %i", ret);
		return ret;
	}
	/* Not enough data */
	else if (ret < 4) {
		LOG_ERR("Not enough mode data: %i", ret);
		return -EIO;
	}

	dev_data->lun_data[lun_index].write_protect = (response[3u] & (1u << 7u)) > 0;

	return 0;
}

#ifndef CONFIG_USBH_MSC_IGNORE_SYNC
/*
 * Flush the device's caches.
 */
static int synchronize_cache(struct usbh_msc_data *dev_data, uint8_t lun_index)
{
	int ret = 0;
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_SYNCHRONIZE_CACHE_10,
		0u, /* Sync to medium */
		0u, /* LBA */
		0u,
		0u,
		0u,
		0u, /* Group number */
		0u, /* Number of blocks (all) */
		0u,
		0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_OUT,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = 0u,
	};

	ret = scsi_transaction(dev_data, cbw, NULL);

	/* Transaction refused */
	if (ret == -EFAULT) {
		struct scsi_sense_data sense_data = {};

		ret = scsi_request_sense(dev_data, &sense_data, lun_index);

		if (ret != 0) {
			LOG_ERR("Failed to request sense data: %i", ret);
			return ret;
		}

		if (sense_data.response_code == SCSI_SENSE_DATA_RESPONSE_CODE_CURRENT) {
			if ((sense_data.sense_key == SCSI_SENSE_DATA_KEY_ILLEGAL_REQUEST &&
			     sense_data.additional_sense_code ==
				     SCSI_SENSE_DATA_ASC_INVALID_COMMAND_OPERATION_CODE) ||
			    /* Some cheap device cut corners and don't implement proper
			     * response codes */
			    sense_data.additional_sense_code == 0u) {
				LOG_DBG("Sync unsupported, ignoring");
				return 0;
			}
			/* Other error */
			else {
				LOG_ERR("Unable to sync the unit's caches: %i - %i",
					sense_data.sense_key, sense_data.additional_sense_code);
				return -EIO;
			}
		}
		/* We are only interested in the current data */
		else {
			ret = -EINVAL;
		}
	}

	return ret;
}
#endif /* CONFIG_USBH_MSC_IGNORE_SYNC */

/*
 * Read a number of blocks from a logical unit.
 */
static int read_blocks(struct usbh_msc_data *dev_data, uint8_t lun_index, uint32_t lba,
		       uint16_t block_count, uint8_t *buffer)
{
	int ret = 0;
	uint32_t transfer_length =
		block_count * dev_data->lun_data[lun_index].block_length_in_bytes;
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_READ_10,
		0u,                   /* Obsolete flags */
		(lba >> 24u) & 0xFFu, /* Big Endian address */
		(lba >> 16u) & 0xFFu,
		(lba >> 8u) & 0xFFu,
		lba & 0xFFu,
		0u,                          /* Obsolete flags */
		(block_count >> 8u) & 0xFFu, /* Big Endian count */
		block_count & 0xFFu,
		0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_IN,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = transfer_length,
	};

	LOG_DBG("Read %i blocks starting from %i (%i bytes)", block_count, lba, transfer_length);

	ret = scsi_transaction(dev_data, cbw, buffer);

	/* Transaction refused */
	if (ret == -EFAULT) {
		ret = check_sense(dev_data, lun_index);
		if (ret == -ENOMEDIUM) {
			dev_data->lun_data[lun_index].unit_state = UNIT_STATE_NO_MEDIUM;
			dev_data->lun_data[lun_index].last_logical_block_address = 0;
			dev_data->lun_data[lun_index].block_length_in_bytes = 0;
		}
		return ret;
	}
	/* Other error */
	else if (ret < 0) {
		LOG_ERR("Unable to read: %i", ret);
		return ret;
	}
	/* Not enough data */
	else if (ret < transfer_length) {
		LOG_ERR("Not enough data read from unit: %i", ret);
		return -EIO;
	}

	return 0;
}

/*
 * Write a number of blocks to a logical unit.
 */
static int write_blocks(struct usbh_msc_data *dev_data, uint8_t lun_index, uint32_t lba,
			uint16_t block_count, const uint8_t *buffer)
{
	int ret = 0;
	uint32_t transfer_length =
		block_count * dev_data->lun_data[lun_index].block_length_in_bytes;
	uint8_t const command_block[10u] = {
		SCSI_COMMAND_WRITE_10,
		0u,                   /* Obsolete flags */
		(lba >> 24u) & 0xFFu, /* Big Endian address */
		(lba >> 16u) & 0xFFu,
		(lba >> 8u) & 0xFFu,
		lba & 0xFFu,
		0u,                          /* Obsolete flags */
		(block_count >> 8u) & 0xFFu, /* Big Endian count */
		block_count & 0xFFu,
		0u,
	};
	struct scsi_cbw cbw = {
		.lun = lun_index,
		.direction = SCSI_DIRECTION_DATA_OUT,
		.command_block_length = sizeof(command_block),
		.command_block = command_block,
		.data_transfer_length = transfer_length,
	};

	LOG_DBG("Write %i blocks starting from %i (%i bytes)", block_count, lba, transfer_length);

	ret = scsi_transaction(dev_data, cbw, (uint8_t *)buffer);

	/* Transaction refused */
	if (ret == -EFAULT) {
		ret = check_sense(dev_data, lun_index);
		if (ret == -ENOMEDIUM) {
			dev_data->lun_data[lun_index].unit_state = UNIT_STATE_NO_MEDIUM;
		}
		return ret;
	}
	/* Other error */
	else if (ret < 0) {
		LOG_ERR("Unable to write: %i", ret);
		return ret;
	}
	/* Not enough data */
	else if (ret < transfer_length) {
		LOG_ERR("Unit did not write all data: %i", ret);
		return -EIO;
	}

	return 0;
}

/*
 * Get the maximum logical unit index for this device.
 */
static int get_max_lun(struct usbh_msc_data *dev_data)
{
	struct net_buf *buf;
	int ret = 0;

	buf = usbh_xfer_buf_alloc(dev_data->udev, 1);
	if (!buf) {
		return -ENOMEM;
	}

	/* See USB MSC Bulk-Only Transport specification, section 3.2 */
	const uint8_t bmRequestType = (USB_REQTYPE_DIR_TO_HOST << 7u) |
				      (USB_REQTYPE_TYPE_CLASS << 5u) |
				      (USB_REQTYPE_RECIPIENT_INTERFACE << 0u);
	ret = usbh_req_setup(dev_data->udev, bmRequestType, GET_MAX_LUN, 0, dev_data->target_iface,
			     1, buf);
	/* A stalled GET_MAX_LUN request shall be interpreted as a unique unit */
	if (ret == -EPIPE) {
		dev_data->max_logical_unit = 0;
		ret = 0;
	}
	/* Otherwise read the response */
	else if (ret == 0) {
		if (buf->len < 1) {
			dev_data->max_logical_unit = 0u;
			return -ENODATA;
		}
		dev_data->max_logical_unit = buf->data[0] & 0xFu;
	}

	usbh_xfer_buf_free(dev_data->udev, buf);

	return ret;
}

/*
 * Require a soft reset of the device.
 */
static int bulk_only_mass_storage_reset(struct usbh_msc_data *dev_data)
{
	/* See USB MSC Bulk-Only Transport specification, section 3.1 */
	const uint8_t bmRequestType = (USB_REQTYPE_DIR_TO_DEVICE << 7u) |
				      (USB_REQTYPE_TYPE_CLASS << 5u) |
				      (USB_REQTYPE_RECIPIENT_INTERFACE << 0u);
	return usbh_req_setup(dev_data->udev, bmRequestType, BULK_ONLY_MASS_STORAGE_RESET, 0,
			      dev_data->target_iface, 0, NULL);
}

/*
 * Apply a reset recovery procedure (see SPC section 5.3.4).
 */
static int reset_recovery(struct usbh_msc_data *dev_data, bool check_for_readiness)
{
	int ret = 0;
	bool currently_recovering_was = dev_data->currently_recovering;

	dev_data->currently_recovering = true;

	/* Soft reset the device */
	ret = bulk_only_mass_storage_reset(dev_data);
	if (ret != 0) {
		LOG_ERR("Could not issue mass storage reset: %i", ret);
		dev_data->currently_recovering = currently_recovering_was;
		return ret;
	}

	/* Clear input endpoint */
	ret = usbh_req_clear_sfs_halt(dev_data->udev, dev_data->in_bulk_ep->bEndpointAddress);
	if (ret != 0) {
		LOG_ERR("Could not restore input endpoint: %i", ret);
		dev_data->currently_recovering = currently_recovering_was;
		return ret;
	}

	/* Clear output endpoint */
	ret = usbh_req_clear_sfs_halt(dev_data->udev, dev_data->out_bulk_ep->bEndpointAddress);
	if (ret != 0) {
		LOG_ERR("Could not restore output endpoint: %i", ret);
		dev_data->currently_recovering = currently_recovering_was;
		return ret;
	}

	if (check_for_readiness) {
		/* Check for readiness again */
		for (size_t lun_index = 0; lun_index <= dev_data->max_logical_unit &&
					   lun_index < CONFIG_USBH_MSC_MAX_SUPPORTED_LUN;
		     lun_index++) {
			ret = test_unit_ready(dev_data, lun_index);
			if (ret == -ENOMEDIUM) {
				dev_data->lun_data[lun_index].unit_state = UNIT_STATE_NO_MEDIUM;
				LOG_WRN("No medium present in unit: %i", lun_index);
			} else if (ret != 0) {
				dev_data->lun_data[lun_index].unit_state = UNIT_STATE_NOT_READY;
				LOG_ERR("Unit %i not ready: %i", lun_index, ret);
			} else {
				dev_data->lun_data[lun_index].unit_state = UNIT_STATE_READY;
			}
		}
	}

	dev_data->currently_recovering = currently_recovering_was;

	return 0;
}

/*
 * Scan endpoints in the interface.
 */
static int scan_interface_endpoints(struct usbh_msc_data *dev_data, uint8_t iface)
{
	const struct usb_desc_header *desc;
	const struct usb_ep_descriptor *ep_desc;
	const struct usb_if_descriptor *if_desc;
	int ep_count = 0;

	if_desc = (const void *)usbh_desc_get_iface(dev_data->udev, iface);
	if (if_desc == NULL) {
		LOG_ERR("Failed to find interface %u", iface);
		return -ENOSYS;
	}

	/* Iterate through all descriptors following the interface descriptor */
	desc = (const struct usb_desc_header *)if_desc;
	while ((desc = usbh_desc_get_next(desc)) != NULL && ep_count < if_desc->bNumEndpoints &&
	       (dev_data->in_bulk_ep == NULL || dev_data->out_bulk_ep == NULL)) {
		{
			/* Stop if we hit another interface descriptor */
			if (desc->bDescriptorType == USB_DESC_INTERFACE) {
				break;
			}

			/* Process endpoint descriptors */
			if (desc->bDescriptorType == USB_DESC_ENDPOINT) {
				ep_desc = (const void *)desc;

				/* Only pick bulk endpoints */
				if ((ep_desc->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
				    USB_EP_TYPE_BULK) {
					/* Input bulk endpoint */
					if (USB_EP_DIR_IS_IN(ep_desc->bEndpointAddress) &&
					    dev_data->in_bulk_ep == NULL) {
						LOG_DBG("Input bulk endpoint: 0x%02X",
							ep_desc->bEndpointAddress);
						dev_data->in_bulk_ep = ep_desc;
					}
					/* Output bulk endpoint */
					else if (USB_EP_DIR_IS_OUT(ep_desc->bEndpointAddress) &&
						 dev_data->out_bulk_ep == NULL) {
						LOG_DBG("Output bulk endpoint: 0x%02X",
							ep_desc->bEndpointAddress);
						dev_data->out_bulk_ep = ep_desc;
					}
				}

				ep_count++;
			}
		}
	}

	if (dev_data->in_bulk_ep == NULL || dev_data->out_bulk_ep == NULL) {
		LOG_ERR("Could not find bulk endpoints");
		return -EIO;
	}

	return 0;
}

/*
 * Retrieve the containing `struct lun_data` from a `disk` field pointer.
 */
static inline struct lun_data *get_lun_data_from_disk(struct disk_info *disk)
{
	return CONTAINER_OF(disk, struct lun_data, disk);
}

/*
 * Fetch medatada (write protect status, capacity) for the specified unit.
 */
static int retrieve_unit_metadata(struct usbh_msc_data *dev_data, uint8_t lun_index)
{
	int ret = 0;
	struct lun_data *lun_data = &dev_data->lun_data[lun_index];

	/* Fetch the unit's capacity */
	ret = read_capacity(dev_data, lun_index);
	if (ret != 0) {
		LOG_ERR("Could not read the unit's capacity: %i", ret);
		return ret;
	}

	/* Fetch the unit's mode */
	ret = mode_sense_10(dev_data, lun_index);
	if (ret != 0) {
		LOG_WRN("Mode sense (10) failed, falling back to Mode sense (6)");
		ret = mode_sense_6(dev_data, lun_index);
	}

	if (ret != 0) {
		LOG_WRN("Could not figure out if the device is read-only: %i", ret);
	} else if (lun_data->write_protect) {
		LOG_INF("Unit %i is read-only", lun_index);
	} else {
		LOG_INF("Unit %i is writeable", lun_index);
	}

	return ret;
}

/*
 * Attempt to initialize a unit
 */
static int initialize_unit(struct usbh_msc_data *dev_data, uint8_t lun_index)
{
	int ret = 0;
	struct lun_data *lun_data = &dev_data->lun_data[lun_index];

	/* Test the unit for readiness */
	ret = test_unit_ready(dev_data, lun_index);
	/* No medium connected */
	if (ret == -ENOMEDIUM) {
		lun_data->unit_state = UNIT_STATE_NO_MEDIUM;
		return ret;
	}
	/* Other error */
	else if (ret != 0) {
		lun_data->unit_state = UNIT_STATE_NOT_READY;
		LOG_ERR("Unit not ready: %i", ret);
		return ret;
	}
	/* Unit is ready */
	else {
		lun_data->unit_state = UNIT_STATE_READY;
		LOG_DBG("Unit %i is ready", lun_index);
	}

	ret = retrieve_unit_metadata(dev_data, lun_index);
	if (ret == 0) {
		lun_data->unit_state = UNIT_STATE_READY;
	} else {
		lun_data->unit_state = UNIT_STATE_ERROR;
	}

	return ret;
}

static int disk_access_status(struct disk_info *disk)
{
	struct lun_data *lun_data = get_lun_data_from_disk(disk);
	struct usbh_msc_data *dev_data = lun_data->dev_data;
	int ret = 0;

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	/* No device connected */
	if (dev_data->udev == NULL) {
		ret = -ENODEV;
		goto error_cleanup;
	}

	switch (lun_data->unit_state) {
	case UNIT_STATE_NO_MEDIUM: {
		ret = DISK_STATUS_NOMEDIA;
		break;
	}
	case UNIT_STATE_READY: {
		if (lun_data->write_protect) {
			ret = DISK_STATUS_WR_PROTECT;
		}
		/* If the disk is registered the drive has been probed and it's ready */
		else {
			ret = DISK_STATUS_OK;
		}
		break;
	}
	default: {
		ret = DISK_STATUS_UNINIT;
		break;
	}
	}

error_cleanup:
	k_mutex_unlock(&dev_data->lock);

	return ret;
}

static int disk_access_read(struct disk_info *disk, uint8_t *data_buf, uint32_t start_sector,
			    uint32_t num_sector)
{
	struct lun_data *lun_data = get_lun_data_from_disk(disk);
	struct usbh_msc_data *dev_data = lun_data->dev_data;
	int ret = 0;

	if (num_sector > 0xFFFF) {
		LOG_ERR("Cannot read more than 65535 sectors, and %" PRIu32 " were requested",
			num_sector);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	/* No device connected */
	if (dev_data->udev == NULL) {
		ret = -ENODEV;
		goto error_cleanup;
	}

	if ((uint64_t)start_sector + num_sector >
	    (uint64_t)lun_data->last_logical_block_address + 1) {
		k_mutex_unlock(&dev_data->lock);
		return -EINVAL;
	}

	if (lun_data->unit_state == UNIT_STATE_READY) {
		clear_recovery_state(dev_data);
		ret = read_blocks(dev_data, lun_data->index, start_sector, num_sector, data_buf);
	} else {
		ret = unit_state_to_errno(lun_data);
	}

error_cleanup:
	k_mutex_unlock(&dev_data->lock);

	return ret;
}

static int disk_access_write(struct disk_info *disk, const uint8_t *data_buf, uint32_t start_sector,
			     uint32_t num_sector)
{
	struct lun_data *lun_data = get_lun_data_from_disk(disk);
	struct usbh_msc_data *dev_data = lun_data->dev_data;
	int ret = 0;

	if (num_sector > 0xFFFF) {
		LOG_ERR("Cannot write more than 65535 sectors, and %" PRIu32 " were requested",
			num_sector);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	/* No device connected */
	if (dev_data->udev == NULL) {
		ret = -ENODEV;
		goto error_cleanup;
	}

	if ((uint64_t)start_sector + num_sector >
	    (uint64_t)lun_data->last_logical_block_address + 1) {
		k_mutex_unlock(&dev_data->lock);
		return -EINVAL;
	}

	if (lun_data->unit_state == UNIT_STATE_READY) {
		clear_recovery_state(dev_data);
		ret = write_blocks(dev_data, lun_data->index, start_sector, num_sector, data_buf);
	} else {
		ret = unit_state_to_errno(lun_data);
	}

error_cleanup:
	k_mutex_unlock(&dev_data->lock);

	return ret;
}

static int disk_access_erase(struct disk_info *disk, uint32_t start_sector, uint32_t num_sector)
{
	ARG_UNUSED(disk);
	ARG_UNUSED(start_sector);
	ARG_UNUSED(num_sector);
	/* Erasing doesn't make sense in the context of USB drives, it should be handled by
	 * the device's own firmware */
	return -ENOTSUP;
}

static int disk_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	struct lun_data *lun_data = get_lun_data_from_disk(disk);
	struct usbh_msc_data *dev_data = lun_data->dev_data;
	int ret = 0;

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	/* No device connected */
	if (dev_data->udev == NULL) {
		ret = -ENODEV;
		goto error_cleanup;
	}

	clear_recovery_state(dev_data);

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT: {
		if (lun_data->unit_state == UNIT_STATE_READY) {
			if (buff != NULL) {
				*(uint32_t *)buff = lun_data->last_logical_block_address + 1;
			} else {
				ret = -EINVAL;
			}
		} else {
			ret = unit_state_to_errno(lun_data);
		}
		break;
	}
	case DISK_IOCTL_GET_SECTOR_SIZE: {
		if (lun_data->unit_state == UNIT_STATE_READY) {
			if (buff != NULL) {
				*(uint32_t *)buff = lun_data->block_length_in_bytes;
			} else {
				ret = -EINVAL;
			}
		} else {
			ret = unit_state_to_errno(lun_data);
		}
		break;
	}
	case DISK_IOCTL_CTRL_SYNC: {
		if (lun_data->unit_state == UNIT_STATE_READY) {
#ifndef CONFIG_USBH_MSC_IGNORE_SYNC
			ret = synchronize_cache(dev_data, lun_data->index);
#endif /* CONFIG_USBH_MSC_IGNORE_SYNC */
		} else {
			ret = unit_state_to_errno(lun_data);
		}
		break;
	}
	case DISK_IOCTL_CTRL_DEINIT: {
		break;
	}
	case DISK_IOCTL_CTRL_INIT: {
		LOG_DBG("Init IOCTL");
		switch (lun_data->unit_state) {
		case UNIT_STATE_READY: {
			/* Unit is ready and responsive */
			break;
		}
		default: {
			/* Unit was not ready; retry */
			ret = initialize_unit(dev_data, lun_data->index);
			break;
		}
		}
		break;
	}
	default: {
		ret = -ENOTSUP;
		break;
	}
	}

error_cleanup:
	k_mutex_unlock(&dev_data->lock);

	return ret;
}

static int disk_access_init(struct disk_info *disk)
{
	return disk_access_ioctl(disk, DISK_IOCTL_CTRL_INIT, NULL);
}

/*
 * Disk access vtable
 */
static const struct disk_operations disk_operations = {
	.init = disk_access_init,
	.status = disk_access_status,
	.read = disk_access_read,
	.write = disk_access_write,
	.erase = disk_access_erase,
	.ioctl = disk_access_ioctl,
};

/*
 * Initialize the MSC host class driver.
 */
static int usbh_msc_init(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct usbh_msc_data *dev_data = (void *)dev->data;

	memset(dev_data, 0x00, sizeof(*dev_data));

	k_mutex_init(&dev_data->lock);
	k_sem_init(&dev_data->sync, 0, 1);

	return 0;
}

/*
 * Probe the USB class driver after a device has been found.
 */
static int usbh_msc_probe(struct usbh_class_data *const c_data, struct usb_device *const udev,
			  uint8_t iface)
{
	const struct device *dev = c_data->priv;
	struct usbh_msc_data *dev_data = (void *)dev->data;
	const struct usbh_msc_config *dev_config = (void *)dev->config;
	int ret;

	LOG_INF("MSC device connected");

	if ((udev == NULL) || (udev->state != USB_STATE_CONFIGURED)) {
		LOG_ERR("USB device not properly configured");
		return -ENODEV;
	}

	if (dev_data == NULL) {
		LOG_ERR("No MSC device instance available");
		return -ENODEV;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	dev_data->udev = udev;

	/* Convert device-level match to interface 0 */
	if (iface == USBH_CLASS_IFNUM_DEVICE) {
		dev_data->target_iface = 0;
	} else {
		dev_data->target_iface = iface;
	}

	/* Fetch bulk endpoints */
	ret = scan_interface_endpoints(dev_data, dev_data->target_iface);
	if (ret != 0) {
		LOG_ERR("Failed to scan endpoints: %d", ret);
		goto error_cleanup;
	}

	/* Set control interface to default alternate setting (0) */
	ret = usbh_device_interface_set(udev, dev_data->target_iface, 0, false);
	if (ret != 0) {
		LOG_ERR("Failed to set control interface alternate setting: %d", ret);
		goto error_cleanup;
	}

	/* Fetch max logical unit address */
	ret = get_max_lun(dev_data);
	if (ret != 0) {
		LOG_ERR("Could not get max LUN: %i", ret);
		goto error_cleanup;
	}
	LOG_DBG("MAX LUN %i", dev_data->max_logical_unit);

	for (size_t lun_index = 0; lun_index <= dev_data->max_logical_unit &&
				   lun_index < CONFIG_USBH_MSC_MAX_SUPPORTED_LUN;
	     lun_index++) {
		struct lun_data *lun_data = &dev_data->lun_data[lun_index];

		clear_recovery_state(dev_data);
		ret = initialize_unit(dev_data, lun_index);
		if (ret != 0 && ret != -ENOMEDIUM) {
			LOG_ERR("Unable to initialize unit %zu: %i", lun_index, ret);
			goto error_cleanup;
		}

		/* Populate other unit data */
		lun_data->disk.dev = dev;
		lun_data->disk.ops = &disk_operations;
		snprintf(lun_data->disk_name, sizeof(lun_data->disk_name), "USB%i_%zu",
			 dev_config->driver_index, lun_index);
		lun_data->disk.name = lun_data->disk_name;
		lun_data->index = lun_index;
		lun_data->dev_data = dev_data;

		/* Register the unit as an accessible disk */
		ret = disk_access_register(&lun_data->disk);
		if (ret != 0) {
			LOG_ERR("Unable to register disk access driver: %i", ret);
			goto error_cleanup;
		}
	}

	LOG_INF("MSC device (addr=%d) initialization completed", dev_data->udev->addr);

error_cleanup:
	k_mutex_unlock(&dev_data->lock);

	if (ret != 0) {
		usbh_msc_remove(c_data);
	}

	return ret;
}

/*
 * Remove the USB class driver on disconnection
 */
static int usbh_msc_remove(struct usbh_class_data *const c_data)
{
	const struct device *dev = c_data->priv;
	struct usbh_msc_data *dev_data = (void *)dev->data;

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	/* Unregister disks */
	for (size_t lun_index = 0; lun_index <= dev_data->max_logical_unit &&
				   lun_index < CONFIG_USBH_MSC_MAX_SUPPORTED_LUN;
	     lun_index++) {
		disk_access_unregister(&dev_data->lun_data[lun_index].disk);
	}

	dev_data->in_bulk_ep = NULL;
	dev_data->out_bulk_ep = NULL;
	dev_data->max_logical_unit = 0;
	dev_data->udev = NULL;

	k_mutex_unlock(&dev_data->lock);

	LOG_INF("MSC device removal completed");

	return 0;
}

/*
 * USB Host class API vtable.
 */
static struct usbh_class_api usbh_msc_api = {
	.init = usbh_msc_init,
	.probe = usbh_msc_probe,
	.removed = usbh_msc_remove,
};

/*
 * USB Host class filters.
 */
static struct usbh_class_filter usbh_msc_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_MASS_STORAGE,
		.sub = SCSI_TRANSPARENT_COMMAND_SET,
		.proto = BULK_ONLY_TRANSPORT,
	},
	{0},
};

#define USBH_DEVICE_DEFINE(index, _)                                                               \
	static struct usbh_msc_config const dev_config_##index = {.driver_index = index};          \
	static struct usbh_msc_data dev_data_##index;                                              \
                                                                                                   \
	DEVICE_DEFINE(usbh_msc_##index, "usbh_msc_" #index, NULL, NULL, &dev_data_##index,         \
		      &dev_config_##index, POST_KERNEL, 50, NULL);                                 \
                                                                                                   \
	USBH_DEFINE_CLASS(usbh_msc_data_##index, &usbh_msc_api,                                    \
			  (void *)DEVICE_GET(usbh_msc_##index), usbh_msc_filters);

LISTIFY(CONFIG_USBH_MSC_INSTANCES_COUNT, USBH_DEVICE_DEFINE, (;), _)
