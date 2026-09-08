/*
 * SPDX-FileCopyrightText: Copyright (c) 2018 Aurelien Jarno
 * SPDX-FileCopyrightText: Copyright (c) 2023 Bjarki Arge Andreasen
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip Enhanced Embedded Flash Controller (EEFC) driver.
 *
 */

#define DT_DRV_COMPAT microchip_eefc_g1_flash

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/flash/mchp_flash.h>
#include <zephyr/sys/barrier.h>
#include <string.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_mchp_eefc_g1, CONFIG_FLASH_LOG_LEVEL);

#define FLASH_WRITE_PAGE_SIZE               512u
#define READ_UNIQUE_ID_TIMEOUT              100000u
#define READ_USER_SIGNATURE_TIMEOUT         100000u
#define FLASH_MCHP_EEFC_UNIQUE_ID_SIZE      16u
#define FLASH_MCHP_EEFC_USER_SIGNATURE_SIZE 512u

struct flash_mchp_eefc_config {
	/* Register block */
	efc_registers_t *regs;

	/* Interrupt configuration function */
	void (*irq_config)(const struct device *dev);

	/* Starting address of the flash memory region */
	off_t area_address;

	/* Size of the flash memory region */
	off_t area_size;

	/* Flash device configuration parameters */
	struct flash_parameters parameters;

	/* Layout of the pages */
	struct flash_pages_layout *pages_layouts;

	/* Number of layout elements */
	size_t pages_layouts_size;
};

struct flash_mchp_eefc_erase_data {
	off_t section_start;
	size_t section_end;
	bool succeeded;
};

struct flash_mchp_eefc_data {
	/* Pointer to the device structure */
	const struct device *dev;

	/* Serializes access to the flash controller across read/write/erase calls */
	struct k_mutex mutex;

	/* Erase data */
	struct flash_mchp_eefc_erase_data erase_data;

	/* Synchronization semaphore */
	struct k_sem done;
};

/*
 * Ensure the offset is correct and doesn't overflow
 */
static bool flash_mchp_eefc_validate_offset_len(off_t offset, size_t len)
{
	if (offset < 0) {
		return false;
	} else if ((offset + len) < len) {
		return false;
	} else {
		return true;
	}
}

/*
 * Check the alignment of an address
 */
static bool flash_mchp_eefc_aligned(size_t value, size_t alignment)
{
	return (value & (alignment - 1)) == 0u;
}

/*
 * Check whether an offset is on write page boundary
 */
static bool flash_mchp_eefc_offset_is_on_write_page_boundary(off_t offset)
{
	return flash_mchp_eefc_aligned(offset, FLASH_WRITE_PAGE_SIZE);
}

/*
 * Mask the ready interrupt flag
 */
static inline void
flash_mchp_eefc_mask_ready_interrupt(const struct flash_mchp_eefc_config *dev_config)
{
	efc_registers_t *const regs = dev_config->regs;

	regs->EEFC_FMR &= ~EEFC_FMR_FRDY_Msk;
}

/*
 * Unmask the ready interrupt flag
 */
static inline void
flash_mchp_eefc_unmask_ready_interrupt(const struct flash_mchp_eefc_config *dev_config)
{
	efc_registers_t *const regs = dev_config->regs;

	regs->EEFC_FMR |= EEFC_FMR_FRDY_Msk;
}

/*
 * Interrupt handler, just signals the synchronization semaphore
 */
static void flash_mchp_eefc_isr(const struct device *dev)
{
	struct flash_mchp_eefc_data *dev_data = dev->data;
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	flash_mchp_eefc_mask_ready_interrupt(dev_config);
	k_sem_give(&dev_data->done);
}

/*
 * Wait until the peripheral has concluded the ongoing operation
 */
static int flash_mchp_eefc_section_wait_until_ready(const struct device *dev)
{
	struct flash_mchp_eefc_data *dev_data = dev->data;
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	efc_registers_t *const regs = dev_config->regs;
	uint32_t eefc_fsr = 0u;

	k_sem_reset(&dev_data->done);
	flash_mchp_eefc_unmask_ready_interrupt(dev_config);

	if (k_sem_take(&dev_data->done, K_MSEC(500u)) < 0) {
		LOG_ERR("Command did not execute in time");
		return -EFAULT;
	}

	/* FSR register is cleared on read */
	eefc_fsr = regs->EEFC_FSR;

	if ((eefc_fsr & EEFC_FSR_FCMDE_Msk) != 0u) {
		LOG_ERR("Invalid command requested");
		return -EPERM;
	}

	if ((eefc_fsr & EEFC_FSR_FLOCKE_Msk) != 0u) {
		LOG_ERR("Tried to modify locked region");
		return -EPERM;
	}

	if ((eefc_fsr & EEFC_FSR_FLERR_Msk) != 0u) {
		LOG_ERR("Programming failed");
		return -EPERM;
	}

	return 0;
}

/*
 * Check if the provided address is included in the configured flash area
 */
static bool flash_mchp_eefc_section_is_within_area(const struct device *dev, off_t offset,
						   size_t len)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	if ((offset + ((off_t)len)) < offset) {
		return false;
	}

	if ((offset >= 0) && ((offset + len) <= dev_config->area_size)) {
		return true;
	}

	LOG_WRN("Section from 0x%x to 0x%x is not within flash area (0x0 to %x)", (size_t)offset,
		(size_t)(offset + len), (size_t)dev_config->area_size);

	return false;
}

/*
 * Check if the provided address is aligned with the write block size
 */
static bool flash_mchp_eefc_section_is_aligned_with_write_block_size(const struct device *dev,
								     off_t offset, size_t len)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	if (flash_mchp_eefc_aligned(offset, dev_config->parameters.write_block_size) &&
	    flash_mchp_eefc_aligned(len, dev_config->parameters.write_block_size)) {
		return true;
	}

	LOG_WRN("Section from 0x%x to 0x%x is not aligned with write block size (%u)",
		(size_t)offset, (size_t)(offset + len), dev_config->parameters.write_block_size);

	return false;
}

/*
 * Check if the provided address is aligned with the page size
 */
static bool flash_mchp_eefc_section_is_aligned_with_pages(const struct device *dev, off_t offset,
							  size_t len)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	struct flash_pages_info pages_info;

	/* Get the page offset points to */
	if (flash_get_page_info_by_offs(dev, offset, &pages_info) < 0) {
		return false;
	}

	/* Validate offset points to start of page */
	if (offset != pages_info.start_offset) {
		return false;
	}

	/* Check if end of section is aligned with end of area */
	if ((offset + len) == (dev_config->area_size)) {
		return true;
	}

	/* Get the page pointed to by end of section */
	if (flash_get_page_info_by_offs(dev, offset + len, &pages_info) < 0) {
		return false;
	}

	/* Validate offset points to start of page */
	if ((offset + len) != pages_info.start_offset) {
		return false;
	}

	return true;
}

static int flash_mchp_eefc_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	struct flash_mchp_eefc_data *dev_data = dev->data;
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	if (len == 0u) {
		return 0;
	}

	if (!flash_mchp_eefc_validate_offset_len(offset, len)) {
		return -EINVAL;
	}

	if (!flash_mchp_eefc_section_is_within_area(dev, offset, len)) {
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	/* Reading can be done simply by pointer access */
	memcpy(data, (uint8_t *)(dev_config->area_address + offset), len);

	k_mutex_unlock(&dev_data->mutex);

	return 0;
}

/*
 * Commit the latch buffer to actual memory
 */
static int flash_mchp_eefc_write_latch_buffer_to_page(const struct device *dev, off_t offset)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	efc_registers_t *const regs = dev_config->regs;
	uint32_t page = offset / FLASH_WRITE_PAGE_SIZE;

	regs->EEFC_FCR = EEFC_FCR_FCMD_WP | EEFC_FCR_FARG(page) | EEFC_FCR_FKEY_PASSWD;

	return flash_mchp_eefc_section_wait_until_ready(dev);
}

/*
 * Commit the latch buffer to the previous page
 */
static int flash_mchp_eefc_write_latch_buffer_to_previous_page(const struct device *dev,
							       off_t offset)
{
	return flash_mchp_eefc_write_latch_buffer_to_page(dev, offset - FLASH_WRITE_PAGE_SIZE);
}

/*
 * Write a uint32_t to the latch buffer
 */
static void flash_mchp_eefc_write_dword_to_latch_buffer(const struct device *dev, off_t offset,
							uint32_t dword)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	/* offset is relative to the start of the flash area; the latch buffer is
	 * written through the flash's own memory-mapped address space.
	 */
	*((uint32_t *)(dev_config->area_address + offset)) = dword;
	barrier_dsync_fence_full();
}

/*
 * The controller can only commit in pages, meaning incomplete or unaligned writes may write
 * spurious contents.
 * This function reads the contents in memory and re-writes them to the latch buffer to preserve
 * them on commit.
 */
static void flash_mchp_eefc_preserve_range(const struct device *dev, off_t start, off_t end)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	uint32_t dword = 0u;

	for (off_t offset = start; offset < end; offset += sizeof(uint32_t)) {
		memcpy(&dword, (uint8_t *)(dev_config->area_address + offset), sizeof(dword));
		flash_mchp_eefc_write_dword_to_latch_buffer(dev, offset, dword);
	}
}

/*
 * Write uint32_t words to flash
 */
static int flash_mchp_eefc_write_dwords_to_flash(const struct device *dev, off_t offset,
						 const uint32_t *dwords, size_t size)
{
	int ret = 0;
	off_t page_start = offset - (offset % FLASH_WRITE_PAGE_SIZE);

	if (!flash_mchp_eefc_offset_is_on_write_page_boundary(offset)) {
		flash_mchp_eefc_preserve_range(dev, page_start, offset);
	}

	for (size_t i = 0u; i < size; i++) {
		/* Write data to the latch buffer */
		flash_mchp_eefc_write_dword_to_latch_buffer(dev, offset, dwords[i]);
		offset += sizeof(uint32_t);
		if (flash_mchp_eefc_offset_is_on_write_page_boundary(offset)) {
			/* If we are passing a page boundary commit the previous one */
			ret = flash_mchp_eefc_write_latch_buffer_to_previous_page(dev, offset);
			if (ret != 0) {
				return ret;
			}
		}
	}

	/* If on page boundary the write was already committed */
	if (!flash_mchp_eefc_offset_is_on_write_page_boundary(offset)) {
		off_t page_end = offset - (offset % FLASH_WRITE_PAGE_SIZE) + FLASH_WRITE_PAGE_SIZE;
		/* Restore the contents that were not overwritten */
		flash_mchp_eefc_preserve_range(dev, offset, page_end);
		/* Commit the write */
		ret = flash_mchp_eefc_write_latch_buffer_to_page(dev, offset);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static int flash_mchp_eefc_write(const struct device *dev, off_t offset, const void *data,
				 size_t len)
{
	struct flash_mchp_eefc_data *dev_data = dev->data;
	int ret = 0;

	if (len == 0u) {
		return 0;
	}

	if (!flash_mchp_eefc_validate_offset_len(offset, len)) {
		return -EINVAL;
	}

	if (!flash_mchp_eefc_section_is_aligned_with_write_block_size(dev, offset, len)) {
		return -EINVAL;
	}

	if (!flash_mchp_eefc_section_is_within_area(dev, offset, len)) {
		return -EINVAL;
	}

	LOG_DBG("Writing sector from 0x%x to 0x%x", (size_t)offset, (size_t)(offset + len));

	k_mutex_lock(&dev_data->mutex, K_FOREVER);
	ret = flash_mchp_eefc_write_dwords_to_flash(dev, offset, data, len / sizeof(uint32_t));
	if (ret != 0) {
		k_mutex_unlock(&dev_data->mutex);
		return ret;
	}

	k_mutex_unlock(&dev_data->mutex);
	return 0;
}

/*
 * Unlock a write page
 */
static int flash_mchp_eefc_unlock_write_page(const struct device *dev, uint16_t page_index)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	efc_registers_t *const regs = dev_config->regs;

	/* Perform unlock command of write page */
	regs->EEFC_FCR = EEFC_FCR_FCMD_CLB | EEFC_FCR_FARG(page_index) | EEFC_FCR_FKEY_PASSWD;

	return flash_mchp_eefc_section_wait_until_ready(dev);
}

/*
 * Unlock a page
 */
static int flash_mchp_eefc_unlock_page(const struct device *dev,
				       const struct flash_pages_info *info)
{
	uint16_t page_index_start;
	uint16_t page_index_end;
	int ret = 0;

	/* Convert from page offset and size to write page index and count */
	page_index_start = info->start_offset / FLASH_WRITE_PAGE_SIZE;
	page_index_end = page_index_start + (info->size / FLASH_WRITE_PAGE_SIZE);

	for (uint16_t i = page_index_start; i < page_index_end; i++) {
		ret = flash_mchp_eefc_unlock_write_page(dev, i);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

/*
 * Erase a page
 */
static int flash_mchp_eefc_erase_page(const struct device *dev, const struct flash_pages_info *info)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	efc_registers_t *const regs = dev_config->regs;
	uint32_t page_index = 0u;
	int ret = 0;

	/* Convert from page offset to write page index */
	page_index = info->start_offset / FLASH_WRITE_PAGE_SIZE;

	LOG_DBG("Erasing page at 0x%x of size 0x%x", (size_t)info->start_offset, info->size);

	/* Perform erase command of page */
	switch (info->size) {
	case 0x800u:
		regs->EEFC_FCR =
			EEFC_FCR_FCMD_EPA | EEFC_FCR_FARG(page_index) | EEFC_FCR_FKEY_PASSWD;
		break;
	case 0x1000u:
		regs->EEFC_FCR =
			EEFC_FCR_FCMD_EPA | EEFC_FCR_FARG(page_index | 1u) | EEFC_FCR_FKEY_PASSWD;
		break;
	case 0x2000u:
		regs->EEFC_FCR =
			EEFC_FCR_FCMD_EPA | EEFC_FCR_FARG(page_index | 2u) | EEFC_FCR_FKEY_PASSWD;
		break;
	case 0x4000u:
		regs->EEFC_FCR =
			EEFC_FCR_FCMD_EPA | EEFC_FCR_FARG(page_index | 3u) | EEFC_FCR_FKEY_PASSWD;
		break;
	default:
		return -EINVAL;
	}

	ret = flash_mchp_eefc_section_wait_until_ready(dev);
	if (ret == 0) {
		return ret;
	}

	LOG_ERR("Failed to erase page at 0x%x of size 0x%x", (size_t)info->start_offset,
		info->size);

	return ret;
}

/*
 * Erase a page, to be used with `flash_page_foreach`
 */
static bool flash_mchp_eefc_erase_foreach_page(const struct flash_pages_info *info, void *data)
{
	struct flash_mchp_eefc_data *dev_data = data;
	const struct device *dev = dev_data->dev;
	struct flash_mchp_eefc_erase_data *erase_data = &dev_data->erase_data;

	/* Validate we reached first page to erase */
	if (info->start_offset < erase_data->section_start) {
		/* Next page */
		return true;
	}

	/* Check if we've reached the end of pages to erase */
	if (info->start_offset >= erase_data->section_end) {
		/* Succeeded, stop iterating */
		return false;
	}

	if (flash_mchp_eefc_unlock_page(dev, info) != 0 ||
	    flash_mchp_eefc_erase_page(dev, info) != 0) {
		/* Failed to unlock page and erase page, stop iterating */
		erase_data->succeeded = false;
		return false;
	}

	/* Next page */
	return true;
}

static int flash_mchp_eefc_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_mchp_eefc_data *dev_data = dev->data;

	if (size == 0) {
		return 0;
	}

	if (!flash_mchp_eefc_validate_offset_len(offset, size)) {
		return -EINVAL;
	}

	if (!flash_mchp_eefc_section_is_aligned_with_pages(dev, offset, size)) {
		return -EINVAL;
	}

	LOG_DBG("Erasing sector from 0x%x to 0x%x", (size_t)offset, (size_t)(offset + size));

	k_mutex_lock(&dev_data->mutex, K_FOREVER);
	dev_data->erase_data.section_start = offset;
	dev_data->erase_data.section_end = offset + size;
	dev_data->erase_data.succeeded = true;
	flash_page_foreach(dev, flash_mchp_eefc_erase_foreach_page, dev_data);
	if (!dev_data->erase_data.succeeded) {
		k_mutex_unlock(&dev_data->mutex);
		return -EFAULT;
	}

	k_mutex_unlock(&dev_data->mutex);
	return 0;
}

static const struct flash_parameters *flash_mchp_eefc_get_parameters(const struct device *dev)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	return &dev_config->parameters;
}

static int flash_mchp_eefc_get_size(const struct device *dev, uint64_t *size)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	*size = (uint64_t)dev_config->area_size;

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_mchp_eefc_page_layout(const struct device *dev,
					const struct flash_pages_layout **layout,
					size_t *layout_size)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	*layout = dev_config->pages_layouts;
	*layout_size = dev_config->pages_layouts_size;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

#if defined(CONFIG_FLASH_EX_OP_ENABLED)

/*
 * Reads the 128-bit unique identifier area.
 * While the "Start Read Unique Identifier" command is active the unique id can be read from the
 * low addresses of the flash memory. In this time other flash memory locations cannot be accessed,
 * so the function must execute from ram and with interrupts disabled.
 */
static __ramfunc int flash_mchp_eefc_read_unique_id_ramfunc(efc_registers_t *const regs,
							    off_t area_address, uint8_t *out)
{
	const uint8_t *unique_id = (const uint8_t *)area_address;
	unsigned int key = irq_lock();
	uint32_t timeout = 0u;
	int ret = 0;

	/* Start Read Unique Identifier */
	regs->EEFC_FCR = EEFC_FCR_FCMD_STUI | EEFC_FCR_FKEY_PASSWD;

	/*
	 * Unlike every other flash command, FRDY *falls* (and generates no
	 * interrupt) once STUI has taken effect - the unique identifier area
	 * is only correctly mapped in once it does.
	 * Also, since flash is now disabled we must resort to a very primitive busy loop.
	 */
	timeout = READ_UNIQUE_ID_TIMEOUT;
	while ((regs->EEFC_FSR & EEFC_FSR_FRDY_Msk) != 0u) {
		timeout--;
		if (timeout == 0u) {
			ret = -ETIMEDOUT;
			goto error_cleanup;
		}
	}

	for (size_t i = 0u; i < FLASH_MCHP_EEFC_UNIQUE_ID_SIZE; i++) {
		out[i] = sys_read8((mem_addr_t)&unique_id[i]);
	}

error_cleanup:
	/* Stop Read Unique Identifier */
	regs->EEFC_FCR = EEFC_FCR_FCMD_SPUI | EEFC_FCR_FKEY_PASSWD;

	/* SPUI behaves like every other command: FRDY rises on completion */
	timeout = READ_UNIQUE_ID_TIMEOUT;
	while ((regs->EEFC_FSR & EEFC_FSR_FRDY_Msk) == 0u) {
		timeout--;
		if (timeout == 0u) {
			ret = -ETIMEDOUT;
			break;
		}
	}

	irq_unlock(key);
	return ret;
}

/*
 * Reads the 512-byte user signature area.
 * While the "Start Read User Signature" command is active the area can be read from the low
 * addresses of the flash memory, and other flash memory locations cannot be accessed, so the
 * function must execute from ram and with interrupts disabled.
 */
static __ramfunc int flash_mchp_eefc_read_user_signature_ramfunc(efc_registers_t *const regs,
								 off_t area_address, uint8_t *out)
{
	const uint8_t *user_signature = (const uint8_t *)area_address;
	unsigned int key = irq_lock();
	uint32_t timeout = READ_USER_SIGNATURE_TIMEOUT;
	int ret = 0;

	/* Start Read User Signature */
	regs->EEFC_FCR = EEFC_FCR_FCMD_STUS | EEFC_FCR_FKEY_PASSWD;

	/*
	 * Same FRDY-falls, no-interrupt exception as STUI - resort to a
	 * primitive busy loop since flash (and therefore normal wait
	 * helpers) is unavailable while this command is active.
	 */
	while ((regs->EEFC_FSR & EEFC_FSR_FRDY_Msk) != 0u) {
		timeout--;
		if (timeout == 0) {
			ret = -ETIMEDOUT;
			goto error_cleanup;
		}
	}

	for (size_t i = 0u; i < FLASH_MCHP_EEFC_USER_SIGNATURE_SIZE; i++) {
		out[i] = sys_read8((mem_addr_t)&user_signature[i]);
	}

	/* Stop Read User Signature */
	regs->EEFC_FCR = EEFC_FCR_FCMD_SPUS | EEFC_FCR_FKEY_PASSWD;

	timeout = READ_USER_SIGNATURE_TIMEOUT;
	/* SPUS behaves like every other command: FRDY rises on completion */
	while ((regs->EEFC_FSR & EEFC_FSR_FRDY_Msk) == 0u) {
		timeout--;
		if (timeout == 0) {
			ret = -ETIMEDOUT;
			goto error_cleanup;
		}
	}

error_cleanup:
	irq_unlock(key);
	return ret;
}

/*
 * Writes the 512-byte user signature area.
 */
static int flash_mchp_eefc_write_user_signature(const struct device *dev, const uint8_t *data)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	efc_registers_t *const regs = dev_config->regs;
	const uint32_t *dwords = (const uint32_t *)data;
	off_t offset = 0;

	for (size_t i = 0u; i < FLASH_MCHP_EEFC_USER_SIGNATURE_SIZE / sizeof(uint32_t); i++) {
		flash_mchp_eefc_write_dword_to_latch_buffer(dev, offset, dwords[i]);
		offset += sizeof(uint32_t);
	}

	regs->EEFC_FCR = EEFC_FCR_FCMD_WUS | EEFC_FCR_FKEY_PASSWD;

	return flash_mchp_eefc_section_wait_until_ready(dev);
}

/*
 * Erases the user signature area (section 21.4.3.9). FARG is meaningless for EUS.
 */
static int flash_mchp_eefc_erase_user_signature(const struct device *dev)
{
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	efc_registers_t *const regs = dev_config->regs;

	regs->EEFC_FCR = EEFC_FCR_FCMD_EUS | EEFC_FCR_FKEY_PASSWD;

	return flash_mchp_eefc_section_wait_until_ready(dev);
}

static int flash_mchp_eefc_ex_op(const struct device *dev, uint16_t code, const uintptr_t in,
				 void *out)
{
	struct flash_mchp_eefc_data *dev_data = dev->data;
	const struct flash_mchp_eefc_config *dev_config = dev->config;
	efc_registers_t *const regs = dev_config->regs;
	const uint8_t *data = NULL;
	uint8_t bit = 0u;
	uint32_t *out_ptr = NULL;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	switch (code) {
	case FLASH_EX_OP_READ_UNIQUE_ID:
		ret = flash_mchp_eefc_read_unique_id_ramfunc(regs, dev_config->area_address, out);
		break;
	case FLASH_EX_OP_READ_USER_SIGNATURE:
		ret = flash_mchp_eefc_read_user_signature_ramfunc(regs, dev_config->area_address,
								  out);
		break;
	case FLASH_EX_OP_WRITE_USER_SIGNATURE:
		data = (const uint8_t *)in;

		ret = flash_mchp_eefc_write_user_signature(dev, data);
		break;
	case FLASH_EX_OP_ERASE_USER_SIGNATURE:
		ret = flash_mchp_eefc_erase_user_signature(dev);
		break;
	case FLASH_EX_OP_SET_GPNVM_BIT:
		bit = (uint8_t)in;

		regs->EEFC_FCR = EEFC_FCR_FCMD_SGPB | EEFC_FCR_FARG(bit) | EEFC_FCR_FKEY_PASSWD;
		ret = flash_mchp_eefc_section_wait_until_ready(dev);
		break;
	case FLASH_EX_OP_CLEAR_GPNVM_BIT:
		bit = (uint8_t)in;

		regs->EEFC_FCR = EEFC_FCR_FCMD_CGPB | EEFC_FCR_FARG(bit) | EEFC_FCR_FKEY_PASSWD;
		ret = flash_mchp_eefc_section_wait_until_ready(dev);
		break;
	case FLASH_EX_OP_GET_GPNVM_BITS:
		out_ptr = out;

		regs->EEFC_FCR = EEFC_FCR_FCMD_GGPB | EEFC_FCR_FKEY_PASSWD;
		ret = flash_mchp_eefc_section_wait_until_ready(dev);
		if (ret == 0) {
			*out_ptr = regs->EEFC_FRR;
		}

		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}
#endif /* CONFIG_FLASH_EX_OP_ENABLED */

static DEVICE_API(flash, flash_mchp_eefc_driver_api) = {
	.read = flash_mchp_eefc_read,
	.write = flash_mchp_eefc_write,
	.erase = flash_mchp_eefc_erase,
	.get_parameters = flash_mchp_eefc_get_parameters,
	.get_size = flash_mchp_eefc_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_mchp_eefc_page_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#if defined(CONFIG_FLASH_EX_OP_ENABLED)
	.ex_op = flash_mchp_eefc_ex_op,
#endif /* CONFIG_FLASH_EX_OP_ENABLED */
};

static int flash_mchp_eefc_init(const struct device *dev)
{
	struct flash_mchp_eefc_data *dev_data = dev->data;
	const struct flash_mchp_eefc_config *dev_config = dev->config;

	dev_data->dev = dev;
	k_mutex_init(&dev_data->mutex);
	k_sem_init(&dev_data->done, 0u, 1u);
	flash_mchp_eefc_mask_ready_interrupt(dev_config);

	dev_config->irq_config(dev);

	return 0;
}

#define FLASH_DEVICE DT_NODELABEL(flash0)

#define FLASH_PAGES_LAYOUT(node_id, prop, idx)                                                     \
	{                                                                                          \
		.pages_count = DT_PHA_BY_IDX(node_id, prop, idx, pages_count),                     \
		.pages_size = DT_PHA_BY_IDX(node_id, prop, idx, pages_size),                       \
	}

#define FLASH_PAGES_LAYOUTS                                                                        \
	DT_FOREACH_PROP_ELEM_SEP(FLASH_DEVICE, erase_blocks, FLASH_PAGES_LAYOUT, (,))

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	static struct flash_pages_layout flash_pages_layouts_##n[] = {FLASH_PAGES_LAYOUTS};        \
                                                                                                   \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), flash_mchp_eefc_isr,        \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct flash_mchp_eefc_config mchp_flash_##n##_config = {                     \
		.regs = (efc_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.irq_config = irq_config_##n,                                                      \
		.area_address = DT_REG_ADDR(FLASH_DEVICE),                                         \
		.area_size = DT_REG_SIZE(FLASH_DEVICE),                                            \
		.parameters =                                                                      \
			{                                                                          \
				.write_block_size = DT_PROP(FLASH_DEVICE, write_block_size),       \
				.erase_value = 0xFF,                                               \
			},                                                                         \
		.pages_layouts = flash_pages_layouts_##n,                                          \
		.pages_layouts_size = ARRAY_SIZE(flash_pages_layouts_##n),                         \
	};                                                                                         \
                                                                                                   \
	static struct flash_mchp_eefc_data mchp_flash_##n##_data;                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, flash_mchp_eefc_init, NULL, &mchp_flash_##n##_data,               \
			      &mchp_flash_##n##_config, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,   \
			      &flash_mchp_eefc_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
