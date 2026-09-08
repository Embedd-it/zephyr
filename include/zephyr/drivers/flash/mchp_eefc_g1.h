/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for Microchip EEFC G1 flash extended operations.
 *
 * @note This file should only be included when targeting devices
 *       with the EEFC G1 flash controller.
 */

#ifndef INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_EEFC_G1_H_
#define INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_EEFC_G1_H_

/**
 * @brief Extended operations for Microchip EEFC G1 flash controller.
 * @defgroup mchp_nvmctrl_g1_flash_ex_op Microchip EEFC G1
 * @ingroup flash_ex_op
 * @{
 */

/**
 * @brief Enumeration of extended operation codes for the MCHP EEFC G1 flash controller.
 */
typedef enum {
	/**
	 * @brief Sets a GPNVM bit.
	 *
	 * @param in  Not used.
	 * @param out Target bit index.
	 */
	FLASH_EX_OP_SET_GPNVM_BIT,
	/**
	 * @brief Clears a GPNVM bit.
	 *
	 * @param in  Not used.
	 * @param out Target bit index.
	 */
	FLASH_EX_OP_CLEAR_GPNVM_BIT,
	/**
	 * @brief Gets a GPNVM bit.
	 *
	 * @param in  Pointer to 32 bits integer where the bits will be stored.
	 * @param out Not used.
	 */
	FLASH_EX_OP_GET_GPNVM_BITS,
	/**
	 * @brief Reads the 128-bit factory-programmed unique identifier.
	 *
	 * @param in  Not used.
	 * @param out Pointer to a 16-byte buffer where the identifier will be stored.
	 */
	FLASH_EX_OP_READ_UNIQUE_ID,
	/**
	 * @brief Reads the 512-byte user signature area.
	 *
	 * @param in  Not used.
	 * @param out Pointer to a 512-byte buffer where the area will be stored.
	 */
	FLASH_EX_OP_READ_USER_SIGNATURE,
	/**
	 * @brief Writes the 512-byte user signature area.
	 *
	 * @param in  Pointer to the 512 bytes of data to write.
	 * @param out Not used.
	 */
	FLASH_EX_OP_WRITE_USER_SIGNATURE,
	/**
	 * @brief Erases the user signature area.
	 *
	 * @param in  Not used.
	 * @param out Not used.
	 */
	FLASH_EX_OP_ERASE_USER_SIGNATURE,
} flash_mchp_ex_ops_t;

#endif /* INCLUDE_ZEPHYR_DRIVERS_FLASH_MCHP_EEFC_G1_H_ */
