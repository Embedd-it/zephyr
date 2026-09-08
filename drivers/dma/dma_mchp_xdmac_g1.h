/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file dma_mchp_xdmac_g1.h
 * @brief Microchip CA70 family Direct Memory Access (XDMAC) driver.
 */

#ifndef ZEPHYR_DRIVERS_DMA_DMA_MCHP_XDMAC_G1_H_
#define ZEPHYR_DRIVERS_DMA_DMA_MCHP_XDMAC_G1_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XDMAC_MBR_UBC_UBLEN_Pos _UINT32_(0)
#define XDMAC_MBR_UBC_UBLEN_Msk (_UINT32_(0xFFFFFF) << XDMAC_MBR_UBC_UBLEN_Pos)
#define XDMAC_MBR_UBC_UBLEN(value)                                                                 \
	(XDMAC_MBR_UBC_UBLEN_Msk & (_UINT32_(value) << XDMAC_MBR_UBC_UBLEN_Pos))

#define XDMAC_MBR_UBC_NDE_Pos _UINT32_(24)
#define XDMAC_MBR_UBC_NDE_Msk (_UINT32_(0x1) << XDMAC_MBR_UBC_NDE_Pos)

#define XDMAC_MBR_UBC_NSEN_Pos _UINT32_(25)
#define XDMAC_MBR_UBC_NSEN_Msk (_UINT32_(0x1) << XDMAC_MBR_UBC_NSEN_Pos)

#define XDMAC_MBR_UBC_NDEN_Pos _UINT32_(26)
#define XDMAC_MBR_UBC_NDEN_Msk (_UINT32_(0x1) << XDMAC_MBR_UBC_NDEN_Pos)

#define XDMAC_MBR_UBC_NVIEW_Pos _UINT32_(27)
#define XDMAC_MBR_UBC_NVIEW_Msk (_UINT32_(0x3) << XDMAC_MBR_UBC_NVIEW_Pos)
#define XDMAC_MBR_UBC_NVIEW(value)                                                                 \
	((_UINT32_(value) << XDMAC_MBR_UBC_NVIEW_Pos) & (XDMAC_MBR_UBC_NVIEW_Msk))
#define XDMAC_MBR_UBC_NVIEW_NDV0 (0)
#define XDMAC_MBR_UBC_NVIEW_NDV1 (1)
#define XDMAC_MBR_UBC_NVIEW_NDV2 (2)
#define XDMAC_MBR_UBC_NVIEW_NDV3 (3)

/** DMA channel configuration parameters */
struct dma_mchp_xdmac_hwchannel_config {
	/** Configuration Register */
	uint32_t cfg;
	/** Data Stride / Memory Set Pattern Register */
	uint32_t ds_msp;
	/** Source Microblock Stride */
	uint32_t sus;
	/** Destination Microblock Stride */
	uint32_t dus;
	/** Channel Interrupt Enable */
	uint32_t cie;
};

/** DMA transfer configuration parameters */
struct dma_mchp_xdmac_transfer_config {
	/** Microblock length */
	uint32_t ublen;
	/** Source Address */
	uint32_t sa;
	/** Destination Address */
	uint32_t da;
	/** Block length (The length of the block is (blen+1) microblocks) */
	uint32_t blen;
	/** Next descriptor address */
	uint32_t nda;
	/** Next descriptor configuration */
	uint32_t ndc;
};

/** DMA Master transfer linked list view 0 structure */
struct dma_mchp_xdmac_linked_list_desc_view0 {
	/** Next Descriptor Address */
	uint32_t mbr_nda;
	/** Microblock Control */
	uint32_t mbr_ubc;
	/** Transfer Address */
	uint32_t mbr_ta;
};

/** DMA Master transfer linked list view 1 structure */
struct dma_mchp_xdmac_linked_list_desc_view1 {
	/** Next Descriptor Address */
	uint32_t mbr_nda;
	/** Microblock Control */
	uint32_t mbr_ubc;
	/** Source Address */
	uint32_t mbr_sa;
	/** Destination Address */
	uint32_t mbr_da;
};

/** DMA Master transfer linked list view 2 structure */
struct dma_mchp_xdmac_linked_list_desc_view2 {
	/** Next Descriptor Address */
	uint32_t mbr_nda;
	/** Microblock Control */
	uint32_t mbr_ubc;
	/** Source Address */
	uint32_t mbr_sa;
	/** Destination Address */
	uint32_t mbr_da;
	/** Configuration Register */
	uint32_t mbr_cfg;
};

/** DMA Master transfer linked list view 3 structure */
struct dma_mchp_xdmac_linked_list_desc_view3 {
	/** Next Descriptor Address */
	uint32_t mbr_nda;
	/** Microblock Control */
	uint32_t mbr_ubc;
	/** Source Address */
	uint32_t mbr_sa;
	/** Destination Address */
	uint32_t mbr_da;
	/** Configuration Register */
	uint32_t mbr_cfg;
	/** Block Control */
	uint32_t mbr_bc;
	/** Data Stride */
	uint32_t mbr_ds;
	/** Source Microblock Stride */
	uint32_t mbr_sus;
	/** Destination Microblock Stride */
	uint32_t mbr_dus;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_DMA_DMA_MCHP_XDMAC_G1_H_ */
