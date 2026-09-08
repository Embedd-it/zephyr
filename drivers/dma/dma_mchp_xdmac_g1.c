/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file dma_mchp_xdmac_g1.c
 * @brief Microchip XDMAC (DMA) controller driver.
 */

#define DT_DRV_COMPAT microchip_xdmac_g1_dma

#include "dma_mchp_xdmac_g1.h"
#include <errno.h>
#include <string.h>
#include <soc.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/irq.h>
#include <zephyr/sys/barrier.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dma_mchp_xdmac_g1, CONFIG_DMA_LOG_LEVEL);

#define XDMAC_INT_ERR (XDMAC_CIE_RBIE_Msk | XDMAC_CIE_WBIE_Msk | XDMAC_CIE_ROIE_Msk)

enum dma_state {
	DMA_STATE_INIT = 0,
	DMA_STATE_CONFIGURED,
	DMA_STATE_RUNNING,
	DMA_STATE_SUSPENDED,
};

/* DMA channel configuration */
struct dma_mchp_xdmac_channel_config {
	void *user_data;
	dma_callback_t callback;
	uint32_t data_size;
	enum dma_state state;
	uint32_t error_callback_dis: 1;
	uint32_t cyclic: 1;
	uint32_t complete_callback_en: 1;
};

/* Device constant configuration parameters */
struct dma_mchp_xdmac_config {
	xdmac_registers_t *regs;
	void (*irq_config)(const struct device *dev);
	const struct device *clock;
	clock_control_subsys_t clock_cfg;
	uint8_t irq_id;
};

/* Device run time data */
typedef struct dma_mchp_xdmac_linked_list_desc_view3
	dma_mchp_xdmac_descs[CONFIG_DMA_MCHP_XDMAC_G1_DESC_COUNT];

enum dma_mchp_xdmac_config_mode {
	DMA_MCHP_XDMAC_G1_CONFIG_SINGLE,
	DMA_MCHP_XDMAC_G1_CONFIG_BLOCKS,
};

struct dma_mchp_xdmac_data {
	struct dma_context dma_ctx;
	enum dma_mchp_xdmac_config_mode config_mode;
	struct dma_mchp_xdmac_channel_config *dma_channels;
	dma_mchp_xdmac_descs *descs;
};

static void dma_mchp_xdmac_isr(const struct device *dev)
{
	const struct dma_mchp_xdmac_config *const dev_config = dev->config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;
	const uint32_t error_msk = XDMAC_CIS_ROIS_Msk | XDMAC_CIS_WBEIS_Msk | XDMAC_CIS_RBEIS_Msk;
	xdmac_registers_t *const regs = dev_config->regs;
	struct dma_mchp_xdmac_channel_config *xdmac_channel_config;
	uint32_t isr_status, channel_status;
	uint32_t err;

	/* Get global interrupt status */
	isr_status = regs->XDMAC_GIS;

	for (size_t channel = 0; channel < channel_num; channel++) {
		if (!(isr_status & (1 << channel))) {
			continue;
		}

		err = 0;
		xdmac_channel_config = &dev_data->dma_channels[channel];

		channel_status = regs->XDMAC_CHID[channel].XDMAC_CIS;
		if (!xdmac_channel_config->error_callback_dis &&
		    ((channel_status & error_msk) != 0)) {
			/* Force highest 0b1 to convert channel status bit as negative number */
			err = channel_status | 0x80000000u;
		} else if ((xdmac_channel_config->cyclic == 0) &&
			   (channel_status & XDMAC_CIS_LIS_Msk)) {
			err = DMA_STATUS_COMPLETE;
			xdmac_channel_config->state = DMA_STATE_CONFIGURED;
		} else if (channel_status & XDMAC_CIS_BIS_Msk) {
			if (xdmac_channel_config->cyclic == 0) {
				err = DMA_STATUS_BLOCK;
			} else {
				/* HW does not raise any interrupt for new cycle, folowing detection
				 * is unreliable
				 */
				if (regs->XDMAC_CHID[channel].XDMAC_CNDA ==
				    ((uint32_t)&dev_data->descs[channel][0])) {
					err = DMA_STATUS_COMPLETE;
				} else if (xdmac_channel_config->complete_callback_en != 0) {
					err = DMA_STATUS_BLOCK;
				} else {
					/* callback on block is not requested */
					continue;
				}
			}
		} else {
			/* Unknown source of interrupt, should not happen */
			err = -1;
		}

		/* Execute callback */
		if (xdmac_channel_config->callback) {
			xdmac_channel_config->callback(dev, xdmac_channel_config->user_data,
						       channel, err);
		}
	}
}

int dma_mchp_xdmac_channel_configure(const struct device *dev, uint32_t channel,
				     struct dma_mchp_xdmac_hwchannel_config *param)
{
	const struct dma_mchp_xdmac_config *const dev_config = dev->config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;

	xdmac_registers_t *const regs = dev_config->regs;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	/* Check if the channel is enabled */
	if (regs->XDMAC_GS & (XDMAC_GS_ST0_Msk << channel)) {
		return -EBUSY;
	}

	/* Disable all channel interrupts */
	regs->XDMAC_CHID[channel].XDMAC_CID = 0xFF;
	/* Clear pending Interrupt Status bit(s) */
	(void)regs->XDMAC_CHID[channel].XDMAC_CIS;

	/* NOTE:
	 * Setting channel configuration is not required for linked list view 2
	 * to 3 modes. It is done anyway to keep the code simple. It has no
	 * negative impact on the DMA functionality.
	 */

	/* Set channel configuration */
	regs->XDMAC_CHID[channel].XDMAC_CC = param->cfg;

	/* Set data stride memory pattern */
	regs->XDMAC_CHID[channel].XDMAC_CDS_MSP = param->ds_msp;
	/* Set source microblock stride */
	regs->XDMAC_CHID[channel].XDMAC_CSUS = param->sus;
	/* Set destination microblock stride */
	regs->XDMAC_CHID[channel].XDMAC_CDUS = param->dus;

	/* Enable selected channel interrupts */
	regs->XDMAC_CHID[channel].XDMAC_CIE = param->cie;

	return 0;
}

int dma_mchp_xdmac_transfer_configure(const struct device *dev, uint32_t channel,
				      struct dma_mchp_xdmac_transfer_config *param)
{
	const struct dma_mchp_xdmac_config *const dev_config = dev->config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;

	xdmac_registers_t *const regs = dev_config->regs;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	/* Check if the channel is enabled */
	if (regs->XDMAC_GS & (XDMAC_GS_ST0_Msk << channel)) {
		return -EBUSY;
	}

	/* NOTE:
	 * Setting source, destination address is not required for linked list
	 * view 1 to 3 modes. It is done anyway to keep the code simple. It has
	 * no negative impact on the DMA functionality.
	 */

	/* Set source address */
	regs->XDMAC_CHID[channel].XDMAC_CSA = param->sa;
	/* Set destination address */
	regs->XDMAC_CHID[channel].XDMAC_CDA = param->da;

	if ((param->ndc & XDMAC_CNDC_NDE_Msk) == XDMAC_CNDC_NDE_DSCR_FETCH_DIS) {
		/*
		 * Linked List is disabled, configure additional transfer
		 * parameters.
		 */

		/* Set length of data in the microblock */
		regs->XDMAC_CHID[channel].XDMAC_CUBC = param->ublen;
		/* Set block length: block length is (blen+1) microblocks */
		regs->XDMAC_CHID[channel].XDMAC_CBC = param->blen;
	} else {
		/*
		 * Linked List is enabled, configure additional transfer
		 * parameters.
		 */

		/* Set next descriptor address */
		regs->XDMAC_CHID[channel].XDMAC_CNDA = param->nda;
	}

	/* Set next descriptor configuration */
	regs->XDMAC_CHID[channel].XDMAC_CNDC = param->ndc;

	dev_data->dma_channels[channel].state = DMA_STATE_CONFIGURED;

	return 0;
}

static int dma_mchp_xdmac_config_single_transfer(const struct device *dev, uint32_t channel,
						 struct dma_config *dma_cfg)
{
	struct dma_mchp_xdmac_channel_config *xdmac_channel_config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	struct dma_mchp_xdmac_hwchannel_config channel_config;
	struct dma_mchp_xdmac_transfer_config transfer_config;
	uint32_t xdmac_inc_config = 0;
	uint32_t burst_size;
	uint32_t data_size;
	int ret;

	burst_size = find_msb_set(dma_cfg->source_burst_length) - 1;
	LOG_DBG("burst_size=%d", burst_size);
	data_size = find_msb_set(dma_cfg->source_data_size) - 1;
	LOG_DBG("data_size=%d", data_size);

	/* Requested block size exceeds maximum number of transfers */
	if ((dma_cfg->head_block->block_size >> data_size) >
	    (XDMAC_CUBC_UBLEN_Msk >> XDMAC_CUBC_UBLEN_Pos)) {
		return -EINVAL;
	}

	if (dma_cfg->head_block->source_addr_adj == DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL) {
		xdmac_inc_config |= XDMAC_CC_SAM_INCREMENTED_AM;
	} else {
		xdmac_inc_config |= XDMAC_CC_SAM_FIXED_AM;
	}

	if (dma_cfg->head_block->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT &&
	    dma_cfg->channel_direction == PERIPHERAL_TO_MEMORY) {
		xdmac_inc_config |= XDMAC_CC_DAM_INCREMENTED_AM;
	} else {
		xdmac_inc_config |= XDMAC_CC_DAM_FIXED_AM;
	}

	switch (dma_cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		channel_config.cfg = XDMAC_CC_TYPE_MEM_TRAN |
				     XDMAC_CC_MBSIZE(burst_size == 0u ? 0 : burst_size - 1) |
				     XDMAC_CC_SAM_INCREMENTED_AM | XDMAC_CC_DAM_INCREMENTED_AM;
		break;
	case MEMORY_TO_PERIPHERAL:
		channel_config.cfg = XDMAC_CC_TYPE_PER_TRAN | XDMAC_CC_CSIZE(burst_size) |
				     XDMAC_CC_DSYNC_MEM2PER | xdmac_inc_config;
		break;
	case PERIPHERAL_TO_MEMORY:
		channel_config.cfg = XDMAC_CC_TYPE_PER_TRAN | XDMAC_CC_CSIZE(burst_size) |
				     XDMAC_CC_DSYNC_PER2MEM | xdmac_inc_config;
		break;
	default:
		LOG_ERR("'channel_direction' value %d is not supported",
			dma_cfg->channel_direction);
		return -EINVAL;
	}

	channel_config.cfg |= XDMAC_CC_DWIDTH(data_size);
#if defined(XDMAC_CC_SIF_AHB_IF1)
	channel_config.cfg |= XDMAC_CC_SIF_AHB_IF1;
#endif
#if defined(XDMAC_CC_DIF_AHB_IF1)
	channel_config.cfg |= XDMAC_CC_DIF_AHB_IF1;
#endif
	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY) {
		/* Errata 2.2.2: memory-to-memory require an unused peripheral ID. */
		channel_config.cfg |= XDMAC_CC_PERID_Msk;
	} else {
		channel_config.cfg |= XDMAC_CC_PERID(dma_cfg->dma_slot);
	}
	channel_config.ds_msp = 0u;
	channel_config.sus = 0u;
	channel_config.dus = 0u;
	channel_config.cie = XDMAC_CIE_LIE_Msk;
	channel_config.cie |= (dma_cfg->complete_callback_en ? XDMAC_CIE_BIE_Msk : 0);
	channel_config.cie |= (dma_cfg->error_callback_dis ? 0 : XDMAC_INT_ERR);
	ret = dma_mchp_xdmac_channel_configure(dev, channel, &channel_config);
	if (ret < 0) {
		return ret;
	}

	(void)memset(&transfer_config, 0, sizeof(transfer_config));
	transfer_config.sa = dma_cfg->head_block->source_address;
	transfer_config.da = dma_cfg->head_block->dest_address;
	transfer_config.ublen = dma_cfg->head_block->block_size >> data_size;
	ret = dma_mchp_xdmac_transfer_configure(dev, channel, &transfer_config);
	if (ret < 0) {
		return ret;
	}

	xdmac_channel_config = &dev_data->dma_channels[channel];
	xdmac_channel_config->callback = dma_cfg->dma_callback;
	xdmac_channel_config->user_data = dma_cfg->user_data;
	xdmac_channel_config->cyclic = dma_cfg->cyclic;
	xdmac_channel_config->error_callback_dis = dma_cfg->error_callback_dis;
	xdmac_channel_config->complete_callback_en = dma_cfg->complete_callback_en;
	xdmac_channel_config->data_size = data_size;

	dev_data->config_mode = DMA_MCHP_XDMAC_G1_CONFIG_SINGLE;

	return ret;
}

static __maybe_unused bool dma_mchp_xdmac_fit_stride_range(int32_t strides)
{
	const uint32_t mask = (XDMAC_CSUS_SUBS_Msk >> XDMAC_CSUS_SUBS_Pos) + 1;

	return (strides <= (mask >> 1) - 1) && (strides >= ((mask >> 1) - mask));
}

static __maybe_unused int
dma_mchp_xdmac_convert_interval_to_strides(uint32_t interval, uint32_t data_size, uint32_t *strides)
{
	uint32_t tmp_strides;

	if ((interval % data_size) != 0) {
		return -ENOTSUP;
	}

	tmp_strides = interval / data_size;
	if (!dma_mchp_xdmac_fit_stride_range(tmp_strides)) {
		return -ENOTSUP;
	}

	*strides = tmp_strides;

	return 0;
}

static int dma_mchp_xdmac_convert(struct dma_config *dma_cfg, struct dma_block_config *blk_cfg,
				  struct dma_mchp_xdmac_linked_list_desc_view3 *desc,
				  struct dma_mchp_xdmac_linked_list_desc_view3 *next_desc)
{
	struct dma_mchp_xdmac_linked_list_desc_view3 tmp_desc = {0};
	uint32_t block_len, data_size, burst_size;

	data_size = find_msb_set(dma_cfg->source_data_size) - 1;
	burst_size = find_msb_set(dma_cfg->source_burst_length) - 1;
	block_len = blk_cfg->block_size >> data_size;

	/* Requested block size exceeds maximum number of transfers */
	if (block_len > (XDMAC_CUBC_UBLEN_Msk >> XDMAC_CUBC_UBLEN_Pos)) {
		return -EINVAL;
	}

	if (blk_cfg->source_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		tmp_desc.mbr_cfg |= XDMAC_CC_SAM(XDMAC_CC_SAM_INCREMENTED_AM_Val);
	} else if (blk_cfg->source_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) {
		tmp_desc.mbr_cfg |= XDMAC_CC_SAM(XDMAC_CC_SAM_FIXED_AM_Val);
	} else {
		return -ENOTSUP;
	}

	if (blk_cfg->dest_addr_adj == DMA_ADDR_ADJ_INCREMENT) {
		tmp_desc.mbr_cfg |= XDMAC_CC_DAM(XDMAC_CC_DAM_INCREMENTED_AM_Val);
	} else if (blk_cfg->dest_addr_adj == DMA_ADDR_ADJ_NO_CHANGE) {
		tmp_desc.mbr_cfg |= XDMAC_CC_DAM(XDMAC_CC_DAM_FIXED_AM_Val);
	} else {
		return -ENOTSUP;
	}

	if (next_desc != NULL) {
		tmp_desc.mbr_ubc |= XDMAC_MBR_UBC_NVIEW(XDMAC_MBR_UBC_NVIEW_NDV3) |
				    XDMAC_MBR_UBC_NDEN_Msk | XDMAC_MBR_UBC_NSEN_Msk |
				    XDMAC_MBR_UBC_NDE_Msk;
	}

#if defined(XDMAC_CC_SIF_AHB_IF1)
	tmp_desc.mbr_cfg |= XDMAC_CC_SIF_AHB_IF1;
#endif
#if defined(XDMAC_CC_DIF_AHB_IF1)
	tmp_desc.mbr_cfg |= XDMAC_CC_DIF_AHB_IF1;
#endif

	tmp_desc.mbr_nda |= (uint32_t)next_desc;
	tmp_desc.mbr_ubc |= XDMAC_MBR_UBC_UBLEN(block_len);
	tmp_desc.mbr_sa = blk_cfg->source_address;
	tmp_desc.mbr_da = blk_cfg->dest_address;

	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY) {
		tmp_desc.mbr_cfg |= XDMAC_CC_PERID_Msk;
	} else {
		tmp_desc.mbr_cfg |= XDMAC_CC_PERID(dma_cfg->dma_slot);
	}

	tmp_desc.mbr_cfg |= XDMAC_CC_TYPE(dma_cfg->channel_direction == MEMORY_TO_MEMORY
						  ? XDMAC_CC_TYPE_MEM_TRAN
						  : XDMAC_CC_TYPE_PER_TRAN_Val);
	tmp_desc.mbr_cfg |= XDMAC_CC_DSYNC(dma_cfg->channel_direction == MEMORY_TO_PERIPHERAL
						   ? XDMAC_CC_DSYNC_MEM2PER_Val
						   : XDMAC_CC_DSYNC_PER2MEM_Val);

	tmp_desc.mbr_cfg |= XDMAC_CC_DWIDTH(data_size);

	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY) {
		tmp_desc.mbr_cfg |= XDMAC_CC_MBSIZE(burst_size == 0 ? 0 : burst_size - 1);
	} else {
		tmp_desc.mbr_cfg |= XDMAC_CC_CSIZE(burst_size);
	}

	*desc = tmp_desc;

	return 0;
}

static int dma_mchp_xdmac_config_blocks_transfer(const struct device *dev, uint32_t channel,
						 struct dma_config *dma_cfg)
{
	struct dma_mchp_xdmac_channel_config *xdmac_channel_config;
	struct dma_mchp_xdmac_hwchannel_config channel_config;
	struct dma_mchp_xdmac_transfer_config transfer_config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	struct dma_block_config *blk_cfg = dma_cfg->head_block;
	int ret = 0, i;

	if (dma_cfg->block_count > CONFIG_DMA_MCHP_XDMAC_G1_DESC_COUNT) {
		return -EINVAL;
	}
	for (i = 0; (i < dma_cfg->block_count) && (ret == 0); i++) {
		if ((i + 1) == dma_cfg->block_count) {
			if (dma_cfg->cyclic != 0) {
				ret = dma_mchp_xdmac_convert(dma_cfg, blk_cfg,
							     &dev_data->descs[channel][i],
							     &dev_data->descs[channel][0]);
			} else if (blk_cfg->next_block == NULL) {
				ret = dma_mchp_xdmac_convert(dma_cfg, blk_cfg,
							     &dev_data->descs[channel][i], NULL);
			} else {
				ret = -EINVAL;
			}
		} else {
			if (blk_cfg->next_block == NULL) {
				ret = -EINVAL;
			} else {
				/* Assemble a next link */
				ret = dma_mchp_xdmac_convert(dma_cfg, blk_cfg,
							     &dev_data->descs[channel][i],
							     &dev_data->descs[channel][i + 1]);
			}
		}

		if (ret != 0) {
			break;
		}
		/* Move to next block */
		blk_cfg = blk_cfg->next_block;
	}

	if (ret != 0) {
		return ret;
	}

	/* Ensure all descriptors were written */
	barrier_dsync_fence_full();

	(void)memset(&channel_config, 0, sizeof(channel_config));
#ifdef XDMAC_CC_SIF_AHB_IF1
	channel_config.cfg |= XDMAC_CC_SIF_AHB_IF1;
#endif
#ifdef XDMAC_CC_DIF_AHB_IF1
	channel_config.cfg |= XDMAC_CC_DIF_AHB_IF1;
#endif
	if (dma_cfg->channel_direction == MEMORY_TO_MEMORY) {
		/* Errata 2.2.2: memory-to-memory require an unused peripheral ID. */
		channel_config.cfg |= XDMAC_CC_PERID_Msk;
	} else {
		channel_config.cfg |= XDMAC_CC_PERID(dma_cfg->dma_slot);
	}
	channel_config.ds_msp = 0u;
	channel_config.sus = 0u;
	channel_config.dus = 0u;
	channel_config.cie = XDMAC_CIE_LIE_Msk;
	/* NOTE: LIE cannot detect end of the cyclic transfer, use BIE and detect crossing
	 * descriptor head
	 */
	channel_config.cie |=
		(dma_cfg->complete_callback_en || (dma_cfg->cyclic != 0) ? XDMAC_CIE_BIE_Msk : 0);
	channel_config.cie |= (dma_cfg->error_callback_dis ? 0 : XDMAC_INT_ERR);

	ret = dma_mchp_xdmac_channel_configure(dev, channel, &channel_config);
	if (ret != 0) {
		return ret;
	}

	(void)memset(&transfer_config, 0, sizeof(transfer_config));
	transfer_config.ndc = XDMAC_CNDC_NDE_Msk | XDMAC_CNDC_NDSUP_Msk | XDMAC_CNDC_NDDUP_Msk |
			      XDMAC_CNDC_NDVIEW(XDMAC_CNDC_NDVIEW_NDV3_Val);
	transfer_config.nda = (uint32_t)&dev_data->descs[channel][0];
	ret = dma_mchp_xdmac_transfer_configure(dev, channel, &transfer_config);
	if (ret != 0) {
		return ret;
	}

	xdmac_channel_config = &dev_data->dma_channels[channel];
	xdmac_channel_config->callback = dma_cfg->dma_callback;
	xdmac_channel_config->user_data = dma_cfg->user_data;
	xdmac_channel_config->cyclic = dma_cfg->cyclic;
	xdmac_channel_config->error_callback_dis = dma_cfg->error_callback_dis;
	xdmac_channel_config->complete_callback_en = dma_cfg->complete_callback_en;
	xdmac_channel_config->data_size = find_msb_set(dma_cfg->source_data_size) - 1;

	dev_data->config_mode = DMA_MCHP_XDMAC_G1_CONFIG_BLOCKS;

	return ret;
}

static int dma_mchp_xdmac_config(const struct device *dev, uint32_t channel, struct dma_config *cfg)
{
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;
	int ret = 0;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	if (dev_data->dma_channels[channel].state != DMA_STATE_INIT &&
	    dev_data->dma_channels[channel].state != DMA_STATE_CONFIGURED) {
		LOG_ERR("Config Channel %d at invalidate state", channel);
		return -EINVAL;
	}

	if (cfg->source_data_size != cfg->dest_data_size) {
		LOG_ERR("Source data size and destination data size must be the same");
		return -EINVAL;
	}

	if (cfg->source_burst_length != cfg->dest_burst_length) {
		LOG_ERR("Source burst_length and destination burst_length must be the same");
		return -EINVAL;
	}

	if (cfg->source_data_size != 1u && cfg->source_data_size != 2u &&
	    cfg->source_data_size != 4u) {
		LOG_ERR("Invalid 'source_data_size' value");
		return -EINVAL;
	}

	if ((cfg->channel_direction != PERIPHERAL_TO_MEMORY) &&
	    (cfg->channel_direction != MEMORY_TO_PERIPHERAL) &&
	    (cfg->channel_direction != MEMORY_TO_MEMORY)) {
		LOG_ERR("Unsupported 'channel_direction'");
		return -ENOTSUP;
	}

	if (cfg->block_count == 0 || cfg->head_block == NULL) {
		return -EINVAL;
	}

	if ((cfg->block_count) == 1u && (cfg->cyclic == 0)) {
		ret = dma_mchp_xdmac_config_single_transfer(dev, channel, cfg);
	} else {
		ret = dma_mchp_xdmac_config_blocks_transfer(dev, channel, cfg);
	}

	return ret;
}

static int dma_mchp_xdmac_transfer_reload(const struct device *dev, uint32_t channel, uint32_t src,
					  uint32_t dst, size_t size)
{
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	struct dma_mchp_xdmac_transfer_config transfer_config = {0};
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	transfer_config.sa = src;
	transfer_config.da = dst;
	transfer_config.ublen = size >> dev_data->dma_channels[channel].data_size;

	/* Cannot reload block configuration */
	if (dev_data->config_mode != DMA_MCHP_XDMAC_G1_CONFIG_SINGLE) {
		return -EINVAL;
	}

	return dma_mchp_xdmac_transfer_configure(dev, channel, &transfer_config);
}

int dma_mchp_xdmac_transfer_start(const struct device *dev, uint32_t channel)
{
	const struct dma_mchp_xdmac_config *config = dev->config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;

	xdmac_registers_t *const regs = config->regs;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	if (dev_data->dma_channels[channel].state != DMA_STATE_CONFIGURED &&
	    dev_data->dma_channels[channel].state != DMA_STATE_RUNNING) {
		LOG_ERR("Start Channel %d at invalidate state", channel);
		return -EINVAL;
	}

	/* Check if the channel is enabled */
	if (regs->XDMAC_GS & (XDMAC_GS_ST0_Msk << channel)) {
		LOG_DBG("Channel %d already enabled", channel);
		return -EBUSY;
	}

	/* Enable channel interrupt */
	regs->XDMAC_GIE = XDMAC_GIE_IE0_Msk << channel;
	/* Enable channel */
	regs->XDMAC_GE = XDMAC_GE_EN0_Msk << channel;

	dev_data->dma_channels[channel].state = DMA_STATE_RUNNING;

	return 0;
}

int dma_mchp_xdmac_transfer_stop(const struct device *dev, uint32_t channel)
{
	const struct dma_mchp_xdmac_config *config = dev->config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;

	xdmac_registers_t *const regs = config->regs;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	if (dev_data->dma_channels[channel].state == DMA_STATE_INIT) {
		LOG_ERR("Channel %d not configured", channel);
		return -EINVAL;
	}

	/* Check if the channel is enabled */
	if (!(regs->XDMAC_GS & (XDMAC_GS_ST0_Msk << channel))) {
		return 0;
	}

	/* Disable channel */
	regs->XDMAC_GD = XDMAC_GD_DI0_Msk << channel;
	/* Disable channel interrupt */
	regs->XDMAC_GID = XDMAC_GID_ID0_Msk << channel;
	/* Disable all channel interrupts */
	regs->XDMAC_CHID[channel].XDMAC_CID = 0xFF;
	/* Clear the pending Interrupt Status bit(s) */
	(void)regs->XDMAC_CHID[channel].XDMAC_CIS;

	dev_data->dma_channels[channel].state = DMA_STATE_CONFIGURED;

	return 0;
}

static int dma_mchp_xdmac_initialize(const struct device *dev)
{
	const struct dma_mchp_xdmac_config *const dev_config = dev->config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t hw_channels;
	int ret;

	xdmac_registers_t *const regs = dev_config->regs;

	/* Configure interrupts */
	dev_config->irq_config(dev);

	/* Enable XDMAC clock in PMC */
	ret = clock_control_on(dev_config->clock, dev_config->clock_cfg);
	if (ret < 0) {
		return -ENODEV;
	}

	hw_channels = FIELD_GET(XDMAC_GTYPE_NB_CH_Msk, regs->XDMAC_GTYPE) + 1;
	if (dev_data->dma_ctx.dma_channels > hw_channels) {
		LOG_ERR("Maximum supported channels is %d", hw_channels);
		(void)clock_control_off(dev_config->clock, dev_config->clock_cfg);
		return -EINVAL;
	}

	/* Disable all channels */
	regs->XDMAC_GD = UINT32_MAX;
	/* Disable all channel interrupts */
	regs->XDMAC_GID = UINT32_MAX;

	/* Enable module's IRQ */
	irq_enable(dev_config->irq_id);

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

static int dma_mchp_xdmac_suspend(const struct device *dev, uint32_t channel)
{
	const struct dma_mchp_xdmac_config *config = dev->config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;

	xdmac_registers_t *const regs = config->regs;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	switch (dev_data->dma_channels[channel].state) {
	case DMA_STATE_RUNNING:
		break;
	case DMA_STATE_SUSPENDED:
		return 0;
	default:
		LOG_ERR("Suspend Channel %d at invalidate state", channel);
		return -EINVAL;
	}

	if (!(regs->XDMAC_GS & BIT(channel))) {
		LOG_DBG("Channel %d not enabled", channel);
		return -EINVAL;
	}

	if (regs->XDMAC_GRS & BIT(channel) || regs->XDMAC_GWS & BIT(channel)) {
		LOG_DBG("Channel %d already suspended", channel);
		return 0;
	}

	regs->XDMAC_GRWS |= BIT(channel);

	dev_data->dma_channels[channel].state = DMA_STATE_SUSPENDED;

	return 0;
}

static int dma_mchp_xdmac_resume(const struct device *dev, uint32_t channel)
{
	const struct dma_mchp_xdmac_config *config = dev->config;
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;

	xdmac_registers_t *const regs = config->regs;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	switch (dev_data->dma_channels[channel].state) {
	case DMA_STATE_SUSPENDED:
		break;
	case DMA_STATE_RUNNING:
		return 0;
	default:
		LOG_ERR("Resume Channel %d at invalidate state", channel);
		return -EINVAL;
	}

	if (!(regs->XDMAC_GS & BIT(channel))) {
		LOG_DBG("Channel %d not enabled", channel);
		return -EINVAL;
	}

	if (!(regs->XDMAC_GRS & BIT(channel) || regs->XDMAC_GWS & BIT(channel))) {
		LOG_DBG("Channel %d not suspended", channel);
		return 0;
	}

	regs->XDMAC_GRWR |= BIT(channel);

	dev_data->dma_channels[channel].state = DMA_STATE_RUNNING;

	return 0;
}

static int dma_mchp_xdmac_get_status(const struct device *dev, uint32_t channel,
				     struct dma_status *status)
{
	struct dma_mchp_xdmac_data *const dev_data = dev->data;
	const struct dma_mchp_xdmac_config *const dev_config = dev->config;
	uint32_t channel_num = dev_data->dma_ctx.dma_channels;
	xdmac_registers_t *const regs = dev_config->regs;
	uint32_t ublen, channel_config;

	if (channel >= channel_num) {
		LOG_ERR("Channel %d out of range", channel);
		return -EINVAL;
	}

	channel_config = regs->XDMAC_CHID[channel].XDMAC_CC;
	ublen = regs->XDMAC_CHID[channel].XDMAC_CUBC;

	/* we need to check some of the XDMAC_CC registers to determine the DMA
	 * direction
	 */
	if ((channel_config & XDMAC_CC_TYPE_Msk) == 0) {
		status->dir = MEMORY_TO_MEMORY;
	} else if ((channel_config & XDMAC_CC_DSYNC_Msk) == XDMAC_CC_DSYNC_MEM2PER) {
		status->dir = MEMORY_TO_PERIPHERAL;
	} else {
		status->dir = PERIPHERAL_TO_MEMORY;
	}

	status->busy = (ublen > 0) || (dev_data->dma_channels[channel].state == DMA_STATE_RUNNING);
	status->pending_length = ublen << dev_data->dma_channels[channel].data_size;

	return 0;
}

static bool dma_mchp_xdmac_chan_filter(const struct device *dev, int channel, void *filter_param)
{
	if (filter_param == NULL) {
		return true;
	}

	return *((uint32_t *)filter_param) == channel;
}

static DEVICE_API(dma, dma_mchp_xdmac_driver_api) = {
	.config = dma_mchp_xdmac_config,
	.reload = dma_mchp_xdmac_transfer_reload,
	.start = dma_mchp_xdmac_transfer_start,
	.stop = dma_mchp_xdmac_transfer_stop,
	.suspend = dma_mchp_xdmac_suspend,
	.resume = dma_mchp_xdmac_resume,
	.get_status = dma_mchp_xdmac_get_status,
	.chan_filter = dma_mchp_xdmac_chan_filter,
};

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	static struct dma_mchp_xdmac_linked_list_desc_view3 mchp_xdmac_##n##_descs[DT_INST_PROP(   \
		n, dma_channels)][CONFIG_DMA_MCHP_XDMAC_G1_DESC_COUNT] __nocache;                  \
	static struct dma_mchp_xdmac_channel_config                                                \
		mchp_xdmac_##n##_channel_config[DT_INST_PROP(n, dma_channels)];                    \
	static void mchp_xdmac_##n##_irq_config(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), dma_mchp_xdmac_isr,         \
			    DEVICE_DT_INST_GET(n), 0);                                             \
	}                                                                                          \
                                                                                                   \
	static const struct dma_mchp_xdmac_config mchp_xdmac_##n##_config = {                      \
		.regs = (xdmac_registers_t *)DT_INST_REG_ADDR(n),                                  \
		.irq_config = mchp_xdmac_##n##_irq_config,                                         \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_cfg = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),            \
		.irq_id = DT_INST_IRQN(n),                                                         \
	};                                                                                         \
                                                                                                   \
	static ATOMIC_DEFINE(mchp_xdmac_##n##_channels_atomic, DT_INST_PROP(n, dma_channels));     \
                                                                                                   \
	static struct dma_mchp_xdmac_data mchp_xdmac_##n##_data = {                                \
		.dma_ctx.magic = DMA_MAGIC,                                                        \
		.dma_ctx.atomic = mchp_xdmac_##n##_channels_atomic,                                \
		.dma_ctx.dma_channels = DT_INST_PROP(n, dma_channels),                             \
		.dma_channels = mchp_xdmac_##n##_channel_config,                                   \
		.descs = mchp_xdmac_##n##_descs,                                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &dma_mchp_xdmac_initialize, NULL, &mchp_xdmac_##n##_data,         \
			      &mchp_xdmac_##n##_config, PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY,    \
			      &dma_mchp_xdmac_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
