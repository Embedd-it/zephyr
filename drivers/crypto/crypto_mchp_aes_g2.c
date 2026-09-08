/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/** @file
 * @brief Microchip AES crypto accelerator driver.
 *
 */

#define DT_DRV_COMPAT microchip_aes_g2_crypto

#include <zephyr/crypto/crypto.h>
#include <zephyr/crypto/cipher.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>
#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
#include <zephyr/drivers/dma.h>
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

#include <string.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crypto_mchp_aes_g2, CONFIG_CRYPTO_LOG_LEVEL);

/* Cipher block size */
#define BLOCK_SIZE    16u
/* Tipical GCM nonce length (12-byte) */
#define GCM_NONCE_LEN 12u

enum gcm_phase {
	/* Feeding Additional Authenticated Data; no output is produced */
	GCM_PHASE_AAD,
	/* Feeding plaintext/ciphertext; output is produced */
	GCM_PHASE_DATA,
	/* Waiting for the automatically generated authentication tag */
	GCM_PHASE_TAG,
};

struct crypto_mchp_aes_config {
	/* Register block */
	aes_registers_t *regs;

	/* Clock driver */
	const struct device *clock;

	/* Main clock subsystem. */
	clock_control_subsys_t clock_subsys;

	/* IRQ setup function */
	void (*irq_config)(const struct device *dev);

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	/* DMA data */
	struct {
		const struct device *dev;
		uint32_t tx_channel;
		uint32_t tx_perid;
		uint32_t rx_channel;
		uint32_t rx_perid;
	} dma;
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */
};

struct crypto_mchp_aes_data {
	/* Driver mutex */
	struct k_mutex mutex;

	/* Driver interrupt synchronization semaphore */
	struct k_sem done;

	/* Cryptographic key */
	uint8_t key[32];

	/* Key length */
	size_t key_len;

	/* Current session's mode */
	enum cipher_mode mode;

	/* Whether we are encrypting or decrypting */
	enum cipher_op dir;

	/* Cursor for the buffers of the current operation */
	size_t buffer_index;

	/* Total number of bytes to process */
	size_t total_length;

	/* Input data for the operation */
	const uint8_t *input_buffer;

	/* Output buffer for the operation */
	uint8_t *output_buffer;

	/* Completion callback */
	cipher_completion_cb completion_cb;

	/* Last packet */
	struct cipher_pkt *last_pkt;

	/* GCM AEAD packet of the operation in progress, holds the tag pointer */
	struct cipher_aead_pkt *aead_pkt;

	/* GCM current phase of the operation in progress */
	enum gcm_phase gcm_phase;

	/* GCM Additional Authenticated Data cursor */
	const uint8_t *aad_buffer;

	/* GCM Additional Authenticated Data length */
	size_t aad_length;

	/* GCM Additional Authenticated Data cursor */
	size_t aad_index;

	/* GCM: length of the authentication tag, in bytes */
	size_t tag_len;

	/* Set by the ISR if, on GCM decryption, the computed tag does not match the
	 * one supplied by the caller
	 */
	bool tag_mismatch: 1;

	/* Only supports one session at a time */
	bool session_active: 1;

	/* Whether the session is asynchronous */
	bool async: 1;

	/* Whether an (async) operation is in progress */
	bool processing: 1;

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	/* Whether the input for the operation in progress is being fed by DMA */
	bool dma_active: 1;
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */
};

static void crypto_mchp_aes_finish(struct crypto_mchp_aes_data *dev_data,
				   const struct crypto_mchp_aes_config *dev_config, int status);

/*
 * Configure the key into the peripheral registers
 */
static void crypto_mchp_aes_set_key(aes_registers_t *const regs, const uint8_t *key, size_t len)
{
	const uint32_t *key_u32_pointer = (const uint32_t *)key;

	for (size_t i = 0u; i < len / sizeof(uint32_t); i++) {
		regs->AES_KEYWR[i] = key_u32_pointer[i];
	}
}

/*
 * Configure the IV into the peripheral registers
 */
static void crypto_mchp_aes_set_initial_vector(aes_registers_t *const regs, const uint8_t *vector)
{
	const uint32_t *vector_u32_pointer = (const uint32_t *)vector;

	for (size_t i = 0u; i < ARRAY_SIZE(regs->AES_IVR); i++) {
		regs->AES_IVR[i] = vector_u32_pointer[i];
	}
}

/*
 * Configure the input into the peripheral registers
 */
static void crypto_mchp_aes_set_input_data(aes_registers_t *const regs, const uint8_t *data)
{
	const uint32_t *data_u32_pointer = (const uint32_t *)data;
	uint8_t size = 4u;

	/* In 32, 16, and 8-bit CFB modes, writing to AES_IDATAR1, AES_IDATAR2 and AES_IDATAR3
	 * is not allowed and may lead to errors in processing.
	 */
	if ((regs->AES_MR & AES_MR_OPMOD_Msk) == AES_MR_OPMOD_CFB) {
		if ((regs->AES_MR & AES_MR_CFBS_Msk) == AES_MR_CFBS_SIZE_128BIT) {
			size = 4u;
		} else if ((regs->AES_MR & AES_MR_CFBS_Msk) == AES_MR_CFBS_SIZE_64BIT) {
			size = 2u;
		} else {
			size = 1u;
		}
	}

	for (size_t i = 0u; i < size; i++) {
		regs->AES_IDATAR[i] = data_u32_pointer[i];
	}
}

/*
 * Get the output from the peripheral registers
 */
static void crypto_mchp_aes_get_output_data(aes_registers_t *const regs, uint8_t *data)
{
	uint32_t *data_u32_pointer = (uint32_t *)data;

	for (size_t i = 0u; i < ARRAY_SIZE(regs->AES_ODATAR); i++) {
		data_u32_pointer[i] = regs->AES_ODATAR[i];
	}
}

/*
 * Get the authentication tag from the peripheral registers
 */
static void crypto_mchp_aes_get_tag(aes_registers_t *const regs, uint8_t *data)
{
	uint32_t *data_u32_pointer = (uint32_t *)data;

	for (size_t i = 0u; i < ARRAY_SIZE(regs->AES_TAGR); i++) {
		data_u32_pointer[i] = regs->AES_TAGR[i];
	}
}

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)

static void crypto_mchp_aes_dma_done_callback(const struct device *dma_dev, void *user_data,
					      uint32_t channel, int status);

/*
 * Start feeding a buffer to AES_IDATAR0 via DMA
 */
static int crypto_mchp_aes_dma_start_input(struct cipher_ctx *ctx, const uint8_t *in_buf,
					   size_t in_len)
{
	const struct crypto_mchp_aes_config *dev_config = ctx->device->config;
	aes_registers_t *const regs = dev_config->regs;
	struct dma_config dma_cfg = {0u};
	struct dma_block_config dma_blk = {0u};
	int ret = 0;

	dma_cfg.channel_direction = MEMORY_TO_PERIPHERAL;
	dma_cfg.source_data_size = sizeof(uint32_t);
	dma_cfg.dest_data_size = sizeof(uint32_t);
	/* Chunk size of 4 words (one AES block) per Table "DMA Data Transfer Type for the
	 * Different Operating Modes"; ECB, CBC, CTR and GCM all use this value.
	 */
	dma_cfg.source_burst_length = 4u;
	dma_cfg.dest_burst_length = 4u;
	dma_cfg.dma_slot = dev_config->dma.tx_perid;
	dma_cfg.block_count = 1u;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_callback = crypto_mchp_aes_dma_done_callback;
	dma_cfg.user_data = ctx;

	dma_blk.source_address = (uint32_t)in_buf;
	dma_blk.dest_address = (uint32_t)&regs->AES_IDATAR[0];
	dma_blk.block_size = in_len;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	/* The datasheet requires a non-incrementing destination address at AES_IDATAR0 */
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	ret = dma_config(dev_config->dma.dev, dev_config->dma.tx_channel, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("Unable to configure the input DMA: %i", ret);
		return ret;
	}

	return dma_start(dev_config->dma.dev, dev_config->dma.tx_channel);
}

/*
 * Start draining AES_ODATAR into a buffer via DMA
 */
static int crypto_mchp_aes_dma_start_output(struct cipher_ctx *ctx, uint8_t *out_buf,
					    size_t out_len)
{
	const struct crypto_mchp_aes_config *dev_config = ctx->device->config;
	aes_registers_t *const regs = dev_config->regs;
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int ret = 0;

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = sizeof(uint32_t);
	dma_cfg.dest_data_size = sizeof(uint32_t);
	dma_cfg.source_burst_length = 4u;
	dma_cfg.dest_burst_length = 4u;
	dma_cfg.dma_slot = dev_config->dma.rx_perid;
	dma_cfg.block_count = 1u;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_callback = crypto_mchp_aes_dma_done_callback;
	dma_cfg.user_data = ctx;

	/* The datasheet requires a non-incrementing source address at AES_ODATAR0 */
	dma_blk.source_address = (uint32_t)&regs->AES_ODATAR[0];
	dma_blk.dest_address = (uint32_t)out_buf;
	dma_blk.block_size = out_len;
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	dma_blk.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;

	ret = dma_config(dev_config->dma.dev, dev_config->dma.rx_channel, &dma_cfg);
	if (ret != 0) {
		LOG_ERR("Unable to configure the output DMA: %i", ret);
		return ret;
	}

	return dma_start(dev_config->dma.dev, dev_config->dma.rx_channel);
}

/*
 * Advance a DMA-driven GCM operation to its next phase.
 */
static void crypto_mchp_aes_dma_gcm_advance(struct cipher_ctx *ctx, uint32_t channel)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;
	const struct crypto_mchp_aes_config *dev_config = ctx->device->config;
	int ret = 0;

	if (dev_data->gcm_phase == GCM_PHASE_AAD) {
		dev_data->aad_index = dev_data->aad_length;
		dev_data->gcm_phase =
			(dev_data->total_length > 0u) ? GCM_PHASE_DATA : GCM_PHASE_TAG;

		if (dev_data->gcm_phase == GCM_PHASE_DATA) {
			/* Arm the output channel before the input one, so it is ready
			 * to drain AES_ODATAR as soon as the first block completes.
			 */
			ret = crypto_mchp_aes_dma_start_output(ctx, dev_data->output_buffer,
							       dev_data->total_length);
			if (ret == 0) {
				ret = crypto_mchp_aes_dma_start_input(ctx, dev_data->input_buffer,
								      dev_data->total_length);
			}
			if (ret != 0) {
				LOG_ERR("Unable to start the DATA phase DMA: %i", ret);
				crypto_mchp_aes_finish(dev_data, dev_config, ret);
			}
			return;
		}
		/* No data: fall through to GCM_PHASE_TAG below */
	} else if (dev_data->gcm_phase == GCM_PHASE_DATA) {
		if (channel != dev_config->dma.rx_channel) {
			return;
		}
		dev_data->buffer_index = dev_data->total_length;
		dev_data->gcm_phase = GCM_PHASE_TAG;
	}

	/* GCM_PHASE_TAG: AES_MR.GTAGEN generates the tag automatically once AADLEN
	 * and CLEN worth of data have been fed. AES_IER.TAGRDY has been enabled
	 * since the start of the operation, so the existing interrupt handler
	 * takes it from here.
	 */
}

/*
 * Completion for the DMA-driven path.
 */
static void crypto_mchp_aes_dma_done_callback(const struct device *dma_dev, void *user_data,
					      uint32_t channel, int status)
{
	ARG_UNUSED(dma_dev);

	struct cipher_ctx *ctx = (struct cipher_ctx *)user_data;
	struct crypto_mchp_aes_data *dev_data = ctx->device->data;
	const struct crypto_mchp_aes_config *dev_config = ctx->device->config;

	if (status != 0) {
		crypto_mchp_aes_finish(dev_data, dev_config, status);
		return;
	}

	if (dev_data->mode == CRYPTO_CIPHER_MODE_GCM) {
		crypto_mchp_aes_dma_gcm_advance(ctx, channel);
		return;
	}

	/* Non-GCM: a clean completion is only meaningful once the output (rx)
	 * channel has drained AES_ODATAR into the caller's buffer; the input (tx)
	 * channel's own completion is not otherwise interesting.
	 */
	if (channel != dev_config->dma.rx_channel) {
		return;
	}

	dev_data->buffer_index = dev_data->total_length;
	dev_data->last_pkt->out_len = dev_data->buffer_index;
	crypto_mchp_aes_finish(dev_data, dev_config, 0);
}

/*
 * Kick off a DMA-driven GCM operation at whatever phase it starts in.
 */
static int crypto_mchp_aes_dma_gcm_start(struct cipher_ctx *ctx)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;
	int ret = 0;

	if (dev_data->gcm_phase == GCM_PHASE_AAD) {
		return crypto_mchp_aes_dma_start_input(ctx, dev_data->aad_buffer,
						       dev_data->aad_length);
	}

	if (dev_data->gcm_phase == GCM_PHASE_DATA) {
		ret = crypto_mchp_aes_dma_start_output(ctx, dev_data->output_buffer,
						       dev_data->total_length);
		if (ret != 0) {
			return ret;
		}
		return crypto_mchp_aes_dma_start_input(ctx, dev_data->input_buffer,
						       dev_data->total_length);
	}

	/* GCM_PHASE_TAG: no AAD and no data; the automatic tag generation is already
	 * under way and AES_IER.TAGRDY is enabled, nothing left to DMA.
	 */
	return 0;
}
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

/*
 * GCM feeds the AES core in fixed 128-bit blocks; the last, possibly partial, block of
 * the AAD or the plaintext/ciphertext is zero-padded in a scratch buffer before being
 * written. AES_AADLENR/AES_CLENR carry the true, unpadded lengths.
 */
static void crypto_mchp_aes_build_padded_block(uint8_t block[BLOCK_SIZE], const uint8_t *src,
					       size_t len)
{
	memset(block, 0u, BLOCK_SIZE);
	memcpy(block, src, len);
}

/*
 * Run a piece of the cipher with certain parameters; common for most modes of operation.
 */
static int crypto_mchp_aes_process(struct cipher_ctx *ctx, int32_t in_len, const uint8_t *in_buf,
				   int32_t *out_len, uint8_t *out_buf, const uint8_t *iv)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;
	const struct crypto_mchp_aes_config *dev_config = ctx->device->config;
	aes_registers_t *const regs = dev_config->regs;
	uint32_t mr = 0u;
#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	bool use_dma = false;
	int ret = 0;
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

	if (dev_data->processing) {
		return -EALREADY;
	}

	/* Reset the peripheral */
	regs->AES_CR = AES_CR_SWRST_Msk;

	/* Configure the operation mode */
	mr |= AES_MR_CKEY_PASSWD | AES_MR_KEYSIZE((ctx->keylen / BITS_PER_BYTE) - 2u) |
	      AES_MR_CIPHER(dev_data->dir == CRYPTO_CIPHER_OP_ENCRYPT ? 1u : 0u);

	switch (dev_data->mode) {
	case CRYPTO_CIPHER_MODE_ECB:
		mr |= AES_MR_OPMOD_ECB;
		break;
	case CRYPTO_CIPHER_MODE_CBC:
		mr |= AES_MR_OPMOD_CBC;
		break;
	case CRYPTO_CIPHER_MODE_CTR:
		mr |= AES_MR_OPMOD_CTR;
		break;
	case CRYPTO_CIPHER_MODE_CFB:
		mr |= AES_MR_OPMOD_CFB;
		break;
	case CRYPTO_CIPHER_MODE_OFB:
		mr |= AES_MR_OPMOD_OFB;
		break;
	default:
		/* Checked earlier */
		break;
	}

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	/* DMA can only feed whole, block-aligned buffers to AES_IDATAR0. ECB and CBC
	 * inputs always are; CTR's may not be for a final short block, in which case
	 * fall back to the manual, per-block path below.
	 */
	use_dma = (in_len > 0) && (((size_t)in_len % BLOCK_SIZE) == 0u);

	if (use_dma) {
		mr |= AES_MR_SMOD_IDATAR0_START | AES_MR_DUALBUFF_ACTIVE;
	}
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

	regs->AES_MR = mr;

	/* Write the key to the registers */
	crypto_mchp_aes_set_key(regs, dev_data->key, dev_data->key_len);

	/* If present, write the initial vector to the registers */
	if (iv != NULL) {
		crypto_mchp_aes_set_initial_vector(regs, iv);
	}

	dev_data->buffer_index = 0;
	dev_data->input_buffer = in_buf;
	dev_data->output_buffer = out_buf;
	dev_data->total_length = in_len;

	/* Kickstart the operation */
	dev_data->processing = true;
	k_sem_take(&dev_data->done, K_NO_WAIT);

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	dev_data->dma_active = use_dma;
	if (use_dma) {
		/* Arm the output channel before the input one, so it is ready to
		 * drain AES_ODATAR as soon as the first block completes.
		 * When DMA is used, interrupts are not fired.
		 */
		ret = crypto_mchp_aes_dma_start_output(ctx, out_buf, (size_t)in_len);
		if (ret != 0) {
			LOG_ERR("Unable to start the output DMA: %i", ret);
			dev_data->processing = false;
			dev_data->dma_active = false;
			return ret;
		}

		ret = crypto_mchp_aes_dma_start_input(ctx, dev_data->input_buffer, (size_t)in_len);
		if (ret != 0) {
			LOG_ERR("Unable to start the input DMA: %i", ret);
			dma_stop(dev_config->dma.dev, dev_config->dma.rx_channel);
			dev_data->processing = false;
			dev_data->dma_active = false;
			return ret;
		}
	} else {
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */
		regs->AES_IER = AES_IER_DATRDY_Msk;

		/* If the input data is shorter than one block pad it */
		if ((size_t)in_len < BLOCK_SIZE) {
			uint8_t block[BLOCK_SIZE] = {0u};

			crypto_mchp_aes_build_padded_block(block, dev_data->input_buffer,
							   (size_t)in_len);
			crypto_mchp_aes_set_input_data(regs, block);
		} else {
			crypto_mchp_aes_set_input_data(regs, dev_data->input_buffer);
		}
		/* Start processing */
		regs->AES_CR = AES_CR_START_Msk;
#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	}
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

	/* If in synchronous operation block until done, otherwise just return */
	if (!dev_data->async) {
		k_sem_take(&dev_data->done, K_FOREVER);
		*out_len = dev_data->total_length;
	}

	return 0;
}

/*
 * In GCM mode, writes the next AAD or data block and starts processing, or moves on
 * to the next phase if the current one is exhausted. Called both to kick off the
 * operation and to chain each subsequent block.
 */
static void crypto_mchp_aes_gcm_advance(struct crypto_mchp_aes_data *dev_data,
					aes_registers_t *const regs)
{
	uint8_t block[BLOCK_SIZE] = {0u};
	size_t remaining = 0u;

	/* Currently feeding additional data, only for authentication */
	if (dev_data->gcm_phase == GCM_PHASE_AAD) {
		if (dev_data->aad_index < dev_data->aad_length) {
			remaining = dev_data->aad_length - dev_data->aad_index;

			crypto_mchp_aes_build_padded_block(
				block, &dev_data->aad_buffer[dev_data->aad_index],
				MIN(remaining, BLOCK_SIZE));
			crypto_mchp_aes_set_input_data(regs, block);
			regs->AES_CR = AES_CR_START_Msk;
			return;
		}

		dev_data->gcm_phase = (dev_data->total_length > 0) ? GCM_PHASE_DATA : GCM_PHASE_TAG;
	}

	/* Currently feeding data to be both authenticated and encrypted */
	if (dev_data->gcm_phase == GCM_PHASE_DATA) {
		if (dev_data->buffer_index < dev_data->total_length) {
			remaining = dev_data->total_length - dev_data->buffer_index;

			crypto_mchp_aes_build_padded_block(
				block, &dev_data->input_buffer[dev_data->buffer_index],
				MIN(remaining, BLOCK_SIZE));
			crypto_mchp_aes_set_input_data(regs, block);
			regs->AES_CR = AES_CR_START_Msk;
			return;
		}

		dev_data->gcm_phase = GCM_PHASE_TAG;
	}

	/* GCM_PHASE_TAG: AES_MR.GTAGEN generates the tag automatically once AADLEN and
	 * CLEN worth of data have been fed; just wait for TAGRDY.
	 */
}

/*
 * Common validation on processing parameters
 */
static int crypto_mchp_aes_verify_context_and_packet(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;

	if (!dev_data || !pkt) {
		return -EINVAL;
	}

	if ((pkt->in_len > 0 && pkt->in_buf == NULL) || pkt->out_buf == NULL) {
		return -EINVAL;
	}

	return 0;
}

/*
 * Handler for GCM mode
 */
static int crypto_mchp_aes_gcm_handler(struct cipher_ctx *ctx, struct cipher_aead_pkt *pkt,
				       uint8_t *nonce)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;
	const struct crypto_mchp_aes_config *dev_config = ctx->device->config;
	aes_registers_t *const regs = dev_config->regs;
	uint32_t nonce_len = ctx->mode_params.gcm_info.nonce_len;
	uint32_t tag_len = ctx->mode_params.gcm_info.tag_len;
	uint8_t j0[BLOCK_SIZE] = {0u};
	uint32_t mr = 0u;
#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	bool use_dma = false;
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */
	int ret = 0;

	if (!dev_data || !pkt || !pkt->pkt) {
		return -EINVAL;
	}

	if ((pkt->pkt->in_len > 0 && pkt->pkt->in_buf == NULL) || pkt->pkt->out_buf == NULL) {
		return -EINVAL;
	}

	if (pkt->pkt->in_len > pkt->pkt->out_buf_max) {
		LOG_ERR("Short of output buffer.");
		return -ENOSR;
	}

	if (pkt->ad_len > 0 && pkt->ad == NULL) {
		return -EINVAL;
	}

	if (pkt->tag == NULL) {
		LOG_ERR("No tag buffer provided");
		return -EINVAL;
	}

	if (nonce_len != GCM_NONCE_LEN) {
		LOG_ERR("Only 96-bit GCM nonces are supported, got %u bits",
			nonce_len * BITS_PER_BYTE);
		return -ENOTSUP;
	}

	if (tag_len == 0u || tag_len > BLOCK_SIZE) {
		LOG_ERR("Invalid GCM tag length: %u", tag_len);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	if (dev_data->processing) {
		k_mutex_unlock(&dev_data->mutex);
		return -EALREADY;
	}

	dev_data->last_pkt = pkt->pkt;
	dev_data->aead_pkt = pkt;
	dev_data->tag_len = tag_len;
	dev_data->tag_mismatch = false;

	/* Reset the peripheral */
	regs->AES_CR = AES_CR_SWRST_Msk;

	mr = AES_MR_CKEY_PASSWD | AES_MR_KEYSIZE((ctx->keylen / BITS_PER_BYTE) - 2u) |
	     AES_MR_CIPHER(dev_data->dir == CRYPTO_CIPHER_OP_ENCRYPT ? 1u : 0u) | AES_MR_OPMOD_GCM |
	     AES_MR_GTAGEN_Msk;

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	/* DMA can only feed whole, block-aligned buffers to AES_IDATAR0; fall back to
	 * the manual, per-block path below if either the AAD or the data is not a
	 * multiple of the block size.
	 */
	use_dma = ((pkt->ad_len % BLOCK_SIZE) == 0u) && ((pkt->pkt->in_len % BLOCK_SIZE) == 0u);

	if (use_dma) {
		mr |= AES_MR_SMOD_IDATAR0_START | AES_MR_DUALBUFF_ACTIVE;
	}
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

	regs->AES_MR = mr;

	regs->AES_IER = AES_IER_DATRDY_Msk | AES_IER_TAGRDY_Msk;

	/* Writing the key triggers automatic GCM hash subkey (H) generation; this has to complete
	 * (DATRDY) before anything else, including writing the IV, can happen.
	 */
	k_sem_take(&dev_data->done, K_NO_WAIT);
	crypto_mchp_aes_set_key(regs, dev_data->key, dev_data->key_len);
	k_sem_take(&dev_data->done, K_FOREVER);

	/* Only 96-bit nonces are handled: J0 = nonce || 0^31 || 1, IV = inc32(J0) */
	memcpy(j0, nonce, GCM_NONCE_LEN);
	sys_put_be32(2u, &j0[BLOCK_SIZE - sizeof(uint32_t)]);
	crypto_mchp_aes_set_initial_vector(regs, j0);

	regs->AES_AADLENR = pkt->ad_len;
	regs->AES_CLENR = pkt->pkt->in_len;

	dev_data->aad_buffer = pkt->ad;
	dev_data->aad_length = pkt->ad_len;
	dev_data->aad_index = 0u;
	dev_data->input_buffer = pkt->pkt->in_buf;
	dev_data->output_buffer = pkt->pkt->out_buf;
	dev_data->total_length = pkt->pkt->in_len;
	dev_data->buffer_index = 0u;
	dev_data->gcm_phase = (dev_data->aad_length > 0u)     ? GCM_PHASE_AAD
			      : (dev_data->total_length > 0u) ? GCM_PHASE_DATA
							      : GCM_PHASE_TAG;

	dev_data->processing = true;
	k_sem_take(&dev_data->done, K_NO_WAIT);

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	dev_data->dma_active = use_dma;
	if (use_dma) {
		/* DMA handles AAD/DATA feeding (and, for DATA, output draining)
		 * entirely on its own; only the final tag readout still needs the
		 * AES interrupt, so DATRDY (only needed for the manual path) is
		 * disabled again now that the key/H generation wait is done.
		 */
		regs->AES_IDR = AES_IDR_DATRDY_Msk;

		ret = crypto_mchp_aes_dma_gcm_start(ctx);
		if (ret != 0) {
			LOG_ERR("Unable to start the DMA: %i", ret);
			dev_data->processing = false;
			dev_data->dma_active = false;
			k_mutex_unlock(&dev_data->mutex);
			return ret;
		}
	} else {
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */
		/* Kickstart the operation */
		crypto_mchp_aes_gcm_advance(dev_data, regs);
#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	}
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

	/* If in synchronous operation block until done, otherwise just return */
	if (!dev_data->async) {
		k_sem_take(&dev_data->done, K_FOREVER);
		pkt->pkt->out_len = dev_data->buffer_index;
		if (dev_data->tag_mismatch) {
			ret = -EBADMSG;
		}
	}

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

/*
 * Handler for ECB mode
 */
static int crypto_mchp_aes_ecb_handler(struct cipher_ctx *ctx, struct cipher_pkt *pkt)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;
	int ret = 0;

	ret = crypto_mchp_aes_verify_context_and_packet(ctx, pkt);
	if (ret != 0) {
		return ret;
	}

	if ((pkt->in_len % BLOCK_SIZE) != 0u) {
		LOG_ERR("Invalid ECB op: in_len=%zu", pkt->in_len);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->last_pkt = pkt;
	ret = crypto_mchp_aes_process(ctx, pkt->in_len, pkt->in_buf, &pkt->out_len, pkt->out_buf,
				      NULL);

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

/*
 * Handler for CBC mode
 */
static int crypto_mchp_aes_cbc_handler(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;
	int32_t in_len = pkt->in_len;
	uint8_t *in_buf = pkt->in_buf;
	uint8_t *out_buf = pkt->out_buf;
	int ret = 0;

	ret = crypto_mchp_aes_verify_context_and_packet(ctx, pkt);
	if (ret != 0) {
		return ret;
	}

	if ((pkt->in_len % BLOCK_SIZE) != 0u) {
		LOG_ERR("Invalid CBC op: in_len=%zu", pkt->in_len);
		return -EINVAL;
	}

	if (dev_data->dir == CRYPTO_CIPHER_OP_ENCRYPT) {
		if ((ctx->flags & CAP_NO_IV_PREFIX) == 0u) {
			if (pkt->in_len > pkt->out_buf_max - (int)BLOCK_SIZE) {
				LOG_ERR("Short of output buffer.");
				return -ENOSR;
			}

			/* CBC prepends IV to output */
			memcpy(out_buf, iv, BLOCK_SIZE);
			out_buf += BLOCK_SIZE;
		} else {
			if (pkt->in_len > pkt->out_buf_max) {
				LOG_ERR("Short of output buffer.");
				return -ENOSR;
			}
		}
	} else {
		if (pkt->in_len < (int)BLOCK_SIZE) {
			LOG_ERR("Invalid CBC op: in_len=%d", pkt->in_len);
			return -EINVAL;
		}

		if ((ctx->flags & CAP_NO_IV_PREFIX) == 0u) {
			if ((pkt->in_len - (int)BLOCK_SIZE) > pkt->out_buf_max) {
				LOG_ERR("Short of output buffer.");
				return -ENOSR;
			}

			/* CBC prepends IV to output, so the real input starts after 1 block */
			in_buf += BLOCK_SIZE;
			in_len -= BLOCK_SIZE;
		} else {
			if (pkt->in_len > pkt->out_buf_max) {
				LOG_ERR("Short of output buffer.");
				return -ENOSR;
			}
		}
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->last_pkt = pkt;
	ret = crypto_mchp_aes_process(ctx, in_len, in_buf, &pkt->out_len, out_buf, iv);

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

/*
 * Handler for CTR mode
 */
static int crypto_mchp_aes_ctr_handler(struct cipher_ctx *ctx, struct cipher_pkt *pkt, uint8_t *iv)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;
	uint32_t ctr_len_bits = ctx->mode_params.ctr_info.ctr_len;
	uint8_t counter_blk[BLOCK_SIZE] = {0u};
	int ret = 0;

	ret = crypto_mchp_aes_verify_context_and_packet(ctx, pkt);
	if (ret != 0) {
		return ret;
	}

	if (ctr_len_bits == 0u || (ctr_len_bits % BITS_PER_BYTE) != 0u || ctr_len_bits > 128u) {
		LOG_ERR("Invalid CTR counter length: %u bits", ctr_len_bits);
		return -EINVAL;
	}

	/* In CTR mode part of the IV is a counter */
	memset(counter_blk, 0, sizeof(counter_blk));
	memcpy(counter_blk, iv, BLOCK_SIZE - (ctr_len_bits / BITS_PER_BYTE));

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->last_pkt = pkt;
	ret = crypto_mchp_aes_process(ctx, pkt->in_len, pkt->in_buf, &pkt->out_len, pkt->out_buf,
				      counter_blk);

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

/*
 * Handler for CFB and OFB modes
 */
static int crypto_mchp_aes_cfb_ofb_handler(struct cipher_ctx *ctx, struct cipher_pkt *pkt,
					   uint8_t *iv)
{
	struct crypto_mchp_aes_data *dev_data = ctx->drv_sessn_state;
	int ret = 0;

	ret = crypto_mchp_aes_verify_context_and_packet(ctx, pkt);
	if (ret != 0) {
		return ret;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	dev_data->last_pkt = pkt;
	ret = crypto_mchp_aes_process(ctx, pkt->in_len, pkt->in_buf, &pkt->out_len, pkt->out_buf,
				      iv);

	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int crypto_mchp_aes_cipher_begin_session(const struct device *dev, struct cipher_ctx *ctx,
						enum cipher_algo algo, enum cipher_mode mode,
						enum cipher_op optype)
{
	struct crypto_mchp_aes_data *dev_data = dev->data;
	int ret = 0;

	if ((ctx->flags & CAP_INPLACE_OPS) > 0) {
		LOG_ERR("Does not support in place operation");
		return -ENOTSUP;
	}

	if (algo != CRYPTO_CIPHER_ALGO_AES) {
		LOG_ERR("Unsupported algorithm: %d", algo);
		return -ENOTSUP;
	}

	if (mode != CRYPTO_CIPHER_MODE_ECB && mode != CRYPTO_CIPHER_MODE_CBC &&
	    mode != CRYPTO_CIPHER_MODE_CTR && mode != CRYPTO_CIPHER_MODE_GCM &&
	    mode != CRYPTO_CIPHER_MODE_CFB && mode != CRYPTO_CIPHER_MODE_OFB) {
		LOG_ERR("Unsupported mode: %d", mode);
		return -ENOTSUP;
	}

	if (ctx->key.bit_stream == NULL) {
		LOG_ERR("No key provided");
		return -EINVAL;
	}

	if (ctx->keylen != 16u && ctx->keylen != 24u && ctx->keylen != 32u) {
		LOG_ERR("Invalid key length: %zu", ctx->keylen);
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	if (dev_data->session_active) {
		LOG_ERR("Only one session at a time is allowed");
		ret = -EALREADY;
		goto error_cleanup;
	}

	ctx->device = dev;
	ctx->drv_sessn_state = dev_data;
	ctx->ops.cipher_mode = mode;

	if (ctx->flags & CAP_ASYNC_OPS) {
		dev_data->async = true;
	} else {
		dev_data->async = false;
	}

	dev_data->dir = optype;
	dev_data->key_len = ctx->keylen;
	dev_data->mode = mode;
	memcpy(dev_data->key, ctx->key.bit_stream, ctx->keylen);

	switch (mode) {
	case CRYPTO_CIPHER_MODE_ECB:
		ctx->ops.block_crypt_hndlr = crypto_mchp_aes_ecb_handler;
		break;
	case CRYPTO_CIPHER_MODE_CBC:
		ctx->ops.cbc_crypt_hndlr = crypto_mchp_aes_cbc_handler;
		break;
	case CRYPTO_CIPHER_MODE_CTR:
		ctx->ops.ctr_crypt_hndlr = crypto_mchp_aes_ctr_handler;
		break;
	case CRYPTO_CIPHER_MODE_GCM:
		ctx->ops.gcm_crypt_hndlr = crypto_mchp_aes_gcm_handler;
		break;
	case CRYPTO_CIPHER_MODE_CFB:
		ctx->ops.cfb_crypt_hndlr = crypto_mchp_aes_cfb_ofb_handler;
		break;
	case CRYPTO_CIPHER_MODE_OFB:
		ctx->ops.ofb_crypt_hndlr = crypto_mchp_aes_cfb_ofb_handler;
		break;
	default:
		/* Checked above */
		break;
	}

	dev_data->session_active = true;

	LOG_DBG("Session started: mode=%d, op=%d, keylen=%zu", mode, dev_data->dir,
		dev_data->key_len);

error_cleanup:
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static inline int crypto_mchp_aes_cipher_free_session(const struct device *dev,
						      struct cipher_ctx *ctx)
{
	struct crypto_mchp_aes_data *dev_data = dev->data;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	if (!dev_data->session_active) {
		LOG_WRN("No session was initialized");
		ret = -EALREADY;
		goto error_cleanup;
	}

	memset(dev_data->key, 0u, sizeof(dev_data->key));
	dev_data->session_active = false;

	ctx->device = NULL;
	ctx->drv_sessn_state = NULL;

	LOG_DBG("Session freed");

error_cleanup:
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

int crypto_mchp_aes_cipher_async_callback_set(const struct device *dev, cipher_completion_cb cb)
{
	struct crypto_mchp_aes_data *dev_data = dev->data;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);
	dev_data->completion_cb = cb;
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int crypto_mchp_aes_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return CAP_RAW_KEY | CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_ASYNC_OPS | CAP_NO_IV_PREFIX;
}

static DEVICE_API(crypto, crypto_mchp_aes_driver_api) = {
	.cipher_begin_session = crypto_mchp_aes_cipher_begin_session,
	.cipher_free_session = crypto_mchp_aes_cipher_free_session,
	.cipher_async_callback_set = crypto_mchp_aes_cipher_async_callback_set,
	.query_hw_caps = crypto_mchp_aes_query_hw_caps,
};

static int crypto_mchp_aes_init(const struct device *dev)
{
	const struct crypto_mchp_aes_config *dev_config = dev->config;
	struct crypto_mchp_aes_data *dev_data = dev->data;
	int ret = 0;

	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Unable to activate peripheral clock");
		return ret;
	}

	memset(dev_data, 0u, sizeof(struct crypto_mchp_aes_data));

	dev_config->irq_config(dev);

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	if (!device_is_ready(dev_config->dma.dev)) {
		return -ENODEV;
	}
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

	ret = k_mutex_init(&dev_data->mutex);
	if (ret != 0) {
		LOG_ERR("Unable to initialize mutex: %i", ret);
		return ret;
	}

	ret = k_sem_init(&dev_data->done, 0, 1);
	if (ret != 0) {
		LOG_ERR("Unable to initialize semaphore: %i", ret);
		return ret;
	}

	return 0;
}

/*
 * Terminate the operation
 */
static void crypto_mchp_aes_finish(struct crypto_mchp_aes_data *dev_data,
				   const struct crypto_mchp_aes_config *dev_config, int status)
{
	aes_registers_t *const regs = dev_config->regs;

	dev_data->processing = false;

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
	if (dev_data->dma_active) {
		dma_stop(dev_config->dma.dev, dev_config->dma.tx_channel);
		dma_stop(dev_config->dma.dev, dev_config->dma.rx_channel);
		dev_data->dma_active = false;
	}
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

	k_sem_give(&dev_data->done);

	/* Disable the interrupts, we only use them for synchronization */
	regs->AES_IDR = AES_IDR_DATRDY_Msk | AES_IDR_TAGRDY_Msk;

	/* Invoke the application callback, if provided */
	if (dev_data->completion_cb) {
		dev_data->completion_cb(dev_data->last_pkt, status);
	}
}

/*
 * Handle interrupts for GCM mode of operation
 */
static void crypto_mchp_aes_isr_gcm(struct crypto_mchp_aes_data *dev_data,
				    const struct crypto_mchp_aes_config *dev_config, uint32_t isr)
{
	aes_registers_t *const regs = dev_config->regs;
	uint8_t tag[BLOCK_SIZE] = {0u};
	/* AAD blocks produce no output; only the data phase does */
	uint8_t block[BLOCK_SIZE] = {0u};
	size_t remaining = 0u;

	if (dev_data->gcm_phase == GCM_PHASE_TAG) {
		if ((isr & AES_ISR_TAGRDY_Msk) == 0) {
			return;
		}

		crypto_mchp_aes_get_tag(regs, tag);

		if (dev_data->dir == CRYPTO_CIPHER_OP_ENCRYPT) {
			memcpy(dev_data->aead_pkt->tag, tag, dev_data->tag_len);
		} else {
			/* Constant time comparation to avoid timing attacks */
			for (size_t idx = 0u; idx < BLOCK_SIZE; idx++) {
				if (idx < dev_data->tag_len) {
					if (dev_data->aead_pkt->tag[idx] != tag[idx]) {
						dev_data->tag_mismatch = true;
					}
				}
			}
		}

		dev_data->last_pkt->out_len = dev_data->buffer_index;
		crypto_mchp_aes_finish(dev_data, dev_config, dev_data->tag_mismatch ? -EBADMSG : 0);
		return;
	}

	/* GCM_PHASE_AAD / GCM_PHASE_DATA: one DATRDY per block written */
	if ((isr & AES_ISR_DATRDY_Msk) == 0) {
		return;
	}

	if (dev_data->gcm_phase == GCM_PHASE_DATA) {
		remaining = dev_data->total_length - dev_data->buffer_index;

		crypto_mchp_aes_get_output_data(regs, block);
		memcpy(&dev_data->output_buffer[dev_data->buffer_index], block,
		       MIN(remaining, BLOCK_SIZE));
		dev_data->buffer_index += BLOCK_SIZE;
	} else {
		dev_data->aad_index += BLOCK_SIZE;
	}

	crypto_mchp_aes_gcm_advance(dev_data, regs);
}

static void crypto_mchp_aes_isr(const struct device *dev)
{
	struct crypto_mchp_aes_data *dev_data = dev->data;
	const struct crypto_mchp_aes_config *dev_config = dev->config;
	aes_registers_t *const regs = dev_config->regs;
	uint32_t isr = regs->AES_ISR;
	bool woken = false;
	uint8_t dummy[BLOCK_SIZE] = {0u};
	uint8_t block[BLOCK_SIZE] = {0u};
	size_t remaining = 0u;
	size_t next_remaining = 0u;

	if (!dev_data->processing) {
		/* One-off synchronous wait, e.g. GCM hash subkey generation. Per the
		 * datasheet, neither DATRDY nor TAGRDY clears on a plain AES_ISR
		 * read: DATRDY only clears via START/SWRST or reading AES_ODATARx,
		 * TAGRDY only via reading the tag, starting new processing, or
		 * writing a new key. Left set, the still-enabled interrupt would
		 * retrigger indefinitely and the waiting thread would never get to
		 * run. Acknowledge with the documented dummy read, which has no
		 * side effect on the computed subkey/tag.
		 */
		if ((isr & AES_ISR_DATRDY_Msk) != 0) {
			crypto_mchp_aes_get_output_data(regs, dummy);
			woken = true;
		}

		if ((isr & AES_ISR_TAGRDY_Msk) != 0) {
			crypto_mchp_aes_get_tag(regs, dummy);
			woken = true;
		}

		if (woken) {
			k_sem_give(&dev_data->done);
		}
		return;
	}

	if (dev_data->mode == CRYPTO_CIPHER_MODE_GCM) {
		crypto_mchp_aes_isr_gcm(dev_data, dev_config, isr);
		return;
	}

	if ((isr & AES_ISR_DATRDY_Msk) != 0) {
		remaining = dev_data->total_length - dev_data->buffer_index;

		/* Get the output */
		crypto_mchp_aes_get_output_data(regs, block);
		memcpy(&dev_data->output_buffer[dev_data->buffer_index], block,
		       MIN(remaining, BLOCK_SIZE));

		/* Advance cursor */
		dev_data->buffer_index += BLOCK_SIZE;

		/* Process more data. Note: this ISR is never reached for a DMA-driven
		 * operation, since AES_IER.DATRDY is only enabled on the manual path;
		 * DMA completion is instead signaled by crypto_mchp_aes_dma_done_callback().
		 */
		if (dev_data->buffer_index < dev_data->total_length) {
			next_remaining = dev_data->total_length - dev_data->buffer_index;

			/* Prepare the input registers, with padding if necessary */
			if (next_remaining < BLOCK_SIZE) {
				crypto_mchp_aes_build_padded_block(
					block, &dev_data->input_buffer[dev_data->buffer_index],
					next_remaining);
				crypto_mchp_aes_set_input_data(regs, block);
			} else {
				crypto_mchp_aes_set_input_data(
					regs, &dev_data->input_buffer[dev_data->buffer_index]);
			}

			/* Start operation again */
			regs->AES_CR = AES_CR_START_Msk;
		}
		/* or signal the operation is finished */
		else {
			dev_data->last_pkt->out_len = dev_data->total_length;
			crypto_mchp_aes_finish(dev_data, dev_config, 0);
		}
	}
}

#if defined(CONFIG_CRYPTO_MCHP_AES_G2_DMA)
#define DMA_CONFIG_INIT(n)                                                                         \
	.dma = {                                                                                   \
		.dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),                            \
		.tx_channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),                           \
		.tx_perid = DT_INST_DMAS_CELL_BY_NAME(n, tx, perid),                               \
		.rx_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),                           \
		.rx_perid = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),                               \
	},
#else
#define DMA_CONFIG_INIT(n)
#endif /* CONFIG_CRYPTO_MCHP_AES_G2_DMA */

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), crypto_mchp_aes_isr,        \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct crypto_mchp_aes_config mchp_aes_##n##_config = {                       \
		.regs = (aes_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),         \
		.irq_config = irq_config_##n,                                                      \
		DMA_CONFIG_INIT(n)};                                                               \
                                                                                                   \
	static struct crypto_mchp_aes_data mchp_aes_##n##_data = {};                               \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, crypto_mchp_aes_init, NULL, &mchp_aes_##n##_data,                 \
			      &mchp_aes_##n##_config, POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,    \
			      &crypto_mchp_aes_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
