/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/** @file
 * @brief Microchip Integrity Check Monitor (ICM) crypto driver.
 *
 */

#define DT_DRV_COMPAT microchip_icm_g1_crypto

#include <zephyr/crypto/crypto.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(crypto_mchp_icm_g1, CONFIG_CRYPTO_LOG_LEVEL);

#define REGION_DESCRIPTOR_SIZE     4u
#define REGION_DESCRIPTOR_RADDR    0u
#define REGION_DESCRIPTOR_RCFG     1u
#define REGION_DESCRIPTOR_RCFG_EOM 2u
#define REGION_DESCRIPTOR_RCTRL    2u
#define REGION_DESCRIPTOR_RNEXT    3u

#define BLOCK_SIZE 64u

struct region_descriptor {
	uint32_t registers[REGION_DESCRIPTOR_SIZE] __aligned(64);
};

struct crypto_mchp_icm_config {
	/* Register block */
	icm_registers_t *regs;

	/* Clock driver */
	const struct device *clock;

	/* Main clock subsystem. */
	clock_control_subsys_t clock_subsys;

	/* IRQ setup function */
	void (*irq_config)(const struct device *dev);
};

struct crypto_mchp_icm_data {
	/* Driver mutex */
	struct k_mutex mutex;

	/* Driver interrupt semaphore */
	struct k_sem done;

	/* Index of the next available descriptor */
	size_t descriptor_index;

	/* Pool of region descriptors */
	struct region_descriptor descriptors[CONFIG_CRYPTO_MCHP_ICM_G1_MAX_DESCRIPTORS];

	/* Padding block */
	uint8_t padding[BLOCK_SIZE];

	/* Length in bits of the provided input */
	size_t input_length;

	/* Completion callback */
	hash_completion_cb cb;

	/* Last packet */
	struct hash_pkt *last_pkt;

	/* Only supports one session at a time */
	bool session_active: 1;

	/* Whether the session is asynchronous */
	bool async: 1;
};

/*
 * Add a data block to the region sequence to be hashed
 */
static int crypto_mchp_icm_add_region(struct crypto_mchp_icm_data *dev_data, size_t input_length,
				      const uint8_t *input)
{
	struct region_descriptor *previous_descriptor = NULL;
	struct region_descriptor *descriptor = NULL;

	if (dev_data->descriptor_index < CONFIG_CRYPTO_MCHP_ICM_G1_MAX_DESCRIPTORS) {
		descriptor = &dev_data->descriptors[dev_data->descriptor_index];
	} else {
		LOG_ERR("Unable to allocate region descriptor");
		return -ENOMEM;
	}

	/* If the next isn't the first descriptor set the previous to point to this one */
	if (dev_data->descriptor_index > 0) {
		previous_descriptor = &dev_data->descriptors[dev_data->descriptor_index - 1];
		previous_descriptor->registers[REGION_DESCRIPTOR_RNEXT] =
			(uint32_t)(uintptr_t)descriptor;
	}

	descriptor = &dev_data->descriptors[dev_data->descriptor_index];
	dev_data->descriptor_index++;

	descriptor->registers[REGION_DESCRIPTOR_RADDR] = (uint32_t)(uintptr_t)input;
	/* Only really effective for the first descriptor */
	descriptor->registers[REGION_DESCRIPTOR_RCFG] = 1u << REGION_DESCRIPTOR_RCFG_EOM;
	descriptor->registers[REGION_DESCRIPTOR_RCTRL] = (input_length / BLOCK_SIZE) - 1;
	/* Prehemptively mark it as last */
	descriptor->registers[REGION_DESCRIPTOR_RNEXT] = 0u;

	return 0;
}

/*
 * Hashing function handler
 */
static int crypto_mchp_icm_hash_handler(struct hash_ctx *ctx, struct hash_pkt *pkt, bool finish)
{
	struct crypto_mchp_icm_data *dev_data = ctx->device->data;
	const struct crypto_mchp_icm_config *dev_config = ctx->device->config;
	icm_registers_t *const regs = dev_config->regs;
	uint32_t start_address = 0;
	int ret = 0;

	start_address = (uint32_t)(uintptr_t)pkt->in_buf;

	if ((start_address & 0x3Fu) != 0u) {
		LOG_ERR("Buffer address must be aligned to 64 bytes");
		return -ENOTSUP;
	}

	if ((pkt->in_len % BLOCK_SIZE) != 0u || pkt->in_len == 0u) {
		LOG_ERR("Buffer size must be multiple of 64 bytes");
		return -ENOTSUP;
	}

	if ((pkt->in_len / BLOCK_SIZE) + 1 > 0xFFFFu) {
		LOG_ERR("Buffer size cannot exceed 0xFFFF blocks of 64 bytes");
		return -ENOTSUP;
	}

	if (pkt->out_buf != NULL && !finish) {
		LOG_ERR("Driver does not support intermediate digests");
		return -ENOTSUP;
	}

	if (pkt->out_buf != NULL && ((uint32_t)(uintptr_t)pkt->out_buf & 0x7Fu) != 0) {
		LOG_ERR("Output buffer must be aligned to 128 bytes");
		return -ENOTSUP;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	ret = crypto_mchp_icm_add_region(dev_data, pkt->in_len, pkt->in_buf);
	if (ret != 0) {
		LOG_ERR("Could not add region descriptor: %i", ret);
		goto error_cleanup;
	}
	dev_data->input_length += pkt->in_len * 8u;

	/* Last block, add padding */
	if (finish) {
		memset(dev_data->padding, 0, sizeof(dev_data->padding));
		dev_data->padding[0] = 0x80;
		sys_put_be32(dev_data->input_length, &dev_data->padding[BLOCK_SIZE - 4]);

		ret = crypto_mchp_icm_add_region(dev_data, BLOCK_SIZE, dev_data->padding);
		if (ret != 0) {
			LOG_ERR("Could not add region descriptor: %i", ret);
			goto error_cleanup;
		}

		dev_data->last_pkt = pkt;

		/* clear the semaphore */
		k_sem_take(&dev_data->done, K_NO_WAIT);

		/* Set the output buffer */
		regs->ICM_HASH = (uint32_t)(uintptr_t)pkt->out_buf;
		/* Enable the peripheral, starting the calculation */

		regs->ICM_CTRL = ICM_CTRL_ENABLE_Msk;

		if (!dev_data->async) {
			/* Wait for the calculation to be complete */
			k_sem_take(&dev_data->done, K_FOREVER);
		}

		/* Finished, reset session */
		dev_data->input_length = 0u;
		dev_data->descriptor_index = 0u;
	}

error_cleanup:
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int crypto_mchp_icm_hash_begin_session(const struct device *dev, struct hash_ctx *ctx,
					      enum hash_algo algo)
{
	struct crypto_mchp_icm_data *dev_data = dev->data;
	const struct crypto_mchp_icm_config *dev_config = dev->config;
	icm_registers_t *const regs = dev_config->regs;
	uint32_t cfg = 0u;
	int ret = 0;

	if ((ctx->flags & CAP_INPLACE_OPS) != 0u) {
		LOG_ERR("Does not support in place operation");
		return -ENOTSUP;
	}

	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA224:
	case CRYPTO_HASH_ALGO_SHA256:
		/* Supported, Ok */
		break;
	default:
		LOG_ERR("Hashing algorithm unsupported");
		return -ENOTSUP;
	}

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	if (dev_data->session_active) {
		LOG_ERR("Only one session at a time is allowed");
		ret = -EALREADY;
		goto error_cleanup;
	}

	ctx->device = dev;
	ctx->started = false;
	ctx->drv_sessn_state = dev_data;
	ctx->hash_hndlr = crypto_mchp_icm_hash_handler;

	if (ctx->flags & CAP_ASYNC_OPS) {
		dev_data->async = true;
	} else {
		dev_data->async = false;
	}

	cfg = regs->ICM_CFG & ~ICM_CFG_UALGO_Msk & ~ICM_CFG_ASCD_Msk & ~ICM_CFG_WBDIS_Msk &
	      ~ICM_CFG_UIHASH_Msk;

	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA224:
		cfg |= ICM_CFG_UALGO_SHA224;
		break;
	case CRYPTO_HASH_ALGO_SHA256:
		cfg |= ICM_CFG_UALGO_SHA256;
		break;
	default:
		/* Checked above */
		break;
	}

	cfg |= ICM_CFG_UIHASH_Msk;

	regs->ICM_CFG = cfg;

	/* Setup the initial values for the different hashing algorithms */
	switch (algo) {
	case CRYPTO_HASH_ALGO_SHA224:
		regs->ICM_UIHVAL[0] = sys_cpu_to_be32(0xC1059ED8);
		regs->ICM_UIHVAL[1] = sys_cpu_to_be32(0x367CD507);
		regs->ICM_UIHVAL[2] = sys_cpu_to_be32(0x3070DD17);
		regs->ICM_UIHVAL[3] = sys_cpu_to_be32(0xF70E5939);
		regs->ICM_UIHVAL[4] = sys_cpu_to_be32(0xFFC00B31);
		regs->ICM_UIHVAL[5] = sys_cpu_to_be32(0x68581511);
		regs->ICM_UIHVAL[6] = sys_cpu_to_be32(0x64F98FA7);
		regs->ICM_UIHVAL[7] = sys_cpu_to_be32(0xBEFA4FA4);
		break;
	case CRYPTO_HASH_ALGO_SHA256:
		regs->ICM_UIHVAL[0] = sys_cpu_to_be32(0x6A09E667);
		regs->ICM_UIHVAL[1] = sys_cpu_to_be32(0xBB67AE85);
		regs->ICM_UIHVAL[2] = sys_cpu_to_be32(0x3C6EF372);
		regs->ICM_UIHVAL[3] = sys_cpu_to_be32(0xA54FF53A);
		regs->ICM_UIHVAL[4] = sys_cpu_to_be32(0x510E527F);
		regs->ICM_UIHVAL[5] = sys_cpu_to_be32(0x9B05688C);
		regs->ICM_UIHVAL[6] = sys_cpu_to_be32(0x1F83D9AB);
		regs->ICM_UIHVAL[7] = sys_cpu_to_be32(0x5BE0CD19);
		break;
	default:
		/* Checked above */
		break;
	}

	/* Clear session */
	dev_data->input_length = 0u;
	dev_data->descriptor_index = 0u;
	/* Select the first descriptor as starting point */
	regs->ICM_DSCR = (uint32_t)(uintptr_t)&dev_data->descriptors[0];

	dev_data->session_active = true;

error_cleanup:
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static inline int crypto_mchp_icm_hash_free_session(const struct device *dev, struct hash_ctx *ctx)
{
	ARG_UNUSED(ctx);

	struct crypto_mchp_icm_data *dev_data = dev->data;
	int ret = 0;

	k_mutex_lock(&dev_data->mutex, K_FOREVER);

	if (!dev_data->session_active) {
		LOG_WRN("No session was initialized");
		ret = -EALREADY;
		goto error_cleanup;
	}

	dev_data->session_active = false;

error_cleanup:
	k_mutex_unlock(&dev_data->mutex);

	return ret;
}

static int crypto_mchp_icm_query_hw_caps(const struct device *dev)
{
	ARG_UNUSED(dev);

	return CAP_SEPARATE_IO_BUFS | CAP_SYNC_OPS | CAP_ASYNC_OPS;
}

static int crypto_mchp_icm_hash_async_callback_set(const struct device *dev, hash_completion_cb cb)
{
	struct crypto_mchp_icm_data *dev_data = dev->data;

	dev_data->cb = cb;

	return 0;
}

static DEVICE_API(crypto, crypto_mchp_icm_driver_api) = {
	.hash_begin_session = crypto_mchp_icm_hash_begin_session,
	.hash_free_session = crypto_mchp_icm_hash_free_session,
	.query_hw_caps = crypto_mchp_icm_query_hw_caps,
	.hash_async_callback_set = crypto_mchp_icm_hash_async_callback_set,
};

static int crypto_mchp_icm_init(const struct device *dev)
{
	struct crypto_mchp_icm_data *dev_data = dev->data;
	const struct crypto_mchp_icm_config *dev_config = dev->config;
	icm_registers_t *const regs = dev_config->regs;
	int ret = 0;

	/* Enable peripheral clock */
	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_ERR("Unable to activate peripheral clock");
		return ret;
	}

	/* Disable all interrupts but the digest complete of the first region */
	regs->ICM_IDR = ~ICM_IDR_RHC(1);
	regs->ICM_IER = ICM_IER_RHC(1);

	memset(dev_data, 0u, sizeof(struct crypto_mchp_icm_data));

	dev_config->irq_config(dev);

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

static void crypto_mchp_icm_isr(const struct device *dev)
{
	struct crypto_mchp_icm_data *dev_data = dev->data;
	const struct crypto_mchp_icm_config *dev_config = dev->config;
	icm_registers_t *const regs = dev_config->regs;

	/* Hash completed */
	if ((regs->ICM_ISR & ICM_ISR_RHC(1)) != 0u) {
		regs->ICM_CTRL = ICM_CTRL_DISABLE_Msk;
		k_sem_give(&dev_data->done);

		if (dev_data->cb != NULL) {
			dev_data->cb(dev_data->last_pkt, 0);
		}
	}
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
                                                                                                   \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), crypto_mchp_icm_isr,        \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct crypto_mchp_icm_config mchp_icm_##n##_config = {                       \
		.regs = (icm_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),         \
		.irq_config = irq_config_##n,                                                      \
	};                                                                                         \
                                                                                                   \
	static struct crypto_mchp_icm_data mchp_icm_##n##_data = {};                               \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, crypto_mchp_icm_init, NULL, &mchp_icm_##n##_data,                 \
			      &mchp_icm_##n##_config, POST_KERNEL, CONFIG_CRYPTO_INIT_PRIORITY,    \
			      &crypto_mchp_icm_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
