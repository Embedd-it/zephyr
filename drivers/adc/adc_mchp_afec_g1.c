/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file adc_mchp_afec_g1.c
 * @brief Microchip AFEC (ADC) controller driver.
 */

#define DT_DRV_COMPAT microchip_afec_g1_adc

#include <soc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <errno.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/init.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#if defined(CONFIG_ADC_MCHP_AFEC_G1_ENABLE_DMA)
#include <zephyr/drivers/dma.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_mchp_afec_g1, CONFIG_ADC_LOG_LEVEL);

#define NUM_CHANNELS              12
#define CONF_ADC_PRESCALER(clkin) (((clkin) / 15000000) - 1)
#ifndef AFEC_MR_ONE_Msk
#define AFEC_MR_ONE_Msk AFEC_MR_ANACH
#endif

typedef void (*cfg_func_t)(const struct device *dev);

#if defined(CONFIG_ADC_MCHP_AFEC_G1_ENABLE_DMA) && defined(CONFIG_DCACHE) &&                       \
	defined(CONFIG_DCACHE_LINE_SIZE)
#define MCHP_AFEC_G1_CACHE_ATTR __aligned(CONFIG_DCACHE_LINE_SIZE)
#define MCHP_AFEC_G1_CACHE_NUM_CHANNELS                                                            \
	((ROUND_UP((NUM_CHANNELS * sizeof(uint32_t)), CONFIG_DCACHE_LINE_SIZE)) /                  \
	 (sizeof(uint32_t)))
#else
#define MCHP_AFEC_G1_CACHE_ATTR
#define MCHP_AFEC_G1_CACHE_NUM_CHANNELS NUM_CHANNELS
#endif

struct adc_mchp_afec_data {
	struct adc_context ctx;
	const struct device *dev;

	/* Pointer to the buffer in the sequence. */
	uint16_t *buffer;

	/* Pointer to the beginning of a sample. Consider the number of
	 * channels in the sequence: this buffer changes by that amount
	 * so all the channels would get repeated.
	 */
	uint16_t *repeat_buffer;

	/* Bit mask of the channels to be sampled. */
	uint32_t channels;

	/* Index of the channel being sampled. */
	uint8_t channel_id;

	/* Number of active channels. */
	uint8_t channels_count;

#if defined(CONFIG_ADC_MCHP_AFEC_G1_ENABLE_DMA)
	/* Temporary storage for samples captured by DMA. */
	uint32_t dma_results[MCHP_AFEC_G1_CACHE_NUM_CHANNELS] MCHP_AFEC_G1_CACHE_ATTR;
#endif
};

#if defined(CONFIG_ADC_MCHP_AFEC_G1_ENABLE_DMA)
struct adc_mchp_afec_dma {
	const struct device *dma_dev;
	uint8_t dma_request;
	uint8_t dma_channel;
};
#endif

struct adc_mchp_afec_config {
	afec_registers_t *regs;
	cfg_func_t cfg_func;
	const struct device *clock;
	clock_control_subsys_t clock_cfg;
	const struct pinctrl_dev_config *pcfg;
#if defined(CONFIG_ADC_MCHP_AFEC_G1_ENABLE_DMA)
	struct adc_mchp_afec_dma adc_dma;
#endif
};

static int adc_mchp_afec_channel_setup(const struct device *dev,
				       const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_mchp_afec_config *const dev_config = dev->config;
	afec_registers_t *const regs = dev_config->regs;
	uint8_t channel_id = channel_cfg->channel_id;

	if (channel_id >= NUM_CHANNELS) {
		return -EINVAL;
	}

	/* Clear the gain bits for the channel. */
	regs->AFEC_CGR &= ~(3 << channel_id * 2u);

	switch (channel_cfg->gain) {
	case ADC_GAIN_1:
		/* A value of 0 in this register is a gain of 1. */
		break;
	case ADC_GAIN_1_2:
		regs->AFEC_CGR |= (1 << (channel_id * 2u));
		break;
	case ADC_GAIN_1_4:
		regs->AFEC_CGR |= (2 << (channel_id * 2u));
		break;
	default:
		LOG_ERR("Selected ADC gain is not valid");
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Selected ADC acquisition time is not valid");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_EXTERNAL0) {
		LOG_ERR("Selected reference is not valid");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential input is not supported");
		return -EINVAL;
	}

	/* Set single ended channels to unsigned and differential channels
	 * to signed conversions.
	 */
	regs->AFEC_EMR &= ~(AFEC_EMR_SIGNMODE(AFEC_EMR_SIGNMODE_SE_UNSG_DF_SIGN_Val));

	return 0;
}

static void adc_mchp_afec_start_conversion(const struct device *dev)
{
	const struct adc_mchp_afec_config *const dev_config = dev->config;
	struct adc_mchp_afec_data *dev_data = dev->data;
	afec_registers_t *const regs = dev_config->regs;

	dev_data->channel_id = find_lsb_set(dev_data->channels) - 1;

	LOG_DBG("Starting channel %d", dev_data->channel_id);

	/* Disable all channels. */
	regs->AFEC_CHDR = 0xfff;
	regs->AFEC_IDR = 0xfff;

	/* Enable the ADC channel. This also enables/selects the channel pin as
	 * an input to the AFEC.
	 */
	regs->AFEC_CHER = (1 << dev_data->channel_id);

	/* Enable the interrupt for the channel. */
	regs->AFEC_IER = (1 << dev_data->channel_id);

	/* Start the conversions. */
	regs->AFEC_CR = AFEC_CR_START_Msk;
}

#if defined(CONFIG_ADC_MCHP_AFEC_G1_ENABLE_DMA)
static void adc_mchp_afec_dma_read_done(const struct device *dma_dev, void *arg, uint32_t id,
					int error_code)
{
	const struct device *dev = (struct device *)arg;
	const struct adc_mchp_afec_config *const dev_config = dev->config;
	afec_registers_t *const regs = dev_config->regs;
	struct adc_mchp_afec_data *dev_data = dev->data;

	(void)dma_stop(dev_config->adc_dma.dma_dev, dev_config->adc_dma.dma_channel);
	if (error_code < 0) {
		regs->AFEC_CHDR = AFEC_CHDR_Msk;
		adc_context_complete(&dev_data->ctx, -EIO);
	} else {
		(void)sys_cache_data_invd_range(dev_data->dma_results,
						dev_data->channels_count * sizeof(uint32_t));
		for (size_t ch_idx = 0; ch_idx < dev_data->channels_count; ch_idx++) {
			*dev_data->buffer = (uint16_t)dev_data->dma_results[ch_idx];
			dev_data->buffer++;
		}
		adc_context_on_sampling_done(&dev_data->ctx, dev);
	}
}

static void adc_mchp_afec_dma_config_and_fire(const struct device *dev)
{
	const struct adc_mchp_afec_config *const dev_config = dev->config;
	afec_registers_t *const regs = dev_config->regs;
	struct adc_mchp_afec_data *dev_data = dev->data;
	struct dma_config dma_cfg = {0};
	struct dma_block_config dma_blk = {0};
	int ret;

	dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	dma_cfg.source_data_size = sizeof(uint32_t);
	dma_cfg.source_burst_length = sizeof(uint32_t);
	dma_cfg.dest_data_size = sizeof(uint32_t);
	dma_cfg.dest_burst_length = sizeof(uint32_t);
	dma_cfg.user_data = (void *)dev;
	dma_cfg.dma_callback = adc_mchp_afec_dma_read_done;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_blk;
	dma_cfg.dma_slot = dev_config->adc_dma.dma_request;

	dma_blk.block_size = dev_data->channels_count * sizeof(uint32_t);
	dma_blk.dest_address = (uint32_t)dev_data->dma_results;
	dma_blk.source_address = (uint32_t)(&(regs->AFEC_LCDR));
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;

	ret = dma_config(dev_config->adc_dma.dma_dev, dev_config->adc_dma.dma_channel, &dma_cfg);
	if (ret == 0) {
		ret = dma_start(dev_config->adc_dma.dma_dev, dev_config->adc_dma.dma_channel);
	}

	if (ret != 0) {
		LOG_ERR("DMA setup failed %d", ret);
		adc_context_complete(&dev_data->ctx, ret);
		return;
	}

	regs->AFEC_CHDR = AFEC_CHDR_Msk;
	regs->AFEC_CHER = dev_data->channels & AFEC_CHDR_Msk;
	regs->AFEC_CR = AFEC_CR_START_Msk;
}
#endif

/**
 * This is only called once at the beginning of all the conversions,
 * all channels as a group.
 */
static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_mchp_afec_data *dev_data = CONTAINER_OF(ctx, struct adc_mchp_afec_data, ctx);

	dev_data->channels = ctx->sequence.channels & GENMASK((NUM_CHANNELS)-1, 0);
	dev_data->channels_count = POPCOUNT(dev_data->channels);
	dev_data->repeat_buffer = dev_data->buffer;

#if defined(CONFIG_ADC_MCHP_AFEC_G1_ENABLE_DMA)
	adc_mchp_afec_dma_config_and_fire(dev_data->dev);
#else
	adc_mchp_afec_start_conversion(dev_data->dev);
#endif
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_mchp_afec_data *dev_data = CONTAINER_OF(ctx, struct adc_mchp_afec_data, ctx);

	if (repeat_sampling != 0) {
		dev_data->buffer = dev_data->repeat_buffer;
	}
}

static int adc_mchp_afec_check_buffer_size(const struct adc_sequence *sequence,
					   uint8_t active_channels)
{
	size_t needed_buffer_size;

	needed_buffer_size = active_channels * sizeof(uint16_t);
	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}
	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)", sequence->buffer_size,
			needed_buffer_size);
		return -ENOMEM;
	}
	return 0;
}

static int adc_mchp_afec_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_mchp_afec_data *dev_data = dev->data;
	uint8_t num_active_channels = 0u;
	uint32_t channels = sequence->channels;
	int ret = 0;

	dev_data->channels = 0u;

	/* Signal an error if the channel selection is invalid (no channels or
	 * a non-existing one is selected).
	 */
	if (channels == 0u || (channels & (~0ul << NUM_CHANNELS))) {
		LOG_ERR("Invalid selection of channels");
		return -EINVAL;
	}

	if (sequence->oversampling != 0u) {
		LOG_ERR("Oversampling is not supported");
		return -EINVAL;
	}

	if (sequence->resolution != 12u) {
		LOG_ERR("ADC resolution value %d is not valid", sequence->resolution);
		return -EINVAL;
	}

	num_active_channels = POPCOUNT(channels);

	ret = adc_mchp_afec_check_buffer_size(sequence, num_active_channels);
	if (ret != 0) {
		return ret;
	}

	/* In the context you have a pointer to the adc_mchp_afec_data
	 * structure only.
	 */
	dev_data->buffer = sequence->buffer;
	dev_data->repeat_buffer = sequence->buffer;

	/* At this point we allow the scheduler to do other things while
	 * we wait for the conversions to complete. This is provided by the
	 * adc_context functions. However, the caller of this function is
	 * blocked until the results are in.
	 */
	adc_context_start_read(&dev_data->ctx, sequence);

	ret = adc_context_wait_for_completion(&dev_data->ctx);
	return ret;
}

static int adc_mchp_afec_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_mchp_afec_data *dev_data = dev->data;
	int ret;

	adc_context_lock(&dev_data->ctx, false, NULL);
	ret = adc_mchp_afec_start_read(dev, sequence);
	adc_context_release(&dev_data->ctx, ret);

	return ret;
}

static int adc_mchp_afec_init(const struct device *dev)
{
	const struct adc_mchp_afec_config *const dev_config = dev->config;
	struct adc_mchp_afec_data *dev_data = dev->data;
	afec_registers_t *const regs = dev_config->regs;
	int ret;
	uint32_t clkin_freq;

	ret = clock_control_get_rate(dev_config->clock, dev_config->clock_cfg, &clkin_freq);
	if (ret != 0) {
		return ret;
	}

	/* Enable AFEC clock in PMC */
	ret = clock_control_on(dev_config->clock, dev_config->clock_cfg);
	if (ret < 0) {
		return -ENODEV;
	}

	/* Reset the AFEC. */
	regs->AFEC_CR = AFEC_CR_SWRST_Msk;

	regs->AFEC_MR = AFEC_MR_TRGEN_DIS | AFEC_MR_SLEEP_NORMAL | AFEC_MR_FWUP_OFF |
			AFEC_MR_FREERUN_OFF | AFEC_MR_PRESCAL(CONF_ADC_PRESCALER(clkin_freq)) |
			AFEC_MR_STARTUP_SUT96 | AFEC_MR_ONE_Msk | AFEC_MR_USEQ_NUM_ORDER;

	/* Set all channels CM voltage to Vrefp/2 (512). */
	for (size_t i = 0; i < NUM_CHANNELS; i++) {
		regs->AFEC_CSELR = i;
		regs->AFEC_COCR = 512;
	}

	/* Enable PGA and Current Bias. */
	regs->AFEC_ACR = AFEC_ACR_IBCTL(1) | AFEC_ACR_PGA0EN_Msk | AFEC_ACR_PGA1EN_Msk;

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	dev_config->cfg_func(dev);

	dev_data->dev = dev;

	adc_context_unlock_unconditionally(&dev_data->ctx);

	return ret;
}

#if defined(CONFIG_ADC_ASYNC)
static int adc_mchp_afec_read_async(const struct device *dev, const struct adc_sequence *sequence,
				    struct k_poll_signal *async)
{
	struct adc_mchp_afec_data *dev_data = dev->data;
	int ret;

	adc_context_lock(&dev_data->ctx, true, async);
	ret = adc_mchp_afec_start_read(dev, sequence);
	adc_context_release(&dev_data->ctx, ret);

	return ret;
}
#endif

static DEVICE_API(adc, adc_mchp_afec_driver_api) = {
	.channel_setup = adc_mchp_afec_channel_setup,
	.read = adc_mchp_afec_read,
#if defined(CONFIG_ADC_ASYNC)
	.read_async = adc_mchp_afec_read_async,
#endif
};

static void adc_mchp_afec_isr(const struct device *dev)
{
	struct adc_mchp_afec_data *dev_data = dev->data;
	const struct adc_mchp_afec_config *const dev_config = dev->config;
	afec_registers_t *const regs = dev_config->regs;
	uint16_t sample;

	regs->AFEC_CHDR = BIT(dev_data->channel_id);
	regs->AFEC_IDR = BIT(dev_data->channel_id);

	regs->AFEC_CSELR = AFEC_CSELR_CSEL(dev_data->channel_id);
	sample = (uint16_t)(regs->AFEC_CDR);

	*dev_data->buffer++ = sample;
	dev_data->channels &= ~BIT(dev_data->channel_id);

	if (dev_data->channels) {
		adc_mchp_afec_start_conversion(dev);
	} else {
		/* Called once all conversions have completed.*/
		adc_context_on_sampling_done(&dev_data->ctx, dev);
	}
}

#if defined(CONFIG_ADC_MCHP_AFEC_G1_ENABLE_DMA)
#define ADC_MCHP_AFEC_G1_DMA_CHANNEL(n)                                                            \
	.adc_dma.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, rx)),                        \
	.adc_dma.dma_request = DT_INST_DMAS_CELL_BY_NAME(n, rx, perid),                            \
	.adc_dma.dma_channel = DT_INST_DMAS_CELL_BY_NAME(n, rx, channel),
#else
#define ADC_MCHP_AFEC_G1_DMA_CHANNEL(n)
#endif

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static void mchp_afec_##n##_irq_config(const struct device *dev);                          \
                                                                                                   \
	static const struct adc_mchp_afec_config mchp_afec_##n##_config = {                        \
		.regs = (afec_registers_t *)DT_INST_REG_ADDR(n),                                   \
		.cfg_func = mchp_afec_##n##_irq_config,                                            \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_cfg = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, subsystem),            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		ADC_MCHP_AFEC_G1_DMA_CHANNEL(n)};                                                  \
                                                                                                   \
	static struct adc_mchp_afec_data mchp_afec_##n##_data = {                                  \
		ADC_CONTEXT_INIT_TIMER(mchp_afec_##n##_data, ctx),                                 \
		ADC_CONTEXT_INIT_LOCK(mchp_afec_##n##_data, ctx),                                  \
		ADC_CONTEXT_INIT_SYNC(mchp_afec_##n##_data, ctx),                                  \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, adc_mchp_afec_init, NULL, &mchp_afec_##n##_data,                  \
			      &mchp_afec_##n##_config, POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,      \
			      &adc_mchp_afec_driver_api);                                          \
                                                                                                   \
	static void mchp_afec_##n##_irq_config(const struct device *dev)                           \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), adc_mchp_afec_isr,          \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
