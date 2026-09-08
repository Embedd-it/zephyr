/*
 * SPDX-FileCopyrightText: Copyright (c) 2021, Piotr Mienkowski
 * SPDX-FileCopyrightText: Copyright (c) 2023, Gerson Fernando Budke
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip timer/counter (TC) counter driver.
 *
 */

#define DT_DRV_COMPAT microchip_tc_g3_counter

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/mchp_clock_control.h>
#include <zephyr/dt-bindings/clock/mchp_pic32cz_ca70_mc70_clock.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
LOG_MODULE_REGISTER(counter_mchp_tc_g3, CONFIG_COUNTER_LOG_LEVEL);

/*
 * Note that each instance of this driver is a separate channel of a TC peripheral, different from
 * the channels specified by the counter API. Those are represented by the two counters available
 * for each peripheral channel.
 */
#define MAX_ALARMS_PER_TC_CHANNEL 2u
#define COUNTER_CHANNEL_1         0u
#define COUNTER_CHANNEL_2         1u
#define COUNTER_CAPTURE_CHANNEL   COUNTER_CHANNEL_1
#define COUNTER_TOP_VALUE_MAX     UINT16_MAX
#define TC_CV_SYNC_TIMEOUT_US     10000u
#define TIMER_CLOCK2_DIVIDER      8u
#define TIMER_CLOCK3_DIVIDER      32u
#define TIMER_CLOCK4_DIVIDER      128u

enum timer_clock_selection {
	TIMER_CLOCK_SELECTION_PCK6_PCK7 = 0u,
	TIMER_CLOCK_SELECTION_MAIN_CLK_8,
	TIMER_CLOCK_SELECTION_MAIN_CLK_32,
	TIMER_CLOCK_SELECTION_MAIN_CLK_128,
	TIMER_CLOCK_SELECTION_SLOW_CLK,
	TIMER_CLOCK_SELECTION_XC0,
	TIMER_CLOCK_SELECTION_XC1,
	TIMER_CLOCK_SELECTION_XC2,
};

/* Device constant configuration parameters */
struct counter_mchp_tc_config {
	/* Common counter config; must be the first field in order to be accessed by casting the
	 * struct
	 */
	struct counter_config_info info;

	/* Register block */
	tc_registers_t *regs;

	/* CMR register initial configuration */
	uint32_t reg_cmr;

	/* RC register initial configuration */
	uint32_t reg_rc;

	/* IRQ setup function */
	void (*irq_config)(const struct device *dev);

	/* IRQ line used to raise late alarms */
	uint32_t irq_line;

	/* Clock driver */
	const struct device *clock;

	/* Clock subsystem for this instance's own channel */
	clock_control_subsys_t clock_subsys;

	/* Pin control configuration */
	const struct pinctrl_dev_config *pcfg;

	/* Channel number, default 0 */
	uint8_t tc_chan_num;

	/* Frequency of the external clock source, if any */
	uint32_t external_frequency;

	/* No DIV on main clock option */
	bool nodivclk;

	/* Whether to use PCK7 as programmable clock source (opposed to the default PCK6). Only for
	 * TC0
	 */
	bool use_pck7;
};

/* Device run time data */
struct counter_mchp_tc_data {
	/* User provided callback to be invoked upon reaching the top value */
	counter_top_callback_t top_cb;

	/* User provided pointer to be passed to the callback */
	void *top_cb_user_data;

	/* Alarm data structure */
	struct {
		/* Callback to be invoked on alarm */
		counter_alarm_callback_t callback;

		/* User provided pointer to be passed to the callback */
		void *user_data;
	} alarm[MAX_ALARMS_PER_TC_CHANNEL];

	/* Cached interrupt flags */
	atomic_t pending;

	/* Alarm compare flags (TC_SR layout) emulated for late alarms */
	atomic_t late_alarm_pending;

	/* Serializes alarm programming against the interrupt handler */
	struct k_spinlock lock;

	/* Guard period for detecting late absolute alarms, in ticks */
	uint32_t guard_period;

#if defined(CONFIG_COUNTER_CAPTURE)
	/* Callback to be invoked on capture */
	counter_capture_cb_t capture_cb;
	/* Capture configuration flags */
	counter_capture_flags_t capture_flags;
	/* Whether the capture should be single shot */
	bool single_shot_capture;
#endif /* CONFIG_COUNTER_CAPTURE */
};

/*
 * CV counts the full 16 bits when TC_RC is 0, 0..TC_RC-1 otherwise
 */
static uint32_t counter_mchp_tc_period(const tc_channel_registers_t *channel_regs)
{
	uint32_t rc = channel_regs->TC_RC;

	return (rc == 0u) ? (COUNTER_TOP_VALUE_MAX + 1u) : rc;
}

static int counter_mchp_tc_start(const struct device *dev)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	channel_regs->TC_CCR = TC_CCR_CLKEN_Msk | TC_CCR_SWTRG_Msk;

	return 0;
}

static int counter_mchp_tc_stop(const struct device *dev)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	channel_regs->TC_CCR = TC_CCR_CLKDIS_Msk;

	return 0;
}

static int counter_mchp_tc_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	*ticks = channel_regs->TC_CV;

	return 0;
}

static int counter_mchp_tc_reset(const struct device *dev)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	channel_regs->TC_CCR = TC_CCR_SWTRG_Msk;

	return 0;
}

static int counter_mchp_tc_set_alarm(const struct device *dev, uint8_t chan_id,
				     const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_mchp_tc_data *dev_data = dev->data;
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];
	uint32_t period = 0u;
	uint32_t alarm_value = 0u;
	uint32_t tc_cv = 0u;
	uint32_t tc_cv_now = 0u;
	uint32_t late_msk = 0u;
	bool late = false;
	bool irq_on_late = false;
	int ret = 0;
	k_spinlock_key_t key;

	/* In capture mode, cannot set alarm */
	if ((channel_regs->TC_CMR & TC_CMR_WAVE_Msk) == 0u) {
		return -EBUSY;
	}

	if (chan_id >= MAX_ALARMS_PER_TC_CHANNEL) {
		return -EINVAL;
	}

	if (alarm_cfg->callback == NULL) {
		return -EINVAL;
	}

	period = counter_mchp_tc_period(channel_regs);

	/* If the target is above the top value it cannot be set */
	if (alarm_cfg->ticks >= period) {
		return -EINVAL;
	}

	/* An alarm is already set */
	if (dev_data->alarm[chan_id].callback != NULL) {
		return -EBUSY;
	}

	key = k_spin_lock(&dev_data->lock);

	/* Use RA for the first alarm and RB for the second */
	if (chan_id == 0u) {
		channel_regs->TC_IDR = TC_IDR_CPAS_Msk;
	} else {
		channel_regs->TC_IDR = TC_IDR_CPBS_Msk;
	}

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0u) {
		alarm_value = alarm_cfg->ticks;
	} else {
		tc_cv = channel_regs->TC_CV;
		alarm_value = (tc_cv + alarm_cfg->ticks) % period;
	}

	dev_data->alarm[chan_id].callback = alarm_cfg->callback;
	dev_data->alarm[chan_id].user_data = alarm_cfg->user_data;

	if (chan_id == 0u) {
		channel_regs->TC_RA = (uint32_t)alarm_value;
		/* Clear interrupt status register */
		(void)channel_regs->TC_SR;
		channel_regs->TC_IER = TC_IER_CPAS_Msk;
	} else {
		channel_regs->TC_RB = (uint32_t)alarm_value;
		/* Clear interrupt status register */
		(void)channel_regs->TC_SR;
		channel_regs->TC_IER = TC_IER_CPBS_Msk;
		LOG_DBG("Registers %i 0x%04X", channel_regs->TC_RB, channel_regs->TC_IMR);
	}

	/* The compare only triggers when TC_CV changes to the programmed value, so a relative alarm
	 * that expired before RA/RB was written would not fire until the counter wraps around.
	 * We detect that and raise the interrupt via software instead.
	 * An absolute alarm is late when the counter has passed the target by less than the guard
	 * period; it then expires immediately only if requested.
	 */
	tc_cv_now = channel_regs->TC_CV;
	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0u) {
		late = ((((tc_cv_now + period) - tc_cv) % period) >= alarm_cfg->ticks);
		irq_on_late = true;
	} else {
		late = ((((tc_cv_now + period) - alarm_value) % period) < dev_data->guard_period);
		irq_on_late = ((alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) != 0u);
		if (late) {
			ret = -ETIME;
		}
	}

	if (late) {
		if (chan_id == 0u) {
			channel_regs->TC_IDR = TC_IDR_CPAS_Msk;
			late_msk = TC_SR_CPAS_Msk;
		} else {
			channel_regs->TC_IDR = TC_IDR_CPBS_Msk;
			late_msk = TC_SR_CPBS_Msk;
		}

		if (irq_on_late) {
			(void)atomic_or(&dev_data->late_alarm_pending, late_msk);
			k_irq_set_pending(dev_config->irq_line);
		} else {
			dev_data->alarm[chan_id].callback = NULL;
			dev_data->alarm[chan_id].user_data = NULL;
		}
	}

	k_spin_unlock(&dev_data->lock, key);

	LOG_DBG("set alarm: channel %u, count %u", chan_id, (uint32_t)alarm_value);

	return ret;
}

static int counter_mchp_tc_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	struct counter_mchp_tc_data *dev_data = dev->data;
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	if (chan_id == 0u) {
		channel_regs->TC_IDR = TC_IDR_CPAS_Msk;
		(void)atomic_and(&dev_data->late_alarm_pending, ~TC_SR_CPAS_Msk);
		channel_regs->TC_RA = 0u;
	} else {
		channel_regs->TC_IDR = TC_IDR_CPBS_Msk;
		(void)atomic_and(&dev_data->late_alarm_pending, ~TC_SR_CPBS_Msk);
		channel_regs->TC_RB = 0u;
	}

	dev_data->alarm[chan_id].callback = NULL;
	dev_data->alarm[chan_id].user_data = NULL;

	LOG_DBG("cancel alarm: channel %u", chan_id);

	return 0;
}

static int counter_mchp_tc_set_top_value(const struct device *dev,
					 const struct counter_top_cfg *top_cfg)
{
	struct counter_mchp_tc_data *dev_data = dev->data;
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];
	int ret = 0;

	for (size_t alarm_index = 0u; alarm_index < MAX_ALARMS_PER_TC_CHANNEL; alarm_index++) {
		/* An alarm is set, cannot change the top value */
		if (dev_data->alarm[alarm_index].callback) {
			return -EBUSY;
		}
	}

	if ((top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) != 0u) {
		if (channel_regs->TC_CV > top_cfg->ticks) {
			ret = -ETIME;
			if ((top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) != 0u) {
				channel_regs->TC_CCR = TC_CCR_SWTRG_Msk;
			}
		}
	} else {
		channel_regs->TC_CCR = TC_CCR_SWTRG_Msk;
	}

	/* Disable the compare interrupt */
	channel_regs->TC_IDR = TC_IDR_CPCS_Msk;

	dev_data->top_cb = top_cfg->callback;
	dev_data->top_cb_user_data = top_cfg->user_data;

	if (top_cfg->ticks == COUNTER_TOP_VALUE_MAX) {
		/* TC_RC = 0 lets the counter run over the full 16 bits */
		channel_regs->TC_RC = 0u;
	} else {
		channel_regs->TC_RC = top_cfg->ticks + 1u;
	}

	/* Enable the compare interrupt */
	channel_regs->TC_IER = TC_IER_CPCS_Msk;

	return ret;
}

static uint32_t counter_mchp_tc_get_top_value(const struct device *dev)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	return counter_mchp_tc_period(channel_regs) - 1u;
}

static uint32_t counter_mchp_tc_get_pending_int(const struct device *dev)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	struct counter_mchp_tc_data *dev_data = dev->data;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	/* Read and clear cached values */
	return atomic_and(&dev_data->pending, 0u) & channel_regs->TC_IMR;
}

static uint32_t counter_mchp_tc_get_guard_period(const struct device *dev, uint32_t flags)
{
	const struct counter_mchp_tc_data *dev_data = dev->data;

	ARG_UNUSED(flags);

	return dev_data->guard_period;
}

static int counter_mchp_tc_set_guard_period(const struct device *dev, uint32_t ticks,
					    uint32_t flags)
{
	struct counter_mchp_tc_data *dev_data = dev->data;
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	ARG_UNUSED(flags);

	/* The guard period cannot exceed the top value (period - 1) */
	if (ticks >= counter_mchp_tc_period(channel_regs)) {
		return -EINVAL;
	}

	dev_data->guard_period = ticks;

	return 0;
}

static uint32_t counter_mchp_tc_get_freq(const struct device *dev)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];
	int ret = 0;
	uint32_t rate = 0u;
	uint32_t subsys = 0u;

	if (dev_config->nodivclk) {
		subsys = MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 0u);
		ret = clock_control_get_rate(dev_config->clock, (clock_control_subsys_t)subsys,
					     &rate);
	} else {
		switch ((channel_regs->TC_CMR & TC_CMR_TCCLKS_Msk) >> TC_CMR_TCCLKS_Pos) {
		case TC_CMR_TCCLKS_TIMER_CLOCK1_Val:
			subsys = dev_config->use_pck7 ? CLOCK_MCHP_PCK7 : CLOCK_MCHP_PCK6;
			ret = clock_control_get_rate(dev_config->clock,
						     (clock_control_subsys_t)subsys, &rate);
			break;
		case TC_CMR_TCCLKS_TIMER_CLOCK2_Val:
			subsys = MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 0u);
			ret = clock_control_get_rate(dev_config->clock,
						     (clock_control_subsys_t)subsys, &rate);
			rate /= TIMER_CLOCK2_DIVIDER;
			break;
		case TC_CMR_TCCLKS_TIMER_CLOCK3_Val:
			subsys = MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 0u);
			ret = clock_control_get_rate(dev_config->clock,
						     (clock_control_subsys_t)subsys, &rate);
			rate /= TIMER_CLOCK3_DIVIDER;
			break;
		case TC_CMR_TCCLKS_TIMER_CLOCK4_Val:
			subsys = MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_PERIPHERAL, 0u);
			ret = clock_control_get_rate(dev_config->clock,
						     (clock_control_subsys_t)subsys, &rate);
			rate /= TIMER_CLOCK4_DIVIDER;
			break;
		case TC_CMR_TCCLKS_TIMER_CLOCK5_Val:
			subsys = MCHP_CLOCK_DERIVE_ID(SUBSYS_TYPE_SLOW_CLK, 0u);
			ret = clock_control_get_rate(dev_config->clock,
						     (clock_control_subsys_t)subsys, &rate);
			break;
		case TC_CMR_TCCLKS_XC0_Val:
		case TC_CMR_TCCLKS_XC1_Val:
		case TC_CMR_TCCLKS_XC2_Val:
			rate = dev_config->external_frequency;
			break;
		}
	}

	if (ret < 0) {
		return 0;
	} else {
		return rate;
	}
}

#if defined(CONFIG_COUNTER_CAPTURE)
static int counter_mchp_tc_enable_capture(const struct device *dev, uint8_t chan)
{
	struct counter_mchp_tc_data *dev_data = dev->data;
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	if (chan != COUNTER_CAPTURE_CHANNEL) {
		LOG_ERR("Cannot use %d as capture channel, only %d", chan, COUNTER_CAPTURE_CHANNEL);
		return -EINVAL;
	}

	/* Make sure a capture callback is set */
	if (dev_data->capture_cb == NULL) {
		LOG_ERR("Capture callback not set for channel %d, please configure", chan);
		return -EINVAL;
	}

	dev_data->single_shot_capture =
		(dev_data->capture_flags & COUNTER_CAPTURE_SINGLE_SHOT) != 0u;

	/* Clear stale interrupts */
	(void)channel_regs->TC_SR;

	/* Enable interrupts */
	channel_regs->TC_IER = TC_IER_LDRAS_Msk | TC_IER_LDRBS_Msk;

	return 0;
}

static int counter_mchp_tc_disable_capture(const struct device *dev, uint8_t chan)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];

	if (chan != COUNTER_CAPTURE_CHANNEL) {
		LOG_ERR("Cannot use %d as capture channel, only %d", chan, COUNTER_CAPTURE_CHANNEL);
		return -EINVAL;
	}

	/* Disable capture mode */
	channel_regs->TC_CMR = dev_config->reg_cmr;

	/* Disable interrupts */
	channel_regs->TC_IDR = TC_IDR_LDRAS_Msk | TC_IDR_LDRBS_Msk;

	return 0;
}

static int counter_mchp_tc_capture_configure(const struct device *dev, uint8_t chan,
					     counter_capture_flags_t flags, counter_capture_cb_t cb,
					     void *user_data)
{
	struct counter_mchp_tc_data *dev_data = dev->data;
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];
	uint32_t tc_cmr = 0u;

	if (chan != COUNTER_CAPTURE_CHANNEL) {
		LOG_ERR("Cannot use %d as capture channel, only %d", chan, COUNTER_CAPTURE_CHANNEL);
		return -EINVAL;
	}

	for (size_t alarm_index = 0u; alarm_index < MAX_ALARMS_PER_TC_CHANNEL; alarm_index++) {
		/* Prevent configuring capture on a channel already used for alarms */
		if (dev_data->alarm[alarm_index].callback != NULL) {
			LOG_ERR("Channel %d already configured for alarm, cannot set capture",
				chan);
			return -EBUSY;
		}
	}

	dev_data->capture_flags = flags;
	dev_data->capture_cb = cb;
	dev_data->alarm[chan].user_data = user_data;

	tc_cmr = channel_regs->TC_CMR;
	/* Enable capture mode */
	tc_cmr &= ~TC_CMR_WAVE_Msk;

	tc_cmr &= ~(TC_CMR_CAPTURE_LDRA_Msk | TC_CMR_CAPTURE_LDRB_Msk);
	if ((dev_data->capture_flags & COUNTER_CAPTURE_BOTH_EDGES) == COUNTER_CAPTURE_BOTH_EDGES) {
		tc_cmr |= TC_CMR_CAPTURE_LDRA(TC_CMR_CAPTURE_LDRA_EDGE_Val) |
			  TC_CMR_CAPTURE_LDRB(TC_CMR_CAPTURE_LDRB_EDGE_Val);
	} else if ((dev_data->capture_flags & COUNTER_CAPTURE_RISING_EDGE) != 0u) {
		tc_cmr |= TC_CMR_CAPTURE_LDRA(TC_CMR_CAPTURE_LDRA_RISING_Val) |
			  TC_CMR_CAPTURE_LDRB(TC_CMR_CAPTURE_LDRB_RISING_Val);
	} else if ((dev_data->capture_flags & COUNTER_CAPTURE_FALLING_EDGE) != 0u) {
		tc_cmr |= TC_CMR_CAPTURE_LDRA(TC_CMR_CAPTURE_LDRA_FALLING_Val) |
			  TC_CMR_CAPTURE_LDRB(TC_CMR_CAPTURE_LDRB_FALLING_Val);
	}

	channel_regs->TC_CMR = tc_cmr;

	return 0;
}
#endif /* CONFIG_COUNTER_CAPTURE */

static DEVICE_API(counter, counter_mchp_tc_driver_api) = {
	.start = counter_mchp_tc_start,
	.stop = counter_mchp_tc_stop,
	.get_value = counter_mchp_tc_get_value,
	.reset = counter_mchp_tc_reset,
	.set_alarm = counter_mchp_tc_set_alarm,
	.cancel_alarm = counter_mchp_tc_cancel_alarm,
	.set_top_value = counter_mchp_tc_set_top_value,
	.get_top_value = counter_mchp_tc_get_top_value,
	.get_pending_int = counter_mchp_tc_get_pending_int,
	.get_freq = counter_mchp_tc_get_freq,
	.get_guard_period = counter_mchp_tc_get_guard_period,
	.set_guard_period = counter_mchp_tc_set_guard_period,
#if defined(CONFIG_COUNTER_CAPTURE)
	.enable_capture = counter_mchp_tc_enable_capture,
	.disable_capture = counter_mchp_tc_disable_capture,
	.capture_configure = counter_mchp_tc_capture_configure,
#endif /* CONFIG_COUNTER_CAPTURE */
};

static int counter_mchp_tc_init(const struct device *dev)
{
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];
	uint32_t peripheral_clock_rate = 0u;
	uint32_t programmable_clock_rate = 0u;
	clock_control_subsys_t programmable_clock = NULL;
	int ret = 0;

	/* Connect pins to the peripheral */
	ret = pinctrl_apply_state(dev_config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		LOG_WRN("Unable to configure pins");
		return ret;
	}

	/* Enable channel's clock */
	ret = clock_control_on(dev_config->clock, dev_config->clock_subsys);
	if (ret != 0) {
		LOG_WRN("Unable to turn on peripheral clock");
		return ret;
	}

	/* If used enable programmable clock */
	if (((dev_config->reg_cmr & TC_CMR_TCCLKS_Msk) >> TC_CMR_TCCLKS_Pos) ==
	    TIMER_CLOCK_SELECTION_PCK6_PCK7) {
		ret = clock_control_get_rate(dev_config->clock, dev_config->clock_subsys,
					     &peripheral_clock_rate);
		if (ret != 0) {
			LOG_ERR("Unable to fetch the peripheral clock rate: %i", ret);
			return ret;
		}

		/* Enable programmable clock */
		if (dev_config->use_pck7) {
			programmable_clock = (clock_control_subsys_t)CLOCK_MCHP_PCK7;
		} else {
			programmable_clock = (clock_control_subsys_t)CLOCK_MCHP_PCK6;
		}

		ret = clock_control_on(dev_config->clock, programmable_clock);
		if (ret != 0) {
			LOG_ERR("Unable to activate PCK: %i", ret);
			return ret;
		}

		ret = clock_control_get_rate(dev_config->clock, programmable_clock,
					     &programmable_clock_rate);
		if (ret != 0) {
			LOG_ERR("Unable to fetch the programmable clock rate: %i", ret);
			return ret;
		}

		/* PCK6/7 must run at least 2.5 times slower than the peripheral clock rate */
		if ((programmable_clock_rate * 5) / 2 > peripheral_clock_rate) {
			LOG_ERR("The PCK frequency must be at least 2.5 times lower than the "
				"peripheral clock frequency");
			return -EINVAL;
		}
	}

	/* Clock and Mode Selection */
	channel_regs->TC_CMR = dev_config->reg_cmr;
	channel_regs->TC_RC = dev_config->reg_rc;

	if (dev_config->nodivclk) {
		channel_regs->TC_EMR = TC_EMR_NODIVCLK_Msk;
	}

	dev_config->irq_config(dev);

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

/*
 * Driver interrupt handler
 */
static void counter_mchp_tc_isr(const struct device *dev)
{
	struct counter_mchp_tc_data *dev_data = dev->data;
	const struct counter_mchp_tc_config *dev_config = dev->config;
	tc_registers_t *const regs = dev_config->regs;
	tc_channel_registers_t *channel_regs = &regs->TC_CHANNEL[dev_config->tc_chan_num];
	uint32_t status = 0u;

	status = channel_regs->TC_SR & channel_regs->TC_IMR;
	/* Late alarms raised in software */
	status |= (uint32_t)atomic_clear(&dev_data->late_alarm_pending);
	dev_data->pending = status;

	if ((status & TC_SR_CPAS_Msk) != 0u) {
		channel_regs->TC_IDR = TC_IDR_CPAS_Msk;
		if (dev_data->alarm[COUNTER_CHANNEL_1].callback) {
			dev_data->alarm[COUNTER_CHANNEL_1].callback(
				dev, COUNTER_CHANNEL_1, channel_regs->TC_RA,
				dev_data->alarm[COUNTER_CHANNEL_1].user_data);
			dev_data->alarm[COUNTER_CHANNEL_1].callback = NULL;
		}
	}

	if ((status & TC_SR_CPBS_Msk) != 0u) {
		channel_regs->TC_IDR = TC_IDR_CPBS_Msk;
		if (dev_data->alarm[COUNTER_CHANNEL_2].callback) {
			dev_data->alarm[COUNTER_CHANNEL_2].callback(
				dev, COUNTER_CHANNEL_2, channel_regs->TC_RB,
				dev_data->alarm[COUNTER_CHANNEL_2].user_data);
			dev_data->alarm[COUNTER_CHANNEL_2].callback = NULL;
		}
	}

	if ((status & TC_SR_CPCS_Msk) != 0u) {
		if (dev_data->top_cb) {
			dev_data->top_cb(dev, dev_data->top_cb_user_data);
		}
	}

#if defined(CONFIG_COUNTER_CAPTURE)
	if ((status & TC_SR_LDRAS_Msk) != 0u) {
		if (dev_data->capture_cb) {
			dev_data->capture_cb(dev, COUNTER_CAPTURE_CHANNEL, dev_data->capture_flags,
					     channel_regs->TC_RA,
					     dev_data->alarm[COUNTER_CAPTURE_CHANNEL].user_data);
		}
		/* If a single shot, disable future interrupts */
		if (dev_data->single_shot_capture) {
			channel_regs->TC_IDR = TC_IDR_LDRAS_Msk | TC_IDR_LDRBS_Msk;
		}
	}

	if ((status & TC_SR_LDRBS_Msk) != 0u) {
		if (dev_data->capture_cb) {
			dev_data->capture_cb(dev, COUNTER_CAPTURE_CHANNEL, dev_data->capture_flags,
					     channel_regs->TC_RB,
					     dev_data->alarm[COUNTER_CAPTURE_CHANNEL].user_data);
		}
		/* If a single shot, disable future interrupts */
		if (dev_data->single_shot_capture) {
			channel_regs->TC_IDR = TC_IDR_LDRAS_Msk | TC_IDR_LDRBS_Msk;
		}
	}
#endif /* CONFIG_COUNTER_CAPTURE */
}

/*
 * Initial CMR value. Includes the clock source taken from the DTS, counting up mode, waveform
 * selection for counter operation and not selecting TIOB as external event in order to have two
 * channels.
 */
#define COUNTER_TC_CMR(n)                                                                          \
	(TC_CMR_TCCLKS(DT_INST_ENUM_IDX_OR(n, clk, 0)) | TC_CMR_WAVEFORM_WAVSEL_UP_RC |            \
	 TC_CMR_WAVE_Msk | TC_CMR_WAVEFORM_EEVT_XC0)

#define COUNTER_TC_REG_CMR(n) DT_INST_PROP_OR(n, reg_cmr, COUNTER_TC_CMR(n))

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, 0, irq), DT_INST_IRQ_BY_IDX(n, 0, priority),     \
			    counter_mchp_tc_isr, DEVICE_DT_INST_GET(n), 0);                        \
		irq_enable(DT_INST_IRQ_BY_IDX(n, 0, irq));                                         \
	}                                                                                          \
                                                                                                   \
	static const struct counter_mchp_tc_config mchp_tc_##n##_config = {                        \
		.info =                                                                            \
			{                                                                          \
				.max_top_value = COUNTER_TOP_VALUE_MAX,                            \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = MAX_ALARMS_PER_TC_CHANNEL,                             \
			},                                                                         \
		.regs = (tc_registers_t *)DT_REG_ADDR(DT_INST_PARENT(n)),                          \
		.reg_cmr = COUNTER_TC_REG_CMR(n),                                                  \
		.reg_rc = DT_INST_PROP_OR(n, reg_rc, 0),                                           \
		.irq_config = &irq_config_##n,                                                     \
		.irq_line = DT_INST_IRQ_BY_IDX(n, 0, irq),                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.nodivclk = DT_INST_PROP(n, nodivclk),                                             \
		.tc_chan_num = DT_INST_PROP_OR(n, channel, 0),                                     \
		.use_pck7 = DT_INST_PROP_OR(n, use_pck7, false),                                   \
		.external_frequency = DT_INST_PROP_OR(n, external_frequency, 0),                   \
		.clock = DEVICE_DT_GET(DT_INST_PHANDLE(n, clocks)),                                \
		.clock_subsys =                                                                    \
			(clock_control_subsys_t)DT_INST_CLOCKS_CELL_BY_IDX(n, 0, subsystem),       \
	};                                                                                         \
                                                                                                   \
	static struct counter_mchp_tc_data mchp_tc_##n##_data = {};                                \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, counter_mchp_tc_init, NULL, &mchp_tc_##n##_data,                  \
			      &mchp_tc_##n##_config, POST_KERNEL, CONFIG_COUNTER_INIT_PRIORITY,    \
			      &counter_mchp_tc_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION)
