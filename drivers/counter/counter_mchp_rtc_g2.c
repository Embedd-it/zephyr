/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip RTC-based counter driver.
 *
 */

#define DT_DRV_COMPAT microchip_rtc_g2_counter

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/timeutil.h>

#include <time.h>
#include <string.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(counter_mchp_rtc_g2, CONFIG_COUNTER_LOG_LEVEL);

#define REG_GET_FIELD(value, field) ((RTC_##field##_Msk & (value)) >> RTC_##field##_Pos)
#define RTC_SYNC_TIMEOUT_US         1000u

/*
 * This RTC peripheral is a BCD calendar/alarm block with no free-running
 * counter-mode registers. This driver converts the calendar to/from a Unix
 * timestamp and uses that as the counter tick value, at the cost of a
 * BCD<->binary conversion (and, for alarms, a struct tm round-trip) on every
 * access. This is a deliberate simplicity-over-efficiency tradeoff: the
 * hardware has no cheaper way to expose a raw tick count.
 */

struct counter_mchp_rtc_config {
	/* Must be first member: generic Counter API helpers cast dev->config
	 * to struct counter_config_info directly.
	 */
	struct counter_config_info info;

	/* Register block */
	rtc_registers_t *regs;

	/* Interrupt number */
	uint16_t irq_num;

	/* Interrupt configuration function */
	void (*irq_config)(const struct device *dev);
};

struct counter_mchp_rtc_data {
	/* User provided callback to be invoked on alarm expiry */
	counter_alarm_callback_t alarm_callback;

	/* User provided pointer to be passed to the alarm callback */
	void *alarm_user_data;

	/* Driver mutex */
	struct k_mutex lock;

	/* Synchronization semaphore with the second tick */
	struct k_sem sec_evt_done;

	/* Synchronization semaphore with update acknowledge */
	struct k_sem upd_ack_done;
};

static int counter_mchp_rtc_read_tm(rtc_registers_t *const regs, struct tm *tm)
{
	uint32_t timr0 = 0u;
	uint32_t calr0 = 0u;
	uint32_t timr1 = 0u;
	uint32_t calr1 = 0u;
	uint32_t wait_cycle_count = k_us_to_cyc_ceil32(RTC_SYNC_TIMEOUT_US);
	uint32_t wait_start = k_cycle_get_32();

	/* Validate time and date are not "never set" */
	if ((regs->RTC_VER & (RTC_VER_NVTIM_Msk | RTC_VER_NVCAL_Msk)) != 0) {
		return -ENODATA;
	}

	/* Read until synchronized (registers updated async) */
	do {
		timr0 = regs->RTC_TIMR;
		calr0 = regs->RTC_CALR;
		timr1 = regs->RTC_TIMR;
		calr1 = regs->RTC_CALR;

	} while (((timr0 != timr1) || (calr0 != calr1)) &&
		 (wait_cycle_count > (k_cycle_get_32() - wait_start)));

	if ((timr0 != timr1) || (calr0 != calr1)) {
		return -ETIMEDOUT;
	}

	memset(tm, 0x00, sizeof(*tm));

	tm->tm_sec = bcd2bin(REG_GET_FIELD(timr0, TIMR_SEC));
	tm->tm_min = bcd2bin(REG_GET_FIELD(timr0, TIMR_MIN));
	tm->tm_hour = bcd2bin(REG_GET_FIELD(timr0, TIMR_HOUR));
	tm->tm_mday = bcd2bin(REG_GET_FIELD(calr0, CALR_DATE));
	tm->tm_mon = bcd2bin(REG_GET_FIELD(calr0, CALR_MONTH)) - 1;

	tm->tm_year = bcd2bin(REG_GET_FIELD(calr0, CALR_YEAR));
	tm->tm_year += ((int)bcd2bin(REG_GET_FIELD(calr0, CALR_CENT))) * 100;
	tm->tm_year -= 1900;

	return 0;
}

static uint32_t counter_mchp_rtc_timalr_from_tm(const struct tm *tm)
{
	uint32_t timalr = (RTC_TIMALR_SECEN_Msk | RTC_TIMALR_MINEN_Msk | RTC_TIMALR_HOUREN_Msk);

	timalr |= RTC_TIMALR_SEC(bin2bcd(tm->tm_sec));
	timalr |= RTC_TIMALR_MIN(bin2bcd(tm->tm_min));
	timalr |= RTC_TIMALR_HOUR(bin2bcd(tm->tm_hour));

	return timalr;
}

static uint32_t counter_mchp_rtc_calalr_from_tm(const struct tm *tm)
{
	uint32_t calalr = (RTC_CALALR_MTHEN_Msk | RTC_CALALR_DATEEN_Msk);

	calalr |= RTC_CALALR_MONTH(bin2bcd(tm->tm_mon + 1));
	calalr |= RTC_CALALR_DATE(bin2bcd(tm->tm_mday));

	return calalr;
}

static uint32_t counter_mchp_rtc_timr_from_tm(const struct tm *tm)
{
	uint32_t timr = 0u;

	timr = RTC_TIMR_SEC(bin2bcd(tm->tm_sec));
	timr |= RTC_TIMR_MIN(bin2bcd(tm->tm_min));
	timr |= RTC_TIMR_HOUR(bin2bcd(tm->tm_hour));

	return timr;
}

static uint32_t counter_mchp_rtc_calr_from_tm(const struct tm *tm)
{
	uint32_t calr = 0u;
	uint8_t centuries = 0u;
	uint8_t years = 0u;

	calr = RTC_CALR_DATE(bin2bcd(tm->tm_mday));
	calr |= RTC_CALR_MONTH(bin2bcd(tm->tm_mon + 1));
	centuries = (uint8_t)((tm->tm_year / 100) + 19);
	years = (uint8_t)(tm->tm_year % 100);
	calr |= RTC_CALR_CENT(bin2bcd(centuries));
	calr |= RTC_CALR_YEAR(bin2bcd(years));
	calr |= RTC_CALR_DAY(bin2bcd(tm->tm_wday + 1));

	return calr;
}

static int counter_mchp_rtc_set_value(const struct device *dev, uint32_t ticks)
{
	struct counter_mchp_rtc_data *dev_data = dev->data;
	const struct counter_mchp_rtc_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;
	time_t target = (time_t)ticks;
	struct tm tm = {0u};

	gmtime_r(&target, &tm);

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	/* Wait for the tick of a second, matching rtc_mchp_set_time() */
	k_sem_reset(&dev_data->sec_evt_done);

	/* Enable the interrupt just for this semaphore wait */
	regs->RTC_IER = RTC_IER_SECEN_Msk;
	k_sem_take(&dev_data->sec_evt_done, K_MSEC(1100));
	regs->RTC_IDR = RTC_IDR_SECDIS_Msk;

	k_sem_reset(&dev_data->upd_ack_done);

	/* Enable update acknowledge interrupt */
	regs->RTC_IER = RTC_IER_ACKEN_Msk;

	/* Request update */
	regs->RTC_CR = (RTC_CR_UPDTIM_Msk | RTC_CR_UPDCAL_Msk);

	/* Await update acknowledge */
	if (k_sem_take(&dev_data->upd_ack_done, K_MSEC(1100)) < 0) {
		regs->RTC_CR = 0;
		regs->RTC_IDR = RTC_IDR_ACKDIS_Msk;

		k_mutex_unlock(&dev_data->lock);
		return -EAGAIN;
	}

	regs->RTC_TIMR = counter_mchp_rtc_timr_from_tm(&tm);
	regs->RTC_CALR = counter_mchp_rtc_calr_from_tm(&tm);
	regs->RTC_CR = 0u;
	regs->RTC_IDR = RTC_IDR_ACKDIS_Msk;

	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static int counter_mchp_rtc_get_value(const struct device *dev, uint32_t *ticks)
{
	const struct counter_mchp_rtc_config *dev_config = dev->config;
	struct tm tm = {0u};
	int ret = 0;

	ret = counter_mchp_rtc_read_tm(dev_config->regs, &tm);
	if (ret < 0) {
		return ret;
	}

	*ticks = (uint32_t)timeutil_timegm(&tm);

	return 0;
}

static int counter_mchp_rtc_start(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int counter_mchp_rtc_stop(const struct device *dev)
{
	ARG_UNUSED(dev);

	return -ENOTSUP;
}

static int counter_mchp_rtc_set_alarm(const struct device *dev, uint8_t chan_id,
				      const struct counter_alarm_cfg *alarm_cfg)
{
	struct counter_mchp_rtc_data *dev_data = dev->data;
	const struct counter_mchp_rtc_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;
	struct tm tm = {0u};
	time_t target = 0;
	uint32_t now = 0u;
	uint32_t timalr = 0u;
	uint32_t calalr = 0u;
	int ret = 0;

	if (chan_id != 0u) {
		return -ENOTSUP;
	}

	if (alarm_cfg->callback == NULL) {
		return -EINVAL;
	}

	if (dev_data->alarm_callback != NULL) {
		return -EBUSY;
	}

	ret = counter_mchp_rtc_get_value(dev, &now);
	if (ret < 0) {
		return ret;
	}

	if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) != 0u) {
		target = (time_t)alarm_cfg->ticks;
	} else {
		/* +1 compensates for the partially elapsed current tick */
		target = (time_t)(now + alarm_cfg->ticks + 1);
	}

	gmtime_r(&target, &tm);

	/*
	 * Note: RTC_TIMALR/RTC_CALALR have no year field, so this alarm
	 * fires whenever sec/min/hour/day/month next match.
	 */
	timalr = counter_mchp_rtc_timalr_from_tm(&tm);
	calalr = counter_mchp_rtc_calalr_from_tm(&tm);

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	irq_disable(dev_config->irq_num);

	dev_data->alarm_callback = alarm_cfg->callback;
	dev_data->alarm_user_data = alarm_cfg->user_data;

	regs->RTC_TIMALR = timalr;
	regs->RTC_CALALR = calalr;

	/* Clear alarm pending status */
	regs->RTC_SCCR = RTC_SCCR_ALRCLR_Msk;
	regs->RTC_IER = RTC_IER_ALREN_Msk;

	irq_enable(dev_config->irq_num);
	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static int counter_mchp_rtc_cancel_alarm(const struct device *dev, uint8_t chan_id)
{
	struct counter_mchp_rtc_data *dev_data = dev->data;
	const struct counter_mchp_rtc_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	if (chan_id != 0u) {
		return -ENOTSUP;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);
	irq_disable(dev_config->irq_num);

	regs->RTC_IDR = RTC_IDR_ALRDIS_Msk;
	regs->RTC_SCCR = RTC_SCCR_ALRCLR_Msk;

	dev_data->alarm_callback = NULL;
	dev_data->alarm_user_data = NULL;

	irq_enable(dev_config->irq_num);
	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static uint32_t counter_mchp_rtc_get_pending_int(const struct device *dev)
{
	const struct counter_mchp_rtc_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	return ((regs->RTC_SR & RTC_SR_ALARM_Msk) != 0u) ? 1u : 0u;
}

static uint32_t counter_mchp_rtc_get_top_value(const struct device *dev)
{
	const struct counter_mchp_rtc_config *dev_config = dev->config;

	return dev_config->info.max_top_value;
}

static int counter_mchp_rtc_set_top_value(const struct device *dev,
					  const struct counter_top_cfg *cfg)
{
	const struct counter_mchp_rtc_config *dev_config = dev->config;
	struct counter_mchp_rtc_data *dev_data = dev->data;

	if (dev_data->alarm_callback != NULL) {
		return -EBUSY;
	}

	/* The calendar's wraparound point (year 2106) is fixed by the
	 * register width and cannot be reprogrammed.
	 */
	if ((cfg->ticks != dev_config->info.max_top_value) ||
	    ((cfg->flags & COUNTER_TOP_CFG_DONT_RESET) == 0u)) {
		return -ENOTSUP;
	}

	return 0;
}

static void counter_mchp_rtc_isr(const struct device *dev)
{
	struct counter_mchp_rtc_data *dev_data = dev->data;
	const struct counter_mchp_rtc_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;
	uint32_t sr = regs->RTC_SR;
	void *user_data = dev_data->alarm_user_data;
	uint32_t ticks = 0u;
	counter_alarm_callback_t callback = dev_data->alarm_callback;

	if ((sr & RTC_SR_ACKUPD_Msk) != 0u) {
		regs->RTC_SCCR = RTC_SCCR_ACKCLR_Msk;
		k_sem_give(&dev_data->upd_ack_done);
	}

	if ((sr & RTC_SR_ALARM_Msk) != 0u) {
		regs->RTC_SCCR = RTC_SCCR_ALRCLR_Msk;

		/* Single-shot: disable the source and clear the callback
		 * before invoking it.
		 */
		regs->RTC_IDR = RTC_IDR_ALRDIS_Msk;
		dev_data->alarm_callback = NULL;
		dev_data->alarm_user_data = NULL;

		if (callback != NULL) {
			(void)counter_mchp_rtc_get_value(dev, &ticks);
			callback(dev, 0u, ticks, user_data);
		}
	}

	if ((sr & RTC_SR_SEC_Msk) != 0u) {
		regs->RTC_SCCR = RTC_SCCR_SECCLR_Msk;
		k_sem_give(&dev_data->sec_evt_done);
	}
}

static DEVICE_API(counter, counter_mchp_rtc_driver_api) = {
	.start = counter_mchp_rtc_start,
	.stop = counter_mchp_rtc_stop,
	.get_value = counter_mchp_rtc_get_value,
	.set_value = counter_mchp_rtc_set_value,
	.set_alarm = counter_mchp_rtc_set_alarm,
	.cancel_alarm = counter_mchp_rtc_cancel_alarm,
	.set_top_value = counter_mchp_rtc_set_top_value,
	.get_pending_int = counter_mchp_rtc_get_pending_int,
	.get_top_value = counter_mchp_rtc_get_top_value,
};

static int counter_mchp_rtc_init(const struct device *dev)
{
	struct counter_mchp_rtc_data *dev_data = dev->data;
	const struct counter_mchp_rtc_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	regs->RTC_MR &= ~(RTC_MR_HRMOD_Msk | RTC_MR_PERSIAN_Msk);
	regs->RTC_CR = 0u;
	regs->RTC_IDR = (RTC_IDR_ACKDIS_Msk | RTC_IDR_ALRDIS_Msk | RTC_IDR_SECDIS_Msk |
			 RTC_IDR_TIMDIS_Msk | RTC_IDR_CALDIS_Msk | RTC_IDR_TDERRDIS_Msk);

	k_mutex_init(&dev_data->lock);
	k_sem_init(&dev_data->sec_evt_done, 0, 1);
	k_sem_init(&dev_data->upd_ack_done, 0, 1);
	dev_data->alarm_callback = NULL;
	dev_data->alarm_user_data = NULL;

	dev_config->irq_config(dev);

	irq_enable(dev_config->irq_num);

	return 0;
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), counter_mchp_rtc_isr,       \
			    DEVICE_DT_INST_GET(n), 0);                                             \
	}                                                                                          \
                                                                                                   \
	static const struct counter_mchp_rtc_config mchp_rtc_##n##_config = {                      \
		.info =                                                                            \
			{                                                                          \
				.max_top_value = UINT32_MAX,                                       \
				.freq = 1,                                                         \
				.flags = COUNTER_CONFIG_INFO_COUNT_UP,                             \
				.channels = 1,                                                     \
			},                                                                         \
		.regs = (rtc_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.irq_num = DT_INST_IRQN(n),                                                        \
		.irq_config = irq_config_##n,                                                      \
	};                                                                                         \
                                                                                                   \
	static struct counter_mchp_rtc_data mchp_rtc_##n##_data;                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, counter_mchp_rtc_init, NULL, &mchp_rtc_##n##_data,                \
			      &mchp_rtc_##n##_config, POST_KERNEL, CONFIG_COUNTER_INIT_PRIORITY,   \
			      &counter_mchp_rtc_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION);
