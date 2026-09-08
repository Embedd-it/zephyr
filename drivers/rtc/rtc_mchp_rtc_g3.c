/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 Bjarki Arge Andreasen
 * SPDX-FileCopyrightText: Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Microchip RTC driver.
 *
 */

#define DT_DRV_COMPAT microchip_rtc_g3_rtc

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/util.h>

#include <string.h>
#include <soc.h>
#include "rtc_utils.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rtc_mchp_rtc_g3, CONFIG_RTC_LOG_LEVEL);

#define REG_GET_FIELD(value, field) ((RTC_##field##_Msk & (value)) >> RTC_##field##_Pos)

#define RTC_CALIBRATE_PPB_MAX       (1950000)
#define RTC_CALIBRATE_PPB_MIN       (-1950000)
#define RTC_CALIBRATE_PPB_QUANTA    (1500)
#define RTC_CALIBRATE_PPB_LOW_SCALE (30500)

#define RTC_TIME_MASK                                                                              \
	(RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |      \
	 RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_YEAR |     \
	 RTC_ALARM_TIME_MASK_WEEKDAY)

#define RTC_SYNC_TIMEOUT_US 1000u

struct rtc_mchp_config {
	/* Register block */
	rtc_registers_t *regs;

	/* Interrupt number */
	uint16_t irq_num;

	/* Interrupt configuration function */
	void (*irq_config)(const struct device *dev);
};

struct rtc_mchp_data {
#if defined(CONFIG_RTC_ALARM)
	/* User provided callback to be invoked on alarm */
	rtc_alarm_callback alarm_callback;

	/* User provided pointer to be passed to the callback*/
	void *alarm_callback_user_data;
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)
	/* User provided callback to be invoked each second */
	rtc_update_callback update_callback;

	/* User provided pointer to be passed to the callback*/
	void *update_callback_user_data;
#endif /* CONFIG_RTC_UPDATE */

	/* Driver mutex */
	struct k_mutex lock;

	/* Synchronization semaphore with the second tick */
	struct k_sem sec_evt_done;

	/* Synchronization semaphore with update acknowledge */
	struct k_sem upd_ack_done;
};

static uint32_t rtc_timr_from_tm(const struct rtc_time *timeptr)
{
	uint32_t timr = 0u;

	timr = RTC_TIMR_SEC(bin2bcd(timeptr->tm_sec));
	timr |= RTC_TIMR_MIN(bin2bcd(timeptr->tm_min));
	timr |= RTC_TIMR_HOUR(bin2bcd(timeptr->tm_hour));

	return timr;
}

static uint32_t rtc_calr_from_tm(const struct rtc_time *timeptr)
{
	uint32_t calr = 0u;
	uint8_t centuries = 0u;
	uint8_t years = 0u;

	calr = RTC_CALR_DATE(bin2bcd(timeptr->tm_mday));
	calr |= RTC_CALR_MONTH(bin2bcd(timeptr->tm_mon + 1));
	centuries = (uint8_t)((timeptr->tm_year / 100) + 19);
	years = (uint8_t)((timeptr->tm_year % 100));
	calr |= RTC_CALR_CENT(bin2bcd(centuries));
	calr |= RTC_CALR_YEAR(bin2bcd(years));
	calr |= RTC_CALR_DAY(bin2bcd(timeptr->tm_wday + 1));
	return calr;
}

static int rtc_mchp_set_time(const struct device *dev, const struct rtc_time *timeptr)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	if (!rtc_utils_validate_rtc_time(timeptr, RTC_TIME_MASK)) {
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	regs->RTC_SCCR = RTC_SCCR_SECCLR_Msk;
	regs->RTC_IER = RTC_IER_SECEN_Msk;

	/* Wait for the tick of a second */
	k_sem_reset(&dev_data->sec_evt_done);
	k_sem_take(&dev_data->sec_evt_done, K_MSEC(1100));

#if defined(CONFIG_RTC_UPDATE)
	/* If no updated callback is requested disable the interrupt after using it */
	if (dev_data->update_callback == NULL) {
		regs->RTC_IDR = RTC_IDR_SECDIS_Msk;
	}
#else
	regs->RTC_IDR = RTC_IDR_SECDIS_Msk;
#endif /* CONFIG_RTC_UPDATE */

	k_sem_reset(&dev_data->upd_ack_done);

	/* Enable update acknowledge interrupt */
	regs->RTC_IER = RTC_IER_ACKEN_Msk;

	/* Request update */
	regs->RTC_CR = (RTC_CR_UPDTIM_Msk | RTC_CR_UPDCAL_Msk);

	/* Await update acknowledge */
	if (k_sem_take(&dev_data->upd_ack_done, K_MSEC(1100)) < 0) {
		regs->RTC_CR = 0u;

		/* Disable update acknowledge interrupt */
		regs->RTC_IDR = RTC_IDR_ACKDIS_Msk;

		k_mutex_unlock(&dev_data->lock);
		return -EAGAIN;
	}

	regs->RTC_TIMR = rtc_timr_from_tm(timeptr);
	regs->RTC_CALR = rtc_calr_from_tm(timeptr);
	regs->RTC_CR = 0u;
	regs->RTC_IDR = RTC_IDR_ACKDIS_Msk;

	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static int rtc_mchp_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;
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

	timeptr->tm_sec = bcd2bin(REG_GET_FIELD(timr0, TIMR_SEC));
	timeptr->tm_min = bcd2bin(REG_GET_FIELD(timr0, TIMR_MIN));
	timeptr->tm_hour = bcd2bin(REG_GET_FIELD(timr0, TIMR_HOUR));
	timeptr->tm_mday = bcd2bin(REG_GET_FIELD(calr0, CALR_DATE));
	timeptr->tm_mon = bcd2bin(REG_GET_FIELD(calr0, CALR_MONTH)) - 1;

	timeptr->tm_year = bcd2bin(REG_GET_FIELD(calr0, CALR_YEAR));
	timeptr->tm_year += ((int)bcd2bin(REG_GET_FIELD(calr0, CALR_CENT))) * 100;
	timeptr->tm_year -= 1900;

	timeptr->tm_wday = bcd2bin(REG_GET_FIELD(calr0, CALR_DAY)) - 1;
	timeptr->tm_yday = -1;
	timeptr->tm_isdst = -1;
	timeptr->tm_nsec = 0;
	return 0;
}

#if defined(CONFIG_RTC_ALARM)
static uint16_t rtc_alarm_get_supported_mask(void)
{
	return (RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |
		RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_MONTH);
}

static uint32_t rtc_timalr_from_tm(const struct rtc_time *timeptr, uint32_t mask)
{
	uint32_t timalr = 0u;

	if ((mask & RTC_ALARM_TIME_MASK_SECOND) != 0u) {
		timalr |= RTC_TIMALR_SECEN_Msk;
		timalr |= RTC_TIMALR_SEC(bin2bcd(timeptr->tm_sec));
	}

	if ((mask & RTC_ALARM_TIME_MASK_MINUTE) != 0u) {
		timalr |= RTC_TIMALR_MINEN_Msk;
		timalr |= RTC_TIMALR_MIN(bin2bcd(timeptr->tm_min));
	}

	if ((mask & RTC_ALARM_TIME_MASK_HOUR) != 0u) {
		timalr |= RTC_TIMALR_HOUREN_Msk;
		timalr |= RTC_TIMALR_HOUR(bin2bcd(timeptr->tm_hour));
	}

	return timalr;
}

static uint32_t rtc_calalr_from_tm(const struct rtc_time *timeptr, uint32_t mask)
{
	uint32_t calalr = RTC_CALALR_MONTH(1) | RTC_CALALR_DATE(1);

	if ((mask & RTC_ALARM_TIME_MASK_MONTH) != 0u) {
		calalr |= RTC_CALALR_MTHEN_Msk;
		calalr |= RTC_CALALR_MONTH(bin2bcd(timeptr->tm_mon + 1));
	}

	if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) != 0u) {
		calalr |= RTC_CALALR_DATEEN_Msk;
		calalr |= RTC_CALALR_DATE(bin2bcd(timeptr->tm_mday));
	}

	return calalr;
}

static uint32_t rtc_alarm_mask_from_timalr(uint32_t timalr)
{
	uint32_t mask = 0u;

	if ((timalr & RTC_TIMALR_SECEN_Msk) != 0u) {
		mask |= RTC_ALARM_TIME_MASK_SECOND;
	}

	if ((timalr & RTC_TIMALR_MINEN_Msk) != 0u) {
		mask |= RTC_ALARM_TIME_MASK_MINUTE;
	}

	if ((timalr & RTC_TIMALR_HOUREN_Msk) != 0u) {
		mask |= RTC_ALARM_TIME_MASK_HOUR;
	}

	return mask;
}

static uint32_t rtc_alarm_mask_from_calalr(uint32_t calalr)
{
	uint32_t mask = 0u;

	if ((calalr & RTC_CALALR_MTHEN_Msk) != 0) {
		mask |= RTC_ALARM_TIME_MASK_MONTH;
	}

	if ((calalr & RTC_CALALR_DATEEN_Msk) != 0) {
		mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
	}

	return mask;
}

static void rtc_tm_from_timalr_calalr(struct rtc_time *timeptr, uint32_t mask, uint32_t timalr,
				      uint32_t calalr)
{
	memset(timeptr, 0u, sizeof(*timeptr));

	if ((mask & RTC_ALARM_TIME_MASK_SECOND) != 0u) {
		timeptr->tm_sec = bcd2bin(REG_GET_FIELD(timalr, TIMALR_SEC));
	}

	if ((mask & RTC_ALARM_TIME_MASK_MINUTE) != 0u) {
		timeptr->tm_min = bcd2bin(REG_GET_FIELD(timalr, TIMALR_MIN));
	}

	if ((mask & RTC_ALARM_TIME_MASK_HOUR) != 0u) {
		timeptr->tm_hour = bcd2bin(REG_GET_FIELD(timalr, TIMALR_HOUR));
	}

	if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) != 0u) {
		timeptr->tm_mday = bcd2bin(REG_GET_FIELD(calalr, CALALR_DATE));
	}

	if ((mask & RTC_ALARM_TIME_MASK_MONTH) != 0u) {
		timeptr->tm_mon = bcd2bin(REG_GET_FIELD(calalr, CALALR_MONTH)) - 1;
	}
}

static int rtc_mchp_alarm_get_supported_fields(const struct device *dev, uint16_t id,
					       uint16_t *mask)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	*mask = rtc_alarm_get_supported_mask();
	return 0;
}

static int rtc_mchp_alarm_set_time(const struct device *dev, uint16_t id, uint16_t mask,
				   const struct rtc_time *timeptr)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;
	uint32_t timalr = 0u;
	uint32_t calalr = 0u;
	uint32_t mask_supported = 0u;

	mask_supported = rtc_alarm_get_supported_mask();

	if ((id != 0)) {
		return -EINVAL;
	}

	if ((mask > 0) && (timeptr == NULL)) {
		return -EINVAL;
	}

	if ((mask & ~mask_supported) != 0u) {
		return -EINVAL;
	}

	if (!rtc_utils_validate_rtc_time(timeptr, mask)) {
		return -EINVAL;
	}

	timalr = rtc_timalr_from_tm(timeptr, mask);
	calalr = rtc_calalr_from_tm(timeptr, mask);

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	irq_disable(dev_config->irq_num);

	/* Set RTC alarm time */
	regs->RTC_TIMALR = timalr;
	regs->RTC_CALALR = calalr;

	/* Clear alarm pending status */
	regs->RTC_SCCR = RTC_SCCR_ALRCLR_Msk;

	irq_enable(dev_config->irq_num);

	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static int rtc_mchp_alarm_get_time(const struct device *dev, uint16_t id, uint16_t *mask,
				   struct rtc_time *timeptr)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;
	uint32_t timalr = 0u;
	uint32_t calalr = 0u;

	if ((id != 0u) || (mask == NULL) || (timeptr == NULL)) {
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	timalr = regs->RTC_TIMALR;
	calalr = regs->RTC_CALALR;

	k_mutex_unlock(&dev_data->lock);

	*mask = rtc_alarm_mask_from_timalr(timalr);
	*mask |= rtc_alarm_mask_from_calalr(calalr);

	rtc_tm_from_timalr_calalr(timeptr, *mask, timalr, calalr);

	return 0;
}

static int rtc_mchp_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	if (id != 0u) {
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	if ((regs->RTC_SR & RTC_SR_ALARM_Msk) == 0u) {
		k_mutex_unlock(&dev_data->lock);
		return 0;
	}

	regs->RTC_SCCR = RTC_SCCR_ALRCLR_Msk;
	k_mutex_unlock(&dev_data->lock);

	return 1;
}

static int rtc_mchp_alarm_set_callback(const struct device *dev, uint16_t id,
				       rtc_alarm_callback callback, void *user_data)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	if (id != 0u) {
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	irq_disable(dev_config->irq_num);
	dev_data->alarm_callback = callback;
	dev_data->alarm_callback_user_data = user_data;

	if (dev_data->alarm_callback) {
		regs->RTC_IER = RTC_IER_ALREN_Msk;
	} else {
		regs->RTC_IDR = RTC_IDR_ALRDIS_Msk;
	}

	irq_enable(dev_config->irq_num);

	k_mutex_unlock(&dev_data->lock);

	return 0;
}
#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)
static int rtc_mchp_update_set_callback(const struct device *dev, rtc_update_callback callback,
					void *user_data)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	irq_disable(dev_config->irq_num);

	dev_data->update_callback = callback;
	dev_data->update_callback_user_data = user_data;

	if (dev_data->update_callback != NULL) {
		regs->RTC_IER = RTC_IER_SECEN_Msk;
	} else {
		regs->RTC_IDR = RTC_IDR_SECDIS_Msk;
	}

	irq_enable(dev_config->irq_num);

	k_mutex_unlock(&dev_data->lock);

	return 0;
}
#endif /* CONFIG_RTC_UPDATE */

#if defined(CONFIG_RTC_CALIBRATION)
static int rtc_mchp_set_calibration(const struct device *dev, int32_t calibration)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;
	bool negative_calibration = false;
	bool high_calibration = false;
	uint32_t slow_clock_calibration = 0u;
	uint32_t mr = 0u;

	if ((calibration < RTC_CALIBRATE_PPB_MIN) || (calibration > RTC_CALIBRATE_PPB_MAX)) {
		return -EINVAL;
	}

	/* The value written to the register is absolute */
	if (calibration < 0) {
		negative_calibration = true;
		calibration = -calibration;
	} else {
		negative_calibration = false;
	}

	/*
	 * Formula adapted from
	 * PIC32CZ-CA70-MC70-Family-Data-Sheet-DS60001825.pdf
	 * section 28.6.2
	 *
	 * Formula if RTC_MR_HIGHPPM is 0
	 *
	 *   RTC_MR_CORRECTION = (3906 / (20 * ppm)) - 1
	 *
	 * Formula if RTC_MR_HIGHPPM is 1
	 *
	 *   RTC_MR_CORRECTION = (3906 / ppm) - 1
	 *
	 * Since we are working with ppb, we adapt the formula by increasing the
	 * terms of the fraction by 1000, turning the ppm into ppb
	 *
	 * Adapted formula if RTC_MR_HIGHPPM is 0
	 *
	 *   RTC_MR_CORRECTION = (3906000 / (20 * ppb)) - 1
	 *
	 * Adapted formula  if RTC_MR_HIGHPPM is 1
	 *
	 *   RTC_MR_CORRECTION = (3906000 / ppb) - 1
	 */
	if (calibration < RTC_CALIBRATE_PPB_QUANTA) {
		high_calibration = false;
		slow_clock_calibration = 0u;
	} else if (calibration < RTC_CALIBRATE_PPB_LOW_SCALE) {
		high_calibration = false;
		slow_clock_calibration = (uint32_t)((3906000 / (20 * calibration)) - 1);
	} else {
		high_calibration = true;
		slow_clock_calibration = (uint32_t)((3906000 / calibration) - 1);
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	mr = regs->RTC_MR;

	if (negative_calibration) {
		mr |= RTC_MR_NEGPPM_Msk;
	} else {
		mr &= ~RTC_MR_NEGPPM_Msk;
	}

	mr &= ~RTC_MR_CORRECTION_Msk;
	mr |= RTC_MR_CORRECTION(slow_clock_calibration);

	if (high_calibration) {
		mr |= RTC_MR_HIGHPPM_Msk;
	} else {
		mr &= ~RTC_MR_HIGHPPM_Msk;
	}

	regs->RTC_MR = mr;

	k_mutex_unlock(&dev_data->lock);

	return 0;
}

static int rtc_mchp_get_calibration(const struct device *dev, int32_t *calibration)
{
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;
	struct rtc_mchp_data *dev_data = dev->data;
	uint32_t mr = 0u;
	int32_t correction = 0;

	if (calibration == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&dev_data->lock, K_FOREVER);

	mr = regs->RTC_MR;

	correction = (int32_t)REG_GET_FIELD(mr, MR_CORRECTION);

	/* Formula documented in rtc_mchp_set_calibration() */
	if (correction == 0) {
		*calibration = 0;
	} else if ((mr & RTC_MR_HIGHPPM_Msk) != 0) {
		*calibration = 3906000 / (correction + 1);
	} else {
		*calibration = 3906000 / ((correction + 1) * 20);
	}

	if ((mr & RTC_MR_NEGPPM_Msk) != 0) {
		*calibration = -*calibration;
	}

	k_mutex_unlock(&dev_data->lock);

	return 0;
}
#endif /* CONFIG_RTC_CALIBRATION */

static DEVICE_API(rtc, rtc_mchp_driver_api) = {
	.set_time = rtc_mchp_set_time,
	.get_time = rtc_mchp_get_time,
#if defined(CONFIG_RTC_ALARM)
	.alarm_get_supported_fields = rtc_mchp_alarm_get_supported_fields,
	.alarm_set_time = rtc_mchp_alarm_set_time,
	.alarm_get_time = rtc_mchp_alarm_get_time,
	.alarm_is_pending = rtc_mchp_alarm_is_pending,
	.alarm_set_callback = rtc_mchp_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */
#if defined(CONFIG_RTC_UPDATE)
	.update_set_callback = rtc_mchp_update_set_callback,
#endif /* CONFIG_RTC_UPDATE */
#if defined(CONFIG_RTC_CALIBRATION)
	.set_calibration = rtc_mchp_set_calibration,
	.get_calibration = rtc_mchp_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */
};

static int rtc_mchp_init(const struct device *dev)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	regs->RTC_MR &= ~(RTC_MR_HRMOD_Msk | RTC_MR_PERSIAN_Msk);
	regs->RTC_CR = 0u;
	regs->RTC_IDR = (RTC_IDR_ACKDIS_Msk | RTC_IDR_ALRDIS_Msk | RTC_IDR_SECDIS_Msk |
			 RTC_IDR_TIMDIS_Msk | RTC_IDR_CALDIS_Msk | RTC_IDR_TDERRDIS_Msk);

	k_mutex_init(&dev_data->lock);
	k_sem_init(&dev_data->sec_evt_done, 0, 1);
	k_sem_init(&dev_data->upd_ack_done, 0, 1);

	dev_config->irq_config(dev);

	irq_enable(dev_config->irq_num);

	return 0;
}

static void rtc_mchp_isr(const struct device *dev)
{
	struct rtc_mchp_data *dev_data = dev->data;
	const struct rtc_mchp_config *dev_config = dev->config;
	rtc_registers_t *const regs = dev_config->regs;

	uint32_t sr = regs->RTC_SR & regs->RTC_IMR;

	if ((sr & RTC_SR_ACKUPD_Msk) != 0u) {
		regs->RTC_SCCR = RTC_SCCR_ACKCLR_Msk;
		k_sem_give(&dev_data->upd_ack_done);
	}

#if defined(CONFIG_RTC_ALARM)
	if ((sr & RTC_SR_ALARM_Msk) != 0u) {
		regs->RTC_SCCR = RTC_SCCR_ALRCLR_Msk;
		if (dev_data->alarm_callback != NULL) {
			dev_data->alarm_callback(dev, 0, dev_data->alarm_callback_user_data);
		}
	}
#endif /* CONFIG_RTC_ALARM */

	if ((sr & RTC_SR_SEC_Msk) != 0u) {
		regs->RTC_SCCR = RTC_SCCR_SECCLR_Msk;
#if defined(CONFIG_RTC_UPDATE)
		if (dev_data->update_callback != NULL) {
			dev_data->update_callback(dev, dev_data->update_callback_user_data);
		}
#endif /* CONFIG_RTC_UPDATE */

		k_sem_give(&dev_data->sec_evt_done);
	}
}

#define DRIVER_INSTANCE_DEFINITION(n)                                                              \
	static void irq_config_##n(const struct device *dev)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), rtc_mchp_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
	}                                                                                          \
                                                                                                   \
	static const struct rtc_mchp_config mchp_rtc_##n##_config = {                              \
		.regs = (rtc_registers_t *)DT_INST_REG_ADDR(n),                                    \
		.irq_num = DT_INST_IRQN(n),                                                        \
		.irq_config = irq_config_##n,                                                      \
	};                                                                                         \
                                                                                                   \
	static struct rtc_mchp_data mchp_rtc_##n##_data;                                           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, rtc_mchp_init, NULL, &mchp_rtc_##n##_data,                        \
			      &mchp_rtc_##n##_config, POST_KERNEL, CONFIG_RTC_INIT_PRIORITY,       \
			      &rtc_mchp_driver_api)

DT_INST_FOREACH_STATUS_OKAY(DRIVER_INSTANCE_DEFINITION);
