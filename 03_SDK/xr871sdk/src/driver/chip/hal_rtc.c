/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "driver/chip/hal_rtc.h"
#include "hal_inc.h"

typedef struct {
	RTC_AlarmIRQCallback	callback;
	void         		   *arg;
} RTC_AlarmPrivate;

static RTC_AlarmPrivate gRtcSecAlarmPriv;
static RTC_AlarmPrivate gRtcWDayAlarmPriv;

__STATIC_INLINE int RTC_WDayAlarmIsHHMMSSReadable(void)
{
	return !HAL_GET_BIT(RTC->CTRL, RTC_WDAY_ALARM_HHMMSS_ACCESS_BIT);
}

static int RTC_IsDDHHMMSSReadable(void)
{
	return !HAL_GET_BIT(RTC->CTRL, RTC_DDHHMMSS_ACCESS_BIT);
}

static int RTC_IsYYMMDDReadable(void)
{
	return !HAL_GET_BIT(RTC->CTRL, RTC_YYMMDD_ACCESS_BIT);
}

static void RTC_SecAlarmSetAlarmTime(uint32_t sec)
{
	RTC->SEC_ALARM_LOAD_VAL = sec;
}

__STATIC_INLINE uint32_t RTC_SecAlarmGetCurrentTime(void)
{
	return RTC->SEC_ALARM_CUR_VAL;
}

static void RTC_SecAlarmEnableIRQ(void)
{
	HAL_SET_BIT(RTC->SEC_ALARM_IRQ_EN, RTC_SEC_ALARM_IRQ_EN_BIT);
}

static void RTC_SecAlarmDisableIRQ(void)
{
	HAL_CLR_BIT(RTC->SEC_ALARM_IRQ_EN, RTC_SEC_ALARM_IRQ_EN_BIT);
}

static int RTC_SecAlarmIsPendingIRQ(void)
{
	return HAL_GET_BIT(RTC->SEC_ALARM_IRQ_STATUS, RTC_SEC_ALARM_IRQ_PENDING_BIT);
}

static void RTC_SecAlarmClearPendingIRQ(void)
{
	HAL_SET_BIT(RTC->SEC_ALARM_IRQ_STATUS, RTC_SEC_ALARM_IRQ_PENDING_BIT);
}

static void RTC_SecAlarmStart(void)
{
	HAL_SET_BIT(RTC->SEC_ALARM_EN, RTC_SEC_ALARM_EN_BIT);
}

static void RTC_SecAlarmStop(void)
{
	HAL_CLR_BIT(RTC->SEC_ALARM_EN, RTC_SEC_ALARM_EN_BIT);
}

static void RTC_WDayAlarmSetAlarmTime(uint8_t hour, uint8_t minute, uint8_t second)
{
	HAL_ASSERT_PARAM(hour >= RTC_HOUR_MIN && hour <= RTC_HOUR_MAX);
	HAL_ASSERT_PARAM(minute >= RTC_MINUTE_MIN && minute <= RTC_MINUTE_MAX);
	HAL_ASSERT_PARAM(second >= RTC_SECOND_MIN && second <= RTC_SECOND_MAX);

	RTC->WDAY_ALARM_HHMMSS =
		(((uint32_t)hour & RTC_WDAY_ALARM_HOUR_VMASK) << RTC_WDAY_ALARM_HOUR_SHIFT) |
		(((uint32_t)minute & RTC_WDAY_ALARM_MINUTE_VMASK) << RTC_WDAY_ALARM_MINUTE_SHIFT) |
		(((uint32_t)second & RTC_WDAY_ALARM_SECOND_VMASK) << RTC_WDAY_ALARM_SECOND_SHIFT);
}

static uint32_t RTC_WDayAlarmSetAlarmDay(uint8_t wdayMask)
{
	return RTC->WDAY_ALARM_WDAY_EN = wdayMask & RTC_WDAY_ALARM_EN_MASK;
}

static void RTC_WDayAlarmEnableIRQ(void)
{
	HAL_SET_BIT(RTC->WDAY_ALARM_IRQ_EN, RTC_WDAY_ALARM_IRQ_EN_BIT);
}

static void RTC_WDayAlarmDisableIRQ(void)
{
	HAL_CLR_BIT(RTC->WDAY_ALARM_IRQ_EN, RTC_WDAY_ALARM_IRQ_EN_BIT);
}

static int RTC_WDayAlarmIsPendingIRQ(void)
{
	return HAL_GET_BIT(RTC->WDAY_ALARM_IRQ_STATUS, RTC_WDAY_ALARM_IRQ_PENDING_BIT);
}

static void RTC_WDayAlarmClearPendingIRQ(void)
{
	HAL_SET_BIT(RTC->WDAY_ALARM_IRQ_STATUS, RTC_WDAY_ALARM_IRQ_PENDING_BIT);
}

void RTC_SecAlarm_IRQHandler()
{
	RTC_SecAlarmStop();
	if (RTC_SecAlarmIsPendingIRQ()) {
		RTC_SecAlarmClearPendingIRQ();
		if (gRtcSecAlarmPriv.callback) {
			gRtcSecAlarmPriv.callback(gRtcSecAlarmPriv.arg);
		}
	}
}

void RTC_WDayAlarm_IRQHandler()
{
	if (RTC_WDayAlarmIsPendingIRQ()) {
		RTC_WDayAlarmClearPendingIRQ();
		if (gRtcWDayAlarmPriv.callback) {
			gRtcWDayAlarmPriv.callback(gRtcWDayAlarmPriv.arg);
		}
	}
}

/* leap year is set by user, never set by h/w? */
void HAL_RTC_SetYYMMDD(uint8_t isLeapYear, uint8_t year, uint8_t month, uint8_t mday)
{
	HAL_ASSERT_PARAM(year <= RTC_YEAR_MAX);
	HAL_ASSERT_PARAM(month >= RTC_MONTH_MIN && month <= RTC_MONTH_MAX);
	HAL_ASSERT_PARAM(mday >= RTC_MDAY_MIN && month <= RTC_MDAY_MAX);

	RTC->YYMMDD = (isLeapYear ? RTC_LEAP_YEAR_BIT : 0) |
				  (((uint32_t)year & RTC_YEAR_VMASK) << RTC_YEAR_SHIFT) |
				  (((uint32_t)month & RTC_MONTH_VMASK) << RTC_MONTH_SHIFT) |
				  (((uint32_t)mday & RTC_MDAY_VMASK) << RTC_MDAY_SHIFT);
}

void HAL_RTC_SetDDHHMMSS(RTC_WeekDay wday, uint8_t hour, uint8_t minute, uint8_t second)
{
	HAL_ASSERT_PARAM(hour >= RTC_HOUR_MIN && hour <= RTC_HOUR_MAX);
	HAL_ASSERT_PARAM(minute >= RTC_MINUTE_MIN && minute <= RTC_MINUTE_MAX);
	HAL_ASSERT_PARAM(second >= RTC_SECOND_MIN && second <= RTC_SECOND_MAX);

	RTC->DDHHMMSS = (((uint32_t)wday & RTC_WDAY_VMASK) << RTC_WDAY_SHIFT) |
				    (((uint32_t)hour & RTC_HOUR_VMASK) << RTC_HOUR_SHIFT) |
				    (((uint32_t)minute & RTC_MINUTE_VMASK) << RTC_MINUTE_SHIFT) |
				    (((uint32_t)second & RTC_SECOND_VMASK) << RTC_SECOND_SHIFT);
}

void HAL_RTC_GetYYMMDD(uint8_t *isLeapYear, uint8_t *year, uint8_t *month, uint8_t *mday)
{
	uint32_t yymmdd;

	while (!RTC_IsYYMMDDReadable()) {
		HAL_MSleep(1);
	}

	yymmdd = RTC->YYMMDD;
	*isLeapYear = HAL_GET_BIT(yymmdd, RTC_LEAP_YEAR_BIT) ? 1 : 0;
	*year = HAL_GET_BIT_VAL(yymmdd, RTC_YEAR_SHIFT, RTC_YEAR_VMASK);
	*month = HAL_GET_BIT_VAL(yymmdd, RTC_MONTH_SHIFT, RTC_MONTH_VMASK);
	*mday = HAL_GET_BIT_VAL(yymmdd, RTC_MDAY_SHIFT, RTC_MDAY_VMASK);
}

void HAL_RTC_GetDDHHMMSS(RTC_WeekDay *wday, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
	uint32_t ddhhmmss;

	while (!RTC_IsDDHHMMSSReadable()) {
		HAL_MSleep(1);
	}

	ddhhmmss = RTC->DDHHMMSS;
	*wday = (RTC_WeekDay)HAL_GET_BIT_VAL(ddhhmmss, RTC_WDAY_SHIFT, RTC_WDAY_VMASK);
	*hour = HAL_GET_BIT_VAL(ddhhmmss, RTC_HOUR_SHIFT, RTC_HOUR_VMASK);
	*minute = HAL_GET_BIT_VAL(ddhhmmss, RTC_MINUTE_SHIFT, RTC_MINUTE_VMASK);
	*second = HAL_GET_BIT_VAL(ddhhmmss, RTC_SECOND_SHIFT, RTC_SECOND_VMASK);
}

void HAL_RTC_StartSecAlarm(RTC_SecAlarmStartParam *param)
{
	/* reset current value and wait it done */
	RTC_SecAlarmStop();
	while (RTC_SecAlarmGetCurrentTime() != 0) {
		HAL_MSleep(10);
	}

	RTC_SecAlarmSetAlarmTime(param->alarmSeconds);
	gRtcSecAlarmPriv.callback = param->callback;
	gRtcSecAlarmPriv.arg = param->arg;
	if (RTC_SecAlarmIsPendingIRQ()) {
		RTC_SecAlarmClearPendingIRQ();
	}
	RTC_SecAlarmEnableIRQ();
	HAL_NVIC_SetPriority(RTC_SEC_ALARM_IRQn, NVIC_PERIPHERAL_PRIORITY_DEFAULT);
	HAL_NVIC_EnableIRQ(RTC_SEC_ALARM_IRQn);
	RTC_SecAlarmStart();
}

void HAL_RTC_StopSecAlarm(void)
{
	RTC_SecAlarmStop();
	HAL_NVIC_DisableIRQ(RTC_SEC_ALARM_IRQn);
	RTC_SecAlarmDisableIRQ();
	if (RTC_SecAlarmIsPendingIRQ()) {
		RTC_SecAlarmClearPendingIRQ();
	}
	gRtcSecAlarmPriv.callback = NULL;
	gRtcSecAlarmPriv.arg = NULL;
}

void HAL_RTC_StartWDayAlarm(RTC_WDayAlarmStartParam *param)
{
	RTC_WDayAlarmSetAlarmDay(0);
	RTC_WDayAlarmSetAlarmTime(param->alarmHour, param->alarmMinute, param->alarmSecond);

	gRtcWDayAlarmPriv.callback = param->callback;
	gRtcWDayAlarmPriv.arg = param->arg;
	if (RTC_WDayAlarmIsPendingIRQ()) {
		RTC_WDayAlarmClearPendingIRQ();
	}
	RTC_WDayAlarmEnableIRQ();
	HAL_NVIC_SetPriority(RTC_WDAY_ALARM_IRQn, NVIC_PERIPHERAL_PRIORITY_DEFAULT);
	HAL_NVIC_EnableIRQ(RTC_WDAY_ALARM_IRQn);
	RTC_WDayAlarmSetAlarmDay(param->alarmWDayMask);
}

void HAL_RTC_StopWDayAlarm(void)
{
	RTC_WDayAlarmSetAlarmDay(0);
	HAL_NVIC_DisableIRQ(RTC_WDAY_ALARM_IRQn);
	RTC_WDayAlarmDisableIRQ();
	if (RTC_WDayAlarmIsPendingIRQ()) {
		RTC_WDayAlarmClearPendingIRQ();
	}
	gRtcWDayAlarmPriv.callback = NULL;
	gRtcWDayAlarmPriv.arg = NULL;
}

#if defined(__CONFIG_CHIP_XR871)
/* we assume that: the total time get this counter is less than 1000/32 ms */
uint64_t HAL_RTC_Get32kConter(void)
{
	uint32_t alte = 0;
	uint32_t cnt_32k_l;
	uint32_t cnt_32k_h;
	uint64_t cnt_32k_o = 0, cnt_32k_n = 0;
	unsigned long flags;

	flags = HAL_EnterCriticalSection();

	do {
		cnt_32k_l = RTC->FREERUN_CNT_L;
		cnt_32k_h = RTC->FREERUN_CNT_H;

		if (alte) {
			cnt_32k_n = cnt_32k_h;
			cnt_32k_n = (cnt_32k_n << 32) | cnt_32k_l;
		} else {
			cnt_32k_o = cnt_32k_h;
			cnt_32k_o = (cnt_32k_o << 32) | cnt_32k_l;
		}

		alte ^= 1;
		if (cnt_32k_n && cnt_32k_n < cnt_32k_o + 1000) {
			HAL_ExitCriticalSection(flags);
			return cnt_32k_n;
		}
	} while (1);
}
#endif
