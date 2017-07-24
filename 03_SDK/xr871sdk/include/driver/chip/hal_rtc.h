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

#ifndef _DRIVER_CHIP_HAL_RTC_H_
#define _DRIVER_CHIP_HAL_RTC_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	__IO uint32_t CTRL;					/*!<RTC control register						Address offset: 0x0000*/
	     uint32_t RESERVED0[3];		/*!<reserved, 0x0004,0x0008,0x000C							 */
	__IO uint32_t YYMMDD;					/*!<RTC Year Month Day Register				Address offset: 0x0010*/
	__IO uint32_t DDHHMMSS;					/*!<RTC Hour Minute second register				Address offset: 0x0014*/
	     uint32_t RESERVED1[2];		/*!<reserved,0x0018,0x001C							 */
	__IO uint32_t SEC_ALARM_LOAD_VAL;			/*!<alarm 0 counter register					Address offset: 0x0020*/
	__IO uint32_t SEC_ALARM_CUR_VAL; /* increase from zero */			/*!<alarm 0 current value register				Address offset: 0x0024*/
	__IO uint32_t SEC_ALARM_EN;				/*!<alarm 0 enable register 					Address offset: 0x0028*/
	__IO uint32_t SEC_ALARM_IRQ_EN;			/*!<alarm 0 IRQ enable register					Address offset: 0x002C*/
	__IO uint32_t SEC_ALARM_IRQ_STATUS;			/*!<alarm 0 IRQ status register					Address offset: 0x0030*/
	     uint32_t RESERVED2[3];		/*!<reserved, 0x0034,0x0038,0x003C							 */
	__IO uint32_t WDAY_ALARM_HHMMSS;			/*!<alarm 1 week hour minute second register		Address offset: 0x0040*/
	__IO uint32_t WDAY_ALARM_WDAY_EN;				/*!<alarm 1 enable register						Address offset: 0x0044*/
	__IO uint32_t WDAY_ALARM_IRQ_EN;			/*!<alarm 1 IRQ enable register					Address offset: 0x0048*/
	__IO uint32_t WDAY_ALARM_IRQ_STATUS;			/*!<alarm 1 IRQ status register					Address offset: 0x004C*/
	     uint32_t RESERVED3[4];             /*!<reserved, 0x0050,0x0054,0x0058,0x5C */
	__IO uint32_t FREERUN_CNT_L;            /*!<free running counter bit[31:0], Address offset: 0x0060*/
	__IO uint32_t FREERUN_CNT_H;            /*!<free running counter bit[48:32], Address offset: 0x0064*/
} RTC_T;

#define RTC		((RTC_T *)RTC_BASE)

/*
 * bit field definition of RTC->CTRL
 */
#define RTC_TEST_MODE_BIT					HAL_BIT(31)
#define RTC_SIMULATION_BIT					HAL_BIT(30)
#define RTC_WDAY_ALARM_HHMMSS_ACCESS_BIT	HAL_BIT(2)
#define RTC_DDHHMMSS_ACCESS_BIT				HAL_BIT(1)
#define RTC_YYMMDD_ACCESS_BIT				HAL_BIT(0)

/*
 * bit field definition of RTC->YYMMDD
 */
#define RTC_LEAP_YEAR_BIT	HAL_BIT(24)

#define RTC_YEAR_SHIFT	16	/* R/W, [0, 255] */
#define RTC_YEAR_VMASK	0xFF
#define RTC_YEAR_MIN	0
#define RTC_YEAR_MAX	255

#define RTC_MONTH_SHIFT	8	/* R/W, [1, 12] */
#define RTC_MONTH_VMASK	0xF
#define RTC_MONTH_MIN	1
#define RTC_MONTH_MAX	12

#define RTC_MDAY_SHIFT	0	/* R/W, [1, 31] */
#define RTC_MDAY_VMASK	0x1F
#define RTC_MDAY_MIN	1
#define RTC_MDAY_MAX	31

/*
 * bit field definition of RTC->DDHHMMSS
 */
#define RTC_WDAY_SHIFT	29	/* R/W */
#define RTC_WDAY_VMASK	0x7
typedef enum {
	RTC_WDAY_MONDAY	   = 0U,
	RTC_WDAY_TUESDAY   = 1U,
	RTC_WDAY_WEDNESDAY = 2U,
	RTC_WDAY_THURSDAY  = 3U,
	RTC_WDAY_FRIDAY	   = 4U,
	RTC_WDAY_SATURDAY  = 5U,
	RTC_WDAY_SUNDAY	   = 6U
} RTC_WeekDay;

#define RTC_HOUR_SHIFT		16	/* R/W, [0, 23] */
#define RTC_HOUR_VMASK		0x1F
#define RTC_HOUR_MIN		0
#define RTC_HOUR_MAX		23

#define RTC_MINUTE_SHIFT	8	/* R/W, [0, 59] */
#define RTC_MINUTE_VMASK	0x3F
#define RTC_MINUTE_MIN		0
#define RTC_MINUTE_MAX		59

#define RTC_SECOND_SHIFT	0	/* R/W, [0, 59] */
#define RTC_SECOND_VMASK	0x3F
#define RTC_SECOND_MIN		0
#define RTC_SECOND_MAX		59

/* RTC->SEC_ALARM_LOAD_VAL */

/* RTC->SEC_ALARM_CUR_VAL */

/* RTC->SEC_ALARM_EN */
#define RTC_SEC_ALARM_EN_BIT	HAL_BIT(0)

/* RTC->SEC_ALARM_IRQ_EN */
#define RTC_SEC_ALARM_IRQ_EN_BIT	HAL_BIT(0)

/* RTC->SEC_ALARM_IRQ_STATUS */
#define RTC_SEC_ALARM_IRQ_PENDING_BIT	HAL_BIT(0)

/* RTC->WDAY_ALARM_DDHHMMSS */
#define RTC_WDAY_ALARM_HOUR_SHIFT	16	/* R/W, [0, 23] */
#define RTC_WDAY_ALARM_HOUR_VMASK	0x1F

#define RTC_WDAY_ALARM_MINUTE_SHIFT	8	/* R/W, [0, 59] */
#define RTC_WDAY_ALARM_MINUTE_VMASK	0x3F

#define RTC_WDAY_ALARM_SECOND_SHIFT	0	/* R/W, [0, 59] */
#define RTC_WDAY_ALARM_SECOND_VMASK	0x3F

/* RTC->WDAY_ALARM_EN */
#define RTC_WDAY_ALARM_EN_BIT(wday)	HAL_BIT(wday)	/* day is RTC_WeekDay */
#define RTC_WDAY_ALARM_EN_MASK		0x7F

/* RTC->WDAY_ALARM_IRQ_EN */
#define RTC_WDAY_ALARM_IRQ_EN_BIT	HAL_BIT(0)

/* RTC->WDAY_ALARM_IRQ_STATUS */
#define RTC_WDAY_ALARM_IRQ_PENDING_BIT	HAL_BIT(0)

/******************************************************************************/

typedef void (*RTC_AlarmIRQCallback) (void *arg);

typedef struct {
	uint32_t 				alarmSeconds;	/* alarm when down count to zero */
	RTC_AlarmIRQCallback	callback;
	void         	       *arg;
} RTC_SecAlarmStartParam;

typedef struct {
	uint8_t 				alarmHour;
	uint8_t 				alarmMinute;
	uint8_t 				alarmSecond;
	uint8_t					alarmWDayMask;	/* bit mask of RTC_WDAY_ALARM_EN_BIT(RTC_WeekDay) */
	RTC_AlarmIRQCallback	callback;
	void         	       *arg;
} RTC_WDayAlarmStartParam;

void HAL_RTC_SetYYMMDD(uint8_t isLeapYear, uint8_t year, uint8_t month, uint8_t mday);
void HAL_RTC_SetDDHHMMSS(RTC_WeekDay wday, uint8_t hour, uint8_t minute, uint8_t second);

void HAL_RTC_GetYYMMDD(uint8_t *isLeapYear, uint8_t *year, uint8_t *month, uint8_t *mday);
void HAL_RTC_GetDDHHMMSS(RTC_WeekDay *wday, uint8_t *hour, uint8_t *minute, uint8_t *second) ;

void HAL_RTC_StartSecAlarm(const RTC_SecAlarmStartParam *param);
void HAL_RTC_StopSecAlarm(void);

void HAL_RTC_StartWDayAlarm(const RTC_WDayAlarmStartParam *param);
void HAL_RTC_StopWDayAlarm(void);

uint64_t HAL_RTC_Get32kConter(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_RTC_H_ */
