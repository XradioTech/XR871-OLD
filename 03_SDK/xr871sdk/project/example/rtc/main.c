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

#include <stdio.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_rtc.h"

void analysis_wday(RTC_WeekDay wday, char *buf)
{
	if (wday == 0)
		sprintf(buf, "%s", "monday");
	else if (wday== 1)
		sprintf(buf, "%s", "tuesday");
	else if (wday == 2)
		sprintf(buf, "%s", "wednesday");
	else if (wday == 3)
		sprintf(buf, "%s", "thursday");
	else if (wday == 4)
		sprintf(buf, "%s", "friday");
	else if (wday == 5)
		sprintf(buf, "%s", "saturday");
	else if (wday == 6)
		sprintf(buf, "%s", "sunday");
}

void rtc_set_time()
{
	/*set time : year mouth day hour minute second*/
	printf("set time : 201-10-12, tuesday,12:21:30\n");
	HAL_RTC_SetYYMMDD(0, 201, 10, 12);
	HAL_RTC_SetDDHHMMSS(RTC_WDAY_TUESDAY, 12, 21, 30);
}


void rtc_read_time()
{
	printf("read time:");

	uint8_t isLeapYear, year, mouth, mday;

	RTC_WeekDay wday;
	uint8_t hour, minute, second;

	HAL_RTC_GetYYMMDD(&isLeapYear, &year, &mouth, &mday);
	HAL_RTC_GetDDHHMMSS(&wday, &hour, &minute, &second);

	char buf[10];
	analysis_wday(wday, buf);

	if (isLeapYear)
		printf("is Leap Year\n");

	printf("%d-%d-%d, ", year, mouth, mday);
	printf("%s,%d:%d:%d\n", buf, hour, minute, second);
}

void wday_alarm_callback(void *arg)
{
	rtc_read_time();
	HAL_RTC_StopWDayAlarm();
	printf("wday alarm is arrive!!!\n\n");
}

void wday_alarm()
{
	printf("\nset wday alarm 5\n");
	RTC_WDayAlarmStartParam wday_param;
	wday_param.alarmHour = 12;
	wday_param.alarmMinute = 21;
	wday_param.alarmSecond = 35;
	wday_param.alarmWDayMask = 0;
	wday_param.alarmWDayMask = RTC_WDAY_ALARM_EN_BIT(RTC_WDAY_TUESDAY);
	wday_param.arg = NULL;
	wday_param.callback = wday_alarm_callback;
	HAL_RTC_StartWDayAlarm(&wday_param);
}

void sec_alarm_callback(void *arg)
{
	rtc_read_time();
	HAL_RTC_StopSecAlarm();
	printf("second alarm is arrive!!!\n\n");
}

void sec_alarm()
{
	printf("set sec alarm 10s\n");
	RTC_SecAlarmStartParam sec_param;
	sec_param.alarmSeconds = 10;
	sec_param.arg = NULL;
	sec_param.callback = sec_alarm_callback;
	HAL_RTC_StartSecAlarm(&sec_param);
}

int main(void)
{
	printf("rtc demo started\n\n");
	/*waiting for  rtc stability*/
	OS_MSleep(200);

	rtc_set_time();
	rtc_read_time();

	wday_alarm();
	sec_alarm();
	printf("wait irq\n\n");

	while (1) {
		OS_MSleep(1000);
	}

	return 0;
}
