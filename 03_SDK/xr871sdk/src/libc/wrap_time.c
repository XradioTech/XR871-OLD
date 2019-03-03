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

#include <time.h>
#include <sys/time.h>
#include "driver/chip/hal_rtc.h"
#include "driver/chip/hal_prcm.h"

#define USEC_PER_SEC	1000000L /* microseconds per second */

/* variable to save time baseline of h/w timer */
static int64_t s_tmbl = 1514764800000000LL; /* default to 2018-01-01 00:00:00 */

#ifndef __CONFIG_BOOTLOADER
void timeofday_save(void)
{
	HAL_PRCM_SetCPUAPrivateData((uint32_t)(s_tmbl & 0xffffffff));
	HAL_PRCM_SetCPUABootAddr((uint32_t)((s_tmbl >> 32) & 0xffffffff));
}

void timeofday_restore(void)
{
	uint32_t high = HAL_PRCM_GetCPUABootAddr();
	uint32_t low = HAL_PRCM_GetCPUAPrivateData();
	if (high != 0 || low != 0) {
		s_tmbl = ((int64_t)high << 32) | low;
	}
}
#endif

int __wrap_settimeofday(const struct timeval *tv, const struct timezone *tz)
{
	int64_t tm;

	if (tv) {
		tm = HAL_RTC_GetFreeRunTime();
		s_tmbl = (int64_t)tv->tv_sec * USEC_PER_SEC + tv->tv_usec - tm;
	}

	return 0;
}

int __wrap_gettimeofday(struct timeval *tv, void *tz)
{
	int64_t tm;

	if (tv) {
		tm = HAL_RTC_GetFreeRunTime() + s_tmbl;
		tv->tv_sec = tm / USEC_PER_SEC;
		tv->tv_usec = tm % USEC_PER_SEC;
	}

	return 0;
}

time_t __wrap_time(time_t *t)
{
	struct timeval tv;

	__wrap_gettimeofday(&tv, NULL);
	if (t) {
		*t = tv.tv_sec;
	}

	return tv.tv_sec;
}
