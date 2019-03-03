/**
  * @file  hal_util.c
  * @author  XRADIO IOT WLAN Team
  */

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

#include <stdint.h>

#define HAL_UDELAY_BY_CPU_INSTRUCTION	0

#if (!HAL_UDELAY_BY_CPU_INSTRUCTION)
#include "driver/chip/hal_rtc.h"
#else
#include "driver/chip/hal_clock.h"
#endif

/**
 * @brief Provide accurate delay (in microsecond), and its accuracy is about
 *        32 microseconds.
 * @param[in] us Time (in microsecond) to delay
 * @return None
 *
 * @note Avoid using this function to delay for a long time (longer than 10ms),
 *       because its execution will occupy a lot of CPU resource.
 */
void HAL_UDelay(uint32_t us)
{
#if (!HAL_UDELAY_BY_CPU_INSTRUCTION)
	uint64_t expire;

	expire = HAL_RTC_GetFreeRunCnt() + HAL_RTC_FreeRunTimeToCnt(us);
	while (expire > HAL_RTC_GetFreeRunCnt())
		;
#else /* HAL_UDELAY_BY_CPU_INSTRUCTION */
#define CPUCLK_PER_LOOP 6
	uint32_t i;
	uint32_t loop = HAL_GetCPUClock() / 1000000 * us / CPUCLK_PER_LOOP;

	for (i = 0; i < loop; ++i) {
		__asm("nop");
	}
#undef CPUCLK_PER_LOOP
#endif /* HAL_UDELAY_BY_CPU_INSTRUCTION */
}
