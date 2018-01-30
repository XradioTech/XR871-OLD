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
#include "driver/chip/hal_timer.h"

#define TIMERID TIMER0_ID

uint32_t sec_count;
void timer_callback()
{
	sec_count++;
	printf(" timer irq: %ds\n", sec_count);
}

void timer_init()
{
	HAL_Status status = HAL_ERROR;
	TIMER_InitParam param;
	param.arg = NULL;
	param.callback = timer_callback;
	param.cfg = HAL_TIMER_MakeInitCfg(TIMER_MODE_REPEAT,
				TIMER_CLK_SRC_HFCLK, TIMER_CLK_PRESCALER_4);
	param.isEnableIRQ = 1;
	param.period = 6000000; //1s

	status = HAL_TIMER_Init(TIMERID, &param);
	if (status != HAL_OK)
		printf("timer int error %d\n", status);

}

int main(void)
{
	printf("timer demo started.\n\n");

	HAL_Status status = HAL_ERROR;

	timer_init();

	HAL_TIMER_Start(TIMERID);
	printf("timer%d start.\n", TIMERID);
	printf("the timer count down 1s\n");
	printf("wait timer irq\n");

	while (1) {
		OS_MSleep(10000);
	}

	HAL_TIMER_Stop(TIMERID);
	status = HAL_TIMER_DeInit(TIMERID);
	if (status != HAL_OK)
		printf("timer deinit error %d\n", status);

	printf("timer demo over.\n");

	return 0;
}
