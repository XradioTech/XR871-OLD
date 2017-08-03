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
#include "driver/chip/hal_pwm.h"

#define OUTPUT_CHANNEL PWM_GROUP0_CH0
#define INPUT_CHANNEL PWM_GROUP0_CH1

void cycle_irq(void *arg, void *irq_sta)
{
	printf("this is pwm output irq!\n");
	HAL_PWM_OUT_IRQDisable(OUTPUT_CHANNEL);
}

void cycle_mode()
{
	HAL_Status status = HAL_ERROR;

	PWM_Init_Param io_param;
	io_param.ch = OUTPUT_CHANNEL;
	HAL_PWM_IO_Init(&io_param);

	PWM_Output_Init param;
	param.ch = OUTPUT_CHANNEL;
	param.hz = 1000;
	param.polarity = PWM_HIGHLEVE;

	PWM_SrcClk src_clk;
	src_clk.chGroup = OUTPUT_CHANNEL / 2;
	src_clk.srcClk = PWM_CLK_HOSC;
	src_clk.srcClkDiv = PWM_SRC_CLK_DIV_1;
	param.srcClkActualFreq = HAL_PWM_SrcClkInit(&src_clk);

	status = HAL_PWM_CycleModeInit(&param);
	if (status != HAL_OK)
		printf("pwm cycle mode init error %d\n", status);

	uint16_t cycle_value = HAL_PWM_GetEnterCycleValue(OUTPUT_CHANNEL);
	printf("pwm out period : %d\n", cycle_value);
	printf("pwm out high level %d\n\n", cycle_value / 2);
	status = HAL_PWM_SetDutyRatio(OUTPUT_CHANNEL, cycle_value / 2);
	if (status != HAL_OK)
		printf("pwm set duty ratio error %d\n", status);

	 status = HAL_PWM_OutModeEnableCh(OUTPUT_CHANNEL);
	 if (status != HAL_OK)
		printf("pwm enable channel error %d\n", status);

	HAL_PWM_ModuleIRQEnable();
	PWM_OutIRQ irq;
	irq.arg = NULL;
	irq.callBack = cycle_irq;
	irq.ch = OUTPUT_CHANNEL;
	HAL_PWM_OutIRQInit(&irq);
	HAL_PWM_OUT_IRQEnable(OUTPUT_CHANNEL);
}

void capture_mode()
{
	HAL_Status status = HAL_ERROR;
	PWM_Init_Param io_param;
	io_param.ch = INPUT_CHANNEL;
	HAL_PWM_IO_Init(&io_param);

	PWM_Input_Init param;
	param.ch = INPUT_CHANNEL;
	param.chClkDiv = 1;

	PWM_SrcClk src_clk;
	src_clk.chGroup = INPUT_CHANNEL / 2;
	src_clk.srcClk = PWM_CLK_HOSC;
	src_clk.srcClkDiv = PWM_SRC_CLK_DIV_1;
	param.srcClkActualFreq = HAL_PWM_SrcClkInit(&src_clk);
	status = HAL_PWM_InputInit(&param);
	if (status != HAL_OK)
		printf("pwm input mode init error\n");

	status = HAL_PWM_InputEnableCh(INPUT_CHANNEL);
	if (status != HAL_OK)
		printf("input ch enable error\n");

	HAL_PWM_ModuleIRQEnable();
	PWM_InputIRQ captureIRQ;
	captureIRQ.arg = NULL;
	captureIRQ.callBack = NULL;
	captureIRQ.ch = INPUT_CHANNEL;
	HAL_PWM_InputIRQInit(&captureIRQ);
	HAL_PWM_InputIRQEnable(PWM_IRQ_BOTHEDGE, INPUT_CHANNEL);
}

void capture_value()
{
	PWM_squareWaveInfo data = HAL_PWM_CaptureResult(INPUT_CHANNEL);
	if (data.highLevelTime && data.lowLevelTime) {
		printf("caputre result:\n");
		printf("\thigh level %d\n", data.highLevelTime);
		printf("\tlow level  %d\n", data.lowLevelTime);
		printf("\tpreiod     %d\n", data.periodTime);
	}
}

/*Run this demo, please connect the PA8 and PA9.*/
int main()
{
	printf("PWM demo started.\n\n");

	cycle_mode();
	OS_MSleep(10);
	capture_mode();

	uint8_t count = 3;
	while (count --) {
		capture_value();
		OS_MSleep(500);
	}

	PWM_Init_Param io_param;
	io_param.ch = OUTPUT_CHANNEL;
	HAL_PWM_DeInit(&io_param);

	io_param.ch = INPUT_CHANNEL;
	HAL_PWM_DeInit(&io_param);

	printf("\nPWM demo over.\n");

	return 0;
}

