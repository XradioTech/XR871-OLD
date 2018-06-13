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

void cycle_irq(void *arg,  PWM_IrqEvent event)
{
	printf("this is pwm output irq!\n");
	HAL_PWM_DisableIRQ(OUTPUT_CHANNEL);
}

void cycle_mode()
{
	HAL_Status status = HAL_ERROR;
	PWM_ClkParam clk_param;
	PWM_ChInitParam ch_param;
	PWM_IrqParam irq_param;
	int max_duty_ratio = 0;

	clk_param.clk = PWM_CLK_HOSC;
	clk_param.div = PWM_SRC_CLK_DIV_1;

	status = HAL_PWM_GroupClkCfg(OUTPUT_CHANNEL / 2, &clk_param);
	if (status != HAL_OK)
		printf("%s(): %d, PWM group clk config error\n", __func__, __LINE__);

	ch_param.hz = 1000;
	ch_param.mode = PWM_CYCLE_MODE;
	ch_param.polarity = PWM_HIGHLEVE;
	max_duty_ratio = HAL_PWM_ChInit(OUTPUT_CHANNEL, &ch_param);
	if (max_duty_ratio == -1)
		printf("%s(): %d, PWM ch init error\n", __func__, __LINE__);

	status = HAL_PWM_ChSetDutyRatio(OUTPUT_CHANNEL, max_duty_ratio / 2);
	if (status != HAL_OK)
		printf("%s(): %d, PWM set duty ratio error\n", __func__, __LINE__);

	status = HAL_PWM_EnableCh(OUTPUT_CHANNEL, PWM_CYCLE_MODE, 1);
	if (status != HAL_OK)
		printf("%s(): %d, PWM ch enable error\n", __func__, __LINE__);

	irq_param.arg = NULL;
	irq_param.callback = cycle_irq;
	irq_param.event = PWM_IRQ_OUTPUT;
	status = HAL_PWM_EnableIRQ(OUTPUT_CHANNEL, &irq_param);
	if (status != HAL_OK)
		printf("%s(): %d, PWM enable irq error\n", __func__, __LINE__);
}

void capture_mode()
{
	HAL_Status status = HAL_ERROR;
	PWM_ClkParam clk_param;
	PWM_ChInitParam ch_param;
	PWM_IrqParam irq_param;

	clk_param.clk = PWM_CLK_HOSC;
	clk_param.div = PWM_SRC_CLK_DIV_1;

	status = HAL_PWM_GroupClkCfg(INPUT_CHANNEL / 2, &clk_param);
	if (status != HAL_OK)
		printf("%s(): %d, PWM group clk config error\n", __func__, __LINE__);

	ch_param.hz = 1000;
	ch_param.mode = PWM_CAPTURE_MODE;
	ch_param.polarity = PWM_HIGHLEVE;

	if (HAL_PWM_ChInit(INPUT_CHANNEL, &ch_param) == -1)
		printf("%s(): %d, PWM ch init error\n", __func__, __LINE__);

	status = HAL_PWM_EnableCh(INPUT_CHANNEL, PWM_CAPTURE_MODE, 1);
	if (status != HAL_OK)
		printf("%s(): %d, PWM ch enable error\n", __func__, __LINE__);

	irq_param.arg = NULL;
	irq_param.callback = NULL;
	irq_param.event = PWM_IRQ_BOTHEDGE;
	status = HAL_PWM_EnableIRQ(INPUT_CHANNEL, &irq_param);
	if (status != HAL_OK)
		printf("%s(): %d, PWM enable irq error\n", __func__, __LINE__);
}

int capture_value()
{
	PWM_CapResult data = HAL_PWM_CaptureResult(PWM_CAP_CYCLE, INPUT_CHANNEL);

	if (data.periodTime) {
		printf("caputre result:\n");
		printf("\thigh level %d\n", data.highLevelTime);
		printf("\tlow level  %d\n", data.lowLevelTime);
		printf("\tpreiod     %d\n", data.periodTime);
		return 1;
	}

	return 0;
}

/*Run this demo, please connect the PA8 and PA9.*/
int main(void)
{
	printf("PWM demo started\n");

	cycle_mode();
	OS_MSleep(10);
	capture_mode();

	uint8_t count = 3;
	while (count) {
		if (capture_value())
			count --;
		OS_MSleep(500);
	}

	/*Stop channel*/
	HAL_PWM_EnableCh(INPUT_CHANNEL, PWM_CYCLE_MODE, 0);
	HAL_PWM_EnableCh(INPUT_CHANNEL, PWM_CAPTURE_MODE, 0);

	/*Deinit channel*/
	HAL_PWM_ChDeinit(OUTPUT_CHANNEL);
	HAL_PWM_ChDeinit(INPUT_CHANNEL);

	printf("\nPWM demo over.\n");

	return 0;
}

