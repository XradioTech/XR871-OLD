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
#include "driver/chip/hal_adc.h"

#define ADC_IT_MODE
//#define ADC_POLL_MODE
//#define ADC_TEST_FIFO

#define ADC_IRQ_MODE 	ADC_IRQ_DATA

#define ADC_INCH 		ADC_CHANNEL_1
#define ADC_FEQ 		500000
#define ADC_FIRST_DELAY 10
#define	ADC_FIFO_LEVEL	32

void adc_callback(void *arg)
{
	HAL_Status status = HAL_ERROR;

#ifdef ADC_TEST_FIFO
	uint8_t i, num;
	uint16_t fifo[64];
	num = HAL_ADC_GetFifoDataCount();
	for(i = 0; i <= num; i++)
		fifo[i] = HAL_ADC_GetFifoData();
	printf("fifo irq mode: adc value = %d\n", fifo[0]);

	status = HAL_ADC_FifoConfigChannel(ADC_INCH, ADC_SELECT_DISABLE);
#else
	if (HAL_ADC_GetIRQState(ADC_INCH) == ADC_DATA_IRQ)
		printf("chan irq mode: adc value = %d\n", HAL_ADC_GetValue(ADC_INCH));

	status = HAL_ADC_ConfigChannel(ADC_INCH, ADC_SELECT_DISABLE, ADC_IRQ_MODE, 0, 0);
#endif
	if (status != HAL_OK)
		printf("ADC config error %d\n", status);
}

void adc_init()
{
	HAL_Status status = HAL_ERROR;
	ADC_InitParam initParam;

	initParam.delay = ADC_FIRST_DELAY;
	initParam.freq = ADC_FEQ;
#ifdef ADC_TEST_FIFO
	initParam.mode = ADC_BURST_CONV;
#else
	initParam.mode = ADC_CONTI_CONV;
#endif
	status = HAL_ADC_Init(&initParam);
	if (status != HAL_OK) {
		printf("ADC init error %d\n", status);
		return;
	}

#ifdef ADC_IT_MODE
#ifdef ADC_TEST_FIFO
	status = HAL_ADC_FifoConfigChannel(ADC_INCH, ADC_SELECT_ENABLE);
#else
	status = HAL_ADC_ConfigChannel(ADC_INCH, ADC_SELECT_ENABLE, ADC_IRQ_MODE, 0, 0);
#endif
	if (status != HAL_OK) {
		printf("ADC config error %d\n", status);
		return;
	}

	status = HAL_ADC_EnableIRQCallback(ADC_INCH, adc_callback, NULL);
	if (status != HAL_OK) {
		printf("ADC IRQ Enable error %d\n", status);
		return;
	}

	status = HAL_ADC_Start_Conv_IT();
	if (status != HAL_OK) {
		printf("ADC it mode start error %d\n", status);
		return;
	}
#endif
}

void adc_deinit()
{
	HAL_Status status = HAL_ERROR;

#ifdef ADC_IT_MODE
#ifdef ADC_TEST_FIFO
	status = HAL_ADC_FifoConfigChannel(ADC_INCH, ADC_SELECT_DISABLE);
#else
	status = HAL_ADC_ConfigChannel(ADC_INCH, ADC_SELECT_DISABLE, ADC_IRQ_MODE, 0, 0);
#endif
	if (status != HAL_OK)
		printf("ADC config error %d\n", status);

	status = HAL_ADC_Stop_Conv_IT();
	if (status != HAL_OK)
		printf("ADC it mode stop error %d\n", status);

 	status = HAL_ADC_DisableIRQCallback(ADC_INCH);
	if (status != HAL_OK)
		printf("ADC cb disable error %d\n", status);
#endif

	status = HAL_ADC_DeInit();
	if (status != HAL_OK)
		printf("ADC deinit error %d\n", status);
}

/* run this demo, please connect the sensor board. */
int main(void)
{
	printf("ADC demo started.\n");

	HAL_Status status = HAL_ERROR;
	adc_init();

	while (1) {
#ifdef ADC_IT_MODE
#ifdef ADC_TEST_FIFO
		status = HAL_ADC_FifoConfigChannel(ADC_INCH, ADC_SELECT_ENABLE);
#else
		status =HAL_ADC_ConfigChannel(ADC_INCH, ADC_SELECT_ENABLE, ADC_IRQ_MODE, 0, 0);
#endif
		if (status != HAL_OK) {
			printf("ADC config error %d\n", status);
			break;
		}
#else
		uint32_t ad_value = 0;
		status = HAL_ADC_Conv_Polling(ADC_INCH, &ad_value, 10000);
		if (status != HAL_OK) {
			printf("ADC poll error %d\n", status);
		}
		printf("poll mode: adc value = %d\n", ad_value);
		#endif
		OS_MSleep(500);
	}

	adc_deinit();
	printf("ADC demo over.\n");
	return 0;
}
