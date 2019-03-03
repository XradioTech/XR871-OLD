/**
  * @file  drv_tcs4056.c
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

#include "stdio.h"
#include "kernel/os/os.h"
#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_adc.h"
#include "driver/component/tcs4056/drv_tcs4056.h"

#define DRV_TCS4056_DBG 0

#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)


#define TCS4056_DBG(fmt, arg...)	\
			//LOG(DRV_TCS4056_DBG, "[TCS4056] "fmt, ##arg)

#define IO_EINTA 6U

static ChrgIrq Chrg_Private;
static ADC_Channel Tcs4056AdcCh = ADC_CHANNEL_VBAT;
static ChrgIo Chrg_Io = {GPIO_PORT_A, GPIO_PIN_21};

CHRG_IRQ_STA Chrg_Status()
{
	GPIO_PinState v = HAL_GPIO_ReadPin(Chrg_Io.chrgPort, Chrg_Io.chrgPin);

	if (v == GPIO_PIN_HIGH)
		return CHRG_HIGH_LEVEL;
	else
		return CHRG_LOW_LEVEL;

	return CHRG_NULL;
}

void Chrg_Irq_Cb()
{
	if (Chrg_Private.chrgCallBack)
		Chrg_Private.chrgCallBack(Chrg_Private.arg, Chrg_Status());
}

void Chrg_Init()
{
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.pull= GPIO_PULL_NONE;
	param.mode = IO_EINTA;
	HAL_GPIO_Init(Chrg_Io.chrgPort,
		          Chrg_Io.chrgPin,&param);
	OS_MSleep(10);
}

void Chrg_DeInit()
{
	DRV_Disable_Chrg_Irq();
	HAL_GPIO_DeInit(Chrg_Io.chrgPort,
		            Chrg_Io.chrgPin);
}


void Bat_Read_Voltage_Cb(void *arg)
{
	static uint32_t ad_value = 0, i = 0;
	uint32_t *value = (uint32_t *)arg;
	ad_value += HAL_ADC_GetValue(Tcs4056AdcCh);
	i++;
	if (i >= 20) {
		*value = ad_value / 20;
		i = 0;
		ad_value = 0;
		HAL_ADC_ConfigChannel(Tcs4056AdcCh, ADC_SELECT_DISABLE,
						  ADC_IRQ_NONE, 0, 0);
	}
}

Component_Status Bat_Init()
{
	ADC_InitParam AD_initParam;
	AD_initParam.delay = 10;
	AD_initParam.freq = 500000;
	AD_initParam.mode = ADC_CONTI_CONV;
	HAL_Status sta = HAL_ADC_Init(&AD_initParam);

	if (sta == HAL_OK || sta == HAL_BUSY) {
		return COMP_OK;
	} else
		COMPONENT_WARN("ADC init error %d\n", sta);

	HAL_ADC_Start_Conv_IT();
	COMPONENT_TRACK("end\n");

	return COMP_OK;
}

/**
  * @brief Register the callback for chrg interrupt.
  * @param cb: callback.
  * @retval None.
  */
void DRV_ChrgCallBackRegister(ChrgIrq *cb)
{
	Chrg_Private.chrgCallBack = cb->chrgCallBack;
	Chrg_Private.arg = cb->arg;
}

/**
  * @brief Enable charg interrupt.
  * @retval None.
  */
void DRV_Enable_Chrg_Irq()
{
	GPIO_IrqParam Irq_param;
	Irq_param.event = GPIO_IRQ_EVT_BOTH_EDGE;
	Irq_param.callback = Chrg_Irq_Cb;
	Irq_param.arg = NULL;
	HAL_GPIO_EnableIRQ(Chrg_Io.chrgPort,
		               Chrg_Io.chrgPin, &Irq_param);
}

/**
  * @brief Disable charg interrupt.
  * @retval None.
  */
void DRV_Disable_Chrg_Irq()
{
	HAL_GPIO_DisableIRQ(Chrg_Io.chrgPort,
		                Chrg_Io.chrgPin);
}

/**
  * @brief Read the bat voltage.
  * @retval The bat voltage.
  */
uint32_t DRV_Read_Bat_Voltage()
{
	uint32_t average = 0;
	while (1) {
		uint32_t time_out_count = 0;
		HAL_ADC_ConfigChannel(Tcs4056AdcCh, ADC_SELECT_ENABLE,ADC_IRQ_DATA, 0, 0);
		HAL_ADC_EnableIRQCallback(Tcs4056AdcCh, Bat_Read_Voltage_Cb, &average);

		while (!average) {
			OS_MSleep(10);
			time_out_count++;
			if(time_out_count >= 50)
				break;
		}

		if(time_out_count >= 50)
			continue;

		HAL_ADC_DisableIRQCallback(Tcs4056AdcCh);
		average = average * 7500 / 4096;
		break;
	}
	return average;
}

/**
  * @brief Config the chrg io and the ad channel that used for read bat voltage.
  * @param chrg_io. chrg io info.
  * @param ch: The ad channel used for read bat voltage.
  * @retval Component_Status : The driver status.
  */
Component_Status DRV_Tcs4056_Init(ChrgIo *chrg_io, ADC_Channel ch)
{
	Tcs4056AdcCh = ch;
	Chrg_Io = *chrg_io;
	Chrg_Init();
	COMPONENT_TRACK("end\n");
	return Bat_Init();
}

/**
  * @brief Deinit chrg io and ad channel.
  * @retval None.
  */
void DRV_Tcs4056_DeInit()
{
	Chrg_DeInit();
	COMPONENT_TRACK("end\n");
}


/**************************example**************************/
void Chrg_Test_Cb(void *arg, CHRG_IRQ_STA sta)
{
	if (sta == CHRG_HIGH_LEVEL)
		printf("%s(), chrg in is high\n", __func__);
	else if (sta == CHRG_LOW_LEVEL)
		printf("%s(), chrg in is low\n", __func__);
}

OS_Thread_t g_tcs4056_test_thread;
#define TCS4056_THREAD_STACK_SIZE	512

void Tcs4056_TestThread()
{
	while (1) {
		printf("bat voltage = %d mv\n", DRV_Read_Bat_Voltage(Tcs4056AdcCh));
		OS_MSleep(1000);
	}
}

void Tcs4056_Test()
{
	DRV_Tcs4056_Init(&Chrg_Io, Tcs4056AdcCh);
	ChrgIrq irq;
	irq.chrgCallBack = Chrg_Test_Cb;
	irq.arg = NULL;
	DRV_ChrgCallBackRegister(&irq);
	DRV_Enable_Chrg_Irq();

	if (OS_ThreadCreate(&g_tcs4056_test_thread,
						"",
						Tcs4056_TestThread,
						NULL,
						OS_THREAD_PRIO_APP,
						TCS4056_THREAD_STACK_SIZE) != OS_OK) {
		printf("create Tcs4056_TestThread failed\n");
	}

}
