/**
  * @file  drv_ad_button.c
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
#include "kernel/os/os_queue.h"
#include "kernel/os/os.h"
#include "driver/chip/hal_def.h"
#include "driver/component/ad_button/drv_ad_button.h"

#define AD_BUTTON_DBG 0

#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)


#define DRV_AD_BUTTON_DBG(fmt, arg...)	\
			//LOG(AD_BUTTON_DBG, "[AD BUTTON] "fmt, ##arg)

#define BUTTON_NUMBER 1

static AD_Button_Irq AD_Button_Private = {NULL, NULL};
ADC_IRQMode Private_ADC_Irq = ADC_IRQ_LOW;
static AD_Button_Config AD_Button;

static uint32_t AD_d_Value(uint32_t v1, uint32_t v2)
{
	uint32_t d_value = 0;
	if (v1 > v2)
		d_value = v1 - v2;
	else
		d_value = v2 - v1;

	return d_value;
}

#define RELUCATION 30
#define AD_COUNT_NUM 10

static uint32_t AD_button_filter(uint32_t ad_value)
{
	static uint32_t ad_value_sum = 0;
	static uint32_t ad_count = 0;
	uint32_t aver_value = 0;
	uint32_t d_value = 0;
	uint8_t ad_flag = 0;

	ad_value_sum += ad_value;
	ad_count ++;
	if (ad_count >= AD_COUNT_NUM) {
		aver_value = ad_value_sum / AD_COUNT_NUM;
		ad_value_sum = 0;
		ad_count = 0;
		ad_flag = 1;
	} else
		aver_value = ad_value_sum / ad_count;

	d_value = AD_d_Value(aver_value, ad_value);
	if (d_value > RELUCATION) {
		ad_value_sum = 0;
		ad_count = 0;
		ad_flag = 0;
	} else {
		if (ad_flag)
			return aver_value;
	}
	return 0;
}

static void AD_Button_Cb(void *arg)
{
	ADC_IRQState irq_sta = HAL_ADC_GetIRQState(AD_Button.channel);
	if (irq_sta == ADC_LOW_IRQ) {
		uint32_t ad_value = AD_button_filter( HAL_ADC_GetValue(AD_Button.channel));
		if (ad_value) {
			if (AD_Button_Private.buttonCallback)
				AD_Button_Private.buttonCallback(AD_Button_Private.arg,
										 irq_sta, ad_value);

			Private_ADC_Irq = ADC_IRQ_LOW_HIGH;
			HAL_ADC_ConfigChannel(AD_Button.channel, ADC_SELECT_ENABLE, Private_ADC_Irq,
						     	 ad_value - 50, AD_Button.highValue);
		}

	} else if (irq_sta == ADC_HIGH_IRQ) {
		if (AD_Button_Private.buttonCallback)
			AD_Button_Private.buttonCallback(AD_Button_Private.arg,
										 irq_sta, HAL_ADC_GetValue(AD_Button.channel));
		Private_ADC_Irq = ADC_IRQ_LOW;
		HAL_ADC_ConfigChannel(AD_Button.channel, ADC_SELECT_ENABLE, Private_ADC_Irq,
						      AD_Button.lowValue, AD_Button.highValue);
	}
}


/**
  * @brief register the callback for ad button.
  * @note This function is used to rigister the callback,when you push the button
  *           and the button is enable, the callback will be run.
  * @param irq: Config callback.
  * @retval None
  */
void DRV_AD_ButtonCallBackRegister(AD_Button_Irq *irq)
{
	 AD_Button_Private.buttonCallback = irq->buttonCallback;
	 AD_Button_Private.arg = irq->arg;
}

/**
  * @brief Enable ad button.
  * @note This function is used to enable the ad button, if ad button is enable,
  *           when you push the button, the interrupt will be tigger.
  * @retval Component_Status: The status of driver.
  */
Component_Status DRV_EnableAD_Button()
{
	HAL_ADC_EnableIRQCallback(AD_Button.channel, AD_Button_Cb, NULL);
	return COMP_OK;
}

/**
  * @brief Disable ad button.
  * @note This function is used to disable the ad button, if ad button is disable,
  *           when you push the button, the interrupt don't tigger.
  * @retval Component_Status: The status of driver.
  */
Component_Status DRV_DisableAD_Button()
{
	HAL_ADC_DisableIRQCallback(AD_Button.channel);
	return COMP_OK;
}

/**
  * @brief Init the ad button.
  * @note This function is used to configure the ad button pin and interrut triggering conditions.
  * @param ad_button_info:
  *        @arg ad_button_info->channel:The channels used for ad button.
  *        @arg ad_button_info->ad_Button_Irq_Mode: interrupt triggering conditions.
  *        @arg ad_button_info->lowValue:  lower limit value in interrupt mode of ADC_IRQ_LOW,
  *            ADC_IRQ_LOW_DATA, ADC_IRQ_LOW_HIGH or ADC_IRQ_LOW_HIGH_DATA
  *        @arg ad_button_info->highValue: Upper limit value in interrupt mode of ADC_IRQ_HIGH,
  *            ADC_IRQ_HIGH_DATA, ADC_IRQ_LOW_HIGH or ADC_IRQ_LOW_HIGH_DATA
  * @retval Component_Status: The status of driver.
  */
Component_Status DRV_AD_ButtonInit(AD_Button_Config *ad_button_info)
{
	AD_Button = *ad_button_info;
	ADC_InitParam initParam;
	initParam.delay = 10;
	initParam.freq = 500000;
	initParam.mode = ADC_CONTI_CONV;

	HAL_Status sta = HAL_ADC_Init(&initParam);
	if (sta == HAL_OK || sta == HAL_BUSY) {
		HAL_ADC_ConfigChannel(AD_Button.channel, ADC_SELECT_ENABLE, AD_Button.ad_Button_Irq_Mode,
						      AD_Button.lowValue, AD_Button.highValue);
		DRV_EnableAD_Button();
		if (sta == HAL_OK)
			HAL_ADC_Start_Conv_IT();
		return COMP_OK;
	} else
		COMPONENT_WARN("AD init error %d\n", sta);
	COMPONENT_TRACK("end\n");
	return COMP_ERROR;
}

/**
  * @brief DeInit ad button.
  * @note This function is used to deinit the ad button pin, and disable ad button.
  * @retval Component_Status: The status of driver.
  */
Component_Status DRV_AD_ButtonDeInit()
{
	DRV_DisableAD_Button();
	HAL_ADC_ConfigChannel(AD_Button.channel, ADC_SELECT_DISABLE, ADC_IRQ_NONE,
						  AD_Button.lowValue, AD_Button.highValue);
	COMPONENT_TRACK("end\n");
	return COMP_OK;
}

