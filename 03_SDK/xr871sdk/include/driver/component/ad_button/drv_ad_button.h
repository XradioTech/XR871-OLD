/**
  * @file  drv_ad_button.h
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

#ifndef _AD_BUTTON_H_
#define _AD_BUTTON_H_

#include "driver/component/component_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_adc.h"
#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief AD button callback set and irq mode set.
  */
typedef struct {
	/*!< The callback for ad button. irq_sta is interrupt status, ad_Value is ad capture value */
	void (*buttonCallback) (void *arg, ADC_IRQState irq_sta, uint32_t ad_Value);
	void *arg;
}AD_Button_Irq;

typedef struct {
	ADC_Channel channel;				/*!< The ad channel used for ad button*/
	ADC_IRQMode ad_Button_Irq_Mode;		/*!< The channel's interrupt mode */
	uint32_t lowValue;					/*!< lower limit value in interrupt mode of ADC_IRQ_LOW,
                                                                      ADC_IRQ_LOW_DATA, ADC_IRQ_LOW_HIGH or ADC_IRQ_LOW_HIGH_DATA*/
	uint32_t highValue;				    /*!< Upper limit value in interrupt mode of ADC_IRQ_HIGH,
                                                                      ADC_IRQ_HIGH_DATA, ADC_IRQ_LOW_HIGH or ADC_IRQ_LOW_HIGH_DATA*/
}AD_Button_Config;


void DRV_AD_ButtonCallBackRegister(AD_Button_Irq *irq);
Component_Status DRV_EnableAD_Button();
Component_Status DRV_DisableAD_Button();
Component_Status DRV_AD_ButtonInit(AD_Button_Config *ad_button_info);
Component_Status DRV_AD_ButtonDeInit();

#ifdef __cplusplus
}
#endif

#endif /* _AD_BUTTON_H_*/
