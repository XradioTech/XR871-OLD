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

#include <string.h>
#include <stdio.h>
#include "driver/component/flash_led/drv_flash_led.h"
#include "kernel/os/os.h"

typedef enum {
	COMMON_CATHODE,
	COMMON_ANODE,
}FLASH_LED_TYPE;

typedef struct {
	uint32_t hz;
	PWM_CHID pwm_Ch;
	FLASH_LED_TYPE type;
}Flash_Led_Info;

Flash_Led_Info Flash_Led_Ctrl = {100000, PWM_GROUP0_CH0, COMMON_CATHODE};

/**************************************************************
***************************************************************/

Flash_Led_MaxBrightness DRV_Flash_Led_Init()
{
	PWM_Init_Param pwm_Param;
	pwm_Param.ch = Flash_Led_Ctrl.pwm_Ch;
	HAL_PWM_IO_Init(&pwm_Param);

	PWM_SrcClk srcClkSet;
	PWM_Output_Init PwmOutputSet;

	srcClkSet.chGroup= Flash_Led_Ctrl.pwm_Ch / 2;
	srcClkSet.srcClkDiv= PWM_SRC_CLK_DIV_1;
	srcClkSet.srcClk= PWM_CLK_HOSC;


	PwmOutputSet.ch= Flash_Led_Ctrl.pwm_Ch;
	if (Flash_Led_Ctrl.type == COMMON_CATHODE)
		PwmOutputSet.polarity= PWM_HIGHLEVE;
	else
		PwmOutputSet.polarity= PWM_LOWLEVE;
	PwmOutputSet.hz = Flash_Led_Ctrl.hz;
	PwmOutputSet.srcClkActualFreq = HAL_PWM_SrcClkInit(&srcClkSet);

	HAL_Status sta = HAL_PWM_CycleModeInit(&PwmOutputSet);
	if (sta != HAL_OK) {
		COMPONENT_WARN("PWM init error %d\n", sta);
		return 0;
	}

	COMPONENT_TRACK("end\n");
	return HAL_PWM_GetEnterCycleValue(Flash_Led_Ctrl.pwm_Ch);
}

void DRV_Flash_Led_DeInit()
{
	PWM_Init_Param pwm_Param;
	pwm_Param.ch = Flash_Led_Ctrl.pwm_Ch;
	HAL_PWM_DeInit(&pwm_Param);
}

void DRV_Flash_LedEnable()
{
	HAL_PWM_OutModeEnableCh(Flash_Led_Ctrl.pwm_Ch);
}

void DRV_Flash_LedDisable()
{
	HAL_PWM_OutModeDisableCh(Flash_Led_Ctrl.pwm_Ch);
}

Component_Status DRV_Flash_LedBrightness(uint32_t brightNess)
{
	if (HAL_PWM_SetDutyRatio(Flash_Led_Ctrl.pwm_Ch, brightNess) == HAL_OK)
		return COMP_OK;
	return COMP_ERROR;
}

void Flash_led_test()
{
	uint32_t i = 0;
	uint32_t bright = DRV_Flash_Led_Init();
	DRV_Flash_LedEnable();
	printf("max bright value = %d\n", bright);
	while (1) {
		for (i = 10; i <= bright ; i += 1) {
			DRV_Flash_LedBrightness(i);
			OS_MSleep(20);
		}
		for (i = bright ;i >= 10 ; i -= 1) {
			DRV_Flash_LedBrightness(i);
			OS_MSleep(20);
		}
	}
}

