/**
  * @file  drv_flash_led.c
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

#include <string.h>
#include <stdio.h>
#include "driver/component/flash_led/drv_flash_led.h"
#include "kernel/os/os.h"

Flash_Led_Info Flash_Led_Ctrl = {100000, PWM_GROUP0_CH0, COMMON_CATHODE};

/**
  * @brief Flash led init.
  * @note This function is used to config the flash led freq and pin.
  * @param param: The pwm channel that used for ctrl led.
  * @retval  success :The led max brightness value, error -1.
  */
Flash_Led_MaxBrightness DRV_Flash_Led_Init(Flash_Led_Info *param)
{
	Flash_Led_Ctrl = *param;
	HAL_Status ret = HAL_ERROR;
	PWM_ClkParam clk_cfg;

	clk_cfg.clk = PWM_CLK_HOSC;
	clk_cfg.div =  PWM_SRC_CLK_DIV_1;

	ret = HAL_PWM_GroupClkCfg(Flash_Led_Ctrl.pwm_Ch / 2, &clk_cfg);
	if (ret != HAL_OK)
		COMPONENT_WARN("group clk cfg error\n");

	PWM_ChInitParam ch_cfg;
	ch_cfg.hz = Flash_Led_Ctrl.hz;
	ch_cfg.mode = PWM_CYCLE_MODE;

	if (Flash_Led_Ctrl.type == COMMON_ANODE)
		ch_cfg.polarity = PWM_HIGHLEVE;
	else
		ch_cfg.polarity = PWM_LOWLEVE;

	int cycle = HAL_PWM_ChInit(Flash_Led_Ctrl.pwm_Ch, &ch_cfg);
	if (cycle == -1)
		COMPONENT_WARN("channel init error\n");

	COMPONENT_TRACK("end\n");

	return cycle;
}

/**
  * @brief Flash led deinit.
  * @note This function is used to led ctrl pin.
  * @retval  None.
  */
void DRV_Flash_Led_DeInit()
{
	HAL_PWM_ChDeinit(Flash_Led_Ctrl.pwm_Ch);
}

/**
  * @brief Enable the led.
  * @retval  None.
  */
void DRV_Flash_LedEnable()
{
	HAL_PWM_EnableCh(Flash_Led_Ctrl.pwm_Ch, PWM_CYCLE_MODE, 1);
}

/**
  * @brief Disable the led.
  * @retval  None.
  */
void DRV_Flash_LedDisable()
{
	HAL_PWM_EnableCh(Flash_Led_Ctrl.pwm_Ch, PWM_CYCLE_MODE, 0);
}

/**
  * @brief Set led brightness.
  * @note This function is used to set led brightness.
  * @param brightNess: The brightness for led.
  * @retval  Driver status.
  */
Component_Status DRV_Flash_LedBrightness(uint32_t brightness)
{
	if (HAL_PWM_ChSetDutyRatio(Flash_Led_Ctrl.pwm_Ch, brightness) == HAL_OK)
		return COMP_OK;

	return COMP_ERROR;
}

void Flash_led_test()
{
	uint32_t i = 0;
	uint32_t bright = DRV_Flash_Led_Init(&Flash_Led_Ctrl);
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
