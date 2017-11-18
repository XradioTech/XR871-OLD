/**
  * @file  drv_motor_ctrl.c
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
#include "driver/component/motor/drv_motor_ctrl.h"
#include "kernel/os/os.h"

#define MOTOR_CTRL_DBG		0
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_MOTOR_CTRL_DBG(fmt, arg...)	\
			LOG(MOTOR_CTRL_DBG, "[MOTOR CTRL] "fmt, ##arg)

static Motor_Ctrl MotorCtrlPriv = {10000, PWM_GROUP3_CH6, MOTOR_HIGH_LEVEL};

/**
  * @brief Init motor ctrl io.
  * @param param: Set the pwm freq, channel and motor type.
  * @retval The motor speed max value.
  */
motor_max_speed DRV_Motor_Ctrl_Init(Motor_Ctrl *param)
{
	MotorCtrlPriv = *param;
	HAL_Status ret = HAL_ERROR;
	PWM_ClkParam clk_cfg;

	clk_cfg.clk = PWM_CLK_HOSC;
	clk_cfg.div =  PWM_SRC_CLK_DIV_1;

	ret = HAL_PWM_GroupClkCfg(MotorCtrlPriv.pwm_Ch / 2, &clk_cfg);
	if (ret != HAL_OK)
		COMPONENT_WARN("group clk cfg error\n");

	PWM_ChInitParam ch_cfg;
	ch_cfg.hz = MotorCtrlPriv.hz;
	ch_cfg.mode = PWM_CYCLE_MODE;

	if (MotorCtrlPriv.type == MOTOR_HIGH_LEVEL)
		ch_cfg.polarity = PWM_HIGHLEVE;
	else
		ch_cfg.polarity = PWM_LOWLEVE;

	int cycle = HAL_PWM_ChInit(MotorCtrlPriv.pwm_Ch, &ch_cfg);
	if (cycle == -1)
		COMPONENT_WARN("channel init error\n");

	COMPONENT_TRACK("end\n");
	return cycle;
}

/**
  * @brief Deinit motor ctrl io.
  * @retval None.
  */
void DRV_Motor_Ctrl_DeInit()
{
	HAL_PWM_ChDeinit(MotorCtrlPriv.pwm_Ch);
}

/**
  * @brief Enable motor.
  * @retval None.
  */
void DRV_Motor_Enable()
{
	HAL_Status ret = HAL_ERROR;
	ret = HAL_PWM_EnableCh(MotorCtrlPriv.pwm_Ch, PWM_CYCLE_MODE, 1);
	if (ret != HAL_OK)
		COMPONENT_WARN("enable channel error\n");
}

/**
  * @brief Disable motor.
  * @retval None.
  */
void DRV_Motor_Disable()
{
	HAL_Status ret = HAL_ERROR;
	ret = HAL_PWM_EnableCh(MotorCtrlPriv.pwm_Ch, PWM_CYCLE_MODE, 0);
	if (ret != HAL_OK)
		COMPONENT_WARN("disable channel error\n");
}

/**
  * @brief Set the speed for motor.
  * @retval None.
  */
Component_Status DRV_Motor_Speed_Ctrl(uint32_t speed)
{
	HAL_Status ret = HAL_ERROR;

	ret = HAL_PWM_ChSetDutyRatio(MotorCtrlPriv.pwm_Ch, speed);
	if (ret != HAL_OK) {
		COMPONENT_WARN("channel set duty ratio error\n");
		return COMP_ERROR;
	}

	return COMP_OK;
}

void Motor_test()
{
	uint32_t i = 0;
	uint32_t speed = DRV_Motor_Ctrl_Init(&MotorCtrlPriv);
	DRV_Motor_Enable();
	DRV_MOTOR_CTRL_DBG("max speed value = %d\n", speed);

	for (i = 0; i <= speed ; i += 5) {
		DRV_Motor_Speed_Ctrl(i);
		OS_MSleep(1);
	}

	DRV_Motor_Speed_Ctrl(0);
	OS_MSleep(500);

	for (i = 0; i <= speed ; i += 5) {
		DRV_Motor_Speed_Ctrl(i);
		OS_MSleep(1);
	}

	DRV_Motor_Speed_Ctrl(0);
	DRV_Motor_Ctrl_DeInit();
}

