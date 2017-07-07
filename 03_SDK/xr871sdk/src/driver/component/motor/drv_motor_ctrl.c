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


HAL_Status board_pwm_cfg(uint32_t id, HAL_BoardReq req, void *arg);

typedef enum {
	MOTOR_HIGH_LEVEL,
	MOTOR_LOW_LEVEL,
}MOTOR_CTRL_TYPE;

typedef struct {
	uint32_t hz;
	PWM_CHID pwm_Ch;
	MOTOR_CTRL_TYPE type;
}Motor_Ctrl;

Motor_Ctrl Motor_ctrl = {10000, PWM_GROUP3_CH6, MOTOR_HIGH_LEVEL};

/**************************************************************
***************************************************************/

motor_max_speed DRV_Motor_Ctrl_Init()
{
	PWM_Init_Param pwm_Param;
	pwm_Param.boardCfg = board_pwm_cfg;
	pwm_Param.ch = Motor_ctrl.pwm_Ch;
	HAL_PWM_IO_Init(&pwm_Param);

	PWM_SrcClk srcClkSet;
	PWM_Output_Init PwmOutputSet;

	srcClkSet.chGroup= Motor_ctrl.pwm_Ch / 2;
	srcClkSet.srcClkDiv= PWM_SRC_CLK_DIV_1;
	srcClkSet.srcClk= PWM_CLK_HOSC;


	PwmOutputSet.ch= Motor_ctrl.pwm_Ch;
	if (Motor_ctrl.type == MOTOR_HIGH_LEVEL)
		PwmOutputSet.polarity= PWM_HIGHLEVE;
	else
		PwmOutputSet.polarity= PWM_LOWLEVE;
	PwmOutputSet.hz = Motor_ctrl.hz;
	PwmOutputSet.srcClkActualFreq = HAL_PWM_SrcClkInit(&srcClkSet);

	HAL_Status sta = HAL_PWM_CycleModeInit(&PwmOutputSet);
	if (sta != HAL_OK) {
		COMPONENT_WARN("PWM init error %d\n", sta);
		return 0;
	}

	COMPONENT_TRACK("end\n");
	return HAL_PWM_GetEnterCycleValue(Motor_ctrl.pwm_Ch);
}

void DRV_Motor_Ctrl_DeInit()
{
	PWM_Init_Param pwm_Param;
	pwm_Param.boardCfg = board_pwm_cfg;
	pwm_Param.ch = Motor_ctrl.pwm_Ch;
	HAL_PWM_DeInit(&pwm_Param);
}

void DRV_Motor_Enable()
{
	HAL_PWM_OutModeEnableCh(Motor_ctrl.pwm_Ch);
}

void DRV_Motor_Disable()
{
	HAL_PWM_OutModeDisableCh(Motor_ctrl.pwm_Ch);
}

Component_Status DRV_Morot_Speed_Ctrl(uint32_t speed)
{
	if (HAL_PWM_SetDutyRatio(Motor_ctrl.pwm_Ch, speed) == HAL_OK)
		return COMP_OK;
	return COMP_ERROR;
}

void Motor_test()
{
	uint32_t i = 0;
	uint32_t speed = DRV_Motor_Ctrl_Init();
	DRV_Motor_Enable();
	DRV_MOTOR_CTRL_DBG("max speed value = %d\n", speed);

	for (i = 0; i <= speed ; i += 5) {
		DRV_Morot_Speed_Ctrl(i);
		OS_MSleep(1);
	}

	DRV_Morot_Speed_Ctrl(0);
	OS_MSleep(500);

	for (i = 0; i <= speed ; i += 5) {
		DRV_Morot_Speed_Ctrl(i);
		OS_MSleep(1);
	}

	DRV_Morot_Speed_Ctrl(0);
	DRV_Motor_Ctrl_DeInit();
}

