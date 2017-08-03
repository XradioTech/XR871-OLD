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
#include "kernel/os/os.h"

#include "../../include/driver/chip/hal_pwm.h"
#include "../../include/driver/chip/hal_gpio.h"
#include "../../include/driver/component/motor/drv_motor_ctrl.h"
#include "../../include/driver/component/rgb_led/drv_rgb_led.h"

#include "manip_ctrl.h"
#include "bbc_main.h"

void plat_ctrl_init()
{
	Rgb_Led_Info led_info;
	led_info.b_Led = PWM_GROUP0_CH0;
	led_info.g_Led = PWM_GROUP0_CH1;
	led_info.r_Led = PWM_GROUP1_CH2;
	led_info.ledFrequency = 500;
	led_info.type = RGB_HIGH_LEVEL_VALID;

	// maxBrightness = 48000
	MaxBrightness maxBrightness = Drv_Rgb_Led_Cfg(&led_info);
	printf("maxBrightness = %d\n",maxBrightness);
	Drv_Rgb_LedEnable();

	// maxspeed = 2400
	motor_max_speed maxSpeed = DRV_Motor_Ctrl_Init();
	printf("maxSpeed = %d\n",maxSpeed);
	DRV_Motor_Enable();
}

void plat_rgbr_ctrl(unsigned int bright)
{
	SetLedBrightness(PWM_GROUP1_CH2, bright);
	SenorUpload.LampRBright = bright / RGB_CONVERT_BMSG;		//record data
}

void plat_rgbg_ctrl(unsigned int bright)
{
	SetLedBrightness(PWM_GROUP0_CH1, bright);
	SenorUpload.LampGBright = bright / RGB_CONVERT_BMSG;		//record data
}

void plat_rgbb_ctrl(unsigned int bright)
{
	SetLedBrightness(PWM_GROUP0_CH0, bright);
	SenorUpload.LampBBright = bright / RGB_CONVERT_BMSG;		//record data
}

void plat_mortor_ctrl(unsigned int speed)
{
	DRV_Morot_Speed_Ctrl(speed);

	SenorUpload.MotorSpeed = speed / MOTOR_CONVERT_BMSG;		//record data
}

