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
#include "kernel/os/os.h"
#include "stdio.h"
#include "common/board/board.h"
#include "led_flag.h"

uint8_t led_mode = LED_FLAG_NONET;

void gpio_led_on(void)
{
	GPIO_InitParam param;	
	param.driving = GPIO_DRIVING_LEVEL_0;
	param.mode    = GPIOx_Pn_F1_OUTPUT;	
	param.pull    = GPIO_PULL_DOWN;	
	HAL_GPIO_Init(GPIO_PORT_A, GPIO_PIN_19, &param);
}

void gpio_led_off(void)
{
	GPIO_InitParam param;	
	//param.driving = GPIO_DRIVING_LEVEL_0;	
	param.mode    = GPIOx_Pn_F7_DISABLE;	
	param.pull    = GPIO_PULL_UP;	
	HAL_GPIO_Init(GPIO_PORT_A, GPIO_PIN_19, &param);
}

#define XLED_THREAD_STACK_SIZE	512
OS_Thread_t xled_task_thread;
uint8_t xled_task_run = 0;

void led_flag_ctrl(void *arg)
{
	while(xled_task_run) {
		OS_MSleep(200);
		switch(led_mode) {
			case LED_FLAG_NONET:
				OS_MSleep(200);
				gpio_led_on();
				OS_MSleep(200);
				gpio_led_off();
				break;
			case LED_FLAG_SMRCON:
				OS_MSleep(50);
				gpio_led_on();
				OS_MSleep(50);
				gpio_led_off();
				break;
			case LED_FLAG_MQTCON:
				gpio_led_on();
				break;
			case LED_FLAG_MQTDICON:
				OS_MSleep(100);
				gpio_led_on();
				OS_MSleep(100);
				gpio_led_off();
				break;
			default:
				break;
		}
		
	}

	OS_ThreadDelete(&xled_task_thread);
}

int led_flag_task_init()
{
	xled_task_run = 1;
	
	if (OS_ThreadCreate(&xled_task_thread,
		                "led_flag_ctrl",
		                led_flag_ctrl,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                XLED_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create error\n");
		return -1;
	}
	printf("led_flag_task_init end\n");
	
	return 0;
}