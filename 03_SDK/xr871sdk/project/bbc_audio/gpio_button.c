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
#include "common/net_ctrl/net_ctrl.h"

#include "common/board/board.h"
#include "net/wlan/wlan.h"

#define XBUTTON_THREAD_STACK_SIZE	512
OS_Thread_t xbutton_task_thread;
uint8_t xbutton_task_run = 0;
uint8_t ConfigSmartStart = 0;
uint8_t SmartConfigFlag = 0;

static char *key = "1234567812345678";

void gpio_button_init()
{
	GPIO_InitParam param;	
	param.driving = GPIO_DRIVING_LEVEL_0;	
	param.mode    = GPIOx_Pn_F0_INPUT;	
	param.pull    = GPIO_PULL_DOWN;	
	HAL_GPIO_Init(GPIO_PORT_A, GPIO_PIN_6, &param);
}
void gpio_button_set(void)
{
	uint8_t gpio_value;
	
	while(xbutton_task_run) {
		OS_MSleep(200);
		gpio_value = HAL_GPIO_ReadPin(GPIO_PORT_A, GPIO_PIN_6);
		if(gpio_value == GPIO_PIN_LOW) {
			printf("DKEY1\n");
			ConfigSmartStart = 1;
			OS_MSleep(300);
		}
		if(ConfigSmartStart == 1) {
			wlan_smart_config_set_key(key);
			wlan_smart_config_start(g_wlan_netif);
			ConfigSmartStart = 0;
			SmartConfigFlag = 1;
		}
		if (g_wlan_netif && netif_is_up(g_wlan_netif) && netif_is_link_up(g_wlan_netif) && SmartConfigFlag) 
		{
			OS_MSleep(100);
			wlan_smart_config_stop();
			SmartConfigFlag = 0;
		}
	}
	OS_ThreadDelete(&xbutton_task_thread);
}

int gpio_button_task_init()
{
	gpio_button_init();
	xbutton_task_run = 1;
	
	if (OS_ThreadCreate(&xbutton_task_thread,
		                "gpio_button",
		                gpio_button_set,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                XBUTTON_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create error\n");
		return -1;
	}
	printf("gpio_button_task_init end\n");
	
	return 0;
}

