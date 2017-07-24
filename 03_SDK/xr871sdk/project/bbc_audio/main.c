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

#include <stdio.h>
#include "types.h"
#include "kernel/os/os.h"
#include "version.h"

#include "common/board/board.h"
#include "common/net_ctrl/net_ctrl.h"
#include "common/sysinfo/sysinfo.h"

#include "sys_ctrl.h"
#include "ctrl_debug.h"
#include "pm/pm.h"
#include "driver/chip/hal_wakeup.h"
#include "audio_player.h"

#include "mqtt_build.h"
#include "gpio_button.h"
#include "cjson_analy.h"
#include "bbc_main.h"
#include "../src/bbc/devguid_get.h"


#define MAIN_THREAD_STACK_SIZE		(5 * 1024)
static OS_Thread_t g_main_thread;

static void main_task(void *arg)
{
	printf("XRadio IoT WLAN SDK %s\n\n", SDK_VERSION_STR);

	sysinfo_init();
	board_init();
	sys_ctrl_init();
	msg_parse_task_init();

#ifdef __CONFIG_ARCH_DUAL_CORE
	net_sys_start(sysinfo_get_wlan_mode());
#endif
	gpio_button_task_init();

#ifdef __CONFIG_XIP_ENABLE
	board_xip_init();
	CTRL_DBG("XIP enable\n");
#endif
#ifdef __CONFIG_XPLAYER 	// only for test
#if 1
	player_task_init();
#endif
#if 0
extern int xrecord_test();
	xrecord_test();
#endif
#if 1
	extern void main_cmd_exec(char *cmd);	
	main_cmd_exec("net sta config \"AW2\" \"1qaz@WSX\"");
	main_cmd_exec("net sta enable");

	/* wait connecting AP */
	while (1) {
		OS_MSleep(500);
		if (g_wlan_netif && netif_is_up(g_wlan_netif) && netif_is_link_up(g_wlan_netif)) {

			bbc_inital();
			cal_set.MqttCon = MQTT_CACK;
			cal_set.MqttSub = MQTT_CACK;
			mqtt_ctrl_task_init();
			
			break;
		}
	}
#endif
	bbc_audio_task_init();
#endif

	OS_ThreadDelete(&g_main_thread);
}

int main(void)
{
	if (OS_ThreadCreate(&g_main_thread,
		                "",
		                main_task,
		                NULL,
		                OS_THREAD_PRIO_APP,
		                MAIN_THREAD_STACK_SIZE) != OS_OK) {
		CTRL_ERR("create main task failed\n");
	}

	OS_ThreadStartScheduler();

	while (1) {
		CTRL_ERR("error\n");
	}

	return 0;
}
