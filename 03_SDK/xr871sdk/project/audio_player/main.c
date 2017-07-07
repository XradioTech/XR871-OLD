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

#define MAIN_THREAD_STACK_SIZE		(2 * 1024)
static OS_Thread_t g_main_thread;

int net_sys_onoff(unsigned int enable)
{
	printf("%s set net to power%s\n", __func__, enable?"on":"off");

	if (enable)
		;//net_sys_start(g_wlan_mac_addr, sizeof(g_wlan_mac_addr));
	else
		;//net_sys_stop();

	return 0;
}

static void main_task(void *arg)
{
	printf("XRadio IoT WLAN SDK %s\n\n", SDK_VERSION_STR);

	sysinfo_init();
	board_init();
	sys_ctrl_init();

#ifdef __CONFIG_ARCH_DUAL_CORE
	net_sys_start(sysinfo_get_wlan_mode());
	pm_register_wlan_power_onoff(&net_sys_onoff, PM_SUPPORT_SLEEP | PM_SUPPORT_STANDBY |
	                             PM_SUPPORT_HIBERNATION | PM_SUPPORT_POWEROFF);
#endif
#ifdef __CONFIG_XIP_ENABLE
	board_xip_init();
	CTRL_DBG("XIP enable\n");
#endif
#ifdef __CONFIG_XPLAYER // only for test
#if 0
	extern int cedarx_test();
	cedarx_test();
#endif
#if 0
extern int xrecord_test();
	xrecord_test();
#endif
#if 0
	extern void main_cmd_exec(char *cmd);
	main_cmd_exec("net wpas set_network 0 ssid \"MERCURY_AD96\"");
	main_cmd_exec("net wpas set_network 0 key_mgmt NONE");
	main_cmd_exec("net wpas enable_network 0");

	/* wait connecting AP */
	while (1) {
		OS_MSleep(500);
		if (g_wlan_netif && netif_is_up(g_wlan_netif) && netif_is_link_up(g_wlan_netif)) {
			extern int cedarx_http_test();
			cedarx_http_test();
			break;
		}
	}
#endif
#endif
	gpio_button_ctrl_init();
	ad_button_init();
	player_task_init();
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
