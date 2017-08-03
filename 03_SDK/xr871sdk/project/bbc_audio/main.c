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

#include "common/framework/platform_init.h"
#include "common/framework/net_ctrl.h"

#include "sys/ota.h"

#include "mqtt_build.h"
#include "gpio_button.h"
#include "cjson_analy.h"
#include "bbc_main.h"
#include "../src/bbc/devguid_get.h"


int main(void)
{
	gpio_button_task_init();

	player_task_init();
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
}
