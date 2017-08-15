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

#include "audio_player.h"
#include "mqtt_build.h"
#include "gpio_button.h"
#include "bbc_main.h"
#include "led_flag.h"

typedef enum {
	RECON_CLEAN 	= 0,
	RECON_STASET	= 1,
	RECON_FIRSTA	= 2
}BBC_RECON_STATUS;

uint8_t start_statue = RECON_FIRSTA;
uint8_t mqtt_recon_flag = RECON_CLEAN;
uint8_t network_link_flag = RECON_CLEAN;
uint8_t net_dwn_up_link_flag = RECON_CLEAN;

void audio_funct_init(void)
{
	led_flag_task_init();		//use led to display result statue
	gpio_button_task_init();	//use DK1 button to smartconfig
	player_task_init();		//start audio function
	bbc_audio_task_init();	//break data upto bbc
}

int main(void)
{
	platform_init();
	audio_funct_init();

	/* wait connecting AP */
	while (1) {
		OS_MSleep(200);
		if (g_wlan_netif && netif_is_up(g_wlan_netif) && netif_is_link_up(g_wlan_netif)) {
			network_link_flag = RECON_STASET;
			mqtt_set_rcome = RECON_STASET;
		}
		else {
			net_dwn_up_link_flag = RECON_STASET;
			mqtt_set_rcome = RECON_CLEAN;
		}
		
		if((network_link_flag == RECON_STASET) && (net_dwn_up_link_flag == RECON_STASET)) {
			mqtt_recon_flag = RECON_STASET;
			network_link_flag = RECON_CLEAN;
			net_dwn_up_link_flag = RECON_CLEAN;
		}
		
		if((SmrtCfgMode == RECON_STASET) || (mqtt_recon_flag == RECON_STASET)) {
			cal_set.MqttCon = MQTT_CACK;
			cal_set.MqttSub = MQTT_CACK;
			SmrtCfgMode = RECON_CLEAN;
			mqtt_recon_flag = RECON_CLEAN;
		}
		
		if((start_statue == RECON_FIRSTA) && (network_link_flag == RECON_STASET)) {
			mqtt_ctrl_task_init();
			start_statue = RECON_CLEAN;
			network_link_flag = RECON_CLEAN;
		}
	}

}
