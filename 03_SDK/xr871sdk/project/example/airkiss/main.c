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
#include <string.h>

#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"

#include "kernel/os/os.h"
#include "common/framework/net_ctrl.h"
#include "common/framework/platform_init.h"

#include "smartlink/sc_assistant.h"
#include "smartlink/airkiss/wlan_airkiss.h"

#define AK_TIME_OUT_MS 120000

static char airkiss_key[17] = "1234567812345678";

static void AirkissDemo(void)
{
	wlan_airkiss_status_t ak_status;
	wlan_airkiss_result_t ak_result;
	sc_assistant_fun_t sca_fun;
	sc_assistant_time_config_t config;

	memset(&ak_result, 0, sizeof(wlan_airkiss_result_t));

	sc_assistant_get_fun(&sca_fun);
	config.time_total = AK_TIME_OUT_MS;
	config.time_sw_ch_long = 400;
	config.time_sw_ch_short = 100;
	sc_assistant_init(g_wlan_netif, &sca_fun, &config);

	ak_status = wlan_airkiss_start(g_wlan_netif, airkiss_key);
	if (ak_status != WLAN_AIRKISS_SUCCESS) {
		printf("airkiss start fiald!\n");
		goto out;
	}

	printf("%s getting ssid and psk...\n", __func__);

	if (wlan_airkiss_wait(AK_TIME_OUT_MS) == WLAN_AIRKISS_TIMEOUT) {
		printf("%s get ssid and psk timeout\n", __func__);
		goto out;
	}
	printf("%s get ssid and psk finished\n", __func__);

	if (wlan_airkiss_get_status() == AIRKISS_STATUS_COMPLETE && \
	    wlan_airkiss_get_result(&ak_result) == WLAN_AIRKISS_SUCCESS) {
		printf("ssid:%s psk:%s random:%d\n", (char *)ak_result.ssid,
		       (char *)ak_result.passphrase, ak_result.random_num);
		if (!wlan_airkiss_connect_ack(g_wlan_netif, AK_TIME_OUT_MS, &ak_result)) {
			printf("connect and ack ok\n");
		}
	}

out:
	wlan_airkiss_stop();
	sc_assistant_deinit(g_wlan_netif);
}

int main(void)
{
	platform_init();

	printf("Airkiss demo started.\n\n");

	AirkissDemo();

	printf("\nAirkiss demo over\n");
	return 0;
}
