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

#include "common/framework/platform_init.h"
#include "kernel/os/os.h"
#include "common/framework/net_ctrl.h"
#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"

#define SC_TIME_OUT 120000
#define SC_ACK_TIME_OUT 30000
static char *key = "1234567812345678";

static void SmartConfigDemo(void)
{
	int ret;
	wlan_smart_config_status_t status;
	wlan_smart_config_result_t result;

	memset(&result, 0, sizeof(result));

	/* Switch to monitor mode */
	net_switch_mode(WLAN_MODE_MONITOR);

	/* Set the aes key. If you don't want to encrypt data, you can not use it. */
	ret = wlan_smart_config_set_key(key, WLAN_SMART_CONFIG_KEY_LEN);
	if (ret != 0)
		printf("SmartConfig set key error\n");

	status = wlan_smart_config_start(g_wlan_netif, SC_TIME_OUT, &result);

	/* Switch back to station mode */
	net_switch_mode(WLAN_MODE_STA);

	if (status == WLAN_SMART_CONFIG_SUCCESS) {
		printf("ssid: %.32s\n", (char *)result.ssid);
		printf("psk: %s\n", (char *)result.passphrase);
		printf("random: %d\n", result.random_num);
	} else {
		printf ("smartconfig failed %d\n", status);
		return;
	}

	/* Set ssid and passphrase */
	if (result.passphrase[0] != '\0') {
		wlan_sta_set(result.ssid, result.ssid_len, result.passphrase);
	} else {
		wlan_sta_set(result.ssid, result.ssid_len, NULL);
	}

	/* Connect to ap */
	wlan_sta_enable();

	/* Ack phone */
	ret = wlan_smart_config_ack_start(g_wlan_netif, result.random_num, SC_ACK_TIME_OUT);
	if (ret < 0)
		printf("smartconfig ack error\n");
}

int main(void)
{
	platform_init();

	printf("SmartConfig demo started.\n\n");

	SmartConfigDemo();

	printf("\nSmartConfig demo over\n");
}
