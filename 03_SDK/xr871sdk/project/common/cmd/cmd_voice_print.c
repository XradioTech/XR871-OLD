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

#if PRJCONF_NET_EN

#include <string.h>

#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"
#include "lwip/netif.h"

#include "cmd_util.h"
#include "common/framework/net_ctrl.h"

#include "smartlink/sc_assistant.h"
#include "smartlink/voice_print/voice_print.h"

#define VP_TIME_OUT_MS 120000

static OS_Thread_t g_thread;
#define THREAD_STACK_SIZE       (2 * 1024)

uint8_t cmd_vp_checksum(uint8_t * buf, int len)
{
	int i;
	uint8_t cs = 0;

	for (i = 0;i < len;i++) {
		cs += buf[i];
	}
	return cs;
}

static int cmd_vp_result_handler(char *result_str, int result_len,
                                 wlan_voiceprint_result_t *vp_result)
{
	const char *str_find;
	char temp[3] = {0};
	uint8_t len_temp;

	/* ssid length */
	str_find = result_str;
	cmd_memcpy(temp, str_find, 2);
	len_temp = cmd_strtol(temp, NULL, 16);
	if (len_temp > WLAN_SSID_MAX_LEN) {
		CMD_DBG("invalid ssid len %d\n", len_temp);
		return -1;
	}

	/* ssid */
	str_find += 2;
	vp_result->ssid_len = len_temp;
	cmd_memcpy(vp_result->ssid, str_find, len_temp);
	CMD_DBG("SSID (%d): %.*s\n", vp_result->ssid_len, vp_result->ssid_len,
	        vp_result->ssid);

#if (VOICE_PRINT_POLICY == 1)
	/* passphrase length */
	str_find += vp_result->ssid_len;
	cmd_memcpy(temp, str_find, 2);
	len_temp = cmd_strtol(temp, NULL, 16);
	if (len_temp != 0) {
		if (len_temp < WLAN_PASSPHRASE_MIN_LEN ||
			len_temp > WLAN_PASSPHRASE_MAX_LEN) {
			CMD_DBG("invalid psk len %d\n", len_temp);
			return -1;
		}

		/* passphrase */
		str_find += 2;
		cmd_memcpy(vp_result->passphrase, str_find, len_temp);
	}
	vp_result->passphrase[len_temp] = '\0';
	CMD_DBG("PSK (%d): %s\n", len_temp, vp_result->passphrase);

	/* checksum */
	str_find += len_temp;
	cmd_memcpy(temp, str_find, 2);
	uint8_t cs = cmd_strtol(temp, NULL, 16);
	len_temp = 2 + vp_result->ssid_len + 2 + len_temp;
	cs += cmd_vp_checksum((uint8_t *)result_str, len_temp);

	if (0xFF != cs) {
		CMD_DBG("cs err: 0x%x\n", cs);
		return -1;
	}
#elif (VOICE_PRINT_POLICY == 2)
	/* passphrase */
	str_find += vp_result->ssid_len;
	len_temp = cmd_strlen(str_find);
	if (len_temp != 0) {
		if (len_temp < WLAN_PASSPHRASE_MIN_LEN ||
		    len_temp > WLAN_PASSPHRASE_MAX_LEN) {
			CMD_DBG("invalid psk len %d\n", len_temp);
			return -1;
		}
	}
	cmd_memcpy(vp_result->passphrase, str_find, len_temp + 1);
	CMD_DBG("PSK (%d): %s\n", len_temp, vp_result->passphrase);
#endif

	return 0;
}

static void vp_task(void *arg)
{
	wlan_voiceprint_result_t vp_result;
	char result[128];
	int ret, len;
	uint8_t *psk;

	CMD_DBG("%s getting ssid and psk...\n", __func__);

	if (voiceprint_wait(VP_TIME_OUT_MS) != WLAN_VOICEPRINT_SUCCESS) {
		goto out;
	}
	CMD_DBG("%s get ssid and psk finished\n", __func__);

	len = sizeof(result) - 1;
	if (voiceprint_get_status() == VP_STATUS_COMPLETE) {
		if (wlan_voiceprint_get_raw_result(result, &len) == WLAN_VOICEPRINT_SUCCESS) {
			result[len] = '\0';
			CMD_DBG("result(%d): %s\n", len, result);
			if (cmd_vp_result_handler(result, len, &vp_result) == 0) {
				g_wlan_netif = sc_assistant_open_sta();
				if (vp_result.passphrase[0] != '\0') {
					psk = vp_result.passphrase;
				} else {
					psk = NULL;
				}
				ret = sc_assistant_connect_ap(vp_result.ssid, vp_result.ssid_len,
				                              psk, VP_TIME_OUT_MS);
				if (ret < 0) {
					CMD_DBG("voiceprint connect ap time out\n");
					goto out;
				}
#if 0
				ret = voiceprint_ack_start(priv, result->random_num, VP_ACK_TIME_OUT_MS);
				if (ret < 0)
					VP_DBG(ERROR, "voice ack error, ap connect time out\n");
#endif
			}
		}
	}

out:
	voiceprint_stop(0);
	sc_assistant_deinit(g_wlan_netif);
	OS_ThreadDelete(&g_thread);
}

static int cmd_vp_start(void)
{
	int ret = 0;
	voiceprint_ret_t vp_status;
	sc_assistant_fun_t sca_fun;
	sc_assistant_time_config_t config;
	voiceprint_param_t vp_param;

	if (OS_ThreadIsValid(&g_thread))
		return -1;

	sc_assistant_get_fun(&sca_fun);
	config.time_total = VP_TIME_OUT_MS;
	config.time_sw_ch_long = 0;
	config.time_sw_ch_short = 0;
	sc_assistant_init(g_wlan_netif, &sca_fun, &config);

	cmd_memset(&vp_param, 0, sizeof(vp_param));
	vp_param.audio_card = AUDIO_CARD0;
	vp_param.nif = g_wlan_netif;
	vp_status = voiceprint_start(&vp_param);
	if (vp_status != WLAN_VOICEPRINT_SUCCESS) {
		CMD_DBG("voiceprint start fiald!\n");
		ret = -1;
		goto out;
	}

	if (OS_ThreadCreate(&g_thread,
	                    "cmd_vp",
	                    vp_task,
	                    NULL,
	                    OS_THREAD_PRIO_APP,
	                    THREAD_STACK_SIZE) != OS_OK) {
		CMD_ERR("create voice_print thread failed\n");
		ret = -1;
	}
out:
	return ret;
}

static int cmd_vp_stop(void)
{
	if (!OS_ThreadIsValid(&g_thread))
		return -1;

	voiceprint_stop(1);

	return 0;
}

enum cmd_status cmd_voice_print_start_exec(char *cmd)
{
	int ret;

	if (OS_ThreadIsValid(&g_thread)) {
		CMD_ERR("Voiceprint is already start\n");
		ret = -1;
	} else {
		ret = cmd_vp_start();
	}

	return (ret == 0 ? CMD_STATUS_OK : CMD_STATUS_FAIL);
}

enum cmd_status cmd_voice_print_stop_exec(char *cmd)
{
	return (cmd_vp_stop() == 0 ? CMD_STATUS_OK : CMD_STATUS_FAIL);
}

static const struct cmd_data g_voice_print_cmds[] = {
    { "start",		cmd_voice_print_start_exec},
    { "stop",		cmd_voice_print_stop_exec},
};

enum cmd_status cmd_voice_print_exec(char *cmd)
{
	if (g_wlan_netif == NULL) {
		return CMD_STATUS_FAIL;
	}
	return cmd_exec(cmd, g_voice_print_cmds, cmd_nitems(g_voice_print_cmds));
}

#endif /* PRJCONF_NET_EN */
