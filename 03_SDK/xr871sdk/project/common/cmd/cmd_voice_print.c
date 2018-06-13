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

#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"
#include "lwip/netif.h"

#include "cmd_util.h"
#include "common/framework/net_ctrl.h"

#include "smartlink/sc_assistant.h"
#include "smartlink/voice_print/voice_print.h"

#define VP_TIME_OUT_MS 120000

static int vp_key_used;
static char *vp_key = "1234567812345678";

static OS_Thread_t g_thread;
#define THREAD_STACK_SIZE       (2 * 1024)

static void vp_task(void *arg)
{
	wlan_voiceprint_result_t vp_result;

	memset(&vp_result, 0, sizeof(wlan_voiceprint_result_t));

	CMD_DBG("%s getting ssid and psk...\n", __func__);

	if (voice_print_wait(VP_TIME_OUT_MS) != WLAN_VOICEPRINT_SUCCESS) {
		goto out;
	}
	CMD_DBG("%s get ssid and psk finished\n", __func__);

	if (voiceprint_get_status() == VP_STATUS_COMPLETE) {
		if (!wlan_voiceprint_connect_ack(g_wlan_netif, VP_TIME_OUT_MS, &vp_result)) {
			CMD_DBG("ssid:%s psk:%s random:%d\n", (char *)vp_result.ssid,
			        (char *)vp_result.passphrase, vp_result.random_num);
		}
	}

out:
	voice_print_stop(0);
	sc_assistant_deinit(g_wlan_netif);
	OS_ThreadDelete(&g_thread);
}

static int cmd_vp_start(void)
{
	int ret = 0;
	voiceprint_ret_t vp_status;
	sc_assistant_fun_t sca_fun;

	if (OS_ThreadIsValid(&g_thread))
		return -1;

	sc_assistant_get_fun(&sca_fun);
	sc_assistant_init(g_wlan_netif, &sca_fun, VP_TIME_OUT_MS);

	vp_status = voice_print_start(g_wlan_netif, vp_key_used ? vp_key : NULL);
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

	voice_print_stop(1);

	return 0;
}

enum cmd_status cmd_voice_print_exec(char *cmd)
{
	int ret = 0;

	if (g_wlan_netif == NULL) {
		return CMD_STATUS_FAIL;
	}

	if (cmd_strcmp(cmd, "set_key") == 0) {
		vp_key_used = 1;
		CMD_DBG("Voiceprint set key : %s\n", vp_key);
	} else if (cmd_strcmp(cmd, "start") == 0) {
		if (OS_ThreadIsValid(&g_thread)) {
			CMD_ERR("Voiceprint is already start\n");
			ret = -1;
		} else {
			ret = cmd_vp_start();
		}
	} else if (cmd_strcmp(cmd, "stop") == 0) {
		ret = cmd_vp_stop();
		vp_key_used = 0;
	} else {
		CMD_ERR("invalid argument '%s'\n", cmd);
		return CMD_STATUS_INVALID_ARG;
	}

	return (ret == 0 ? CMD_STATUS_OK : CMD_STATUS_FAIL);
}
