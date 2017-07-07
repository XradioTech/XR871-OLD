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

#include "cmd_debug.h"
#include "cmd_util.h"

#include "common/net_ctrl/net_ctrl.h"

/*
 * net wlan mode [sta|ap]
 */

static const char *g_wlan_mode_str[WLAN_MODE_NUM] = {
	[WLAN_MODE_STA] 	= "station",
	[WLAN_MODE_HOSTAP]	= "hostap",
	[WLAN_MODE_MONITOR] = "monitor",
};

enum cmd_status cmd_wlan_mode_exec(char *cmd)
{
	enum wlan_mode cur_mode, new_mode;
	const char *mode_str;

	cur_mode = wlan_if_get_mode(g_wlan_netif);

	if (cmd_strcmp(cmd, "") == 0) {
		if (cur_mode < WLAN_MODE_NUM) {
			mode_str = g_wlan_mode_str[cur_mode];
		} else {
			mode_str = "invalid";
		}
		cmd_write_respond(CMD_STATUS_OK, "%s", mode_str);
		return CMD_STATUS_ACKED;
	}

	if (cmd_strcmp(cmd, "sta") == 0) {
		new_mode = WLAN_MODE_STA;
	} else if (cmd_strcmp(cmd, "ap") == 0) {
		new_mode = WLAN_MODE_HOSTAP;
	} else if (cmd_strcmp(cmd, "mon") == 0) {
		new_mode = WLAN_MODE_MONITOR;
	} else {
		return CMD_STATUS_INVALID_ARG;
	}

	if (new_mode == cur_mode) {
		CMD_DBG("no need to switch wlan mode %d\n", cur_mode);
		return CMD_STATUS_OK;
	}

	if (netif_is_up(g_wlan_netif)) {
		net_config(g_wlan_netif, 0);
	}

	if (cur_mode == WLAN_MODE_STA && netif_is_link_up(g_wlan_netif)) {
		wlan_ctrl_request(WPA_CTRL_CMD_DISABLE_NETWORK, 0);
	}

	switch (new_mode) {
	case WLAN_MODE_HOSTAP:
		net_sys_stop();
		net_sys_start(new_mode);
		break;
	case WLAN_MODE_STA:
	case WLAN_MODE_MONITOR:
		if (cur_mode == WLAN_MODE_HOSTAP) {
			net_sys_stop();
			net_sys_start(new_mode);
		} else {
			net_close(g_wlan_netif);
			g_wlan_netif = net_open(new_mode);
		}
		break;
	default:
		break;
	}

	return CMD_STATUS_OK;
}

static struct cmd_data g_wlan_cmds[] = {
	{ "mode",		cmd_wlan_mode_exec },
};

enum cmd_status cmd_wlan_exec(char *cmd)
{
	return cmd_exec(cmd, g_wlan_cmds, cmd_nitems(g_wlan_cmds));
}
