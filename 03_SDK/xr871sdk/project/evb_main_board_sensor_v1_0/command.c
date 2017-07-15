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

#include "common/cmd/cmd_debug.h"
#include "common/cmd/cmd_util.h"
#include "common/cmd/cmd_echo.h"
#include "common/cmd/cmd_upgrade.h"
#include "common/cmd/cmd_mem.h"
#if (defined(__CONFIG_ARCH_DUAL_CORE))
#include "common/cmd/cmd_wlan.h"
#include "common/cmd/cmd_ifconfig.h"
#include "common/cmd/cmd_smart_config.h"
#include "common/cmd/cmd_airkiss.h"
#include "common/cmd/cmd_iperf.h"
#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"
#include "common/cmd/cmd_ping.h"
#include "common/cmd/cmd_httpc.h"
#include "../common/cmd/cmd_mqtt.h"
#endif

#define COMMAND_IPERF	1
#define COMMAND_PING	1
#define COMMAND_HTTPC	0
#define COMMAND_MQTT	0

/*
 * net commands
 */
static struct cmd_data g_net_cmds[] = {
	{ "mode",		cmd_wlan_mode_exec },
	{ "sta",		cmd_wlan_sta_exec },
	{ "ap",			cmd_wlan_ap_exec },
	{ "ifconfig",	cmd_ifconfig_exec },
	{ "smartconfig",cmd_smart_config_exec },
	{ "airkiss",	cmd_airkiss_exec },
#if COMMAND_IPERF
	{ "iperf",		cmd_iperf_exec },
#endif
#if COMMAND_PING
	{ "ping",		cmd_ping_exec },
#endif

#if COMMAND_HTTPC
	{ "httpc",		cmd_httpc_exec },
#endif

#if COMMAND_MQTT
	{ "mqtt",		cmd_mqtt_exec },
#endif
};

static enum cmd_status cmd_net_exec(char *cmd)
{
	return cmd_exec(cmd, g_net_cmds, cmd_nitems(g_net_cmds));
}

/*
 * ducc command, only for test
 */
static enum cmd_status cmd_ducc_exec(char *cmd)
{
	if (cmd_strcmp(cmd, "ping") == 0) {
		if (ducc_app_ioctl(DUCC_APP_CMD_PING, 0) == 0)
			return CMD_STATUS_OK;
		else
			return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_UNKNOWN_CMD;
}

/*
 * driver commands
 */
static struct cmd_data g_drv_cmds[] = {
};

static enum cmd_status cmd_drv_exec(char *cmd)
{
	return cmd_exec(cmd, g_drv_cmds, cmd_nitems(g_drv_cmds));
}

/*
 * main commands
 */
static struct cmd_data g_main_cmds[] = {
	{ "net",	cmd_net_exec },
	{ "drv",	cmd_drv_exec },
	{ "echo",	cmd_echo_exec },
	{ "ducc",	cmd_ducc_exec },
	{ "upgrade",cmd_upgrade_exec },
	{ "reboot",	cmd_reboot_exec },
	{ "mem",	cmd_mem_exec },
};

void main_cmd_exec(char *cmd)
{
	enum cmd_status status;

	if (cmd[0] == '\0') { /* empty command */
		CMD_LOG(1, "$\n");
		return;
	}

	CMD_LOG(CMD_DEBUG_ON, "$ %s\n", cmd);

	status = cmd_exec(cmd, g_main_cmds, cmd_nitems(g_main_cmds));
	if (status == CMD_STATUS_ACKED) {
		return; /* already acked, just return */
	}

	cmd_write_respond(status, cmd_get_status_desc(status));
}
