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

#include "common/cmd/cmd_util.h"
#include "common/cmd/cmd.h"

#define COMMAND_IPERF       1
#define COMMAND_PING        1
#define COMMAND_HTTPC       0
#define COMMAND_TLS         0
#define COMMAND_HTTPD       0
#define COMMAND_MQTT        0
#define COMMAND_NOPOLL      0
#define COMMAND_SNTP        0
#define COMMAND_DHCPD       0
#define COMMAND_BRROADCAST  0
#define COMMAND_ARP         0

/*
 * net commands
 */
static struct cmd_data g_net_cmds[] = {
#ifdef __PRJ_CONFIG_WLAN_STA_AP
	{ "mode",		cmd_wlan_mode_exec },
	{ "ap", 		cmd_wlan_ap_exec },
#endif

	{ "sta",		cmd_wlan_sta_exec },
	{ "ifconfig",	cmd_ifconfig_exec },
	{ "airkiss", cmd_airkiss_exec },

#if COMMAND_IPERF
	{ "iperf",		cmd_iperf_exec },
#endif

#if COMMAND_PING
	{ "ping",		cmd_ping_exec },
#endif

#if COMMAND_HTTPC
	{ "httpc",		cmd_httpc_exec },
#endif

#if COMMAND_TLS
	{ "tls",		cmd_tls_exec },
#endif

#if COMMAND_HTTPD
	{ "httpd",		cmd_httpd_exec },
#endif

#if COMMAND_SNTP
	{ "sntp",		cmd_sntp_exec },
#endif

#if COMMAND_NOPOLL
	{ "nopoll",		cmd_nopoll_exec },
#endif

#if COMMAND_MQTT
	{ "mqtt",		cmd_mqtt_exec },
#endif

#if COMMAND_DHCPD
	{ "dhcpd",		cmd_dhcpd_exec },
#endif

#if COMMAND_BRROADCAST
	{ "broadcast",  cmd_broadcast_exec },
#endif

#if COMMAND_ARP
	{ "arp",        cmd_arp_exec },
#endif
};

static enum cmd_status cmd_net_exec(char *cmd)
{
	return cmd_exec(cmd, g_net_cmds, cmd_nitems(g_net_cmds));
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
	{ "mem",	cmd_mem_exec },
	{ "heap",	cmd_heap_exec },
	{ "upgrade",cmd_upgrade_exec },
	{ "reboot", cmd_reboot_exec },
#ifdef __PRJ_CONFIG_OTA
	{ "ota",    cmd_ota_exec },
	{ "etf",	cmd_etf_exec },
#endif
	{ "pm",		cmd_pm_exec },
	{ "efpg",	cmd_efpg_exec },
	{ "netcmd",	cmd_netcmd_exec },
	{ "sysinfo",cmd_sysinfo_exec },
};

void main_cmd_exec(char *cmd)
{
	enum cmd_status status;

	if (cmd[0] != '\0') {
#if (!CONSOLE_ECHO_EN)
		if (cmd_strcmp(cmd, "efpg"))
			CMD_LOG(CMD_DBG_ON, "$ %s\n", cmd);
#endif
		status = cmd_exec(cmd, g_main_cmds, cmd_nitems(g_main_cmds));
		if (status != CMD_STATUS_ACKED) {
			cmd_write_respond(status, cmd_get_status_desc(status));
		}
	}
#if (!CONSOLE_ECHO_EN)
	else { /* empty command */
		CMD_LOG(1, "$\n");
	}
#endif
#if CONSOLE_ECHO_EN
	console_write((uint8_t *)"$ ", 2);
#endif
}
