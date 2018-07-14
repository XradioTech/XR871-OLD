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

/*
 * main commands
 */
static struct cmd_data g_main_cmds[] = {
	{ "upgrade",cmd_upgrade_exec },
	{ "reboot", cmd_reboot_exec },
	{ "netcmd",	cmd_netcmd_exec },
	{ "mem",	cmd_mem_exec },
//	{ "atsc",	cmd_atsc_exec },
	{ "etf",	cmd_etf_exec },
	{ "efpg",	cmd_efpg_exec },
	{ "flash",	cmd_flash_exec },
};

void main_cmd_exec(char *cmd)
{
	enum cmd_status status;

	if (cmd[0] == '\0') { /* empty command */
		CMD_LOG(1, "$\n");
		return;
	}

	if (cmd_strncmp(cmd, "efpg", 4) &&
		cmd_strncmp(cmd, "flash", 5))
		CMD_LOG(CMD_DBG_ON, "$ %s\n", cmd);

	status = cmd_exec(cmd, g_main_cmds, cmd_nitems(g_main_cmds));
	if (status == CMD_STATUS_ACKED) {
		return; /* already acked, just return */
	}

	cmd_write_respond(status, cmd_get_status_desc(status));
}
