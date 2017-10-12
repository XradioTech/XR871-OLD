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

#include "cmd_util.h"
#include "cmd_dhcpd.h"
#include "net/udhcp/usr_dhcpd.h"
#include "net/lwip/ipv4/lwip/inet.h"

#define CMD_DHCPD_ADDR_START "192.168.51.150"
#define CMD_DHCPD_ADDR_END   "192.168.51.155"

static struct dhcp_server_info dhcpd_info;

enum cmd_status cmd_dhcpd_exec(char *cmd)
{
        int argc;
        char *argv[2];

        argc = cmd_parse_argv(cmd, argv, 3);
        if (argc < 1) {
                CMD_ERR("invalid dhcpd cmd, argc %d\n", argc);
                return CMD_STATUS_INVALID_ARG;
        }
        int enable = atoi(argv[0]);

	dhcpd_info.addr_start = inet_addr(CMD_DHCPD_ADDR_START);
	dhcpd_info.addr_end = inet_addr(CMD_DHCPD_ADDR_END);
	dhcpd_info.lease_time = 60*60*12;

        if (enable == 1)
                dhcp_server_start(&dhcpd_info);
        else
                udhcpd_stop(NULL);

        return CMD_STATUS_OK;
}
