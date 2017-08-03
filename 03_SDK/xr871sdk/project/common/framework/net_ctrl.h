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

#ifndef _NET_CTRL_H_
#define _NET_CTRL_H_

#include "lwip/netif.h"
#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

//#define CONFIG_AUTO_RECONNECT_AP

enum net_ctrl_msg_type {
	NET_CTRL_MSG_WLAN_CONNECTED				= WLAN_EVENT_CONNECTED,
	NET_CTRL_MSG_WLAN_DISCONNECTED			= WLAN_EVENT_DISCONNECTED,
	NET_CTRL_MSG_WLAN_SCAN_SUCCESS			= WLAN_EVENT_SCAN_SUCCESS,
	NET_CTRL_MSG_WLAN_SCAN_FAILED			= WLAN_EVENT_SCAN_FAILED,
	NET_CTRL_MSG_WLAN_4WAY_HANDSHAKE_FAILED	= WLAN_EVENT_4WAY_HANDSHAKE_FAILED,
	NET_CTRL_MSG_WLAN_CONNECT_FAILED		= WLAN_EVENT_CONNECT_FAILED,
	NET_CTRL_MSG_WLAN_SMART_CONFIG_RESULT,
	NET_CTRL_MSG_WLAN_AIRKISS_RESULT,

	NET_CTRL_MSG_NETWORK_UP,
	NET_CTRL_MSG_NETWORK_DOWN,
};

extern struct netif *g_wlan_netif;

int net_sys_start(enum wlan_mode mode);
int net_sys_stop(void);
int net_sys_onoff(unsigned int enable);

struct netif *net_open(enum wlan_mode mode);
void net_close(struct netif *nif);
void net_config(struct netif *nif, uint8_t bring_up);
int net_ctrl_connect_ap(void);
int net_ctrl_disconnect_ap(void);

int net_ctrl_msg_send(uint16_t type, uint32_t data);
void net_ctrl_msg_process(uint16_t type, uint32_t data);

#ifdef __cplusplus
}
#endif

#endif /* _NET_CTRL_H_ */
