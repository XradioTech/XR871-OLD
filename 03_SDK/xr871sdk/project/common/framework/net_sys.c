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
#include <stdlib.h>
#include "kernel/os/os.h"
#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"
#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"
#include "lwip/tcpip.h"

#include "common/framework/sysinfo.h"
#include "net_ctrl.h"
#include "net_ctrl_debug.h"

#include "driver/chip/hal_prcm.h"

struct netif *g_wlan_netif = NULL;

static int net_sys_callback(uint32_t param0, uint32_t param1)
{
	switch (param0) {
	case DUCC_NET_CMD_WLAN_EVENT:
		switch (param1) {
		case WLAN_EVENT_CONNECTED:
		case WLAN_EVENT_DISCONNECTED:
		case WLAN_EVENT_SCAN_SUCCESS:
		case WLAN_EVENT_SCAN_FAILED:
		case WLAN_EVENT_4WAY_HANDSHAKE_FAILED:
		case WLAN_EVENT_CONNECT_FAILED:
			net_ctrl_msg_send(param1, 0);
			break;
		default:
			break;
		}
		break;
	case DUCC_NET_CMD_WLAN_SMART_CONFIG_RESULT:
	{
		struct wlan_smart_config_result *p;
		p = malloc(sizeof(struct wlan_smart_config_result));
		memcpy(p, (void *)param1, sizeof(struct wlan_smart_config_result));
		net_ctrl_msg_send_with_free(NET_CTRL_MSG_WLAN_SMART_CONFIG_RESULT, (uint32_t)p);
		break;
	}
	case DUCC_NET_CMD_WLAN_AIRKISS_RESULT:
	{
		struct wlan_smart_config_result *p;
		p = malloc(sizeof(struct wlan_smart_config_result));
		memcpy(p, (void *)param1, sizeof(struct wlan_smart_config_result));
		net_ctrl_msg_send_with_free(NET_CTRL_MSG_WLAN_AIRKISS_RESULT, (uint32_t)p);
		break;
	}
	default:
		break;
	}
	return 0;
}

static void netif_tcpip_init_done(void *arg)
{
//	NET_DBG("%s()\n", __func__);
}

static int tcpip_init_flg;

int net_sys_start(enum wlan_mode mode)
{
	uint8_t mac_addr[SYSINFO_MAC_ADDR_LEN];

	if (wlan_sys_init(mode, net_sys_callback) != 0) {
		NET_ERR("net system start failed\n");
		return -1;
	}

	if (!tcpip_init_flg) {
		tcpip_init(netif_tcpip_init_done, NULL); /* Init lwip module */
		tcpip_init_flg = 1;
	}

	if (sysinfo_get(SYSINFO_MAC_ADDR, &mac_addr) == 0) {
		wlan_set_mac_addr(mac_addr, SYSINFO_MAC_ADDR_LEN);
	}

	g_wlan_netif = net_open(mode);
	return 0;
}

int net_sys_stop(void)
{
	net_close(g_wlan_netif);
	g_wlan_netif = NULL;

#if LWIP_XR_DEINIT
	tcpip_deinit(); /* DeInit lwip module */
	tcpip_init_flg = 0;
#endif

	while (HAL_PRCM_IsSys3Alive()) {
		OS_MSleep(10); /* wait net */
	}

	if (wlan_sys_deinit()) {
		NET_ERR("net system stop failed\n");
		return -1;
	}

	return 0;
}

int net_sys_onoff(unsigned int enable)
{
	enum wlan_mode mode = WLAN_MODE_INVALID;

	printf("%s set net to power%s\n", __func__, enable?"on":"off");

	if (enable) {
		sysinfo_get(SYSINFO_WLAN_MODE, &mode);
		net_sys_start(mode);
#ifdef CONFIG_AUTO_RECONNECT_AP
		net_ctrl_connect_ap(NULL);
#endif
	} else {
#ifdef CONFIG_AUTO_RECONNECT_AP
		net_ctrl_disconnect_ap(NULL, 1);
#endif
		net_sys_stop();
	}

	return 0;
}
