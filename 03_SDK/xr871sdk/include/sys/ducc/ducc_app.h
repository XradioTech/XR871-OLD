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

#ifndef _SYS_DUCC_DUCC_APP_H_
#define _SYS_DUCC_DUCC_APP_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DUCC_APP_CMD_PING_SUPPORT	1

enum ducc_app_cmd {
#if DUCC_APP_CMD_PING_SUPPORT
	DUCC_APP_CMD_PING,
#endif
	DUCC_APP_CMD_POWER_NOTIFY,
	DUCC_APP_CMD_WLAN_ATTACH,
	DUCC_APP_CMD_WLAN_DETACH,
	DUCC_APP_CMD_WLAN_IF_CREATE,
	DUCC_APP_CMD_WLAN_IF_DELETE,
	DUCC_APP_CMD_WLAN_START,
	DUCC_APP_CMD_WLAN_STOP,
	DUCC_APP_CMD_WLAN_GET_MAC_ADDR,
	DUCC_APP_CMD_WLAN_SET_MAC_ADDR,
	DUCC_APP_CMD_WLAN_SET_IP_ADDR,
	DUCC_APP_CMD_WLAN_WPA_CTRL_OPEN,
	DUCC_APP_CMD_WLAN_WPA_CTRL_CLOSE,
	DUCC_APP_CMD_WLAN_WPA_CTRL_REQUEST,
	DUCC_APP_CMD_WLAN_SMART_CONFIG_START,
	DUCC_APP_CMD_WLAN_SMART_CONFIG_STOP,
	DUCC_APP_CMD_WLAN_SMART_CONFIG_SET_KEY,
	DUCC_APP_CMD_WLAN_AIRKISS_START,
	DUCC_APP_CMD_WLAN_AIRKISS_STOP,
	DUCC_APP_CMD_WLAN_AIRKISS_SET_KEY,

	DUCC_APP_CMD_WLAN_LINKOUTPUT,
};

#define DUCC_APP_IS_DATA_CMD(c) \
	((c) == DUCC_APP_CMD_WLAN_LINKOUTPUT)

struct ducc_param_wlan_create {
	uint32_t mode;
	void *nif;
	const char *name;
	void *ifp;	/* @out */
};

struct ducc_param_wlan_get_mac_addr {
	void *ifp;
	uint8_t *buf;
	int buf_len;
};

struct ducc_param_wlan_set_mac_addr {
	uint8_t *mac_addr;
	int mac_len;
};

struct ducc_param_wlan_linkoutput {
	void *ifp;
	void *mbuf;
};

struct ducc_param_wlan_set_ip_addr {
	void *ifp;
	uint8_t *ip_addr;
	int ip_len;
};

struct ducc_param_wlan_wpa_ctrl_req {
	uint32_t cmd;
	void *data;
};

#if (!defined(__CONFIG_ARCH_DUAL_CORE) || defined(__CONFIG_ARCH_APP_CORE))
typedef int (*ducc_cb_func)(uint32_t param0, uint32_t param1);

struct ducc_app_param {
	ducc_cb_func cb;
};

int ducc_app_start(struct ducc_app_param *param);
int ducc_app_ioctl(enum ducc_app_cmd cmd, void *param);
int ducc_app_stop(void);
#endif /* (!defined(__CONFIG_ARCH_DUAL_CORE) || defined(__CONFIG_ARCH_APP_CORE)) */

#ifdef __cplusplus
}
#endif

#endif	/* _SYS_DUCC_DUCC_APP_H_ */
