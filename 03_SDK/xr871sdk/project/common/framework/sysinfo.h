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

#ifndef _SYSINFO_H_
#define _SYSINFO_H_

#include <stdint.h>
#include "lwip/netif.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYSINFO_MAC_ADDR_LEN	(6)

#define SYSINFO_SSID_ASCII_LEN	(33)
#define SYSINFO_SSID_LEN		(65)
#define SYSINFO_PSK_ASCII_LEN	(64)
#define SYSINFO_PSK_LEN			(66)

struct sysinfo_wlan_param {
	union {
		uint8_t ssid_ascii[SYSINFO_SSID_ASCII_LEN];
		uint8_t ssid[SYSINFO_SSID_LEN];
	} u_ssid;
	union {
		uint8_t psk_ascii[SYSINFO_PSK_ASCII_LEN];
		uint8_t psk[SYSINFO_PSK_LEN];
	} u_psk;
	uint8_t channel;
};

struct sysinfo_netif_param {
	uint8_t use_dhcp;
	ip_addr_t ip_addr;
	ip_addr_t net_mask;
	ip_addr_t gateway;
};

struct sysinfo {
	uint8_t mac_addr[SYSINFO_MAC_ADDR_LEN];

	uint8_t wlan_mode;
	struct sysinfo_wlan_param wlan_param;

	struct sysinfo_netif_param sta_netif_param;
	struct sysinfo_netif_param ap_netif_param;
};

#define SYSINFO_SIZE	sizeof(struct sysinfo)

enum sysinfo_type {
	SYSINFO_WHOLE = 0,

	SYSINFO_MAC_ADDR,

	SYSINFO_WLAN_MODE,
	SYSINFO_WLAN_PARAM,

	SYSINFO_STA_NETIF_PARAM,
	SYSINFO_AP_NETIF_PARAM,
};

int sysinfo_init(void);
void sysinfo_deinit(void);

int sysinfo_save(enum sysinfo_type type);
int sysinfo_load(enum sysinfo_type type);

int sysinfo_set(enum sysinfo_type type, void *info);
int sysinfo_get(enum sysinfo_type type, void *info);

#ifdef __cplusplus
}
#endif

#endif /* _SYSINFO_H_ */
