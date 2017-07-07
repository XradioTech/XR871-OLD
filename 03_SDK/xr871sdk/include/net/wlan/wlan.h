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

#ifndef _NET_WLAN_WLAN_H_
#define _NET_WLAN_WLAN_H_

#if (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE))

#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"
#include "lwip/netif.h"
#include "net/wlan/wpa_defs.h"
#include "net/wlan/wpa_ctrl_req.h"

#ifdef __cplusplus
extern "C" {
#endif

struct wlan_smart_config_result {
	uint8_t valid;
	/**
	 * ssid: network name in one of the optional formats:
	 *   - an ASCII string with double quotation, length is [0, 32] + 2
	 *   - a hex string (two characters per octet of SSID), length is [0, 64]
	 *   - a printf-escaped ASCII string P"<escaped string>"
	 */
	uint8_t ssid[65];

	/**
	 * psk: WPA preshared key in one of the optional formats:
	 *   - an ASCII string with double quotation, length is [8, 63] + 2
	 *   - a hex string (two characters per octet of PSK), length is 64
	 */
	uint8_t psk[66];
	uint32_t random_num;
};

struct wlan_config_info {
	uint8_t valid;

	/**
	 * ssid: network name in one of the optional formats:
	 *   - an ASCII string with double quotation, length is [0, 32] + 2
	 *   - a hex string (two characters per octet of SSID), length is [0, 64]
	 *   - a printf-escaped ASCII string P"<escaped string>"
	 */
	uint8_t ssid[65];

	/**
	 * scan_ssid - Scan this SSID with Probe Requests
	 *
	 * scan_ssid can be used to scan for APs using hidden SSIDs.
	 * Note: Many drivers do not support this. ap_mode=2 can be used with
	 * such drivers to use hidden SSIDs.
	 */
	int scan_ssid;

	/**
	 * psk: WPA preshared key in one of the optional formats:
	 *   - an ASCII string with double quotation, length is [8, 63] + 2
	 *   - a hex string (two characters per octet of PSK), length is 64
	 */
	uint8_t psk[66];

	/**
	 * key_mgmt - Bitfield of allowed key management protocols
	 *
	 * WPA_KEY_MGMT_*
	 */
	uint32_t key_mgmt;

#define NUM_WEP_KEYS 4
	/**
	 * wep_key - WEP keys
	 */
	uint8_t wep_key[NUM_WEP_KEYS][27];

	/**
	 * wep_tx_keyidx - Default key index for TX frames using WEP
	 */
	int wep_tx_keyidx;
	int wep_alg_used;
};

typedef void (*wlan_ctrl_cb_func)(enum wpa_ctrl_cmd cmd, struct wpa_ctrl_req_network *req);

static __inline int wlan_attach(void)
{
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_ATTACH, NULL);
}

static __inline int wlan_detach(void)
{
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_DETACH, NULL);
}

static __inline struct netif *wlan_if_create(enum wlan_mode mode)
{
	return ethernetif_create(mode);
}

static __inline int wlan_if_delete(struct netif *nif)
{
	ethernetif_delete(nif);
	return 0;
}

static __inline enum wlan_mode wlan_if_get_mode(struct netif *nif)
{
	return ethernetif_get_mode(nif);
}

int wlan_start(struct netif *nif);
int wlan_stop(void); /* Note: make sure wlan is disconnect before calling wlan_stop() */
int wlan_ctrl_request(enum wpa_ctrl_cmd cmd, void *data);
int wlan_set_mac_addr(uint8_t *mac_addr, int mac_len);
int wlan_set_ip_addr(void *ifp, uint8_t *ip_addr, int ip_len);

int wlan_smart_config_start(struct netif *nif);
int wlan_smart_config_stop(void);
int wlan_smart_config_set_key(char *key);

int wlan_airkiss_start(struct netif *nif);
int wlan_airkiss_stop(void);
int wlan_airkiss_set_key(char *key);
void wlan_airkiss_ack_start(struct wlan_smart_config_result *result, struct netif *netif);
void wlan_airkiss_online_ack_start();
void wlan_airkiss_online_ack_stop();

int wlan_sys_init(enum wlan_mode mode, ducc_cb_func cb, wlan_ctrl_cb_func ctrl_cb);
int wlan_sys_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE)) */

#endif /* _NET_WLAN_WLAN_H_ */
