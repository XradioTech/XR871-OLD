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

#if (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE))

#include "wlan_debug.h"
#include "wpa_ctrl_req.h"
#include "lwip/netif.h"
#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"
#include "airkiss/airkiss_ack.h"
#include "airkiss/airkiss_discover.h"

#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"

#define WLAN_ASSERT_POINTER(p)						\
	do {								\
		if (p == NULL) {					\
			WLAN_ERR("%s: invalid parameter\n", __func__);	\
			return -1;					\
		}							\
	} while (0)

/* STA */
int wlan_sta_set(uint8_t *ssid, uint8_t *psk)
{
	WLAN_ASSERT_POINTER(ssid);

	wlan_sta_config_t config;
	wlan_memset(&config, 0, sizeof(config));

	/* ssid */
	config.field = WLAN_STA_FIELD_SSID;
	wlan_strlcpy((char *)config.u.ssid, (char *)ssid, sizeof(config.u.ssid));
	if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
		return -1;

	/* auth_alg: OPEN */
	config.field = WLAN_STA_FIELD_AUTH_ALG;
	config.u.auth_alg = WPA_AUTH_ALG_OPEN;
	if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
		return -1;

	if (psk == NULL) {
		/* key_mgmt: NONE */
		config.field = WLAN_STA_FIELD_KEY_MGMT;
		config.u.key_mgmt = WPA_KEY_MGMT_NONE;
		if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
			return -1;
	} else {
		/* psk */
		config.field = WLAN_STA_FIELD_PSK;
		wlan_strlcpy((char *)config.u.psk, (char *)psk, sizeof(config.u.psk));
		if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
			return -1;

		/* proto: WPA | RSN */
		config.field = WLAN_STA_FIELD_PROTO;
		config.u.proto = WPA_PROTO_WPA | WPA_PROTO_RSN;
		if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
			return -1;

		/* key_mgmt: PSK */
		config.field = WLAN_STA_FIELD_KEY_MGMT;
		config.u.key_mgmt = WPA_KEY_MGMT_PSK;
		if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
			return -1;

		/* pairwise: CCMP | TKIP */
		config.field = WLAN_STA_FIELD_PAIRWISE_CIPHER;
		config.u.pairwise_cipher = WPA_CIPHER_CCMP | WPA_CIPHER_TKIP;
		if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
			return -1;

		/* group: CCMP | TKIP | WEP40 | WEP104 */
		config.field = WLAN_STA_FIELD_GROUP_CIPHER;
		config.u.pairwise_cipher = WPA_CIPHER_CCMP | WPA_CIPHER_TKIP
					   | WPA_CIPHER_WEP40 | WPA_CIPHER_WEP104;
		if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
			return -1;
	}

	/* scan_ssid: 1 */
	config.field = WLAN_STA_FIELD_SCAN_SSID;
	config.u.scan_ssid = 1;
	if (wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, &config) != 0)
		return -1;

	return 0;
}

int wlan_sta_set_config(wlan_sta_config_t *config)
{
	WLAN_ASSERT_POINTER(config);

	return wpa_ctrl_request(WPA_CTRL_CMD_STA_SET, config);
}

int wlan_sta_get_config(wlan_sta_config_t *config)
{
	WLAN_ASSERT_POINTER(config);

	return wpa_ctrl_request(WPA_CTRL_CMD_STA_GET, config);
}

int wlan_sta_enable(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_STA_ENABLE, NULL);
}

int wlan_sta_disable(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_STA_DISABLE, NULL);
}

int wlan_sta_scan_once(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_STA_SCAN, NULL);
}

int wlan_sta_scan_result(wlan_sta_scan_results_t *results)
{
	WLAN_ASSERT_POINTER(results);

	return wpa_ctrl_request(WPA_CTRL_CMD_STA_SCAN_RESULTS, results);
}

int wlan_sta_bss_flush(int age)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_STA_BSS_FLUSH, (void *)age);
}

int wlan_sta_connect(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_STA_REASSOCIATE, NULL);
}

int wlan_sta_disconnect(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_STA_DISCONNECT, NULL);
}

int wlan_sta_wps_pbc(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_STA_WPS_PBC, NULL);
}

int wlan_sta_wps_pin_get(wlan_sta_wps_pin_t *wps)
{
	WLAN_ASSERT_POINTER(wps);

	return wpa_ctrl_request(WPA_CTRL_CMD_STA_WPS_GET_PIN, wps);
}

int wlan_sta_wps_pin_set(wlan_sta_wps_pin_t *wps)
{
	WLAN_ASSERT_POINTER(wps);

	return wpa_ctrl_request(WPA_CTRL_CMD_STA_WPS_SET_PIN, wps);
}

/* softAP */
int wlan_ap_set(uint8_t *ssid, uint8_t *psk)
{
	WLAN_ASSERT_POINTER(ssid);

	wlan_ap_config_t config;
	wlan_memset(&config, 0, sizeof(config));

	/* ssid */
	config.field = WLAN_AP_FIELD_SSID;
	wlan_strlcpy((char *)config.u.ssid, (char *)ssid, sizeof(config.u.ssid));
	if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
		return -1;

	/* auth_alg: OPEN */
	config.field = WLAN_AP_FIELD_AUTH_ALG;
	config.u.auth_alg = WPA_AUTH_ALG_OPEN;
	if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
		return -1;

	if (psk == NULL) {
		/* proto: 0 */
		config.field = WLAN_AP_FIELD_PROTO;
		config.u.proto = 0;
		if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
			return -1;

		/* key_mgmt: NONE */
		config.field = WLAN_AP_FIELD_KEY_MGMT;
		config.u.key_mgmt = WPA_KEY_MGMT_NONE;
		if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
			return -1;
	} else {
		/* psk */
		config.field = WLAN_AP_FIELD_PSK;
		wlan_strlcpy((char *)config.u.psk, (char *)psk, sizeof(config.u.psk));
		if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
			return -1;

		/* proto: WPA | RSN */
		config.field = WLAN_AP_FIELD_PROTO;
		config.u.proto = WPA_PROTO_WPA | WPA_PROTO_RSN;
		if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
			return -1;

		/* key_mgmt: PSK */
		config.field = WLAN_AP_FIELD_KEY_MGMT;
		config.u.key_mgmt = WPA_KEY_MGMT_PSK;
		if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
			return -1;
	}

	/* wpa_cipher: TKIP */
	config.field = WLAN_AP_FIELD_WPA_CIPHER;
	config.u.wpa_cipher = WPA_CIPHER_TKIP;
	if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
		return -1;

	/* rsn_cipher: CCMP */
	config.field = WLAN_AP_FIELD_RSN_CIPHER;
	config.u.rsn_cipher = WPA_CIPHER_CCMP;
	if (wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, &config) != 0)
		return -1;

	return 0;
}

int wlan_ap_set_config(wlan_ap_config_t *config)
{
	WLAN_ASSERT_POINTER(config);

	return wpa_ctrl_request(WPA_CTRL_CMD_AP_SET, config);
}

int wlan_ap_get_config(wlan_ap_config_t *config)
{
	WLAN_ASSERT_POINTER(config);

	return wpa_ctrl_request(WPA_CTRL_CMD_AP_GET, config);
}

int wlan_ap_enable(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_AP_ENABLE, NULL);
}

int wlan_ap_reload(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_AP_RELOAD, NULL);
}

int wlan_ap_disable(void)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_AP_DISABLE, NULL);
}

int wlan_ap_sta_num(int *num)
{
	WLAN_ASSERT_POINTER(num);

	return wpa_ctrl_request(WPA_CTRL_CMD_AP_STA_NUM, num);
}

int wlan_ap_sta_info(wlan_ap_stas_t *stas)
{
	WLAN_ASSERT_POINTER(stas);

	return wpa_ctrl_request(WPA_CTRL_CMD_AP_STA_INFO, stas);
}

/* smart config */
int wlan_smart_config_start(struct netif *nif)
{
	WLAN_ASSERT_POINTER(nif);

	/* disconnect */
	if (wpa_ctrl_request(WPA_CTRL_CMD_STA_DISCONNECT, NULL) != 0)
		return -1;

	/* scan and get results */
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_SMART_CONFIG_START, nif->state);
}

int wlan_smart_config_stop(void)
{
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_SMART_CONFIG_STOP, NULL);
}

int wlan_smart_config_set_key(char *key)
{
	WLAN_ASSERT_POINTER(key);

	if (wlan_strlen(key) != 16) {
		WLAN_ERR("%s(), %d, smart config set key error\n", __func__, __LINE__);
		return -1;
	}

	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_SMART_CONFIG_SET_KEY, key);
}

/* airkiss */
int wlan_airkiss_start(struct netif *nif)
{
	WLAN_ASSERT_POINTER(nif);

	/* disconnect */
	if (wpa_ctrl_request(WPA_CTRL_CMD_STA_DISCONNECT, NULL) != 0)
		return -1;

	/* scan and get results */
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_AIRKISS_START, nif->state);
}

int wlan_airkiss_stop(void)
{
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_AIRKISS_STOP, NULL);
}

int wlan_airkiss_set_key(char *key)
{
	WLAN_ASSERT_POINTER(key);

	if (wlan_strlen(key) != 16) {
		WLAN_ERR("%s(), %d, airkiss set key error\n", __func__, __LINE__);
		return -1;
	}

	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_AIRKISS_SET_KEY, key);
}

int wlan_airkiss_ack_start(wlan_smart_config_result_t *result, struct netif *netif)
{
	WLAN_ASSERT_POINTER(result);
	WLAN_ASSERT_POINTER(netif);

	if(result->valid)
		airkiss_ack_start(result->random_num, netif);

	return 0;
}

int wlan_airkiss_online_cycle_ack_start(char *app_id, char *drv_id, uint32_t period_ms)
{
	Airkiss_Online_Ack_Info param;
	param.app_id = app_id;
	param.device_id = drv_id;
	param.ack_period_ms = period_ms;
	return airkiss_online_cycle_ack_start(&param);
}

void wlan_airkiss_online_cycle_ack_stop(void)
{
	airkiss_online_cycle_ack_stop();
}

int  wlan_airkiss_online_dialog_mode_start(char *app_id, char *drv_id, uint32_t period_ms)
{
	Airkiss_Online_Ack_Info param;
	param.app_id = app_id;
	param.device_id = drv_id;
	param.ack_period_ms = period_ms;
	return airkiss_online_dialog_mode_start(&param);
}

void wlan_airkiss_online_dialog_mode_stop(void)
{
	airkiss_online_dialog_mode_stop();
}

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE)) */
