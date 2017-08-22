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

#include "kernel/os/os.h"

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

int wlan_sta_set_ascii(char *ssid_ascii, char *psk_ascii)
{
	WLAN_ASSERT_POINTER(ssid_ascii);

	uint8_t ssid[35];
	uint8_t	psk[66];
	uint32_t len;

	len = wlan_strlen(ssid_ascii);
	len = len > 32 ? 32 : len;

	ssid[0] = '"';
	wlan_memcpy(ssid + 1, ssid_ascii, len);
	ssid[len + 1] = '"';
	ssid[len + 2] = '\0';

	if (psk_ascii) {
		len = wlan_strlen(psk_ascii);
		len = len > 63 ? 63 : len;

		psk[0] = '"';
		wlan_memcpy(psk + 1, psk_ascii, len);
		psk[len + 1] = '"';
		psk[len + 2] = '\0';

		return wlan_sta_set(ssid, psk);
	}

	return wlan_sta_set(ssid, NULL);
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

int wlan_sta_state(wlan_sta_states_t *state)
{
	return wpa_ctrl_request(WPA_CTRL_CMD_STA_STATE, state);
}

int wlan_sta_ap_info(wlan_sta_ap_t *ap)
{
	WLAN_ASSERT_POINTER(ap);

	return wpa_ctrl_request(WPA_CTRL_CMD_STA_AP, ap);
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

int wlan_ap_set_ascii(char *ssid_ascii, char *psk_ascii)
{
	WLAN_ASSERT_POINTER(ssid_ascii);

	uint8_t ssid[35];
	uint8_t	psk[66];
	uint32_t len;

	len = wlan_strlen(ssid_ascii);
	len = len > 32 ? 32 : len;

	ssid[0] = '"';
	wlan_memcpy(ssid + 1, ssid_ascii, len);
	ssid[len + 1] = '"';
	ssid[len + 2] = '\0';

	if (psk_ascii) {
		len = wlan_strlen(psk_ascii);
		len = len > 63 ? 63 : len;

		psk[0] = '"';
		wlan_memcpy(psk + 1, psk_ascii, len);
		psk[len + 1] = '"';
		psk[len + 2] = '\0';

		return wlan_ap_set(ssid, psk);
	}

	return wlan_ap_set(ssid, NULL);
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

typedef struct {
	OS_Timer_t *timer;
	void *arg;
	void (*callback)(void *arg);
} Wlan_Time_Out_Cfg;

static void wlan_time_out(void *arg)
{
	Wlan_Time_Out_Cfg *param = (Wlan_Time_Out_Cfg *)arg;

	WLAN_WARN("%s(), %d, Time out\n", __func__, __LINE__);
	param->callback(param->arg);
}

static int wlan_time_out_enable(uint32_t time_out_ms, Wlan_Time_Out_Cfg *cfg)
{
	if (time_out_ms == 0)
		return 0;

	OS_Status ret = OS_TimerCreate(cfg->timer, OS_TIMER_ONCE,
                        		   wlan_time_out,
                        		   cfg, time_out_ms);
	if (ret != OS_OK) {
		WLAN_ERR("%s(), %d, create os_time error \n", __func__, __LINE__);
		OS_TimerStop(cfg->timer);
		OS_TimerDelete(cfg->timer);
		return -1;
	}

	OS_TimerStart(cfg->timer);
	return 0;
}

static void wlan_time_out_clear(OS_Timer_t *timer)
{
	OS_Status ret;
	if (OS_TimerIsValid(timer)) {
		ret = OS_TimerStop(timer);
		if (ret != OS_OK)
			WLAN_ERR("%s(), %d, OS_TimerStop error \n", __func__, __LINE__);

		ret = OS_TimerDelete(timer);
		if (ret != OS_OK)
			WLAN_ERR("%s(), %d, OS_TimerDelete error \n", __func__, __LINE__);
	}
}

/* smart config */
static OS_Timer_t sc_timer;
static Wlan_Time_Out_Cfg sc_time_out_cfg;

int wlan_smart_config_start(struct netif *nif, uint32_t time_out_ms)
{
	WLAN_ASSERT_POINTER(nif);

	sc_time_out_cfg.callback = (void *) wlan_smart_config_stop;
	sc_time_out_cfg.timer = &sc_timer;
	/* disconnect */
	if (wpa_ctrl_request(WPA_CTRL_CMD_STA_DISCONNECT, NULL) != 0)
		return -1;

	if (wlan_time_out_enable(time_out_ms, &sc_time_out_cfg) != 0)
		return -1;

	/* scan and get results */
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_SMART_CONFIG_START, nif->state);
}

int wlan_smart_config_stop(void)
{
	wlan_time_out_clear(sc_time_out_cfg.timer);

	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_SMART_CONFIG_STOP, NULL);
}

#define SC_KEY_LEN 16

int wlan_smart_config_set_key(char *key)
{
	WLAN_ASSERT_POINTER(key);
	if (strlen(key) != SC_KEY_LEN) {
		WLAN_ERR("%s(), %d, smart config set key error\n", __func__, __LINE__);
		return -1;
	}

	char sc_key_buf[SC_KEY_LEN + 1];
	memcpy(sc_key_buf, key, SC_KEY_LEN);
	sc_key_buf[SC_KEY_LEN] = 0;

	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_SMART_CONFIG_SET_KEY, sc_key_buf);
}

/* airkiss */
static OS_Timer_t ak_timer;
static Wlan_Time_Out_Cfg ak_time_out_cfg;

int wlan_airkiss_start(struct netif *nif, uint32_t time_out_ms)
{
	WLAN_ASSERT_POINTER(nif);

	ak_time_out_cfg.callback = (void *) wlan_airkiss_stop;
	ak_time_out_cfg.timer = &ak_timer;

	/* disconnect */
	if (wpa_ctrl_request(WPA_CTRL_CMD_STA_DISCONNECT, NULL) != 0)
		return -1;

	if (wlan_time_out_enable(time_out_ms, &ak_time_out_cfg) != 0)
		return -1;

	/* scan and get results */
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_AIRKISS_START, nif->state);
}

int wlan_airkiss_stop(void)
{
	wlan_time_out_clear(ak_time_out_cfg.timer);

	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_AIRKISS_STOP, NULL);
}

#define AK_KEY_LEN 16

int wlan_airkiss_set_key(char *key)
{
	WLAN_ASSERT_POINTER(key);
	if (strlen(key) != AK_KEY_LEN) {
		WLAN_ERR("%s(), %d, airkiss set key error\n", __func__, __LINE__);
		return -1;
	}

	char ak_key_buf[AK_KEY_LEN + 1];
	memcpy(ak_key_buf, key, AK_KEY_LEN);
	ak_key_buf[AK_KEY_LEN] = 0;

	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_AIRKISS_SET_KEY, ak_key_buf);
}


int wlan_airkiss_ack_start(wlan_smart_config_result_t *result, struct netif *netif)
{
	WLAN_ASSERT_POINTER(result);
	WLAN_ASSERT_POINTER(netif);

	wlan_time_out_clear(ak_time_out_cfg.timer);

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
