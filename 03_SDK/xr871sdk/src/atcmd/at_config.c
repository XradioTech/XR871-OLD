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

#include "atcmd/at_command.h"
#include "at_private.h"
#include "at_debug.h"

#define VERIFY_FUNC(name) name##_verify
#define VERIFY_DEFINE(name) static s32 VERIFY_FUNC(name)(at_value_t *value)

extern s32 at_get_value(char *strbuf, s32 pt, void *pvar, s32 vsize);
extern s32 at_set_value(s32 pt, void *pvar, s32 vsize, at_value_t *value);

//VERIFY_DEFINE(blink_led);
VERIFY_DEFINE(localecho1);
VERIFY_DEFINE(console1_speed);
VERIFY_DEFINE(console1_hwfc);
//VERIFY_DEFINE(console1_enabled);
//VERIFY_DEFINE(sleep_enabled);
//VERIFY_DEFINE(standby_enabled);
//VERIFY_DEFINE(standby_time);
//VERIFY_DEFINE(wifi_beacon_wakeup);
//VERIFY_DEFINE(wifi_beacon_interval);
//VERIFY_DEFINE(wifi_listen_interval);
//VERIFY_DEFINE(wifi_rts_threshold);
VERIFY_DEFINE(wifi_ssid_len);
VERIFY_DEFINE(wifi_mode);
//VERIFY_DEFINE(wifi_auth_type);
//VERIFY_DEFINE(wifi_powersave);
//VERIFY_DEFINE(wifi_tx_power);
//VERIFY_DEFINE(wifi_priv_mode);
VERIFY_DEFINE(ip_use_dhcp);
//VERIFY_DEFINE(ip_use_httpd);
//VERIFY_DEFINE(ip_mtu);
//VERIFY_DEFINE(ip_sockd_timeout);

at_config_t at_cfg;

static const at_var_descriptor_t at_cfg_table[] = {
	{"nv_manuf",				APT_TEXT,		APO_RO,		&at_cfg.nv_manuf,					sizeof(at_cfg.nv_manuf),					NULL},
	{"nv_model",				APT_TEXT,		APO_RO,		&at_cfg.nv_model,					sizeof(at_cfg.nv_model),					NULL},
	{"nv_serial",				APT_TEXT,		APO_RO,		&at_cfg.nv_serial,					sizeof(at_cfg.nv_serial),					NULL},
	{"nv_wifi_macaddr",			APT_HEX,		APO_RO,		&at_cfg.nv_wifi_macaddr,			sizeof(at_cfg.nv_wifi_macaddr),				NULL},
	//{"blink_led",				APT_DI,			APO_RW,		&at_cfg.blink_led,					sizeof(at_cfg.blink_led),					VERIFY_FUNC(blink_led)},
	{"wind_off_low",			APT_HI,			APO_RW,		&at_cfg.wind_off_low,				sizeof(at_cfg.wind_off_low),				NULL},
	{"wind_off_medium",			APT_HI,			APO_RW,		&at_cfg.wind_off_medium,			sizeof(at_cfg.wind_off_medium),				NULL},
	{"wind_off_high",			APT_HI,			APO_RW,		&at_cfg.wind_off_high,				sizeof(at_cfg.wind_off_high),				NULL},
	{"user_desc",				APT_TEXT,		APO_RW,		&at_cfg.user_desc,					sizeof(at_cfg.user_desc),					NULL},
	{"escape_seq",				APT_TEXT,		APO_RW,		&at_cfg.escape_seq,					sizeof(at_cfg.escape_seq),					NULL},
	{"localecho1",				APT_DI,			APO_RW,		&at_cfg.localecho1,					sizeof(at_cfg.localecho1),					VERIFY_FUNC(localecho1)},
	{"console1_speed",			APT_DI,			APO_RW,		&at_cfg.console1_speed,				sizeof(at_cfg.console1_speed),				VERIFY_FUNC(console1_speed)},
	{"console1_hwfc",			APT_DI,			APO_RW,		&at_cfg.console1_hwfc,				sizeof(at_cfg.console1_hwfc),				VERIFY_FUNC(console1_hwfc)},
	//{"console1_enabled",		APT_DI,			APO_RW,		&at_cfg.console1_enabled,			sizeof(at_cfg.console1_enabled),			VERIFY_FUNC(console1_enabled)},
	//{"sleep_enabled",			APT_DI,			APO_RW,		&at_cfg.sleep_enabled,				sizeof(at_cfg.sleep_enabled),				VERIFY_FUNC(sleep_enabled)},
	//{"standby_enabled",			APT_DI,			APO_RW,		&at_cfg.standby_enabled,			sizeof(at_cfg.standby_enabled),				VERIFY_FUNC(standby_enabled)},
	//{"standby_time",			APT_DI,			APO_RW,		&at_cfg.standby_time,				sizeof(at_cfg.standby_time),				VERIFY_FUNC(standby_time)},
	//{"wifi_tx_msdu_lifetime",	APT_DI,			APO_RW,		&at_cfg.wifi_tx_msdu_lifetime,		sizeof(at_cfg.wifi_tx_msdu_lifetime),		NULL},
	//{"wifi_rx_msdu_lifetime",	APT_DI,			APO_RW,		&at_cfg.wifi_rx_msdu_lifetime,		sizeof(at_cfg.wifi_rx_msdu_lifetime),		NULL},
	//{"wifi_operational_mode",	APT_HI,			APO_RW,		&at_cfg.wifi_operational_mode,		sizeof(at_cfg.wifi_operational_mode),		NULL}, /* */
	//{"wifi_beacon_wakeup",		APT_DI,			APO_RW,		&at_cfg.wifi_beacon_wakeup,			sizeof(at_cfg.wifi_beacon_wakeup),			VERIFY_FUNC(wifi_beacon_wakeup)},
	//{"wifi_beacon_interval",	APT_DI,			APO_RW,		&at_cfg.wifi_beacon_interval,		sizeof(at_cfg.wifi_beacon_interval),		VERIFY_FUNC(wifi_beacon_interval)},
	//{"wifi_listen_interval",	APT_DI,			APO_RW,		&at_cfg.wifi_listen_interval,		sizeof(at_cfg.wifi_listen_interval),		VERIFY_FUNC(wifi_listen_interval)},
	//{"wifi_rts_threshold",		APT_DI,			APO_RW,		&at_cfg.wifi_rts_threshold,			sizeof(at_cfg.wifi_rts_threshold),			VERIFY_FUNC(wifi_rts_threshold)},
	{"wifi_channelnum",			APT_DI,			APO_RW,		&at_cfg.wifi_channelnum,			sizeof(at_cfg.wifi_channelnum),				NULL},
	//{"wifi_opr_rate_mask",		APT_HI,			APO_RW,		&at_cfg.wifi_opr_rate_mask,			sizeof(at_cfg.wifi_opr_rate_mask),			NULL},
	//{"wifi_bas_rate_mask",		APT_HI,			APO_RW,		&at_cfg.wifi_bas_rate_mask,			sizeof(at_cfg.wifi_bas_rate_mask),			NULL},
	{"wifi_mode",				APT_DI,			APO_RW,		&at_cfg.wifi_mode,					sizeof(at_cfg.wifi_mode),					VERIFY_FUNC(wifi_mode)},
	//{"wifi_auth_type",			APT_DI,			APO_RW,		&at_cfg.wifi_auth_type,				sizeof(at_cfg.wifi_auth_type),				VERIFY_FUNC(wifi_auth_type)},
	//{"wifi_powersave",			APT_DI,			APO_RW,		&at_cfg.wifi_powersave,				sizeof(at_cfg.wifi_powersave),				VERIFY_FUNC(wifi_powersave)},
	//{"wifi_tx_power",			APT_DI,			APO_RW,		&at_cfg.wifi_tx_power,				sizeof(at_cfg.wifi_tx_power),				VERIFY_FUNC(wifi_tx_power)},
	//{"wifi_rssi_thresh",		APT_DI,			APO_RW,		&at_cfg.wifi_rssi_thresh,			sizeof(at_cfg.wifi_rssi_thresh),			NULL},
	//{"wifi_rssi_hyst",			APT_DI,			APO_RW,		&at_cfg.wifi_rssi_hyst,				sizeof(at_cfg.wifi_rssi_hyst),				NULL},
	//{"wifi_ap_idle_timeout",	APT_DI,			APO_RW,		&at_cfg.wifi_ap_idle_timeout,		sizeof(at_cfg.wifi_ap_idle_timeout),		NULL},
	//{"wifi_beacon_loss_thresh",	APT_DI,			APO_RW,		&at_cfg.wifi_beacon_loss_thresh,	sizeof(at_cfg.wifi_beacon_loss_thresh),		NULL},
	//{"wifi_priv_mode",			APT_DI,			APO_RW,		&at_cfg.wifi_priv_mode,				sizeof(at_cfg.wifi_priv_mode),				VERIFY_FUNC(wifi_priv_mode)},
	//{"wifi_wep_keys[0]",		APT_HEX,		APO_RW,		&at_cfg.wifi_wep_keys[0][0],		16,											NULL},
	//{"wifi_wep_keys[1]",		APT_HEX,		APO_RW,		&at_cfg.wifi_wep_keys[1][0],		16,											NULL},
	//{"wifi_wep_keys[2]",		APT_HEX,		APO_RW,		&at_cfg.wifi_wep_keys[2][0],		16,											NULL},
	//{"wifi_wep_keys[3]",		APT_HEX,		APO_RW,		&at_cfg.wifi_wep_keys[3][0],		16,											NULL},
	//{"wifi_wep_key_lens",		APT_HEX,		APO_RW,		&at_cfg.wifi_wep_key_lens,			sizeof(at_cfg.wifi_wep_key_lens),			NULL},
	//{"wifi_wep_default_key",	APT_DI,			APO_RW,		&at_cfg.wifi_wep_default_key,		sizeof(at_cfg.wifi_wep_default_key),		NULL},
	//{"wifi_wpa_psk_raw",		APT_HEX,		APO_RW,		&at_cfg.wifi_wpa_psk_raw,			sizeof(at_cfg.wifi_wpa_psk_raw),			NULL},
	{"wifi_ssid",				APT_HEX,		APO_RW,		&at_cfg.wifi_ssid,					sizeof(at_cfg.wifi_ssid),					NULL},
	{"wifi_ssid_len",			APT_DI,			APO_RW,		&at_cfg.wifi_ssid_len,				sizeof(at_cfg.wifi_ssid_len),				VERIFY_FUNC(wifi_ssid_len)},
	{"wifi_wpa_psk_text",		APT_TEXT,		APO_RW,		&at_cfg.wifi_wpa_psk_text,			sizeof(at_cfg.wifi_wpa_psk_text),			NULL},
	{"ip_use_dhcp",				APT_DI,			APO_RW,		&at_cfg.ip_use_dhcp,				sizeof(at_cfg.ip_use_dhcp),					VERIFY_FUNC(ip_use_dhcp)},
	//{"ip_use_httpd",			APT_DI,			APO_RW,		&at_cfg.ip_use_httpd,				sizeof(at_cfg.ip_use_httpd),				VERIFY_FUNC(ip_use_httpd)},
	//{"ip_mtu",					APT_DI,			APO_RW,		&at_cfg.ip_mtu,						sizeof(at_cfg.ip_mtu),						VERIFY_FUNC(ip_mtu)},
	//{"ip_hostname",				APT_TEXT,		APO_RW,		&at_cfg.ip_hostname,				sizeof(at_cfg.ip_hostname),					NULL},
	//{"ip_apdomainname",			APT_TEXT,		APO_RW,		&at_cfg.ip_apdomainname,			sizeof(at_cfg.ip_apdomainname),				NULL},
	{"ip_ipaddr",				APT_IP,			APO_RW,		&at_cfg.ip_ipaddr,					sizeof(at_cfg.ip_ipaddr),					NULL},
	{"ip_netmask",				APT_IP,			APO_RW,		&at_cfg.ip_netmask,					sizeof(at_cfg.ip_netmask),					NULL},
	{"ip_gw",					APT_IP,			APO_RW,		&at_cfg.ip_gw,						sizeof(at_cfg.ip_gw),						NULL},
	{"ip_dns",					APT_IP,			APO_RW,		&at_cfg.ip_dns,						sizeof(at_cfg.ip_dns),						NULL},
	//{"ip_http_get_recv_timeout",APT_DI,			APO_RW,		&at_cfg.ip_http_get_recv_timeout,	sizeof(at_cfg.ip_http_get_recv_timeout),	NULL},
	//{"ip_dhcp_timeout",			APT_DI,			APO_RW,		&at_cfg.ip_dhcp_timeout,			sizeof(at_cfg.ip_dhcp_timeout),				NULL},
	//{"ip_sockd_timeout",		APT_DI,			APO_RW,		&at_cfg.ip_sockd_timeout,			sizeof(at_cfg.ip_sockd_timeout),			VERIFY_FUNC(ip_sockd_timeout)},
};

AT_ERROR_CODE at_getcfg(char *key)
{
	char strbuf[AT_PARA_MAX_SIZE*4];
	s32 i;

	if (key == NULL) {
		return AEC_NULL_POINTER; /* null pointer */
	}

	for (i = 0; i < TABLE_SIZE(at_cfg_table); i++) {
		if (!strcmp(key, at_cfg_table[i].key)) {
			at_get_value(strbuf, at_cfg_table[i].pt, at_cfg_table[i].pvar, at_cfg_table[i].vsize);

			at_dump("# %s = %s\r\n", at_cfg_table[i].key, strbuf);

			return AEC_OK; /* succeed */
		}
	}

	return AEC_NOT_FOUND; /* not found */
}

AT_ERROR_CODE at_typecfg(char *key)
{
	s32 i;

	if (key == NULL) {
		return AEC_NULL_POINTER; /* null pointer */
	}

	for (i = 0; i < TABLE_SIZE(at_cfg_table); i++) {
		if (!strcmp(key, at_cfg_table[i].key)) {
			return at_cfg_table[i].pt; /* succeed */
		}
	}

	return AEC_NOT_FOUND; /* not found */
}

AT_ERROR_CODE at_setcfg(char *key, at_value_t *value)
{
	s32 i;

	if (key == NULL || value == NULL) {
		return AEC_NOT_FOUND; /* null pointer */
	}

	for (i = 0; i < TABLE_SIZE(at_cfg_table); i++) {
		if (!strcmp(key, at_cfg_table[i].key)) {
			if (at_cfg_table[i].po != APO_RW) {
				return AEC_READ_ONLY; /* read only */
			}

			if (at_cfg_table[i].verify != NULL) {
				if (at_cfg_table[i].verify(value) != 0) {
					return AEC_OUT_OF_RANGE; /* out of range */
				}
			}

			at_set_value(at_cfg_table[i].pt, at_cfg_table[i].pvar, at_cfg_table[i].vsize, value);

			return AEC_OK; /* succeed */
		}
	}

	return AEC_NOT_FOUND; /* not found */
}

AT_ERROR_CODE at_ssidtxt(char *ssid)
{
	if (ssid != NULL) {
		s32 len;

		len = strlen(ssid);
		memset(at_cfg.wifi_ssid, 0, sizeof(at_cfg.wifi_ssid));
		memcpy(at_cfg.wifi_ssid, ssid, len);
		at_cfg.wifi_ssid_len = len;

		return AEC_OK; /* succeed */
	}
	else {
		return AEC_NOT_FOUND; /* null pointer */
	}
}

AT_ERROR_CODE at_config(void)
{
	char strbuf[AT_PARA_MAX_SIZE*4];
	s32 i;

	for (i = 0; i < TABLE_SIZE(at_cfg_table); i++) {
		at_get_value(strbuf, at_cfg_table[i].pt, at_cfg_table[i].pvar, at_cfg_table[i].vsize);

		at_dump("# %s = %s\r\n", at_cfg_table[i].key, strbuf);
	}

	return AEC_OK;
}

AT_ERROR_CODE at_factory(void)
{
	at_callback_para_t para;

	para.cfg = &at_cfg;

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_FACTORY, &para, NULL);
	}

	return AEC_OK;
}

AT_ERROR_CODE at_save(void)
{
	at_callback_para_t para;

	para.cfg = &at_cfg;

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_SAVE, &para, NULL);
	}

	return AEC_OK;
}
#if 0
VERIFY_DEFINE(blink_led)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}
#endif
VERIFY_DEFINE(localecho1)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(console1_speed)
{
	at_di_t speed_tbl[] = {9600, 19200, 38400, 115200, 921600};
	s32 i;
	s32 res = 1; /* Error */

	for (i = 0; i < TABLE_SIZE(speed_tbl); i++) {
		if (value->di == speed_tbl[i]) {
			res = 0; /* OK */
			break;
		}
	}

	return res;
}

VERIFY_DEFINE(console1_hwfc)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}
#if 0
VERIFY_DEFINE(console1_enabled)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(sleep_enabled)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(standby_enabled)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(standby_time)
{
	if (value->di >= 0 && value->di <= 232-1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(wifi_beacon_wakeup)
{
	if (at_cfg.wifi_listen_interval == 0) {
		if (value->di >= 0 && value->di <= 255) {
			return 0; /* OK */
		}
		else {
			return 1; /* Error */
		}
	}
	else if (at_cfg.wifi_listen_interval == 1) {
		if (value->di >= 0 && value->di <= 65535) {
			return 0; /* OK */
		}
		else {
			return 1; /* Error */
		}
	}
	else {
		return 2; /* Error */
	}

}

VERIFY_DEFINE(wifi_beacon_interval)
{
	if (value->di >= 0 && value->di <= 65535) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(wifi_listen_interval)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(wifi_rts_threshold)
{
	if (value->di >= 0 && value->di <= 3000) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}
#endif
VERIFY_DEFINE(wifi_ssid_len)
{
	if (value->di >= 0 && value->di <= 32) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(wifi_mode)
{
	if (value->di >= 0 && value->di <= 3) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}
#if 0
VERIFY_DEFINE(wifi_auth_type)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(wifi_powersave)
{
	if (value->di >= 0 && value->di <= 2) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(wifi_tx_power)
{
	if (value->di >= 0 && value->di <= 18) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(wifi_priv_mode)
{
	if (value->di >= 0 && value->di <= 2) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}
#endif
VERIFY_DEFINE(ip_use_dhcp)
{
	if (value->di >= 0 && value->di <= 2) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}
#if 0
VERIFY_DEFINE(ip_use_httpd)
{
	if (value->di >= 0 && value->di <= 1) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(ip_mtu)
{
	if (value->di >= 634 && value->di <= 2412) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}

VERIFY_DEFINE(ip_sockd_timeout)
{
	if (value->di >= 5 && value->di <= 250) {
		return 0; /* OK */
	}
	else {
		return 1; /* Error */
	}
}
#endif