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

#if (defined(__CONFIG_ARCH_DUAL_CORE))

#include "cmd_debug.h"
#include "cmd_util.h"
#include "cmd_wpas.h"
#include "net/wlan/wlan.h"

int wpas_parse_int(const char *value, int min, int max, int *dst)
{
	int val;
	char *end;

	val = cmd_strtol(value, &end, 0);
	if (*end) {
		CMD_ERR("Invalid number '%s'", value);
		return -1;
	}

	if (val < min || val > max) {
		CMD_ERR("out of range value %d (%s), range is [%d, %d]\n",
			 val, value, min, max);
		return -1;
	}

	*dst = val;
	return 0;
}

int waps_parse_key_mgmt(const char *value)
{
	int val = 0, last, errors = 0;
	char *start, *end, *buf;

	buf = cmd_strdup(value);
	if (buf == NULL)
		return -1;
	start = buf;

	while (*start != '\0') {
		while (*start == ' ' || *start == '\t')
			start++;
		if (*start == '\0')
			break;
		end = start;
		while (*end != ' ' && *end != '\t' && *end != '\0')
			end++;
		last = *end == '\0';
		*end = '\0';
		if (cmd_strcmp(start, "WPA-PSK") == 0)
			val |= WPA_KEY_MGMT_PSK;
		else if (cmd_strcmp(start, "WPA-EAP") == 0)
			val |= WPA_KEY_MGMT_IEEE8021X;
		else if (cmd_strcmp(start, "IEEE8021X") == 0)
			val |= WPA_KEY_MGMT_IEEE8021X_NO_WPA;
		else if (cmd_strcmp(start, "NONE") == 0)
			val |= WPA_KEY_MGMT_NONE;
		else if (cmd_strcmp(start, "WPA-NONE") == 0)
			val |= WPA_KEY_MGMT_WPA_NONE;
		else {
			CMD_DBG("Invalid key_mgmt '%s'", start);
			errors++;
		}

		if (last)
			break;
		start = end + 1;
	}
	cmd_free(buf);

	if (val == 0) {
		CMD_DBG("No key_mgmt values configured\n");
		errors++;
	}

	CMD_DBG("key_mgmt: 0x%x\n", val);
	return errors ? -1 : val;
}

int wpas_parse_cipher(const char *value)
{
	int val = 0, last;
	char *start, *end, *buf;

	buf = cmd_strdup(value);
	if (buf == NULL)
		return -1;
	start = buf;

	while (*start != '\0') {
		while (*start == ' ' || *start == '\t')
			start++;
		if (*start == '\0')
			break;
		end = start;
		while (*end != ' ' && *end != '\t' && *end != '\0')
			end++;
		last = *end == '\0';
		*end = '\0';
		if (cmd_strcmp(start, "CCMP-256") == 0)
			val |= WPA_CIPHER_CCMP_256;
		else if (cmd_strcmp(start, "GCMP-256") == 0)
			val |= WPA_CIPHER_GCMP_256;
		else if (cmd_strcmp(start, "CCMP") == 0)
			val |= WPA_CIPHER_CCMP;
		else if (cmd_strcmp(start, "GCMP") == 0)
			val |= WPA_CIPHER_GCMP;
		else if (cmd_strcmp(start, "TKIP") == 0)
			val |= WPA_CIPHER_TKIP;
		else if (cmd_strcmp(start, "WEP104") == 0)
			val |= WPA_CIPHER_WEP104;
		else if (cmd_strcmp(start, "WEP40") == 0)
			val |= WPA_CIPHER_WEP40;
		else if (cmd_strcmp(start, "NONE") == 0)
			val |= WPA_CIPHER_NONE;
		else if (cmd_strcmp(start, "GTK_NOT_USED") == 0)
			val |= WPA_CIPHER_GTK_NOT_USED;
		else {
			cmd_free(buf);
			return -1;
		}

		if (last)
			break;
		start = end + 1;
	}
	cmd_free(buf);

	return val;
}

int wpas_parse_proto(const char *value)
{
	int val = 0, last, errors = 0;
	char *start, *end, *buf;

	buf = cmd_strdup(value);
	if (buf == NULL)
		return -1;
	start = buf;

	while (*start != '\0') {
		while (*start == ' ' || *start == '\t')
			start++;
		if (*start == '\0')
			break;
		end = start;
		while (*end != ' ' && *end != '\t' && *end != '\0')
			end++;
		last = *end == '\0';
		*end = '\0';
		if (cmd_strcmp(start, "WPA") == 0)
			val |= WPA_PROTO_WPA;
		else if (cmd_strcmp(start, "RSN") == 0 ||
			 cmd_strcmp(start, "WPA2") == 0)
			val |= WPA_PROTO_RSN;
		else if (cmd_strcmp(start, "OSEN") == 0)
			val |= WPA_PROTO_OSEN;
		else {
			CMD_DBG("Invalid proto '%s'\n", start);
			errors++;
		}

		if (last)
			break;
		start = end + 1;
	}
	cmd_free(buf);

	if (val == 0) {
		CMD_DBG("No proto values configured\n");
		errors++;
	}

	CMD_DBG("proto: 0x%x\n", val);
	return errors ? -1 : val;
}

int wpas_parse_auth_alg(const char *value)
{
	int val = 0, last, errors = 0;
	char *start, *end, *buf;

	buf = cmd_strdup(value);
	if (buf == NULL)
		return -1;
	start = buf;

	while (*start != '\0') {
		while (*start == ' ' || *start == '\t')
			start++;
		if (*start == '\0')
			break;
		end = start;
		while (*end != ' ' && *end != '\t' && *end != '\0')
			end++;
		last = *end == '\0';
		*end = '\0';
		if (cmd_strcmp(start, "OPEN") == 0)
			val |= WPA_AUTH_ALG_OPEN;
		else if (cmd_strcmp(start, "SHARED") == 0)
			val |= WPA_AUTH_ALG_SHARED;
		else if (cmd_strcmp(start, "LEAP") == 0)
			val |= WPA_AUTH_ALG_LEAP;
		else {
			CMD_DBG("Invalid auth_alg '%s'\n", start);
			errors++;
		}

		if (last)
			break;
		start = end + 1;
	}
	cmd_free(buf);

	if (val == 0) {
		CMD_DBG("No auth_alg values configured\n");
		errors++;
	}

	CMD_DBG("auth_alg: 0x%x\n", val);
	return errors ? -1 : val;
}

/* @return
 *   -2: CMD_STATUS_INVALID_ARG
 *   -1: CMD_STATUS_FAIL
 *    0: CMD_STATUS_OK
 */
static int cmd_wpas_set_network(char *cmd)
{
	int tmp;
	char *name, *value;
	struct wpa_ctrl_req_network req;

	/* cmd: "<network id> <variable name> <value>" */
	name = cmd_strchr(cmd, ' ');
	if (name == NULL)
		return -2;
	*name++ = '\0';

	value = cmd_strchr(name, ' ');
	if (value == NULL)
		return -2;
	*value++ = '\0';

	if (wpas_parse_int(cmd, 0, INT32_MAX, &tmp) != 0)
		return -2;

	cmd_memset(&req, 0, sizeof(req));
	req.id = tmp;
	req.field = WPA_CTRL_NETWORK_FIELD_NUM;

	if (cmd_strcmp(name, "ssid") == 0) {
		req.field = WPA_CTRL_NETWORK_FIELD_SSID;
		cmd_strlcpy((char *)req.u.ssid, value, sizeof(req.u.ssid));
	} else if (cmd_strcmp(name, "psk") == 0) {
		req.field = WPA_CTRL_NETWORK_FIELD_PSK;
		cmd_strlcpy((char *)req.u.psk, value, sizeof(req.u.psk));
	} else if (cmd_strcmp(name, "wep_key0") == 0) {
		req.field = WPA_CTRL_NETWORK_FIELD_WEP_KEY0;
		cmd_strlcpy((char *)req.u.wep_key, value, sizeof(req.u.wep_key));
	} else if (cmd_strcmp(name, "wep_key1") == 0) {
		req.field = WPA_CTRL_NETWORK_FIELD_WEP_KEY1;
		cmd_strlcpy((char *)req.u.wep_key, value, sizeof(req.u.wep_key));
	} else if (cmd_strcmp(name, "wep_key2") == 0) {
		req.field = WPA_CTRL_NETWORK_FIELD_WEP_KEY2;
		cmd_strlcpy((char *)req.u.wep_key, value, sizeof(req.u.wep_key));
	} else if (cmd_strcmp(name, "wep_key3") == 0) {
		req.field = WPA_CTRL_NETWORK_FIELD_WEP_KEY3;
		cmd_strlcpy((char *)req.u.wep_key, value, sizeof(req.u.wep_key));
	} else if (cmd_strcmp(name, "wep_tx_keyidx") == 0) {
		if (wpas_parse_int(value, 0, 3, &tmp) == 0) {
			req.field = WPA_CTRL_NETWORK_FIELD_WEP_KEY_INDEX;
			req.u.wep_tx_keyidx = tmp;
		}
	} else if (cmd_strcmp(name, "key_mgmt") == 0) {
		tmp = waps_parse_key_mgmt(value);
		if (tmp > 0) {
			req.field = WPA_CTRL_NETWORK_FIELD_KEY_MGMT;
			req.u.key_mgmt = tmp;
		}
	} else if (cmd_strcmp(name, "pairwise") == 0) {
		tmp = wpas_parse_cipher(value);
		if (tmp > 0) {
			req.field = WPA_CTRL_NETWORK_FIELD_PAIRWISE_CIPHER;
			req.u.pairwise_cipher = tmp;
		}
	} else if (cmd_strcmp(name, "group") == 0) {
		tmp = wpas_parse_cipher(value);
		if (tmp > 0) {
			req.field = WPA_CTRL_NETWORK_FIELD_GROUP_CIPHER;
			req.u.group_cipher = tmp;
		}
	} else if (cmd_strcmp(name, "proto") == 0) {
		tmp = wpas_parse_proto(value);
		if (tmp > 0) {
			req.field = WPA_CTRL_NETWORK_FIELD_PROTO;
			req.u.proto = tmp;
		}
	} else if (cmd_strcmp(name, "auth_alg") == 0) {
		tmp = wpas_parse_auth_alg(value);
		if (tmp > 0) {
			req.field = WPA_CTRL_NETWORK_FIELD_AUTH_ALG;
			req.u.auth_alg = tmp;
		}
	} else if (cmd_strcmp(name, "wpa_ptk_rekey") == 0) { // in seconds
		if (wpas_parse_int(value, 0, INT32_MAX, &tmp) == 0) {
			req.field = WPA_CTRL_NETWORK_FIELD_WPA_PTK_REKEY;
			req.u.wpa_ptk_rekey = tmp;
		}
	} else if (cmd_strcmp(name, "scan_ssid") == 0) { // 0, 1
		if (wpas_parse_int(value, 0, 1, &tmp) == 0) {
			req.field = WPA_CTRL_NETWORK_FIELD_SCAN_SSID;
			req.u.scan_ssid = tmp;
		}
	}

	if (req.field < WPA_CTRL_NETWORK_FIELD_NUM) {
		return wlan_ctrl_request(WPA_CTRL_CMD_SET_NETWORK, &req);
	}

	CMD_ERR("invalid %s: '%s'\n", name, value);
	return -2;
}

/* @return
 *   -2: CMD_STATUS_INVALID_ARG
 *   -1: CMD_STATUS_FAIL
 *    0: CMD_STATUS_OK
 */
static int cmd_wpas_get_network(char *cmd)
{
	int id;
	char *name;
	struct wpa_ctrl_req_network req;

	/* cmd: "<network id> <variable name>" */
	name = cmd_strchr(cmd, ' ');
	if (name == NULL)
		return -2;
	*name++ = '\0';

	if (wpas_parse_int(cmd, 0, INT32_MAX, &id) != 0)
		return -2;

	cmd_memset(&req, 0, sizeof(req));
	req.id = id;

	if (cmd_strcmp(name, "ssid") == 0) {
		req.field = WPA_CTRL_NETWORK_FIELD_SSID;
	} else if (cmd_strcmp(name, "psk") == 0) {
		req.field = WPA_CTRL_NETWORK_FIELD_PSK;
	} else {
		CMD_ERR("invalid argument: '%s'\n", name);
		return -2;
	}

	if (wlan_ctrl_request(WPA_CTRL_CMD_GET_NETWORK, &req) < 0) {
		CMD_ERR("get network failed.\n");
		return -1;
	}

	if (req.field == WPA_CTRL_NETWORK_FIELD_SSID) {
		CMD_LOG(1, "ssid: %s\n", req.u.ssid);
	} else if (req.field == WPA_CTRL_NETWORK_FIELD_PSK) {
		CMD_LOG(1, "psk: %s\n", req.u.psk);
	}

	return 0;
}

static void wpas_print_scan_results(struct wpa_ctrl_req_scan_result *req)
{
	uint8_t i, len;

	for (i = 0; i < req->count; ++i) {
		len = req->entry[i].ssid_len;
		if (len > sizeof(req->entry[i].ssid) - 1)
			len = sizeof(req->entry[i].ssid) - 1;
		req->entry[i].ssid[len] = '\0';
		CMD_LOG(1, "%02d    %02x:%02x:%02x:%02x:%02x:%02x    %-32.32s    "
			 "%d    %d    flags: %08x    wpa_key_mgmt: %08x    "
			 "wpa_cipher: %08x    wpa2_key_mgmt: %08x    wpa2_cipher: %08x\n",
			 i + 1, (req->entry[i].bssid)[0], (req->entry[i].bssid)[1],
			 (req->entry[i].bssid)[2], (req->entry[i].bssid)[3],
			 (req->entry[i].bssid)[4], (req->entry[i].bssid)[5],
			 req->entry[i].ssid, req->entry[i].freq, req->entry[i].level,
			 req->entry[i].wpa_flags, req->entry[i].wpa_key_mgmt_flags,
			 req->entry[i].wpa_cipher_flags, req->entry[i].wpa2_key_mgmt_flags,
			 req->entry[i].wpa2_cipher_flags);
	}
}

enum cmd_status cmd_wpas_exec(char *cmd)
{
	int tmp;
	int ret = -1;

	if (cmd_strcmp(cmd, "scan") == 0) {
		struct wpa_ctrl_req_scan req;
		cmd_memset(&req, 0, sizeof(req));
		ret = wlan_ctrl_request(WPA_CTRL_CMD_SCAN, &req);
	} else if (cmd_strncmp(cmd, "scan_results ", 13) == 0) {
		if (wpas_parse_int(cmd + 13, 1, 30, &tmp) == 0) {
			struct wpa_ctrl_req_scan_result req;
			cmd_memset(&req, 0, sizeof(req));
			req.size = tmp;
			req.entry = cmd_malloc(tmp * sizeof(struct wpa_ctrl_req_scan_entry));
			ret = wlan_ctrl_request(WPA_CTRL_CMD_SCAN_RESULTS, &req);
			if (ret == 0) {
				wpas_print_scan_results(&req);
			}
			cmd_free(req.entry);
		}
	} else if (cmd_strncmp(cmd, "scan_interval ", 14) == 0) {  // in seconds
		if (wpas_parse_int(cmd + 14, 0, INT32_MAX, &tmp) == 0) {
			ret = wlan_ctrl_request(WPA_CTRL_CMD_SCAN_INTERVAL, (void *)tmp);
		}
	} else if (cmd_strcmp(cmd, "reassociate") == 0) {
		ret = wlan_ctrl_request(WPA_CTRL_CMD_REASSOCIATE, NULL);
	} else if (cmd_strcmp(cmd, "reattach") == 0) {
		ret = wlan_ctrl_request(WPA_CTRL_CMD_REATTACH, NULL);
	} else if (cmd_strcmp(cmd, "reconnect") == 0) {
		ret = wlan_ctrl_request(WPA_CTRL_CMD_RECONNECT, NULL);
	} else if (cmd_strcmp(cmd, "terminate") == 0) {
		ret = wlan_ctrl_request(WPA_CTRL_CMD_TERMINATE, NULL);
	} else if (cmd_strcmp(cmd, "disconnect") == 0) {
		ret = wlan_ctrl_request(WPA_CTRL_CMD_DISCONNECT, NULL);
	} else if (cmd_strncmp(cmd, "enable_network ", 15) == 0) {  // 0
		if (wpas_parse_int(cmd + 15, 0, 0, &tmp) == 0) {
			ret = wlan_ctrl_request(WPA_CTRL_CMD_ENABLE_NETWORK, (void *)tmp);
		}
	} else if (cmd_strncmp(cmd, "disable_network ", 16) == 0) {  // 0
		if (wpas_parse_int(cmd + 16, 0, 0, &tmp) == 0) {
			ret = wlan_ctrl_request(WPA_CTRL_CMD_DISABLE_NETWORK, (void *)tmp);
		}
	} else if (cmd_strncmp(cmd, "sta_autoconnect ", 16) == 0) { // 0 or 1
		if (wpas_parse_int(cmd + 16, 0, 1, &tmp) == 0) {
			ret = wlan_ctrl_request(WPA_CTRL_CMD_STA_AUTOCONNECT, (void *)tmp);
		}
	} else if (cmd_strncmp(cmd, "bss_expire_age ", 15) == 0) {  // in seconds
		if (wpas_parse_int(cmd + 15, 0, INT32_MAX, &tmp) == 0) {
			ret = wlan_ctrl_request(WPA_CTRL_CMD_BSS_EXPIRE_AGE, (void *)tmp);
		}
	} else if (cmd_strncmp(cmd, "bss_expire_count ", 17) == 0) { // int
		if (wpas_parse_int(cmd + 17, 0, INT32_MAX, &tmp) == 0) {
			ret = wlan_ctrl_request(WPA_CTRL_CMD_BSS_EXPIRE_COUNT, (void *)tmp);
		}
	} else if (cmd_strncmp(cmd, "bss_flush ", 10) == 0) { // int
		if (wpas_parse_int(cmd + 10, 0, INT32_MAX, &tmp) == 0) {
			ret = wlan_ctrl_request(WPA_CTRL_CMD_BSS_FLUSH, (void *)tmp);
		}
	} else if (cmd_strncmp(cmd, "set_network ", 12) == 0) {
		ret = cmd_wpas_set_network(cmd + 12);
	} else if (cmd_strncmp(cmd, "get_network ", 12) == 0) {
		ret = cmd_wpas_get_network(cmd + 12);
	} else if (cmd_strcmp(cmd, "wps_pbc") == 0) {
		ret = wlan_ctrl_request(WPA_CTRL_CMD_WPS_PBC, NULL);
	} else if (cmd_strcmp(cmd, "wps_get_pin") == 0) {
		struct wpa_ctrl_req_wps wps;
		ret = wlan_ctrl_request(WPA_CTRL_CMD_WPS_GET_PIN, &wps);
		CMD_SYSLOG("wps get pin: %s\n", wps.pin);
	} else if (cmd_strncmp(cmd, "wps_set_pin ", 12) == 0) {
		if (cmd_strlen(cmd + 12) == 8) {
			struct wpa_ctrl_req_wps wps;
			cmd_memcpy(wps.pin, cmd + 12, 8);
			wps.pin[8] = '\0';
			ret = wlan_ctrl_request(WPA_CTRL_CMD_WPS_SET_PIN, &wps);
		}
	} else {
		ret = -2;
	}

	if (ret == 0) {
		return CMD_STATUS_OK;
	} else if (ret == -2) {
		return CMD_STATUS_INVALID_ARG;
	} else {
		CMD_ERR("execute wpas command '%s' failed\n", cmd);
		return CMD_STATUS_FAIL;
	}
}

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE)) */
