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

#ifndef _NET_WLAN_WPA_CTRL_REQ_H_
#define _NET_WLAN_WPA_CTRL_REQ_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum wpa_ctrl_cmd {
	WPA_CTRL_CMD_SCAN,
	WPA_CTRL_CMD_SCAN_RESULTS,
	WPA_CTRL_CMD_SCAN_INTERVAL,

	WPA_CTRL_CMD_REASSOCIATE,
	WPA_CTRL_CMD_REATTACH,
	WPA_CTRL_CMD_RECONNECT,
	WPA_CTRL_CMD_TERMINATE,
	WPA_CTRL_CMD_DISCONNECT,

	WPA_CTRL_CMD_ENABLE_NETWORK,
	WPA_CTRL_CMD_DISABLE_NETWORK,

	WPA_CTRL_CMD_SET_NETWORK,
	WPA_CTRL_CMD_GET_NETWORK,
	WPA_CTRL_CMD_STA_AUTOCONNECT,

	WPA_CTRL_CMD_BSS_EXPIRE_AGE,
	WPA_CTRL_CMD_BSS_EXPIRE_COUNT,
	WPA_CTRL_CMD_BSS_FLUSH,

	WPA_CTRL_CMD_WPS_PBC,
	WPA_CTRL_CMD_WPS_GET_PIN,
	WPA_CTRL_CMD_WPS_SET_PIN,

	/* hostapd */
	WPA_CTRL_CMD_ENABLE,
	WPA_CTRL_CMD_RELOAD,
	WPA_CTRL_CMD_DISABLE,
	WPA_CTRL_CMD_STATUS,
	WPA_CTRL_CMD_GET_CONFIG,
	WPA_CTRL_CMD_STA_NUM,
	WPA_CTRL_CMD_STA,
	WPA_CTRL_CMD_SET,
};

#define WPA_CTRL_REQ_SCAN_MAX_SCAN_ID	2

struct wpa_ctrl_req_scan {
	uint8_t	scan_only;
	uint8_t	scan_passive;	/* passive scan or not */

	/* Scan SSID of configured network with Probe Requests
	 */
	uint8_t	scan_id[WPA_CTRL_REQ_SCAN_MAX_SCAN_ID];
	uint8_t	scan_id_count;	/* valid count of scan_id[] */
};

struct wpa_ctrl_req_scan_entry {
	uint8_t bssid[6];
	uint8_t ssid[33];
	uint8_t ssid_len;
	int32_t freq;
	int32_t level;
	uint32_t wpa_flags;
	uint32_t wpa_cipher_flags;
	uint32_t wpa_key_mgmt_flags;
	uint32_t wpa2_cipher_flags;
	uint32_t wpa2_key_mgmt_flags;
};

struct wpa_ctrl_req_scan_result {
	uint8_t size;
	uint8_t count;
	struct wpa_ctrl_req_scan_entry *entry;
};

enum WPA_CTRL_NETWORK_FIELD {
	WPA_CTRL_NETWORK_FIELD_SSID = 0,
	WPA_CTRL_NETWORK_FIELD_PSK,
	WPA_CTRL_NETWORK_FIELD_WEP_KEY0,
	WPA_CTRL_NETWORK_FIELD_WEP_KEY1,
	WPA_CTRL_NETWORK_FIELD_WEP_KEY2,
	WPA_CTRL_NETWORK_FIELD_WEP_KEY3,
	WPA_CTRL_NETWORK_FIELD_WEP_KEY_INDEX,
	WPA_CTRL_NETWORK_FIELD_KEY_MGMT,
	WPA_CTRL_NETWORK_FIELD_PAIRWISE_CIPHER,
	WPA_CTRL_NETWORK_FIELD_GROUP_CIPHER,
	WPA_CTRL_NETWORK_FIELD_PROTO,
	WPA_CTRL_NETWORK_FIELD_AUTH_ALG,
	WPA_CTRL_NETWORK_FIELD_WPA_PTK_REKEY,
	WPA_CTRL_NETWORK_FIELD_SCAN_SSID,

	WPA_CTRL_NETWORK_FIELD_NUM,
};

struct wpa_ctrl_req_network {
	uint8_t	id;		/* network id */
	uint8_t	field;		/* one of enum WPA_CTRL_NETWORK_FIELD */
	uint8_t	pad[2];

	union {
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

		/**
		 * wep_key: WEP key in one of the optional formats:
		 *   - an ASCII string with double quotation, length is [5, 13] + 2
		 *   - a hex string (two characters per octet of PSK), length is [10, 26]
		 */
		uint8_t wep_key[27];

		/**
		 * wep_tx_keyidx - Default key index for TX frames using WEP
		 */
		int32_t wep_tx_keyidx;

		/**
		 * key_mgmt - Bitfield of allowed key management protocols
		 *
		 * WPA_KEY_MGMT_*
		 */
		uint32_t key_mgmt;

		/**
		 * pairwise_cipher - Bitfield of allowed pairwise ciphers
		 *
		 * WPA_CIPHER_*
		 */
		uint32_t pairwise_cipher;

		/**
		 * group_cipher - Bitfield of allowed group ciphers
		 *
		 * WPA_CIPHER_*
		 */
		uint32_t group_cipher;

		/**
		 * proto - Bitfield of allowed protocols
		 *
		 * WPA_PROTO_*
		 */
		uint32_t proto;

		/**
		 * auth_alg -  Bitfield of allowed authentication algorithms
		 *
		 * WPA_AUTH_ALG_*
		 */
		uint32_t auth_alg;

		/**
		 * wpa_ptk_rekey - Maximum lifetime for PTK in seconds
		 *
		 * This value can be used to enforce rekeying of PTK to
		 * mitigate some attacks against TKIP deficiencies.
		 */
		int32_t wpa_ptk_rekey;

		/**
		 * scan_ssid - Scan this SSID with Probe Requests
		 *
		 * scan_ssid can be used to scan for APs using hidden SSIDs.
		 */
		int32_t scan_ssid;
	} u;
};

struct wpa_ctrl_req_wps {
	uint8_t	pin[9];
};

/*
 * hostapd
 */
struct wpa_ctrl_req_status {
	uint8_t bssid[6];
	uint8_t channel;
	uint8_t ssid_len;
	uint8_t ssid[32];
	int32_t num_sta;
};

struct wpa_ctrl_req_get_config {
	int32_t wpa;
	int32_t auth_algs;
	int32_t wpa_key_mgmt;
	int32_t wpa_group;
	int32_t wpa_pairwise;
	int32_t rsn_pairwise;
	int32_t wpa_group_rekey;
	int32_t wpa_strict_rekey;
	int32_t wpa_gmk_rekey;
	int32_t wpa_ptk_rekey;
};

struct wpa_ctrl_req_sta_entry {
	uint8_t addr[6];
};

struct wpa_ctrl_req_sta_list {
	uint32_t size;
	struct wpa_ctrl_req_sta_entry *entry;
};

enum WPA_CTRL_SET_FIELD {
	WPA_CTRL_SET_FIELD_SSID = 0,
	WPA_CTRL_SET_FIELD_PASSPHRASE,
	WPA_CTRL_SET_FIELD_WPA,
	WPA_CTRL_SET_FIELD_AUTH_ALGS,
	WPA_CTRL_SET_FIELD_WPA_PAIRWISE,
	WPA_CTRL_SET_FIELD_RSN_PAIRWISE,
	WPA_CTRL_SET_FIELD_KEY_MGMT,
	WPA_CTRL_SET_FIELD_GROUP_REKEY,
	WPA_CTRL_SET_FIELD_STRICT_REKEY,
	WPA_CTRL_SET_FIELD_GMK_REKEY,
	WPA_CTRL_SET_FIELD_PTK_REKEY,
	WPA_CTRL_SET_FIELD_HW_MODE,
	WPA_CTRL_SET_FIELD_IEEE80211N,
	WPA_CTRL_SET_FIELD_CHANNEL,
	WPA_CTRL_SET_FIELD_BEACON_INT,
	WPA_CTRL_SET_FIELD_DTIM,
	WPA_CTRL_SET_FIELD_MAX_NUM_STA,

	WPA_CTRL_SET_FIELD_NUM,
};

struct wpa_ctrl_req_set {
	uint8_t	field;		/* one of enum WPA_CTRL_NETWORK_FIELD */
	uint8_t	pad[3];

	union {
		/**
		 * ssid: network name in one of the optional formats:
		 *   - an ASCII string with double quotation, length is [0, 32] + 2
		 *   - a hex string (two characters per octet of SSID), length is [0, 64]
		 *   - a printf-escaped ASCII string P"<escaped string>"
		 */
		uint8_t ssid[65];

		/**
		 * passphrase - WPA ASCII passphrase
		 *
		 * If this is set, psk will be generated using the SSID and passphrase
		 * configured for the network. ASCII passphrase must be between 8 and
		 * 63 characters (inclusive).
		 */
		uint8_t passphrase[64];

		/**
		 * wpa - Bitfield of WPA_PROTO_WPA, WPA_PROTO_RSN
		 *
		 * WPA_PROTO_*
		 */
		uint32_t wpa;

		/**
		 * auth_algs - Bitfield of allowed authentication algorithms
		 *
		 * WPA_AUTH_ALG_*
		 */
		uint32_t auth_algs;

		/**
		 * wpa_pairwise - Bitfield of allowed WPA pairwise ciphers
		 *
		 * WPA_CIPHER_*
		 */
		uint32_t wpa_pairwise;

		/**
		 * wpa_key_mgmt - Bitfield of allowed key management protocols
		 *
		 * WPA_KEY_MGMT_*
		 */
		uint32_t wpa_key_mgmt;

		/**
		 * rsn_pairwise - Bitfield of allowed RSN pairwise ciphers
		 *
		 * WPA_CIPHER_*
		 */
		uint32_t rsn_pairwise;

		/**
		 * wpa_group_rekey - Maximum lifetime for GTK in seconds
		 */
		int32_t wpa_group_rekey;

		/**
		 * wpa_strict_rekey - Rekey GTK when any STA that possesses the
		 *		      current GTK is leaving the BSS
		 */
		int32_t wpa_strict_rekey;

		/**
		 * wpa_gmk_rekey - Maximum lifetime for GMK in seconds
		 */
		int32_t wpa_gmk_rekey;

		/**
		 * wpa_ptk_rekey - Maximum lifetime for PTK in seconds
		 */
		int32_t wpa_ptk_rekey;

		/**
		 * hw_mode - Hardware mode
		 *
		 * HOSTAPD_MODE_*
		 */
		int32_t hw_mode;

		/**
		 * ieee80211n
		 */
		int32_t ieee80211n;

		/**
		 * channel
		 */
		uint8_t channel;

		/**
		 * beacon_int
		 *
		 * MIB defines range as 1..65535, but very small values
		 * cause problems with the current implementation.
		 * Since it is unlikely that this small numbers are
		 * useful in real life scenarios, do not allow beacon
		 * period to be set below 15 TU.
		 */
		uint16_t beacon_int;

		/**
		 * dtim_period
		 */
		int32_t dtim_period;

		/**
		 * max_num_sta - maximum number of STAs in station table
		 */
		int32_t max_num_sta;
	} u;
};

#ifdef __cplusplus
}
#endif

#endif /* _NET_WLAN_WPA_CTRL_REQ_H_ */
