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

#ifndef _NET_WLAN_WLAN_DEFS_H_
#define _NET_WLAN_WLAN_DEFS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WPA_BIT
#define WPA_BIT(x)				(1 << (x))
#endif

#define WPA_CIPHER_NONE				WPA_BIT(0)
#define WPA_CIPHER_WEP40			WPA_BIT(1)
#define WPA_CIPHER_WEP104			WPA_BIT(2)
#define WPA_CIPHER_TKIP				WPA_BIT(3)
#define WPA_CIPHER_CCMP				WPA_BIT(4)
#define WPA_CIPHER_AES_128_CMAC			WPA_BIT(5)
#define WPA_CIPHER_GCMP				WPA_BIT(6)
#define WPA_CIPHER_SMS4				WPA_BIT(7)
#define WPA_CIPHER_GCMP_256			WPA_BIT(8)
#define WPA_CIPHER_CCMP_256			WPA_BIT(9)
#define WPA_CIPHER_BIP_GMAC_128			WPA_BIT(11)
#define WPA_CIPHER_BIP_GMAC_256			WPA_BIT(12)
#define WPA_CIPHER_BIP_CMAC_256			WPA_BIT(13)
#define WPA_CIPHER_GTK_NOT_USED			WPA_BIT(14)
#define WPA_CIPHER_MASK				0x7fffUL

#define WPA_KEY_MGMT_IEEE8021X			WPA_BIT(0)
#define WPA_KEY_MGMT_PSK			WPA_BIT(1)
#define WPA_KEY_MGMT_NONE			WPA_BIT(2)
#define WPA_KEY_MGMT_IEEE8021X_NO_WPA		WPA_BIT(3)
#define WPA_KEY_MGMT_WPA_NONE			WPA_BIT(4)
#define WPA_KEY_MGMT_FT_IEEE8021X		WPA_BIT(5)
#define WPA_KEY_MGMT_FT_PSK			WPA_BIT(6)
#define WPA_KEY_MGMT_IEEE8021X_SHA256		WPA_BIT(7)
#define WPA_KEY_MGMT_PSK_SHA256			WPA_BIT(8)
#define WPA_KEY_MGMT_WPS			WPA_BIT(9)
#define WPA_KEY_MGMT_SAE			WPA_BIT(10)
#define WPA_KEY_MGMT_FT_SAE			WPA_BIT(11)
#define WPA_KEY_MGMT_WAPI_PSK			WPA_BIT(12)
#define WPA_KEY_MGMT_WAPI_CERT			WPA_BIT(13)
#define WPA_KEY_MGMT_CCKM			WPA_BIT(14)
#define WPA_KEY_MGMT_OSEN			WPA_BIT(15)
#define WPA_KEY_MGMT_IEEE8021X_SUITE_B		WPA_BIT(16)
#define WPA_KEY_MGMT_IEEE8021X_SUITE_B_192	WPA_BIT(17)
#define WPA_KEY_MGMT_MASK			0x3ffffUL

#define WPA_PROTO_WPA				WPA_BIT(0)
#define WPA_PROTO_RSN				WPA_BIT(1)
#define WPA_PROTO_WAPI				WPA_BIT(2)
#define WPA_PROTO_OSEN				WPA_BIT(3)
#define WPA_PROTO_MASK				0xfUL

#define WPA_AUTH_ALG_OPEN			WPA_BIT(0)
#define WPA_AUTH_ALG_SHARED			WPA_BIT(1)
#define WPA_AUTH_ALG_LEAP			WPA_BIT(2)
#define WPA_AUTH_ALG_FT				WPA_BIT(3)
#define WPA_AUTH_ALG_SAE			WPA_BIT(4)
#define WPA_AUTH_ALG_MASK			0x1fUL

/* for scan results */
#define WPA_FLAGS_WPA				WPA_BIT(0)
#define WPA_FLAGS_WPA2				WPA_BIT(1)
#define WPA_FLAGS_WEP				WPA_BIT(2)
#define WPA_FLAGS_WPS_PBC			WPA_BIT(3)
#define WPA_FLAGS_WPS_AUTH			WPA_BIT(4)
#define WPA_FLAGS_WPS_PIN			WPA_BIT(5)
#define WPA_FLAGS_WPS				WPA_BIT(6)
#define WPA_FLAGS_IBSS				WPA_BIT(7)
#define WPA_FLAGS_ESS				WPA_BIT(8)

/* wlan event */
typedef enum wlan_event {
	WLAN_EVENT_CONNECTED,
	WLAN_EVENT_DISCONNECTED,
	WLAN_EVENT_SCAN_SUCCESS,
	WLAN_EVENT_SCAN_FAILED,
	WLAN_EVENT_4WAY_HANDSHAKE_FAILED,
	WLAN_EVENT_CONNECT_FAILED,

	WLAN_EVENT_SMART_CONFIG_RESULT,
	WLAN_EVENT_AIRKISS_RESULT,
	WLAN_EVENT_DEV_HANG,
} wlan_event_t;

/* STA */
typedef enum wlan_sta_field {
	WLAN_STA_FIELD_SSID = 0,
	WLAN_STA_FIELD_PSK,
	WLAN_STA_FIELD_WEP_KEY0,
	WLAN_STA_FIELD_WEP_KEY1,
	WLAN_STA_FIELD_WEP_KEY2,
	WLAN_STA_FIELD_WEP_KEY3,
	WLAN_STA_FIELD_WEP_KEY_INDEX,
	WLAN_STA_FIELD_KEY_MGMT,
	WLAN_STA_FIELD_PAIRWISE_CIPHER,
	WLAN_STA_FIELD_GROUP_CIPHER,
	WLAN_STA_FIELD_PROTO,
	WLAN_STA_FIELD_AUTH_ALG,
	WLAN_STA_FIELD_WPA_PTK_REKEY,
	WLAN_STA_FIELD_SCAN_SSID,

	WLAN_STA_FIELD_NUM,
} wlan_sta_field_t;

typedef struct wlan_sta_config {
	wlan_sta_field_t field;

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
		int wep_tx_keyidx;

		/**
		 * key_mgmt - Bitfield of allowed key management protocols
		 *
		 * WPA_KEY_MGMT_*
		 */
		int key_mgmt;

		/**
		 * pairwise_cipher - Bitfield of allowed pairwise ciphers
		 *
		 * WPA_CIPHER_*
		 */
		int pairwise_cipher;

		/**
		 * group_cipher - Bitfield of allowed group ciphers
		 *
		 * WPA_CIPHER_*
		 */
		int group_cipher;

		/**
		 * proto - Bitfield of allowed protocols
		 *
		 * WPA_PROTO_*
		 */
		int proto;

		/**
		 * auth_alg -  Bitfield of allowed authentication algorithms
		 *
		 * WPA_AUTH_ALG_*
		 */
		int auth_alg;

		/**
		 * wpa_ptk_rekey - Maximum lifetime for PTK in seconds
		 *
		 * This value can be used to enforce rekeying of PTK to
		 * mitigate some attacks against TKIP deficiencies.
		 */
		int wpa_ptk_rekey;

		/**
		 * scan_ssid - Scan this SSID with Probe Requests
		 *
		 * scan_ssid can be used to scan for APs using hidden SSIDs.
		 */
		int scan_ssid;
	} u;
} wlan_sta_config_t;

typedef struct wlan_sta_scan_ap {
	uint8_t	bssid[6];
	uint8_t	ssid[65];
	int	freq;
	int	level;
	int	wpa_flags;
	int	wpa_cipher;
	int	wpa_key_mgmt;
	int	wpa2_cipher;
	int	wpa2_key_mgmt;
}wlan_sta_scan_ap_t;

typedef struct wlan_sta_scan_results {
	wlan_sta_scan_ap_t *ap;
	int size;
	int num;
} wlan_sta_scan_results_t;

typedef struct wlan_sta_wps_pin {
	uint8_t pin[9];
} wlan_sta_wps_pin_t;

/* softAP */
typedef enum wlan_ap_field {
	WLAN_AP_FIELD_SSID = 0,
	WLAN_AP_FIELD_PSK,
	WLAN_AP_FIELD_KEY_MGMT,
	WLAN_AP_FIELD_WPA_CIPHER,
	WLAN_AP_FIELD_RSN_CIPHER,
	WLAN_AP_FIELD_PROTO,
	WLAN_AP_FIELD_AUTH_ALG,
	WLAN_AP_FIELD_GROUP_REKEY,
	WLAN_AP_FIELD_STRICT_REKEY,
	WLAN_AP_FIELD_GMK_REKEY,
	WLAN_AP_FIELD_PTK_REKEY,
	WLAN_AP_FIELD_HW_MODE,
	WLAN_AP_FIELD_IEEE80211N,
	WLAN_AP_FIELD_CHANNEL,
	WLAN_AP_FIELD_BEACON_INT,
	WLAN_AP_FIELD_DTIM,
	WLAN_AP_FIELD_MAX_NUM_STA,

	WLAN_AP_FIELD_NUM,
} wlan_ap_field_t;

typedef enum wlan_ap_hw_mode {
	WLAN_AP_HW_MODE_IEEE80211B = 0,
	WLAN_AP_HW_MODE_IEEE80211G,
	WLAN_AP_HW_MODE_IEEE80211A,
	WLAN_AP_HW_MODE_IEEE80211AD,

	WLAN_AP_HW_MODE_NUM,
} wlan_ap_hw_mode_t;

typedef struct wlan_ap_config {
	wlan_ap_field_t field;

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
		 * key_mgmt - Bitfield of allowed key management protocols
		 *
		 * WPA_KEY_MGMT_*
		 */
		int key_mgmt;

		/**
		 * wpa_cipher - Bitfield of allowed WPA pairwise ciphers
		 *
		 * WPA_CIPHER_*
		 */
		int wpa_cipher;

		/**
		 * rsn_cipher - Bitfield of allowed RSN pairwise ciphers
		 *
		 * WPA_CIPHER_*
		 */
		int rsn_cipher;

		/**
		 * proto - Bitfield of allowed protocols
		 *
		 * WPA_PROTO_*
		 */
		int proto;

		/**
		 * auth_alg -  Bitfield of allowed authentication algorithms
		 *
		 * WPA_AUTH_ALG_*
		 */
		int auth_alg;

		/**
		 * group_rekey - Maximum lifetime for GTK in seconds
		 */
		int group_rekey;

		/**
		 * strict_rekey - Rekey GTK when any STA that possesses the
		 *		  current GTK is leaving the BSS
		 */
		int strict_rekey;

		/**
		 * gmk_rekey - Maximum lifetime for GMK in seconds
		 */
		int gmk_rekey;

		/**
		 * ptk_rekey - Maximum lifetime for PTK in seconds
		 */
		int ptk_rekey;

		/**
		 * hw_mode - Hardware mode
		 */
		wlan_ap_hw_mode_t hw_mode;

		/**
		 * ieee80211n
		 */
		int ieee80211n;

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
		 * dtim
		 */
		int dtim;

		/**
		 * max_num_sta - maximum number of STAs in station table
		 */
		int max_num_sta;
	} u;
} wlan_ap_config_t;

typedef struct wlan_ap_sta {
	uint8_t addr[6];
} wlan_ap_sta_t;

typedef struct wlan_ap_stas {
	wlan_ap_sta_t *sta;
	int size;
	int num;
} wlan_ap_stas_t;

/* smart config */
typedef struct wlan_smart_config_result {
	uint8_t valid;
	uint8_t ssid[65];
	uint8_t psk[66];
	uint8_t random_num;
} wlan_smart_config_result_t;

#ifdef __cplusplus
}
#endif

#endif /* _NET_WLAN_WLAN_DEFS_H_ */
