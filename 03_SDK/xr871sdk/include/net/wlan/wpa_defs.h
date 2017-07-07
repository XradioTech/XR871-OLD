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

#ifndef _NET_WLAN_WPA_DEFS_H_
#define _NET_WLAN_WPA_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WPA_BIT
#define WPA_BIT(x) (1 << (x))
#endif

#define WPA_CIPHER_NONE WPA_BIT(0)
#define WPA_CIPHER_WEP40 WPA_BIT(1)
#define WPA_CIPHER_WEP104 WPA_BIT(2)
#define WPA_CIPHER_TKIP WPA_BIT(3)
#define WPA_CIPHER_CCMP WPA_BIT(4)
#define WPA_CIPHER_AES_128_CMAC WPA_BIT(5)
#define WPA_CIPHER_GCMP WPA_BIT(6)
#define WPA_CIPHER_SMS4 WPA_BIT(7)
#define WPA_CIPHER_GCMP_256 WPA_BIT(8)
#define WPA_CIPHER_CCMP_256 WPA_BIT(9)
#define WPA_CIPHER_BIP_GMAC_128 WPA_BIT(11)
#define WPA_CIPHER_BIP_GMAC_256 WPA_BIT(12)
#define WPA_CIPHER_BIP_CMAC_256 WPA_BIT(13)
#define WPA_CIPHER_GTK_NOT_USED WPA_BIT(14)
#define WPA_CIPHER_MASK 0x7fffUL

#define WPA_KEY_MGMT_IEEE8021X WPA_BIT(0)
#define WPA_KEY_MGMT_PSK WPA_BIT(1)
#define WPA_KEY_MGMT_NONE WPA_BIT(2)
#define WPA_KEY_MGMT_IEEE8021X_NO_WPA WPA_BIT(3)
#define WPA_KEY_MGMT_WPA_NONE WPA_BIT(4)
#define WPA_KEY_MGMT_FT_IEEE8021X WPA_BIT(5)
#define WPA_KEY_MGMT_FT_PSK WPA_BIT(6)
#define WPA_KEY_MGMT_IEEE8021X_SHA256 WPA_BIT(7)
#define WPA_KEY_MGMT_PSK_SHA256 WPA_BIT(8)
#define WPA_KEY_MGMT_WPS WPA_BIT(9)
#define WPA_KEY_MGMT_SAE WPA_BIT(10)
#define WPA_KEY_MGMT_FT_SAE WPA_BIT(11)
#define WPA_KEY_MGMT_WAPI_PSK WPA_BIT(12)
#define WPA_KEY_MGMT_WAPI_CERT WPA_BIT(13)
#define WPA_KEY_MGMT_CCKM WPA_BIT(14)
#define WPA_KEY_MGMT_OSEN WPA_BIT(15)
#define WPA_KEY_MGMT_IEEE8021X_SUITE_B WPA_BIT(16)
#define WPA_KEY_MGMT_IEEE8021X_SUITE_B_192 WPA_BIT(17)
#define WPA_KEY_MGMT_MASK 0x3ffffUL

#define WPA_PROTO_WPA WPA_BIT(0)
#define WPA_PROTO_RSN WPA_BIT(1)
#define WPA_PROTO_WAPI WPA_BIT(2)
#define WPA_PROTO_OSEN WPA_BIT(3)
#define WPA_PROTO_MASK 0xfUL

#define WPA_AUTH_ALG_OPEN WPA_BIT(0)
#define WPA_AUTH_ALG_SHARED WPA_BIT(1)
#define WPA_AUTH_ALG_LEAP WPA_BIT(2)
#define WPA_AUTH_ALG_FT WPA_BIT(3)
#define WPA_AUTH_ALG_SAE WPA_BIT(4)
#define WPA_AUTH_ALG_MASK 0x1fUL

/* for scan results */
#define WPA_FLAGS_WPA		WPA_BIT(0)
#define WPA_FLAGS_WPA2		WPA_BIT(1)
#define WPA_FLAGS_WEP		WPA_BIT(2)
#define WPA_FLAGS_WPS_PBC	WPA_BIT(3)
#define WPA_FLAGS_WPS_AUTH	WPA_BIT(4)
#define WPA_FLAGS_WPS_PIN	WPA_BIT(5)
#define WPA_FLAGS_WPS		WPA_BIT(6)
#define WPA_FLAGS_IBSS		WPA_BIT(7)
#define WPA_FLAGS_ESS		WPA_BIT(8)

/**
 * enum hostapd_hw_mode - Hardware mode
 */
enum hostapd_hw_mode {
	HOSTAPD_MODE_IEEE80211B,
	HOSTAPD_MODE_IEEE80211G,
	HOSTAPD_MODE_IEEE80211A,
	HOSTAPD_MODE_IEEE80211AD,
	NUM_HOSTAPD_MODES
};

#ifdef __cplusplus
}
#endif

#endif /* _NET_WLAN_WPA_DEFS_H_ */
