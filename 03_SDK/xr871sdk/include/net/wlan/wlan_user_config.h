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

#ifndef _NET_WLAN_WLAN_USER_CONFIG_H_
#define _NET_WLAN_WLAN_USER_CONFIG_H_

#if (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE))

#include <stdint.h>
#include "lwip/netif.h"
#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"
#include "net/wlan/wlan_defs.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Wlan user config type definition
 */
typedef enum wlan_user_change {
    WLAN_USER_CHANGE_PM_DTIM        = 0,
    WLAN_USER_CHANGE_PS_MODE        = 1,
    WLAN_USER_CHANGE_AMPDU_TXNUM    = 2,
    WLAN_USER_CHANGE_TX_RETRY_CNT   = 3,
} wlan_user_change_t;

/**
 * @brief Wlan user config info definition
 */
typedef struct wlan_user_config_info {
	wlan_user_change_t field;

	union {
		/* pm dtim period param */
		int pm_dtim;	/* 1 ~ 10, base on beacon_period */

		/* ps mode param */
		int ps_mode;

		/* ampdu param */
		int ampdu_txnum;

		/* max retry count */
		int tx_retry_cnt;
	} u;
} wlan_user_config_info_t;

/**
 * @brief Wlan send user config definition
 */
typedef struct wlan_user_config {
	void *ifp;
	uint8_t *buf;
	int len;
} wlan_user_config_t;

int wlan_user_config_pm_dtim(struct netif *nif, int dtim);
int wlan_user_config_ps_mode(struct netif *nif, int mode);
int wlan_user_config_ampdu_txnum(struct netif *nif, int num);
int wlan_user_config_tx_retry(struct netif *nif, int retry_cnt);

#ifdef __cplusplus
}
#endif

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE)) */

#endif /* _NET_WLAN_WLAN_USER_CONFIG_H_ */
