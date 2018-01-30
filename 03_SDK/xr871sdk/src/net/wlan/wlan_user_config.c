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
#include "net/wlan/wlan_user_config.h"
#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"

int wlan_user_config_pm_dtim(struct netif *nif, int dtim)
{
	wlan_user_config_info_t info;
	wlan_user_config_t config;

	wlan_memset(&info, 0, sizeof(wlan_user_config_info_t));
	info.field = WLAN_USER_CHANGE_PM_DTIM;
	info.u.pm_dtim = dtim;
	config.ifp = nif->state;
	config.buf = (uint8_t *)(&info);
	config.len = sizeof(wlan_user_config_info_t);
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_USER_CONFIG, &config);
}

int wlan_user_config_ps_mode(struct netif *nif, int mode)
{
	wlan_user_config_info_t info;
	wlan_user_config_t config;

	wlan_memset(&info, 0, sizeof(wlan_user_config_info_t));
	info.field = WLAN_USER_CHANGE_PS_MODE;
	info.u.ps_mode = mode;
	config.ifp = nif->state;
	config.buf = (uint8_t *)(&info);
	config.len = sizeof(wlan_user_config_info_t);
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_USER_CONFIG, &config);
}

int wlan_user_config_ampdu_txnum(struct netif *nif, int num)
{
	wlan_user_config_info_t info;
	wlan_user_config_t config;

	wlan_memset(&info, 0, sizeof(wlan_user_config_info_t));
	info.field = WLAN_USER_CHANGE_AMPDU_TXNUM;
	info.u.ampdu_txnum = num;
	config.ifp = nif->state;
	config.buf = (uint8_t *)(&info);
	config.len = sizeof(wlan_user_config_info_t);
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_USER_CONFIG, &config);
}

int wlan_user_config_tx_retry(struct netif *nif, int retry_cnt)
{
	wlan_user_config_info_t info;
	wlan_user_config_t config;

	wlan_memset(&info, 0, sizeof(wlan_user_config_info_t));
	info.field = WLAN_USER_CHANGE_TX_RETRY_CNT;
	info.u.tx_retry_cnt = retry_cnt;
	config.ifp = nif->state;
	config.buf = (uint8_t *)(&info);
	config.len = sizeof(wlan_user_config_info_t);
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_USER_CONFIG, &config);
}

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE)) */
