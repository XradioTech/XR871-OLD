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

#include <string.h>
#include <stdlib.h>

#include "efpg/efpg.h"
#include "sys/fdcm.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "sysinfo.h"
#include "sysinfo_debug.h"

static struct sysinfo g_sysinfo;
static fdcm_handle_t *g_fdcm_hdl;

static uint8_t g_sysinfo_mac_addr[] = { 0x00, 0x80, 0xE1, 0x29, 0xE8, 0xD1 };

static inline void sysinfo_default_value(void)
{
	SYSINFO_DBG("%s(), %d, set default values\n", __func__, __LINE__);

	if (efpg_read(EFPG_AREA_MAC, g_sysinfo.mac_addr) != 0) {
		SYSINFO_DBG("%s(), %d, set default mac addr\n", __func__, __LINE__);
		memcpy(g_sysinfo.mac_addr, g_sysinfo_mac_addr, SYSINFO_MAC_ADDR_LEN);
	}

	g_sysinfo.wlan_mode = WLAN_MODE_STA;

	g_sysinfo.sta_netif_param.use_dhcp = 1;
	g_sysinfo.ap_netif_param.use_dhcp = 0;

	IP4_ADDR(&g_sysinfo.ap_netif_param.ip_addr, 192, 168, 51, 1);
	IP4_ADDR(&g_sysinfo.ap_netif_param.net_mask, 255, 255, 255, 0);
	IP4_ADDR(&g_sysinfo.ap_netif_param.gateway, 192, 168, 51, 1);
}

static inline void sysinfo_init_value(void)
{
	if (fdcm_read(g_fdcm_hdl, &g_sysinfo, SYSINFO_SIZE) != SYSINFO_SIZE)
		sysinfo_default_value();
}

int sysinfo_init(void)
{
	g_fdcm_hdl = fdcm_open(PRJCONF_SYSINFO_FLASH, PRJCONF_SYSINFO_ADDR, PRJCONF_SYSINFO_SIZE);
	if (g_fdcm_hdl == NULL) {
		SYSINFO_ERR("fdcm open failed, hdl %p\n", g_fdcm_hdl);
		return -1;
	}

	sysinfo_init_value();

	return 0;
}

void sysinfo_deinit(void)
{
	fdcm_close(g_fdcm_hdl);
}

int sysinfo_save(enum sysinfo_type type)
{
	struct sysinfo *info;

	if (g_fdcm_hdl == NULL) {
		SYSINFO_ERR("uninitialized, hdl %p\n", g_fdcm_hdl);
		return -1;
	}

	SYSINFO_DBG("%s(), %d, type %d\n", __func__, __LINE__, type);

	if (type == SYSINFO_WHOLE) {
		if (fdcm_write(g_fdcm_hdl, &g_sysinfo, SYSINFO_SIZE) != SYSINFO_SIZE) {
			SYSINFO_ERR("fdcm write failed, type %d\n", type);
			return -1;
		}

		return 0;
	}

	info = malloc(SYSINFO_SIZE);
	if (info == NULL) {
		SYSINFO_ERR("malloc failed, info %p\n", info);
		return -1;
	}

	if (fdcm_read(g_fdcm_hdl, info, SYSINFO_SIZE) != SYSINFO_SIZE) {
		SYSINFO_ERR("fdcm read failed\n");
		free(info);
		return -1;
	}

	switch (type) {
	case SYSINFO_MAC_ADDR:
		memcpy(info->mac_addr, g_sysinfo.mac_addr, SYSINFO_MAC_ADDR_LEN);
		break;
	case SYSINFO_WLAN_MODE:
		info->wlan_mode = g_sysinfo.wlan_mode;
		break;
	case SYSINFO_WLAN_PARAM:
		memcpy(&info->wlan_param, &g_sysinfo.wlan_param, sizeof(g_sysinfo.wlan_param));
		break;
	case SYSINFO_STA_NETIF_PARAM:
		memcpy(&info->sta_netif_param, &g_sysinfo.sta_netif_param, sizeof(g_sysinfo.sta_netif_param));
		break;
	case SYSINFO_AP_NETIF_PARAM:
		memcpy(&info->ap_netif_param, &g_sysinfo.ap_netif_param, sizeof(g_sysinfo.ap_netif_param));
		break;
	default:
		SYSINFO_ERR("invalid type %d\n", type);
		free(info);
		return -1;
	}

	if (fdcm_write(g_fdcm_hdl, info, SYSINFO_SIZE) != SYSINFO_SIZE) {
		SYSINFO_ERR("fdcm write failed, type %d\n", type);
		free(info);
		return -1;
	}

	free(info);
	return 0;
}

int sysinfo_load(enum sysinfo_type type)
{
	struct sysinfo *info;

	if (g_fdcm_hdl == NULL) {
		SYSINFO_ERR("uninitialized, hdl %p\n", g_fdcm_hdl);
		return -1;
	}

	SYSINFO_DBG("%s(), %d, type %d\n", __func__, __LINE__, type);

	if (type == SYSINFO_WHOLE) {
		if (fdcm_read(g_fdcm_hdl, &g_sysinfo, SYSINFO_SIZE) != SYSINFO_SIZE) {
			SYSINFO_ERR("fdcm read failed, type %d\n", type);
			return -1;
		}

		return 0;
	}

	info = malloc(SYSINFO_SIZE);
	if (info == NULL) {
		SYSINFO_ERR("malloc failed, info %p\n", info);
		return -1;
	}

	if (fdcm_read(g_fdcm_hdl, info, SYSINFO_SIZE) != SYSINFO_SIZE) {
		SYSINFO_ERR("fdcm read failed\n");
		free(info);
		return -1;
	}

	switch (type) {
	case SYSINFO_MAC_ADDR:
		memcpy(g_sysinfo.mac_addr, info->mac_addr, SYSINFO_MAC_ADDR_LEN);
		break;
	case SYSINFO_WLAN_MODE:
		g_sysinfo.wlan_mode = info->wlan_mode;
		break;
	case SYSINFO_WLAN_PARAM:
		memcpy(&g_sysinfo.wlan_param, &info->wlan_param, sizeof(g_sysinfo.wlan_param));
		break;
	case SYSINFO_STA_NETIF_PARAM:
		memcpy(&g_sysinfo.sta_netif_param, &info->sta_netif_param, sizeof(g_sysinfo.sta_netif_param));
		break;
	case SYSINFO_AP_NETIF_PARAM:
		memcpy(&g_sysinfo.ap_netif_param, &info->ap_netif_param, sizeof(g_sysinfo.ap_netif_param));
		break;
	default:
		SYSINFO_ERR("invalid type %d\n", type);
		free(info);
		return -1;
	}

	free(info);
	return 0;
}

int sysinfo_set(enum sysinfo_type type, void *info)
{
	if (info == NULL) {
		SYSINFO_ERR("invalid param, info %p\n", info);
		return -1;
	}

	SYSINFO_DBG("%s(), %d, type %d\n", __func__, __LINE__, type);

	switch (type) {
	case SYSINFO_WHOLE:
		memcpy(&g_sysinfo, info, SYSINFO_SIZE);
		break;
	case SYSINFO_MAC_ADDR:
		memcpy(g_sysinfo.mac_addr, info, SYSINFO_MAC_ADDR_LEN);
		break;
	case SYSINFO_WLAN_MODE:
		g_sysinfo.wlan_mode = *(uint8_t *)info;
		break;
	case SYSINFO_WLAN_PARAM:
		memcpy(&g_sysinfo.wlan_param, info, sizeof(g_sysinfo.wlan_param));
		break;
	case SYSINFO_STA_NETIF_PARAM:
		memcpy(&g_sysinfo.sta_netif_param, info, sizeof(g_sysinfo.sta_netif_param));
		break;
	case SYSINFO_AP_NETIF_PARAM:
		memcpy(&g_sysinfo.ap_netif_param, info, sizeof(g_sysinfo.ap_netif_param));
		break;
	default:
		SYSINFO_ERR("invalid type %d\n", type);
		return -1;
	}

	return 0;
}

int sysinfo_get(enum sysinfo_type type, void *info)
{
	if (info == NULL) {
		SYSINFO_ERR("invalid param, info %p\n", info);
		return -1;
	}

	SYSINFO_DBG("%s(), %d, type %d\n", __func__, __LINE__, type);

	switch (type) {
	case SYSINFO_WHOLE:
		memcpy(info, &g_sysinfo, SYSINFO_SIZE);
		break;
	case SYSINFO_MAC_ADDR:
		memcpy(info, g_sysinfo.mac_addr, SYSINFO_MAC_ADDR_LEN);
		break;
	case SYSINFO_WLAN_MODE:
		*(uint8_t *)info = g_sysinfo.wlan_mode;
		break;
	case SYSINFO_WLAN_PARAM:
		memcpy(info, &g_sysinfo.wlan_param, sizeof(g_sysinfo.wlan_param));
		break;
	case SYSINFO_STA_NETIF_PARAM:
		memcpy(info, &g_sysinfo.sta_netif_param, sizeof(g_sysinfo.sta_netif_param));
		break;
	case SYSINFO_AP_NETIF_PARAM:
		memcpy(info, &g_sysinfo.ap_netif_param, sizeof(g_sysinfo.ap_netif_param));
		break;
	default:
		SYSINFO_ERR("invalid type %d\n", type);
		return -1;
	}

	return 0;
}

