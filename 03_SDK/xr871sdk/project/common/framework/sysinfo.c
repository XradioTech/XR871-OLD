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

#include "image/fdcm.h"
#include "image/image.h"
#if PRJCONF_NET_EN
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "driver/chip/hal_crypto.h"
#include "efpg/efpg.h"
#endif /* PRJCONF_NET_EN */
#include "sysinfo.h"
#include "sysinfo_debug.h"

static struct sysinfo g_sysinfo;
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
static fdcm_handle_t *g_fdcm_hdl;
#endif

#if PRJCONF_NET_EN

static uint8_t m_sysinfo_mac_addr[] = { 0x00, 0x80, 0xE1, 0x29, 0xE8, 0xD1 };

static void sysinfo_gen_mac_random(uint8_t mac_addr[6])
{
#if (PRJCONF_CE_EN && PRJCONF_PRNG_INIT_SEED)
	HAL_PRNG_Generate(mac_addr, 6);
#else
	int i;
	for (i = 0; i < 6; ++i) {
		mac_addr[i] = (uint8_t)OS_Rand32();
	}
#endif
	mac_addr[0] &= 0xFC;
}

static void sysinfo_gen_mac_by_chipid(uint8_t mac_addr[6])
{
	int i;
	uint8_t chipid[16];

	efpg_read(EFPG_FIELD_CHIPID, chipid);

	for (i = 0; i < 2; ++i) {
		mac_addr[i] = chipid[i] ^ chipid[i + 6] ^ chipid[i + 12];
	}
	for (i = 2; i < 6; ++i) {
		mac_addr[i] = chipid[i] ^ chipid[i + 6] ^ chipid[i + 10];
	}
	mac_addr[0] &= 0xFC;
}

static void sysinfo_init_mac_addr(void)
{
	int i;

	SYSINFO_DBG("mac addr source: %d\n", PRJCONF_MAC_ADDR_SOURCE);

	switch (PRJCONF_MAC_ADDR_SOURCE) {
	case SYSINFO_MAC_ADDR_CODE:
		memcpy(g_sysinfo.mac_addr, m_sysinfo_mac_addr, SYSINFO_MAC_ADDR_LEN);
		return;
	case SYSINFO_MAC_ADDR_EFUSE:
		if (efpg_read(EFPG_FIELD_MAC, g_sysinfo.mac_addr) != 0) {
			SYSINFO_WRN("read mac addr from eFuse fail\n");
			goto random_mac_addr;
		}
		return;
	case SYSINFO_MAC_ADDR_CHIPID:
		sysinfo_gen_mac_by_chipid(g_sysinfo.mac_addr);
		for (i = 0; i < sizeof(g_sysinfo.mac_addr); ++i) {
			if (g_sysinfo.mac_addr[i] != 0)
				return;
		}
		goto random_mac_addr;
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
	case SYSINFO_MAC_ADDR_FLASH: {
		struct sysinfo *info = malloc(SYSINFO_SIZE);
		if (info == NULL) {
			SYSINFO_ERR("malloc fail\n");
			goto random_mac_addr;
		}
		if (fdcm_read(g_fdcm_hdl, info, SYSINFO_SIZE) != SYSINFO_SIZE) {
			SYSINFO_WRN("read mac addr from flash fail\n");
			free(info);
			goto random_mac_addr;
		}
		memcpy(g_sysinfo.mac_addr, info->mac_addr, SYSINFO_MAC_ADDR_LEN);
		free(info);
		return;
	}
#endif
	default:
		SYSINFO_ERR("invalid mac addr source\n");
		goto random_mac_addr;
	}

random_mac_addr:
	SYSINFO_DBG("random mac addr\n");
	sysinfo_gen_mac_random(g_sysinfo.mac_addr);
}
#endif /* PRJCONF_NET_EN */

static void sysinfo_init_value(void)
{
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
	if (sysinfo_load() != 0) {
		sysinfo_default();
		return;
	}
#if PRJCONF_NET_EN
	if (PRJCONF_MAC_ADDR_SOURCE != SYSINFO_MAC_ADDR_FLASH) {
		sysinfo_init_mac_addr();
	}
#endif
#else
	sysinfo_default();
#endif
}

/**
 * @brief Initialize the sysinfo module
 * @return 0 on success, -1 on failure
 */
int sysinfo_init(void)
{
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
#if PRJCONF_SYSINFO_CHECK_OVERLAP
	uint32_t image_size = image_get_size();
	if (image_size == 0) {
		SYSINFO_ERR("get image size failed\n");
		return -1;
	}
	if (image_size > PRJCONF_SYSINFO_ADDR) {
		SYSINFO_ERR("image is too big: %#x, please make it smaller than %#x\n",
		            image_size, PRJCONF_SYSINFO_ADDR);
		return -1;
	}
#endif
	g_fdcm_hdl = fdcm_open(PRJCONF_SYSINFO_FLASH, PRJCONF_SYSINFO_ADDR, PRJCONF_SYSINFO_SIZE);
	if (g_fdcm_hdl == NULL) {
		SYSINFO_ERR("fdcm open failed, hdl %p\n", g_fdcm_hdl);
		return -1;
	}
#endif /* PRJCONF_SYSINFO_SAVE_TO_FLASH */
	sysinfo_init_value();
	return 0;
}

/**
 * @brief DeInitialize the sysinfo module
 * @return None
 */
void sysinfo_deinit(void)
{
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
	fdcm_close(g_fdcm_hdl);
#endif
}

/**
 * @brief Set default value to sysinfo
 * @return 0 on success, -1 on failure
 */
int sysinfo_default(void)
{
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
	if (g_fdcm_hdl == NULL) {
		SYSINFO_ERR("uninitialized, hdl %p\n", g_fdcm_hdl);
		return -1;
	}
#endif

	memset(&g_sysinfo, 0, SYSINFO_SIZE);

#if PRJCONF_NET_EN
	/* MAC address */
	sysinfo_init_mac_addr();

	/* wlan mode */
	g_sysinfo.wlan_mode = WLAN_MODE_STA;

	/* netif STA */
	g_sysinfo.sta_use_dhcp = 1;

	/* netif AP */
	IP4_ADDR(&g_sysinfo.netif_ap_param.ip_addr, 192, 168, 51, 1);
	IP4_ADDR(&g_sysinfo.netif_ap_param.net_mask, 255, 255, 255, 0);
	IP4_ADDR(&g_sysinfo.netif_ap_param.gateway, 192, 168, 51, 1);
#endif

	SYSINFO_DBG("set default value\n");
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
	sysinfo_save();
#endif
	return 0;
}

#if PRJCONF_SYSINFO_SAVE_TO_FLASH
/**
 * @brief Save sysinfo to flash
 * @return 0 on success, -1 on failure
 */
int sysinfo_save(void)
{
	if (g_fdcm_hdl == NULL) {
		SYSINFO_ERR("uninitialized, hdl %p\n", g_fdcm_hdl);
		return -1;
	}

	if (fdcm_write(g_fdcm_hdl, &g_sysinfo, SYSINFO_SIZE) != SYSINFO_SIZE) {
		SYSINFO_ERR("fdcm write failed\n");
		return -1;
	}

	SYSINFO_DBG("save sysinfo to flash\n");

	return 0;
}

/**
 * @brief Load sysinfo from flash
 * @return 0 on success, -1 on failure
 */
int sysinfo_load(void)
{
	if (g_fdcm_hdl == NULL) {
		SYSINFO_ERR("uninitialized, hdl %p\n", g_fdcm_hdl);
		return -1;
	}

	if (fdcm_read(g_fdcm_hdl, &g_sysinfo, SYSINFO_SIZE) != SYSINFO_SIZE) {
		SYSINFO_WRN("fdcm read failed\n");
		return -1;
	}

	SYSINFO_DBG("load sysinfo from flash\n");

	return 0;
}
#endif /* PRJCONF_SYSINFO_SAVE_TO_FLASH */

/**
 * @brief Get the pointer of the sysinfo
 * @return Pointer to the sysinfo, NULL on failure
 */
struct sysinfo *sysinfo_get(void)
{
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
	if (g_fdcm_hdl == NULL) {
		SYSINFO_ERR("uninitialized, hdl %p\n", g_fdcm_hdl);
		return NULL;
	}
#endif

	return &g_sysinfo;
}
