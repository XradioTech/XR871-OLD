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
#include "lwip/netif.h"

#include "common/sysinfo/sysinfo.h"
#include "sysinfo_debug.h"


static const uint8_t g_wlan_mac_addr[] = { 0x00, 0x80, 0xE1, 0x29, 0xE8, 0xD1 };

static struct sysinfo g_sysinfo;

void sysinfo_init(void)
{
	g_sysinfo.wlan_mode = WLAN_MODE_STA;
	memcpy(g_sysinfo.mac_addr, g_wlan_mac_addr, sizeof(g_sysinfo.mac_addr));
}

void sysinfo_set_wlan_mode(uint8_t mode)
{
	g_sysinfo.wlan_mode = mode;
}

uint8_t sysinfo_get_wlan_mode(void)
{
	if (g_sysinfo.wlan_mode < WLAN_MODE_NUM) {
		return (enum wlan_mode)g_sysinfo.wlan_mode;
	} else {
		return WLAN_MODE_INVALID;
	}
}

int sysinfo_set_mac_addr(uint8_t *mac_addr, int mac_len)
{
	if (mac_len == SYSINFO_MAC_ADDR_LEN) {
		memcpy(g_sysinfo.mac_addr, mac_addr, SYSINFO_MAC_ADDR_LEN);
		return 0;
	} else {
		return -1;
	}
}

int sysinfo_get_mac_addr(uint8_t *mac_addr, int mac_len)
{
	if (mac_len >= SYSINFO_MAC_ADDR_LEN) {
		memcpy(mac_addr, g_sysinfo.mac_addr, SYSINFO_MAC_ADDR_LEN);
		return SYSINFO_MAC_ADDR_LEN;
	} else {
		return 0;
	}
}

void sysinfo_save(void)
{
}

struct sysinfo *sysinfo_get(void)
{
	return &g_sysinfo;
}
