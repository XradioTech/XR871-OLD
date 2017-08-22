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

#ifndef _NET_WLAN_WLAN_H_
#define _NET_WLAN_WLAN_H_

#if (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE))

#include <stdint.h>
#include "lwip/netif.h"
#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"
#include "net/wlan/wlan_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* wlan sys */
int wlan_sys_init(enum wlan_mode mode, ducc_cb_func cb);
int wlan_sys_deinit(void);

int wlan_start(struct netif *nif);
int wlan_stop(void); /* Note: make sure wlan is disconnect before calling wlan_stop() */

static __inline int wlan_attach(void)
{
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_ATTACH, NULL);
}

static __inline int wlan_detach(void)
{
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_DETACH, NULL);
}

static __inline struct netif *wlan_if_create(enum wlan_mode mode)
{
	return ethernetif_create(mode);
}

static __inline int wlan_if_delete(struct netif *nif)
{
	ethernetif_delete(nif);
	return 0;
}

static __inline enum wlan_mode wlan_if_get_mode(struct netif *nif)
{
	return ethernetif_get_mode(nif);
}

int wlan_set_mac_addr(uint8_t *mac_addr, int mac_len);
int wlan_set_ip_addr(void *ifp, uint8_t *ip_addr, int ip_len);

/* STA */
int wlan_sta_set_ascii(char *ssid_ascii, char *psk_ascii);

int wlan_sta_set(uint8_t *ssid, uint8_t *psk);
int wlan_sta_set_config(wlan_sta_config_t *config);
int wlan_sta_get_config(wlan_sta_config_t *config);

int wlan_sta_enable(void);
int wlan_sta_disable(void);

int wlan_sta_scan_once(void);
int wlan_sta_scan_result(wlan_sta_scan_results_t *results);
int wlan_sta_bss_flush(int age);

int wlan_sta_connect(void);
int wlan_sta_disconnect(void);

int wlan_sta_state(wlan_sta_states_t *state);
int wlan_sta_ap_info(wlan_sta_ap_t *ap);

int wlan_sta_wps_pbc(void);
int wlan_sta_wps_pin_get(wlan_sta_wps_pin_t *wps);
int wlan_sta_wps_pin_set(wlan_sta_wps_pin_t *wps);

/* softAP */
int wlan_ap_set_ascii(char *ssid_ascii, char *psk_ascii);

int wlan_ap_set(uint8_t *ssid, uint8_t *psk);
int wlan_ap_set_config(wlan_ap_config_t *config);
int wlan_ap_get_config(wlan_ap_config_t *config);

int wlan_ap_enable(void);
int wlan_ap_reload(void);
int wlan_ap_disable(void);

int wlan_ap_sta_num(int *num);
int wlan_ap_sta_info(wlan_ap_stas_t *stas);

/* smart config */
int wlan_smart_config_start(struct netif *nif, uint32_t time_out_ms);
int wlan_smart_config_stop();
int wlan_smart_config_set_key(char *key);

/* airkiss */
int wlan_airkiss_start(struct netif *nif, uint32_t time_out_ms);
int wlan_airkiss_stop();
int wlan_airkiss_set_key(char *key);
int wlan_airkiss_ack_start(struct wlan_smart_config_result *result, struct netif *netif);

/*
 * The wechat_public_id and devic_id should be global variables.
 * In this mode, the driver will be send online data by cycle
 */
int wlan_airkiss_online_cycle_ack_start(char *app_id, char *drv_id, uint32_t period_ms);
void wlan_airkiss_online_cycle_ack_stop(void);

/*
 * The wechat_public_id and devic_id should be global variables.
 * In this mode,  the drivers will be listen to server's request and send ack for server, then the driver will
 * be send online data by cycle
 */
int wlan_airkiss_online_dialog_mode_start(char *app_id, char *drv_id, uint32_t period_ms);
void wlan_airkiss_online_dialog_mode_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE)) */

#endif /* _NET_WLAN_WLAN_H_ */
