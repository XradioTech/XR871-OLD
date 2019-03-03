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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "sys/interrupt.h"

#include "atcmd/at_command.h"

#include "common/cmd/cmd_util.h"
#include "common/cmd/cmd_wlan.h"
#include "net/wlan/wlan.h"
#include "common/cmd/cmd_ping.h"

#include "lwip/inet.h"
#include "common/framework/net_ctrl.h"

#include "driver/chip/hal_rtc.h"

#include "image/fdcm.h"

#include "serial.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "errno.h"

#include "common/framework/sys_ctrl/sys_ctrl.h"
#include "common/framework/sysinfo.h"
#include "driver/chip/hal_wdg.h"

#include "atcmd.h"

#define FUN_DEBUG_ON	1

#if FUN_DEBUG_ON == 1
#define FUN_DEBUG(fmt...) {\
	printf("file:%s line:%d ", __FILE__, __LINE__);\
	printf(fmt);\
	}
#else
#define FUN_DEBUG(fmt...)
#endif

#define MANUFACTURER	"XRADIO"
#define MODEL			"serial-to-wifi"
#define SERIAL			"01234567"
//#define MAC				{0x00, 0x11, 0x22, 0x33, 0x44, 0x55}

#define CONFIG_FDCM_FLASH	(0)
#define CONFIG_FDCM_ADDR	0x120000UL
#define CONFIG_FDCM_SIZE	0x10000UL

#define CONFIG_CONTAINNER_SIZE sizeof(config_containner_t)

#define MAX_SCAN_RESULTS	10
#define MAX_SOCKET_NUM	8
#define IP_ADDR_SIZE	15
#define SOCKET_CACHE_BUFFER_SIZE	1024

#define SERVER_THREAD_STACK_SIZE	(2 * 1024)

typedef struct {
	s32 cmd;
	AT_ERROR_CODE (*handler)(at_callback_para_t *para, at_callback_rsp_t *rsp);
} callback_handler_t;

typedef struct {
	u32 cnt;
	at_config_t cfg;
} config_containner_t;

typedef struct {
	char ip[IP_ADDR_SIZE+1];
	s16 port;
	s32 protocol;
	s32 fd;
	u32 flag;
} connect_t;

typedef struct {
	s32 count;
	connect_t connect[MAX_SOCKET_NUM];
} network_t;

typedef struct {
	u32 flag;
	s32 offset;
	s32 cnt;
	u8 buffer[SOCKET_CACHE_BUFFER_SIZE];
} socket_cache_t;

typedef struct {
	s16 port;
	s32 protocol;
} server_arg_t;

typedef struct {
	u32 flag; /* 0: disconnect    1: connect */
	s32 sock_fd;
	s32 conn_fd;
} server_net_t;

typedef struct {
	u32 flag;
	s16 port;
	s32 protocol;
	s32 conn_fd;
} server_ctrl_t;

static socket_cache_t socket_cache[MAX_SOCKET_NUM+1];

static OS_Thread_t g_server_thread;
static OS_Mutex_t g_server_mutex;
static server_arg_t g_server_arg;
static server_net_t g_server_net;
static server_ctrl_t g_server_ctrl;

static u32 g_server_enable = 0;
static OS_Semaphore_t g_server_sem;

static AT_ERROR_CODE callback(AT_CALLBACK_CMD cmd, at_callback_para_t *para, at_callback_rsp_t *rsp);

static AT_ERROR_CODE act(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE reset(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE mode(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE save(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE load(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE status(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE factory(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE peer(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE ping(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE sockon(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE sockw(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE sockq(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE sockr(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE sockc(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE sockd(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE wifi(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE reassociate(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE gpioc(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE gpior(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE gpiow(at_callback_para_t *para, at_callback_rsp_t *rsp);
static AT_ERROR_CODE scan(at_callback_para_t *para, at_callback_rsp_t *rsp);

static const callback_handler_t callback_tbl[] = {
	{ACC_ACT,				act},
	{ACC_RST,				reset},
	{ACC_MODE,				mode},
	{ACC_SAVE,				save},
	{ACC_LOAD,				load},
	{ACC_STATUS,			status},
	{ACC_FACTORY,			factory},
	{ACC_PEER,				peer},
	{ACC_PING,				ping},
	{ACC_SOCKON,			sockon},
	{ACC_SOCKW,				sockw},
	{ACC_SOCKQ,				sockq},
	{ACC_SOCKR,				sockr},
	{ACC_SOCKC,				sockc},
	{ACC_SOCKD,				sockd},
	{ACC_WIFI,				wifi},
	{ACC_REASSOCIATE,		reassociate},
	{ACC_GPIOC,				gpioc},
	{ACC_GPIOR,				gpior},
	{ACC_GPIOW,				gpiow},
	{ACC_SCAN,				scan},
};

static const u32 channel_freq_tbl[] = {
	2412,2417,2422,2427,2432,2437,2442,2447,2452,2457,2462,2467,2472
};

static const char *event[] = {
	"wlan connected",
	"wlan disconnected",
	"wlan scan success",
	"wlan scan failed",
	"wlan 4way handshake failed",
	"wlan connect failed",
	"network up",
	"network down",
};

static const fdcm_handle_t fdcm_hdl_tbl[] = {
    {CONFIG_FDCM_FLASH, CONFIG_FDCM_ADDR, CONFIG_FDCM_SIZE},
    {CONFIG_FDCM_FLASH, CONFIG_FDCM_ADDR+CONFIG_FDCM_SIZE, CONFIG_FDCM_SIZE}
};

/* factory default */
static const at_config_t default_cfg = {
	.blink_led = 0,
	.wind_off_low = 0x0,
	.wind_off_medium = 0x0,
	.wind_off_high = 0x0,
	.user_desc = "XRADIO-AP",
	.escape_seq = "at+s.",
	.localecho1 = 0,
	.console1_speed = 115200,
	.console1_hwfc = 0,
	.console1_enabled = 0,
	.sleep_enabled = 0,
	.standby_enabled = 0,
	.standby_time = 10,
	.wifi_tx_msdu_lifetime = 0,
	.wifi_rx_msdu_lifetime = 0,
	.wifi_operational_mode = 0x00000011,
	.wifi_beacon_wakeup = 1,
	.wifi_beacon_interval = 100,
	.wifi_listen_interval = 0,
	.wifi_rts_threshold = 3000,
	.wifi_ssid = "iot-ap",
	.wifi_ssid_len = 6,
	.wifi_channelnum = 6,
	.wifi_opr_rate_mask = 0xFFFFFFFF,
	.wifi_bas_rate_mask = 0x0000000F,
	.wifi_mode = 1,
	.wifi_auth_type = 0,
	.wifi_powersave = 1,
	.wifi_tx_power = 18,
	.wifi_rssi_thresh = -50,
	.wifi_rssi_hyst = 10,
	.wifi_ap_idle_timeout = 120,
	.wifi_beacon_loss_thresh = 10,
	.wifi_priv_mode = 2,
	/*.wifi_wep_keys[4][16],*/
	/*.wifi_wep_key_lens[4],*/
	.wifi_wep_default_key = 0,
	/*.wifi_wpa_psk_raw[32],*/
	/*.wifi_wpa_psk_text[64],*/
	.ip_use_dhcp = 1,
	.ip_use_httpd = 1,
	.ip_mtu = 1500,
	.ip_hostname = "xr-iot-dev",
	.ip_apdomainname = "xradio.com",
	.ip_ipaddr = {192, 168, 0, 123},
	.ip_netmask = {255, 255, 255, 0},
	.ip_gw = {192, 168, 0, 1},
	.ip_dns = {192, 168, 0, 1},
	.ip_http_get_recv_timeout = 1000,
	.ip_dhcp_timeout = 20,
	.ip_sockd_timeout = 250,
};

static network_t networks;

static __always_inline int server_is_isr_context(void)
{
	return __get_IPSR();
}

static void server_mutex_lock(void)
{
	if (server_is_isr_context() || !OS_ThreadIsSchedulerRunning()) {
		return;
	}

	if (OS_MutexIsValid(&g_server_mutex)) {
		OS_RecursiveMutexLock(&g_server_mutex, OS_WAIT_FOREVER);
	} else {
		OS_RecursiveMutexCreate(&g_server_mutex);
		OS_RecursiveMutexLock(&g_server_mutex, OS_WAIT_FOREVER);
	}
}

static void server_mutex_unlock(void)
{
	if (server_is_isr_context() || !OS_ThreadIsSchedulerRunning()) {
		return;
	}

	if (OS_MutexIsValid(&g_server_mutex)) {
		OS_RecursiveMutexUnlock(&g_server_mutex);
	}
}

s32 at_cmdline(char *buf, u32 size)
{
	u32 i;

	for (i = 0; i < size; i++) {
		if (buf[i] == AT_LF) {
			return i+1;
		}
		else if (buf[i] == AT_CR) {
			if (((i+1) < size) && (buf[i+1] == AT_LF)) {
				return i+2;
			}
			else {
				return i+1;
			}
		}
	}

	return -1;
}

static u8 queue_buf[1024];

void occur(uint32_t evt, uint32_t data, void *arg);

void at_cmd_init(void)
{
	at_callback_t at_cb;

	at_queue_init(queue_buf, sizeof(queue_buf), serial_read);

	at_cb.handle_cb = callback;
	at_cb.dump_cb = serial_write;

	at_init(&at_cb);

	observer_base *obs = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
	                                                  NET_CTRL_MSG_ALL,
	                                                  occur,
	                                                  NULL);
	sys_ctrl_attach(obs);
}

void at_cmd_exec(void)
{
	at_parse();
}

static AT_ERROR_CODE callback(AT_CALLBACK_CMD cmd, at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	s32 i;

	/* FUN_DEBUG("callback cmd = %d\n",cmd); */

	for (i = 0; i < TABLE_SIZE(callback_tbl); i++) {
		if (cmd == callback_tbl[i].cmd) {
			if (callback_tbl[i].handler != NULL) {
				return callback_tbl[i].handler(para, rsp);
			}
			else {
				/* FUN_DEBUG("callback cmd = %d is unimplimented!\n", cmd); */

				return AEC_UNDEFINED;
			}
		}
	}

	/* FUN_DEBUG("callback cmd = %d is unsupported!\n", cmd); */

	return AEC_UNSUPPORTED;
}

static AT_ERROR_CODE act(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	AT_ERROR_CODE aec = AEC_OK;
	uint8_t ap_ssid[32+1];
	uint8_t ap_psk[] = "12345678";

	switch (para->cfg->wifi_mode) {
	case 0: /* IDLE */

		net_switch_mode(WLAN_MODE_STA);
		wlan_sta_disable();

		break;

	case 1: /* STA */

		net_switch_mode(WLAN_MODE_STA);
		wlan_sta_set(para->cfg->wifi_ssid, para->cfg->wifi_ssid_len, (uint8_t *)para->cfg->wifi_wpa_psk_text);
		wlan_sta_enable();

		break;

	case 2: /* AP */

		net_switch_mode(WLAN_MODE_HOSTAP);
		wlan_ap_disable();
		snprintf((char *)ap_ssid, 32, "xr-ap-%02x%02x%02x", para->cfg->nv_wifi_macaddr[3], para->cfg->nv_wifi_macaddr[4], para->cfg->nv_wifi_macaddr[5]);
		wlan_ap_set(ap_ssid, strlen((char *)ap_ssid), ap_psk);
		wlan_ap_enable();

		break;
#if 0
	case 3: /* IBSS */

		break;
#endif
	default:

		aec = AEC_PARA_ERROR;

		break;
	}

	return aec;
}

static AT_ERROR_CODE reset(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	AT_ERROR_CODE aec = AEC_OK;

	HAL_WDG_Reboot();

	return aec;
}

static AT_ERROR_CODE mode(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	AT_ERROR_CODE res = AEC_OK;
	s32 id;
	u8 *buffer;
	s32 len;
	s32 timeout_ms=10;
	fd_set fdset_r,fdset_w;
	int fd;
	struct sockaddr_in address;
	socklen_t addr_len;
	char ip[IP_ADDR_SIZE+1];
	s16 port;
	s32 protocol;

	if (!g_server_enable) { /* as client */
		id = 0;
		fd = networks.connect[id].fd;
		protocol = networks.connect[id].protocol;
		port = networks.connect[id].port;
		strncpy(ip, networks.connect[id].ip, sizeof(ip));

		if (networks.count > 0) {
			if (networks.connect[id].flag) {
				int rc = -1;
				struct timeval tv;

				FD_ZERO(&fdset_w);
				FD_ZERO(&fdset_r);
				FD_SET(fd, &fdset_w);
				FD_SET(fd, &fdset_r);

				tv.tv_sec = timeout_ms / 1000;
				tv.tv_usec = (timeout_ms % 1000) * 1000;

				rc = select(fd + 1, &fdset_r, NULL, NULL, &tv);
				if (rc > 0) {
					if (FD_ISSET(fd, &fdset_r)) {
						buffer = socket_cache[id].buffer;
						len = SOCKET_CACHE_BUFFER_SIZE;

						if (protocol == 0) { /* TCP */
							rc = recv(fd, buffer, len, 0);
						}
						else if (protocol == 1) { /* UDP */
							address.sin_port = htons(port);
							address.sin_family = AF_INET;
							address.sin_addr.s_addr= inet_addr(ip);

							addr_len = sizeof(address);

							rc = recvfrom(fd, buffer, len, 0, (struct sockaddr *)&address, &addr_len);
						}

						if (rc > 0) {
							/* received normally */
							serial_write(buffer, rc);
						}
						else if (rc == 0) {
							/* has disconnected with server */
							res = AEC_DISCONNECT;
						}
						else {
							/* network error */
							res = AEC_NETWORK_ERROR;
						}
					}
				}
				else if (rc == 0) {
					/* timeouted and sent 0 bytes */
				}
				else {
					 /* network error */
					res = AEC_NETWORK_ERROR;
				}

				rc = select(fd + 1, NULL, &fdset_w, NULL, &tv);
				if (rc > 0) {
					if (FD_ISSET(fd, &fdset_w)) {
						buffer = para->u.mode.buf;
						len = para->u.mode.len;

						if (buffer != NULL && len >0) {
							if (protocol == 0) { /* TCP */
								rc = send(fd, buffer, len, 0);
							}
							else if (protocol == 1) { /* UDP */
								address.sin_port = htons(port);
								address.sin_family = AF_INET;
								address.sin_addr.s_addr= inet_addr(ip);

								addr_len = sizeof(address);
								rc = sendto(fd, buffer, len, 0, (struct sockaddr *)&address, sizeof(address));
							}

							if (rc > 0) {
								/* do nothing */
							}
							else if (rc == 0) {
								/* disconnected with server */
								res = AEC_DISCONNECT;
							}
							else {
								/* network error */
								res = AEC_NETWORK_ERROR;
							}
						}
					}
				}
				else if (rc == 0) {
					/* timeouted and sent 0 bytes */
				}
				else {
					 /* network error */
					res = AEC_NETWORK_ERROR;
				}
			}
			else {
				res = AEC_SWITCH_MODE;
			}
		}
		else {
			res = AEC_SWITCH_MODE;
		}
	}
	else { /* as server */
		if (!g_server_ctrl.flag) {
			g_server_ctrl.protocol = g_server_arg.protocol;

			server_mutex_lock();
			g_server_ctrl.conn_fd = g_server_net.conn_fd;
			g_server_ctrl.flag = g_server_net.flag;
			server_mutex_unlock();
		}

		if (g_server_ctrl.flag) {
			int rc = -1;
			struct timeval tv;

			port = g_server_ctrl.port;
			protocol = g_server_ctrl.protocol;
			fd = g_server_ctrl.conn_fd;

			FD_ZERO(&fdset_w);
			FD_ZERO(&fdset_r);
			FD_SET(fd, &fdset_w);
			FD_SET(fd, &fdset_r);

			tv.tv_sec = timeout_ms / 1000;
			tv.tv_usec = (timeout_ms % 1000) * 1000;

			rc = select(fd + 1, &fdset_r, NULL, NULL, &tv);
			if (rc > 0) {
				if (FD_ISSET(fd, &fdset_r)) {
					buffer = socket_cache[MAX_SOCKET_NUM].buffer;
					len = SOCKET_CACHE_BUFFER_SIZE;

					if (protocol == 0) { /* TCP */
						rc = recv(fd, buffer, len, 0);
					}
					else if (protocol == 1) { /* UDP */
						address.sin_port = htons(port);
						address.sin_family = AF_INET;
						address.sin_addr.s_addr= inet_addr(ip);

						addr_len = sizeof(address);

						rc = recvfrom(fd, buffer, len, 0, (struct sockaddr *)&address, &addr_len);
					}

					if (rc > 0) {
						/* received normally */
						serial_write(buffer, rc);
					}
					else if (rc == 0) {
						/* has disconnected with server */
						if (protocol == 0) { /* TCP */
							server_mutex_lock();
							g_server_net.conn_fd = -1;
							g_server_net.flag = 0;
							server_mutex_unlock();

							OS_SemaphoreRelease(&g_server_sem);

							g_server_ctrl.flag = 0;
						}

						res = AEC_DISCONNECT;
					}
					else {
						/* network error */
						res = AEC_NETWORK_ERROR;
					}
				}
			}
			else if (rc == 0) {
				/* timeouted and sent 0 bytes */
			}
			else {
				 /* network error */
				res = AEC_NETWORK_ERROR;
			}

			rc = select(fd + 1, NULL, &fdset_w, NULL, &tv);
			if (rc > 0) {
				if (FD_ISSET(fd, &fdset_w)) {
					buffer = para->u.mode.buf;
					len = para->u.mode.len;

					if (buffer != NULL && len >0) {
						if (protocol == 0) { /* TCP */
							rc = send(fd, buffer, len, 0);
						}
						else if (protocol == 1) { /* UDP */
							FUN_DEBUG("Unsupported!\n");
						}

						if (rc > 0) {
							/* do nothing */
						}
						else if (rc == 0) {
							/* disconnected with server */
							res = AEC_DISCONNECT;
						}
						else {
							/* network error */
							res = AEC_NETWORK_ERROR;
						}
					}
				}
			}
			else if (rc == 0) {
				/* timeouted and sent 0 bytes */
			}
			else {
				 /* network error */
				res = AEC_NETWORK_ERROR;
			}
		}
		else {
			res = AEC_DISCONNECT;
		}
	}

	return res;

}

static AT_ERROR_CODE save(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	config_containner_t *containner;
	s32 i;
	fdcm_handle_t *fdcm_hdl;
	u32 flag[2];
	s32 idx,cnt;

	containner = (config_containner_t *)malloc(2*sizeof(config_containner_t));

	if (containner == NULL) {
		FUN_DEBUG("malloc faild.\n");
		return AEC_NOT_ENOUGH_MEMORY;
	}

	for (i=0; i<TABLE_SIZE(fdcm_hdl_tbl); i++) {
		flag[i] = 1;

		fdcm_hdl = fdcm_open(fdcm_hdl_tbl[i].flash, fdcm_hdl_tbl[i].addr, fdcm_hdl_tbl[i].size);

		if (fdcm_hdl == NULL) {
			FUN_DEBUG("fdcm_open faild.\n");
			free(containner);
			return AEC_UNDEFINED;
		}

		if (fdcm_read(fdcm_hdl, &containner[i], CONFIG_CONTAINNER_SIZE) != CONFIG_CONTAINNER_SIZE) {
			flag[i] = 0;
		}

		fdcm_close(fdcm_hdl);
	}

	if(flag[0] && flag[1]) {
		if (containner[0].cnt > containner[1].cnt) {
			idx = 1;
			cnt = containner[0].cnt;
		}
		else {
			idx = 0;
			cnt = containner[1].cnt;
		}
	}
	else if(flag[0]) {
		idx = 1;
		cnt = containner[0].cnt;
	}
	else if(flag[1]) {
		idx = 0;
		cnt = containner[1].cnt;
	}
	else {
		idx = 0;
		cnt = 0;
	}

	fdcm_hdl = fdcm_open(fdcm_hdl_tbl[idx].flash, fdcm_hdl_tbl[idx].addr, fdcm_hdl_tbl[idx].size);

	if (fdcm_hdl == NULL) {
		FUN_DEBUG("fdcm_open faild.\n");
		free(containner);
		return AEC_UNDEFINED;
	}

	containner[idx].cnt = cnt + 1;
	memcpy(&containner[idx].cfg, para->cfg, sizeof(at_config_t));

	if (fdcm_write(fdcm_hdl, &containner[idx], CONFIG_CONTAINNER_SIZE) != CONFIG_CONTAINNER_SIZE) {
		FUN_DEBUG("fdcm_write faild.\n");
		fdcm_close(fdcm_hdl);
		free(containner);
		return AEC_UNDEFINED;
	}

	fdcm_close(fdcm_hdl);
	free(containner);

	return AEC_OK;
}

static AT_ERROR_CODE load(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	config_containner_t *containner;
	s32 i;
	fdcm_handle_t *fdcm_hdl;
	u32 flag[2];
	s32 idx;

	containner = (config_containner_t *)malloc(2*sizeof(config_containner_t));

	if (containner == NULL) {
		FUN_DEBUG("malloc faild.\n");
		free(containner);
		return AEC_UNDEFINED;
	}

	for (i=0; i<TABLE_SIZE(fdcm_hdl_tbl); i++) {
		flag[i] = 1;

		fdcm_hdl = fdcm_open(fdcm_hdl_tbl[i].flash, fdcm_hdl_tbl[i].addr, fdcm_hdl_tbl[i].size);

		if (fdcm_hdl == NULL) {
			FUN_DEBUG("fdcm_open faild.\n");
			free(containner);
			return AEC_UNDEFINED;
		}

		if (fdcm_read(fdcm_hdl, &containner[i], CONFIG_CONTAINNER_SIZE) != CONFIG_CONTAINNER_SIZE) {
			flag[i] = 0;
		}

		fdcm_close(fdcm_hdl);
	}

	if(flag[0] && flag[1]) {
		if (containner[0].cnt > containner[1].cnt) {
			idx = 0;
		}
		else {
			idx = 1;
		}
	}
	else if(flag[0]) {
		idx = 0;
	}
	else if(flag[1]) {
		idx = 1;
	}
	else {
		FUN_DEBUG("load fiald.\n");
		free(containner);
		return AEC_UNDEFINED;
	}

	memcpy(para->cfg, &containner[idx].cfg, sizeof(at_config_t));

	free(containner);

	return AEC_OK;
}

static AT_ERROR_CODE status(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	struct netif *nif = g_wlan_netif;
	s32 i;

	memset(para->sts, 0, sizeof(*para->sts));

	if (nif == NULL) {
		return AEC_UNDEFINED;
	}

	if (NET_IS_IP4_VALID(nif) && netif_is_link_up(nif)) {
#if 0
		char address[16];
		char gateway[16];
		char netmask[16];
		inet_ntoa_r(nif->ip_addr, address, sizeof(address));
		inet_ntoa_r(nif->gw, gateway, sizeof(gateway));
		inet_ntoa_r(nif->netmask, netmask, sizeof(netmask));
#endif
		memcpy(para->sts->ip_ipaddr, &nif->ip_addr, 4);
		memcpy(para->sts->ip_gw, &nif->gw, 4);
		memcpy(para->sts->ip_netmask, &nif->netmask, 4);
	}

	para->sts->current_time = time(NULL);

	para->sts->ip_sock_open = 0;

	for (i=0; i<MAX_SOCKET_NUM; i++) {
		if (networks.connect[i].flag) {
			para->sts->ip_sock_open |= (1<<i);
		}
	}

	if (g_server_enable) {
		para->sts->ip_sockd_port = g_server_arg.port;
	}

	wlan_ap_sta_num(&para->sts->wifi_num_assoc);

	return AEC_OK;
}

static AT_ERROR_CODE factory(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	//u8 mac[] = MAC;
	struct sysinfo *psysinfo;

	memcpy(para->cfg, &default_cfg, sizeof(at_config_t));

/* non-volatile config */
	strcpy(para->cfg->nv_manuf, MANUFACTURER);
	strcpy(para->cfg->nv_model, MODEL);
	strcpy(para->cfg->nv_serial, SERIAL);
	//memcpy(para->cfg->nv_wifi_macaddr, mac, sizeof(mac));
#if PRJCONF_SYSINFO_SAVE_TO_FLASH
	sysinfo_load();
#endif
	psysinfo = sysinfo_get();
	memcpy(para->cfg->nv_wifi_macaddr, psysinfo->mac_addr, sizeof(para->cfg->nv_wifi_macaddr));

	save(para, NULL);

	return AEC_OK;
}

extern s32 test_ping(char *hostname, int count);
static AT_ERROR_CODE ping(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	test_ping(para->u.ping.hostname,3);

	return AEC_OK;
}

static AT_ERROR_CODE peer(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	return AEC_OK;
}

static AT_ERROR_CODE disconnect(s32 id)
{
	if (networks.count > 0) {
		if (networks.connect[id].flag) {
			closesocket(networks.connect[id].fd);
			networks.connect[id].flag = 0;
			networks.count--;

			return AEC_OK;
		}
	}

	return AEC_DISCONNECT;
}

static AT_ERROR_CODE sockon(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	int type;
	int family;
	struct addrinfo hints_tcp = {0, AF_INET, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL};
	struct addrinfo hints_udp = {0, AF_INET, SOCK_DGRAM, IPPROTO_UDP, 0, NULL, NULL, NULL};

	int rc = -1;
	struct sockaddr_in address;
	struct addrinfo *result = NULL;
	s32 id = -1;
	int fd = 0;
	s32 i;

	if (networks.count < MAX_SOCKET_NUM) {
		for (i = 0; i < MAX_SOCKET_NUM; i++) {
			if (!networks.connect[i].flag) {
				id = i;
				break;
			}
		}

		if (id == -1) {
			return AEC_UNDEFINED;
		}

		strncpy(networks.connect[id].ip, para->u.sockon.hostname, IP_ADDR_SIZE);
		networks.connect[id].port = para->u.sockon.port;
		networks.connect[id].protocol = para->u.sockon.protocol;

		if (networks.connect[id].protocol == 0) { /* TCP */
			type = SOCK_STREAM;
			family = AF_INET;
			if ((rc = getaddrinfo(networks.connect[id].ip, NULL, &hints_tcp, &result)) == 0) {
				struct addrinfo *res = result;

				while (res) {
					if (res->ai_family == family)
					{
						result = res;
						break;
					}
					res = res->ai_next;
				}

				if (result->ai_family == family)
				{
					address.sin_port = htons(networks.connect[id].port);
					address.sin_family = family;
					address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
				}
				else
					rc = -1;

				freeaddrinfo(result);
			}

			if (rc == 0) {
				fd = socket(family, type, 0);
				if (fd < 0)
					return AEC_SOCKET_FAIL;

				rc = connect(fd, (struct sockaddr *)&address, sizeof(address));
				if (rc < 0) {
					closesocket(fd);
					return AEC_CONNECT_FAIL;
				}
			}
			else {
				return AEC_CONNECT_FAIL;
			}

			networks.connect[id].fd = fd;
			networks.connect[id].flag = 1;

			networks.count++;

			memset(&socket_cache[id], 0 ,sizeof(socket_cache_t));

			rsp->type = 0;
			rsp->vptr = (void *)id;

			return AEC_OK;
		}
		else if (networks.connect[id].protocol == 1) { /* UDP */
			type = SOCK_DGRAM;
			family = AF_INET;
			if ((rc = getaddrinfo(networks.connect[id].ip, NULL, &hints_udp, &result)) == 0) {
				struct addrinfo *res = result;

				while (res) {
					if (res->ai_family == family)
					{
						result = res;
						break;
					}
					res = res->ai_next;
				}

				if (result->ai_family == family)
				{
					address.sin_port = htons(networks.connect[id].port);
					address.sin_family = family;
					address.sin_addr.s_addr= htonl(INADDR_ANY);
				}
				else
					rc = -1;

				freeaddrinfo(result);
			}

			if (rc == 0) {
				fd = socket(family, type, 0);
				if (fd < 0)
					return AEC_SOCKET_FAIL;
				/* for receive */
				rc = bind(fd, (struct sockaddr *)&address, sizeof(address));
				if (rc < 0) {
					closesocket(fd);
					return AEC_BIND_FAIL;
				}
			}
			else {
				return AEC_BIND_FAIL;
			}

			networks.connect[id].fd = fd;
			networks.connect[id].flag = 1;

			networks.count++;

			memset(&socket_cache[id], 0 ,sizeof(socket_cache_t));

			rsp->type = 0;
			rsp->vptr = (void *)id;

			return AEC_OK;
		}
	}
	else {
		return AEC_LIMITED;
	}

	return AEC_OK;
}

static AT_ERROR_CODE sockw(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	s32 id;
	u8 *buffer;
	s32 len;
	struct sockaddr_in address;
	s32 timeout_ms = 10;
	int sentLen = 0;

	id = para->u.sockw.id;
	buffer = para->u.sockw.buf;
	len = para->u.sockw.len;

	if (networks.count > 0) {
		if (networks.connect[id].flag) {
			if (networks.connect[id].protocol == 0) { /* TCP */
				int rc = -1;
				fd_set fdset;
				struct timeval tv;

				FD_ZERO(&fdset);
				FD_SET(networks.connect[id].fd, &fdset);

				tv.tv_sec = timeout_ms / 1000;
				tv.tv_usec = (timeout_ms % 1000) * 1000;

				rc = select(networks.connect[id].fd + 1, NULL, &fdset, NULL, &tv);
				if (rc > 0) {
					if ((rc = send(networks.connect[id].fd, buffer, len, 0)) > 0)
						sentLen = rc;
					else if (rc == 0) {
						disconnect(id);
						sentLen = -1; /* disconnected with server */
					}
					else {
						sentLen = -2; /* network error */
					}
				}
				else if (rc == 0) {
					sentLen = 0; /* timeouted and sent 0 bytes */
				}
				else {
					sentLen = -2; /* network error */
				}
			}
			else if (networks.connect[id].protocol == 1) { /* UDP */
				int rc = -1;
				fd_set fdset;
				struct timeval tv;

				FD_ZERO(&fdset);
				FD_SET(networks.connect[id].fd, &fdset);

				tv.tv_sec = timeout_ms / 1000;
				tv.tv_usec = (timeout_ms % 1000) * 1000;

				address.sin_port = htons(networks.connect[id].port);
				address.sin_family = AF_INET;
				address.sin_addr.s_addr= inet_addr(networks.connect[id].ip);

				rc = select(networks.connect[id].fd + 1, NULL, &fdset, NULL, &tv);
				if (rc > 0) {
					if ((rc = sendto(networks.connect[id].fd, buffer, len, 0, (struct sockaddr *)&address, sizeof(address))) > 0)
						sentLen = rc;
					else if (rc == 0) {
						disconnect(id);
						sentLen = -1; /* disconnected with server */
					}
					else {
						sentLen = -2; /* network error */
					}
				}
				else if (rc == 0) {
					sentLen = 0; /* timeouted and sent 0 bytes */
				}
				else {
					sentLen = -2; /* network error */
				}
			}
		}
		else {
			return AEC_DISCONNECT;
		}
	}
	else {
		return AEC_DISCONNECT;
	}

	if (sentLen == -1) {
		return AEC_DISCONNECT;
	}
	else if (sentLen == -2) {
		return AEC_NETWORK_ERROR;
	}
	else if (sentLen == len) {
		return AEC_OK;
	}
	else {
		return AEC_SEND_FAIL;
	}
}


typedef struct Timer Timer;

struct Timer
{
	unsigned int end_time;
};


/** countdown_ms - set timeout value in mil seconds
 * @param timer - timeout timer where the timeout value save
 * @param timeout_ms - timeout in timeout_ms mil seconds
 */
static void countdown_ms(Timer* timer, unsigned int timeout_ms)
{
	timer->end_time = OS_TicksToMSecs(OS_GetTicks()) + timeout_ms;
}

/** countdown - set timeout value in seconds
 * @param timer - timeout timer where the timeout value save
 * @param timeout - timeout in timeout seconds
 */
 /*
static void countdown(Timer* timer, unsigned int timeout)
{
	countdown_ms(timer, timeout * 1000);
}
*/

/** left_ms - calculate how much time left before timeout
 * @param timer - timeout timer
 * @return the time left befor timeout, or 0 if it has expired
 */
static int left_ms(Timer* timer)
{
	int diff = (int)(timer->end_time) - (int)(OS_TicksToMSecs(OS_GetTicks()));
	return (diff < 0) ? 0 : diff;
}

/** expired - has it already timeouted
 * @param timer - timeout timer
 * @return 0 if it has already timeout, or otherwise.
 */
static char expired(Timer* timer)
{
	return 0 <= (int)OS_TicksToMSecs(OS_GetTicks()) - (int)(timer->end_time); /* is time_now over than end time */
}

static AT_ERROR_CODE read_socket(s32 id)
{
	int recvLen = 0;
	int leftms;
	int rc = -1;
	struct timeval tv;
	Timer timer;
	fd_set fdset;
	struct sockaddr_in address;
	socklen_t addr_len;

	u8 *buffer;
	s32 len;
	s32 timeout_ms = 10;

	buffer = socket_cache[id].buffer;
	len = SOCKET_CACHE_BUFFER_SIZE;

	countdown_ms(&timer, timeout_ms);

	do {
		leftms = left_ms(&timer);
		tv.tv_sec = leftms / 1000;
		tv.tv_usec = (leftms % 1000) * 1000;

		FD_ZERO(&fdset);
		FD_SET(networks.connect[id].fd, &fdset);

		rc = select(networks.connect[id].fd + 1, &fdset, NULL, NULL, &tv);
		if (rc > 0) {
			if (networks.connect[id].protocol == 0) { /* TCP */
				rc = recv(networks.connect[id].fd, buffer + recvLen, len - recvLen, 0);
			}
			else if (networks.connect[id].protocol == 1) { /* UDP */
				address.sin_port = htons(networks.connect[id].port);
				address.sin_family = AF_INET;
				address.sin_addr.s_addr= inet_addr(networks.connect[id].ip);

				addr_len = sizeof(address);

				rc = recvfrom(networks.connect[id].fd, buffer + recvLen, len - recvLen, 0, (struct sockaddr *)&address, &addr_len);
			}

			if (rc > 0) {
				/* received normally */
				recvLen += rc;
			} else if (rc == 0) {
				/* has disconnected with server */
				recvLen = -1;
				break;
			} else {
				/* network error */
				recvLen = -2;
				break;
			}
		} else if (rc == 0) {
			/* timeouted and return the length received */
			break;
		} else {
			/* network error */
			recvLen = -2;
			break;
		}
	} while (recvLen < len && !expired(&timer)); /* expired() is redundant? */

	if (recvLen >= 0) {
		if (recvLen > 0) {
			socket_cache[id].cnt = recvLen;
			socket_cache[id].offset = 0;
			socket_cache[id].flag = 1;
		}

		return AEC_OK;
	}
	else if (recvLen == -1) {
		return AEC_DISCONNECT;
	}
	else if (recvLen == -2) {
		return AEC_NETWORK_ERROR;
	}
	else {
		return AEC_UNDEFINED;
	}
}

static AT_ERROR_CODE sockq(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	AT_ERROR_CODE aec;
	s32 id;

	id = para->u.sockq.id;

	if (networks.count > 0) {
		if (networks.connect[id].flag) {
			if (socket_cache[id].flag) {
				rsp->type = 0;
				rsp->vptr = (void *)socket_cache[id].cnt;

				return AEC_OK;
			}

			aec = read_socket(id);
			if (aec != AEC_OK) {
				return aec;
			}

			if (socket_cache[id].flag) {
				rsp->type = 0;
				rsp->vptr = (void *)socket_cache[id].cnt;

				return AEC_OK;
			}
			else {
				rsp->type = 0;
				rsp->vptr = (void *)0;

				return AEC_OK;
			}
		}
		else {
			return AEC_DISCONNECT;
		}
	}
	else {
		return AEC_DISCONNECT;
	}
}

static AT_ERROR_CODE sockr(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	AT_ERROR_CODE aec;
	s32 id;
	u8 *buffer;
	s32 len,rlen;

	id = para->u.sockr.id;
	len = para->u.sockr.len;

	if (networks.count > 0) {
		if (networks.connect[id].flag) {
			/* FUN_DEBUG("len = %d\n", len); */
			if (len > 0) {
				if (!socket_cache[id].flag) {

					aec = read_socket(id);
					if (aec != AEC_OK) {
						return aec;
					}
				}

				if (socket_cache[id].flag) {
					buffer = socket_cache[id].buffer;
					rlen = len < socket_cache[id].cnt ? len : socket_cache[id].cnt;

					serial_write(&buffer[socket_cache[id].offset], rlen);

					socket_cache[id].cnt -= rlen;
					socket_cache[id].offset += rlen;
					len -= rlen;

					if (socket_cache[id].cnt <= 0) {
						socket_cache[id].flag = 0;
					}
				}
			}
		}
		else {
			return AEC_DISCONNECT;
		}
	}
	else {
		return AEC_DISCONNECT;
	}

	return AEC_OK;
}

static AT_ERROR_CODE sockc(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	s32 id;

	id = para->u.sockc.id;

	if (networks.count > 0) {
		if (networks.connect[id].flag) {
			closesocket(networks.connect[id].fd);
			networks.connect[id].flag = 0;
			networks.count--;

			return AEC_OK;
		}
	}

	return AEC_DISCONNECT;
}

static void server_task(void *arg)
{
	server_arg_t *server_arg;

	server_arg = arg;

	do {
		if (server_arg->protocol == 0) { /* TCP */
			struct sockaddr_in server_addr;
			struct sockaddr_in conn_addr;
			int sock_fd;				  /* server socked */
			int sock_conn;		  /* request socked */
			socklen_t addr_len;
			int err;
			int option;

			sock_fd = socket(AF_INET, SOCK_STREAM, 0);
			if (sock_fd == -1) {
				FUN_DEBUG("failed to create sock_fd!\n");
				break;
			}

			server_mutex_lock();
			g_server_net.sock_fd = sock_fd;
			server_mutex_unlock();

			FUN_DEBUG("sock fd = %d\n", sock_fd);

			memset(&server_addr, 0, sizeof(server_addr));
			server_addr.sin_family = AF_INET;
			server_addr.sin_addr.s_addr =htonl(INADDR_ANY);
			server_addr.sin_port = htons(server_arg->port);

			option = 1;
			if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (char *)&option, sizeof(option)) < 0) {
			 	FUN_DEBUG("failed to setsockopt sock_fd!\n");
				closesocket(sock_fd);
				break;
			}

			option = 1;
			if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEPORT, (char *)&option, sizeof(option)) < 0) {
			 	FUN_DEBUG("failed to setsockopt sock_fd!\n");
				closesocket(sock_fd);
				break;
			}

			err = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
			if (err < 0) {
			 	FUN_DEBUG("bind err = %d\n", err);
			 	FUN_DEBUG("failed to bind sock_fd!\n");
				closesocket(sock_fd);
				break;
			}

			err = listen(sock_fd, 1);
			if (err < 0) {
			  	FUN_DEBUG("failed to listen sock_fd!\n");
				closesocket(sock_fd);
				break;
			}

			addr_len = sizeof(struct sockaddr_in);

			while (1) {
				if (OS_SemaphoreWait(&g_server_sem, OS_WAIT_FOREVER) != OS_OK)
					continue;

				FUN_DEBUG("before accept!\n");
				sock_conn = accept(sock_fd, (struct sockaddr *)&conn_addr, &addr_len);
				FUN_DEBUG("after accept!\n");

				if (sock_conn > 0) {
					server_mutex_lock();
					g_server_net.flag = 1;
					g_server_net.conn_fd = sock_conn;
					server_mutex_unlock();
				}

				FUN_DEBUG("conn fd = %d\n", sock_conn);
			}
		}
		else if (server_arg->protocol == 1) { /* UDP */
			/* do nothing */
		}
	}while (0);

	server_mutex_lock();
	g_server_net.flag = 0;
	g_server_net.sock_fd = -1;
	g_server_net.conn_fd = -1;
	server_mutex_unlock();

	FUN_DEBUG("%s() end\n", __func__);
	OS_ThreadDelete(&g_server_thread);
}

static AT_ERROR_CODE sockd(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	s16 port;
	s32 protocol;

	port = para->u.sockd.port;
	protocol = para->u.sockd.protocol;

	if (port > 0) {
		if (!g_server_enable) {
			g_server_arg.port = para->u.sockd.port;
			g_server_arg.protocol = para->u.sockd.protocol;
			memset(&socket_cache[MAX_SOCKET_NUM], 0 ,sizeof(socket_cache_t));

			if (protocol == 0) { /* TCP */
				server_mutex_lock();
				g_server_net.flag = 0;
				g_server_net.sock_fd = -1;
				g_server_net.conn_fd = -1;
				server_mutex_unlock();

				if (OS_SemaphoreCreate(&g_server_sem, 1, 1) != OS_OK) {
					FUN_DEBUG("create semaphore failed\n");

					return AEC_UNDEFINED;
				}

				if (OS_ThreadCreate(&g_server_thread,
				                    "",
				                    server_task,
				                    &g_server_arg,
				                    OS_PRIORITY_NORMAL,
				                    SERVER_THREAD_STACK_SIZE) != OS_OK) {
					FUN_DEBUG("create server task failed\n");

					return AEC_UNDEFINED;
				}

				memset(&g_server_ctrl, 0, sizeof(g_server_ctrl));
			}
			else if (protocol == 1) { /* UDP */
				struct sockaddr_in	address;
				int rc;
				int fd;

				fd = socket(AF_INET, SOCK_DGRAM, 0);
				if (fd < 0) {
					return AEC_SOCKET_FAIL;
				}

				memset(&address, 0, sizeof(address));
				address.sin_family = AF_INET;
				address.sin_addr.s_addr =htonl(INADDR_ANY);
				address.sin_port = htons(port);
				/* for receive */
				rc = bind(fd, (struct sockaddr *)&address, sizeof(address));
				if (rc < 0) {
					closesocket(fd);
					return AEC_BIND_FAIL;
				}

				server_mutex_lock();
				g_server_net.flag = 1;
				g_server_net.sock_fd = -1;
				g_server_net.conn_fd = fd;
				server_mutex_unlock();
			}

			g_server_enable = 1;

			return AEC_OK;
		}
	}
	else if (g_server_enable) {
		u32 flag;
		s32 sock_fd,conn_fd;

		server_mutex_lock();
		flag = g_server_net.flag;
		sock_fd = g_server_net.sock_fd;
		conn_fd = g_server_net.conn_fd;
		server_mutex_unlock();

		if (g_server_arg.protocol == 0) { /* TCP */
			if (sock_fd != -1) {
				FUN_DEBUG("close fd = %d\n", sock_fd);
				closesocket(sock_fd);
			}

			if (flag) {
				FUN_DEBUG("close fd = %d\n", conn_fd);
				closesocket(conn_fd);
			}

			OS_SemaphoreDelete(&g_server_sem);
			OS_ThreadDelete(&g_server_thread);

		}
		else if (g_server_arg.protocol == 1) { /* UDP */
			if (flag) {
				FUN_DEBUG("close fd = %d\n", conn_fd);
				closesocket(conn_fd);
			}
		}

		g_server_enable = 0;

		return AEC_OK;
	}

	return AEC_UNDEFINED;
}

static AT_ERROR_CODE wifi(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	AT_ERROR_CODE aec = AEC_OK;

	switch (para->cfg->wifi_mode) {
	case 0: /* IDLE */
		aec = AEC_IMPROPER_OPERATION;
		break;

	case 1: /* STA */

		if (para->u.wifi.value == 0) {
			wlan_sta_disable();
		}
		else if (para->u.wifi.value == 1) {
			wlan_sta_enable();
		}
		else {
			aec = AEC_PARA_ERROR;
		}

		break;

	case 2: /* AP */

		if (para->u.wifi.value == 0) {
			wlan_ap_disable();
		}
		else if (para->u.wifi.value == 1) {
			wlan_ap_enable();
		}
		else {
			aec = AEC_PARA_ERROR;
		}

		break;

	case 3: /* IBSS */
		aec = AEC_IMPROPER_OPERATION;
		break;

	default:

		aec = AEC_PARA_ERROR;

		break;
	}

	return aec;
}

static AT_ERROR_CODE reassociate(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	wlan_sta_connect();

	return AEC_OK;
}

static AT_ERROR_CODE gpioc(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	return AEC_OK;
}

static AT_ERROR_CODE gpior(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	return AEC_OK;
}

static AT_ERROR_CODE gpiow(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	return AEC_OK;
}

static s32 freq_to_chan(s32 freq)
{
	s32 i;

	for (i = 0; i < TABLE_SIZE(channel_freq_tbl); ++i) {
		if(freq == channel_freq_tbl[i]) {
			break;
		}
	}

	if(i >= TABLE_SIZE(channel_freq_tbl)) {
		return -1;
	}

	return i+1;
}

static AT_ERROR_CODE scan(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	AT_ERROR_CODE aec = AEC_OK;
	int ret = -1;
	int size;
	char ssid[32+1];
	wlan_sta_scan_results_t results;

	if (para->cfg->wifi_mode != 1) { /* STA */
		return AEC_IMPROPER_OPERATION;
	}

	wlan_sta_scan_once();

	size = MAX_SCAN_RESULTS;

	results.ap = cmd_malloc(size * sizeof(wlan_sta_ap_t));
	if (results.ap == NULL) {
		aec = AEC_SCAN_FAIL;
	}

	if (aec == AEC_OK) {
		results.size = size;
		ret = wlan_sta_scan_result(&results);

		if (ret == 0) {
			/* cmd_wlan_sta_print_scan_results(&results); */
			int i;

			for (i = 0; i < results.num; i++) {
				memcpy(ssid, results.ap[i].ssid.ssid, results.ap[i].ssid.ssid_len);
				ssid[results.ap[i].ssid.ssid_len] = '\0';
				at_dump("%2d    BSS %02X:%02X:%02X:%02X:%02X:%02X    SSID: %-32.32s    "
					 "CHAN: %2d    RSSI: %d    flags: %08x    wpa_key_mgmt: %08x    "
					 "wpa_cipher: %08x    wpa2_key_mgmt: %08x    wpa2_cipher: %08x\n",
					 i + 1, (results.ap[i].bssid)[0], (results.ap[i].bssid)[1],
					 (results.ap[i].bssid)[2], (results.ap[i].bssid)[3],
					 (results.ap[i].bssid)[4], (results.ap[i].bssid)[5],
					 ssid, freq_to_chan(results.ap[i].freq),results.ap[i].level,
					 results.ap[i].wpa_flags, results.ap[i].wpa_key_mgmt,
					 results.ap[i].wpa_cipher,results.ap[i].wpa2_key_mgmt,
					 results.ap[i].wpa2_cipher);
			}
		}

		cmd_free(results.ap);
	}

	return aec;
}

void occur(uint32_t evt, uint32_t data, void *arg)
{
	int idx = EVENT_SUBTYPE(evt);

	if(idx >= 0 && idx < TABLE_SIZE(event)) {
		if (at_event(idx)) {
			at_dump("+EVENT:%d:%s\r\n", idx, event[idx]);
		}
	}
	else {
		FUN_DEBUG("Unsupported.\r\n");
	}
}

