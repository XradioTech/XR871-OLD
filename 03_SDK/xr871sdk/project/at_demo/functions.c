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


#include "at_command.h"
#include "at_queue.h"

#ifdef __arm__
#include "cmd_util.h"
#include "cmd_wlan.h"
#include "net/wlan/wlan.h"
#include "common/cmd/cmd_ping.h"

#include "lwip/inet.h"
#include "common/net_ctrl/net_ctrl.h"

#include "driver/chip/hal_rtc.h"

#include "sys/fdcm.h"

#include "serial.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "errno.h"

#endif

#define MANUFACTURER	"XRADIO"
#define MODEL			"serial-to-wifi"
#define SERIAL			"01234567"
#define MAC				{0x00, 0x11, 0x22, 0x33, 0x44, 0x55}

#define CONFIG_FDCM_ADDR	0x120000UL
#define CONFIG_FDCM_SIZE	0x10000UL

#define CONFIG_CONTAINNER_SIZE sizeof(config_containner_t)

#define MAX_SOCKET_NUM	8
#define IP_ADDR_SIZE	15
#define SOCKET_CACHE_BUFFER_SIZE	1024


typedef struct
{
	u32 cnt;
	at_config_t cfg;
}config_containner_t;

typedef struct
{
	char ip[IP_ADDR_SIZE+1];
	s16 port;
	s32 protocol;
	s32 fd;
	u32 flag;
}connect_t;

typedef struct
{
	s32 count;
	connect_t connect[MAX_SOCKET_NUM];	
}network_t;

typedef struct {
	u32 flag;
	s32 offset;
	s32 cnt;
	u8 buffer[SOCKET_CACHE_BUFFER_SIZE];
}socket_cache_t;

static socket_cache_t socket_cache[MAX_SOCKET_NUM];


static AT_ERROR_CODE callback(AT_CALLBACK_CMD cmd, at_callback_para_t *para, at_callback_rsp_t *rsp);

static AT_ERROR_CODE act(at_callback_para_t *para, at_callback_rsp_t *rsp);
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

static const struct
{
	s32 cmd;
	AT_ERROR_CODE (*handler)(at_callback_para_t *para, at_callback_rsp_t *rsp);
}callback_tbl[] =
{
	{ACC_ACT,				act},
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

static const u32 channel_freq_tbl[] = 
{
	2412,2417,2422,2427,2432,2437,2442,2447,2452,2457,2462,2467,2472
};

static const char *event[] = {
	"wlan connected",
	"wlan disconnected",
	"wlan scan success",
	"wlan scan failed",
	"wlan 4way handshake failed",
	"wlan connect failed",
	"wlan smart config result",
	"wlan airkiss result",
	"network up",
	"network down",
};

#ifdef __arm__
static const fdcm_handle_t fdcm_hdl_tbl[] = {
    {CONFIG_FDCM_ADDR, CONFIG_FDCM_SIZE},
    {CONFIG_FDCM_ADDR+CONFIG_FDCM_SIZE, CONFIG_FDCM_SIZE}
};
#endif

static network_t networks;


#ifdef __arm__
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
#endif

static u8 queue_buf[1024];
#ifdef _DEBUG
extern s32 serial_read(u8 *buf, s32 size);
#endif
void at_cmd_init(void)
{
	at_queue_init(queue_buf, sizeof(queue_buf), serial_read);

	at_init(callback);
}

void at_cmd_exec(void)
{
	at_parse();
}

static AT_ERROR_CODE callback(AT_CALLBACK_CMD cmd, at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	s32 i;
#ifdef _DEBUG
	printf("callback cmd = %d\n",cmd);
#endif
	for (i = 0; i < TABLE_SIZE(callback_tbl); i++) {
		if (cmd == callback_tbl[i].cmd) {
			if (callback_tbl[i].handler != NULL) {
				return callback_tbl[i].handler(para, rsp);
			}
			else {
#ifdef _DEBUG
				printf("callback cmd = %d is unimplimented!\n", cmd);
#endif
				return AEC_UNDEFINED;
			}
		}
	}
#ifdef _DEBUG	
	printf("callback cmd = %d is unsupported!\n", cmd);
#endif
	return AEC_UNSUPPORTED;
}

static AT_ERROR_CODE act(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	char cmd[AT_PARA_MAX_SIZE*2];

	switch (para->u.act.num) {
	case 0:
	
		break;

	case 1:
	
		switch (para->cfg->wifi_mode) {
		case 0: /* IDLE */

			break;

		case 1: /* STA */

			sprintf(cmd, "set ssid \"%s\"", para->cfg->wifi_ssid);
#ifdef __arm__
			cmd_wlan_sta_exec(cmd);
#endif

			sprintf(cmd, "set psk \"%s\"", para->cfg->wifi_wpa_psk_text);
#ifdef __arm__
			cmd_wlan_sta_exec(cmd);
#endif

			sprintf(cmd, "enable");
#ifdef __arm__
			cmd_wlan_sta_exec(cmd);
#endif

			break;

		case 2: /* IBSS */

			break;

		case 3: /* AP */

			break;

		default:

			return AEC_UNSUPPORTED;

			break;

		}

		break;

	case 2:
	
		break;

	case 3:
	
		break;

	case 4:
	
		break;

	default:
	
		return AEC_PARA_ERROR;

		break;
	}

	return AEC_OK;
}

static AT_ERROR_CODE mode(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef __arm__
	AT_ERROR_CODE res = AEC_OK;
	s32 id;
	u8 *buffer;
	s32 len;
	s32 timeout_ms=10;
	fd_set fdset_r,fdset_w;
	int fd;	

	id = 0;
	fd = networks.connect[id].fd;
	
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
			
					rc = recv(fd, buffer, len, 0);
					if (rc > 0) {
						/* received normally */
						serial_write(buffer, rc);						
					} 
					else if (rc == 0) {
						/* has disconnected with server */
						res = AEC_UNDEFINED;
					}
					else {
						/* network error */
						res = AEC_UNDEFINED;
					}
				}			
			} 
			else if (rc == 0) {
				/* timeouted and sent 0 bytes */
			}
			else {
				 /* network error */
				res = AEC_UNDEFINED;
			}

			rc = select(fd + 1, NULL, &fdset_w, NULL, &tv);
			if (rc > 0) {				
				if (FD_ISSET(fd, &fdset_w)) {						
					buffer = para->u.mode.buf;
					len = para->u.mode.len;
					
					if (buffer != NULL && len >0) {
						if ((rc = send(fd, buffer, len, 0)) > 0) {							
							/* do nothing */
						}
						else if (rc == 0) {
							/* disconnected with server */
							res = AEC_UNDEFINED;
						}
						else {
							/* network error */
							res = AEC_UNDEFINED;							
						}
					}
				}			
			} 
			else if (rc == 0) {
				/* timeouted and sent 0 bytes */
			}
			else {
				 /* network error */
				res = AEC_UNDEFINED;
			}			
		}
		else {
			res = AEC_SWITCH_MODE;
		}
	}
	else {
		res = AEC_SWITCH_MODE;
	}

	return res;
#else
	return AEC_OK;
#endif	

}

static AT_ERROR_CODE save(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef _WIN32
	FILE *fp;

	fp = fopen("config.bin","wb");

	if (fp != NULL) {
		fwrite(para->cfg, sizeof(at_config_t), 1, fp);

		fclose(fp);
	}
#endif
#ifdef __arm__
	config_containner_t *containner;
	s32 i;
	fdcm_handle_t *fdcm_hdl;
	u32 flag[2];
	s32 idx,cnt;

	containner = (config_containner_t *)malloc(2*sizeof(config_containner_t));

	if (containner == NULL) {
		DEBUG("malloc faild.\n");
		return AEC_UNDEFINED;
	}

	for (i=0; i<TABLE_SIZE(fdcm_hdl_tbl); i++) {
		flag[i] = 1;
		
		fdcm_hdl = fdcm_open(fdcm_hdl_tbl[i].addr, fdcm_hdl_tbl[i].size);

		if (fdcm_hdl == NULL) {
			DEBUG("fdcm_open faild.\n");
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

	fdcm_hdl = fdcm_open(fdcm_hdl_tbl[idx].addr, fdcm_hdl_tbl[idx].size);

	if (fdcm_hdl == NULL) {
		DEBUG("fdcm_open faild.\n");
		free(containner);		
		return AEC_UNDEFINED;
	}

	containner[idx].cnt = cnt + 1;
	memcpy(&containner[idx].cfg, para->cfg, sizeof(at_config_t));	

	if (fdcm_write(fdcm_hdl, &containner[idx], CONFIG_CONTAINNER_SIZE) != CONFIG_CONTAINNER_SIZE) {
		DEBUG("fdcm_write faild.\n");
		fdcm_close(fdcm_hdl);
		free(containner);		
		return AEC_UNDEFINED;
	}

	fdcm_close(fdcm_hdl);
	free(containner);
#endif
	return AEC_OK;
}

static AT_ERROR_CODE load(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef _WIN32
	FILE *fp;

	fp = fopen("config.bin","rb");

	if (fp != NULL) {
		fread(para->cfg, sizeof(at_config_t), 1, fp);

		fclose(fp);
	}
#endif
#ifdef __arm__
	config_containner_t *containner;
	s32 i;
	fdcm_handle_t *fdcm_hdl;
	u32 flag[2];
	s32 idx;
	u8 mac[]=MAC;

	containner = (config_containner_t *)malloc(2*sizeof(config_containner_t));

	if (containner == NULL) {
		DEBUG("malloc faild.\n");
		free(containner);
		return AEC_UNDEFINED;
	}	

	for (i=0; i<TABLE_SIZE(fdcm_hdl_tbl); i++) {
		flag[i] = 1;
		
		fdcm_hdl = fdcm_open(fdcm_hdl_tbl[i].addr, fdcm_hdl_tbl[i].size);

		if (fdcm_hdl == NULL) {
			DEBUG("fdcm_open faild.\n");
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
		DEBUG("load fiald.\n");
		free(containner);		
		return AEC_UNDEFINED;
	}

	memcpy(para->cfg, &containner[idx].cfg, sizeof(at_config_t));

	free(containner);
	
/* non-volatile config */
	strcpy(para->cfg->nv_manuf, MANUFACTURER);
	strcpy(para->cfg->nv_model, MODEL);
	strcpy(para->cfg->nv_serial, SERIAL);
	memcpy(para->cfg->nv_wifi_macaddr, mac, sizeof(mac));
#endif

	return AEC_OK;
}

static AT_ERROR_CODE status(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef __arm__
	struct netif *nif = g_wlan_netif;
	struct tm beg,now;
	s32 sec;
	u8 leap, year, month, mday, hour, minute, second;
	RTC_WeekDay wday;
	
	if (nif == NULL) {
		return AEC_UNDEFINED;
	}
	
	if (netif_is_up(nif) && netif_is_link_up(nif)) {
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

	HAL_RTC_GetYYMMDD(&leap, &year, &month, &mday);
	HAL_RTC_GetDDHHMMSS(&wday, &hour, &minute, &second);

	now.tm_year = year + 1900;
	now.tm_mon = month;	
	now.tm_mday = mday;
	now.tm_hour = hour;
	now.tm_min = minute;
	now.tm_sec = second;	
#if 0
	printf("year=%d,month=%d,day=%d,hour=%d,minute=%d,second=%d\n", 
		year, month, mday, hour, minute, second);
#endif
	beg.tm_year = 1900;
	beg.tm_mon = 1;	
	beg.tm_mday = 1;
	beg.tm_hour = 0;
	beg.tm_min = 0;
	beg.tm_sec = 0;

	sec = difftime(mktime(&now), mktime(&beg));
	
	para->sts->current_time = sec;
#endif

	return AEC_OK;
}

static AT_ERROR_CODE factory(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	return AEC_OK;
}
static AT_ERROR_CODE ping(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	char cmd[AT_PARA_MAX_SIZE*2];

	sprintf(cmd, "%s %d", para->u.ping.hostname,3);
#ifdef __arm__	
	cmd_ping_exec(cmd);
#endif

	return AEC_OK;
}

static AT_ERROR_CODE peer(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	return AEC_OK;
}

static AT_ERROR_CODE disconnect(s32 id)
{
#ifdef __arm__
	if (networks.count > 0) {
		if (networks.connect[id].flag) {
			closesocket(networks.connect[id].fd);
			networks.connect[id].flag = 0;			
			networks.count--;
			
			return AEC_OK;
		}
	}		

	return AEC_DISCONNECT;
#else
	return AEC_OK;
#endif
}

static AT_ERROR_CODE sockon(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef __arm__
	int type = SOCK_STREAM;
	int family = AF_INET;
	struct addrinfo hints = {0, family, type, IPPROTO_TCP, 0, NULL, NULL, NULL};
	
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

		type = SOCK_STREAM;
		family = AF_INET;
	
		strncpy(networks.connect[id].ip, para->u.sockon.hostname, IP_ADDR_SIZE);
		networks.connect[id].port = para->u.sockon.port;
		networks.connect[id].protocol = para->u.sockon.protocol;

		if (networks.connect[id].protocol == 0) { /* TCP */
			if ((rc = getaddrinfo(networks.connect[id].ip, NULL, &hints, &result)) == 0) {
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

		}
	}
	else {
		return AEC_LIMITED;
	}
#endif	
	return AEC_OK;
}

static AT_ERROR_CODE sockw(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef __arm__
	s32 id;
	u8 *buffer;
	s32 len;
	s32 timeout_ms = 10;
	int sentLen = 0;
	
	id = para->u.sockw.id;
	buffer = para->u.sockw.buf;
	len = para->u.sockw.len;
	
	if (networks.count > 0) {
		if (networks.connect[id].flag) {
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
#else
	return AEC_OK;
#endif	
}

#ifdef __arm__
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
			rc = recv(networks.connect[id].fd, buffer + recvLen, len - recvLen, 0);
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
#endif

static AT_ERROR_CODE sockq(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef __arm__
	/* it's a bug fixed version which may cause a blocking even it has been timeouted */
	int recvLen = 0;
	int leftms;
	int rc = -1;
	struct timeval tv;
	Timer timer;
	fd_set fdset;

	s32 id;
	u8 *buffer;
	s32 len;
	s32 timeout_ms = 10;
	
	id = para->u.sockq.id;	

	if (networks.count > 0) {
		if (networks.connect[id].flag) {
			if (socket_cache[id].flag) {
				rsp->type = 0;
				rsp->vptr = (void *)socket_cache[id].cnt;
	
				return AEC_OK;
			}			
			
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

				/* DEBUG("rc = %d\n", rc); */
								
				if (rc > 0) {
					rc = recv(networks.connect[id].fd, buffer + recvLen, len - recvLen, 0);
					/* DEBUG("rc = %d\n", rc); */
					if (rc > 0) {
						/* received normally */
						recvLen += rc;
					} else if (rc == 0) {
						/* has disconnected with server */
						recvLen = -1;
						disconnect(id);
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

				rsp->type = 0;
				rsp->vptr = (void *)recvLen;

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
		else {
			return AEC_DISCONNECT;
		}
	}
	else {
		return AEC_DISCONNECT;
	}
#else
	return AEC_OK;
#endif
}

static AT_ERROR_CODE sockr(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef __arm__
	AT_ERROR_CODE aec;
	s32 id;
	u8 *buffer;
	s32 len,rlen;
	
	id = para->u.sockr.id;
	len = para->u.sockr.len; 

	if (networks.count > 0) {
		if (networks.connect[id].flag) {
			/* DEBUG("len = %d\n", len); */
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
#endif
	return AEC_OK;
}

static AT_ERROR_CODE sockc(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
#ifdef __arm__
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

	return AEC_UNDEFINED;
#else
	return AEC_OK;
#endif
}

static AT_ERROR_CODE sockd(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	return AEC_OK;
}

static AT_ERROR_CODE wifi(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	char cmd[AT_PARA_MAX_SIZE*2];

	if (para->u.wifi.value == 0) {
		sprintf(cmd, "disable");
	}
	else if (para->u.wifi.value == 1) {
		sprintf(cmd, "enable");
	}
	else {
		return AEC_PARA_ERROR;
	}

#ifdef __arm__
	cmd_wlan_sta_exec(cmd);
#endif

	return AEC_OK;
}

static AT_ERROR_CODE reassociate(at_callback_para_t *para, at_callback_rsp_t *rsp)
{
	char cmd[AT_PARA_MAX_SIZE*2];
	
	sprintf(cmd, "connect");
	
#ifdef __arm__
	cmd_wlan_sta_exec(cmd);
#endif

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
#ifdef __arm__
	int ret = -1;
	int size;
	//int tmp;
	wlan_sta_scan_results_t results;	
#endif

	char cmd[AT_PARA_MAX_SIZE*2];
	
	sprintf(cmd, "scan once");
		
#ifdef __arm__
	cmd_wlan_sta_exec(cmd);
#endif

#ifdef __arm__
	size = 10;

	results.ap = (wlan_sta_scan_ap_t *)cmd_malloc(size * sizeof(wlan_sta_scan_ap_t));
	if (results.ap == NULL) {	
		return AEC_SCAN_FAIL;
	}
	results.size = size;
	ret = wlan_sta_scan_result(&results);
	
	if (ret == 0) {
		/* cmd_wlan_sta_print_scan_results(&results); */
		int i;

		for (i = 0; i < results.num; i++) {
			AT_DUMP("%2d    BSS %02X:%02X:%02X:%02X:%02X:%02X    SSID: %-32.32s    "
				 "CHAN: %2d    RSSI: %d    flags: %08x    wpa_key_mgmt: %08x    "
				 "wpa_cipher: %08x    wpa2_key_mgmt: %08x    wpa2_cipher: %08x\n",
				 i + 1, (results.ap[i].bssid)[0], (results.ap[i].bssid)[1],
				 (results.ap[i].bssid)[2], (results.ap[i].bssid)[3],
				 (results.ap[i].bssid)[4], (results.ap[i].bssid)[5],
				 results.ap[i].ssid, freq_to_chan(results.ap[i].freq),results.ap[i].level,
				 results.ap[i].wpa_flags, results.ap[i].wpa_key_mgmt,
				 results.ap[i].wpa_cipher,results.ap[i].wpa2_key_mgmt,
				 results.ap[i].wpa2_cipher);			
		}		
	}
	
	cmd_free(results.ap);
			
#if 0
	tmp = 10;
	cmd_memset(&req, 0, sizeof(req));
	req.size = tmp;
	req.entry = cmd_malloc(tmp * sizeof(struct wpa_ctrl_req_scan_entry));
	ret = wlan_ctrl_request(WPA_CTRL_CMD_SCAN_RESULTS, &req);
	if (ret == 0) {
		u8 i, len;

		for (i = 0; i < req.count; ++i) {
			len = req.entry[i].ssid_len;
			if (len > sizeof(req.entry[i].ssid) - 1)
				len = sizeof(req.entry[i].ssid) - 1;
			req.entry[i].ssid[len] = '\0';
			printf("%2d    BSS %02X:%02X:%02X:%02X:%02X:%02X    SSID: %-32.32s    "
				 "CHAN: %2d    RSSI: %d    flags: %08x    wpa_key_mgmt: %08x    "
				 "wpa_cipher: %08x    wpa2_key_mgmt: %08x    wpa2_cipher: %08x\n",
				 i + 1, (req.entry[i].bssid)[0], (req.entry[i].bssid)[1],
				 (req.entry[i].bssid)[2], (req.entry[i].bssid)[3],
				 (req.entry[i].bssid)[4], (req.entry[i].bssid)[5],
				 req.entry[i].ssid, freq_to_chan(req.entry[i].freq), req.entry[i].level,
				 req.entry[i].wpa_flags, req.entry[i].wpa_key_mgmt_flags,
				 req.entry[i].wpa_cipher_flags, req.entry[i].wpa2_key_mgmt_flags,
				 req.entry[i].wpa2_cipher_flags);
	}		
	}
	cmd_free(req.entry);
#endif	
#endif			

	return AEC_OK;
}

int occur(int idx)
{
	printf("+WIND:%d:%s\r\n", idx, event[idx]);
	
	return 0;
}

