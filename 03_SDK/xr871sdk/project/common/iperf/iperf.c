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

#include "kernel/os/os_thread.h"
#include "kernel/os/os_errno.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"
#include "lwip/sockets.h"

#include "iperf.h"
#include "iperf_debug.h"

#define iperf_thread_exit(thread)	OS_ThreadDelete(thread);
#define iperf_errno			OS_GetErrno()

#define IPERF_SPEED_FORMAT			1 // 0: Mbps, 1: KBps

#define IPERF_UDP_SEND_PORT 		5001
#define IPERF_UDP_RECV_PORT 		5002
#define IPERF_TCP_SEND_PORT 		5003
#define IPERF_TCP_RECV_PORT 		5004

#define IPERF_BUF_SIZE 				(1500)
#define IPERF_UDP_SEND_DATA_LEN		(1470)	// UDP: 1470 + 8  + 20 = 1498
#define IPERF_TCP_SEND_DATA_LEN		(1460)	// TCP: 1460 + 20 + 20 = 1500

#define IPERF_THREAD_STACK_SIZE		(2 * 1024)
static OS_Thread_t g_iperf_thread;

// in 1ms
#define IPERF_TIME_PER_SEC		1000
#define IPERF_TIME()			OS_GetTicks()
#define IPERF_SEC_2_INTERVAL(sec)	((sec) * IPERF_TIME_PER_SEC)
#define IPERF_CALC_SPEED_INTERVAL	IPERF_SEC_2_INTERVAL(2)

#define IPERF_SELECT_TIMEOUT		2000
#define WAIT_TIMES			10
#define _IPERF_SELECT
static int iperf_thread_stop_flag = 0;

#define CLEAR_IPERF_STOP_FLAG(flg)	 (flg = 0)
#define SET_IPERF_STOP_FLAG(flg)	 (flg = 1)
#define CHECK_IPERF_RUN_FLAG	CHECK_FLAG

int CHECK_FLAG(int flg)
{
	if (flg)
		return 0;
	else
		return 1;
}

#if (IPERF_SPEED_FORMAT == 0)
// @return Mbits/sec
static __inline uint32_t iperf_speed(uint32_t time, uint32_t bytes)
{
	return (bytes * 8 / 1024 / 1024 / (time / IPERF_TIME_PER_SEC));
}
#define IPERF_SPEED_UNIT	"Mb/s"
#elif (IPERF_SPEED_FORMAT == 1)
// @return KBytes/sec
static __inline uint32_t iperf_speed(uint32_t time, uint32_t bytes)
{
	return (bytes / 1024 / (time / IPERF_TIME_PER_SEC));
}
#define IPERF_SPEED_UNIT	"KB/s"
#endif

static __inline void iperf_udp_seq_set(uint32_t *p, uint32_t seq)
{
	*p = htonl(seq);
}

static int iperf_sock_create(int sock_type, short local_port, int do_bind)
{
	int local_sock;
	struct sockaddr_in local_addr;
	int ret, tmp;

	local_sock = socket(AF_INET, sock_type, 0);
	if (local_sock < 0) {
		IPERF_ERR("socket() return %d\n", local_sock);
		return local_sock;
	}

	if (!do_bind)
		return local_sock;

	tmp = 1;
	ret = setsockopt(local_sock, SOL_SOCKET, SO_REUSEADDR, &tmp, sizeof(int));
	if (ret != 0) {
		IPERF_ERR("setsockopt(SO_REUSEADDR) failed, err %d\n", iperf_errno);
		closesocket(local_sock);
		return -1;
	}

	if (sock_type == SOCK_STREAM) {
		tmp = 1;
		ret = setsockopt(local_sock, SOL_SOCKET, SO_KEEPALIVE, &tmp, sizeof(int));
		if (ret != 0) {
			IPERF_ERR("setsockopt(SO_KEEPALIVE) failed, err %d\n", iperf_errno);
			closesocket(local_sock);
			return -1;
		}
	}

	/* bind socket to port */
	memset(&local_addr, 0, sizeof(struct sockaddr_in));
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	local_addr.sin_port = htons(local_port);
	local_addr.sin_family = AF_INET;
	ret = bind(local_sock, (struct sockaddr *)&local_addr, sizeof(struct sockaddr_in));
	if (ret < 0) {
		IPERF_ERR("Failed to bind socket %d, err %d\n", local_sock, iperf_errno);
		closesocket(local_sock);
		return -1;
	}
	return local_sock;
}

static uint8_t *iperf_buf_new(uint32_t size)
{
	uint32_t i;
	uint8_t *data_buf = malloc(size);
	if (data_buf == NULL) {
		IPERF_ERR("malloc() failed!\n");
	} else {
		for (i = 0; i < size; ++i) {
			data_buf[i] = '0' + i % 10;
		}
	}
	return data_buf;
}

#define iperf_loop_init() 							\
	data_total_cnt = 0;								\
	data_cnt = 0;									\
	beg_tm = IPERF_TIME();							\
	end_tm = beg_tm + IPERF_CALC_SPEED_INTERVAL;	\
	run_beg_tm = beg_tm;							\
	run_end_tm = run_beg_tm + run_time;

#define iperf_calc_speed()										\
	cur_tm = IPERF_TIME();										\
	if (cur_tm > end_tm) {										\
		IPERF_LOG(1, "%u "IPERF_SPEED_UNIT"\n",					\
		          iperf_speed(cur_tm - beg_tm, data_cnt));		\
		data_total_cnt += data_cnt;								\
		if (cur_tm > run_end_tm && run_time) {					\
			IPERF_LOG(1, "TEST END: %u "IPERF_SPEED_UNIT"\n",	\
				  iperf_speed(cur_tm - run_beg_tm,				\
				              data_total_cnt));					\
			break;												\
		}														\
		data_cnt = 0;											\
		beg_tm = IPERF_TIME();									\
		end_tm = beg_tm + IPERF_CALC_SPEED_INTERVAL;			\
	}

#ifdef _IPERF_SELECT

static int IPERF_SELECT(int socket, int timeout, void *set, int rw)
{
        int rc = -1;
        fd_set *FDSET = (fd_set *)set;
        struct timeval tv;
        FD_ZERO(FDSET);
        FD_SET(socket, FDSET);
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
	if (rw == 0)
        	rc = select(socket + 1, FDSET, NULL, NULL, &tv);
	else
		rc = select(socket + 1, NULL, FDSET, NULL, &tv);
        return rc;
}

int IPERF_SET_NONBLOCK_MODE(int fd)
{
	int ret = -1, val = 1;
	if (ioctlsocket(fd, FIONBIO, (void *)&val) != 0) {
		IPERF_ERR("nonblock: fcntl(F_SETFL).\n");
	} else {
		ret = 0;	/* Success */
	}
	return ret;
}

#define CHECK_SELECT_RESULT(r)	if (r < 0) { \
        IPERF_ERR("select return err %d\n",iperf_errno); \
        	break;	\
		} else if (r == 0) {	\
			continue;	\
	} else

//#define CHECK_FAIL
#ifdef CHECK_FAIL
#define	CHECK_FAIL_TIMES(m,n)	\
			if (n) m = 0; \
			else if (++m > WAIT_TIMES)\
				break
#else
#define	CHECK_FAIL_TIMES(m,n) NULL
#endif

#endif

void iperf_udp_send_task(void *arg)
{
	int local_sock = -1;
	struct sockaddr_in remote_addr;
	struct iperf_data *idata = (struct iperf_data *)arg;
	char *remote_ip = idata->remote_ip;
	uint32_t run_time = IPERF_SEC_2_INTERVAL(idata->run_time);
	uint32_t port = idata->port;
	uint8_t *data_buf = NULL;
	int32_t data_len;
	uint32_t udp_seq_no = 0;

	local_sock = iperf_sock_create(SOCK_DGRAM, IPERF_UDP_SEND_PORT, 1);
	if (local_sock < 0) {
		IPERF_ERR("socket() return %d\n", local_sock);
		goto socket_error;
	}

	data_buf = iperf_buf_new(IPERF_BUF_SIZE);
	if (data_buf == NULL) {
		IPERF_ERR("malloc() failed!\n");
		goto socket_error;
	}

	memset(&remote_addr, 0, sizeof(struct sockaddr_in));
	remote_addr.sin_addr.s_addr = inet_addr(remote_ip);
	if (port == 0) {
		port = IPERF_UDP_RECV_PORT;
	}
	remote_addr.sin_port = htons(port);
	remote_addr.sin_family = AF_INET;

	IPERF_DBG("iperf: UDP send to %s:%d\n", remote_ip, port);

	uint32_t run_end_tm, run_beg_tm, beg_tm, end_tm, cur_tm;
	uint32_t data_total_cnt, data_cnt;

	iperf_udp_seq_set((uint32_t *)data_buf, udp_seq_no);
	iperf_loop_init();

	while (CHECK_IPERF_RUN_FLAG(iperf_thread_stop_flag)) {

		data_len = sendto(local_sock, data_buf, IPERF_UDP_SEND_DATA_LEN, 0,
		                  (struct sockaddr *)&remote_addr, sizeof(remote_addr));
		if (data_len == IPERF_UDP_SEND_DATA_LEN) {
			data_cnt += data_len;
			iperf_udp_seq_set((uint32_t *)data_buf, ++udp_seq_no);
		} else if (data_len < 0) {
			//IPERF_ERR("sendto return %d, err %d\n", data_len, iperf_errno);
		} else {
			//IPERF_ERR("sendto return %d, err %d\n", data_len, iperf_errno);
		}
		iperf_calc_speed();
	}

socket_error:
	if (data_buf)
		free(data_buf);
	if (local_sock >= 0)
		closesocket(local_sock);

	IPERF_DBG("%s() exit!\n", __func__);
	iperf_thread_exit(&g_iperf_thread);
}

void iperf_udp_recv_task(void *arg)
{
	int local_sock = -1;
	struct iperf_data *idata = (struct iperf_data *)arg;
	uint32_t run_time = IPERF_SEC_2_INTERVAL(idata->run_time);
	uint32_t port = idata->port;
	uint8_t *data_buf = NULL;
	int32_t data_len;

	if (port == 0) {
		port = IPERF_UDP_RECV_PORT;
	}
	local_sock = iperf_sock_create(SOCK_DGRAM, port, 1);
	if (local_sock < 0) {
		IPERF_ERR("socket() return %d\n", local_sock);
		goto socket_error;
	}

	data_buf = iperf_buf_new(IPERF_BUF_SIZE);
	if (data_buf == NULL) {
		IPERF_ERR("malloc() failed!\n");
		goto socket_error;
	}

	IPERF_DBG("iperf: UDP recv at port %d\n", port);

	uint32_t run_end_tm, run_beg_tm, beg_tm, end_tm, cur_tm;
	uint32_t data_total_cnt, data_cnt;
#if 0
	/* do calc after receiving the first packet */
	data_len = recvfrom(local_sock, data_buf, IPERF_BUF_SIZE, 0, NULL, 0);
	if (data_len <= 0) {
		IPERF_ERR("first recvfrom return %d, err %d\n", data_len, iperf_errno);
		goto socket_error;
	}
#endif
	iperf_loop_init();

	#ifdef _IPERF_SELECT
        int select_ret = 0;
	#ifdef CHECK_FAIL
	int times_delay = 0;
	#endif
	IPERF_SET_NONBLOCK_MODE(local_sock);
	#endif
	while (CHECK_IPERF_RUN_FLAG(iperf_thread_stop_flag)) {
		#ifdef _IPERF_SELECT
		fd_set FDSET;
                select_ret = IPERF_SELECT(local_sock, IPERF_SELECT_TIMEOUT, &FDSET, 0);
		#ifdef CHECK_FAIL
		CHECK_FAIL_TIMES(times_delay,select_ret);
		#endif
                CHECK_SELECT_RESULT(select_ret){
			if (FD_ISSET(local_sock, &FDSET)) {
		#endif
				data_len = recvfrom(local_sock, data_buf, IPERF_BUF_SIZE, 0, NULL, 0);
				if (data_len > 0) {
					data_cnt += data_len;
				} else {
					//IPERF_ERR("recvfrom return %d, err %d\n", data_len, iperf_errno);
				}

				iperf_calc_speed();
		#ifdef _IPERF_SELECT
			}
                }
		#endif
	}

socket_error:
	if (data_buf)
		free(data_buf);
	if (local_sock >= 0)
		closesocket(local_sock);

	IPERF_DBG("%s() exit!\n", __func__);
	iperf_thread_exit(&g_iperf_thread);
}

void iperf_tcp_send_task(void *arg)
{
	int local_sock = -1;
	struct sockaddr_in remote_addr;
	struct iperf_data *idata = (struct iperf_data *)arg;
	char *remote_ip = idata->remote_ip;
	uint32_t port = idata->port;
	uint32_t run_time = IPERF_SEC_2_INTERVAL(idata->run_time);
	uint8_t *data_buf = NULL;
	int32_t data_len;
	int ret;

//	OS_MSleep(1); // enable task create function return
	local_sock = iperf_sock_create(SOCK_STREAM, IPERF_TCP_SEND_PORT, 0);
	if (local_sock < 0) {
		IPERF_ERR("socket() return %d\n", local_sock);
		goto socket_error;
	}

	data_buf = iperf_buf_new(IPERF_BUF_SIZE);
	if (data_buf == NULL) {
		IPERF_ERR("malloc() failed!\n");
		goto socket_error;
	}

	memset(&remote_addr, 0, sizeof(struct sockaddr_in));
	remote_addr.sin_addr.s_addr = inet_addr(remote_ip);
	if (port == 0) {
		port = IPERF_TCP_RECV_PORT;
	}
	remote_addr.sin_port = htons(port);
	remote_addr.sin_family = AF_INET;
	ret = connect(local_sock, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
	if (ret < 0) {
		IPERF_ERR("connect to %s:%d return %d, err %d\n",
		          remote_ip, port, ret, iperf_errno);
		goto socket_error;
	}

	IPERF_DBG("iperf: TCP send to %s:%d\n", remote_ip, port);

	uint32_t run_end_tm, run_beg_tm, beg_tm, end_tm, cur_tm;
	uint32_t data_total_cnt, data_cnt;

	iperf_loop_init();
#ifdef _IPERF_SELECT
        int select_ret = 0;
	IPERF_SET_NONBLOCK_MODE(local_sock);
#endif
	while (CHECK_IPERF_RUN_FLAG(iperf_thread_stop_flag)) {
		#ifdef _IPERF_SELECT
		fd_set FDSET;
                select_ret = IPERF_SELECT(local_sock, IPERF_SELECT_TIMEOUT, &FDSET, 1);
                CHECK_SELECT_RESULT(select_ret){
			if (FD_ISSET(local_sock, &FDSET)) {
		#endif
				data_len = send(local_sock, data_buf, IPERF_TCP_SEND_DATA_LEN, 0);
				if (data_len > 0) {
					data_cnt += data_len;
				} else {
					IPERF_WARN("send return %d, err %d\n", data_len, iperf_errno);
					break;
				}

				iperf_calc_speed();
		#ifdef _IPERF_SELECT
			}
                }
		#endif
	}

socket_error:
	if (data_buf)
		free(data_buf);
	if (local_sock >= 0)
		closesocket(local_sock);

	IPERF_DBG("%s() exit!\n", __func__);
	iperf_thread_exit(&g_iperf_thread);
}

void iperf_tcp_recv_task(void *arg)
{
	int local_sock = -1;
	int remote_sock = -1;
	struct sockaddr_in remote_addr;
	struct iperf_data *idata = (struct iperf_data *)arg;
	uint32_t port = idata->port;
	uint32_t run_time = IPERF_SEC_2_INTERVAL(idata->run_time);
	uint8_t *data_buf = NULL;
	int32_t data_len;
	uint32_t remote_ip;
	uint16_t remote_port;
	int ret;

	if (port == 0) {
		port = IPERF_TCP_RECV_PORT;
	}
	local_sock = iperf_sock_create(SOCK_STREAM, port, 1);
	if (local_sock < 0) {
		IPERF_ERR("socket() return %d\n", local_sock);
		goto socket_error;
	}

	data_buf = iperf_buf_new(IPERF_BUF_SIZE);
	if (data_buf == NULL) {
		IPERF_ERR("malloc() failed!\n");
		goto socket_error;
	}

	IPERF_DBG("iperf: TCP listen at port %d\n", port);

	ret = listen(local_sock, 5);
	if (ret < 0) {
		IPERF_ERR("Failed to listen socket %d, err %d\n", local_sock, iperf_errno);
		goto socket_error;
	}

	ret = sizeof(struct sockaddr_in);

#ifdef _IPERF_SELECT
	fd_set FDSET;
	int select_ret = 0;
	IPERF_SET_NONBLOCK_MODE(local_sock);

	while (CHECK_IPERF_RUN_FLAG(iperf_thread_stop_flag)) {
		select_ret = IPERF_SELECT(local_sock, IPERF_SELECT_TIMEOUT, &FDSET, 0);
		CHECK_SELECT_RESULT(select_ret){
			if (FD_ISSET(local_sock, &FDSET)) {
				break;
			}
		}
	}
#endif

	remote_sock = accept(local_sock, (struct sockaddr *)&remote_addr, (socklen_t *)&ret);
	if (remote_sock < 0) {
		IPERF_ERR("Failed to accept socket, ret %d, err %d\n", remote_sock, iperf_errno);
		goto socket_error;
	}

	remote_ip = ntohl(remote_addr.sin_addr.s_addr);
	remote_port = ntohs(remote_addr.sin_port);
	IPERF_DBG("iperf: client from %u.%u.%u.%u:%d\n",
		(remote_ip) >> 24 & 0xFF, (remote_ip) >> 16 & 0xFF,
		(remote_ip) >> 8 & 0xFF, (remote_ip) & 0xFF, remote_port);

	uint32_t run_end_tm, run_beg_tm, beg_tm, end_tm, cur_tm;
	uint32_t data_total_cnt, data_cnt;

	iperf_loop_init();
	#ifdef CHECK_FAIL
	int times_delay = 0;
	#endif
	while (CHECK_IPERF_RUN_FLAG(iperf_thread_stop_flag)) {
	#ifdef _IPERF_SELECT
		fd_set FDSET;

		select_ret = IPERF_SELECT(remote_sock, IPERF_SELECT_TIMEOUT, &FDSET, 0);
		#ifdef CHECK_FAIL
		CHECK_FAIL_TIMES(times_delay,select_ret);
		#endif

		CHECK_SELECT_RESULT(select_ret) {
			if (FD_ISSET(remote_sock, &FDSET)) {
	#endif
				data_len = recv(remote_sock, data_buf, IPERF_BUF_SIZE, 0);
				if (data_len > 0) {
					data_cnt += data_len;
				} else {
					IPERF_WARN("recv return %d, err %d\n", data_len, iperf_errno);
					break;
				}
				iperf_calc_speed();
	#ifdef _IPERF_SELECT
			}
		}
	#endif
	}

socket_error:
	if (remote_sock)
		closesocket(remote_sock);
	if (data_buf)
		free(data_buf);
	if (local_sock >= 0)
		closesocket(local_sock);

	IPERF_DBG("%s() exit!\n", __func__);
	iperf_thread_exit(&g_iperf_thread);
}

static const OS_ThreadEntry_t iperf_thread_entry[IPERF_MODE_NUM] = {
	iperf_udp_send_task,
	iperf_udp_recv_task,
	iperf_tcp_send_task,
	iperf_tcp_recv_task,
}; /* index by enum IPERF_MODE */

static struct iperf_data g_iperf_data;

int obtain_iperf_thread_status(void *param)
{
	if (OS_ThreadIsValid(&g_iperf_thread)) {
		IPERF_DBG("iperf task is running\n");
		return 1;
	}
	return 0;
}

int iperf_start(struct netif *nif, struct iperf_data *idata)
{
	if (OS_ThreadIsValid(&g_iperf_thread)) {
		IPERF_ERR("iperf task is running\n");
		return -1;
	}

	if (idata->mode >= IPERF_MODE_NUM) {
		IPERF_ERR("invalid iperf mode %d\n", idata->mode);
		return -1;
	}

	if (!netif_is_up(nif)) {
		IPERF_ERR("net is down, iperf start failed\n");
		return -1;
	}
	CLEAR_IPERF_STOP_FLAG(iperf_thread_stop_flag);
	memcpy(&g_iperf_data, idata, sizeof(struct iperf_data));
	if (OS_ThreadCreate(&g_iperf_thread,
	                    "",
	                    iperf_thread_entry[idata->mode],
	                    (void *)&g_iperf_data,
	                    OS_THREAD_PRIO_APP,
	                    IPERF_THREAD_STACK_SIZE) != OS_OK) {
		IPERF_ERR("iperf task create failed\n");
		return -1;
	}

	return 0;
}

int iperf_stop(void *data) {
	if (OS_ThreadIsValid(&g_iperf_thread)) {
		SET_IPERF_STOP_FLAG(iperf_thread_stop_flag);
		return 0;
	} else {
		IPERF_ERR("iperf task not exist.\n");
		return -1;
	}
}