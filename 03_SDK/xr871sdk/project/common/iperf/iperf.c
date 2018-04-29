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
#include <getopt.h>

#include "kernel/os/os_thread.h"
#include "kernel/os/os_errno.h"
#include "lwip/sockets.h"
#include "lwip/netif.h"

#include "common/cmd/cmd_util.h"
#include "iperf.h"
#include "iperf_debug.h"

#define iperf_thread_exit(thread)	OS_ThreadDelete(thread);
#define iperf_errno			OS_GetErrno()

#define IPERF_PORT					5001

#define IPERF_BUF_SIZE 				(1500)
#define IPERF_UDP_SEND_DATA_LEN		(1470)	// UDP: 1470 + 8  + 20 = 1498
#define IPERF_TCP_SEND_DATA_LEN		(1460)	// TCP: 1460 + 20 + 20 = 1500

#define IPERF_THREAD_STACK_SIZE		(2 * 1024)

iperf_arg *g_iperf_arg_handle[IPERF_ARG_HANDLE_MAX] = {NULL};

// in 1ms
#define IPERF_TIME_PER_SEC		   1000
#define IPERF_TIME()			   OS_GetTicks()
#define IPERF_SEC_2_INTERVAL(sec)  ((sec) * IPERF_TIME_PER_SEC)

#define IPERF_SELECT_TIMEOUT		100
#define WAIT_TIMES			        10

#define CLEAR_IPERF_STOP_FLAG(flg)  (flg = 0)
#define SET_IPERF_STOP_FLAG(flg)    (flg = 1)
#define CHECK_IPERF_RUN_FLAG        CHECK_FLAG

int CHECK_FLAG(int flg)
{
	if (flg)
		return 0;
	else
		return 1;
}

static __inline float iperf_speed(uint32_t time,
										uint32_t bytes,
										iperf_arg *arg)
{
	//printf("time=%d bytes=%d\n", time, bytes/1024);
	if (arg->flags & IPERF_FLAG_FORMAT)
		/* return  KBytes/sec */
		return (bytes / 1024.0 / ((float)time / IPERF_TIME_PER_SEC));
	else
		/* return Mbits/sec */
		return (bytes * 8 / 1024.0 / 1024 / ((float)time / IPERF_TIME_PER_SEC));
}


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

#define iperf_loop_init() 													\
	data_total_cnt = 0;														\
	data_cnt = 0;															\
	beg_tm = IPERF_TIME();													\
	end_tm = beg_tm + IPERF_SEC_2_INTERVAL(((iperf_arg *)arg)->interval);	\
	run_beg_tm = beg_tm;													\
	run_end_tm = run_beg_tm + run_time;

#define iperf_calc_speed()													\
	data_total_cnt += data_len; 											\
	cur_tm = IPERF_TIME();													\
	if (cur_tm > end_tm) {													\
		if (((iperf_arg *)arg)->flags & IPERF_FLAG_FORMAT) {				\
			IPERF_LOG(1, "[%d] %.1f KB/s\n", ((iperf_arg *)arg)->handle,		\
				iperf_speed(cur_tm - beg_tm, data_cnt, ((iperf_arg *)arg)));\
		} else {															\
			IPERF_LOG(1, "[%d] %.2f Mb/s\n", ((iperf_arg *)arg)->handle,		\
				iperf_speed(cur_tm - beg_tm, data_cnt, ((iperf_arg *)arg)));\
		}																	\
		data_cnt = 0;														\
		beg_tm = IPERF_TIME();												\
		end_tm = beg_tm + IPERF_SEC_2_INTERVAL(((iperf_arg *)arg)->interval);\
	}																		\
	if (cur_tm > run_end_tm && run_time) {									\
		if (((iperf_arg *)arg)->flags & IPERF_FLAG_FORMAT) {				\
			IPERF_LOG(1, "[%d] TEST END: %.1f KB/s\n",						\
					((iperf_arg *)arg)->handle, 							\
					iperf_speed(cur_tm - run_beg_tm, data_total_cnt,		\
									((iperf_arg *)arg)));					\
		} else {															\
			IPERF_LOG(1, "[%d] TEST END: %.2f Mb/s\n",						\
					((iperf_arg *)arg)->handle, 							\
					iperf_speed(cur_tm - run_beg_tm, data_total_cnt,		\
									((iperf_arg *)arg)));					\
		}																	\
		break;																\
	}

#define iperf_calc_speed_fin() 												\
		cur_tm = IPERF_TIME();												\
		data_total_cnt += data_len; 										\
		if (((iperf_arg *)arg)->flags & IPERF_FLAG_FORMAT) {				\
			IPERF_LOG(1, "[%d] TEST END: %.1f KB/s\n",						\
					((iperf_arg *)arg)->handle, 							\
					iperf_speed(cur_tm - run_beg_tm, data_total_cnt,		\
									((iperf_arg *)arg)));					\
		} else {															\
			IPERF_LOG(1, "[%d] TEST END: %.2f Mb/s\n",						\
					((iperf_arg *)arg)->handle, 							\
					iperf_speed(cur_tm - run_beg_tm, data_total_cnt,		\
									((iperf_arg *)arg)));					\
		}


/* -------------------------------------------------------------------
 * Send a datagram on the socket. The datagram's contents should signify
 * a FIN to the application. Keep re-transmitting until an
 * acknowledgement datagram is received.
 * ------------------------------------------------------------------- */

void write_UDP_FIN(int local_sock,
						struct sockaddr *remote_addr,
						uint8_t *data_buf)
{
	int rc;
	fd_set readSet;
	struct timeval timeout;
	struct UDP_datagram* mBuf_UDP = (struct UDP_datagram*) data_buf;

	int count = 0;
	int packetid;
	while (count < 10) {
		 count++;

		 // write data
		 sendto(local_sock, data_buf, IPERF_UDP_SEND_DATA_LEN, 0,
		                  remote_addr, sizeof(*remote_addr));
		// decrement the packet count
		packetid = ntohl(mBuf_UDP->id);
		packetid--;
		mBuf_UDP->id = htonl(packetid);

		// wait until the socket is readable, or our timeout expires
		FD_ZERO(&readSet);
		FD_SET(local_sock, &readSet);
		timeout.tv_sec  = 0;
		timeout.tv_usec = 250000; // quarter second, 250 ms

		rc = select(local_sock + 1, &readSet, NULL, NULL, &timeout);
		if (rc == 0) {
		 // select timed out
		 continue;
		} else {
			// socket ready to read
			rc = recvfrom(local_sock, data_buf, IPERF_BUF_SIZE, 0, NULL, 0);
			if (rc < 0)
				break;
			return;
		}
	}

	IPERF_WARN("did not receive ack of last datagram after %d tries.\n", count);
}

/* -------------------------------------------------------------------
 * Send an AckFIN (a datagram acknowledging a FIN) on the socket,
 * then select on the socket for some time. If additional datagrams
 * come in, probably our AckFIN was lost and they are re-transmitted
 * termination datagrams, so re-transmit our AckFIN.
 * ------------------------------------------------------------------- */

void write_UDP_AckFIN(int local_sock,
							struct sockaddr *remote_addr,
							uint8_t *data_buf) {
	int rc;
	fd_set readSet;
	FD_ZERO(&readSet);
	struct timeval timeout;
	int count = 0;

	while ( count < 10 ) {
	    count++;

	    // write data
		sendto(local_sock, data_buf, IPERF_UDP_SEND_DATA_LEN, 0,
		                  remote_addr, sizeof(*remote_addr));

	    // wait until the socket is readable, or our timeout expires
	    FD_SET(local_sock, &readSet);
	    timeout.tv_sec  = 1;
	    timeout.tv_usec = 0;

	    rc = select(local_sock + 1, &readSet, NULL, NULL, &timeout);
	    if ( rc == 0 ) {
	        // select timed out
	        return;
	    } else {
	        // socket ready to read
	        rc = recvfrom(local_sock, data_buf, IPERF_BUF_SIZE, 0, NULL, 0);
	        if ( rc <= 0 ) {
	            // Connection closed or errored
	            // Stop using it.
	            return;
	        }
	    }
	}

	IPERF_WARN("ack of last datagram failed after %d tries.\n", count);
}

void iperf_udp_send_task(void *arg)
{
	int local_sock = -1;
	struct sockaddr_in remote_addr;
	iperf_arg *idata = (iperf_arg *)arg;
	uint32_t ip_addr;
	uint32_t run_time = IPERF_SEC_2_INTERVAL(idata->run_time);
	uint32_t port = idata->port;
	uint8_t *data_buf = NULL;
	int32_t data_len;
	int packetID = 0;
	struct UDP_datagram* mBuf_UDP;

	local_sock = iperf_sock_create(SOCK_DGRAM, 0, 1);
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
#ifdef __CONFIG_LWIP_V1
	inet_addr_from_ipaddr(&remote_addr.sin_addr, &idata->remote_ip);
	ip_addr = ip4_addr_get_u32(&idata->remote_ip);
#elif LWIP_IPV4 /* now only for IPv4 */
	inet_addr_from_ip4addr(&remote_addr.sin_addr, ip_2_ip4(&idata->remote_ip));
	ip_addr = ip4_addr_get_u32(ip_2_ip4(&idata->remote_ip));
#else
	#error "IPv4 not support!"
#endif
	remote_addr.sin_port = htons(port);
	remote_addr.sin_family = AF_INET;

	IPERF_DBG("iperf: UDP send to %s:%d\n", inet_ntoa(ip_addr), port);

	uint32_t run_end_tm = 0, run_beg_tm = 0, beg_tm = 0, end_tm = 0, cur_tm = 0;
	uint32_t data_total_cnt = 0, data_cnt = 0;

	mBuf_UDP = (struct UDP_datagram*) data_buf;
	mBuf_UDP->id = htonl(packetID);
	iperf_loop_init();

	while (CHECK_IPERF_RUN_FLAG(idata->flags & IPERF_FLAG_STOP)) {
		data_len = sendto(local_sock, data_buf, IPERF_UDP_SEND_DATA_LEN, 0,
		                  (struct sockaddr *)&remote_addr, sizeof(remote_addr));
		if (data_len > 0) {
			data_cnt += data_len;
			packetID++;
			mBuf_UDP->id = htonl(packetID);
		} else {
			data_len = 0;
		}
		iperf_calc_speed();
	}

	mBuf_UDP->id = htonl(-packetID);
	write_UDP_FIN(local_sock, (struct sockaddr *)&remote_addr, data_buf);

socket_error:
	if (data_buf)
		free(data_buf);
	if (local_sock >= 0)
		closesocket(local_sock);

	OS_Thread_t thread = idata->iperf_thread;
	iperf_handle_free(idata->handle);
	IPERF_DBG("%s() [%d] exit!\n", __func__, idata->handle);
	iperf_thread_exit(&thread);
}

void iperf_udp_recv_task(void *arg)
{
	int local_sock = -1;
	struct sockaddr_in remote_addr;
	socklen_t addr_len = sizeof(remote_addr);
	iperf_arg *idata = (iperf_arg *)arg;
	uint32_t run_time = IPERF_SEC_2_INTERVAL(idata->run_time);
	uint32_t port = idata->port;
	uint8_t *data_buf = NULL;
	int32_t data_len;
	int timeout = IPERF_SELECT_TIMEOUT; //ms
	int packetID = 0;
	struct UDP_datagram* mBuf_UDP;
	uint8_t wait = 1;

	if (port == 0) {
		port = IPERF_PORT;
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

	uint32_t run_end_tm = 0, run_beg_tm = 0, beg_tm = 0, end_tm = 0, cur_tm = 0;
	uint32_t data_total_cnt = 0, data_cnt = 0;

	if (setsockopt(local_sock, SOL_SOCKET, SO_RCVTIMEO,
					(char *)&timeout, sizeof(timeout)) < 0) {
		IPERF_ERR("set socket option err %d\n", iperf_errno);
		goto socket_error;
	}
	mBuf_UDP = (struct UDP_datagram*)data_buf;
	iperf_loop_init();

	while (CHECK_IPERF_RUN_FLAG(idata->flags & IPERF_FLAG_STOP)) {
		data_len = recvfrom(local_sock, data_buf, IPERF_BUF_SIZE, 0,
							(struct sockaddr *)&remote_addr, &addr_len);
		if (data_len > 0) {
			if (wait) {
				wait = 0;
				iperf_loop_init(); // reinitialization time
				IPERF_DBG("iperf udp_recv_task reinit time and reclocking\n");
			}
			data_cnt += data_len;
		} else {
			data_len = 0;
		}

		packetID = ntohl(mBuf_UDP->id);
		if (packetID < 0) {
			iperf_calc_speed_fin();
			IPERF_DBG("iperf udp_recv_task receive a FIN datagram\n");
			write_UDP_AckFIN(local_sock, (struct sockaddr *)&remote_addr,
								data_buf);
			break;
		}
		iperf_calc_speed();
	}

socket_error:
	if (data_buf)
		free(data_buf);
	if (local_sock >= 0)
		closesocket(local_sock);

	OS_Thread_t thread = idata->iperf_thread;
	iperf_handle_free(idata->handle);
	IPERF_DBG("%s() [%d] exit!\n", __func__, idata->handle);
	iperf_thread_exit(&thread);
}

void iperf_tcp_send_task(void *arg)
{
	int local_sock = -1;
	struct sockaddr_in remote_addr;
	iperf_arg *idata = (iperf_arg *)arg;
	uint32_t ip_addr;
	uint32_t run_time = IPERF_SEC_2_INTERVAL(idata->run_time);
	uint32_t port = idata->port;
	uint8_t *data_buf = NULL;
	int32_t data_len;
	int ret;

//	OS_MSleep(1); // enable task create function return
	local_sock = iperf_sock_create(SOCK_STREAM, 0, 0);
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
#ifdef __CONFIG_LWIP_V1
	inet_addr_from_ipaddr(&remote_addr.sin_addr, &idata->remote_ip);
	ip_addr = ip4_addr_get_u32(&idata->remote_ip);
#elif LWIP_IPV4 /* now only for IPv4 */
	inet_addr_from_ip4addr(&remote_addr.sin_addr, ip_2_ip4(&idata->remote_ip));
	ip_addr = ip4_addr_get_u32(ip_2_ip4(&idata->remote_ip));
#else
	#error "IPv4 not support!"
#endif
	remote_addr.sin_port = htons(port);
	remote_addr.sin_family = AF_INET;
	ret = connect(local_sock, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
	if (ret < 0) {
		IPERF_ERR("connect to %s:%d return %d, err %d\n",
		          inet_ntoa(ip_addr), port, ret, iperf_errno);
		goto socket_error;
	}

	IPERF_DBG("iperf: TCP send to %s:%d\n", inet_ntoa(ip_addr), port);

	uint32_t run_end_tm = 0, run_beg_tm = 0, beg_tm = 0, end_tm = 0, cur_tm = 0;
	uint32_t data_total_cnt = 0, data_cnt = 0;

	iperf_loop_init();

	while (CHECK_IPERF_RUN_FLAG(idata->flags & IPERF_FLAG_STOP)) {
		data_len = send(local_sock, data_buf, IPERF_TCP_SEND_DATA_LEN, 0);
		if (data_len > 0) {
			data_cnt += data_len;
		} else {
			IPERF_WARN("send return %d, err %d\n", data_len, iperf_errno);
			break;
		}
		iperf_calc_speed();
	}

socket_error:
	if (data_buf)
		free(data_buf);
	if (local_sock >= 0)
		closesocket(local_sock);

	OS_Thread_t thread = idata->iperf_thread;
	iperf_handle_free(idata->handle);
	IPERF_DBG("%s() [%d] exit!\n", __func__, idata->handle);
	iperf_thread_exit(&thread);
}

void iperf_tcp_recv_task(void *arg)
{
	int local_sock = -1;
	int remote_sock = -1;
	struct sockaddr_in remote_addr;
	iperf_arg *idata = (iperf_arg *)arg;
	uint32_t port = idata->port;
	uint32_t run_time = IPERF_SEC_2_INTERVAL(idata->run_time);
	uint8_t *data_buf = NULL;
	int32_t data_len;
	uint32_t remote_ip;
	uint16_t remote_port;
	int ret;
	int timeout = IPERF_SELECT_TIMEOUT; //ms

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

	/* Set the recv timeout to set the accept timeout */
	if (setsockopt(local_sock, SOL_SOCKET, SO_RCVTIMEO,
					(char *)&timeout, sizeof(timeout)) < 0) {
		IPERF_ERR("set socket option err %d\n", iperf_errno);
		goto socket_error;
	}

	while (CHECK_IPERF_RUN_FLAG(idata->flags & IPERF_FLAG_STOP)) {
		remote_sock = accept(local_sock, (struct sockaddr *)&remote_addr,
							(socklen_t *)&ret);
		if (remote_sock > 0)
			break;
	}

	if (CHECK_IPERF_RUN_FLAG(idata->flags & IPERF_FLAG_STOP)) {
		remote_ip = ntohl(remote_addr.sin_addr.s_addr);
		remote_port = ntohs(remote_addr.sin_port);
		IPERF_DBG("iperf: client from %u.%u.%u.%u:%d\n",
			(remote_ip) >> 24 & 0xFF, (remote_ip) >> 16 & 0xFF,
			(remote_ip) >> 8 & 0xFF, (remote_ip) & 0xFF, remote_port);
	}

	uint32_t run_end_tm = 0, run_beg_tm = 0, beg_tm = 0, end_tm = 0, cur_tm = 0;
	uint32_t data_total_cnt = 0, data_cnt = 0;

	iperf_loop_init();

	while (CHECK_IPERF_RUN_FLAG(idata->flags & IPERF_FLAG_STOP)) {
		data_len = recv(remote_sock, data_buf, IPERF_BUF_SIZE, 0);
		if (data_len > 0) {
			data_cnt += data_len;
		} else {
			iperf_calc_speed_fin();
			IPERF_WARN("recv return %d, err %d\n", data_len, iperf_errno);
			break;
		}
		iperf_calc_speed();
	}

socket_error:
	if (remote_sock)
		closesocket(remote_sock);
	if (data_buf)
		free(data_buf);
	if (local_sock >= 0)
		closesocket(local_sock);

	OS_Thread_t thread = idata->iperf_thread;
	iperf_handle_free(idata->handle);
	IPERF_DBG("%s() [%d] exit!\n", __func__, idata->handle);
	iperf_thread_exit(&thread);
}

static const OS_ThreadEntry_t iperf_thread_entry[IPERF_MODE_NUM] = {
	iperf_udp_send_task,
	iperf_udp_recv_task,
	iperf_tcp_send_task,
	iperf_tcp_recv_task,
}; /* index by enum IPERF_MODE */

int iperf_handle_start(struct netif *nif, int handle)
{
	if (nif == NULL || handle < 0 || handle >= IPERF_ARG_HANDLE_MAX)
		return -1;

	if (OS_ThreadIsValid(&(g_iperf_arg_handle[handle]->iperf_thread))) {
		IPERF_ERR("iperf task is running\n");
		return -1;
	}

	g_iperf_arg_handle[handle]->flags &= ~IPERF_FLAG_STOP; /* clean stop flag */
	if (OS_ThreadCreate(&(g_iperf_arg_handle[handle]->iperf_thread),
							"iperf",
							iperf_thread_entry[g_iperf_arg_handle[handle]->mode],
							(void *)g_iperf_arg_handle[handle],
							OS_THREAD_PRIO_APP,
							IPERF_THREAD_STACK_SIZE) != OS_OK) {
			IPERF_ERR("iperf task create failed\n");
			return -1;
	}
	return 0;
}

int iperf_handle_stop(int handle)
{
	int i = 0;

	if (handle < 0)
		return -1;

	if ((handle < IPERF_ARG_HANDLE_MAX) && (g_iperf_arg_handle[handle] != NULL)) {
		if (OS_ThreadIsValid(&(g_iperf_arg_handle[handle]->iperf_thread))) {
			g_iperf_arg_handle[handle]->flags |= IPERF_FLAG_STOP;
		} else {
			IPERF_ERR("iperf task %d not exist.\n", handle);
			return -1;
		}
	} else if (handle >= IPERF_ARG_HANDLE_MAX) {
		for (i = 0; i < IPERF_ARG_HANDLE_MAX; i++) {
			if (g_iperf_arg_handle[i] != NULL) {
				iperf_handle_stop(i);
			}
		}
	}
	return 0;
}

int iperf_start(struct netif *nif, int handle)
{
	return iperf_handle_start(nif, handle);
}

int iperf_stop(char* arg)
{
	int handle = 0;
	if (cmd_strcmp("a", arg) == 0)
		return iperf_handle_stop(IPERF_ARG_HANDLE_MAX);
	handle = cmd_atoi(arg);
	if (handle < 0 || handle >= IPERF_ARG_HANDLE_MAX ||
		(handle == 0 && cmd_strcmp("0", arg))) {
		IPERF_DBG("handle '%s' input err\n", arg);
		return -1;
	}
	return iperf_handle_stop(handle);
}

int iperf_handle_new(iperf_arg* arg)
{
	int i = 0, pos = 0, find = 0;
	if (arg == NULL)
		return -1;

	for (i = 0; i < IPERF_ARG_HANDLE_MAX; i++) {
		if (g_iperf_arg_handle[i] == NULL) {
			if (!find) {
				find = 1;
				pos = i;
				if ((arg->mode & IPERF_MODE_TCP_SEND) ||
					(arg->mode & IPERF_MODE_UDP_SEND))
					break;
			}
		} else {
			if ((arg->mode & IPERF_MODE_TCP_RECV) ||
				(arg->mode & IPERF_MODE_UDP_RECV)) {
				if(g_iperf_arg_handle[i]->port == arg->port) {
					IPERF_DBG("the port %d has been used\n", arg->port);
					return -1;
				}
			}
		}
	}

	if (find) {
		g_iperf_arg_handle[pos] = malloc(sizeof(iperf_arg));
		if (g_iperf_arg_handle[pos] == NULL) {
			IPERF_ERR("iperf malloc faild\n");
			return -1;
		}
		cmd_memcpy(g_iperf_arg_handle[pos], arg, sizeof(iperf_arg));
		g_iperf_arg_handle[pos]->handle = pos;
		return pos;
	}

	IPERF_DBG("cannot new more handle, buf is full\n");
	return -1;
}

int iperf_handle_free(int handle)
{
	if (handle >= IPERF_ARG_HANDLE_MAX)
		return -1;

	if (g_iperf_arg_handle[handle] != NULL) {
		free(g_iperf_arg_handle[handle]);
		g_iperf_arg_handle[handle] = NULL;
	}
	return 0;
}

static const char *iperf_mode_str[IPERF_MODE_NUM] = {
	"udp-send",
	"udp-recv",
	"tcp-send",
	"tcp-recv",
};

int iperf_show_list(void) {
	int i = 0;
	int run_num = 0;
	uint32_t ip_addr;

	for (i = 0; i < IPERF_ARG_HANDLE_MAX; i++) {
		if (g_iperf_arg_handle[i] != NULL) {
			run_num++;
#ifdef __CONFIG_LWIP_V1
			ip_addr = ip4_addr_get_u32(&g_iperf_arg_handle[i]->remote_ip);
#elif LWIP_IPV4 /* now only for IPv4 */
			ip_addr = ip4_addr_get_u32(ip_2_ip4(&g_iperf_arg_handle[i]->remote_ip));
#else
			#error "IPv4 not support!"
#endif
			IPERF_LOG(1, "\nhandle    = %d\n", i);
			IPERF_LOG(1, "mode      = %s\n",
						iperf_mode_str[g_iperf_arg_handle[i]->mode]);
			IPERF_LOG(1, "remote ip = %s\n", inet_ntoa(ip_addr));
			IPERF_LOG(1, "port      = %u\n", g_iperf_arg_handle[i]->port);
			IPERF_LOG(1, "run time  = %u\n", g_iperf_arg_handle[i]->run_time);
			IPERF_LOG(1, "interval  = %u\n\n", g_iperf_arg_handle[i]->interval);
		}
	}
	if (!run_num)
		return -1;
	return 0;
}

int iperf_parse_argv(int argc, char *argv[])
{
	iperf_arg iperf_arg_t;
	uint32_t ip_addr;
	uint32_t port;
	int opt = 0;
	char *short_opts = "LusS:c:f:p:t:i:";
	cmd_memset(&iperf_arg_t, 0, sizeof(iperf_arg_t));

	optind = 0; /* reset the index */
	//opterr = 0; /* close the "invalid option" warning */
	while ((opt = getopt(argc, argv, short_opts)) != -1) {
		//CMD_DBG("opt=%c\n", opt);
		switch (opt) {
			case 'f':
				if (cmd_strcmp("m", optarg) == 0 )
					iperf_arg_t.flags &= ~IPERF_FLAG_FORMAT;
				else if (cmd_strcmp("K", optarg) == 0)
					iperf_arg_t.flags |= IPERF_FLAG_FORMAT;
				else {
					IPERF_ERR("invalid format arg '%s'\n", optarg);
					return -1;
				}
				break;
			case 'p':
				port = (uint32_t)cmd_atoi(optarg);
				if (port > 65535 || port == 0) {
					IPERF_ERR("invalid port arg '%s'\n", optarg);
					return -1;
				} else {
					iperf_arg_t.port = port;
					iperf_arg_t.flags |= IPERF_FLAG_PORT;
				}
				break;
			case 'u':
				iperf_arg_t.flags |= IPERF_FLAG_UDP;
				break;
			case 's':
				iperf_arg_t.flags |= IPERF_FLAG_SERVER;
				break;
			case 'c':
				ip_addr = inet_addr(optarg);
				if (ip_addr == IPADDR_NONE) {
					IPERF_ERR("invalid ip arg '%s'\n", optarg);
					return -1;
				} else {
#ifdef __CONFIG_LWIP_V1
					ip4_addr_set_u32(&iperf_arg_t.remote_ip, ip_addr);
#elif LWIP_IPV4 /* now only for IPv4 */
					ip_addr_set_ip4_u32(&iperf_arg_t.remote_ip, ip_addr);
#else
					#error "IPv4 not support!"
#endif
					iperf_arg_t.flags |= IPERF_FLAG_CLINET;
				}
				break;
			case 'i':
				iperf_arg_t.interval = (uint32_t)cmd_atoi(optarg);
				break;
			case 't':
				iperf_arg_t.run_time = (uint32_t)cmd_atoi(optarg);
				break;
			case 'S':
				iperf_stop(optarg);
				return -2;
				break;
			case 'L':
				if(iperf_show_list())
					IPERF_DBG("no task running\n");
				return -2;
				break;
			default :
				return -1;
				break;
		}
	}

	/* set the default interval time 1 second */
	if (!iperf_arg_t.interval)
		iperf_arg_t.interval =1;

	/* Cannot be client and server simultaneously */
	uint32_t flag = IPERF_FLAG_SERVER | IPERF_FLAG_CLINET;
	if ((iperf_arg_t.flags & flag) == 0 || (~iperf_arg_t.flags & flag) == 0) {
		IPERF_ERR("invalid iperf cmd\n");
		return -1;
	}

	if (iperf_arg_t.port == 0)
		iperf_arg_t.port = IPERF_PORT;

	if ((iperf_arg_t.flags & IPERF_FLAG_UDP) == IPERF_FLAG_UDP) {
		if ((iperf_arg_t.flags & IPERF_FLAG_SERVER) == IPERF_FLAG_SERVER)
			iperf_arg_t.mode = IPERF_MODE_UDP_RECV;
		else
			iperf_arg_t.mode = IPERF_MODE_UDP_SEND;
	} else {
		if ((iperf_arg_t.flags & IPERF_FLAG_SERVER) == IPERF_FLAG_SERVER)
			iperf_arg_t.mode = IPERF_MODE_TCP_RECV;
		else
			iperf_arg_t.mode = IPERF_MODE_TCP_SEND;
	}

	int handle = iperf_handle_new(&iperf_arg_t);
	return handle;
}

