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

#ifndef _IPERF_H_
#define _IPERF_H_

#if PRJCONF_NET_EN

#include "lwip/netif.h"
#include "lwip/inet.h"
#include "kernel/os/os_thread.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IPERF_OPT_BANDWIDTH		1	/* -b, bandwidth to send at in bits/sec */
#define IPERF_OPT_NUM			1	/* -n, number of bytes to transmit (instead of -t) */
#define IPERF_OPT_TOS			1	/* -S, the type-of-service for outgoing packets */

#define MAX_INTERVAL 60
#define IPERF_ARG_HANDLE_MAX    4

#ifndef INET_ADDRSTRLEN
#define INET_ADDRSTRLEN         16
#endif

#if (!defined(__CONFIG_LWIP_V1) && LWIP_IPV6)
#define IPERF_ADDR_STRLEN_MAX   INET6_ADDRSTRLEN
#else
#define IPERF_ADDR_STRLEN_MAX   INET_ADDRSTRLEN
#endif

enum IPERF_MODE {
	IPERF_MODE_UDP_SEND = 0,
	IPERF_MODE_UDP_RECV,
	IPERF_MODE_TCP_SEND,
	IPERF_MODE_TCP_RECV,
	IPERF_MODE_NUM,
};

enum IPERF_FLAGS {
	IPERF_FLAG_PORT    = 0x00000001,
	IPERF_FLAG_IP      = 0x00000002,
	IPERF_FLAG_CLINET  = 0x00000004,
	IPERF_FLAG_SERVER  = 0x00000008,
	IPERF_FLAG_UDP     = 0x00000010,
	IPERF_FLAG_FORMAT  = 0x00000020,
	IPERF_FLAG_STOP    = 0x00000040,
};

typedef struct {
	enum IPERF_MODE	mode;
	char		remote_ip[IPERF_ADDR_STRLEN_MAX];
	uint32_t	port;
	uint32_t	run_time; // in seconds, 0 means forever
	uint32_t	interval; // in seconds, 0 means 1 second(default)
#if IPERF_OPT_TOS
	uint8_t		tos; // type-of-service
#endif
#if IPERF_OPT_NUM
	uint8_t		mode_time; // 1: time limited, 0: byte limited
	uint64_t	amount; // in bytes (K == 1024, M == 1024 * 1024)
#endif
#if IPERF_OPT_BANDWIDTH
	uint32_t	bandwidth; // in bits/sec (k == 1000, m == 1000 * 1000)
#endif
	uint32_t	flags;
	OS_Thread_t iperf_thread;
	int 		handle;
}iperf_arg;

typedef struct UDP_datagram {
    signed int id ;
} UDP_datagram;


int iperf_start(struct netif *nif, int handle);
int iperf_stop(char* arg);
int iperf_parse_argv(int argc, char *argv[]);
int iperf_handle_free(int handle);
int iperf_handle_start(struct netif * nif, int handle);

#ifdef __cplusplus
}
#endif

#endif /* PRJCONF_NET_EN */
#endif /* _IPERF_H_ */
