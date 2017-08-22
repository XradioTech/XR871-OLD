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

#include "net/mbedtls/debug.h"
#include "net/mbedtls/x509_crt.h"
#include "net/mbedtls/entropy.h"
#include "net/mbedtls/ctr_drbg.h"
#include "net/mbedtls/ssl.h"
#include "net/mbedtls/net.h"
#include "lwip/sockets.h"

/**
 * Server certificate(CA/CRL/KEY) container
 */
typedef struct {
	char           *pCa;    /* ca pointer   */
	unsigned int   nCa;     /* ca length    */
	char           *pCert;  /* cert pointer */
	unsigned int   nCert;   /* cert length  */
	char           *pKey;   /* key pointer  */
	unsigned int   nKey;    /* key length   */
} security_server;

/**
 * Client certificate(CA) container
 */
typedef struct {
	char            *pCa;  /* ca pointer */
	unsigned int    nCa;   /* ca length  */
	security_server certs;
} security_client;

/**
 * Container for certificate and Public key container.
 */
typedef union {
	struct {
		mbedtls_x509_crt   ca;       /* CA used for verify server crt */

		mbedtls_x509_crt   cert;     /* crt for oneself */
		mbedtls_pk_context key;      /* key for oneself */
	} cli_cert;

	struct {
		mbedtls_x509_crt   cert;
		mbedtls_pk_context key;      /* Public key container */
	} srv_cert;
} crt_context;

/**
 * mbedtls wrapper context structure
 *
 * The structure ensures that mbedtls api works properly and is dynamically created by
 * api (mbedtls_init_context). It contains all the info needed in the tls process.
 */
typedef struct
{
	int                       is_client;
	crt_context               cert;
	mbedtls_entropy_context   entropy;
	mbedtls_ctr_drbg_context  ctr_drbg;
	mbedtls_ssl_context       ssl;
	mbedtls_ssl_config        conf;
} mbedtls_context;

typedef mbedtls_net_context mbedtls_sock;

#if defined (MBEDTLS_SSL_CLI_C)
#define MBEDTLS_CLIENT
#endif

#if defined (MBEDTLS_SSL_SRV_C)
#define MBEDTLS_SERVER
#endif

#define MBEDTLS_SSL_CLIENT_VERIFY_LEVEL         MBEDTLS_SSL_VERIFY_OPTIONAL
#define MBEDTLS_SSL_SERVER_VERIFY_LEVEL         MBEDTLS_SSL_VERIFY_NONE

mbedtls_sock* mbedtls_socket(int nonblock);

mbedtls_context* mbedtls_init_context(int client);

void mbedtls_deinit_context(mbedtls_context *context);

int mbedtls_closesocket(mbedtls_sock* fd);

int mbedtls_config_context(mbedtls_context *context, void *param, int verify);

int mbedtls_handshake(mbedtls_context *context, mbedtls_sock* fd);

int mbedtls_send(mbedtls_context *context,char *buf, int len);

int mbedtls_recv(mbedtls_context *context, char *buf, int len);

int mbedtls_recv_pending(mbedtls_context *context);

int mbedtls_connect(mbedtls_context *context, mbedtls_sock* fd, struct sockaddr *name, int namelen, char *hostname);

int mbedtls_accept(mbedtls_context *context, mbedtls_sock *local_fd, mbedtls_sock *remote_fd);