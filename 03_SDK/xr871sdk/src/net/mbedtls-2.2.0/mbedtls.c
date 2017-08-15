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
#include <stdlib.h>
#include <string.h>
#include "mbedtls/mbedtls.h"

#define MBEDTLS_API_DEBUG
#if defined(MBEDTLS_API_DEBUG)
#define DEBUG_PREFIX             "[mbedtls] "
#define MBEDTLS_DEBUG_ERR        0
#define MBEDTLS_DEBUG_INF        1
#define MBEDTLS_DEBUG_LEVEL      0

#define mbedtls_err(msg...) \
	do{ \
		if (MBEDTLS_DEBUG_LEVEL >= MBEDTLS_DEBUG_ERR) { \
			printf(DEBUG_PREFIX "err %s:%04d:", __FILE__, __LINE__); \
			printf(msg); \
		} \
	}while(0)

#define mbedtls_inf(msg...) \
	do{ \
		if (MBEDTLS_DEBUG_LEVEL >= MBEDTLS_DEBUG_INF) { \
			printf(DEBUG_PREFIX "%s:%04d:", __FILE__, __LINE__); \
			printf(msg); \
		} \
	}while(0)

#define mbedtls_dbg(level, msg...) mbedtls_##level(msg)
#else
#define mbedtls_dbg(level, msg...)
#endif

#define TLS_DEBUG_LEVEL                  0
#define MBEDTLS_SSL_CLIENT_VERIFY_LEVEL         MBEDTLS_SSL_VERIFY_OPTIONAL
#define MBEDTLS_SSL_SERVER_VERIFY_LEVEL         MBEDTLS_SSL_VERIFY_NONE
const char *pers = "custom tls";

static void mbedtls_debug(void *ctx, int level,const char *file,
                                 int line, const char *str)
{
	((void) level);
	fprintf((FILE *) ctx, "%s:%04d: %s", file, line, str);
	fflush((FILE *) ctx);
}
/**
 * Create and Initializes the mbedtls context
 * client: create server(1) or client(0)
 *
 */
mbedtls_context* mbedtls_init_context(int client)
{
	mbedtls_context *pContext = NULL;
	if ((pContext = malloc(sizeof(*pContext))) == NULL) {
		mbedtls_dbg(err, "Malloc mem failed.\n");
		return (void *)NULL;
	}

	mbedtls_dbg(inf, "Init tls context.\n");
	memset(pContext, 0, sizeof(*pContext));

	pContext->is_client = client;
#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT) {
		mbedtls_net_init(&(pContext->net_fd.cli_fd));
	}
#endif
#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER) {
		mbedtls_net_init(&(pContext->net_fd.srv_fd.fd0));
		mbedtls_net_init(&(pContext->net_fd.srv_fd.fd1));
	}
#endif
	mbedtls_ssl_init(&(pContext->ssl));
	mbedtls_ssl_config_init(&(pContext->conf));
#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT) {
		mbedtls_x509_crt_init(&(pContext->cert.cli_cert.ca));
		mbedtls_x509_crt_init(&(pContext->cert.cli_cert.cert));
		mbedtls_pk_init(&(pContext->cert.cli_cert.key));
	}
#endif
#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER) {
		mbedtls_x509_crt_init(&(pContext->cert.srv_cert.cert));
		mbedtls_pk_init(&(pContext->cert.srv_cert.key));
	}
#endif
	mbedtls_ctr_drbg_init(&(pContext->ctr_drbg));
	mbedtls_entropy_init(&(pContext->entropy));
	mbedtls_dbg(inf, "Init ok.\n");

#if defined(MBEDTLS_DEBUG_C)
	mbedtls_debug_set_threshold(TLS_DEBUG_LEVEL);
#endif
	return pContext;
}

int mbedtls_config_context(mbedtls_context *context, void *param)
{
	int ret = 0;
#if defined(MBEDTLS_SSL_CLI_C)
	security_client *client = NULL;
#endif

#if defined(MBEDTLS_SSL_SRV_C)
	security_server *server = NULL;
#endif
	mbedtls_dbg(inf, "Config start..\n");
	if (context == NULL || param == NULL)
		return -1;
	//secure = param;
	mbedtls_context *pContext = (mbedtls_context *)context;

#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT)
		client = (security_client *)param;
#endif
#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER)
		server = (security_server *)param;
#endif

#if defined(MBEDTLS_SSL_CLI_C)
	/* Load the certificates and private RSA key */
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT) {
		if ((ret = mbedtls_x509_crt_parse(&(pContext->cert.cli_cert.ca),
		                                    (const unsigned char *)(client->pCa),
		                                    client->nCa)) != 0) {
			mbedtls_dbg(err, "mbedtls_x509_crt_parse failed..%d\n", ret);
			return -1;
		}

		if (client->certs != NULL) {
			/* Tls client parse own crl*/
			if ((ret = mbedtls_x509_crt_parse(&(pContext->cert.cli_cert.cert),
		                                            (const unsigned char *)(client->certs->pCert),
		                                            client->certs->nCert)) != 0) {
				mbedtls_dbg(err, "mbedtls_x509_crt_parse failed.. %d\n", ret);
				return -1;
			}
			if ((ret = mbedtls_x509_crt_parse(&(pContext->cert.cli_cert.ca),
		                                           (const unsigned char *)(client->certs->pCa),
		                                            client->certs->nCa)) != 0) {
				mbedtls_dbg(err, "mbedtls_x509_crt_parse failed.. %d\n", ret);
				return -1;
			}
			if ((ret = mbedtls_pk_parse_key(&(pContext->cert.cli_cert.key),
		                                          (const unsigned char *)(client->certs->pKey),
		                                          client->certs->nKey, NULL, 0)) != 0) {
				mbedtls_dbg(err, "mbedtls_pk_parse_key failed.. %d", ret);
				return -1;
			}
		}

	}
#endif

#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER) {
		if ((ret = mbedtls_x509_crt_parse(&(pContext->cert.srv_cert.cert),
		                                    (const unsigned char *)(server->pCert),
		                                    server->nCert)) != 0) {
			mbedtls_dbg(err, "mbedtls_x509_crt_parse failed.. %d\n", ret);
			return -1;
		}
		if ((ret = mbedtls_x509_crt_parse(&(pContext->cert.srv_cert.cert),
		                                    (const unsigned char *)(server->pCa),
		                                    server->nCa)) != 0) {
			mbedtls_dbg(err, "mbedtls_x509_crt_parse failed.. %d\n", ret);
			return -1;
		}

		if ((ret = mbedtls_pk_parse_key(&(pContext->cert.srv_cert.key),
		                                  (const unsigned char *)(server->pKey),
		                                  server->nKey, NULL, 0)) != 0) {
			mbedtls_dbg(err, "mbedtls_pk_parse_key failed.. %d", ret);
			return -1;
		}
	}
#endif
	if ((ret = mbedtls_ctr_drbg_seed(&(pContext->ctr_drbg), mbedtls_entropy_func,
		                          &(pContext->entropy),
		                          (const unsigned char *) pers,
		                          strlen(pers))) != 0) {
		mbedtls_dbg(err, "mbedtls_ctr_drbg_seed failed.. %d", ret);
		return -1;
	}

#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT) {
		if ((ret = mbedtls_ssl_config_defaults(&(pContext->conf), MBEDTLS_SSL_IS_CLIENT,
		                                        MBEDTLS_SSL_TRANSPORT_STREAM,
		                                        MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
			return -1;
	}
#endif

#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER) {
		if ((ret = mbedtls_ssl_config_defaults(&(pContext->conf), MBEDTLS_SSL_IS_SERVER,
		                                        MBEDTLS_SSL_TRANSPORT_STREAM,
		                                        MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
			return -1;
	}
#endif

#if defined(MBEDTLS_SSL_CLI_C)
	/* set auth mode*/
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT)
		mbedtls_ssl_conf_authmode(&(pContext->conf), MBEDTLS_SSL_CLIENT_VERIFY_LEVEL);
#endif

#if defined(MBEDTLS_SSL_SRV_C)
	/* set auth mode*/
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER)
		mbedtls_ssl_conf_authmode(&(pContext->conf), MBEDTLS_SSL_SERVER_VERIFY_LEVEL);
#endif

#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT) {
		mbedtls_ssl_conf_ca_chain(&(pContext->conf), &(pContext->cert.cli_cert.ca), NULL);
		if (client->certs != NULL) {
			if ((ret = mbedtls_ssl_conf_own_cert(&(pContext->conf), &(pContext->cert.cli_cert.cert),
			                                      &(pContext->cert.cli_cert.key))) != 0) {
				mbedtls_dbg(err, "mbedtls_ssl_conf_own_cert failed %d\n", ret);
				return -1;
			}
		}
	}
#endif

#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER) {
		mbedtls_ssl_conf_ca_chain(&(pContext->conf), pContext->cert.srv_cert.cert.next, NULL);
		if ((ret = mbedtls_ssl_conf_own_cert(&(pContext->conf), &(pContext->cert.srv_cert.cert),
		                                      &(pContext->cert.srv_cert.key))) != 0) {
			mbedtls_dbg(err, "mbedtls_ssl_conf_own_cert failed %d\n", ret);
			return -1;
		}
	}
#endif
	mbedtls_ssl_conf_rng(&(pContext->conf), mbedtls_ctr_drbg_random, &(pContext->ctr_drbg));
	mbedtls_ssl_conf_dbg(&(pContext->conf), mbedtls_debug, stdout );

#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT) {
		mbedtls_ssl_conf_max_frag_len(&(pContext->conf), MBEDTLS_SSL_MAX_FRAG_LEN_4096);
	}
#endif

	if ((ret = mbedtls_ssl_setup( &(pContext->ssl), &(pContext->conf))) != 0) {
		mbedtls_dbg(err, "mbedtls_ssl_setup failed..\n");
		return -1;
	}
	mbedtls_dbg(inf, "Config ok..\n");
	return 0;
}

#if defined(MBEDTLS_SSL_CLI_C)
static int mbedtls_get_noblock(mbedtls_net_context *ctx);
int mbedtls_connect(mbedtls_context *context, struct sockaddr *name,int namelen,char *hostname)
{
	int socket =0, ret = 0;
	int is_noblock = 0;

	mbedtls_dbg(inf, "Connect start..\n");

	struct sockaddr *ServerAddress = (struct sockaddr *)name;
	mbedtls_context *pContext = (mbedtls_context *)context;
	mbedtls_net_context *net_fd = &(pContext->net_fd.cli_fd);
	if (!pContext || net_fd->fd < 0) {
		mbedtls_dbg(err, "Connect invalid arg..\n");
		return -1;
	}
	if (hostname != NULL) {
		if ((ret = mbedtls_ssl_set_hostname(&(pContext->ssl), hostname)) != 0)
		{
			mbedtls_dbg(err, "mbedtls_ssl_set_hostname returned %d\n", ret );
			return -1;
		}
	}
	if ((is_noblock = mbedtls_get_noblock(net_fd)) == 1)
		mbedtls_net_set_block(net_fd);
	if ((ret = connect(socket, ServerAddress, namelen)) != 0) {
		mbedtls_dbg(err, "tls connect failed\n");
		return ret;
	}
	if (is_noblock == 1)
		mbedtls_net_set_nonblock(net_fd);
	mbedtls_dbg(inf, "Connect ok..\n");
	return ret;
}

static int mbedtls_get_noblock(mbedtls_net_context *ctx)
{
	if (ctx == NULL) {
		mbedtls_dbg(err, "Ctx is NULL.\n");
		return -1;
	}
	if ((lwip_fcntl(ctx->fd, F_GETFL, 0) & O_NONBLOCK) != O_NONBLOCK)
		return( 0 );
	return 1;
}

#endif

#if defined(MBEDTLS_SSL_SRV_C)
int mbedtls_accept(mbedtls_context *context)
{
	int ret = 0;
	mbedtls_context *pContext = (mbedtls_context *)context;
	mbedtls_net_context *client = &(pContext->net_fd.srv_fd.fd1);
	mbedtls_net_context *server = &(pContext->net_fd.srv_fd.fd0);

	mbedtls_net_free(&(pContext->net_fd.srv_fd.fd1));
	mbedtls_ssl_session_reset(&(pContext->ssl));
	/* Wait until a client connects */
	mbedtls_dbg(inf, "Waiting for a remote connection.\n");
	if ((ret = mbedtls_net_accept(server, client, NULL, 0, NULL)) != 0) {
		mbedtls_dbg(err, "Failed mbedtls_net_accept returned %d\n", ret);
		return -1;
	}
	return ret;
}
#endif

void mbedtls_deinit_context(mbedtls_context *context)
{
	mbedtls_dbg(inf, "Deinit context.\n");
	mbedtls_context *pContext = (mbedtls_context *)context;

	mbedtls_ssl_close_notify(&(pContext->ssl));

#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT) {
		mbedtls_net_free(&(pContext->net_fd.cli_fd));

		mbedtls_x509_crt_free(&(pContext->cert.cli_cert.ca));
		mbedtls_x509_crt_free(&(pContext->cert.cli_cert.cert));
		mbedtls_pk_free(&(pContext->cert.cli_cert.key));
	}
#endif

#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER) {
		mbedtls_net_free(&(pContext->net_fd.srv_fd.fd0));
		mbedtls_net_free(&(pContext->net_fd.srv_fd.fd1));
		mbedtls_x509_crt_free(&(pContext->cert.srv_cert.cert));
		mbedtls_pk_free(&(pContext->cert.srv_cert.key));
	}
#endif
	mbedtls_ssl_free(&(pContext->ssl));
	mbedtls_ssl_config_free(&(pContext->conf));
	mbedtls_ctr_drbg_free(&(pContext->ctr_drbg));
	mbedtls_entropy_free(&(pContext->entropy));

	free(pContext);
}

int mbedtls_handshake(mbedtls_context *context)
{
	int ret = 0;
	mbedtls_net_context *net_fd = NULL;
	mbedtls_context *pContext = (mbedtls_context *)context;

#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT)
		net_fd = &(pContext->net_fd.cli_fd);
#endif

#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER)
		net_fd = &(pContext->net_fd.srv_fd.fd1);
#endif

	mbedtls_ssl_set_bio(&(pContext->ssl), net_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

	while ((ret = mbedtls_ssl_handshake(&(pContext->ssl))) != 0) {
		if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE ) {
			mbedtls_dbg(err, "mbedtls_ssl_handshake failed.(%d)\n", ret);
			goto exit;
		}
	}
	/* In real life, we probably want to bail out when ret != 0 */
	if ((ret = mbedtls_ssl_get_verify_result(&(pContext->ssl))) != 0) {
		#define MBEDTLS_VRFY_BUF_SIZE 512
		char *vrfy_buf = malloc(MBEDTLS_VRFY_BUF_SIZE);
		if (!vrfy_buf) {
			mbedtls_dbg(err, "Malloc vrfy buf failed.\n");
		} else {
			mbedtls_x509_crt_verify_info( vrfy_buf, MBEDTLS_VRFY_BUF_SIZE, "! ", ret);
			mbedtls_dbg(err, " %s\n", vrfy_buf);
			free(vrfy_buf);
		}
		if ((MBEDTLS_X509_BADCERT_NOT_TRUSTED == ret && pContext->is_client == MBEDTLS_SSL_IS_CLIENT) ||
		     (MBEDTLS_X509_BADCERT_SKIP_VERIFY == ret && pContext->is_client == MBEDTLS_SSL_IS_SERVER)) {
			/* In real life, we would have used MBEDTLS_SSL_VERIFY_REQUIRED so that the
			 * handshake would not succeed if the peer's cert is bad.  Even if we used
			 * MBEDTLS_SSL_VERIFY_OPTIONAL, we would bail out here if ret != 0 */
			ret = 0;
		} else {
			mbedtls_dbg(err, "Verify failed(%d).\n", ret);
			ret = -1;
		}
	}
	if (ret == 0) {
		mbedtls_dbg(inf, "Handshake ok(%s).\n", mbedtls_ssl_get_ciphersuite(&(pContext->ssl)));
		return 0;
	}
exit:
	return ret;
}

int mbedtls_send(mbedtls_context *context,char *buf, int len)
{
	int ret = 0;
	mbedtls_context *pContext = (mbedtls_context *)context;
	while ((ret = mbedtls_ssl_write(&(pContext->ssl), (const unsigned char *)buf, len)) <= 0) {
		if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE ) {
			mbedtls_dbg(err,"mbedtls_ssl_write failed.\n");
			goto exit;
		}
	}
	return ret;
exit:
	return -1;
}

int mbedtls_recv(mbedtls_context *context, char *buf, int len)
{
	int ret = 0;
	mbedtls_context *pContext = (mbedtls_context *)context;
	memset(buf, 0, len);

	mbedtls_dbg(inf,"recv.\n");

	do ret = mbedtls_ssl_read(&(pContext->ssl), (unsigned char *)buf, len);
	while (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE);
	if (ret < 0) {
		mbedtls_dbg(err, "mbedtls_ssl_read failed..(%#x)\n",ret);
		goto exit;;
	}
	if (ret == 0) {
		mbedtls_dbg(inf,"\nEOF\n");
		return 0;
	}
exit:
	return ret;

}

int mbedtls_recv_pending(mbedtls_context *context)
{
	int ret = 0;
	mbedtls_context *pContext = (mbedtls_context *)context;
	ret = mbedtls_ssl_get_bytes_avail(&(pContext->ssl));
	mbedtls_dbg(inf, "recv pending:%d(bytes).\n", ret);
	return ret;
}

int mbedtls_set_fd(mbedtls_context *context, int fd)
{
	if (context == NULL)
		return -1;
	mbedtls_context *pContext = (mbedtls_context *)context;

#if defined(MBEDTLS_SSL_CLI_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_CLIENT)
		pContext->net_fd.cli_fd.fd = fd;
#endif

#if defined(MBEDTLS_SSL_SRV_C)
	if (pContext->is_client == MBEDTLS_SSL_IS_SERVER)
		pContext->net_fd.srv_fd.fd1.fd = fd;
#endif
	return 0;
}
