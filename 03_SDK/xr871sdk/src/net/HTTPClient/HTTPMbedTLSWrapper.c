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

#include "net/mbedtls/mbedtls.h"
#include "net/HTTPClient/HTTPMbedTLSWrapper.h"

#ifdef HTTPC_SSL

//#define HTTP_CLIENT_CA
//#define HTTPC_CERTIFICATE

#if defined(HTTP_CLIENT_CA)
#define CUSTOM_HTTPC_CRT_RSA                                            \
"-----BEGIN CERTIFICATE-----\r\n"                                       \
"MIICtDCCAZwCAQAwDQYJKoZIhvcNAQELBQAwIDEeMBwGA1UEAxMVQ2VydGlmaWNh\r\n"  \
"dGUgQXV0aG9yaXR5MB4XDTE3MDcxOTExMzU1NVoXDTIyMDcxODExMzU1NVowIDEe\r\n"  \
"MBwGA1UEAxMVQ2VydGlmaWNhdGUgQXV0aG9yaXR5MIIBIjANBgkqhkiG9w0BAQEF\r\n"  \
"AAOCAQ8AMIIBCgKCAQEA3FPtvNnMiETM5qprK4625nf8z39HnM5pdkyjW3a4JWt4\r\n"  \
"RTuLv6x7b56OnJttZPL0hYyVwGsxt3LeuoXD3pBlLn61iwUK6Fb4NX5Xo04LaKl7\r\n"  \
"gQFQ+lENPPSu/JdzcERly0d1JYIQ4Z11732yCdblf5oZJ/0skUzhTVCusWnYvY3Z\r\n"  \
"xs/O3wjvbuCucxgZoyDv6AZ8ZQ0xKZ3JKjm8URD4yrRYEkQ+gBeTkNC3nVFJ/u8X\r\n"  \
"nrNT/qzhhI6HS8Lf88dkQu3W5gvIVVy5qv49hqpWGXwbEkrIsMgC9OJCZ5qoo17Y\r\n"  \
"iUj/SBH6xVA9ikaFOlVeH9rD1euzwgjIm+X2Wu7S5wIDAQABMA0GCSqGSIb3DQEB\r\n"  \
"CwUAA4IBAQCt9EYWA2vTVJmEajB87MzHHvjTV/cTWRGLKnLoBHL3OKf+Lembmu6Q\r\n"  \
"YxoN8OCqRjrcvh0sgQ1H6jpfmVSIoLKoT9BVy16t5PZ8x0XSSrMlXQKz+pAuOZBc\r\n"  \
"sinSoRPDMr0M92m2CAnJ8mIpr6o0lTtWJfY1xeuT2+LbFzvaI6dtWnQYmN1mxPqA\r\n"  \
"JLhAlQhCqmRiDhgFPfKSwKsmEPdUtrA2InOsgxDGa0utawK2BWgQc6hkhT9uZ4dF\r\n"  \
"DxLkNG8w4QFnHXcm8pdpg5zbO7GSrtPRs2CU2fMpE79CMPKSH3ErxV2F4aPtHFP5\r\n"  \
"sV1Ai+iyFoUKCzjW4iUDTJux2gUTzpmH\r\n"                                  \
"-----END CERTIFICATE-----\r\n"

/* Concatenation of all available CA certificates */
const char httpc_custom_cas_pem[] = CUSTOM_HTTPC_CRT_RSA;
const size_t httpc_custom_cas_pem_len = sizeof(httpc_custom_cas_pem);

#define HTTPC_CUSTOM_CAS_PEM          httpc_custom_cas_pem
#define HTTPC_CUSTOM_CAS_PEM_LEN      httpc_custom_cas_pem_len

#if defined(HTTPC_CERTIFICATE)
#define HTTPC_CUSTOM_CA_PEM
#define HTTPC_CUSTOM_CA_PEM_LEN
#define HTTPC_CUSTOM_CRT_PEM
#define HTTPC_CUSTOM_CRT_PEM_LEN
#define HTTPC_CUSTOM_KEY
#define HTTPC_CUSTOM_KEY_LEN
#endif

#else
extern const char mbedtls_test_cas_pem[];
extern const size_t mbedtls_test_cas_pem_len;

#define HTTPC_CUSTOM_CAS_PEM          mbedtls_test_cas_pem
#define HTTPC_CUSTOM_CAS_PEM_LEN      mbedtls_test_cas_pem_len

#if defined(HTTPC_CERTIFICATE)
extern const char *mbedtls_test_srv_key;
extern const size_t mbedtls_test_srv_key_len;
extern const char *mbedtls_test_srv_crt;
extern const size_t mbedtls_test_srv_crt_len;

#define HTTPC_CUSTOM_CA_PEM           mbedtls_test_cas_pem
#define HTTPC_CUSTOM_CA_PEM_LEN       mbedtls_test_cas_pem_len
#define HTTPC_CUSTOM_CRT_PEM          mbedtls_test_srv_crt
#define HTTPC_CUSTOM_CRT_PEM_LEN      mbedtls_test_srv_crt_len
#define HTTPC_CUSTOM_KEY              mbedtls_test_srv_key
#define HTTPC_CUSTOM_KEY_LEN          mbedtls_test_srv_key_len
#endif
#endif

static security_client client_param;
mbedtls_context *g_pContext = NULL;
mbedtls_sock g_httpc_net_fd = {.fd = -1};

int HTTPWrapperSSLConnect(int s,const struct sockaddr *name,int namelen,char *hostname)
{
	int ret = 0;
	HC_DBG(("Https:connect.."));
	struct sockaddr *ServerAddress = (struct sockaddr *)name;
	int net_fd = s;
	/* Init client context */
	mbedtls_context *pContext = (mbedtls_context *)mbedtls_init_context(0);
	if (!pContext || !ServerAddress)
		return -1;
	g_pContext = pContext;

	memset(&client_param, 0, sizeof(client_param));

	security_client *user_cert = NULL;
	if ((user_cert = HTTPC_obtain_user_certs()) == NULL) {
		HC_DBG(("https: config defaults certs.."));
		client_param.pCa = (char *)HTTPC_CUSTOM_CAS_PEM;
		client_param.nCa = HTTPC_CUSTOM_CAS_PEM_LEN;
#if defined(HTTPC_CERTIFICATE)
		client_param.certs.pCa = (char *) HTTPC_CUSTOM_CAS_PEM;
		client_param.certs.nCa = HTTPC_CUSTOM_CAS_PEM_LEN;
		client_param.certs.pCert = (char *) HTTPC_CUSTOM_CRT_PEM;
		client_param.certs.nCert = HTTPC_CUSTOM_CRT_PEM_LEN;
		client_param.certs.pKey = (char *) HTTPC_CUSTOM_KEY;
		client_param.certs.nKey = HTTPC_CUSTOM_KEY_LEN;
#endif
	} else {
		HC_DBG(("https: config user certs.."));
		memcpy(&client_param, user_cert, sizeof(client_param));
	}

	int verify_mode = HTTPC_get_ssl_verify_mode();
	if (verify_mode != MBEDTLS_SSL_VERIFY_NONE && verify_mode != MBEDTLS_SSL_VERIFY_OPTIONAL &&
		verify_mode != MBEDTLS_SSL_VERIFY_REQUIRED && verify_mode != MBEDTLS_SSL_VERIFY_UNSET)
		verify_mode = MBEDTLS_SSL_VERIFY_NONE;
	if ((ret = mbedtls_config_context(pContext, (void *) &client_param, verify_mode)) != 0) {
		HC_ERR(("https: config failed.."));
		return -1;
	}

	if ((ret = mbedtls_connect(pContext, (mbedtls_sock*) &net_fd, ServerAddress, namelen, hostname)) != 0) {
		HC_ERR(("https: connect failed.."));
		return -1;
	}
	HC_DBG(("Https:connect ok.."));

	return ret;
}

int HTTPWrapperSSLNegotiate(int s,const struct sockaddr *name,int namelen,char *hostname)
{
	int ret = 0;
	g_httpc_net_fd.fd = s;
	HC_DBG(("Https:negotiate.."));
	if ((ret = mbedtls_handshake(g_pContext, &g_httpc_net_fd)) != 0)
		return -1;
	HC_DBG(("Https:negotiate ok.."));
	return 0;
}

int HTTPWrapperSSLSend(int s,char *buf, int len,int flags)
{
	int ret = 0;
	HC_DBG(("Https:send.."));
	if ((ret = mbedtls_send(g_pContext, buf, len)) < 0)
		return -1;
	return ret;
}

int HTTPWrapperSSLRecv(int s,char *buf, int len,int flags)
{
	int ret = 0;
	HC_DBG(("Https:recv.."));
	if ((ret = mbedtls_recv(g_pContext, buf, len)) < 0)
		return -1;
	return ret;
}

int HTTPWrapperSSLRecvPending(int s)
{
	int ret = 0;
	ret = mbedtls_recv_pending(g_pContext);
	HC_DBG(("Https:recv pending : %d (bytes)..", ret));
	return ret;
}

int HTTPWrapperSSLClose(int s)
{
	HC_DBG(("Https:close.."));
	mbedtls_deinit_context(g_pContext);
	s = -1;
	g_httpc_net_fd.fd = -1;
	return 0;
}
#endif /* HTTPC_SSL */