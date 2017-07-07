#include "net/mbedtls/include/mbedtls/net.h"
#include "net/mbedtls/include/mbedtls/entropy.h"
#include "net/mbedtls/include/mbedtls/ctr_drbg.h"
#include "net/mbedtls/include/mbedtls/ssl.h"
#include "net/mbedtls/include/mbedtls/x509_crt.h"

#include "net/HTTPClient/HTTPMbedTLSWrapper.h"

#ifdef MBED_TLS

typedef struct
{
        mbedtls_net_context server_fd;
        mbedtls_entropy_context entropy;
        mbedtls_ctr_drbg_context ctr_drbg;
        mbedtls_ssl_context ssl;
        mbedtls_ssl_config conf;
        mbedtls_x509_crt cacert;
}
mbedtls_context;

const char *pers = "ssl_client";
mbedtls_context *g_pContext = NULL;

mbedtls_context* MBEDTLSInitContext()
{
	mbedtls_context *pContext = NULL;
	if ((pContext = malloc(sizeof(*pContext))) == NULL) {
		HC_ERR(("ALLOC mem failed..\n"));
		return (void *)NULL;
	}

	HC_DBG(("Init tls context.."));
	memset(pContext, 0, sizeof(*pContext));
        mbedtls_net_init(&(pContext->server_fd ));

        mbedtls_ssl_init(&(pContext->ssl));
        mbedtls_ssl_config_init(&(pContext->conf));
        mbedtls_x509_crt_init(&(pContext->cacert));
        mbedtls_ctr_drbg_init(&(pContext->ctr_drbg));
        mbedtls_entropy_init(&(pContext->entropy));

	return pContext;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern const char mbedtls_test_cas_pem[];
extern const size_t mbedtls_test_cas_pem_len;


static void https_debug( void *ctx, int level,
                      const char *file, int line,
                      const char *str )
{
	((void) level);

	fprintf( (FILE *) ctx, "%s:%04d: %s", file, line, str );
    	fflush(  (FILE *) ctx  );

}

int MBEDTLSConfigContext(void *param)
{
	int ret = 0;
	mbedtls_context *pContext = (mbedtls_context *)param;
	HC_DBG(("Config tls context.."));
        if ((ret = mbedtls_ctr_drbg_seed(&(pContext->ctr_drbg), mbedtls_entropy_func, &(pContext->entropy),
                                        (const unsigned char *) pers,
                                        strlen(pers))) != 0) {
                HC_ERR(("Seed rng failed.."));
        	return -1;
        }
        if ((ret = mbedtls_x509_crt_parse(&(pContext->cacert), (const unsigned char *) mbedtls_test_cas_pem,
                        mbedtls_test_cas_pem_len)) < 0) {
		HC_ERR(("Parse cert failed.."));
		return -1;
        }
        if ((ret = mbedtls_ssl_config_defaults(&(pContext->conf), MBEDTLS_SSL_IS_CLIENT,
                                        MBEDTLS_SSL_TRANSPORT_STREAM,
                                        MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
        	return -1;
        /* OPTIONAL is not optimal for security,
         * but makes interop easier in this simplified example */
        mbedtls_ssl_conf_authmode(&(pContext->conf), MBEDTLS_SSL_VERIFY_OPTIONAL);
        mbedtls_ssl_conf_ca_chain(&(pContext->conf), &(pContext->cacert), NULL);
        mbedtls_ssl_conf_rng(&(pContext->conf), mbedtls_ctr_drbg_random, &(pContext->ctr_drbg));
	mbedtls_ssl_conf_max_frag_len(&(pContext->conf), MBEDTLS_SSL_MAX_FRAG_LEN_4096);

        if ((ret = mbedtls_ssl_setup( &(pContext->ssl), &(pContext->conf))) != 0) {
		HC_ERR(("mbedtls_ssl_setup failed.."));
      		return -1;
        }
#if 0
        if( ( ret = mbedtls_ssl_set_hostname( &(pContext->ssl), server ) ) != 0 )
        {
                mbedtls_printf( " failed\n  ! mbedtls_ssl_set_hostname returned %d\n\n", ret );
                goto exit;
        }
#endif
        mbedtls_ssl_set_bio(&(pContext->ssl), &(pContext->server_fd), mbedtls_net_send, mbedtls_net_recv, NULL);
	mbedtls_ssl_conf_dbg(&(pContext->conf), https_debug, stdout );
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int MBEDTLSHandshake(void *param)
{
	int ret = 0;
	mbedtls_context *pContext = (mbedtls_context *)param;
	while ((ret = mbedtls_ssl_handshake(&(pContext->ssl))) != 0) {
                if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE ) {
			HC_ERR(("mbedtls_ssl_handshake failed.."));
                        goto exit;
                }
        }
        /* In real life, we probably want to bail out when ret != 0 */
        if ((ret = mbedtls_ssl_get_verify_result(&(pContext->ssl))) != 0) {
		int flags = 0;
                char *vrfy_buf = malloc(512);
                if (!vrfy_buf)
                        HC_ERR(("[TLS-CLI]Malloc vrfy buf failed"));
                else {
                	mbedtls_x509_crt_verify_info( vrfy_buf, sizeof( vrfy_buf ), "  ! ", flags );
                        HC_ERR(("verify failed:%s end.\n", vrfy_buf ));
                        free(vrfy_buf);
                }
		if (MBEDTLS_X509_BADCERT_NOT_TRUSTED == ret) {
			/* In real life, we would have used MBEDTLS_SSL_VERIFY_REQUIRED so that the
     			* handshake would not succeed if the peer's cert is bad.  Even if we used
     			* MBEDTLS_SSL_VERIFY_OPTIONAL, we would bail out here if ret != 0 */

			return 0;
		} else {
			HC_ERR(("verify result : %d, failed .....\n", ret ));
			return -1;
		}
        }
        else
                return 0;
exit:
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int MBEDTLSSend(void *param,char *buf, int len)
{
	int ret = 0;
	mbedtls_context *pContext = (mbedtls_context *)param;
	while ((ret = mbedtls_ssl_write(&(pContext->ssl), (const unsigned char *)buf, len )) <= 0) {
                if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE ) {
                	HC_ERR(("mbedtls_ssl_write failed"));
                        goto exit;
                }
        }
	return ret;
exit:
	return -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int MBEDTLSRecv(void *param, char *buf, int len)
{
	int ret = 0;
	mbedtls_context *pContext = (mbedtls_context *)param;
        memset(buf, 0, len);

        do ret = mbedtls_ssl_read( &(pContext->ssl), (unsigned char *)buf, len );
        while (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE);
        if (ret < 0) {
            HC_ERR(("mbedtls_ssl_read failed..(%#x)",ret));
            goto exit;;
        }
        if (ret == 0) {
            HC_DBG(("\n\nEOF\n\n"));
            return 0;
        }
	return ret;
exit:
	return -1;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int MBEDTLSDeinitContext(void *param)
{
	HC_DBG(("tls:Deinit context.."));
	mbedtls_context *pContext = (mbedtls_context *)param;
        mbedtls_ssl_close_notify(&(pContext->ssl));
        mbedtls_net_free(&(pContext->server_fd));
        mbedtls_x509_crt_free(&(pContext->cacert));
        mbedtls_ssl_free(&(pContext->ssl));
        mbedtls_ssl_config_free(&(pContext->conf));
        mbedtls_ctr_drbg_free(&(pContext->ctr_drbg));
        mbedtls_entropy_free(&(pContext->entropy));
	free(pContext);
	g_pContext = pContext = NULL;
        return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern int sktSetNonblocking( int socket , int on_off );
int HTTPWrapperSSLConnect(int s,const struct sockaddr *name,int namelen,char *hostname)
{
	int socket =0, ret = 0;
	HC_DBG(("tls:connect.."));
	struct sockaddr *ServerAddress = (struct sockaddr *)name;
	socket = s;

	mbedtls_context *pContext = (mbedtls_context *)MBEDTLSInitContext();

	if (pContext != NULL) {
		pContext->server_fd.fd = socket;
	} else
		return -1;
	g_pContext = pContext;

	sktSetNonblocking(s , 0);
	if ((ret = connect(socket, ServerAddress, namelen)) != 0) {
		HC_ERR(("tls connect failed"));
		return ret;
	}
	sktSetNonblocking(s , 1);
	HC_DBG(("tls:connect ok.."));
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int HTTPWrapperSSLNegotiate(int s,const struct sockaddr *name,int namelen,char *hostname)
{
	int ret = 0;
	HC_DBG(("tls:negotiate.."));
	if ((ret = MBEDTLSConfigContext(g_pContext)) != 0)
		return -1;
	if ((ret = MBEDTLSHandshake(g_pContext)) != 0)
		return -1;
	HC_DBG(("tls:negotiate ok.."));
        return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int HTTPWrapperSSLSend(int s,char *buf, int len,int flags)
{
	int ret = 0;
	HC_DBG(("tls:send.."));
	if ((ret = MBEDTLSSend(g_pContext, buf, len)) < 0)
		return -1;
	else
		return ret;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int HTTPWrapperSSLRecv(int s,char *buf, int len,int flags)
{
	int ret = 0;
	HC_DBG(("tls:recv.."));
	if ((ret = MBEDTLSRecv(g_pContext, buf, len)) < 0)
        	return -1;
	else
		return ret;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int HTTPWrapperSSLRecvPending(int s)
{
	int ret = 0;
	ret = mbedtls_ssl_get_bytes_avail(&(g_pContext->ssl));
	HC_DBG(("recv pending : %d (bytes)..", ret));
	return ret;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int HTTPWrapperSSLClose(int s)
{
	s = s;
	HC_DBG(("tls:close.."));
	return MBEDTLSDeinitContext(g_pContext);
}

#endif //MBED_TLS