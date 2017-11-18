/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *
 *******************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "MQTTXrRTOS.h"
#include "MQTTDebug.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "errno.h"

#ifdef XR_MQTT_PLATFORM_UTEST
static unsigned int tick;

#ifdef OS_GetTicks
#undef OS_GetTicks
#endif /* OS_GetTicks */
#define OS_GetTicks() (tick)

#endif /* XR_MQTT_PLATFORM_UTEST */

/** countdown_ms - set timeout value in mil seconds
 * @param timer - timeout timer where the timeout value save
 * @param timeout_ms - timeout in timeout_ms mil seconds
 */
void countdown_ms(Timer* timer, unsigned int timeout_ms)
{
	timer->end_time = OS_TicksToMSecs(OS_GetTicks()) + timeout_ms;
}

/** countdown - set timeout value in seconds
 * @param timer - timeout timer where the timeout value save
 * @param timeout - timeout in timeout seconds
 */
void countdown(Timer* timer, unsigned int timeout)
{
	countdown_ms(timer, timeout * 1000);
}

/** left_ms - calculate how much time left before timeout
 * @param timer - timeout timer
 * @return the time left befor timeout, or 0 if it has expired
 */
int left_ms(Timer* timer)
{
	int diff = (int)(timer->end_time) - (int)(OS_TicksToMSecs(OS_GetTicks()));
	return (diff < 0) ? 0 : diff;
}

/** expired - has it already timeouted
 * @param timer - timeout timer
 * @return 0 if it has already timeout, or otherwise.
 */
char expired(Timer* timer)
{
	return 0 <= (int)OS_TicksToMSecs(OS_GetTicks()) - (int)(timer->end_time); /* is time_now over than end time */
}

/** InitTimer - initialize the timer
 * @param timer - timeout timer
 */
void InitTimer(Timer* timer)
{
	timer->end_time = 0;
}

#ifdef PACKET_SPLICE_SIMULATE
static int pkt_splice_force = 100;
#endif

/** xr_rtos_read - read data from network with TCP/IP based on xr_rtos platform
 * @param n - the network has been connected
 * @param buffer - where the data will buffer in
 * @param len - the data length hoped to receive
 * @param timeout_ms - timeouted value to abandon this reading
 * @return the read size, or 0 if timeouted, or -1 if network has been disconnected,
 * @       or -2 if error occured.
 */
static int xr_rtos_read(Network* n, unsigned char *buffer, int len, int timeout_ms)
{
	/* it's a bug fixed version which may cause a blocking even it has been timeouted */
	int recvLen = 0;
	int leftms;
	int rc = -1;
	struct timeval tv;
	Timer timer;
	fd_set fdset;

	MQTT_PLATFORM_ENTRY();

	countdown_ms(&timer, timeout_ms);

#ifdef PACKET_SPLICE_SIMULATE
	if ((pkt_splice_force-- < 0) && (len != 1)) {
		pkt_splice_force = 300;

		leftms = left_ms(&timer);
		tv.tv_sec = leftms / 1000;
		tv.tv_usec = (leftms % 1000) * 1000;

		FD_ZERO(&fdset);
		FD_SET(n->my_socket, &fdset);

		rc = select(n->my_socket + 1, &fdset, NULL, NULL, &tv);
		if (rc > 0) {
			rc = recv(n->my_socket, buffer + recvLen, (len - recvLen) / 2, 0);
			if (rc > 0) {
				/* received normally */
				recvLen += rc;
			} else if (rc == 0) {
				/* has disconnected with server */
				recvLen = -1;
			} else {
				/* network error */
				MQTT_PLATFORM_WARN("recv return %d, errno = %d\n", rc, errno);
				recvLen = -2;
			}
		} else if (rc == 0) {
			if (recvLen != 0)
				MQTT_PLATFORM_WARN("received timeout and length had received is %d\n", recvLen);
			/* timeouted and return the length received */
		} else {
			/* network error */
			MQTT_PLATFORM_WARN("select return %d, errno = %d\n", rc, errno);
					recvLen = -2;
				}
	} else
#endif
	do {
		leftms = left_ms(&timer);
		tv.tv_sec = leftms / 1000;
		tv.tv_usec = (leftms % 1000) * 1000;

		FD_ZERO(&fdset);
		FD_SET(n->my_socket, &fdset);

		rc = select(n->my_socket + 1, &fdset, NULL, NULL, &tv);
		if (rc > 0) {
			rc = recv(n->my_socket, buffer + recvLen, len - recvLen, 0);
			if (rc > 0) {
				/* received normally */
				recvLen += rc;
			} else if (rc == 0) {
				/* has disconnected with server */
				recvLen = -1;
				break;
			} else {
				/* network error */
				MQTT_PLATFORM_WARN("recv return %d, errno = %d\n", rc, errno);
				recvLen = -2;
				break;
			}
		} else if (rc == 0) {
			if (recvLen != 0)
				MQTT_PLATFORM_WARN("received timeout and length had received is %d\n", recvLen);
			/* timeouted and return the length received */
			break;
		} else {
			/* network error */
			MQTT_PLATFORM_WARN("select return %d, errno = %d\n", rc, errno);
			recvLen = -2;
			break;
		}
	} while (recvLen < len && !expired(&timer)); /* expired() is redundant? */

	MQTT_PLATFORM_EXIT(recvLen);

	return recvLen;
}

/** xr_rtos_write - write data throught TCP/IP network based on xr_rtos platform
 * @param n - the network has been connected
 * @param buffer - data which need to be written out
 * @param len - the data length hoped to send
 * @param timeout_ms - timeouted value to abandon this writing
 * @return the writed size, or 0 if timeouted, or -1 if network has been disconnected,
 * @       or -2 if error occured.
 */
static int xr_rtos_write(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	int rc = -1;
	int sentLen = 0;
	fd_set fdset;
	struct timeval tv;

	MQTT_PLATFORM_ENTRY();

	FD_ZERO(&fdset);
	FD_SET(n->my_socket, &fdset);

	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	rc = select(n->my_socket + 1, NULL, &fdset, NULL, &tv);
	if (rc > 0) {
		if ((rc = send(n->my_socket, buffer, len, 0)) > 0)
			sentLen = rc;
		else if (rc == 0)
			sentLen = -1; /* disconnected with server */
		else {
			MQTT_PLATFORM_WARN("send return %d, errno = %d\n", rc, errno);
			sentLen = -2; /* network error */
		}
	} else if (rc == 0)
		sentLen = 0; /* timeouted and sent 0 bytes */
	 else {
	 	MQTT_PLATFORM_WARN("select return %d, errno = %d\n", rc, errno);
		sentLen = -2; /* network error */
	 }

	MQTT_PLATFORM_EXIT(sentLen);

	return sentLen;
}

/** xr_rtos_disconnect - disconnect the nectwork
 * @param n - the network has been connected
 */
static void xr_rtos_disconnect(Network* n)
{
	closesocket(n->my_socket);
}

/** NewNetwork - initialize the network
 * @param n - the network hoped to be connected
 */
void NewNetwork(Network* n)
{
	n->my_socket = 0;
	n->mqttread = xr_rtos_read;
	n->mqttwrite = xr_rtos_write;
	n->disconnect = xr_rtos_disconnect;
}

/** ConnectNetwork - connect the network with destination
 * @param n - the network need to be connected
 * @param addr - the host name
 * @param port - the TCP port
 */
int ConnectNetwork(Network* n, char* addr, int port)
{
	int type = SOCK_STREAM;
	int family = AF_INET;
	struct addrinfo hints = {0, family, type, IPPROTO_TCP, 0, NULL, NULL, NULL};

	int rc = -1;
	struct sockaddr_in address;
	struct addrinfo *result = NULL;

	if ((rc = getaddrinfo(addr, NULL, &hints, &result)) == 0) {
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
			address.sin_port = htons(port);
			address.sin_family = family;
			address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
		}
		else
			rc = -1;

		freeaddrinfo(result);
	}

	if (rc == 0) {
		n->my_socket = socket(family, type, 0);
		if (n->my_socket < 0)
			return -2;

		rc = connect(n->my_socket, (struct sockaddr *)&address, sizeof(address));
		if (rc < 0) {
			MQTT_PLATFORM_WARN("lwip_connect failed, error code = %d\n", errno);
			closesocket(n->my_socket);
			return -3;
		}
	}

	return rc;
}


#if 0
int NetworkConnectTLS(Network *n, char* addr, int port, SlSockSecureFiles_t* certificates, unsigned char sec_method, unsigned int cipher, char server_verify)
{
	SlSockAddrIn_t sAddr;
	int addrSize;
	int retVal;
	unsigned long ipAddress;

	retVal = sl_NetAppDnsGetHostByName(addr, strlen(addr), &ipAddress, AF_INET);
	if (retVal < 0) {
		return -1;
	}

	sAddr.sin_family = AF_INET;
	sAddr.sin_port = sl_Htons((unsigned short)port);
	sAddr.sin_addr.s_addr = sl_Htonl(ipAddress);

	addrSize = sizeof(SlSockAddrIn_t);

	n->my_socket = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, SL_SEC_SOCKET);
	if (n->my_socket < 0) {
		return -1;
	}

	SlSockSecureMethod method;
	method.secureMethod = sec_method;
	retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECMETHOD, &method, sizeof(method));
	if (retVal < 0) {
		return retVal;
	}

	SlSockSecureMask mask;
	mask.secureMask = cipher;
	retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &mask, sizeof(mask));
	if (retVal < 0) {
		return retVal;
	}

	if (certificates != NULL) {
		retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_FILES, certificates->secureFiles, sizeof(SlSockSecureFiles_t));
		if (retVal < 0)
		{
			return retVal;
		}
	}

	retVal = sl_Connect(n->my_socket, (SlSockAddr_t *)&sAddr, addrSize);
	if (retVal < 0) {
		if (server_verify || retVal != -453) {
			sl_Close(n->my_socket);
			return retVal;
		}
	}

	SysTickIntRegister(SysTickIntHandler);
	SysTickPeriodSet(80000);
	SysTickEnable();

	return retVal;
}
#endif


int mqtt_random()
{
    return OS_Rand32();//(((unsigned int)rand() << 16) + rand());
}

static int mqtt_ssl_random(void *p_rng, unsigned char *output, size_t output_len)
{
    unsigned int rnglen = output_len;
    unsigned char rngoffset = 0;

    while (rnglen > 0) {
        *(output + rngoffset) = (unsigned char)mqtt_random() ;
        rngoffset++;
        rnglen--;
    }
    return 0;
}

static void mqtt_ssl_debug( void *ctx, int level, const char *file, int line, const char *str )
{
    printf("%s\n", str);
}

int mqtt_real_confirm(int verify_result)
{
#define VERIFY_ITEM(Result, Item, ErrMsg) \
    do { \
        if (((Result) & (Item)) != 0) { \
            printf(ErrMsg); \
        } \
    } while (0)

    printf("  certificate verification result: 0x%02x", verify_result);
    VERIFY_ITEM(verify_result, MBEDTLS_X509_BADCERT_EXPIRED, "! fail ! server certificate has expired\n");
    VERIFY_ITEM(verify_result, MBEDTLS_X509_BADCERT_REVOKED, "! fail ! server certificate has been revoked\n");
    VERIFY_ITEM(verify_result, MBEDTLS_X509_BADCERT_CN_MISMATCH, "! fail ! CN mismatch\n");
    VERIFY_ITEM(verify_result, MBEDTLS_X509_BADCERT_NOT_TRUSTED, "! fail ! self-signed or not signed by a trusted CA\n");

    return 0;
}

static int ssl_parse_crt(mbedtls_x509_crt *crt)
{
    char buf[1024];
    mbedtls_x509_crt *local_crt = crt;
    int i = 0;
    while (local_crt) {
        printf("# %d\r\n", i);
        mbedtls_x509_crt_info(buf, sizeof(buf) - 1, "", local_crt);
        {
            char str[512];
            const char *start, *cur;
            start = buf;
            for (cur = buf; *cur != '\0'; cur++) {
                if (*cur == '\n') {
                    size_t len = cur - start + 1;
                    if (len > 511) {
                        len = 511;
                    }
                    memcpy(str, start, len);
                    str[len] = '\0';
                    start = cur + 1;
                    printf("%s", str);
                }
            }
        }
        printf("crt content:%d!\r\n", strlen(buf));
        local_crt = local_crt->next;
        i++;
    }
    return i;
}

//#include "net/mbedtls/debug.h"
int mqtt_ssl_client_init(mbedtls_ssl_context *ssl,
                         mbedtls_net_context *tcp_fd,
                         mbedtls_ssl_config *conf,
                         mbedtls_x509_crt *crt509_ca, const char *ca_crt, size_t ca_len,
                         mbedtls_x509_crt *crt509_cli, const char *cli_crt, size_t cli_len,
                         mbedtls_pk_context *pk_cli, const char *cli_key, size_t key_len,  const char *cli_pwd, size_t pwd_len
                        )
{
    int ret = -1;
    //verify_source_t *verify_source = &custom_config->verify_source;

    /*
     * 0. Initialize the RNG and the session data
     */
#if defined(MBEDTLS_DEBUG_C)
    //mbedtls_debug_set_threshold(5);
#endif
    mbedtls_net_init( tcp_fd );
    mbedtls_ssl_init( ssl );
    mbedtls_ssl_config_init( conf );
    mbedtls_x509_crt_init(crt509_ca);

    /*verify_source->trusted_ca_crt==NULL
     * 0. Initialize certificates
     */

    printf( "  . Loading the CA root certificate ...\n" );
    if (NULL != ca_crt) {
        if (0 != (ret = mbedtls_x509_crt_parse(crt509_ca, (const unsigned char *)ca_crt, ca_len))) {
            printf(" failed ! x509parse_crt returned -0x%04x\n", -ret);
            return ret;
        }
    }
    ssl_parse_crt(crt509_ca);
    printf( " ok (%d skipped)\n", ret );

    /* Setup Client Cert/Key */
#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if defined(MBEDTLS_CERTS_C)
    mbedtls_x509_crt_init(crt509_cli);
    mbedtls_pk_init( pk_cli );
#endif
    if ( cli_crt != NULL && cli_key != NULL) {
#if defined(MBEDTLS_CERTS_C)
        printf("start prepare client cert .\n");
        ret = mbedtls_x509_crt_parse( crt509_cli, (const unsigned char *) cli_crt, cli_len );
#else
        {
            ret = 1;
            printf("MBEDTLS_CERTS_C not defined.");
        }
#endif
        if ( ret != 0 ) {
            printf( " failed!  mbedtls_x509_crt_parse returned -0x%x\n\n", -ret );
            return ret;
        }

#if defined(MBEDTLS_CERTS_C)
        printf("start mbedtls_pk_parse_key[%s]", cli_pwd);
        ret = mbedtls_pk_parse_key( pk_cli,
                                    (const unsigned char *) cli_key, key_len,
                                    (const unsigned char *) cli_pwd, pwd_len);
#else
        {
            ret = 1;
            printf("MBEDTLS_CERTS_C not defined.");
        }
#endif

        if ( ret != 0 ) {
            printf( " failed\n  !  mbedtls_pk_parse_key returned -0x%x\n\n", -ret);
            return ret;
        }
    }
#endif /* MBEDTLS_X509_CRT_PARSE_C */

    return 0;
}

int mqtt_ssl_read(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
    size_t readLen = 0;
    int ret = -1;

    mbedtls_ssl_conf_read_timeout(&(n->conf), timeout_ms);

    while (readLen < len) {
		ret = mbedtls_ssl_read(&(n->ssl), (unsigned char *)(buffer + readLen), (len - readLen));
        if (ret > 0) {
            readLen += ret;
        }
		else if (ret == 0) {
            printf("mqtt ssl read timeout\n");
            return -2; 	//eof
        }
		else {
            if(ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) {
                printf("MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY \n");
                return -2;
            }
            return -1; 	//Connnection error
        }
    }
    //printf("mqtt_ssl_read_all readlen=%d \n", readLen);

    return readLen;
}

int mqtt_ssl_write(Network *n, unsigned char *buffer, int len, int timeout_ms)
{
    size_t writtenLen = 0;
    int ret = -1;

    while (writtenLen < len) {
        ret = mbedtls_ssl_write(&(n->ssl), (unsigned char *)(buffer + writtenLen), (len - writtenLen));
        if (ret > 0) {
            writtenLen += ret;
            continue;
        }
		else if (ret == 0) {
            printf("mqtt ssl write timeout \n");
            return writtenLen;
        }
		else {
            printf("mqtt ssl write fail \n");
            return -1; 	//Connnection error
        }
    }
    //printf("mqtt ssl write len = %d \n", writtenLen);
    return writtenLen;
}

void mqtt_ssl_disconnect(Network *n)
{
    mbedtls_ssl_close_notify(&(n->ssl));
    mbedtls_net_free(&(n->fd));

#if defined(MBEDTLS_X509_CRT_PARSE_C)
    mbedtls_x509_crt_free( &(n->cacertl));
    if ((n->pkey).pk_info != NULL) {
        printf("mqtt need free client crt&key");
        mbedtls_x509_crt_free( &(n->clicert));
        mbedtls_pk_free( &(n->pkey) );
    }
#endif

    mbedtls_ssl_free( &(n->ssl));
    mbedtls_ssl_config_free(&(n->conf));
    printf( "mqtt_ssl_disconnect\n" );
}

int TLSConnectNetwork(Network *n, const char *addr, const char *port,
                      const char *ca_crt, size_t ca_crt_len,
                      const char *client_crt,	size_t client_crt_len,
                      const char *client_key,	size_t client_key_len,
                      const char *client_pwd, size_t client_pwd_len)
{
    int ret = -1;
    /*
     * 0. Init
     */
    if (0 != (ret = mqtt_ssl_client_init(&(n->ssl), &(n->fd), &(n->conf),
                                         &(n->cacertl), ca_crt, ca_crt_len,
                                         &(n->clicert), client_crt, client_crt_len,
                                         &(n->pkey), client_key, client_key_len, client_pwd, client_pwd_len))) {
        printf(" failed ! ssl_client_init returned -0x%04x", -ret );
        return ret;
    }

    /*
     * 1. Start the connection
     */
    printf("  . Connecting to tcp/%s/%s...", addr, port);
    if (0 != (ret = mbedtls_net_connect(&(n->fd), addr, port, MBEDTLS_NET_PROTO_TCP))) {
        printf(" failed ! net_connect returned -0x%04x", -ret);
        return ret;
    }
    printf( " ok\n" );

    /*
     * 2. Setup stuff
     */
    printf( "  . Setting up the SSL/TLS structure..." );
    if ( (ret = mbedtls_ssl_config_defaults( &(n->conf),
                 MBEDTLS_SSL_IS_CLIENT,
                 MBEDTLS_SSL_TRANSPORT_STREAM,
                 MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 ) {
        printf( " failed! mbedtls_ssl_config_defaults returned %d", ret );
        return ret;
    }
    printf( " ok\n" );

	mbedtls_ssl_conf_max_version(&n->conf, MBEDTLS_SSL_MAJOR_VERSION_3, MBEDTLS_SSL_MINOR_VERSION_3);
	mbedtls_ssl_conf_min_version(&n->conf, MBEDTLS_SSL_MAJOR_VERSION_3, MBEDTLS_SSL_MINOR_VERSION_3);

    /* OPTIONAL is not optimal for security,
         * but makes interop easier in this simplified example */
    if (ca_crt != NULL) {
        mbedtls_ssl_conf_authmode( &(n->conf), MBEDTLS_SSL_VERIFY_OPTIONAL );
    } else {
        mbedtls_ssl_conf_authmode( &(n->conf), MBEDTLS_SSL_VERIFY_NONE);
    }

#if defined(MBEDTLS_X509_CRT_PARSE_C)
    mbedtls_ssl_conf_ca_chain( &(n->conf), &(n->cacertl), NULL);

    if ( (ret = mbedtls_ssl_conf_own_cert( &(n->conf), &(n->clicert), &(n->pkey) ) ) != 0 ) {
        printf( " failed\n  ! mbedtls_ssl_conf_own_cert returned %d\n\n", ret );
        return ret;
    }
#endif
    mbedtls_ssl_conf_rng( &(n->conf), mqtt_ssl_random, NULL);
    mbedtls_ssl_conf_dbg( &(n->conf), mqtt_ssl_debug, NULL);

    if (( ret = mbedtls_ssl_setup(&(n->ssl), &(n->conf)) ) != 0 ) {
        printf( " failed! mbedtls_ssl_setup returned %d", ret );
        return ret;
    }
    mbedtls_ssl_set_hostname(&(n->ssl), addr);
    mbedtls_ssl_set_bio( &(n->ssl), &(n->fd), mbedtls_net_send, mbedtls_net_recv, mbedtls_net_recv_timeout);

    /*
      * 4. Handshake
      */
    printf("  . Performing the SSL/TLS handshake...");
    while ((ret = mbedtls_ssl_handshake(&(n->ssl))) != 0) {
        if ((ret != MBEDTLS_ERR_SSL_WANT_READ) && (ret != MBEDTLS_ERR_SSL_WANT_WRITE)) {
            printf( " failed  ! mbedtls_ssl_handshake returned -0x%04x", -ret);
            return ret;
        }
    }
    printf( " ok\n" );
    /*
     * 5. Verify the server certificate
     */
    printf("  . Verifying peer X.509 certificate..\n");
    if (0 != (ret = mqtt_real_confirm(mbedtls_ssl_get_verify_result(&(n->ssl))))) {
        printf(" failed  ! verify result not confirmed.\n");
        return ret;
    }
    n->my_socket = (int)((n->fd).fd);
    printf("  . my_socket = %d \n\n", n->my_socket);

    n->mqttread = mqtt_ssl_read;
    n->mqttwrite = mqtt_ssl_write;
    n->disconnect = mqtt_ssl_disconnect;

    return 0;
}

int mqtt_ssl_establish(Network *n, const char *addr, const char *port, const char *ca_crt, size_t ca_crt_len)
{
	return TLSConnectNetwork(n, addr, port, ca_crt, ca_crt_len, NULL, 0, NULL, 0, NULL, 0);
}

