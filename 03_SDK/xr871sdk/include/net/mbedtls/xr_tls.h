#ifndef XR_TLS_H_H
#define XR_TLS_H_H

#include "kernel/os/os_time.h"
#include "kernel/os/os_thread.h"
#include "kernel/os/FreeRTOS/os_common.h"

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL	5
#endif

#ifndef DEFALT_TEST_TIME
#define DEFALT_TEST_TIME	20000 //ms
#endif

#ifndef TLS_TEST_BUF_SIZE
#define TLS_TEST_BUF_SIZE	4096
#endif

#ifndef TLS_GET_TIME
#define TLS_GET_TIME	OS_GetTicks
#endif

#ifndef TLS_THREAD_STACK_SIZE
#define TLS_THREAD_STACK_SIZE		(4 * 1024)
#endif

#define SERVER_PORT	"443"
#define GET_REQUEST	"GET / HTTP/1.1\r\nHost: %s\r\nUser-Agent: allwinnertech\r\nConnection: Close\r\n\r\n"
#define CLIENT_DATA	"1234567890QWERTYUIOPASDFGHJKLZXCVBNM"

#define HTTP_RESPONSE \
        "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n" \
	"<h2>mbed TLS Test Server</h2>\r\n" \
	"<p>Successful connection using: %s</p>\r\n"

typedef struct
{
        char server_port[10];
        char server_name[30];
        unsigned int continue_ms;
        unsigned int flags;
}
mbedtls_test_param;

typedef enum
{
        MBEDTLS_SSL_FLAG_SERVER_PORT = 0x1,
        MBEDTLS_SSL_FLAG_SERVER_NAME = 0x2,
        MBEDTLS_SSL_FLAG_CONTINUE = 0x4,
        MBEDTLS_SSL_FLAG_CLINET = 0x8,
        MBEDTLS_SSL_FLAG_SERVER = 0x10,
        MBEDTLS_SSL_FLAG_WEBSERVER = 0x20,
        MBEDTLS_SSL_FLAG_WEBCLIENT = 0x40,
}
mbedtls_test_flags;

#define TLS_THREAD_EXIT	OS_ThreadDelete

#define	FREE_BUF(P)	\
        if (P != NULL) { \
                free(P);\
                P = NULL;}

extern int tls_start(mbedtls_test_param *param);
#endif /* XR_TLS_H_H */
