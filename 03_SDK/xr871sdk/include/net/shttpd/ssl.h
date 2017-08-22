/*
 * Copyright (c) 2004-2005 Sergey Lyubka <valenok@gmail.com>
 * All rights reserved
 *
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Sergey Lyubka wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 */

/*
 * Snatched from OpenSSL includes. I put the prototypes here to be independent
 * from the OpenSSL source installation. Having this, shttpd + SSL can be
 * built on any system with binary SSL libraries installed.
 */
#include "net/mbedtls/mbedtls.h"
typedef mbedtls_context SSL_CTX;

SSL_CTX* shttpd_ssl_wrapper_new();
void shttpd_ssl_wrapper_free(SSL_CTX *ssl_context);
int shttpd_ssl_wrapper_read(SSL_CTX *ssl_context, char *buf, int length);
int shttpd_ssl_wrapper_write(SSL_CTX *ssl_context, char *buf, int length);
int shttpd_ssl_wrapper_config(SSL_CTX *ssl_context, void *param);
int shttpd_ssl_wrapper_negotiation(SSL_CTX *ssl_context, int fd);

#define SSL_WRAPPER_NEW                    shttpd_ssl_wrapper_new
#define SSL_WRAPPER_FREE                   shttpd_ssl_wrapper_free
#define SSL_WRAPPER_READ                   shttpd_ssl_wrapper_read
#define SSL_WRAPPER_WRITE                  shttpd_ssl_wrapper_write
#define SSL_WRAPPER_CONFIG                 shttpd_ssl_wrapper_config
#define SSL_WRAPPER_HANDSHAKE              shttpd_ssl_wrapper_negotiation

