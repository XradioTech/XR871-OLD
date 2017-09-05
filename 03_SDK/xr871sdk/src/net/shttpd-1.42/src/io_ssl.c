/*
 * Copyright (c) 2004-2005 Sergey Lyubka <valenok@gmail.com>
 * All rights reserved
 *
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Sergey Lyubka wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 */

#include "defs.h"

#if defined(SHTTPD_SSL)
static mbedtls_sock g_shttpd_net_fd = {.fd = -1};

SSL_CTX*
shttpd_ssl_wrapper_new()
{
	return (SSL_CTX*) mbedtls_init_context(1);
}

void
shttpd_ssl_wrapper_free(SSL_CTX *ssl_context)
{
	if (!ssl_context)
		return ;
	mbedtls_deinit_context(ssl_context);
}

int
shttpd_ssl_wrapper_read(SSL_CTX *ssl_context, char *buf, int length)
{
	if (!ssl_context)
		return -1;
	return mbedtls_recv(ssl_context, buf, length);
}

int
shttpd_ssl_wrapper_write(SSL_CTX *ssl_context, char *buf, int length)
{
	if (!ssl_context)
		return -1;
	return mbedtls_send(ssl_context, buf, length);
}

int
shttpd_ssl_wrapper_config(SSL_CTX *ssl_context, void *param)
{
	if (!ssl_context || !param)
		return -1;
	return mbedtls_config_context(ssl_context, param, MBEDTLS_SSL_SERVER_VERIFY_LEVEL);
}

int
shttpd_ssl_wrapper_negotiation(SSL_CTX *ssl_context, int fd)
{
	if (!ssl_context)
		return -1;
	g_shttpd_net_fd.fd = fd;

	return mbedtls_handshake(ssl_context, &g_shttpd_net_fd);
}


void
_shttpd_ssl_handshake(struct stream *stream)
{
	int n;
	assert(stream->chan.ssl.ssl != NULL);
	if ((n = SSL_WRAPPER_HANDSHAKE(stream->chan.ssl.ssl, stream->chan.ssl.sock)) == 0) {
		DBG(("handshake: SSL accepted"));
		stream->flags |= FLAG_SSL_ACCEPTED;
		return;
	} else {
		stream->flags |= FLAG_CLOSED;
		DBG(("SSL_accept error %d", n));
	}
}

static int
read_ssl(struct stream *stream, void *buf, size_t len)
{
	int nread = -1;

	assert(stream->chan.ssl.ssl != NULL);
	if (!(stream->flags & FLAG_SSL_ACCEPTED))
		_shttpd_ssl_handshake(stream);

	if (stream->flags & FLAG_SSL_ACCEPTED)
		nread = SSL_WRAPPER_READ(stream->chan.ssl.ssl, buf, len);

	return (nread);
}

static int
write_ssl(struct stream *stream, const void *buf, size_t len)
{
	assert(stream->chan.ssl.ssl != NULL);
	return (SSL_WRAPPER_WRITE(stream->chan.ssl.ssl, (char *)buf, len));
}

static void
close_ssl(struct stream *stream)
{
	assert(stream->chan.ssl.sock != -1);
	assert(stream->chan.ssl.ssl != NULL);
	SSL_WRAPPER_FREE(stream->chan.ssl.ssl);
	(void) closesocket(stream->chan.ssl.sock);
	g_shttpd_net_fd.fd = -1;
	stream->chan.ssl.ssl = NULL;
	stream->conn->ctx->ssl_ctx = NULL;
}

const struct io_class	_shttpd_io_ssl =  {
	"ssl",
	read_ssl,
	write_ssl,
	close_ssl
};
#endif /* SHTTPD_SSL */
