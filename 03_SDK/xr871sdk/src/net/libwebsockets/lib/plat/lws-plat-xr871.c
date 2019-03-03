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

#include "private-libwebsockets.h"
#ifdef LWS_WITH_XRADIO
#include "kernel/os/os_thread.h"
#include "lwip/sockets.h"

int
lws_plat_socket_offset(void)
{
	return 0;
}

unsigned long long time_in_microseconds(void)
{
	return ((unsigned long long)OS_GetTicks() * 1000LL);
}

LWS_VISIBLE int
lws_get_random(struct lws_context *context, void *buf, int len)
{
	uint8_t *pb = buf;

	while (len) {
		uint32_t r = OS_Rand32();
		uint8_t *p = (uint8_t *)&r;
		int b = 4;

		if (len < b)
			b = len;

		len -= b;

		while (b--)
			*pb++ = p[b];
	}

	return pb - (uint8_t *)buf;
}

/*
 *check the socket choked or not
 */
LWS_VISIBLE int
lws_send_pipe_choked(struct lws *wsi)
{
	struct lws *wsi_eff = wsi;
	fd_set writefds;
	struct timeval tv = { 0, 0 };
#if defined(LWS_WITH_HTTP2)
	wsi_eff = lws_get_network_wsi(wsi);
#endif
	int n;

	/* treat the fact we got a truncated send pending as if we're choked */
	if (wsi_eff->trunc_len)
		return 1;

	FD_ZERO(&writefds);
	FD_SET(wsi_eff->desc.sockfd, &writefds);

	n = select(wsi_eff->desc.sockfd + 1, NULL, &writefds, NULL, &tv);
	if (n < 0)
		return 1; /* choked */

	return !n; /* n = 0 = not writable = choked */
}

LWS_VISIBLE int
lws_poll_listen_fd(struct lws_pollfd *fd)
{
	fd_set readfds;
	struct timeval tv = { 0, 0 };

	FD_ZERO(&readfds);
	FD_SET(fd->fd, &readfds);

	return select(fd->fd + 1, &readfds, NULL, NULL, &tv);
}

LWS_VISIBLE void lwsl_emit_syslog(int level, const char *line)
{
	printf("%d: %s", level, line);
}

LWS_VISIBLE LWS_EXTERN int
_lws_plat_service_tsi(struct lws_context *context, int timeout_ms, int tsi)
{
	struct lws_context_per_thread *pt;
	int n = -1, m, c;

	/* stay dead once we are dead */

	if (!context || !context->vhost_list)
		return 1;

	pt = &context->pt[tsi];
	lws_stats_atomic_bump(context, pt, LWSSTATS_C_SERVICE_ENTRY, 1);
/*
	{
		unsigned long m = lws_now_secs();

		if (m > context->time_last_state_dump) {
			context->time_last_state_dump = m;
			n = esp_get_free_heap_size();
			if (n != context->last_free_heap) {
				if (n > context->last_free_heap)
					lwsl_notice(" heap :%d (+%d)\n", n,
						    n - context->last_free_heap);
				else
					lwsl_notice(" heap :%d (-%d)\n", n,
						    context->last_free_heap - n);
				context->last_free_heap = n;
			}
		}
	}
*/
	if (timeout_ms < 0)
		goto faked_service;

	if (!context->service_tid_detected) {
		struct lws *_lws = lws_zalloc(sizeof(*_lws), "tid probe");

		_lws->context = context;

		context->service_tid_detected =
			context->vhost_list->protocols[0].callback(
			_lws, LWS_CALLBACK_GET_THREAD_ID, NULL, NULL, 0);
		context->service_tid = context->service_tid_detected;
		context->service_tid_detected = 1;
		lws_free(_lws);
	}

	/*
	 * is there anybody with pending stuff that needs service forcing?
	 */
	if (!lws_service_adjust_timeout(context, 1, tsi)) {
		/* -1 timeout means just do forced service */
		_lws_plat_service_tsi(context, -1, pt->tid);
		/* still somebody left who wants forced service? */
		if (!lws_service_adjust_timeout(context, 1, pt->tid))
			/* yes... come back again quickly */
			timeout_ms = 0;
	}

//	n = poll(pt->fds, pt->fds_count, timeout_ms);
	{
		fd_set readfds, writefds, errfds;
		struct timeval tv = { timeout_ms / 1000,
				      (timeout_ms % 1000) * 1000 }, *ptv = &tv;
		int max_fd = 0;
		FD_ZERO(&readfds);
		FD_ZERO(&writefds);
		FD_ZERO(&errfds);

		for (n = 0; n < pt->fds_count; n++) {
			pt->fds[n].revents = 0;
			if (pt->fds[n].fd >= max_fd)
				max_fd = pt->fds[n].fd;
			if (pt->fds[n].events & LWS_POLLIN)
				FD_SET(pt->fds[n].fd, &readfds);
			if (pt->fds[n].events & LWS_POLLOUT)
				FD_SET(pt->fds[n].fd, &writefds);
			FD_SET(pt->fds[n].fd, &errfds);
		}

		n = select(max_fd + 1, &readfds, &writefds, &errfds, ptv);
		n = 0;
		for (m = 0; m < pt->fds_count; m++) {
			c = 0;
			if (FD_ISSET(pt->fds[m].fd, &readfds)) {
				pt->fds[m].revents |= LWS_POLLIN;
				c = 1;
			}
			if (FD_ISSET(pt->fds[m].fd, &writefds)) {
				pt->fds[m].revents |= LWS_POLLOUT;
				c = 1;
			}
			if (FD_ISSET(pt->fds[m].fd, &errfds)) {
				// lwsl_notice("errfds %d\n", pt->fds[m].fd);
				pt->fds[m].revents |= LWS_POLLHUP;
				c = 1;
			}

			if (c)
				n++;
		}
	}
//	lwsl_debug("%s %d n=%d\n", __FUNCTION__, __LINE__, n);
#ifdef LWS_OPENSSL_SUPPORT
	if (!pt->rx_draining_ext_list &&
	    !lws_ssl_anybody_has_buffered_read_tsi(context, tsi) && !n) {
#else
	if (!pt->rx_draining_ext_list && !n) /* poll timeout */ {
#endif
//		lwsl_debug("%s %d n=%d\n", __FUNCTION__, __LINE__, n);
		lws_service_fd_tsi(context, NULL, tsi);
		return 0;
	}

faked_service:
	m = lws_service_flag_pending(context, tsi);
	if (m)
		c = -1; /* unknown limit */
	else
		if (n < 0) {
			if (LWS_ERRNO != LWS_EINTR)
				return -1;
			return 0;
		} else
			c = n;

	/* any socket with events to service? */
	for (n = 0; n < pt->fds_count && c; n++) {
		if (!pt->fds[n].revents)
			continue;

		c--;

		m = lws_service_fd_tsi(context, &pt->fds[n], tsi);
		if (m < 0)
			return -1;
		/* if something closed, retry this slot */
		if (m)
			n--;
	}

	return 0;
}

LWS_VISIBLE int
lws_plat_check_connection_error(struct lws *wsi)
{
	return 0;
}

LWS_VISIBLE int
lws_plat_service(struct lws_context *context, int timeout_ms)
{
	int n = _lws_plat_service_tsi(context, timeout_ms, 0);

	lws_service_fd_tsi(context, NULL, 0);

	return n;
}


LWS_VISIBLE int
lws_plat_set_socket_options(struct lws_vhost *vhost, int fd)
{
	int optval = 1;
	socklen_t optlen = sizeof(optval);
	int ret = 0;

	if (vhost == NULL || fd < 0) {
		lwsl_err("%s: invaild arg: %p %d\n", __func__, vhost, fd);
		return 1;
	}

	if (vhost->ka_time) {
		/* enable keepalive on this socket */
		optval = 1;
		if ((ret = setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE,
			       (const void *)&optval, optlen)) < 0) {
			lwsl_err("%s: SO_KEEPALIVE set err = %d\n", __func__, ret);
			return 1;
		}

		/* set the keepalive conditions we want on it too */
		optval = vhost->ka_time;
		if ((ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,
			       (const void *)&optval, optlen) < 0)) {
			lwsl_err("%s TCP_KEEPIDLE set err = %d\n", __func__, ret);
			return 1;
		}

		optval = vhost->ka_interval;
		if ((ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL,
			       (const void *)&optval, optlen)) < 0) {
			lwsl_err("%s IPPROTO_TCP set err = %d\n", __func__, ret);
			return 1;
		}

		optval = vhost->ka_probes;
		if ((ret = setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,
			       (const void *)&optval, optlen) < 0)) {
			lwsl_err("%s IPPROTO_TCP set err = %d\n", __func__, ret);
			return 1;
		}
	}

	/* Disable Nagle */
	optval = 1;
	if ((ret = setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &optval, optlen) < 0)) {
		lwsl_err("%s IPPROTO_TCP set err = %d\n", __func__, ret);
		return 1;
	}

	/* We are nonblocking... */
	int val = 1;
	if ((ret = ioctlsocket(fd, FIONBIO,  (void *)&val)) != 0) {
		lwsl_err("%s O_NONBLOCK set err = %d\n", __func__, ret);
		return 1;
	}

	return 0;
}

LWS_VISIBLE void
lws_plat_drop_app_privileges(struct lws_context_creation_info *info)
{
}


int
lws_plat_pipe_create(struct lws *wsi)
{
	return 1;
}

int
lws_plat_pipe_signal(struct lws *wsi)
{
	return 1;
}

void
lws_plat_pipe_close(struct lws *wsi)
{
}

LWS_VISIBLE int
lws_plat_context_early_init(void)
{
	return 0;
}

LWS_VISIBLE void
lws_plat_context_early_destroy(struct lws_context *context)
{
}

LWS_VISIBLE void
lws_plat_context_late_destroy(struct lws_context *context)
{
#ifdef LWS_WITH_PLUGINS
	if (context->plugin_list)
		lws_plat_plugins_destroy(context);
#endif

	if (context->lws_lookup)
		lws_free(context->lws_lookup);
}

/* cast a struct sockaddr_in6 * into addr for ipv6 */

LWS_VISIBLE int
lws_interface_to_sa(int ipv6, const char *ifname, struct sockaddr_in *addr,
		    size_t addrlen)
{
#if 0
	int rc = -1;

	struct ifaddrs *ifr;
	struct ifaddrs *ifc;
#ifdef LWS_WITH_IPV6
	struct sockaddr_in6 *addr6 = (struct sockaddr_in6 *)addr;
#endif

	getifaddrs(&ifr);
	for (ifc = ifr; ifc != NULL && rc; ifc = ifc->ifa_next) {
		if (!ifc->ifa_addr)
			continue;

		lwsl_info(" interface %s vs %s\n", ifc->ifa_name, ifname);

		if (strcmp(ifc->ifa_name, ifname))
			continue;

		switch (ifc->ifa_addr->sa_family) {
		case AF_INET:
#ifdef LWS_WITH_IPV6
			if (ipv6) {
				/* map IPv4 to IPv6 */
				bzero((char *)&addr6->sin6_addr,
						sizeof(struct in6_addr));
				addr6->sin6_addr.s6_addr[10] = 0xff;
				addr6->sin6_addr.s6_addr[11] = 0xff;
				memcpy(&addr6->sin6_addr.s6_addr[12],
					&((struct sockaddr_in *)ifc->ifa_addr)->sin_addr,
							sizeof(struct in_addr));
			} else
#endif
				memcpy(addr,
					(struct sockaddr_in *)ifc->ifa_addr,
						    sizeof(struct sockaddr_in));
			break;
#ifdef LWS_WITH_IPV6
		case AF_INET6:
			memcpy(&addr6->sin6_addr,
			  &((struct sockaddr_in6 *)ifc->ifa_addr)->sin6_addr,
						       sizeof(struct in6_addr));
			break;
#endif
		default:
			continue;
		}
		rc = 0;
	}

	freeifaddrs(ifr);

	if (rc == -1) {
		/* check if bind to IP address */
#ifdef LWS_WITH_IPV6
		if (inet_pton(AF_INET6, ifname, &addr6->sin6_addr) == 1)
			rc = 0;
		else
#endif
		if (inet_pton(AF_INET, ifname, &addr->sin_addr) == 1)
			rc = 0;
	}

	return rc;
#endif

	struct addrinfo *result = NULL;
	struct addrinfo hints;

	memset(&hints, 0, sizeof(hints) );
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	if (!getaddrinfo(ifname, 0, &hints, &result)) {
		memcpy(addr,result->ai_addr,sizeof(*addr));
		return 0;
	}

	return -1;
}

LWS_VISIBLE void
lws_plat_insert_socket_into_fds(struct lws_context *context, struct lws *wsi)
{
	struct lws_context_per_thread *pt = &context->pt[(int)wsi->tsi];

	pt->fds[pt->fds_count++].revents = 0;
}

LWS_VISIBLE void
lws_plat_delete_socket_from_fds(struct lws_context *context,
						struct lws *wsi, int m)
{
	struct lws_context_per_thread *pt = &context->pt[(int)wsi->tsi];

	pt->fds_count--;
}

LWS_VISIBLE void
lws_plat_service_periodic(struct lws_context *context)
{
}

LWS_VISIBLE int
lws_plat_change_pollfd(struct lws_context *context,
		      struct lws *wsi, struct lws_pollfd *pfd)
{
	return 0;
}

LWS_VISIBLE const char *
lws_plat_inet_ntop(int af, const void *src, char *dst, int cnt)
{
	return inet_ntoa_r(src, dst, cnt);
}

LWS_VISIBLE int
lws_plat_inet_pton(int af, const char *src, void *dst)
{
	return inet_aton(src, dst);
}

LWS_VISIBLE lws_fop_fd_t
_lws_plat_file_open(const struct lws_plat_file_ops *fops, const char *filename,
		    const char *vpath, lws_fop_flags_t *flags)
{
	return NULL;
}

LWS_VISIBLE int
_lws_plat_file_close(lws_fop_fd_t *fops_fd)
{
	return 0;
}

LWS_VISIBLE lws_fileofs_t
_lws_plat_file_seek_cur(lws_fop_fd_t fops_fd, lws_fileofs_t offset)
{
	return 0;
}

LWS_VISIBLE int
_lws_plat_file_read(lws_fop_fd_t fops_fd, lws_filepos_t *amount,
		    uint8_t *buf, lws_filepos_t len)
{

	return 0;
}

LWS_VISIBLE int
_lws_plat_file_write(lws_fop_fd_t fops_fd, lws_filepos_t *amount,
		     uint8_t *buf, lws_filepos_t len)
{

	return 0;
}

#if defined(LWS_WITH_HTTP2)
/*
 * These are the default SETTINGS used on this platform.  The user
 * can selectively modify them for a vhost during vhost creation.
 */
const struct http2_settings const lws_h2_defaults_xradio = { {
	1,
	/* H2SET_HEADER_TABLE_SIZE */			 512,
	/* H2SET_ENABLE_PUSH */				   0,
	/* H2SET_MAX_CONCURRENT_STREAMS */		   8,
	/* H2SET_INITIAL_WINDOW_SIZE */		       65535,
	/* H2SET_MAX_FRAME_SIZE */		       16384,
	/* H2SET_MAX_HEADER_LIST_SIZE */	 	 512,
	/* H2SET_RESERVED7 */				   0,
	/* H2SET_ENABLE_CONNECT_PROTOCOL */		   1,
}};
#endif

LWS_VISIBLE int
lws_plat_init(struct lws_context *context,
	      struct lws_context_creation_info *info)
{
	/* master context has the global fd lookup array */
	context->lws_lookup = lws_zalloc(sizeof(struct lws *) *
					 context->max_fds, "xrdio lws_lookup");
	if (context->lws_lookup == NULL) {
		lwsl_err("OOM on lws_lookup array for %d connections\n",
			 context->max_fds);
		return 1;
	}

	lwsl_notice(" mem: platform fd map: %5lu bytes\n",
		    (unsigned long)(sizeof(struct lws *) * context->max_fds));

#ifdef LWS_WITH_PLUGINS
	if (info->plugin_dirs)
		lws_plat_plugins_init(context, info->plugin_dirs);
#endif
#if defined(LWS_WITH_HTTP2)
	/* override settings */
	context->set = lws_h2_defaults_xradio;
#endif

	return 0;
}

#endif
