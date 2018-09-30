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
#include "sys/io.h"
#include "defs.h"
#include "source.h"

struct shttpd_ap_info {
	char g_ssid[20];
	char g_passwd[20];
};

/* Program termination flag */
static char _shttpd_exit_flag;

static const struct usr_file file_list[] = {
	{"./",              ""},
	{"./index.html",    index_html},
	{"./config.shtml",  config_shtml},
#if !defined(NO_CHECKSUM)
	{"./checksum1.txt",  checksum1_txt},
	{"./checksum2.txt",  checksum2_txt},
#endif
	{NULL},
};

struct shttpd_ap_info ap_info;

static const char init_ssid[] = "ap-ssid";
static const char init_passwd[] = "ap-passwd";

#if defined(SHTTPD_SSL)
extern const char *mbedtls_test_srv_crt;
extern const size_t mbedtls_test_srv_crt_len;
extern const char mbedtls_test_cas_pem[];
extern const size_t mbedtls_test_cas_pem_len;
extern const char *mbedtls_test_srv_key;
extern const size_t mbedtls_test_srv_key_len;
security_server ca_param;
#endif

static void set_ap_info(struct shttpd_arg *arg)
{
	struct shttpd_ap_info *info = (struct shttpd_ap_info *)(arg->user_data);
	char value[20];
	const char *request_method, *query_string, *request_uri;

	request_method = shttpd_get_env(arg, "REQUEST_METHOD");
	request_uri = shttpd_get_env(arg, "REQUEST_URI");
	query_string = shttpd_get_env(arg, "QUERY_STRING");
	request_method = request_method;
	request_uri = request_uri;
	query_string = query_string;

	/* Change the value of integer variable */
	value[0] = '\0';
	if (!strcmp(request_method, "POST")) {
		/* If not all data is POSTed, wait for the rest */
		if (arg->flags & SHTTPD_MORE_POST_DATA)
			return;
		(void) shttpd_get_var("DeviceName", arg->in.buf, arg->in.len,
		    value, sizeof(value));
		_shttpd_strlcpy(info->g_ssid, value, sizeof(info->g_ssid));

		(info->g_ssid)[strlen(value)] = '\0';

		(void) shttpd_get_var("DevicePasswd", arg->in.buf, arg->in.len,
		value, sizeof(value));
		_shttpd_strlcpy(info->g_passwd, value, sizeof(info->g_passwd));
		(info->g_passwd)[strlen(value)] = '\0';
	}

	shttpd_printf(arg, "%s", "HTTP/1.1 302 OK\r\nLocation: /config.shtml\r\n\r\n");
	arg->flags |= SHTTPD_END_OF_OUTPUT;
}

#if !defined(NO_CHECKSUM)

char *g_file_pointer = NULL;
static void set_post_info(struct shttpd_arg *arg)
{
	char		value[20];
	const char	*request_method, *query_string;
	if (g_file_pointer != NULL)
		goto send_data;

	request_method = shttpd_get_env(arg, "REQUEST_METHOD");
	query_string = shttpd_get_env(arg, "QUERY_STRING");

	/* Change the value of integer variable */
	value[0] = '\0';
	if (!strcmp(request_method, "POST")) {
		/* If not all data is POSTed, wait for the rest */
		if (arg->flags & SHTTPD_MORE_POST_DATA)
			return;
		(void) shttpd_get_var("filename", arg->in.buf, arg->in.len,
		    value, sizeof(value));
	} else if (query_string != NULL) {

		(void) shttpd_get_var("filename", query_string,
		    strlen(query_string), value, sizeof(value));
	}

	if (value[0] == '\0') {
		arg->flags |= SHTTPD_END_OF_OUTPUT;
		shttpd_printf(arg, "%s", "HTTP/1.1 400 Bad request\r\n\r\n");
		return;
	}

	if (!strcmp(value, "checksum1"))
		g_file_pointer = checksum1_txt;
	else if (!strcmp(value, "checksum2"))
		g_file_pointer = checksum2_txt;
	else {
		_shttpd_elog(E_FATAL, NULL,
		            "wrong command(%s).", __LINE__);
		arg->flags |= SHTTPD_END_OF_OUTPUT;
		shttpd_printf(arg, "%s", "HTTP/1.1 400 Bad request\r\n\r\n");
		return;
	}

	shttpd_printf(arg, "%s", "HTTP/1.1 200 OK\r\n");
	shttpd_printf(arg, "%s", "Content-Type: text/html\r\n");
	shttpd_printf(arg, "Content-Length: %lu\r\n\r\n", strlen(g_file_pointer));

	unsigned int len = 0, buflen = 0;
send_data:
	buflen = arg->out.len - arg->out.num_bytes;
	if (buflen > strlen(g_file_pointer)) {
		len = strlen(g_file_pointer);
		arg->flags |= SHTTPD_END_OF_OUTPUT;
	} else
		len = buflen - 1;
	shttpd_printf(arg, "%.*s", len, g_file_pointer);
	g_file_pointer += len;

	if (arg->flags & SHTTPD_END_OF_OUTPUT)
		g_file_pointer = NULL;

}
#endif

/*
 * This function will be called on SSI
 * <!--#call DeviceName -->
 */
static void ssi_get_ssid(struct shttpd_arg *arg)
{
	shttpd_printf(arg,
	              "%s", arg->user_data);
	arg->flags |= SHTTPD_END_OF_OUTPUT;
}

static void ssi_get_passwd(struct shttpd_arg *arg)
{
	shttpd_printf(arg,
	              "%s", arg->user_data);
	arg->flags |= SHTTPD_END_OF_OUTPUT;
}

int webserver_start(int argc, char *argv[])
{
	struct shttpd_ctx *ctx;

	_shttpd_init_local_file(file_list, ARRAY_SIZE(file_list));

	if ((ctx = shttpd_init(argc, argv)) == NULL) {
		_shttpd_elog(E_FATAL, NULL, "Cannot initialize SHTTPD context.");
		return -1;
	}
#if defined(SHTTPD_SSL)
	ca_param.nCa = mbedtls_test_cas_pem_len;
	ca_param.pCa = (char *)mbedtls_test_cas_pem;
	ca_param.nCert = mbedtls_test_srv_crt_len;
	ca_param.pCert = (char *)mbedtls_test_srv_crt;
	ca_param.nKey = mbedtls_test_srv_key_len;
	ca_param.pKey = (char *)mbedtls_test_srv_key;

	shttpd_set_ssl_cert((void*) &ca_param);
	shttpd_set_option(ctx, "ssl_cert", NULL);
	shttpd_set_option(ctx, "ports", "443s");
#else
	shttpd_set_option(ctx, "ports", "80");
#endif

	unsigned int length = (sizeof(ap_info.g_ssid) > strlen(init_ssid)) ?
	                      strlen(init_ssid) : sizeof(ap_info.g_ssid);
	_shttpd_strlcpy(ap_info.g_ssid, init_ssid, length);

	length = (sizeof(ap_info.g_passwd) > strlen(init_passwd)) ?
	                      strlen(init_passwd) : sizeof(ap_info.g_passwd);
	_shttpd_strlcpy(ap_info.g_passwd, init_passwd, length);

	_shttpd_elog(E_LOG, NULL, "shttpd %s started on port %s, serving %s",
	    VERSION, ctx->options[OPT_PORTS], ctx->options[OPT_ROOT]);

	shttpd_register_ssi_func(ctx, "DeviceName", ssi_get_ssid, ap_info.g_ssid);
	shttpd_register_ssi_func(ctx, "DevicePasswd", ssi_get_passwd, ap_info.g_passwd);
	shttpd_register_uri(ctx, "/devicename", set_ap_info, (void *)&ap_info);

#if !defined(NO_CHECKSUM)
	shttpd_register_uri(ctx, "/post/key/checksum1_txt", set_post_info, NULL);
	shttpd_register_uri(ctx, "/post/key/checksum2_txt", set_post_info, NULL);
	shttpd_register_uri(ctx, "/get/key/checksum1_txt", set_post_info, NULL);
#endif

	_shttpd_exit_flag = 0;
	while (_shttpd_exit_flag == 0)
		shttpd_poll(ctx, 1000);

	_shttpd_elog(E_LOG, NULL, "Shttpd server exit.");
	shttpd_fini(ctx);
	return 0;
}

void webserver_stop()
{
	_shttpd_exit_flag = 1;
}
