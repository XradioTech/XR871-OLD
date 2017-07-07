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
#include "source.h"

static int exit_flag;	/* Program termination flag	*/

struct usrfile filelist[] = {
	{"./",""},
	{"./index.html",index_html},
	{"./config.shtml",config_shtml},
	{NULL},
};

char g_ssid[20];
char *init_ssid = "testserver";

static void set_ssid(struct shttpd_arg *arg)
{
	char		*p = arg->user_data;	/* integer passed to us */
	char		value[20];
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

	}

	memcpy(p,value,strlen(value));
	p[strlen(value)] = '\0';

	shttpd_printf(arg, "%s", "HTTP/1.1 302 OK\r\nLocation: /config.shtml\r\n\r\n");
	//shttpd_printf(arg, "%s", "HTTP/1.1 200 OK\r\n");
	arg->flags |= SHTTPD_END_OF_OUTPUT;
}

/*
 * This function will be called on SSI <!--#call DeviceName -->
 */
static void ssi_get_ssid(struct shttpd_arg *arg)
{

	shttpd_printf(arg,
	    "%s", arg->user_data);
	arg->flags |= SHTTPD_END_OF_OUTPUT;
}


int shttpd_start(int argc, char *argv[])
{
	struct shttpd_ctx	*ctx;
	_shttpd_init_local(filelist,ARRAY_SIZE(filelist));
	if ((ctx = shttpd_init(argc, argv)) == NULL) {
		_shttpd_elog(E_FATAL, NULL, "%s",
		    "Cannot initialize SHTTPD context");
		return -1;
	}
	memcpy(g_ssid,init_ssid,strlen(init_ssid));
	g_ssid[strlen(g_ssid)] = '\0';

	_shttpd_elog(E_LOG, NULL, "shttpd %s started on port(s) %s, serving %s",
	    VERSION, ctx->options[OPT_PORTS], ctx->options[OPT_ROOT]);

	shttpd_register_ssi_func(ctx, "DeviceName", ssi_get_ssid, g_ssid);
	shttpd_register_uri(ctx, "/devicename", &set_ssid, (void *) g_ssid);
	exit_flag = 0;
	while (exit_flag == 0)
		shttpd_poll(ctx, 1000);

	_shttpd_elog(E_LOG, NULL, "Exit on signal %d", exit_flag);
	shttpd_fini(ctx);
	return 0;
}

void shttpd_stop()
{
	exit_flag = 1;
}