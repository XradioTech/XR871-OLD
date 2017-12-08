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

#include "atcmd/at_command.h"
#include "at_private.h"
#include "at_debug.h"

#define CMD_CACHE_MAX_LEN 1024

typedef struct cmd_cache {
	u32 cnt;
	u8 buf[CMD_CACHE_MAX_LEN];
} cmd_cache_t;

typedef struct {
	char *ptr;
} at_para_t;

typedef struct {
	const char *cmd;
	AT_ERROR_CODE (*handler)(at_para_t *at_para);
	const char *usage;
} at_command_handler_t;

typedef struct {
	at_text_t key[AT_PARA_MAX_SIZE];
} at_getcfg_t;

typedef struct {
	at_text_t key[AT_PARA_MAX_SIZE];
} config_key_t;

typedef struct {
	at_text_t ssid[AT_PARA_MAX_SIZE];
} at_ssidtxt_para_t;

typedef struct {
	at_text_t sts_var[AT_PARA_MAX_SIZE];
} at_status_para_t;

typedef struct {
	at_di_t peer_number;
	at_text_t peer_var[AT_PARA_MAX_SIZE];
} at_peers_para_t;

typedef struct {
	at_text_t hostname[AT_PARA_MAX_SIZE];
} at_ping_para_t;

typedef struct {
	at_text_t hostname[AT_PARA_MAX_SIZE];
	at_di_t port;
	at_text_t protocol[1+1];
	at_text_t ind[3+1];
} at_sockon_para_t;

typedef struct {
	at_text_t id[AT_PARA_MAX_SIZE];
	at_di_t len;
} at_sockw_para_t;

typedef struct {
	at_text_t id[AT_PARA_MAX_SIZE];
} at_sockq_para_t;

typedef struct {
	at_text_t id[AT_PARA_MAX_SIZE];
	at_di_t len;
} at_sockr_para_t;

typedef struct {
	at_text_t id[AT_PARA_MAX_SIZE];
} at_sockc_para_t;

typedef struct {
	at_di_t port;
	at_text_t protocol[1+1];
} at_sockd_para_t;

typedef struct {
	at_di_t value;
} at_wifi_para_t;

typedef struct {
	at_di_t num;
	at_text_t direction[3+1];
	at_text_t interrupt[1+1];
} at_gpioc_para_t;

typedef struct {
	at_di_t num;
} at_gpior_para_t;

typedef struct {
	at_di_t num;
	at_di_t value;
} at_gpiow_para_t;

typedef struct {
	at_text_t hostname[AT_PARA_MAX_SIZE];
	at_text_t path[1+1];
	at_di_t port;
} at_upgrade_para_t;

typedef struct {
	at_text_t mode[1+1];
	at_text_t repeat[1+1];
} at_scan_para_t;

extern s32 at_get_value(char *strbuf, s32 pt, void *pvar, s32 vsize);
extern s32 at_set_value(s32 pt, void *pvar, s32 vsize, at_value_t *value);

extern AT_ERROR_CODE at_get_parameters(char **ppara, at_para_descriptor_t *list, s32 lsize, s32 *pcnt);
extern void at_response(AT_ERROR_CODE aec);

extern AT_ERROR_CODE at_act(void);
extern AT_ERROR_CODE at_reset(void);
extern AT_ERROR_CODE at_help(void);
extern AT_ERROR_CODE at_getcfg(char *key);
extern AT_ERROR_CODE at_typecfg(char *key);
extern AT_ERROR_CODE at_setcfg(char *key,at_value_t *value);
extern AT_ERROR_CODE at_ssidtxt(char *ssid);
extern AT_ERROR_CODE at_config(void);
extern AT_ERROR_CODE at_factory(void);
extern AT_ERROR_CODE at_save(void);
extern AT_ERROR_CODE at_status(char *sts_var);
extern AT_ERROR_CODE at_peers(s32 pn,char *pv);
extern AT_ERROR_CODE at_ping(char *hostname);
extern AT_ERROR_CODE at_sockon(char *hostname, s32 port, char *protocol, char *ind);
extern AT_ERROR_CODE at_sockw(char *id, s32 len);
extern AT_ERROR_CODE at_sockq(char *id);
extern AT_ERROR_CODE at_sockr(char *id, s32 len);
extern AT_ERROR_CODE at_sockc(char *id);
extern AT_ERROR_CODE at_sockd(s32 port, char *protocol);
extern AT_ERROR_CODE at_mode(AT_MODE mode);
extern AT_ERROR_CODE at_wifi(s32 value);
extern AT_ERROR_CODE at_reassociate(void);
extern AT_ERROR_CODE at_gpioc(s32 num, char *direction, char *interrupt);
extern AT_ERROR_CODE at_gpior(s32 num);
extern AT_ERROR_CODE at_gpiow(s32 num, s32 value);
extern AT_ERROR_CODE at_upgrade(char *hostname, char *path, s32 port);
extern AT_ERROR_CODE at_scan(char *mode, char *repeat);

static AT_ERROR_CODE attention_handler(at_para_t *at_para);
static AT_ERROR_CODE act_handler(at_para_t *at_para);
static AT_ERROR_CODE reset_handler(at_para_t *at_para);
static AT_ERROR_CODE help_handler(at_para_t *at_para);
static AT_ERROR_CODE getcfg_handler(at_para_t *at_para);
static AT_ERROR_CODE setcfg_handler(at_para_t *at_para);
static AT_ERROR_CODE ssidtxt_handler(at_para_t *at_para);
static AT_ERROR_CODE config_handler(at_para_t *at_para);
static AT_ERROR_CODE factory_handler(at_para_t *at_para);
static AT_ERROR_CODE save_handler(at_para_t *at_para);
static AT_ERROR_CODE status_handler(at_para_t *at_para);
//static AT_ERROR_CODE peers_handler(at_para_t *at_para);
static AT_ERROR_CODE ping_handler(at_para_t *at_para);
static AT_ERROR_CODE sockon_handler(at_para_t *at_para);
static AT_ERROR_CODE sockw_handler(at_para_t *at_para);
static AT_ERROR_CODE sockq_handler(at_para_t *at_para);
static AT_ERROR_CODE sockr_handler(at_para_t *at_para);
static AT_ERROR_CODE sockc_handler(at_para_t *at_para);
static AT_ERROR_CODE sockd_handler(at_para_t *at_para);
static AT_ERROR_CODE mode_handler(at_para_t *at_para);
static AT_ERROR_CODE wifi_handler(at_para_t *at_para);
static AT_ERROR_CODE reassociate_handler(at_para_t *at_para);
//static AT_ERROR_CODE gpioc_handler(at_para_t *at_para);
//static AT_ERROR_CODE gpior_handler(at_para_t *at_para);
//static AT_ERROR_CODE gpiow_handler(at_para_t *at_para);
//static AT_ERROR_CODE upgrade_handler(at_para_t *at_para);
static AT_ERROR_CODE scan_handler(at_para_t *at_para);

at_callback_t at_callback;
static cmd_cache_t cache;

static const at_command_handler_t at_command_table[] = {
	{"AT",					attention_handler,	" -- Null cmd, always returns OK"},
	{"AT+ACT",				act_handler,		" -- Switch WiFi work mode"},
	{"AT+RST",				reset_handler,		" -- Reset the module"},
	{"AT+S.HELP",			help_handler,		" -- This text"},
	{"AT+S.GCFG",			getcfg_handler,		" =<key> -- Get config key"},
	{"AT+S.SCFG",			setcfg_handler,		" =<key>,<value> -- Set config key"},
	{"AT+S.SSIDTXT",		ssidtxt_handler,	" [=<ssidtxt>] -- Set a textual SSID (not hex), otherwise prints current SSID",},
	{"AT&V",				config_handler,		" -- Dump all settings",	},
	{"AT&F",				factory_handler,	" -- Restore factory default settings"},
	{"AT&W",				save_handler,		" -- Save current settings to flash",},
	{"AT+S.STS",			status_handler,		" [=<sts_var>] -- Report current status/statistics",},
	//{"AT+S.PEERS",			peers_handler,		" [=peer_number[,peer_var]] -- Dump contents of the peer table"},
	{"AT+S.PING",			ping_handler,		" =<hostname> -- Send a ping to a specified host"},
	{"AT+S.SOCKON",			sockon_handler,		" =<hostname>,<port>,<t|u> -- Open a network socket"},
	{"AT+S.SOCKW",			sockw_handler,		" =<id>,<len> -- Write data to socket"},
	{"AT+S.SOCKQ",			sockq_handler,		" =<id> -- Query socket for pending data",},
	{"AT+S.SOCKR",			sockr_handler,		" =<id>,<len> -- Read data from socket"},
	{"AT+S.SOCKC",			sockc_handler,		" =<id> -- Close socket"},
	{"AT+S.SOCKD",			sockd_handler,		" =<0|port>,<t|u> -- Disable/Enable socket server. Default is TCP"},
	{"AT+S.",				mode_handler,		" -- Switch to data mode",},
	//{"AT+S.HTTPGET",		NULL,				" =<hostname>,<path&queryopts>[,port] -- Http GET of the given path to the specified host/port"},
	//{"AT+S.HTTPPOST",		NULL,				" =<hostname>,<path&queryopts>,<formcontent>[,port] -- Http POST of the given path to the specified host/port"},
	//{"AT+S.FSC",			NULL,				" =<fname>,<max_len> -- Create a file for httpd use"},
	//{"AT+S.FSA",			NULL,				" =<fname>,<datalen><CR><data> -- Append to an existing file"},
	//{"AT+S.FSD",			NULL,				" =<fname> -- Delete an existing file"},
	//{"AT+S.FSL",			NULL,				" -- List existing filename(s)"},
	//{"AT+S.FSP",			NULL,				" =<fname>[,<offset>,<length>] -- Print the contents of an existing file"},
	{"AT+S.WIFI",			wifi_handler,		" =<0|1> -- Disable/Enable WiFi"},
	{"AT+S.ROAM",			reassociate_handler," -- Trigger a WiFi Roam"},
	//{"AT+S.GPIOC",			gpioc_handler,		" =<num>,<in|out>[,<0|R|F|B>] -- Configure specified GPIO [Optional IRQ]"},
	//{"AT+S.GPIOR",			gpior_handler,		" =<num> -- Read specified GPIO"},
	//{"AT+S.GPIOW",			gpiow_handler,		" =<num>,<val> -- Write specified GPIO",},
	//{"AT+S.FWUPDATE",		upgrade_handler,	" =<hostname>,<path&queryopts>[,port] -- Upgrade the onboard firmware from the specified host/port"},
	//{"AT+S.HTTPDFSUPDATE",	NULL,				" =<hostname>,<path&queryopts>[,port] -- Download a new httpd filesystem from the specified host/port",},
	//{"AT+S.HTTPDFSERASE",	NULL,				" -- Erase the external httpd filesystem"},
	//{"AT+S.HTTPD",			NULL,				" =<0|1> -- Disable/Enable web server"},
	{"AT+S.SCAN",			scan_handler,		" -- Perform a scan"},
};

AT_ERROR_CODE at_help(void)
{
	s32 i;

	for (i = 0; i < TABLE_SIZE(at_command_table); i++) {
		at_dump("# %s%s\r\n", at_command_table[i].cmd, at_command_table[i].usage);
	}

	return AEC_OK;
}

static s32 at_match(char *cmd)
{
	s32 i;

	if (cmd == NULL) {
		return -2;
	}

	for (i = 0; i < TABLE_SIZE(at_command_table); i++) {
		if (!strcmp(cmd, at_command_table[i].cmd)) {
			return i;
		}
	}

	return -1;
}
#if 0
static s32 newline_style(char *cmdline, s32 size)
{
	s32 i;

	for (i = 0; i < size; i++) {
		if (cmdline[i] == AT_LF) {
			return ANL_UNIX;
		}
		else if (cmdline[i] == AT_CR) {
			if (((i+1) < size) && (cmdline[i+1] == AT_LF)) {
				return ANL_WINDOWS;
			}
			else {
				return ANL_MAC;
			}
		}
	}

	return -1;
}
#endif

/**
  * @brief  AT command initializer.
  * @param	cb: AT command callback function
  * @retval AEC_OK: succeed		Other: fail
  */
AT_ERROR_CODE at_init(at_callback_t *cb)
{
	at_callback_para_t para;

	memset(&at_callback, 0, sizeof(at_callback));

	if (cb ==NULL) {
		return AEC_UNDEFINED;
	}

	at_callback = *cb;

	para.cfg = &at_cfg;

	if (at_callback.handle_cb != NULL) {
		if (at_callback.handle_cb(ACC_LOAD, &para, NULL) != AEC_OK) {
			at_callback.handle_cb(ACC_FACTORY, &para, NULL);
		}
	}

	memset(&cache, 0, sizeof(cache));

	return AEC_OK;
}

static AT_ERROR_CODE at_parse_cmd(char *cmdline, s32 size)
{
	char at_cmd[AT_CMD_MAX_SIZE+1];
	at_para_t at_para;
	char *cptr;
	s32 idx;
	s32 cnt = 0;
	s32 i;

	cptr = cmdline;

	for (i = 0; i < size; i++) {
		if (*cptr == AT_EQU) {
			break;
		}
		else if (*cptr == AT_LF) {
			break;
		}
		else if (*cptr == AT_CR) {
			if (((i+1) < size) && (*(cptr+1) == AT_LF)) {
				break;
			}
			else {
				break;
			}
		}
		else {
			if (cnt < sizeof(at_cmd)-1) {
				at_cmd[cnt++] = *cptr++;
			}
			else {
				return AEC_CMD_ERROR;
			}
		}
	}

	at_cmd[cnt] = '\0'; /* add string termination */

	if (cnt == 0) { /* skip blank line */
		return AEC_BLANK_LINE;
	}

	idx = at_match(at_cmd);

	if (idx >= 0) {
		if (at_command_table[idx].handler != NULL) {
			at_para.ptr = cptr;
			return at_command_table[idx].handler(&at_para);
		}
		else {
			return AEC_CMD_ERROR;
		}
	}
	else {
		return AEC_CMD_ERROR;
	}
}

/**
  * @brief  AT command parser.
  * @param	none
  * @retval AEC_OK: succeed		Other: fail
  */
AT_ERROR_CODE at_parse(void)
{
	AT_ERROR_CODE aec;
	AT_QUEUE_ERROR_CODE aqec;
	u8 tmp;
	u32 flag = 0;

	while(1) {
		aqec = at_queue_get(&tmp);

		if(aqec == AQEC_OK) {
			if (tmp == AT_LF) {
				if(cache.cnt < CMD_CACHE_MAX_LEN) {
					cache.buf[cache.cnt++] = tmp;
				}
				else {
					cache.cnt = 0;
					AT_DBG("command is discarded!\n");
					continue; /* command is discarded */
				}

				flag = 1;
			}
			else if (tmp == AT_CR) {
				cache.buf[cache.cnt++] = tmp;

				aqec = at_queue_peek(&tmp);
				if(aqec == AQEC_OK && tmp == AT_LF) {
					aqec = at_queue_peek(&tmp);
					if(cache.cnt < CMD_CACHE_MAX_LEN) {
						cache.buf[cache.cnt++] = tmp;
					}
					else {
						cache.cnt = 0;
						AT_DBG("command is discarded!\n");
						continue; /* command is discarded */
					}
				}

				flag = 1;
			}

			if(cache.cnt < CMD_CACHE_MAX_LEN) {
				cache.buf[cache.cnt++] = tmp;
			}
			else {
				cache.cnt = 0;
				AT_DBG("command is discarded!\n");
				continue; /* command is discarded */
			}
		}

		if (flag) {
			/* echo */
			if (at_cfg.localecho1) {
				cache.buf[cache.cnt] = '\0';
				at_dump("%s", cache.buf);
			}

			aec = at_parse_cmd((char *)cache.buf, cache.cnt);

			at_response(aec);

			cache.cnt = 0;

			flag = 0;
		}
	}

	return AEC_OK;
}

static AT_ERROR_CODE attention_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	return res;
}

static AT_ERROR_CODE act_handler(at_para_t *at_para)
{
	AT_ERROR_CODE res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}
	else {
		return at_act();
	}
}

static AT_ERROR_CODE reset_handler(at_para_t *at_para)
{
	AT_ERROR_CODE res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}
	else {
		return at_reset();
	}
}

static AT_ERROR_CODE help_handler(at_para_t *at_para)
{
	AT_ERROR_CODE res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}
	else {
		return at_help();
	}
}

static AT_ERROR_CODE getcfg_handler(at_para_t *at_para)
{
	at_getcfg_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.key,  AET_LINE | SIZE_LIMIT(sizeof(cmd_para.key))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK)
		{
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		return at_getcfg(cmd_para.key);
	}
}

static AT_ERROR_CODE setcfg_handler(at_para_t *at_para)
{
	config_key_t config_key;
	at_value_t config_value;
    at_para_descriptor_t cmd_key_list[] = {
		{APT_TEXT,	&config_key.key,  AET_PARA | SIZE_LIMIT(sizeof(config_key.key))},
    };
	at_para_descriptor_t cmd_value_list[] = {
		{APT_TEXT,	&config_value.text,		AET_LINE | SIZE_LIMIT(sizeof(config_value.text))},
		{APT_HEX,	&config_value.hex,		AET_LINE | SIZE_LIMIT(sizeof(config_value.hex))},
		{APT_DI,	&config_value.di,		AET_LINE | SIZE_LIMIT(sizeof(config_value.di))},
		{APT_HI,	&config_value.hi,		AET_LINE | SIZE_LIMIT(sizeof(config_value.di))},
		{APT_IP,	&config_value.ip,		AET_LINE | SIZE_LIMIT(sizeof(config_value.ip))},
    };
	s32 paracnt;
	s32 res;
	s32 type;
	s32 idx;
	s32 match = 0;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */
		memset(&config_key, 0, sizeof(config_key)); /* default value */
		res = at_get_parameters(&at_para->ptr, cmd_key_list, TABLE_SIZE(cmd_key_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		type = at_typecfg(config_key.key);

		for (idx = 0; idx < TABLE_SIZE(cmd_value_list); idx++) {
			if (cmd_value_list[idx].pt == type ) {
				match = 1;
				break;
			}
		}

		if (!match) {
			return AEC_NOT_FOUND;
		}
		else {
			memset(&config_value, 0, sizeof(config_value)); /* default value */
			res = at_get_parameters(&at_para->ptr, &cmd_value_list[idx], 1, &paracnt);

			if (res != AEC_OK)	{
				return AEC_PARA_ERROR;
			}

			if (paracnt != 1) {
				return AEC_PARA_ERROR;
			}

			return at_setcfg(config_key.key, &config_value);
		}
	}
}

static AT_ERROR_CODE ssidtxt_handler(at_para_t *at_para)
{
	at_ssidtxt_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.ssid,  AET_LINE | SIZE_LIMIT(sizeof(cmd_para.ssid))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		return at_ssidtxt(cmd_para.ssid);
	}
}

static AT_ERROR_CODE config_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr,NULL,0,NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}
	else {
		return at_config();
	}
}

static AT_ERROR_CODE factory_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}
	else {
		return at_factory();
	}
}

static AT_ERROR_CODE save_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}

	return at_save();
}

static AT_ERROR_CODE status_handler(at_para_t *at_para)
{
	at_status_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.sts_var,  AET_LINE | SIZE_LIMIT(sizeof(cmd_para.sts_var))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}
		else {
			return at_status(NULL);
		}
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK)	{
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		return at_status(cmd_para.sts_var);
	}
}
#if 0
static AT_ERROR_CODE peers_handler(at_para_t *at_para)
{
	at_peers_para_t cmd_para = { /* default value */
		0,
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_DI,	&cmd_para.peer_number,  AET_PARA | SIZE_LIMIT(sizeof(cmd_para.peer_number))},
		{APT_TEXT,	&cmd_para.peer_var,		AET_LINE | SIZE_LIMIT(sizeof(cmd_para.peer_var))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}
		else {
			return at_peers(0, NULL);
		}
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt != 2) {
			return AEC_PARA_ERROR;
		}

		return at_peers(cmd_para.peer_number, cmd_para.peer_var);
	}
}
#endif
static AT_ERROR_CODE ping_handler(at_para_t *at_para)
{
	at_ping_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.hostname,		AET_LINE | SIZE_LIMIT(sizeof(cmd_para.hostname))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		return at_ping(cmd_para.hostname);
	}
}

static AT_ERROR_CODE sockon_handler(at_para_t *at_para)
{
	at_sockon_para_t cmd_para =	{ /* default value */
		"",
		0,
		"t",
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.hostname,	AET_PARA | SIZE_LIMIT(sizeof(cmd_para.hostname))},
		{APT_DI,	&cmd_para.port,		AET_PARA | SIZE_LIMIT(sizeof(cmd_para.port))},
		{APT_TEXT,	&cmd_para.protocol,	AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(cmd_para.protocol))},
		{APT_TEXT,	&cmd_para.ind,		AET_LINE | SIZE_LIMIT(sizeof(cmd_para.ind))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 3) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.port < 0 || cmd_para.port > 65535) {
			return AEC_OUT_OF_RANGE;
		}

		if (!(!strcmp(cmd_para.protocol,"t") || !strcmp(cmd_para.protocol,"u"))) {
			return AEC_PARA_ERROR;
		}

		if (paracnt == 4) {
			if (!(!strcmp(cmd_para.ind, "ind"))) {
				return AEC_PARA_ERROR;
			}
		}

		return at_sockon(cmd_para.hostname, cmd_para.port, cmd_para.protocol, cmd_para.ind);
	}
}

static AT_ERROR_CODE sockw_handler(at_para_t *at_para)
{
	at_sockw_para_t cmd_para = { /* default value */
		"",
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.id,	AET_PARA | SIZE_LIMIT(sizeof(cmd_para.id))},
		{APT_DI,	&cmd_para.len,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.len))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		return at_sockw(cmd_para.id, cmd_para.len);
	}
}

static AT_ERROR_CODE sockq_handler(at_para_t *at_para)
{
	at_sockq_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.id,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.id))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 1) {
			return AEC_PARA_ERROR;
		}

		return at_sockq(cmd_para.id);
	}
}

static AT_ERROR_CODE sockr_handler(at_para_t *at_para)
{
	at_sockr_para_t cmd_para = { /* default value */
		"",
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.id,	AET_PARA | SIZE_LIMIT(sizeof(cmd_para.id))},
		{APT_DI,	&cmd_para.len,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.len))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		return at_sockr(cmd_para.id, cmd_para.len);
	}
}

static AT_ERROR_CODE sockc_handler(at_para_t *at_para)
{
	at_sockc_para_t cmd_para = { /* default value */
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.id,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.id))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 1) {
			return AEC_PARA_ERROR;
		}

		return at_sockc(cmd_para.id);
	}
}

static AT_ERROR_CODE sockd_handler(at_para_t *at_para)
{
	at_sockd_para_t cmd_para = { /* default value */
		0,
		"t"
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_DI,	&cmd_para.port,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(cmd_para.port))},
		{APT_TEXT,	&cmd_para.protocol,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.protocol))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 1) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.port < 0 || cmd_para.port > 65535) {
			return AEC_OUT_OF_RANGE;
		}

		if (!(!strcmp(cmd_para.protocol, "t") || !strcmp(cmd_para.protocol, "u"))) {
			return AEC_PARA_ERROR;
		}

		return at_sockd(cmd_para.port, cmd_para.protocol);
	}
}

static AT_ERROR_CODE mode_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}
	else {
		return at_mode(AM_DATA);
	}
}

static AT_ERROR_CODE wifi_handler(at_para_t *at_para)
{
	at_wifi_para_t cmd_para = { /* default value */
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_DI,	&cmd_para.value,   AET_LINE | SIZE_LIMIT(sizeof(cmd_para.value))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */
		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt != 1) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.value >= 0 && cmd_para.value <= 1) {
			return at_wifi(cmd_para.value);
		}
		else {
			return AEC_OUT_OF_RANGE;
		}
	}
}

static AT_ERROR_CODE reassociate_handler(at_para_t *at_para)
{
	int res;

	res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

	if (res != AEC_OK) {
		return AEC_PARA_ERROR;
	}
	else {
		return at_reassociate();
	}
}
#if 0
static AT_ERROR_CODE gpioc_handler(at_para_t *at_para)
{
	at_gpioc_para_t cmd_para = { /* default value */
		0,
		"",
		"0"
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_DI,	&cmd_para.num,			AET_PARA | SIZE_LIMIT(sizeof(cmd_para.num))},
		{APT_TEXT,	&cmd_para.direction,	AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(cmd_para.direction))},
		{APT_TEXT,	&cmd_para.interrupt,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.interrupt))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.num < 0 || cmd_para.num > 15) {
			return AEC_OUT_OF_RANGE;
		}

		if (!(!strcmp(cmd_para.direction, "in") || !strcmp(cmd_para.direction, "out"))) {
			return AEC_PARA_ERROR;
		}

		if (!(!strcmp(cmd_para.interrupt, "0") || !strcmp(cmd_para.interrupt, "R") || !strcmp(cmd_para.interrupt, "F") || !strcmp(cmd_para.interrupt, "B"))) {
			return AEC_PARA_ERROR;
		}

		return at_gpioc(cmd_para.num, cmd_para.direction, cmd_para.interrupt);
	}
}

static AT_ERROR_CODE gpior_handler(at_para_t *at_para)
{
	at_gpior_para_t cmd_para = { /* default value */
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_DI,	&cmd_para.num,			AET_LINE | SIZE_LIMIT(sizeof(cmd_para.num))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 1) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.num < 0 || cmd_para.num > 15) {
			return AEC_OUT_OF_RANGE;
		}

		return at_gpior(cmd_para.num);
	}
}

static AT_ERROR_CODE gpiow_handler(at_para_t *at_para)
{
	at_gpiow_para_t cmd_para = { /* default value */
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_DI,	&cmd_para.num,		AET_PARA | SIZE_LIMIT(sizeof(cmd_para.num))},
		{APT_DI,	&cmd_para.value,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.value))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		if (cmd_para.num < 0 || cmd_para.num > 15) {
			return AEC_OUT_OF_RANGE;
		}

		if (cmd_para.value < 0 || cmd_para.value > 1) {
			return AEC_OUT_OF_RANGE;
		}

		return at_gpiow(cmd_para.num, cmd_para.value);
	}
}

static AT_ERROR_CODE upgrade_handler(at_para_t *at_para)
{
	at_upgrade_para_t cmd_para = { /* default value */
		"",
		"",
		0
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.hostname,	AET_PARA | SIZE_LIMIT(sizeof(cmd_para.hostname))},
		{APT_TEXT,	&cmd_para.path,		AET_PARA | AET_LINE | SIZE_LIMIT(sizeof(cmd_para.path))},
		{APT_DI,	&cmd_para.port,		AET_LINE | SIZE_LIMIT(sizeof(cmd_para.port))}
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {
		return AEC_NO_PARA;
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		return at_upgrade(cmd_para.hostname, cmd_para.path, cmd_para.port);
	}
}
#endif
static AT_ERROR_CODE scan_handler(at_para_t *at_para)
{
	at_scan_para_t cmd_para = { /* default value */
		"",
		""
	};
    at_para_descriptor_t cmd_para_list[] = {
		{APT_TEXT,	&cmd_para.mode,		AET_PARA | SIZE_LIMIT(sizeof(cmd_para.mode))},
		{APT_TEXT,	&cmd_para.repeat,	AET_LINE | SIZE_LIMIT(sizeof(cmd_para.repeat))},
    };
	s32 paracnt;
	int res;

	if (*at_para->ptr != AT_EQU) {

		res = at_get_parameters(&at_para->ptr, NULL, 0, NULL);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}
		else {
			return at_scan(NULL, NULL);
		}
	}
	else {
		at_para->ptr++; /* skip '=' */

		res = at_get_parameters(&at_para->ptr, cmd_para_list, TABLE_SIZE(cmd_para_list), &paracnt);

		if (res != AEC_OK) {
			return AEC_PARA_ERROR;
		}

		if (paracnt < 2) {
			return AEC_PARA_ERROR;
		}

		if (!(!strcmp(cmd_para.mode, "a") || !strcmp(cmd_para.mode, "p"))) {
			return AEC_PARA_ERROR;
		}

		if (!(!strcmp(cmd_para.repeat, "r"))) {
			return AEC_PARA_ERROR;
		}

		return at_scan(cmd_para.mode, cmd_para.repeat);
	}
}
