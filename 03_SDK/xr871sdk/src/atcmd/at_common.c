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

#include <stdarg.h>
#include "atcmd/at_command.h"
#include "at_private.h"
#include "at_debug.h"

extern u8 at_socket_buf[AT_SOCKET_BUFFER_SIZE];

typedef struct {
	AT_ERROR_CODE aec;
	const char *info;
} err_info_t;

static const err_info_t err_info[] = {
	{AEC_CMD_ERROR,			"Error command."},
	{AEC_NO_PARA,			"No parameter."},
	{AEC_PARA_ERROR,		"Error parameter."},
	{AEC_UNSUPPORTED,		"Unsupported."},
	{AEC_NOT_FOUND,			"Not found."},
	{AEC_NULL_POINTER,		"Null pointer."},
	{AEC_OUT_OF_RANGE,		"Out of range."},
	{AEC_SCAN_FAIL,			"Scan fail."},
	{AEC_READ_ONLY,			"Read only."},
	{AEC_SEND_FAIL,			"Send fail."},
	{AEC_CONNECT_FAIL,		"Connect fail."},
	{AEC_BIND_FAIL,			"Bind fail."},
	{AEC_SOCKET_FAIL,		"Socket fail."},
	{AEC_LIMITED,			"Limited."},
	{AEC_DISCONNECT,		"Disconnect."},
	{AEC_NETWORK_ERROR,		"Network error."},
	{AEC_NOT_ENOUGH_MEMORY,	"Not enough memory."},
	{AEC_IMPROPER_OPERATION,"Improper operation."},
	{AEC_UNDEFINED,			"Undefined."},
};

void at_response(AT_ERROR_CODE aec)
{
	if (aec == AEC_OK) {
		at_dump("\r\nOK\r\n");
	}
	else {
		s32 i;

		for (i=0; i<TABLE_SIZE(err_info); i++) {
			if (err_info[i].aec == aec) {
				at_dump("\r\nERROR: %s\r\n",err_info[i].info);

				return ;
			}
		}

		if (aec != AEC_BLANK_LINE) {
			AT_DBG("no rsp message!\r\n");
		}
	}
}

static char dump_buffer[MAX_DUMP_BUFF_SIZE];

s32 at_event(s32 idx)
{
	if (idx < 32) {
		return !(at_cfg.wind_off_low & (1<<idx));
	}
	else if (idx < 64) {
		idx -= 32;
		return !(at_cfg.wind_off_medium & (1<<idx));
	}
	else if (idx < 96) {
		idx -= 64;
		return !(at_cfg.wind_off_high & (1<<idx));
	}
	else {
		return 0;
	}
}

s32 at_serial(at_serial_para_t *ppara)
{
	ppara->baudrate = at_cfg.console1_speed;
	ppara->hwfc = at_cfg.console1_hwfc;

	return 0;
}

s32 at_dump(char* format, ...)
{
    int len;
    va_list vp;

    va_start(vp, format);
    len = vsnprintf(dump_buffer, MAX_DUMP_BUFF_SIZE, format, vp);
    va_end(vp);

	if (strlen(dump_buffer) < len) {
		AT_ERR("dump info is too long!");
		return -1;
	}

	if (at_callback.dump_cb != NULL) {
		at_callback.dump_cb((u8 *)dump_buffer, len);
	}

	return 0;
}

AT_ERROR_CODE at_act(void)
{
	at_callback_para_t para;

	if (at_callback.handle_cb != NULL) {
		memset(&para, 0, sizeof(para));
		para.cfg = &at_cfg;
		at_callback.handle_cb(ACC_ACT, &para, NULL);
	}

	return AEC_OK;
}

AT_ERROR_CODE at_reset(void)
{
	if (at_callback.handle_cb != NULL) {
		at_dump("Reboot...\r\n");
		at_callback.handle_cb(ACC_RST, NULL, NULL);
	}

	return AEC_OK;
}

AT_ERROR_CODE at_mode(AT_MODE mode)
{
	AT_QUEUE_ERROR_CODE aqec;
	at_callback_para_t para;
	s32 len,escape_len;
	u8 tmp;

	if (at_callback.handle_cb != NULL) {
		memset(&para, 0, sizeof(para));
		escape_len = strlen(at_cfg.escape_seq);
		at_dump("Enter data mode.\r\n");
		while (1) {
			para.u.mode.buf = at_socket_buf;

			len = 0;

			while (len < AT_SOCKET_BUFFER_SIZE) {
				aqec = at_queue_get(&tmp);

				//AT_DBG("aqec = %d\n", aqec);
				//AT_DBG("len = %d\n", len);
				if(aqec == AQEC_OK) {
					at_socket_buf[len++] = tmp;
				}
				else {
					break;
				}
			}

			if (len > 2 && (len >= escape_len && len <= escape_len + 2) &&
				!strncmp(at_cfg.escape_seq, (const char *)at_socket_buf, escape_len)) {
				at_dump("Exit data mode.\r\n");
				break;
			}

			para.u.mode.len = len;
			//AT_DBG("Enter callback\n");
			if (at_callback.handle_cb(ACC_MODE, &para, NULL) != AEC_OK) {
				at_dump("Exit data mode.\r\n");
				break;
			}
		}
	}

	return AEC_OK;
}
