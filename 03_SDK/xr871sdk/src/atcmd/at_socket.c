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

u8 at_socket_buf[AT_SOCKET_BUFFER_SIZE];

AT_ERROR_CODE at_sockon(char *hostname, s32 port, char *protocol, char *ind)
{
	AT_ERROR_CODE aec;
	at_callback_para_t para;
	at_callback_rsp_t rsp;

	memset(&para, 0, sizeof(para));
	memset(&rsp, 0, sizeof(rsp));

	strncpy(para.u.sockon.hostname, hostname, sizeof(para.u.sockon.hostname) - 1);
	para.u.sockon.port = port;

	if (!strcmp(protocol, "t")) {
		para.u.sockon.protocol = 0;
	}
	else if (!strcmp(protocol, "u")) {
		para.u.sockon.protocol = 1;
	}

	if (!strcmp(ind,"ind")) {
		para.u.sockon.ind = 1;
	}

	if (at_callback.handle_cb != NULL) {
		aec = at_callback.handle_cb(ACC_SOCKON, &para, &rsp);
		if (aec == AEC_OK) {
			if (rsp.type == 0) {
				int id;

				id = (s32)rsp.vptr;

				at_dump("ID: %02d\r\n", id);
			}
		}

		/* AT_DBG("aec=%d\n", aec); */
		return aec;
	}

	return AEC_UNDEFINED;
}

AT_ERROR_CODE at_sockw(char *id, s32 len)
{
	at_callback_para_t para;
	char *cptr;
	s32 rlen;
	s32 i;

	memset(&para, 0, sizeof(para));

	para.u.sockw.id = strtol(id, &cptr, 10);
	para.u.sockw.buf = at_socket_buf;

	while (len > 0) {
		rlen = len < sizeof(at_socket_buf) ? len : sizeof(at_socket_buf);

		para.u.sockw.len = rlen;

		for (i = 0; i < rlen; i++) {
			while (at_queue_get(&at_socket_buf[i]) != AQEC_OK) {
				;
			}
		}

		if (at_callback.handle_cb != NULL) {
			if (at_callback.handle_cb(ACC_SOCKW, &para, NULL) != AEC_OK) {
				return AEC_SEND_FAIL; /* fail */
			}
		}

		len -= rlen;
	}

	return AEC_OK;
}

AT_ERROR_CODE at_sockq(char *id)
{
	AT_ERROR_CODE aec = AEC_UNDEFINED;
	at_callback_para_t para;
	at_callback_rsp_t rsp;
	char *cptr;

	memset(&para, 0, sizeof(para));
	memset(&rsp, 0, sizeof(rsp));

	para.u.sockq.id = strtol(id, &cptr, 10);

	if (at_callback.handle_cb != NULL) {
		aec = at_callback.handle_cb(ACC_SOCKQ, &para, &rsp);
		if (aec == AEC_OK) {
			if (rsp.type == 0) {
				int len;

				len = (s32)rsp.vptr;

				at_dump("DATALEN: %d\r\n",len);
			}
		}
	}

	return aec;
}

AT_ERROR_CODE at_sockr(char *id, s32 len)
{
	at_callback_para_t para;
	char *cptr;

	memset(&para, 0, sizeof(para));

	para.u.sockr.id = strtol(id, &cptr, 10);
	para.u.sockr.len = len;

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_SOCKR, &para, NULL);
	}

	return AEC_OK;
}

AT_ERROR_CODE at_sockc(char *id)
{
	at_callback_para_t para;
	char *cptr;

	memset(&para, 0, sizeof(para));

	para.u.sockc.id = strtol(id, &cptr, 10);

	if (at_callback.handle_cb != NULL) {
		return at_callback.handle_cb(ACC_SOCKC, &para, NULL);
	}

	return AEC_UNDEFINED;
}

AT_ERROR_CODE at_sockd(s32 port, char *protocol)
{
	at_callback_para_t para;

	memset(&para, 0, sizeof(para));

	para.u.sockd.port = port;

	if (!strcmp(protocol, "t")) {
		para.u.sockd.protocol = 0;
	}
	else if (!strcmp(protocol, "u")) {
		para.u.sockd.protocol = 1;
	}

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_SOCKD, &para, NULL);
	}

	return AEC_OK;
}
