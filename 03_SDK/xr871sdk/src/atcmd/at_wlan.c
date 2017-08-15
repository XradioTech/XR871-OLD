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

static at_peer_t peers[AT_MAX_PEER_NUM];

AT_ERROR_CODE at_peers(s32 pn, char *pv)
{
	AT_ERROR_CODE aec;

	if (pn == 0 && pv == NULL) {
		s32 i;

		for (i = 0; i < AT_MAX_PEER_NUM; i++) {
			aec = at_peer(i, &peers[i], NULL);

			if (aec != AEC_OK) {
				return aec; /* error */
			}
		}

		return AEC_OK;
	}
	else if (pn >= 0 && pn < AT_MAX_PEER_NUM) {
		return at_peer(pn, &peers[pn], pv);
	}

	return AEC_PARA_ERROR; /* error */
}

AT_ERROR_CODE at_ping(char *hostname)
{
	at_callback_para_t para;

	strcpy(para.u.ping.hostname, hostname);

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_PING, &para, NULL);
	}

	return AEC_OK;
}

AT_ERROR_CODE at_wifi(s32 value)
{
	at_callback_para_t para;

	para.u.wifi.value = value;
	para.cfg = &at_cfg;

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_WIFI, &para, NULL);
	}

	return AEC_OK;
}

AT_ERROR_CODE at_reassociate(void)
{
	at_callback_para_t para;

	para.cfg = &at_cfg;

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_REASSOCIATE, &para, NULL);
	}

	return AEC_OK;
}

AT_ERROR_CODE at_scan(char *mode, char *repeat)
{
	at_callback_para_t para;

	if (mode == NULL && repeat == NULL) {
		para.u.scan.method = 0;
	}
	else if (mode != NULL && repeat != NULL) {
		if (!strcmp(mode,"a") && !strcmp(repeat,"r")) {
			para.u.scan.method = 1;
		}
		else if (!strcmp(mode,"p") && !strcmp(repeat,"r")) {
			para.u.scan.method = 2;
		}
		else {
			return AEC_PARA_ERROR; /* error parameter */
		}
	}
	else {
		return AEC_PARA_ERROR; /* error parameter */
	}

	para.cfg = &at_cfg;

	if (at_callback.handle_cb != NULL) {
		at_callback.handle_cb(ACC_SCAN, &para, NULL);
	}

	return AEC_OK; /* OK */
}
