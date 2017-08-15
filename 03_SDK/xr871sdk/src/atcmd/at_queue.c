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

static at_queue_callback_t at_queue_callback = NULL;
static at_queue_t at_queue;

s32 at_queue_init(void *buf, s32 size, at_queue_callback_t cb)
{
	at_queue_t *q = &at_queue;

	if (buf == NULL || cb == NULL) {
		return -1; /* null pointer */
	}

	memset(q, 0, sizeof(at_queue_t));

	q->qbuf = buf;
	q->qsize = size;

	at_queue_callback = cb;

	return 0;
}

AT_QUEUE_ERROR_CODE at_queue_get(u8 *element)
{
	at_queue_t *q = &at_queue;
	u8 winbuf[32];
	s32 dcnt;

	if (q->qcnt <= 0) {
		if (at_queue_callback != NULL) {
			dcnt = at_queue_callback(winbuf, sizeof(winbuf));

			if (dcnt > 0) {
				if (dcnt > q->qsize - q->qcnt) {
					AT_DBG("queue is overflow\n");
				}
				else {
					s32 i;

					for (i=0; i < dcnt; i++) {
						q->qbuf[q->widx++] = winbuf[i];
						q->widx = q->widx >= q->qsize ? 0 : q->widx;
						q->qcnt++;
					}
				}
			}
			else {
				return AQEC_EMPTY;
			}
		}
		else {
			return AQEC_EMPTY;
		}
	}

	*element = q->qbuf[q->ridx++];
	q->ridx = q->ridx >= q->qsize ? 0 : q->ridx;
	q->qcnt--;

	return AQEC_OK;
}

AT_QUEUE_ERROR_CODE at_queue_peek(u8 *element)
{
	at_queue_t *q = &at_queue;
	u8 winbuf[32];
	s32 dcnt;

	if (q->qcnt <= 0) {
		if (at_queue_callback != NULL) {
			dcnt = at_queue_callback(winbuf, sizeof(winbuf));

			if (dcnt > 0) {
				if (dcnt > q->qsize - q->qcnt) {
					AT_DBG("queue is overflow\n");
				}
				else {
					s32 i;

					for (i=0; i < dcnt; i++) {
						q->qbuf[q->widx++] = winbuf[i];
						q->widx = q->widx >= q->qsize ? 0 : q->widx;
						q->qcnt++;
					}
				}
			}
			else {
				return AQEC_EMPTY;
			}
		}
		else {
			return AQEC_EMPTY;
		}
	}

	*element = q->qbuf[q->ridx];

	return AQEC_OK;
}
