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

#ifndef _AIRKISS_DISCOVER_H_
#define _AIRKISS_DISCOVER_H_

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	char *app_id;
	char *device_id;
	uint32_t ack_period_ms;
}Airkiss_Online_Ack_Info;

/*The wechat_public_id and devic_id should be global variable*/
/*In this mode, the driver will be send online data by cycle*/
int airkiss_online_cycle_ack_start(Airkiss_Online_Ack_Info *param);
void airkiss_online_cycle_ack_stop();

/*The wechat_public_id and devic_id should be global variable*/
/*In this mode,  the drivers will be listen to server's request and send ack for server, then the driver will
be send online data by cycle*/
int airkiss_online_dialog_mode_start(Airkiss_Online_Ack_Info *param);
void airkiss_online_dialog_mode_stop();

#ifdef __cplusplus
}
#endif

#endif /*_AIRKISS_DISCOVER_H_*/
