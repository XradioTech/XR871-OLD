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

#ifndef _MQTTECHO_H_
#define _MQTTECHO_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	MES_DIARVE	= 1,
	MES_ARVE	= 2
} MESSAGE_STATUS;

typedef enum {
	MQTT_DICACK	= 1,
	MQTT_CACK		= 2
} MQTT_CALLBACK;

typedef struct mqt_cal{
	unsigned char mqtt_con;
	unsigned char mqtt_quit;
	unsigned char mqtt_sub;
	unsigned char mqtt_pub;
}mqt_cal;

#define MES_FLAG_Clean		MES_DIARVE
#define mqtt_buff_size		350

extern unsigned char BbcSubGet[mqtt_buff_size];
extern unsigned char BbcPubSet[mqtt_buff_size];

extern uint8_t MessageArriveFlag;
extern uint8_t mqtt_set_rcome;
extern mqt_cal cal_set;

int mqtt_ctrl_task_init();

#ifdef __cplusplus
}
#endif

#endif
