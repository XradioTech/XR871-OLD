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

#ifndef CJSON_ANALY_H_
#define CJSON_ANALY_H_

#include "mqtt_build.h"
#include "cjson/cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	BBC_LAMP_OFF 	= 2,
	BBC_LAMP_ON	= 3
} LAMP_STATUS;

typedef enum {
	BBC_MOTOR_OFF 	= 2,
	BBC_MOTOR_ON	= 3
} MOTOR_STATUS;

typedef enum {
	BBC_DO_NONE 	= 0,
	BBC_UP_LOAD	= 1,
	BBC_SET_CTRL	= 2,
	GET_BBC_TIME	= 3,
	BBC_REQ_MSG	= 4,
	BBC_PUB_OTA	= 5
}OPER_CMD;

typedef struct bbc_senor_ctrl{
	unsigned char lamp_sta_r;
	unsigned char lamp_sta_g;
	unsigned char lamp_sta_b;
	unsigned int brigR;
	unsigned int brigG;
	unsigned int brigB;
	unsigned char motor;
	unsigned int speed;
}senor_ctrl;

typedef struct bbc_ota_ctrl{
	char ota_ver[8];
	char ota_pack_url[120];
	char ota_pack_md5[32];
	uint32_t ota_pack_size;
}bbc_ota_msg;

extern senor_ctrl LampSet;
extern bbc_ota_msg BbcOtaMsg;
extern unsigned char BbcOperType;

int msg_parse_task_init();

#ifdef __cplusplus
}
#endif

#endif /* BBC_CJSON_ANALY_H_ */
