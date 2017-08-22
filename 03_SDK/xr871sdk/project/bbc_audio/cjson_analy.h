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
	BBC_MUSIC_NONE_OP	= 0,
	BBC_MUSIC_PUSH 	= 1,
	BBC_MUSIC_PLAY	 	= 2,
	BBC_MUSIC_NEXT		= 3,
	BBC_MUSIC_PRE		= 4
} MUSIC_PLAY_STA;

typedef enum {
	PLAY_STA_OFF 	= 1,
	PLAY_STA_ON	= 2
} PLAY_FUCTION_FLAG;

typedef enum {
	BBC_DO_NONE 	= 0,
	BBC_UP_LOAD	= 1,
	BBC_SET_CTRL	= 2,
	GET_BBC_TIME	= 3,
	BBC_REQ_MSG	= 4,
	BBC_PUB_OTA	= 5
}OPER_CMD;

typedef struct bbc_music_ctrl{
	unsigned char play_fun_flag;
	unsigned char play_funct;
	unsigned char play_vol_flag;
	unsigned char play_vol;
}music_ctrl_set;

typedef struct bbc_ota_ctrl{
	char ota_ver[8];
	char ota_pack_url[120];
	char ota_pack_md5[32];
	uint32_t ota_pack_size;
}bbc_ota_msg;

extern music_ctrl_set MusicCtrlSet;
extern bbc_ota_msg BbcOtaMsg;
extern unsigned char BbcOperType;

int msg_parse_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BBC_CJSON_ANALY_H_ */
