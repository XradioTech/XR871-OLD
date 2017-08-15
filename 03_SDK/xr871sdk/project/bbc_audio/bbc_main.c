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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "kernel/os/os.h"

#include "bbc_main.h"
#include "cjson_analy.h"
#include "mqtt_build.h"

#define BBC_MAIN_DBG_SET 	1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define BBC_MAIN_DBG(fmt, arg...)	\
			LOG(BBC_MAIN_DBG_SET, "[BBC_MAIN_DBG] "fmt, ##arg)


audio_upload AudUpload;
Audio_DATA_CALLBACK AudioDataCall;

char *devguid = devguid_get;

void pub_audio_callback(void)
{
	cJSON *upload 	= cJSON_CreateObject();
	cJSON *jData 	= cJSON_CreateObject();
	cJSON *jdps 	= cJSON_CreateObject();

	cJSON_AddItemToObject(upload,"fcode",	cJSON_CreateNumber(1));
	cJSON_AddItemToObject(upload,"ts", 	cJSON_CreateString("1459168450"));
	cJSON_AddItemToObject(jData,"devId", 	cJSON_CreateString(devguid));

	if(AudioDataCall.STA_CALLBACK_FLAG == AUD_CALL_BACK) {
		cJSON_AddItemToObject(jdps,"musicS",	cJSON_CreateNumber(AudUpload.audio_play_sta));
		AudioDataCall.STA_CALLBACK_FLAG = AUD_CALL_DI_BACK;
	}

	if(AudioDataCall.VOL_CALLBACK_FLAG == AUD_CALL_BACK) {
		cJSON_AddItemToObject(jdps,"musicV", 	cJSON_CreateNumber(AudUpload.audio_play_vol));
		AudioDataCall.VOL_CALLBACK_FLAG = AUD_CALL_DI_BACK;
	}

	cJSON_AddItemToObject(jData,"dps",jdps);
	cJSON_AddItemToObject(upload,"data",jData);

	char *data_str = cJSON_PrintUnformatted(upload);
	sprintf((char*)BbcPubSet,"%s",data_str);
	BBC_MAIN_DBG("BbcPubSet = %s\n",BbcPubSet);
	cal_set.MqttPub = MQTT_CACK;

	if(upload) cJSON_Delete(upload);
	if(data_str) free(data_str);
}

void bbc_ota_get(void)
{
	char ota_cmd[124];
	
	extern void main_cmd_exec(char *cmd);
	sprintf((char*)ota_cmd,"ota %s",BbcOtaMsg.ota_pack_url);
	BBC_MAIN_DBG("ota_cmd = %s\n",ota_cmd);
	main_cmd_exec(ota_cmd);
}

#define OPER_THREAD_STACK_SIZE	1024 * 2
OS_Thread_t oper_task_thread;
uint8_t oper_task_run = 0;

void plat_opera_set()
{
	while(oper_task_run) {
		OS_MSleep(200);
		if(BbcOperType == BBC_REQ_MSG) {
			BBC_MAIN_DBG("device data up load\n");
			AudioDataCall.STA_CALLBACK_FLAG = AUD_CALL_BACK;
			AudioDataCall.VOL_CALLBACK_FLAG = AUD_CALL_BACK;
			pub_audio_callback();
		}
		if(BbcOperType == BBC_PUB_OTA) {
			bbc_ota_get();
		}
		BbcOperType = BBC_DO_NONE;
	}
	BBC_MAIN_DBG("plat_oper_set end\n");
	OS_ThreadDelete(&oper_task_thread);
}

int bbc_audio_task_init()
{
	oper_task_run = 1;
	msg_parse_task_init();	//json parse string
	
	if (OS_ThreadCreate(&oper_task_thread,
		                "bbc_main",
		                plat_opera_set,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                OPER_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create error\n");
		return -1;
	}
	printf("bbc_audio_task_init end\n");
	
	return 0;
}

