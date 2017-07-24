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

#include "kernel/os/os.h"
#include "cjson_analy.h"
#include "stdio.h"

music_ctrl_set MusicCtrlSet;
bbc_ota_msg BbcOtaMsg;
unsigned char BbcOperType = 0;

void json_fcode2_music_analy(cJSON* get_data)
{
	cJSON* dps = NULL;
	cJSON* play_statu = NULL, *play_opra = NULL, *play_vol = NULL;
	
	dps = cJSON_GetObjectItem(get_data,"dps");
	
	play_statu = cJSON_GetObjectItem(dps,"musicS");
	if(play_statu) {
		MusicCtrlSet.play_fun_flag = PLAY_STA_ON;
		if(play_statu->type == cJSON_Number) {	  
			printf("play_statu =  %d\r\n",play_statu->valueint);
			if(play_statu->valueint) 
				MusicCtrlSet.play_funct = BBC_MUSIC_PLAY;
			else
				MusicCtrlSet.play_funct = BBC_MUSIC_PUSH;
		}	
	}
	play_opra = cJSON_GetObjectItem(dps,"musicC");
	if(play_opra) {
		MusicCtrlSet.play_fun_flag = PLAY_STA_ON;
		if(play_opra->type == cJSON_String)	{	  
			printf("play_opra =  %s\r\n",play_opra->valuestring);
			if(!memcmp("next", play_opra->valuestring, 4))
				MusicCtrlSet.play_funct = BBC_MUSIC_NEXT;
			else
				MusicCtrlSet.play_funct = BBC_MUSIC_PRE;
		}		
	}
	play_vol = cJSON_GetObjectItem(dps,"musicV");
	if(play_vol) {
		MusicCtrlSet.play_vol_flag = PLAY_STA_ON;
		if(play_vol->type == cJSON_Number)	{	 
			printf("play_vol =  %d\r\n",play_vol->valueint);
			MusicCtrlSet.play_vol = play_vol->valueint;
		}	
	}	
}

void json_fcode5_ota_analy(cJSON* get_data)
{
	cJSON* NewVersion = NULL;
	cJSON* PackData = NULL;
	cJSON* PackDataEle = NULL;
	cJSON* PackUrl = NULL, *PackMD5 = NULL; //*idx = NULL;

	NewVersion = cJSON_GetObjectItem(get_data,"newVersion");
	if(NewVersion) {
		if(NewVersion->type == cJSON_String) {
			printf("OTA NewVersion = %s\n",NewVersion->valuestring);
			sprintf(BbcOtaMsg.ota_ver,"%s",NewVersion->valuestring);
		}
	}
	
	uint8_t arry_size;
	PackData = cJSON_GetObjectItem(get_data,"packs");
	if(PackData) {
		arry_size = cJSON_GetArraySize(PackData);
		printf("arry_size = %d\n",arry_size);
		//only a pack. analy only pack,if want analy more array,you should change it
		PackDataEle = cJSON_GetArrayItem(PackData, 0);
		if(PackDataEle) {
			PackUrl = cJSON_GetObjectItem(PackDataEle,"url");
			if(PackUrl) {
				if(PackUrl->type == cJSON_String) {
					printf("PackUrl = %s\n",PackUrl->valuestring);
					sprintf(BbcOtaMsg.ota_pack_url,"%s",PackUrl->valuestring);
				}
			}
			PackMD5 = cJSON_GetObjectItem(PackDataEle,"packMD5");
			if(PackMD5) {
				if(PackMD5->type == cJSON_String) {
					printf("PackMD5 = %s\n",PackMD5->valuestring);
					sprintf(BbcOtaMsg.ota_pack_md5, "%s", PackMD5->valuestring);
				}
			}
		}
	}
}


void json_plat_analy(char * get_statue)
{
	cJSON* json_message = NULL, *cmd = NULL, *data = NULL;
	
	json_message = cJSON_Parse(get_statue);
	if (!json_message) {  
		printf("cjson Error before: [%s]\n",cJSON_GetErrorPtr());  
	}
	else  
	{	
		//解析命令	
		cmd  = cJSON_GetObjectItem(json_message, "fcode");
		if(cmd) {
			if(cmd->type == cJSON_Number)
			{
				printf("cmd type = %d\n",cmd->valueint);
				BbcOperType = cmd->valueint;
			}
		}
		// 解析开关值
		data = cJSON_GetObjectItem(json_message, "data");
		
		switch(BbcOperType) {
			case BBC_DO_NONE:
				break;
			case BBC_UP_LOAD:
				break;
			case BBC_SET_CTRL:
				json_fcode2_music_analy(data);
				break;
			case GET_BBC_TIME:
				break;
			case BBC_REQ_MSG:
				break;
			case BBC_PUB_OTA:
				json_fcode5_ota_analy(data);
				break;
			default:
				break;
		}
	}

	if(json_message) cJSON_Delete(json_message);
	memset(BbcSubGet, 0, mqtt_buff_size);
}

#define PARSE_THREAD_STACK_SIZE		1024 * 3
OS_Thread_t parse_task_thread;
char parse_set = 0;

void msg_parse(void)
{
	while(parse_set) {
		OS_MSleep(200);
		if(MessageArriveFlag == MES_ARVE) 
		{
			printf("Enerty Sub Flag\r\n");
			json_plat_analy(BbcSubGet);
			//json_analy_test(BbcSubGet);
			MessageArriveFlag = MES_FLAG_Clean;
		}
	}
	printf("msg_parse end\n");
	OS_ThreadDelete(&parse_task_thread);
}

int msg_parse_task_init()
{
	parse_set = 1;

	if (OS_ThreadCreate(&parse_task_thread,
		                "cjson_analy",
		                msg_parse,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                PARSE_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create error\n");
		return -1;
	}
	printf("end\n");
	
	return 0;
}

