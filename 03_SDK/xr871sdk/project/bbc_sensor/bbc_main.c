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
#include "sys/ota.h"

#include "manip_ctrl.h"
#include "indic_plat.h"
#include "bbc_main.h"
#include "cjson_analy.h"
#include "mqtt_build.h"
#include "led_flag.h"

#define BBC_MAIN_DBG_SET 	1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define BBC_MAIN_DBG(fmt, arg...)	\
			LOG(BBC_MAIN_DBG_SET, "[BBC_MAIN_DBG] "fmt, ##arg)


senor_upload SenorUpload;
DEV_DATA_CALLBACK DevCallBackFlag;
unsigned int bbc_rgb_r_set;
unsigned int bbc_rgb_g_set;
unsigned int bbc_rgb_b_set;
unsigned int bbc_motor_set;

void senor_funct_call_init()
{
	senor_init();
	plat_ctrl_init();
}

void bbc_xr871senor_ctrl()
{
	if(LampSet.lamp_sta_r == BBC_LAMP_ON)
	{
		bbc_rgb_r_set = LampSet.brigR * RGB_CONVERT_BMSG;
		DevCallBackFlag.RGB_R_CALLBACK_FLAG = DEV_CALL_BACK;
		LampSet.lamp_sta_r = BBC_LAMP_OFF;
	}
	if(LampSet.lamp_sta_g == BBC_LAMP_ON)
	{
		bbc_rgb_g_set = LampSet.brigG * RGB_CONVERT_BMSG;		//48000 /255
		DevCallBackFlag.RGB_G_CALLBACK_FLAG = DEV_CALL_BACK;
		LampSet.lamp_sta_g = BBC_LAMP_OFF;
	}
	if(LampSet.lamp_sta_b == BBC_LAMP_ON)
	{
		bbc_rgb_b_set = LampSet.brigB * RGB_CONVERT_BMSG;		//48000 /255
		DevCallBackFlag.RGB_B_CALLBACK_FLAG = DEV_CALL_BACK;
		LampSet.lamp_sta_b = BBC_LAMP_OFF;
	}

	if(LampSet.motor == BBC_MOTOR_ON)
	{
		bbc_motor_set 	= LampSet.speed * MOTOR_CONVERT_BMSG;		//2400 /10
		DevCallBackFlag.MOTOR_CALLBACK_FLAG = DEV_CALL_BACK;
		LampSet.motor = BBC_MOTOR_OFF;
	}

	plat_rgbr_ctrl(bbc_rgb_r_set);
	plat_rgbg_ctrl(bbc_rgb_g_set);
	plat_rgbb_ctrl(bbc_rgb_b_set);
	plat_mortor_ctrl(bbc_motor_set);
}

void bbc_xr871data_get()
{
	int temp,hum,brig;
	temp 	= bbc_bme280_temp();
	hum 	= bbc_bme280_humd();
	brig 	= bbc_set_bright();
	
	SenorUpload.Bme280Tem 		= (float)temp / 100.0;
	SenorUpload.Bme280Hum 		= (float)hum / 1000.0;
	SenorUpload.PhotoSensit 	= brig;
}

char *devguid = devguid_get;

void pub_device_data(void)
{
	cJSON *upload 	= cJSON_CreateObject();
	cJSON *jData 	= cJSON_CreateObject();
	cJSON *jdps 	= cJSON_CreateObject();

	bbc_xr871data_get();
	
	cJSON_AddItemToObject(upload,"fcode", 	cJSON_CreateNumber(1));
	cJSON_AddItemToObject(upload,"ts", 		cJSON_CreateString("1459168450"));
	cJSON_AddItemToObject(jData,"devId", 		cJSON_CreateString(devguid));
	
	cJSON_AddItemToObject(jdps,"envBrightV", 	cJSON_CreateNumber(SenorUpload.PhotoSensit));
	cJSON_AddItemToObject(jdps,"humidity", 	cJSON_CreateNumber(SenorUpload.Bme280Hum));
	cJSON_AddItemToObject(jdps,"temp", 		cJSON_CreateNumber(SenorUpload.Bme280Tem));

	cJSON_AddItemToObject(jdps,"ledR",	cJSON_CreateNumber(SenorUpload.LampRBright));
	cJSON_AddItemToObject(jdps,"ledG", 	cJSON_CreateNumber(SenorUpload.LampGBright));
	cJSON_AddItemToObject(jdps,"ledB",	cJSON_CreateNumber(SenorUpload.LampBBright));
	cJSON_AddItemToObject(jdps,"motorRS",	cJSON_CreateNumber(SenorUpload.MotorSpeed));

	cJSON_AddItemToObject(jData,"dps",jdps);
	cJSON_AddItemToObject(upload,"data",jData);

	char *data_str = cJSON_PrintUnformatted(upload);

	BBC_MAIN_DBG("data_str length = %d\n",strlen(data_str));
	sprintf((char*)BbcPubSet,"%s",data_str);
	BBC_MAIN_DBG("BbcPubSet = %s\n",BbcPubSet);
	cal_set.MqttPub = MQTT_CACK;

	if(upload) cJSON_Delete(upload);
	if(data_str) free(data_str);
}

void pub_devctrl_callback(void)
{
	cJSON *upload 	= cJSON_CreateObject();
	cJSON *jData 	= cJSON_CreateObject();
	cJSON *jdps 	= cJSON_CreateObject();

	cJSON_AddItemToObject(upload,"fcode",	cJSON_CreateNumber(1));
	cJSON_AddItemToObject(upload,"ts", 	cJSON_CreateString("1459168450"));
	cJSON_AddItemToObject(jData,"devId", 	cJSON_CreateString(devguid));

	if(DevCallBackFlag.RGB_R_CALLBACK_FLAG == DEV_CALL_BACK) {
		cJSON_AddItemToObject(jdps,"ledR",	cJSON_CreateNumber(SenorUpload.LampRBright));
		DevCallBackFlag.RGB_R_CALLBACK_FLAG = DEV_CALL_DI_BACK;
	}

	if(DevCallBackFlag.RGB_G_CALLBACK_FLAG == DEV_CALL_BACK) {
		cJSON_AddItemToObject(jdps,"ledG", 	cJSON_CreateNumber(SenorUpload.LampGBright));
		DevCallBackFlag.RGB_G_CALLBACK_FLAG = DEV_CALL_DI_BACK;
	}
	if(DevCallBackFlag.RGB_B_CALLBACK_FLAG == DEV_CALL_BACK) {
		cJSON_AddItemToObject(jdps,"ledB",	cJSON_CreateNumber(SenorUpload.LampBBright));
		DevCallBackFlag.RGB_B_CALLBACK_FLAG = DEV_CALL_DI_BACK;
	}
	if(DevCallBackFlag.MOTOR_CALLBACK_FLAG == DEV_CALL_BACK) {
		cJSON_AddItemToObject(jdps,"motorRS",	cJSON_CreateNumber(SenorUpload.MotorSpeed));
		DevCallBackFlag.MOTOR_CALLBACK_FLAG = DEV_CALL_DI_BACK;
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

void senor_board_start_init(void)
{
	bbc_rgb_r_set = 1000;
	bbc_rgb_g_set = 1000;
	bbc_rgb_b_set = 1000;
	bbc_motor_set = 240;
	plat_rgbr_ctrl(bbc_rgb_r_set);
	plat_rgbg_ctrl(bbc_rgb_g_set);
	plat_rgbb_ctrl(bbc_rgb_b_set);
	plat_mortor_ctrl(bbc_motor_set);
}

void bbc_ota_set(void)
{
	//char ota_cmd[124];

	//extern void main_cmd_exec(char *cmd);
	//sprintf((char*)ota_cmd,"ota %s",BbcOtaMsg.ota_pack_url);
	//printf("ota_cmd = %s\n",ota_cmd);
	//main_cmd_exec(ota_cmd);

	if(ota_update(BbcOtaMsg.ota_pack_url) != OTA_STATUS_OK) 
	{
		BBC_MAIN_DBG("OTA Updata Fail!\n");
		led_mode = LED_FLAG_OTA_FAIL;
	}
}

#define OPER_THREAD_STACK_SIZE	1024 * 2
OS_Thread_t oper_task_thread;
uint8_t oper_task_run = 0;

void plat_oper_set()
{
	while(oper_task_run) {
		OS_MSleep(200);
		switch(BbcOperType) {
			case BBC_DO_NONE:	
				break;
			case BBC_UP_LOAD: 
				break;
			case BBC_SET_CTRL:
				BBC_MAIN_DBG("device operation set\n");
				bbc_xr871senor_ctrl();
				pub_devctrl_callback();		//data call back
				break;
			case GET_BBC_TIME:
				break;
			case BBC_REQ_MSG:
				BBC_MAIN_DBG("device data up load\n");
				pub_device_data();
				break;
			case BBC_PUB_OTA:
				bbc_ota_set();
				break;
			default :
				break;
		}
		BbcOperType = BBC_DO_NONE;
	}
	BBC_MAIN_DBG("plat_oper_set end\n");
	OS_ThreadDelete(&oper_task_thread);
}

int bbc_senor_task_init()
{
	msg_parse_task_init();		//sub message parse, get next operation
	senor_funct_call_init();
	senor_board_start_init();
	//pub_device_data();

	oper_task_run = 1;

	if (OS_ThreadCreate(&oper_task_thread,
		                "bbc_main",
		                plat_oper_set,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                OPER_THREAD_STACK_SIZE) != OS_OK) {
		BBC_MAIN_DBG("thread create error\n");
		return -1;
	}
	BBC_MAIN_DBG("bbc_senor_task_init end\n");
	
	return 0;
}

