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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/chip/hal_norflash.h"
#include "net/mqtt/MQTTClient-C/MQTTClient.h"
#include "net/mbedtls/sha1.h"

#include "bbc/devguid_get.h"
#include "bbc/utils.h"

#include "led_flag.h"
#include "mqtt_build.h"
#include "cjson_analy.h"

#define MQTT_BUILD_DEG_SET 	1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define  MQTT_BUILD_DEBUG(fmt, arg...)	\
			LOG(MQTT_BUILD_DEG_SET, "[MQTT_BUILD_DEBUG] "fmt, ##arg)


//define device para
#define DEVICE_ID			"201707202029"
#define DEVICE_CLASS		"Tbu1XR871test2"
#define BBC_LICSENCE		"195e93278e40c16b0a3fb48cd60a5dd842f48f659989e01beafa48d04259652b63d78a7bbde91219"
#define PRODECT_KEY			"71823e60dd38419a"
#define BBC_SDK_VERSION		"0.8.3"

#define BBC_ADDRESS			"mqtt.bigbigon.com"
#define BBC_PORT			1883

//if your device register a error plalse re register
#define DI_REGISTER		0
#define ENREGISTER  	1

#define DO_REGISTER		ENREGISTER


unsigned char BbcSubGet[mqtt_buff_size] = {0};
unsigned char BbcPubSet[mqtt_buff_size] = {0};
char devguid_get[40] = {0};

unsigned char MessageArriveFlag = 0;
mqt_cal cal_set;
uint8_t mqtt_set_rcome = 0;

void messageArrived(MessageData* data)
{
	MQTT_BUILD_DEBUG("Message arrived on topic %.*s: %.*s\n", 
				data->topicName->lenstring.len, 
				data->topicName->lenstring.data,
				data->message->payloadlen, 
				(char *)data->message->payload);

	memcpy(BbcSubGet,(char *)data->message->payload,data->message->payloadlen);
	//printf("BbcSubGet =  %s\r\n",BbcSubGet);
	MessageArriveFlag = MES_ARVE;
}

void get_devguid_from_flash()
{
	bbc_inital(DO_REGISTER);		//if not register ,use http to get deviceguid ;(0 = dis-re - register; 1 = re -register )
	
	SF_Config flash_config;
	SF_Handler hdl;

	flash_config.spi = SPI0;
	flash_config.cs = SPI_TCTRL_SS_SEL_SS0;
	flash_config.dma_en = 1;
	flash_config.sclk = 12 * 1000 * 1000;

	HAL_SF_Init(&hdl, &flash_config);
	//HAL_SF_Erase(&hdl, SF_ERASE_SIZE_32KB, devguid_in_flash, 1);
	HAL_SF_Read(&hdl, devguid_in_flash, (uint8_t *)devguid_get, 40);
}


#define MQTT_THREAD_STACK_SIZE	(1024 * 6)
OS_Thread_t MQTT_ctrl_task_thread;
char MQTT_ctrl = 0;
uint8_t mqtt_con_nums = 0;
uint8_t mqtt_con_rsp_times = 0;

void check_mqtt_server(char out_stand)
{
	if(out_stand == 1) {
		mqtt_con_rsp_times ++;
	}
	else {
		mqtt_con_rsp_times = 0;
	}
	if(mqtt_con_rsp_times > 20) {
		led_mode = LED_FLAG_MQTDICON;
		mqtt_con_rsp_times = 0;
		cal_set.MqttCon = MQTT_CACK;
		cal_set.MqttSub = MQTT_CACK;
	}
}

unsigned char sendbuf[350] = {0}, readbuf[350] = {0};
void mqtt_work_set()
{
	Client client;
	Network network;
	int rc = 0;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
	unsigned char str_pwd[160] = {0};
	unsigned char pwd_sha1_out[64] = {0};
	char pwd_to_hex[64] = {0};
	char str_clientID[126] = {0};
	int pwd_leng = 0;
	char mqt_sev_chk;

	device_info(DEVICE_ID, DEVICE_CLASS, BBC_LICSENCE);
	get_devguid_from_flash();
	
	char sub_topic[50] = {0}; 
	char pub_topic[50] = {0};
	sprintf(sub_topic, "p2p/%s/cmd", devguid_get);
	sprintf(pub_topic, "p2p/%s/ntf", devguid_get);
	MQTT_BUILD_DEBUG("sub_topic = %s\n",sub_topic);
	MQTT_BUILD_DEBUG("pub_topic = %s\n",pub_topic);

	unsigned int sub_qos = 1;
	MQTTMessage message;
	message.qos = 1;
	message.retained = 0;

	pwd_leng = sprintf((char*)str_pwd,"%s%s%s",PRODECT_KEY, devguid_get, BBC_LICSENCE);
	str_pwd[pwd_leng] = '\0';
	sprintf(str_clientID,"device:%s:%s:%s",PRODECT_KEY, devguid_get, BBC_SDK_VERSION);
	mbedtls_sha1((unsigned char*)str_pwd, pwd_leng, pwd_sha1_out);
	to_hex_str((unsigned char*)pwd_sha1_out, (char*)pwd_to_hex, 20);

	MQTT_BUILD_DEBUG("pwd_leng = %d\n",pwd_leng);
	MQTT_BUILD_DEBUG("str_pwd = %s\n",str_pwd);
	MQTT_BUILD_DEBUG("pwd_to_hex = %s\n",pwd_to_hex);
	MQTT_BUILD_DEBUG("str_clientID = %s\n",str_clientID);

	//connect
	connectData.MQTTVersion 		= 4;
	connectData.keepAliveInterval 	= 60;
	connectData.cleansession		= 1;
	connectData.clientID.cstring 	= str_clientID;
	connectData.username.cstring 	= "luowq_senor";
   	connectData.password.cstring 	= pwd_to_hex;

	while(MQTT_ctrl) 
	{
		OS_MSleep(200);
		
		if(mqtt_set_rcome == 1) {
			if(cal_set.MqttCon == MQTT_CACK) 
			{
				NewNetwork(&network);
				MQTTClient(&client, &network, 30000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));
				rc = ConnectNetwork(&network, BBC_ADDRESS, BBC_PORT);
				if (rc  != 0) {
					MQTT_BUILD_DEBUG("Return code from network connect is %d\n", rc);
					MQTT_BUILD_DEBUG("MQTT net connect error!\r\n");
					mqtt_con_nums ++;
					if(mqtt_con_nums < 3) {
						OS_MSleep(1000);
						continue;
					}
					if(mqtt_con_nums < 10) {
						OS_Sleep(30);
						continue;
					}
					if(mqtt_con_nums > 100)	mqtt_con_nums = 15;
					OS_Sleep(60);
					continue;
				}
				else {
					mqtt_con_nums = 0;
				}
				rc = MQTTConnect(&client, &connectData);
				if (rc != 0) {
					MQTT_BUILD_DEBUG("Return code from MQTT connect is %d\n", rc);
					MQTT_BUILD_DEBUG("MQTT client connect error!\r\n");
					continue;
				}
				else {
					MQTT_BUILD_DEBUG("MQTT Connected\n");
					led_mode = LED_FLAG_MQTCON;
				}
				cal_set.MqttCon = MQTT_DICACK;
			}
			
			if(cal_set.MqttSub == MQTT_CACK) 
			{
				rc = MQTTSubscribe(&client, sub_topic, sub_qos, messageArrived);
				if (rc != 0) {
					MQTT_BUILD_DEBUG("Return code from MQTT subscribe is %d\n", rc);
					cal_set.MqttCon = MQTT_CACK;
					continue;
				}
				else {
					MQTT_BUILD_DEBUG("MQTT Subscrible is success\n");	
				}
				cal_set.MqttSub = MQTT_DICACK;
			}
			if(cal_set.MqttPub == MQTT_CACK)
			{
				message.payload = BbcPubSet;
				message.payloadlen = strlen((char*)BbcPubSet);
				
				rc = MQTTPublish(&client, pub_topic, &message);
				if (rc != 0) {
					MQTT_BUILD_DEBUG("Return code from MQTT publish is %d\n", rc);
					cal_set.MqttCon = MQTT_CACK;
					cal_set.MqttSub = MQTT_CACK;
					continue;
				}
				else {
					MQTT_BUILD_DEBUG("MQTT publish is success\n");
				}
				memset(BbcPubSet, 0, mqtt_buff_size);
				cal_set.MqttPub = MQTT_DICACK;
			}
			if(cal_set.MqttQuit== MQTT_CACK)
			{
				rc = MQTTUnsubscribe(&client, sub_topic);
				if (rc  != 0)
					MQTT_BUILD_DEBUG("Return code from MQTT unsubscribe is %d\n", rc);
				rc = MQTTDisconnect(&client);
				if (rc != 0)
					MQTT_BUILD_DEBUG("Return code from MQTT disconnect is %d\n", rc);
				cal_set.MqttQuit = MQTT_DICACK;
			}
			rc = MQTTYield(&client, 3000);
			if (rc != 0) {
				MQTT_BUILD_DEBUG("Return code from yield is %d\n", rc);
				cal_set.MqttCon = MQTT_CACK;
				cal_set.MqttSub = MQTT_CACK;
			}

			mqt_sev_chk = client.ping_outstanding;
			check_mqtt_server(mqt_sev_chk);
		}

	}

	MQTT_BUILD_DEBUG("mqtt_work_set end\n");
	OS_ThreadDelete(&MQTT_ctrl_task_thread);

}

int mqtt_ctrl_task_init()
{
	MQTT_ctrl = 1;
	if (OS_ThreadCreate(&MQTT_ctrl_task_thread,
		                "mqtt_build",
		                mqtt_work_set,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                MQTT_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create error\n");
		return -1;
	}
	printf("end\n");
	return 0;
}


