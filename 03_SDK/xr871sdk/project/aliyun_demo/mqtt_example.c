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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kernel/os/os.h"

#include "net/cloud/aliyun/mqtt_client.h"
#include "net/cloud/aliyun/ca.h"

#include "mqtt_example.h"


#define aliot_debug_on      1
#define aliot_mes_call_on   1

#define ALIOT_LOG(flags, fmt, arg...)   \
    do {                                \
        if (flags)                      \
            printf(fmt, ##arg);         \
    } while (0)

#define ALIOT_DEBUG(fmt, arg...)   \
    ALIOT_LOG(aliot_debug_on, "[AliotDebug] "fmt, ##arg)

#define ALIOT_PRINT(fmt, arg...)   \
    ALIOT_LOG(aliot_mes_call_on, "[ALIOT_Print] "fmt, ##arg)

//message for aliyun,define product key, device name, devcie secret
#define PRODUCT_KEY			"vo84Hm3xbUj"
#define DEVICE_NAME			"xr871_senor_set0"
#define DEVICE_SECRET			"I8fMIYnC38gax3Rj2Q9EuCqbHTiCiI7S"
#define DEV_CLIENT_ID			"12345"

//define secure mode ;  2 : tls connect mode, 3: tcp stright connect mode
#define MQTT_TCP_MODE			"3"
#define MQTT_TLS_MODE			"2"

#define SECURE_MODE			MQTT_TLS_MODE

//define use mqtt stright mode or https get mqtt para mode
//#define USE_HTTPS_GET_PARA

//This is the pre-defined topic
#define TOPIC_UPDATE 		"/"PRODUCT_KEY"/"DEVICE_NAME"/update"
#define TOPIC_ERROR 		"/"PRODUCT_KEY"/"DEVICE_NAME"/update/error"
#define TOPIC_GET			"/"PRODUCT_KEY"/"DEVICE_NAME"/get"

//NOTE!!! you must define the following topic in IOT consle before running this sample.
#define TOPIC_DATA			"/"PRODUCT_KEY"/"DEVICE_NAME"/data"

#define TOPIC_SUB_DATA		TOPIC_DATA
#define TOPIC_PUB_DATA		TOPIC_DATA


#define mqtt_buff_size		350
unsigned char BbcSubGet[mqtt_buff_size] = {0};
unsigned char BbcPubSet[mqtt_buff_size] = {0};

mqt_cal cal_set;
uint8_t MessageArriveFlag = 0;

void messageArrived(MessageData* data)
{
	ALIOT_PRINT("Message arrived on topic : %.*s: %.*s\n",
				data->topicName->lenstring.len,
				data->topicName->lenstring.data,
				data->message->payloadlen,
				(char *)data->message->payload);

	memcpy(BbcSubGet,(char *)data->message->payload,data->message->payloadlen);
	MessageArriveFlag = MES_ARVE;
}

#define MQTT_THREAD_STACK_SIZE	(1024 * 5)
OS_Thread_t mqtt_ctrl_task_thread;
char mqtt_ctrl = 0;

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
		mqtt_con_rsp_times = 0;
		cal_set.mqtt_con = MQTT_CACK;
		cal_set.mqtt_sub = MQTT_CACK;
	}
}

void mqtt_reconnect(void)
{
	mqtt_con_nums ++;

	if(mqtt_con_nums < 3) {
		OS_Sleep(3);
	}
	if(mqtt_con_nums > 100) {
		mqtt_con_nums = 10;
		OS_Sleep(60);
	}
}

#define MAX_MQTT_BUFF_SIZE		500

void mqtt_work_set()
{
	int rc = 0;
	Client client;
	char mqt_sev_chk = 0;
	MQTTPacket_connectData connectData = MQTTPacket_connectData_initializer;
	unsigned char *sendbuf = NULL;
	unsigned char *readbuf = NULL;

	if(NULL == (sendbuf = (unsigned char*)malloc(MAX_MQTT_BUFF_SIZE))) {
		ALIOT_DEBUG("not enough memory in mqtt sendbuf!\n");
		goto mqtt_exit;
	}

	if(NULL == (readbuf = (unsigned char*)malloc(MAX_MQTT_BUFF_SIZE))) {
		ALIOT_DEBUG("not enough memory in mqtt readbuf!\n");
		goto mqtt_exit;
	}

	alink_devc_init();
	mqtt_para_init();

#ifdef USE_HTTPS_GET_PARA
	rc = alink_https_set_device_info(PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET, DEV_CLIENT_ID);
	if(rc == -1) {
		ALIOT_DEBUG("alink set device para error!\n");
		goto mqtt_exit;
	}
#else
	aliot_device_info(PRODUCT_KEY, DEVICE_NAME, DEVICE_SECRET, DEV_CLIENT_ID, SECURE_MODE);
#endif

	/* push para */
	MQTTMessage message;
	message.qos = XR_MQTT_QOS1;
	message.retained = 0;

	/* connect para */
	connectData.MQTTVersion 		= 4;
	connectData.keepAliveInterval 	= 60;
	connectData.cleansession		= 0;
	connectData.clientID.cstring 	= xr_devc_info.client_id;
	connectData.username.cstring 	= xr_devc_info.user_name;
   	connectData.password.cstring 	= xr_devc_info.password;

	/* mqtt para */
	xr_mqtt_para.command_timeout_ms = 30000;
	xr_mqtt_para.read_buf = readbuf;
	xr_mqtt_para.send_buf = sendbuf;
	xr_mqtt_para.read_buf_size = MAX_MQTT_BUFF_SIZE;
	xr_mqtt_para.send_buf_size = MAX_MQTT_BUFF_SIZE;
	xr_mqtt_para.port	= xr_devc_info.port;
	strncpy(xr_mqtt_para.host_name, xr_devc_info.host_name, strlen(xr_devc_info.host_name));

	while(mqtt_ctrl)
	{
		OS_MSleep(500);
		sprintf((char*)BbcPubSet, "%s", "lwq_test...");
		cal_set.mqtt_pub = MQTT_CACK;

		if(cal_set.mqtt_con == MQTT_CACK) {
			if(0 == memcmp(SECURE_MODE, MQTT_TCP_MODE, 1))
				rc = tcp_mqtt_client(&client);
			if(0 == memcmp(SECURE_MODE, MQTT_TLS_MODE, 1))
				rc = ssl_mqtt_client(&client, alink_ca_get(), strlen(alink_ca_get())+1);
			if (rc  != 0) {
				mqtt_reconnect();
				continue;
			}
			rc = MQTTConnect(&client, &connectData);
			if (rc != 0) {
				ALIOT_PRINT("MQTT client connect error! Return error code is %d\n", rc);
				continue;
			}
			ALIOT_PRINT("MQTT Connected\n");
			cal_set.mqtt_con = MQTT_DICACK;
		}

		if(cal_set.mqtt_sub == MQTT_CACK) {
			rc = MQTTSubscribe(&client, TOPIC_SUB_DATA, XR_MQTT_QOS1, messageArrived);
			if (rc != 0) {
				ALIOT_PRINT("Return code from MQTT subscribe is %d\n", rc);
				cal_set.mqtt_con = MQTT_CACK;
				continue;
			}
			ALIOT_PRINT("MQTT Subscrible is success\n");
			cal_set.mqtt_sub = MQTT_DICACK;
		}
		if(cal_set.mqtt_pub == MQTT_CACK) {
			message.payload = BbcPubSet;
			message.payloadlen = strlen((char*)BbcPubSet);

			rc = MQTTPublish(&client, TOPIC_PUB_DATA, &message);
			if (rc != 0) {
				ALIOT_PRINT("Return code from MQTT publish is %d\n", rc);
				cal_set.mqtt_con = MQTT_CACK;
				cal_set.mqtt_sub = MQTT_CACK;
				continue;
			}
			ALIOT_PRINT("MQTT publish is success\n");
			memset(BbcPubSet, 0, mqtt_buff_size);
			cal_set.mqtt_pub = MQTT_DICACK;
		}
		if(cal_set.mqtt_quit== MQTT_CACK) {
			rc = MQTTUnsubscribe(&client, TOPIC_SUB_DATA);
			if (rc  != 0)
				ALIOT_PRINT("Return code from MQTT unsubscribe is %d\n", rc);
			rc = MQTTDisconnect(&client);
			if (rc != 0)
				ALIOT_PRINT("Return code from MQTT disconnect is %d\n", rc);
			cal_set.mqtt_quit = MQTT_DICACK;
		}
		rc = MQTTYield(&client, 3000);
		if (rc != 0)
			ALIOT_PRINT("Return code from yield is %d\n", rc);

		mqtt_con_nums = 0;
		mqt_sev_chk = client.ping_outstanding;
		check_mqtt_server(mqt_sev_chk);
	}

mqtt_exit:
	if(NULL != sendbuf) {
		free(sendbuf);
	}

	if(NULL != readbuf) {
		free(readbuf);
	}
	ALIOT_PRINT("mqtt_work_set end\n");
	OS_ThreadDelete(&mqtt_ctrl_task_thread);

}

int mqtt_ctrl_task_init()
{
	mqtt_ctrl = 1;
	if (OS_ThreadCreate(&mqtt_ctrl_task_thread,
		                "mqtt_build",
		                mqtt_work_set,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                MQTT_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create error\n");
		return -1;
	}
	ALIOT_DEBUG("end\n");
	return 0;
}

