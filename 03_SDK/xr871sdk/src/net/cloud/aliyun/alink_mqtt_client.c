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

#include "stdio.h"
#include <string.h>
#include <stdlib.h>

#include "alink_debug.h"
#include "net/mbedtls/md.h"

#include "net/cloud/aliyun/mqtt_client.h"
#include "alink_auth.h"
#include "alink_utils.h"


//set aliot mqtt addr and port,if use mqtt stright mode
#define ALIOT_HOST				"iot-as-mqtt.cn-shanghai.aliyuncs.com"
#define ALIOT_PORT				1883

aliot_devc_info_t xr_devc_info;
xr_mqtt_param_t	xr_mqtt_para;

void alink_devc_init(void)
{
	memset(&xr_devc_info, 0, sizeof(aliot_devc_info_t));
	ALINK_DBG("device init success!\n");
}

void mqtt_para_init(void)
{
	memset(&xr_mqtt_para, 0, sizeof(xr_mqtt_param_t));
	ALINK_DBG("mqtt para init success!\n");
}

void aliot_device_info( const char *product_key,
							   const char *device_name,
							   const char *device_secret,
						 	   const char *dev_client_id,
							   const char *secure_mode
							  )
{
	ALINK_DBG("start to set device info !\n");

	unsigned char str_pwd[128] = {0};
	char str_clientID[64] = {0};
	char str_user_name[64] = {0};
	char alink_mqtt_host[64] = {0};

	sprintf(str_clientID, "%s|securemode=%s,signmethod=hmacsha1,timestamp=789|",dev_client_id, secure_mode);
	sprintf(str_user_name, "%s&%s",device_name, product_key);

	uint8_t pwd_leng = 0;
	char pwd_sha1_out[64] = {0};
	char pwd_to_hex[64] = {0};
	pwd_leng = sprintf((char*)str_pwd,"clientId%sdeviceName%sproductKey%stimestamp%s",
									dev_client_id, device_name, product_key,"789");
	str_pwd[pwd_leng] = '\0';

	const mbedtls_md_info_t *res_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
	mbedtls_md_hmac(res_info, (const unsigned char*)device_secret, 32, \
					(const unsigned char*)str_pwd, pwd_leng, (unsigned char*)pwd_sha1_out);

	to_hex_str((unsigned char*)pwd_sha1_out, (char*)pwd_to_hex, 20);

	//host = product.iot-as-mqtt.cn-shanghai.aliyuncs.com
	sprintf(alink_mqtt_host, "%s.%s", product_key, ALIOT_HOST);

	strncpy(xr_devc_info.client_id, str_clientID, strlen(str_clientID));
	strncpy(xr_devc_info.user_name, str_user_name, strlen(str_user_name));
	strncpy(xr_devc_info.password, pwd_to_hex,40);
	strncpy(xr_devc_info.host_name, alink_mqtt_host, strlen(alink_mqtt_host));
	xr_devc_info.port = ALIOT_PORT;

	ALINK_DBG("str_clientID = %s\n",xr_devc_info.client_id);
	ALINK_DBG("str_user_name = %s\n",xr_devc_info.user_name);
	ALINK_DBG("sha1_pwd = %s\n",xr_devc_info.password);
}

int alink_https_set_device_info(const char* product_key,
											 const char* devcice_name,
											 const char* device_secret,
											 const char *dev_client_id
											 )
{
	char iot_id[ALIOT_AUTH_IOT_ID + 1], iot_token[ALIOT_AUTH_IOT_TOKEN + 1], host[HOST_ADDRESS_LEN + 1];
	uint16_t port = 0;
	int ret = 0;

	ret = https_mqtt_para_get(iot_id, iot_token, host, &port,
								product_key,
								devcice_name,
								device_secret,
								dev_client_id
								);
	if(ret == -1) {
		ALINK_DBG("https get mqtt paras error!\n");
		return -1;
	}

	strncpy(xr_devc_info.client_id, dev_client_id, strlen(dev_client_id));
	strncpy(xr_devc_info.user_name, iot_id, strlen(iot_id));
	strncpy(xr_devc_info.password, iot_token, strlen(iot_token));
	strncpy(xr_devc_info.host_name, host, strlen(host));
	xr_devc_info.port = port;

	return ret;
}

int tcp_mqtt_client(Client* client)
{
	int rc;
	Network *network = NULL;

	network = (Network *)malloc(sizeof(Network));
	NewNetwork(network);

	rc = ConnectNetwork(network, xr_mqtt_para.host_name, xr_mqtt_para.port);
	if(rc != 0) {
		ALINK_PRINF("MQTT net connect error! Return error code is %d\n", rc);
		return -1;
	}

	int i;
    client->ipstack = network;

    for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
        client->messageHandlers[i].topicFilter = 0;

    client->command_timeout_ms = xr_mqtt_para.command_timeout_ms;
    client->buf = xr_mqtt_para.send_buf;
    client->buf_size = xr_mqtt_para.send_buf_size;
    client->readbuf = xr_mqtt_para.read_buf;
    client->readbuf_size = xr_mqtt_para.read_buf_size;
    client->isconnected = 0;
    client->ping_outstanding = 0;
    client->defaultMessageHandler = NULL;
    InitTimer(&client->ping_timer);

#ifdef PACKET_SPLICE_BUGFIX
    client->remain_pktfrag_len = 0;
#endif

	return 0;
}

int ssl_mqtt_client(Client* client, const char *ca_cert, uint32_t ca_length)
{
	int rc;
	Network *network = NULL;

	network = (Network *)malloc(sizeof(Network));
	NewNetwork(network);

	char port_get[8];
	sprintf(port_get, "%u", xr_mqtt_para.port);
	rc = mqtt_ssl_establish(network, xr_mqtt_para.host_name, port_get, ca_cert, ca_length);
	if(rc != 0) {
		ALINK_PRINF("MQTT tls connect error! Return error code is %d\n", rc);
		return -1;
	}

	int i;
    client->ipstack = network;

    for (i = 0; i < MAX_MESSAGE_HANDLERS; ++i)
        client->messageHandlers[i].topicFilter = 0;

    client->command_timeout_ms = xr_mqtt_para.command_timeout_ms;
    client->buf = xr_mqtt_para.send_buf;
    client->buf_size = xr_mqtt_para.send_buf_size;
    client->readbuf = xr_mqtt_para.read_buf;
    client->readbuf_size = xr_mqtt_para.read_buf_size;
    client->isconnected = 0;
    client->ping_outstanding = 0;
    client->defaultMessageHandler = NULL;
    InitTimer(&client->ping_timer);

#ifdef PACKET_SPLICE_BUGFIX
    client->remain_pktfrag_len = 0;
#endif

	return 0;
}

