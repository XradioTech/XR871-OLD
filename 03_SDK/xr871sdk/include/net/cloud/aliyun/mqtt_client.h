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

#ifndef ALINK_MQTT_CLIENT_H_
#define ALINK_MQTT_CLIENT_H_

#include "net/mqtt/MQTTClient-C/MQTTClient.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HOST_ADDRESS_LEN    	(64)
#define HOST_PORT_LEN       	(8)
#define CLIENT_ID_LEN       	(128)
#define USER_NAME_LEN       	(32)
#define PASSWORD_LEN        	(40)

#define ALIOT_AUTH_IOT_ID       	32
#define ALIOT_AUTH_IOT_TOKEN    	32

typedef enum {
    XR_MQTT_QOS0 = 0,
    XR_MQTT_QOS1,
    XR_MQTT_QOS2
}xr_mqtt_qos_t;

typedef struct {
    char client_id[CLIENT_ID_LEN + 1];
    char user_name[USER_NAME_LEN + 1];
    char password[PASSWORD_LEN + 1];
	char host_name[HOST_ADDRESS_LEN + 1];
	uint16_t port;
} aliot_devc_info_t, *aliot_devc_info_pt;

typedef struct {

    uint16_t		port;
    char			host_name[HOST_ADDRESS_LEN + 1];

    uint32_t		command_timeout_ms;

    unsigned char	*send_buf;
    uint32_t		send_buf_size;
    unsigned char	*read_buf;
    uint32_t 		read_buf_size;

} xr_mqtt_param_t, *xr_mqtt_param_pt;

void alink_devc_init(void);
void mqtt_para_init(void);

void aliot_device_info(
						const char *product_key,
						const char *device_name,
						const char *device_secret,
						const char *dev_client_id,
						const char *secure_mode);

int alink_https_set_device_info(const char* product_key,
											 const char* devcice_name,
											 const char* device_secret,
											 const char *dev_client_id
											 );

int tcp_mqtt_client(Client* c);
int ssl_mqtt_client(Client* client, const char *ca_cert, uint32_t ca_length);

extern aliot_devc_info_t xr_devc_info;
extern xr_mqtt_param_t	xr_mqtt_para;

#ifdef __cplusplus
}
#endif

#endif
