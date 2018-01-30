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

#include "alink_cjson_get.h"
#include "alink_debug.h"


char* alink_cjson_get_iot_para(const char *cjson_explain, const char *get_name)
{
	cJSON* json_message = NULL, *get_data = NULL, *get_iotId = NULL;

	static char para_get[64] = {0};

	json_message = cJSON_Parse(cjson_explain);
	if (!json_message) {
		ALINK_DBG("cjson get json_message error\n");
		goto cjosn_exit;
	}

	get_data = cJSON_GetObjectItem(json_message, "data");
	if(!get_data) {
		ALINK_DBG("cjson get data error\n");
		goto cjosn_exit;
	}

	get_iotId = cJSON_GetObjectItem(get_data, get_name);
	if(!get_iotId) {
		ALINK_DBG("cjson get_iotId error\n");
		goto cjosn_exit;
	}
	if(get_iotId->type == cJSON_String) {
		ALINK_DBG("cjson get_iotId = %s\n", get_iotId->valuestring);
		memcpy(para_get, get_iotId->valuestring, strlen(get_iotId->valuestring)+1);
	}

cjosn_exit:

	if(json_message) cJSON_Delete(json_message);

	return para_get;
}

char* alink_cjson_get_mqtt_addr(const char *cjson_explain, const char *get_name)
{
	cJSON* json_message = NULL, *get_data = NULL, *resourece = NULL, *mqtt = NULL, *addr = NULL;

	static char para_get[64] = {0};

	json_message = cJSON_Parse(cjson_explain);
	if (!json_message) {
		ALINK_DBG("cjson get json_message error\n");
		goto cjosn_exit;
	}

	get_data = cJSON_GetObjectItem(json_message, "data");
	if(!get_data) {
		ALINK_DBG("cjson get data error\n");
		goto cjosn_exit;
	}

	resourece = cJSON_GetObjectItem(get_data, "resources");
	if(!resourece) {
		ALINK_DBG("cjson get resourece error\n");
		goto cjosn_exit;
	}

	mqtt = cJSON_GetObjectItem(resourece, "mqtt");
	if(!resourece) {
		ALINK_DBG("cjson get mqtt error\n");
		goto cjosn_exit;
	}

	addr = cJSON_GetObjectItem(mqtt, get_name);
	if(!resourece) {
		ALINK_DBG("cjson get resourece error\n");
		goto cjosn_exit;
	}
	if(addr->type == cJSON_String) {
		ALINK_DBG("cjson addr = %s\n", addr->valuestring);
		memcpy(para_get, addr->valuestring, strlen(addr->valuestring)+1);
	}
	if(addr->type == cJSON_Number) {
		ALINK_DBG("cjson addr = %d\n", addr->valueint);
		char port[6] = {0};
		sprintf(port, "%d", addr->valueint);
		memcpy(para_get, port, strlen(port) + 1);
	}

cjosn_exit:

	if(json_message) cJSON_Delete(json_message);

	return para_get;
}


