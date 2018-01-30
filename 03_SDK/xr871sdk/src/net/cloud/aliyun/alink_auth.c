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

#include "net/HTTPClient/HTTPCUsr_api.h"
#include "net/mbedtls/md.h"
#include "net/mbedtls/mbedtls.h"

#include "net/cloud/aliyun/ca.h"
#include "alink_cjson_get.h"
#include "alink_auth.h"
#include "alink_utils.h"
#include "alink_debug.h"

#define IOT_AUTH_HOSTNAME  	"https://iot-auth.cn-shanghai.aliyuncs.com/auth/devicename"

char https_resp_data[1024] = {0};

security_client alink_user_param;

void* alink_https_get_certs(void)
{
	memset(&alink_user_param, 0, sizeof(alink_user_param));
	alink_user_param.pCa = (char *)alink_ca_get();
	alink_user_param.nCa = strlen(alink_ca_get()) + 1;

	return &alink_user_param;
}

void* alink_get_heads(void)
{
	static char *headers = "Content-Type:application/x-www-form-urlencoded;charset=utf-8&\
							Accept: text/xml,text/javascript,text/html,application/json";
	return headers;
}

static int alink_https_post(HTTPParameters *clientParams, char *post_buf, uint32_t max_post_length, uint32_t max_resp_length)
{
	int ret = 0;
	unsigned int toReadLength = max_post_length;
	unsigned int Received = 0;
	char *buf = malloc(toReadLength);
	if (buf == NULL)
		ALINK_DBG("malloc pbuffer failed..\n");

	HTTP_CLIENT httpClient;
	memset((void *)&httpClient, 0, sizeof(httpClient));
	memset(buf, 0, toReadLength);

	clientParams->HttpVerb = VerbPost;
	clientParams->pData = buf;
	memcpy(buf, post_buf, strlen(post_buf));
	clientParams->pLength = strlen(post_buf);

	ALINK_DBG("pData = %s\n", (char*)(clientParams->pData));
	ALINK_DBG("pLength = %d\n", (int)(clientParams->pLength));
	ALINK_DBG("uri = %s\n", (char*)(clientParams->Uri));

request:
	if ((ret = HTTPC_open(clientParams)) != 0) {
		ALINK_DBG("http open err..\n");
		goto relese;
	}

	if ((ret = HTTPC_request(clientParams, alink_get_heads)) != 0) {
		ALINK_DBG("http request err..\n");
		goto relese;
	}
	if ((ret = HTTPC_get_request_info(clientParams, &httpClient)) != 0) {
		ALINK_DBG("http get request info err..\n");
		goto relese;
	}
	ALINK_DBG("HTTPStatusCode = %d\n",(int)(httpClient.HTTPStatusCode));
	if (httpClient.HTTPStatusCode != HTTP_STATUS_OK) {
		if((httpClient.HTTPStatusCode == HTTP_STATUS_OBJECT_MOVED) ||
				(httpClient.HTTPStatusCode == HTTP_STATUS_OBJECT_MOVED_PERMANENTLY)) {
			ALINK_DBG("Redirect url..\n");
			HTTPC_close(clientParams);
			memset(clientParams, 0, sizeof(*clientParams));
			clientParams->HttpVerb = VerbGet;
			if (httpClient.RedirectUrl->nLength < sizeof(clientParams->Uri))
				strncpy(clientParams->Uri, httpClient.RedirectUrl->pParam,
						httpClient.RedirectUrl->nLength);
			else
				goto relese;
			ALINK_DBG("go to request.\n");
			goto request;

		} else {
			ret = -1;
			ALINK_DBG("get result not correct..\n");
			goto relese;
		}
	}
	if (httpClient.TotalResponseBodyLength != 0 || (httpClient.HttpFlags & HTTP_CLIENT_FLAG_CHUNKED )) {

		memset(buf, 0, max_resp_length);
		if ((ret = HTTPC_read(clientParams, buf, toReadLength, (void *)&Received)) != 0) {
			ALINK_DBG("get data,Received:%d\n",Received);
			if (ret == 1000) {
				ret = 0;
				sprintf(https_resp_data, "%s", buf);
				ALINK_DBG("buf = %s\n",buf);
				ALINK_DBG("The end..\n");
			} else
				ALINK_DBG("Transfer err...ret:%d\n",ret);
		}
	}
relese:
	free(buf);
	HTTPC_close(clientParams);

	return ret;
}

int alink_get_https_resp(const char *auth_host,
								   const char *product_key,
								   const char *device_name,
								   const char *device_secret,
								   const char *client_id,
								   const char *version,
								   const char *timestamp,
								   const char *resources
								  )
{
#define SIGN_SOURCE_LEN     (256)
#define HTTP_POST_MAX_LEN   (1024)
#define HTTP_RESP_MAX_LEN   (1024)

	int ret = 0,length;
	char sign[33] = {0};
	char md5_out[64] = {0};

	HTTPParameters *clientParams;
	clientParams = (HTTPParameters*)malloc(sizeof(HTTPParameters));
	if(!clientParams) {
		return -1;
	}
	else {
		memset(clientParams, 0, sizeof(HTTPParameters));
	}

	length = strlen(client_id);
    length += strlen(product_key);
    length += strlen(device_name);
    length += strlen(timestamp);
    length += 40; 		//40 chars space for key strings(clientId,deviceName,productKey,timestamp)
    if(length > SIGN_SOURCE_LEN) {
		ALINK_DBG("The total length may be is too long. client_id=%s, product_key=%s, device_name=%s, timestamp= %s\n",
                       client_id, product_key, device_name, timestamp);
	}

	char *buf = NULL, *post_buf = NULL;

	buf = malloc(length);
	if(buf == NULL) {
		ALINK_DBG("buf malloc error! \n");
		goto do_exit;
	}

    ret = snprintf(buf,
                   SIGN_SOURCE_LEN,
                   "clientId%sdeviceName%sproductKey%stimestamp%s",
                   client_id,
                   device_name,
                   product_key,
                   timestamp);
    if ((ret < 0) || (ret > SIGN_SOURCE_LEN)) {
		ALINK_DBG("fail in %d\n",__LINE__);
		ret = -2;
        goto do_exit;
    }
	ALINK_DBG("sign source = %s\n", buf);
	const mbedtls_md_info_t *res_info = mbedtls_md_info_from_type(MBEDTLS_MD_MD5);
	mbedtls_md_hmac(res_info, (const unsigned char*)device_secret, 32, \
					(const unsigned char*)buf, strlen(buf), (unsigned char*)md5_out);
	to_hex_str((unsigned char*)md5_out, (char*)sign, 16);
	ALINK_DBG("sign = %s\n",sign);

	post_buf =  (char*)malloc(HTTP_POST_MAX_LEN);
	if(post_buf == NULL) {
		free(clientParams);
		ALINK_DBG("malloc http post buf failed!\n");
		return -2;
	}
	memset(post_buf, 0, HTTP_POST_MAX_LEN);
    ret = snprintf(post_buf,
                   HTTP_POST_MAX_LEN,
                   "productKey=%s&deviceName=%s&sign=%s&version=%s&clientId=%s&timestamp=%s&resources=%s",
                   product_key,
                   device_name,
                   sign,
                   version,
                   client_id,
                   timestamp,
                   resources);

    if ((ret < 0) || (ret >= HTTP_POST_MAX_LEN)) {
        ALINK_DBG("http message body is too long");
        ret = -3;
        goto do_exit;
    }
    ALINK_DBG("http content:%s\n\r", post_buf);

	HTTPC_Register_user_certs(alink_https_get_certs);
	strcpy(clientParams->Uri, auth_host);
	alink_https_post(clientParams, post_buf, HTTP_POST_MAX_LEN, HTTP_RESP_MAX_LEN);

do_exit:
	if (NULL != buf)
        free(buf);

    if (NULL != post_buf)
        free(post_buf);

	return ret;
}

int https_mqtt_para_get(char *iot_id,
								   char *iot_token,
								   char *host,
								   uint16_t *port,
								   const char *product_key,
								   const char *device_name,
								   const char *device_secret,
								   const char *client_id)
{
	int ret = 0;

	ret = alink_get_https_resp(IOT_AUTH_HOSTNAME,
						 		  product_key,
						 		  device_name,
						 		  device_secret,
						 		  client_id,
						 		  "default",
						 		  "2524608000000",
						 		  "mqtt"
								  );
	if(ret < 0) {
		ALINK_DBG("alink http get response data error!\n");
		return -1;
	}

	char* data_get = NULL;

	data_get = alink_cjson_get_iot_para(https_resp_data, "iotId");
	if(data_get == NULL) {
		ALINK_DBG("data get cjson data error!\n");
		return -1;
	}
	memcpy(iot_id, data_get, strlen(data_get));
	iot_id[strlen(data_get)] = '\0';

	data_get = alink_cjson_get_iot_para(https_resp_data, "iotToken");
	if(data_get == NULL) {
		ALINK_DBG("data get cjson data error!\n");
		return -1;
	}
	memcpy(iot_token, data_get, strlen(data_get));
	iot_token[strlen(data_get)] = '\0';

	data_get = alink_cjson_get_mqtt_addr(https_resp_data, "host");
	if(data_get == NULL) {
		ALINK_DBG("data get cjson data error!\n");
		return -1;
	}
	memcpy(host, data_get, strlen(data_get));
	host[strlen(data_get)] = '\0';

	data_get = alink_cjson_get_mqtt_addr(https_resp_data, "port");
	if(data_get == NULL) {
		ALINK_DBG("data get cjson data error!\n");
		return -1;
	}
	*port = atoi(data_get);

	return ret;
}

