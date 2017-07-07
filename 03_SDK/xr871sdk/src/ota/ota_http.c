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

#ifdef __CONFIG_ARCH_DUAL_CORE
#include <string.h>

#include "ota_i.h"
#include "ota_http.h"
#include "sys/ota.h"

#include "fs/fatfs/ff.h"
typedef struct {
	char *url;
	FATFS fs;
	FRESULT res;
	FIL file;
} OtaFsParam;

static HTTPParameters	http_param;
static OtaFsParam fs_param;
static uint32_t ota_type = 0;

ota_status ota_update_http_init(void *arg)
{
	if (!memcmp(arg, "file://", 7)) {
#ifdef OTA_FS
		fs_param.url = strdup(arg + 7);

		if ((fs_param.res = f_mount(&fs_param.fs, "0:/", 1)) != FR_OK) {
	    	printf("can not mount\n");
	    } else {
	    	printf("mount success\n");
	    }

		if ((fs_param.res = f_open(&fs_param.file, fs_param.url, FA_READ | FA_OPEN_EXISTING)) != FR_OK) {
	    	printf("can not OPEN\n");
	    } else {
	    	printf("OPEN success\n");
	    }

		ota_type = 1;
#else
		(void)fs_param;
		return OTA_STATUS_ERROR;
#endif
	} else {
		ota_memset(&http_param, 0, sizeof(HTTPParameters));
		ota_memcpy(http_param.Uri, arg, strlen(arg));

		ota_type = 0;
	}

	return OTA_STATUS_OK;
}

ota_status ota_update_http_get(uint8_t *buf, uint32_t buf_size,
							   uint32_t *recv_size, uint8_t *eof_flag)
{
	int	ret;

	if (ota_type == 0) {
		ret = HTTPC_get(&http_param, (CHAR *)buf, (INT32)buf_size, (INT32 *)recv_size);
		if (ret == HTTP_CLIENT_SUCCESS) {
			*eof_flag = 0;
			return OTA_STATUS_OK;
		} else if (ret == HTTP_CLIENT_EOS) {
			*eof_flag = 1;
			return OTA_STATUS_OK;
		} else {
			OTA_ERR("ota update http get failed: ret %d\n", ret);
			return OTA_STATUS_ERROR;
		}
	} else {
#ifdef OTA_FS
		if ((fs_param.res = f_read(&fs_param.file, buf, buf_size, recv_size)) != FR_OK) {
	    	printf("can not READ\n");
			return OTA_STATUS_ERROR;
	    }

		if (*recv_size < buf_size)
			*eof_flag = 1;
		else
			*eof_flag = 0;

		return OTA_STATUS_OK;
#else
		return OTA_STATUS_ERROR;
#endif

	}
}
#endif /* __CONFIG_ARCH_DUAL_CORE */
