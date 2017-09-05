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
#include <malloc.h>
#include <lwip/netdb.h>
#include <string.h>
#include <stdint.h>

#include "bbc_sdk.h"
#include "cjson/cJSON.h"
#include "url.h"

#include "driver/chip/hal_flash.h"
#include "cloud/bbc/devguid_get.h"

#define DEC_GET_DBG_SET 	1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DEC_GET_DBG(fmt, arg...)	\
			LOG(DEC_GET_DBG_SET, "[DEC_GET_DBG] "fmt, ##arg)


//设备信息，根据设备具体信息赋值
Device device = {
		.name = "testliu",
		.mac = "cda03200ab",
		.vendor = "test",
		.firmwareVersion = "0.0.3",
		.romType = "STABLE",
		.sdkVersion = SDK_VERSION,
};

char bbc_lic[128] = {0};
void device_info(const char* devid, const char* dev_name, const char* licence)
{
	sprintf(device.deviceId, "%s", devid);
	sprintf(device.deviceClass.name, "%s", dev_name);
	sprintf(bbc_lic, "%s", licence);
}

#define REGIST_KEY		"luowenqiangsetflag"

int bbc_inital()
{
	unsigned char readbuff[40];
	unsigned char writebuff[40];
	unsigned char read_register_flag[20] = {0};
	unsigned char device_id_comp[32] = {0};
	char* deviceGuid = NULL;

	HAL_Flash_Open(0, 5000);
	//HAL_Flash_Erase(0, FLASH_ERASE_4KB, devguid_in_flash, 1);

	if(HAL_OK == HAL_Flash_Read(0, regist_flag_in_flash, read_register_flag, 20)) {
		if(0 == memcmp(read_register_flag, REGIST_KEY, 18)) {
			if(HAL_OK == HAL_Flash_Read(0, (regist_flag_in_flash + 100), device_id_comp, 32)) {
				if(0 == memcmp(device_id_comp, device.deviceId, strlen(device.deviceId))) {
					HAL_Flash_Read(0, devguid_in_flash, readbuff, 40);
					deviceGuid = (char*)readbuff;
				}
				else {
					memset(deviceGuid,0,40);
				}
			}
		}
	}

	//the device start first need to register
	if (deviceGuid == NULL)
	{
		DEC_GET_DBG("The first Time Register!\n");
		deviceGuid = register_device(&device);
		if(deviceGuid == NULL){
			DEC_GET_DBG("register_device error !!!");
			return -1;
		}
		sprintf((char*)writebuff, "%s", deviceGuid);
		HAL_Flash_Erase(0, FLASH_ERASE_4KB, devguid_in_flash, 1);
		HAL_Flash_Write(0, devguid_in_flash, writebuff, 40);
		//write register flag and device id
		HAL_Flash_Write(0, regist_flag_in_flash, (uint8_t*)REGIST_KEY, 20);
		HAL_Flash_Write(0, (regist_flag_in_flash + 100), (uint8_t*)(device.deviceId), strlen(device.deviceId) + 1);
		//get the deviceguid value
		DEC_GET_DBG("devguid = %s\n",deviceGuid);
		DEC_GET_DBG("deviceGuid OK!!\n");
	}
	else {
		DEC_GET_DBG("deviceGuid = %s\n",deviceGuid);
		DEC_GET_DBG("devguid had registed! plase get it in flash 1980k!\n");
	}
	HAL_Flash_Close(0);

	return 0;
}

