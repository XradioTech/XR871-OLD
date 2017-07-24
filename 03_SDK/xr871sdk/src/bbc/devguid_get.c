#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>
#include <lwip/netdb.h>
#include <string.h>

#include "property.h"
#include "bbc_sdk.h"
#include "cjson.h"
#include "url.h"
#include "http_connect.h"
#include "dbg.h"
#include <stdint.h>
#include "../include/driver/chip/hal_norflash.h"
#include "devguid_get.h"

//设备信息，根据设备具体信息赋值
Device device = {
#ifdef USE_SENOR_DEF
		.deviceId = "201707202027",
#else	
		.deviceId = "201707202029",
#endif
		.name = "testliu",
		.mac = "cda03200ab",
		.vendor = "test",
		.firmwareVersion = "0.0.3",
		.romType = "STABLE",
		.sdkVersion = SDK_VERSION,
		.deviceClass = {
#ifdef USE_SENOR_DEF
			.name = "Tbu1XR871test"
#else	
			.name = "Tbu1XR871test2"
#endif
		}
};

char* deviceGuid = NULL;

int bbc_inital(void)
{
	unsigned char readbuff[40];
	unsigned char writebuff[40];
	
	SF_Config flash_config;
	SF_Handler hdl;
	flash_config.spi = SPI0;
	flash_config.cs = SPI_TCTRL_SS_SEL_SS0;
	flash_config.dma_en = 1;
	flash_config.sclk = 12 * 1000 * 1000;
	HAL_SF_Init(&hdl, &flash_config);

	int i;
	int recod_ff = 0;
	if(HAL_OK == HAL_SF_Read(&hdl, devguid_in_flash, readbuff, 40)) {
		for(i = 0; i < 40; i++) {
			if(readbuff[i] == 255) {
				recod_ff++;
			}
			else {
				break;
			}
		}
		if(recod_ff < 40) {
			deviceGuid = readbuff;
		}
		else {
			memset(deviceGuid,0,40); 
		}
	}
	
	//the device start first need to register
	if (deviceGuid == NULL)	
	{
		printf("The first Time Register!\n");
		deviceGuid = register_device(&device);
		if(deviceGuid == NULL){
			printf("register_device error !!!");
			return -1;
		}
		sprintf(writebuff, "%s", deviceGuid);
		HAL_SF_Write(&hdl, devguid_in_flash, writebuff, 40);
		//get the deviceguid value
		printf("devguid = %s\n",deviceGuid);
		printf("deviceGuid OK!!\n");
	}
	else {
		printf("deviceGuid = %s\n",deviceGuid);
		printf("devguid had registed! plase get it in flash 1980k!\n");
	}

	return 0;
}

