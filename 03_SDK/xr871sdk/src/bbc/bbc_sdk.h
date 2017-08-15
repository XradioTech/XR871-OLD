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

#ifndef BBC_BBC_SDK_H_
#define BBC_BBC_SDK_H_
#include "bbc/cjson.h"
#include "bbc/devguid_get.h"
#include "url.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USE_SENOR_DEF
//senor licence
#define LICENCE "c1c1f7408965eb52f21219e8b6b56e16093ebf9d3f6d97e8b05dccdaedb91ef0cc27b1e28b5d8847" //lwq
#else
//audio licence
#define LICENCE "195e93278e40c16b0a3fb48cd60a5dd842f48f659989e01beafa48d04259652b63d78a7bbde91219"
#endif

#define PRODUCT_KEY "89"		//add bu luo,constant value
#define SDK_VERSION "0.8.3"
#define RESTFUL_URI "http://api.bigbigcloud.cn/dh/v2/rest"
#define WEBSOCKET_URI "ws://ws.bigbigcloud.cn/dh/v2/websocket"
#define SERVER_URI "http://api.bigbigcloud.cn/dh/v2/rest"
//#define RESTFUL_URI "http://115.28.45.40:8080/dh/v2/rest"
//#define WEBSOCKET_URI "ws://115.28.45.40:8080/dh/v2/websocket"
//#define SERVER_URI "http://115.28.45.40:8080/dh/v2/rest"

typedef struct DeviceClass{
	char name[32];  //设备类型名
}DeviceClass;

typedef struct Device{
	char deviceId[32];  //厂商分配给每个设备的唯一ID
	char name[32];
	char mac[32];      //设备mac地址
	char vendor[32];    //设备的厂商标识
	char firmwareVersion[32]; //固件版本号，以“xxx.xxx.xxx"形式表示
	char sdkVersion[32];  //sdk 版本
	char romType[32];   //固件版本类型，分 STABLE、DEV 两种类型
	DeviceClass deviceClass;
}Device;

//ota升级失败信息
typedef struct OtaFailedInfo{
	char curVersion[16];//当前版本号
	int errorCode;//错误码
}OtaFailedInfo;

typedef struct PackInfo{
	char packUrl[128];
	char packMD5[64];
	int  packSize;
	int  orderNo;
}PackInfo;


//设备注册
/*
 * @param device  设备的信息结构体指针
 * @return 成功：返回设备deviceGuid，失败：null
 */
char* register_device(Device *device);

//设备信息同步
/*
 * @param deviceGuid 云平台分配给设备的唯一Guid
 * @param device  设备的信息结构体指针
 * @parma failedInfo  设备升级失败信息，没有则可为NULL
 * @return 成功 ：返回 1， 失败：返回0
 */
int sync_device(char* deviceGuid, Device *device, OtaFailedInfo *failedInfo);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDES_BBC_SDK_H_ */
