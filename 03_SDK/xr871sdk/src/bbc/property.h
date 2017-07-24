/*
 * property.h
 *
 *  Created on: 2015年12月18日
 *      Author: Administrator
 */

#ifndef BBC_PROPERTY_H_
#define BBC_PROPERTY_H_

#include <stdio.h>
#include <stdlib.h>

#include "cjson.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PROPERTY_FILE "./mydevice.property"

cJSON*  initProperty();
void saveProperty(cJSON* config);
void setLastTimestamp(cJSON* config,char* timestamp);//保存设备上次在线最后接收命令的时间戳
char* getLastTimestamp(cJSON* config);
void setLastVersion(cJSON* config,char* version);//保存固件版本号，用以对比是否需要更新设备信息
char* getLastVersion(cJSON* config);
void setDeviceGuid(cJSON* config, char* deviceGuid);//保存平台分配给device的唯一id
char* getDeviceGuid(cJSON* config);


#ifdef __cplusplus
}
#endif


#endif /* BBC_SDK_INCLUDES_PROPERTY_H_ */
