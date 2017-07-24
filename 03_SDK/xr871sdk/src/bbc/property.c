/*
 * property.c
 *
 *  Created on: 2015年12月18日
 *      Author: Administrator
 */

#include "property.h"
#include "dbg.h"

cJSON*  initProperty(){
	char* str;
		int len;
		cJSON* config = NULL;
		FILE* fp = fopen(PROPERTY_FILE, "r");
		if(fp == NULL) {
			config = cJSON_CreateObject();
		}else{
			fseek(fp,0L,SEEK_END);
			len = ftell(fp);
			debug(" len %i ", len);
			str = (char*)calloc(len + 1, 1);
			if(str == NULL) {
				fclose(fp);
				return NULL;
			}
			fseek(fp,0L,SEEK_SET);
			fread(str, len, 1, fp);
			str[len] = 0;
			debug(" init file string %s ", str);
			config = cJSON_Parse(str);
			fclose(fp);
		}
		return config;
}

void saveProperty(cJSON* config){
	FILE* fp;
	char* str = cJSON_PrintUnformatted(config);
	debug(" save %s ", str);
	fp = fopen(PROPERTY_FILE, "w+");
	if(fp == NULL) {
		debug("No this file\n");
		return;
	}
	fprintf(fp, "%s", str);
	fwrite(str, 1, strlen(str), fp);
	fclose(fp);
	fp = fopen(PROPERTY_FILE, "w+");
}

void setLastTimestamp(cJSON* config,char* timestamp){
	cJSON* item = cJSON_GetObjectItem(config, "timestamp");
	if(item != NULL){
		cJSON_DeleteItemFromObject(config,"timestamp");
	}
	cJSON_AddItemToObject(config, "timestamp", cJSON_CreateString(timestamp));
	saveProperty(config);
}
char* getLastTimestamp(cJSON* config){
	cJSON* item = cJSON_GetObjectItem(config, "timestamp");
	char* timestamp = NULL;
	if(item != NULL && item->valuestring != NULL){
		timestamp = calloc(strlen(item->valuestring) + 1, 1);
		strcpy(timestamp, item->valuestring);
	}else{
		timestamp = calloc(1, 64*sizeof(char));
	}
	return timestamp;
}
void setLastVersion(cJSON* config,char* version){
	cJSON* item = cJSON_GetObjectItem(config, "version");
	if(item != NULL){
		cJSON_DeleteItemFromObject(config,"version");
	}
	cJSON_AddItemToObject(config, "version", cJSON_CreateString(version));
	saveProperty(config);
}
char* getLastVersion(cJSON* config){
	cJSON* item = cJSON_GetObjectItem(config, "version");
	char* version = NULL;
	if(item != NULL && item->valuestring != NULL){
		version = calloc(strlen(item->valuestring) + 1, 1);
		strcpy(version, item->valuestring);
	}
	return version;
}

void setDeviceGuid(cJSON* config, char* deviceGuid){
	cJSON* item = cJSON_GetObjectItem(config, "deviceGuid");
	if(item != NULL){
		cJSON_DeleteItemFromObject(config,"deviceGuid");
	}
	cJSON_AddItemToObject(config, "deviceGuid", cJSON_CreateString(deviceGuid));
	saveProperty(config);
}

char* getDeviceGuid(cJSON* config){
	cJSON* item = cJSON_GetObjectItem(config, "deviceGuid");
	char* deviceGuid = NULL;
	if(item != NULL && item->valuestring != NULL){
		deviceGuid = calloc(strlen(item->valuestring) + 1, 1);
		strcpy(deviceGuid, item->valuestring);
	}
	return deviceGuid;
}
