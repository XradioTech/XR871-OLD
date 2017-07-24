/*
 * utils.c
 *
 *  Created on: 2016年1月11日
 *      Author: Administrator
 */
#include <unistd.h>
#include "utils.h"

#include "buffer.h"
#include "dbg.h"

char* get_name_from_url(char* url){
	Buffer* buffer = buffer_alloc(1024);
	strtok(url, "/");
	buffer_appendf(buffer, "%s", url);
	char *p = NULL;
	while((p = strtok(NULL, "/"))){
		buffer_appendf(buffer, "%s", p);
	}
	p = buffer_to_s(buffer);
	buffer_free(buffer);
	return p;
}

char*  copy_str(char* src){
	int len = strlen(src);
	char* content = (char*)calloc(1, sizeof(char)*(len + 1));
    strncpy(content, src, len);
	return content;
}

void to_hex_str(unsigned char* src, char* dest, int len){
	int i;
	for(i = 0;i < len; i++){
		sprintf(dest + i*2, "%02x", src[i]);
	}
}


