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

#include<stdlib.h>
#include<string.h>
#include "url.h"

int url_check(Url *url);

Url *url_parse(char *url_to_parse)
{
	Url *url = (Url*)malloc(sizeof(Url));
	memset(url,0, sizeof(Url));
	//scheme
	char* p = strstr(url_to_parse, "://");
	const size_t scheme_len = (p - url_to_parse);
	url->scheme = (char*)malloc(scheme_len*sizeof(char) + 1);
	strncpy(url->scheme, url_to_parse, scheme_len);
	url->scheme[scheme_len] = '\0';
	url_to_parse += scheme_len + 3;

	//host
	const size_t host_len = strcspn(url_to_parse, ":/?#");
	url->hostname = (char*)malloc(host_len*sizeof(char) + 1);
	strncpy(url->hostname, url_to_parse, host_len);
	url->hostname[host_len] = '\0';
	url_to_parse += host_len;
	//port
	if (url_to_parse[0] == ':')
	   {
	      const size_t port_len = strcspn(++url_to_parse, "/?#");
	      if (port_len)
	      {
	    	  url->port = (char*)malloc(port_len*sizeof(char) + 1);
	    	  strncpy(url->port, url_to_parse, port_len);
	          url_to_parse += port_len;
	      }
	   }
	else{
		char* default_port = "80";
		if (!strncmp(url->scheme, "http", strlen("http")))
			default_port = "80";
		      else if (!strncmp(url->scheme, "https", strlen("https")))
		    	  default_port = "443";
		      else if (!strncmp(url->scheme, "ftp", strlen("ftp")))
		    	  default_port = "21";
		      else if (!strncmp(url->scheme, "ws", strlen("ws")))
		    	  default_port = "80";
		      else if (!strncmp(url->scheme, "wss", strlen("wss")))
		    	  default_port = "443";
		url->port = (char*)malloc(strlen(default_port)*sizeof(char) + 1);
		strcpy(url->port, default_port);
	}
	//path
	url->path = (char*)malloc(strlen(url_to_parse)*sizeof(char) + 1);
	strcpy(url->path, url_to_parse);
	return url;
}


void add_path(char* url, const char* path){
	if(path){
		strcat(url, path);
		strcat(url, "/");
	}
}


int url_check(Url *url)
{
    int invalid = 0;

    invalid = invalid || url->scheme == NULL || strcmp(url->scheme, "http");
    invalid = invalid || url->hostname == NULL;

    return invalid;
}

void url_free(Url *url)
{
    url_free_part(url->scheme);
    url_free_part(url->hostname);
    url_free_part(url->port);
    url_free_part(url->path);
    free(url);
}
