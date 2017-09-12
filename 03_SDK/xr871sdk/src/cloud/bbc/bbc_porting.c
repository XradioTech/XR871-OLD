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

#include <string.h>
#include <net/lwip/posix/sys/socket.h>
//#include <sys/select.h>
#include <sys/time.h>
#include <net/lwip/lwip/netdb.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
//#include <netinet/in.h>
#include <lwip/tcp.h>
#include <errno.h>
#include "bbc_porting.h"

#define BBC_PORT_DBG_SET 	0
#define BBC_PORT_PRINT		0

#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define BBC_PORT_DBG(fmt, arg...)	\
			LOG(BBC_PORT_DBG_SET, "[BBC_PORT_DBG] "fmt, ##arg)
#define PORT_PRINT(fmt, arg...)	\
			LOG(BBC_PORT_PRINT,fmt, ##arg)

int connect_to(const char *host, const char *port);

char* execute_request(char* host_name, char* port, char* request)
{
	int socketfd = 0;
	int pcontent_size = 0;
	int send_byte, totalsend, nbytes;
	char* content = NULL;
	int i = 0, b=0;
	int content_size, index = 0;
	char buffer[1024];

	socketfd = connect_to(host_name, port);
	BBC_PORT_DBG("socketfd = %d\n",socketfd);
	if(socketfd <0){
		BBC_PORT_DBG(" execute request open socket error \n");
		return NULL;
	}
	/*发送http请求request*/
	send_byte = 0;
	totalsend = 0;
	nbytes=strlen(request);
	while(totalsend < nbytes)
	{
		send_byte = send(socketfd, request + totalsend, nbytes - totalsend, 0);
		if(send_byte==-1)
		{
			BBC_PORT_DBG("send error!%s\n", strerror(errno));
			return NULL;
		}
		totalsend+=send_byte;
		BBC_PORT_DBG("%d bytes send OK!\n\n", totalsend);
	}
	BBC_PORT_DBG("Flag 1+\n");
	/* 连接成功了，接收http响应，response */

	i = 0;
	b = 0;
	while(i < 4 && recv(socketfd,&buffer[b],1, 0) == 1)
	{
		if(buffer[b] == '\r' || buffer[b] == '\n')
			i++;
		else
			i = 0;
		PORT_PRINT("%c", buffer[b]);
		b++;
	}
	buffer[b] = '\0';

	int responseStatus = 0;
	sscanf (strstr (buffer, "HTTP/"), "HTTP/%*f %d", &responseStatus);
	BBC_PORT_DBG("responsstatus code %d\n ", responseStatus);
	char* temp_s = strstr(buffer, "Content-Length: ");

	if( responseStatus == 200 && temp_s != NULL)
	{
		sscanf(temp_s, "Content-Length: %d", &pcontent_size);
		content = malloc(pcontent_size + 1);

		content_size = pcontent_size;
		BBC_PORT_DBG("content_size = %d\n",content_size);
		while(content_size > 0)
		{
			nbytes = recv(socketfd, &content[index], pcontent_size-index, 0);
			if( nbytes == 0 || nbytes == -1 )
			{
				PORT_PRINT("recv end!");
				break;
			}
			content_size -= nbytes;
			index += nbytes;
		}
		if( content_size != 0 )
			PORT_PRINT("recv content size (%d != %d)\n", index ,pcontent_size);
		content[pcontent_size] = '\0';
	}
	else
	{
		BBC_PORT_DBG("%s\n", buffer);
	}
	BBC_PORT_DBG("socketfd = %d\n",socketfd);
	if(socketfd) close(socketfd);

	return content;
}


int connect_to(const char *host, const char *port)
{
	int sockfd, status;
	struct addrinfo *res = NULL;
    struct addrinfo hints;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

	BBC_PORT_DBG("host = %s\n",host);
	BBC_PORT_DBG("port = %s\n",port);
    status = getaddrinfo(host, port, &hints, &res);
	BBC_PORT_DBG("status = %d\n",status);
    if(status != 0){
    	//BBC_PORT_DBG("Could not resolve host: %s\n", gai_strerror(status));
    	BBC_PORT_DBG("Could not resolve host\n");
    	return -1;
    }

    sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if(!(sockfd >= 0)) {
		goto error;
	}

    status = connect(sockfd, res->ai_addr, res->ai_addrlen);
    if(!(status == 0)) {
		goto error;
	}

	freeaddrinfo(res);
	return sockfd;

    error:
		freeaddrinfo(res);
        return -1;
}

int make_non_block(int fd)
{
  int flags, r;
  while((flags = fcntl(fd, F_GETFL, 0)) == -1 && errno == EINTR);
  if(flags == -1) {
    return -1;
  }
  while((r = fcntl(fd, F_SETFL, flags | O_NONBLOCK)) == -1 && errno == EINTR);
  if(r == -1) {
    return -1;
  }
  return 0;
}

int make_no_delay(int fd)
{
	int val = 1;
	return setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, (socklen_t)sizeof(val));
}

void get_random(char* random, int len)
{
	  int fd = open("/dev/urandom", O_RDONLY);
	  int bytes = read(fd, random, len);
	  if(bytes == -1) BBC_PORT_DBG(" read urandom erroe ");
	  close(fd);
}


