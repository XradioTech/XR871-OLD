/*
 * bbc_linux_porting.c
 *
 *  Created on: 2016年1月13日
 *      Author: yangfuyang
 */

#include <../../include/net/lwip/posix/sys/socket.h>
//#include <sys/select.h>
#include <sys/time.h>
#include <../include/net/lwip/lwip/netdb.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
//#include <netinet/in.h>
#include <lwip/tcp.h>
#include <errno.h>
#include "bbc_porting.h"
#include "dbg.h"

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
	printf("socketfd = %d\n",socketfd);
	if(socketfd <0){
		printf(" execute request open socket error \n");
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
			printf("send error!%s\n", strerror(errno));
			return NULL;
		}
		totalsend+=send_byte;
		printf("%d bytes send OK!\n\n", totalsend);
	}
	printf("Flag 1+\n");
	/* 连接成功了，接收http响应，response */

	i = 0;
	b = 0;
	while(i < 4 && recv(socketfd,&buffer[b],1, 0) == 1)
	{
		if(buffer[b] == '\r' || buffer[b] == '\n')
			i++;
		else
			i = 0;
		printf("%c", buffer[b]);
		b++;
	}
	buffer[b] = '\0';
	printf("Flag 2+\n");

	int responseStatus = 0;
	sscanf (strstr (buffer, "HTTP/"), "HTTP/%*f %d", &responseStatus);
	printf("responsstatus code %d\n ", responseStatus);
	char* temp_s = strstr(buffer, "Content-Length: ");

	if( responseStatus == 200 && temp_s != NULL)
	{
		sscanf(temp_s, "Content-Length: %d", &pcontent_size);
		content = malloc(pcontent_size + 1);

		content_size = pcontent_size;
		printf("content_size = %d\n",content_size);
		while(content_size > 0)
		{
			nbytes = recv(socketfd, &content[index], pcontent_size-index, 0);
			if( nbytes == 0 || nbytes == -1 )
			{
				printf("recv end!");
				break;
			}
			content_size -= nbytes;
			index += nbytes;
		}
		if( content_size != 0 )
			printf("recv content size (%d != %d)\n", index ,pcontent_size);
		content[pcontent_size] = '\0';
	}
	else
	{
		printf("%s\n", buffer);
	}
	printf("socketfd = %d\n",socketfd);
	if(socketfd) close(socketfd);
	printf("Flag 3+\n");
	
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

	printf("host = %s\n",host);
	printf("port = %s\n",port);
    status = getaddrinfo(host, port, &hints, &res);
	printf("status = %d\n",status);
    if(status != 0){
    	//debug("Could not resolve host: %s\n", gai_strerror(status));
    	debug("Could not resolve host\n");
    	return -1;
    }
	
    sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    jump_unless(sockfd >= 0);

    status = connect(sockfd, res->ai_addr, res->ai_addrlen);
    jump_unless(status == 0);

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
	  if(bytes == -1) debug(" read urandom erroe ");
	  close(fd);
}


