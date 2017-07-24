/*
 * http_connect.h
 *
 *  Created on: 2016年1月13日
 *      Author: yangfuyang
 */

#ifndef BBC_HTTP_CONNECT_H_
#define BBC_HTTP_CONNECT_H_
#include <lwip/netdb.h>
#include <unistd.h>

#include "buffer.h"
#include "dbg.h"

#define RECV_SIZE 1024
#define BUF_SIZE  RECV_SIZE + 1

typedef struct download_info{
	char* url;
	char* dir;
}download_info;

int open_connection(char *hostname, char *port);
void close_connection(int sockfd);
int send_request(int sockfd, char* request);
int make_request(int sockfd, char *hostname, char *request_path);
int fetch_response(int sockfd, Buffer **response, int recv_size);
void asyn_download(char* string_url, char* dir);
void free_download_info(download_info* info);


#endif /* LINUX_IMPL_INCLUDES_HTTP_CONNECT_H_ */
