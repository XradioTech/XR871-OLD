/*
 * http_connect.c
 *
 *  Created on: 2016年1月13日
 *      Author: yangfuyang
 */
#include <stdio.h>
#include "../cedarx/os_glue/include/pthread.h"
#include <sys/stat.h>
#include "http_connect.h"

#include "url.h"
#include "utils.h"
#include "bbc_porting.h"

void download(char* string_url, char* dir){
	Url* url = url_parse(string_url);
	printf(" host %s path %s ", url->hostname, url->path);
	int socketfd;
	char http_protol[RECV_SIZE];
	int ret;
	float http_ver = 0.0;
	int status;
	int write_length;
	int file_len;
	char recvbuf[RECV_SIZE]; /* recieves http server http protol HEAD */
	char buffer[RECV_SIZE];
	FILE *fp = NULL;
	void *start = NULL;
	bzero (http_protol, sizeof (http_protol));
	bzero (recvbuf, sizeof (recvbuf));
	socketfd = connect_to(url->hostname, url->port);
	if(socketfd < 0){
		debug("open socket failed ");
		return;
	}
	sprintf (http_protol, "GET %s HTTP/1.1\r\n" \
			"Host: %s\r\n" \
	        "Conection: Keep-Alive\r\n\r\n",
	        url->path, url->hostname);
    ret = write (socketfd, http_protol, strlen (http_protol));
    if (ret == -1)
    {
    	debug ("write failed:%d\n", errno);
        exit (1);
    }

    ret = read (socketfd, recvbuf, sizeof (recvbuf));
    if (ret == 0)
    {
    	debug ("server closed:%d\n", errno);
        exit (1);
    }
    else if (ret == -1)
    {
    	debug ("read failed:%d\n", errno);
        exit (1);
    }

    debug ("%s", recvbuf);
    sscanf (strstr (recvbuf, "HTTP/"), "HTTP/%f %d", &http_ver, &status);
    sscanf (strstr (recvbuf, "Content-Length"), "Content-Length: %d", &file_len);

    if (status != 200 || file_len == 0)
    {
    	debug ("http connect failed!\n");
        exit (1);
    }
   if(mkdir(dir, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH)){
	   debug(" make dir %s  failed", dir);
   }
   	char* name_from_path = get_name_from_url(url->path);
   	debug("name from path string : %s ", name_from_path);
    Buffer* filename = buffer_alloc(BUF_SIZE);
    buffer_appendf(filename, dir);
    buffer_appendf(filename, name_from_path);
    char* file_name = buffer_to_s(filename);
    fp = fopen(file_name, "wb");
    buffer_free(filename);
    free(name_from_path);
    if (fp == NULL)
    {
    	debug ("File:%s Can not open:%d\n", file_name, errno);
        exit (1);
    }

    bzero (buffer, sizeof (buffer));

    /* download file's address start here whithout http protol HEAD */
    start = (void *) strstr(recvbuf, "\r\n\r\n") + sizeof ("\r\n\r\n")-1;
    //fwrite (start, sizeof (char), ret - ((void *)start - (void *)&recvbuf), fp);


    /* 加入文件大小判断 */
    int filesize = 0;
    filesize = ret - ( ( void * )start - ( void * )&recvbuf );
    fwrite( start, sizeof( char ), filesize, fp );

    while (1)
    {
        ret = read (socketfd, buffer, sizeof (buffer));

        if (ret == 0) break; /* download finish */

        if (ret < 0)
        {
        	debug ("Recieve data from server [%s] failed!\n", url->hostname);
            break;
        }

        write_length = fwrite (buffer, sizeof (char), ret, fp);
        if (write_length < ret)
        {
        	debug ("File: %s write failed.\n", file_name);
            break;
        }

        /* 下载完就退出 */
        filesize += ret;
        if(filesize >= file_len)
        	break;
        bzero (buffer, sizeof (buffer));
    }

    debug ("\ndownload %s file finish.\n", file_name);

    close (socketfd);
    fclose (fp);
    free(file_name);
    url_free(url);
}

void free_download_info(download_info* info){
	if(NULL != info){
		free(info->url);
		free(info->dir);
		free(info);
	}
}

void* download_runner(void* args){
	download_info* info = (download_info*)args;
	debug(" url string : %s dir %s ", info->url, info->dir);
	download(info->url, info->dir);
	free_download_info(info);
	pthread_exit(0);
}

void asyn_download(char* string_url, char* dir){
	debug(" asyn download url : %s   dir %s ", string_url, dir);
	download_info* info = (download_info*)malloc(sizeof(download_info));
	info->url = copy_str(string_url);
	info->dir = copy_str(dir);
	pthread_t tid;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
	pthread_create(&tid,&attr,download_runner, info);
	pthread_attr_destroy(&attr);
}

char * build_request(char *hostname, char *request_path)
{
    char *request = NULL;
    Buffer *request_buffer = buffer_alloc(BUF_SIZE);

    buffer_appendf(request_buffer, "GET %s HTTP/1.0\r\n", request_path);
    buffer_appendf(request_buffer, "Host: %s\r\n", hostname);
    buffer_appendf(request_buffer, "Connection: close\r\n\r\n");

    request = buffer_to_s(request_buffer);
    buffer_free(request_buffer);

    return request;
}

int make_request(int sockfd, char *hostname, char *request_path)
{
    char *request           = build_request(hostname, request_path);
    size_t bytes_sent       = 0;
    size_t total_bytes_sent = 0;
    size_t bytes_to_send    = strlen(request);

    debug("Bytes to send: %d", bytes_to_send);

    while (1) {
        bytes_sent = send(sockfd, request, strlen(request), 0);
        total_bytes_sent += bytes_sent;

        debug("Bytes sent: %d", bytes_sent);

        if (total_bytes_sent >= bytes_to_send) {
            break;
        }
    }

    free(request);

    return total_bytes_sent;
}

int send_request(int sockfd, char* request){
    size_t bytes_sent       = 0;
    size_t total_bytes_sent = 0;
    size_t bytes_to_send    = strlen(request);

    debug("Bytes to send: %d", bytes_to_send);
    //debug("Bytes to send: %d", bytes_to_send);
    while (1) {
        bytes_sent = send(sockfd, request, strlen(request), 0);
        total_bytes_sent += bytes_sent;

        debug("Bytes sent: %d", bytes_sent);
        //debug("Bytes send: %d", bytes_sent);

        if (total_bytes_sent >= bytes_to_send) {
            break;
        }
    }

    free(request);

    return total_bytes_sent;
}

int fetch_response(int sockfd, Buffer **response, int recv_size)
{
    size_t bytes_received;
    int status = 0;
    char data[recv_size];

    debug("Receiving data ...");
    while (1) {
        bytes_received = recv(sockfd, data, RECV_SIZE, 0);
        debug(" bytes received %i ",  bytes_received);
        if (bytes_received == -1) {
            return -1;
        } else if (bytes_received == 0) {
            return 0;
        }

        if (bytes_received > 0) {
            status = buffer_append(*response, data, bytes_received);
            debug(" buffer append status %i ", status);
            if (status != 0) {
                fprintf(stderr, "Failed to append to buffer.\n");
                return -1;
            }
        }
    }

    debug("Finished receiving data.");
    return status;
}



