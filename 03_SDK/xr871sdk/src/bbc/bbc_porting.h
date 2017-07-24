/*
 * bbc_porting.h
 *
 *  Created on: 2016年1月13日
 *      Author: yangfuyang
 */

#ifndef BBC_BBC_PORTING_H_
#define BBC_BBC_PORTING_H_


#include "bbc_sdk.h"

/*
 * host：主机地址或域名
 * port：端口
 * return: 成功返回socket_fd，失败返回-1
 * 打开socket,连接到对应的host与port的服务器，返回socket_fd
 */
int connect_to(const char *host, const char *port);

/*
 * fd : socket套接字句柄
 * return： 成功：0 ， 失败：-1
 * 设置socket为非阻塞模式
 */
int make_non_block(int fd);

/*
 *fd：socket套接字句柄
 *return： 成功：0 ， 失败：-1
 *设置socket为no_delay模式
 */
int make_no_delay(int fd);

/*
 * random: 存放产生的随机字节序列
 * len：需要产生的随机字节序列长度
 */
void get_random(char* random, int len);



#endif /* BBC_SDK_INCLUDES_BBC_PORTING_H_ */
