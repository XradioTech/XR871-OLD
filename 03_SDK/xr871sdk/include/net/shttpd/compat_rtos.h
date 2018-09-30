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

#ifndef __COMPAT_RTOS_H_H
#define __COMPAT_RTOS_H_H

#include "lwip/sockets.h"
#include "kernel/os/os.h"

#define DIRSEP                          '/'
#define IS_DIRSEP_CHAR(c)               ((c) == '/')
#define SHTTPD_SSI
#define SHTTPD_SSI_CALL
#define SHTTPD_SINGLE_CONNECTION
#define SHTTPD_LOG_ALT
#define SHTTPD_MEM_IN_HEAP
//#define SHTTPD_SSL
//#define SHTTPD_CUSTOM_LOG_ON
//#define SHTTPD_DEBUG_ON

#include "kernel/os/os_time.h"
#include <time.h>
#include <errno.h>

struct usr_file {
	char *name;
	char *body;
};

struct f_stat {
	unsigned long st_size;
	unsigned int  st_mode;
};

time_t TIME(time_t *timer);

#define ERRNO                      errno

#define stat                       f_stat
#define _S_IFMT                   (0x0170000)
#define _S_IFDIR                  (0x0040000)
#define _S_IFREG                  (0x0100000)
#define _S_IEXEC                  (0x0000100)
#define _S_IWRITE                 (0x0000200)
#define _S_IREAD                  (0x0000400)
#define S_ISDIR(x)                ((x) & _S_IFDIR)

void _shttpd_free(void *ptr);
void *_shttpd_zalloc(size_t size);
void _shttpd_init_local_file(const struct usr_file *list, int count);

#if defined(SHTTPD_THREADS)
#define HTTP_THREAD_STACK_SIZE	(4 * 1024)
#define _beginthread(a, b, c) do { \
	OS_Thread_t shttpd_thread;\
	OS_ThreadCreate(&shttpd_thread,\
                                "shttpd",\
                                a,\
                                (void *)c,\
                                OS_THREAD_PRIO_APP,\
                                HTTP_THREAD_STACK_SIZE);} while (0)
#endif /* SHTTPD_THREADS */
#endif
