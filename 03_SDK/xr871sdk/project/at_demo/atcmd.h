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

#ifndef _ATCMD_H_
#define _ATCMD_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATCMD_DBG_ON		0
#define ATCMD_WARN_ON		1
#define ATCMD_ERR_ON		1

#define ATCMD_CHECK_OVERFLOW		1

#define ATCMD_SYSLOG		printf
#define ATCMD_ABORT()	do { } while (1) //sys_abort()

#define ATCMD_LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			ATCMD_SYSLOG(fmt, ##arg);	\
	} while (0)

#define ATCMD_DBG(fmt, arg...)	ATCMD_LOG(ATCMD_DBG_ON, "[atcmd] "fmt, ##arg)
#define ATCMD_WARN(fmt, arg...)	ATCMD_LOG(ATCMD_WARN_ON, "[atcmd WARN] "fmt, ##arg)
#define ATCMD_ERR(fmt, arg...)								\
	do {													\
		ATCMD_LOG(ATCMD_ERR_ON, "[atcmd ERR] %s():%d, "fmt,	\
	           __func__, __LINE__, ##arg);					\
	    if (ATCMD_ERR_ON)									\
			ATCMD_ABORT();									\
	} while (0)

extern void atcmd_start(void);

#ifdef __cplusplus
}
#endif

#endif /* _SYS_CTRL_H_ */
