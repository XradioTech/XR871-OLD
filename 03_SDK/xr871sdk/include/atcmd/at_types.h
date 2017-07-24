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

#ifndef _AT_TYPES_H_
#define _AT_TYPES_H_

#ifdef __arm__
#define DEBUG(fmt...) {\
	printf("file:%s line:%d ", __FILE__, __LINE__);\
	printf(fmt);\
	}

#else
#define DEBUG(fmt,...) {\
	printf("file:%s line:%d ", __FILE__, __LINE__);\
	printf(fmt);\
	}
#endif

#ifdef __arm__
#define AT_DUMP	printf
#else
#define AT_DUMP printf
#endif

#define AT_MAX_PEER_NUM	5
#define AT_CMD_MAX_SIZE	32
#define AT_PARA_MAX_SIZE 64UL /* should be pow2(n), n is a integer */
#define AT_SOCKET_BUFFER_SIZE	1024L

#define AT_SEPARATOR	','
#define AT_CR			0x0d
#define AT_LF			0x0a
#define AT_COLON		':'
#define AT_DOT			'.'
#define AT_EQU			'='

#define ANL_WINDOWS	0
#define ANL_UNIX	1
#define ANL_MAC		2

#ifndef BIT
#define BIT(n) (1UL<<(n))
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

#define SIZE_LIMIT_MASK	((AT_PARA_MAX_SIZE<<1)-1)
#define SIZE_LIMIT(opt)	((opt) & SIZE_LIMIT_MASK)
/* #define END_PARA(opt)	((((opt) & AET_PARA)!=0)?1:0) */
/* #define END_LINE(opt)	((((opt) & AET_LINE)!=0)?1:0) */

#define TABLE_SIZE(tbl)	(sizeof(tbl)/sizeof((tbl)[0]))

typedef enum
{
	AEC_OK=0,
	AEC_BLANK_LINE,
	AEC_CMD_ERROR,
	AEC_PARA_ERROR,
	AEC_NO_PARA,
	AEC_UNSUPPORTED,
	AEC_NOT_FOUND,
	AEC_NULL_POINTER,
	AEC_OUT_OF_RANGE,
	AEC_SCAN_FAIL,
	AEC_READ_ONLY,
	AEC_SEND_FAIL,
	AEC_SWITCH_MODE,
	AEC_CONNECT_FAIL,
	AEC_SOCKET_FAIL,
	AEC_LIMITED,
	AEC_DISCONNECT,
	AEC_NETWORK_ERROR,
	AEC_UNDEFINED,
}AT_ERROR_CODE;

typedef enum
{
	APT_TEXT, /* text */
	APT_HEX, /* hex format data */
	APT_DI, /* decimal integer */
	APT_HI, /* hexadecimal integer */
	APT_IP,	/* ip format data */
}AT_PARA_TYPE;

typedef enum
{
	APO_RO, /* read only */
	APO_RW, /* read and write */
}AT_PARA_OPTION;

typedef enum
{
	AET_PARA=AT_PARA_MAX_SIZE<<2,
	AET_LINE=AT_PARA_MAX_SIZE<<3,
}AT_END_TYPE;

typedef enum
{
	AM_CMD=0,
	AM_DATA,
}AT_MODE;

typedef signed char s8,S8;
typedef signed short s16,S16;
typedef signed int s32,S32;
typedef unsigned char u8,U8;
typedef unsigned short u16,U16;
typedef unsigned int u32,U32;

typedef char at_text_t; /* text */
typedef unsigned char at_hex_t; /* hex format data */
typedef int at_di_t; /* decimal integer */
typedef unsigned int at_hi_t; /* hexadecimal integer */
typedef unsigned char at_ip_t[4]; /* ip format data */

typedef struct
{
	AT_PARA_TYPE pt; /* parameter type */
	void *pvar; /* the pointer to the variable */
	u32 option; /* <end type> | <size limit> */
}at_para_descriptor_t;

typedef union
{
	at_text_t text[AT_PARA_MAX_SIZE];
	at_hex_t hex[AT_PARA_MAX_SIZE];		
	at_di_t di;
	at_hi_t hi;
	at_ip_t ip;
}at_value_t;

typedef struct
{
	char *key;	/* variable keyword */
	AT_PARA_TYPE pt; /* parameter type */
	AT_PARA_OPTION po; /* parameter option */
	void *pvar; /* pointer to variable */
	s32 vsize; /* data size limit */
	s32 (*verify)(at_value_t *value); /* check data range */
}at_var_descriptor_t; /* variable descriptor */

#endif
