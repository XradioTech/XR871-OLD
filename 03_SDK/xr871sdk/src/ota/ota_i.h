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

#include <stdio.h>
#include <string.h>

#include "types.h"
#include "kernel/os/os.h"
#include "sys/ota.h"

#ifndef _OTA_I_H_
#define _OTA_I_H_

#define DBG_OTA		0

#if DBG_OTA
#define OTA_DBG(fmt, arg...)		printf("[OTA] "fmt, ##arg)
#define OTA_WRN						OTA_DBG
#define OTA_ERR						OTA_DBG
#else /* DBG_OTA */
#define OTA_DBG(...)
#define OTA_WRN(...)
#define OTA_ERR(...)
#endif /* DBG_OTA */

#define ota_malloc(l)				malloc(l)
#define ota_free(p)					free(p)
#define ota_memcpy(d, s, n)			memcpy(d, s, n)
#define ota_memset(s, c, n) 		memset(s, c, n)

#define OTA_BUF_SIZE				(2 << 10)

#define OTA_BOOT_IMAGE_1ST			(0x5555)
#define OTA_BOOT_IMAGE_2ND			(0xAAAA)
#define OTA_BOOT_STATE_UNVERIFIED	(0x6996)
#define OTA_BOOT_STATE_VERIFIED		(0x9669)

typedef struct {
	uint16_t	boot_image;
	uint16_t	boot_state;
} ota_boot_cfg;

#define OTA_BOOT_CFG_SIZE	sizeof(ota_boot_cfg)

typedef struct {
	uint32_t			boot_offset;
	uint32_t			boot_cfg_offset;
	uint32_t			image_offset_1st;
	uint32_t			image_offset_2nd;
	ota_flash_init_cb	init_cb;
	ota_flash_deinit_cb	deinit_cb;
} ota_priv_t;

#endif /* _OTA_I_H_ */
