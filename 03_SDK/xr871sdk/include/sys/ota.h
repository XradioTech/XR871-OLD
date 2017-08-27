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

#include "driver/chip/hal_flash.h"

#ifndef _SYS_OTA_H_
#define _SYS_OTA_H_

typedef enum {
	OTA_IMAGE_1ST = 1,
	OTA_IMAGE_2ND = 2,
} ota_image;

typedef enum {
	OTA_STATE_UNVERIFIED	= 0,
	OTA_STATE_VERIFIED		= 1,
} ota_state;

typedef struct {
	ota_image	image;
	ota_state	state;
} ota_cfg;

typedef enum {
	OTA_STATUS_OK		= 0,
	OTA_STATUS_ERROR	= -1,
} ota_status;

typedef int (*ota_flash_init_cb)(uint32_t arg);
typedef void (*ota_flash_deinit_cb)(uint32_t arg);

ota_status ota_init(uint32_t boot_offset, uint32_t boot_cfg_offset,
					uint32_t image_offset_1st, uint32_t image_offset_2nd,
					ota_flash_init_cb init_cb, ota_flash_deinit_cb deinit_cb);
ota_status ota_read_cfg(ota_cfg *cfg);
ota_status ota_write_cfg(ota_cfg *cfg);
#ifdef __CONFIG_ARCH_DUAL_CORE
ota_status ota_update(void *cmd);
#endif /* __CONFIG_ARCH_DUAL_CORE */
void ota_deinit(void);

#endif /* _SYS_OTA_H_ */
