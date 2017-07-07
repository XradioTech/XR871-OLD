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

#ifndef _IMAGE_FLASH_H_
#define _IMAGE_FLASH_H_

#include "driver/chip/hal_spi.h"
#include "driver/chip/hal_norflash.h"

#define DBG_FLASH	0

#if DBG_FLASH
#define FLASH_DBG(fmt, arg...)	printf("[FLASH] "fmt, ##arg)
#define FLASH_WRN				FLASH_DBG
#define FLASH_ERR				FLASH_DBG
#else /* DBG_IMAGE */
#define FLASH_DBG(...)
#define FLASH_WRN(...)
#define FLASH_ERR(...)
#endif /* DBG_IMAGE */

uint32_t flash_read(SF_Handler *hdl, uint32_t src_addr, void *buf, uint32_t size);
uint32_t flash_write(SF_Handler *hdl, uint32_t dst_addr, void *buf, uint32_t size);
int32_t flash_erase_check(SF_Handler *hdl, uint32_t addr, uint32_t size);
uint32_t flash_erase(SF_Handler *hdl, uint32_t addr, uint32_t size);

#endif /* _IMAGE_FLASH_H_ */