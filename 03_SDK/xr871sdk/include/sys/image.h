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

#ifndef _SYS_IMAGE_H_
#define _SYS_IMAGE_H_

#include <stdio.h>
#include <string.h>

#include "types.h"
#include "kernel/os/os.h"
#include "driver/chip/hal_spi.h"
#include "driver/chip/hal_norflash.h"

#define IMAGE_BOOT_ID			(0xA5FF5A00)
#define IMAGE_APP_ID			(0xA5FE5A01)
#define IMAGE_APP_XIP_ID		(0xA5FD5A02)
#define IMAGE_NET_ID			(0xA5FC5A03)
#define IMAGE_NET_AP_ID			(0xA5FB5A04)
#define IMAGE_WLAN_BL_ID		(0xA5FA5A05)
#define IMAGE_WLAN_FW_ID		(0xA5F95A06)
#define IMAGE_WLAN_SDD_ID		(0xA5F85A07)

#define IMAGE_INVALID_OFFSET	(0xFFFFFFFF)

typedef int (*image_flash_init_cb)(SF_Handler *hdl);
typedef void (*image_flash_deinit_cb)(SF_Handler *hdl);

typedef enum image_sequence {
	IMAGE_SEQ_1ST		= 0,
	IMAGE_SEQ_2ND		= 1,
	IMAGE_SEQ_NUM		= 2,
} image_seq_t;

typedef enum image_validity {
	IMAGE_INVALID		= 0,
	IMAGE_VALID			= 1,
} image_validity_t;

typedef enum image_segment {
	IMAGE_SEG_HEADER	= 0,
	IMAGE_SEG_BODY		= 1,
	IMAGE_SEG_TAILER	= 2,
} image_seg_t;

typedef struct image_handle {
	SF_Handler		flash_handle;
} image_handle_t;

/* section header magic number (AWIH) */
#define IMAGE_MAGIC_NUMBER	(0x48495741)

/* section header (64B) */
typedef struct section_header {
	uint32_t magic_number;	/* magic number			*/
	uint32_t version;		/* version: 0.0.0.0		*/
	uint16_t header_chksum;	/* header checksum		*/
	uint16_t data_chksum;	/* data checksum		*/
	uint32_t data_size;		/* data size			*/
	uint32_t load_addr;		/* load address			*/
	uint32_t entry;			/* entry point			*/
	uint32_t body_len;		/* body length			*/
	uint32_t attribute;		/* attribute			*/
	uint32_t next_addr;		/* next section address	*/
	uint32_t id;			/* section ID			*/
	uint32_t priv[6];		/* private data			*/
} section_header_t;

#define IMAGE_HEADER_SIZE	sizeof(section_header_t)

uint16_t image_get_checksum(void *buf, uint32_t len);
image_validity_t image_check_header(section_header_t *sh);
image_validity_t image_check_data(section_header_t *sh,
								  void *body, uint32_t body_len,
								  void *tailer, uint32_t tailer_len);

image_validity_t image_check_section(image_handle_t *hdl, image_seq_t seq, uint32_t id);
image_validity_t image_check_sections(image_handle_t *hdl, image_seq_t seq, uint32_t id);
uint32_t image_get_section_addr(image_handle_t *hdl, uint32_t id);

void image_init(uint32_t boot_offset, uint32_t image_offset_1st, uint32_t image_offset_2nd,
				image_seq_t image_seq, image_flash_init_cb init_cb, image_flash_deinit_cb deinit_cb);
image_handle_t *image_open(void);
uint32_t image_read(image_handle_t *hdl, uint32_t id, image_seg_t seg,
					uint32_t offset, void *buf, uint32_t size);
uint32_t image_write(image_handle_t *hdl, uint32_t id, image_seg_t seg,
					 uint32_t offset, void *buf, uint32_t size);
void image_close(image_handle_t *hdl);
void image_deinit(void);

#endif /* _SYS_IMAGE_H_ */
