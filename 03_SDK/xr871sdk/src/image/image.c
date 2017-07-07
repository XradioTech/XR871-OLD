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

#include "flash.h"
#include "image_debug.h"
#include "sys/image.h"

#define image_malloc(l)			malloc(l)
#define image_free(p)			free(p)
#define image_memcpy(d, s, n)	memcpy(d, s, n)
#define image_memset(s, c, n) 	memset(s, c, n)

#define IMAGE_CHECK_SIZE	(1024)
#define IMAGE_EXPAND_SIZE	(5)

typedef struct {
	uint32_t	id;
	uint32_t	offset;
}sec_offset_t;

typedef struct {
	uint8_t					sec_num[IMAGE_SEQ_NUM];
	sec_offset_t		   *sec_offset[IMAGE_SEQ_NUM];
	uint32_t				boot_offset;
	uint32_t				image_offset[IMAGE_SEQ_NUM];
	image_seq_t				image_seq;
	image_flash_init_cb		init_cb;
	image_flash_deinit_cb	deinit_cb;
} image_priv_t;

image_priv_t	image_priv;

static uint16_t image_checksum16(uint8_t *data, uint32_t len)
{
	uint16_t cs = 0;
	uint16_t *p = (uint16_t *)data;

	while (len > 1) {
		cs += *p++;
		len -= 2;
	}
	if (len) {
		cs += *(uint8_t *)p;
	}

	return cs;
}

static sec_offset_t *image_expand_offset(image_seq_t seq)
{
	sec_offset_t *sec_offset;

	if (image_priv.sec_num[seq] == 0)
		return image_malloc(IMAGE_EXPAND_SIZE * sizeof(sec_offset_t));

	sec_offset = (sec_offset_t *)image_malloc((image_priv.sec_num[seq] + IMAGE_EXPAND_SIZE) * sizeof(sec_offset_t));
	if (sec_offset) {
		image_memcpy(sec_offset, image_priv.sec_offset[seq], image_priv.sec_num[seq] * sizeof(sec_offset_t));
		image_free(image_priv.sec_offset[seq]);
	}

	return sec_offset;
}

static uint32_t image_get_offset(SF_Handler *flash_hdl, image_seq_t seq, uint32_t id)
{
	uint8_t			index;
	uint32_t		offset;
	uint32_t		next_addr;
	sec_offset_t   *sec_offset;

	for (index = 0; index < image_priv.sec_num[seq]; index++) {
		if (id == image_priv.sec_offset[seq][index].id)
			return image_priv.sec_offset[seq][index].offset;
	}

	if (image_priv.sec_num[seq] == 0) {
		offset = image_priv.boot_offset;
	} else {
		offset = image_priv.sec_offset[seq][image_priv.sec_num[seq] - 1].offset;
		flash_read(flash_hdl, offset + offsetof(section_header_t, next_addr), &next_addr, sizeof(next_addr));
		offset = image_priv.image_offset[seq] + next_addr;
	}

	while (next_addr != IMAGE_INVALID_OFFSET) {
		if ((image_priv.sec_num[seq] % IMAGE_EXPAND_SIZE) == 0) {
			sec_offset = image_expand_offset(seq);
			if (sec_offset == NULL) {
				IMAGE_ERR("image get offset failed: failed to expand offset\n");
				return IMAGE_INVALID_OFFSET;
			} else {
				image_priv.sec_offset[seq] = sec_offset;
			}
		}

		image_priv.sec_offset[seq][image_priv.sec_num[seq]].offset = offset;
		flash_read(flash_hdl, offset + offsetof(section_header_t, id),
				   &(image_priv.sec_offset[seq][image_priv.sec_num[seq]].id), sizeof(id));
		image_priv.sec_num[seq]++;

		if (id == image_priv.sec_offset[seq][image_priv.sec_num[seq] - 1].id) {
			return image_priv.sec_offset[seq][image_priv.sec_num[seq] - 1].offset;
		}
		flash_read(flash_hdl, offset + offsetof(section_header_t, next_addr),
				   &next_addr, sizeof(next_addr));
		offset = image_priv.image_offset[seq] + next_addr;
	}

	IMAGE_ERR("image get offset failed: invalid section ID %#010x\n", id);
	return IMAGE_INVALID_OFFSET;
}

static void image_clear_offset(void)
{
	image_seq_t seq;

	for (seq = IMAGE_SEQ_1ST; seq < IMAGE_SEQ_NUM; seq++) {
		image_priv.sec_num[seq] = 0;
		if (image_priv.sec_offset[seq]) {
			image_free(image_priv.sec_offset[seq]);
			image_priv.sec_offset[seq] = NULL;
		}
	}
}

uint16_t image_get_checksum(void *buf, uint32_t len)
{
	if (buf == NULL) {
		IMAGE_ERR("image get checksum failed: invalid paremeter\n");
		return 0;
	}

	return image_checksum16(buf, len);
}

image_validity_t image_check_header(section_header_t *sh)
{
	if (sh == NULL) {
		IMAGE_ERR("image check header failed: invalid parameter\n");
		return IMAGE_INVALID;
	}

	if ((sh->magic_number != IMAGE_MAGIC_NUMBER)
		|| (0xFFFF != image_get_checksum(sh, IMAGE_HEADER_SIZE))) {
		IMAGE_WARN("image check header failed: magic number %#010x, checksum %#06x\n",
				  sh->magic_number, image_get_checksum(sh, IMAGE_HEADER_SIZE));
		return IMAGE_INVALID;
	}

	return IMAGE_VALID;
}

image_validity_t image_check_data(section_header_t *sh,
								  void *body, uint32_t body_len,
								  void *tailer, uint32_t tailer_len)
{
	uint8_t joint[2];
	uint16_t data_checksum;

	if ((sh == NULL)
		|| ((body == NULL) && (body_len != 0))
		|| ((tailer == NULL) && (tailer_len != 0))) {
		IMAGE_ERR("image check data failed: invalid parameter\n");
		return IMAGE_INVALID;
	}

	if (tailer_len == 0) {
		data_checksum = sh->data_chksum
						+ image_checksum16(body, body_len);
	} else if ((body_len & 0x1) == 0) {
		data_checksum = sh->data_chksum
						+ image_checksum16(body, body_len)
						+ image_checksum16(tailer, tailer_len);
	} else {
		image_memcpy(joint, ((uint8_t *)body) + body_len - 1, 1);
		image_memcpy(joint + 1, tailer, 1);
		data_checksum = sh->data_chksum
						+ image_checksum16(body, body_len - 1)
						+ image_checksum16(joint, 2)
						+ image_checksum16(((uint8_t *)tailer) + 1, tailer_len - 1);
	}

	if (data_checksum != 0xFFFF) {
		IMAGE_WARN("image check data failed: checksum %#06x\n", data_checksum);
		return IMAGE_INVALID;
	}

	return IMAGE_VALID;
}

static image_validity_t _image_check_section(SF_Handler *flash_hdl, uint32_t offset)
{
	uint16_t			data_chksum;
	uint32_t			data_size;
	uint8_t			   *buf_check;
	section_header_t   *sh;

	sh = (section_header_t *)image_malloc(IMAGE_HEADER_SIZE);
	if (sh == NULL) {
		IMAGE_ERR("image check section failed: malloc failed\n");
		return IMAGE_INVALID;
	}

	flash_read(flash_hdl, offset, sh, IMAGE_HEADER_SIZE);
	if (image_check_header(sh) == IMAGE_INVALID) {
		IMAGE_ERR("image check section failed: failed to check header\n");
		image_free(sh);
		return IMAGE_INVALID;
	}

	data_chksum = sh->data_chksum;
	data_size = sh->data_size;
	image_free(sh);

	buf_check = (uint8_t *)image_malloc(IMAGE_CHECK_SIZE);
	if (buf_check == NULL) {
		IMAGE_ERR("image check section failed: malloc failed\n");
		return IMAGE_INVALID;
	}

	offset += IMAGE_HEADER_SIZE;
	while (data_size > 0) {
		if (data_size >= IMAGE_CHECK_SIZE) {
			flash_read(flash_hdl, offset, buf_check, IMAGE_CHECK_SIZE);
			data_chksum += image_checksum16(buf_check, IMAGE_CHECK_SIZE);
			offset += IMAGE_CHECK_SIZE;
			data_size -= IMAGE_CHECK_SIZE;
		} else {
			flash_read(flash_hdl, offset, buf_check, data_size);
			data_chksum += image_checksum16(buf_check, data_size);
			offset += data_size;
			data_size -= data_size;
		}
	}
	image_free(buf_check);

	if (data_chksum != 0xFFFF) {
		IMAGE_WARN("image check section failed: data checksum %#06x\n", data_chksum);
		return IMAGE_INVALID;
	} else {
		return IMAGE_VALID;
	}
}

image_validity_t image_check_section(image_handle_t *hdl, image_seq_t seq, uint32_t id)
{
	uint32_t	offset;

	if (hdl == NULL) {
		IMAGE_ERR("image check section failed: invalid parameter\n");
		return IMAGE_INVALID;
	}

	offset = image_get_offset(&hdl->flash_handle, seq, id);
	if (offset == IMAGE_INVALID_OFFSET) {
		IMAGE_ERR("image check section failed: failed to get offset\n");
		return IMAGE_INVALID;
	}

	IMAGE_DBG("image check section: id %#010x, offset %#010x\n", id, offset);

	return _image_check_section(&hdl->flash_handle, offset);
}

image_validity_t image_check_sections(image_handle_t *hdl, image_seq_t seq, uint32_t id)
{
	uint32_t	offset;
	uint32_t	next_addr = 0;

	if (hdl == NULL) {
		IMAGE_ERR("image check sections failed: invalid parameter\n");
		return IMAGE_INVALID;
	}

	offset = image_get_offset(&hdl->flash_handle, seq, id);
	if (offset == IMAGE_INVALID_OFFSET) {
		IMAGE_ERR("image check sections failed: failed to get offset\n");
		return IMAGE_INVALID;
	}

	IMAGE_DBG("image check sections: id %#010x\n", id);

	while (next_addr != IMAGE_INVALID_OFFSET) {
		IMAGE_DBG("image check sections: offset %#010x\n", offset);
		if (_image_check_section(&hdl->flash_handle, offset) == IMAGE_INVALID)
			return IMAGE_INVALID;
		flash_read(&hdl->flash_handle, offset + offsetof(section_header_t, next_addr),
				   &next_addr, sizeof(next_addr));
		offset = image_priv.image_offset[seq] + next_addr;
	}

	return IMAGE_VALID;
}

uint32_t image_get_section_addr(image_handle_t *hdl, uint32_t id)
{
	uint32_t addr;

	if (hdl == NULL) {
		IMAGE_WARN("hdl is NULL\n");
		return IMAGE_INVALID_OFFSET;
	}

	addr = image_get_offset(&hdl->flash_handle, image_priv.image_seq, id);
	if (addr == IMAGE_INVALID_OFFSET) {
		IMAGE_WARN("get section 0x%x addr failed\n", id);
	}

	return addr;
}

void image_init(uint32_t boot_offset, uint32_t image_offset_1st, uint32_t image_offset_2nd,
				image_seq_t image_seq, image_flash_init_cb init_cb, image_flash_deinit_cb deinit_cb)
{
	if ((init_cb == NULL) || (deinit_cb == NULL) || (image_seq >= IMAGE_SEQ_NUM))
		IMAGE_ERR("image init failed: invalid parameter\n");

	image_clear_offset();
	image_priv.boot_offset	= boot_offset;
	image_priv.image_offset[IMAGE_SEQ_1ST] = image_offset_1st;
	image_priv.image_offset[IMAGE_SEQ_2ND] = image_offset_2nd;
	image_priv.image_seq	= image_seq;
	image_priv.init_cb		= init_cb;
	image_priv.deinit_cb	= deinit_cb;
}

image_handle_t *image_open(void)
{
	image_handle_t *hdl;

	if (image_priv.init_cb == NULL) {
		IMAGE_ERR("image open failed: without image init\n");
		return NULL;
	}

	hdl = (image_handle_t *)image_malloc(sizeof(image_handle_t));
	if (hdl == NULL) {
		IMAGE_ERR("image open failed: malloc failed\n");
		return NULL;
	}

	image_priv.init_cb(&hdl->flash_handle);

	return hdl;
}

uint32_t image_read(image_handle_t *hdl, uint32_t id, image_seg_t seg,
					uint32_t offset, void *buf, uint32_t size)
{
	uint32_t addr;
	uint32_t body_len;

	if ((hdl == NULL) || (buf == NULL)) {
		IMAGE_ERR("image read failed: invalid parameter\n");
		return 0;
	}

	addr = image_get_offset(&hdl->flash_handle, image_priv.image_seq, id);
	if (addr == IMAGE_INVALID_OFFSET) {
		IMAGE_ERR("image read failed: failed to get offset\n");
		return 0;
	}

	switch (seg) {
	case IMAGE_SEG_HEADER:
		break;
	case IMAGE_SEG_BODY:
		addr += IMAGE_HEADER_SIZE;
		break;
	case IMAGE_SEG_TAILER:
		flash_read(&hdl->flash_handle, addr + offsetof(section_header_t, body_len),
				   &body_len, sizeof(body_len));
		addr += IMAGE_HEADER_SIZE + body_len;
		break;
	default :
		IMAGE_ERR("image read failed: invalid segment %d\n", seg);
		return 0;
	}

	addr += offset;

	return flash_read(&hdl->flash_handle, addr, buf, size);
}

uint32_t image_write(image_handle_t *hdl, uint32_t id, image_seg_t seg,
					 uint32_t offset, void *buf, uint32_t size)
{
	uint32_t addr;
	uint32_t body_len;

	if ((hdl == NULL) || (buf == NULL)) {
		IMAGE_ERR("image write failed: invalid parameter\n");
		return 0;
	}

	addr = image_get_offset(&hdl->flash_handle, image_priv.image_seq, id);
	if (addr == IMAGE_INVALID_OFFSET) {
		IMAGE_ERR("image write failed: failed to get offset\n");
		return 0;
	}

	switch (seg) {
	case IMAGE_SEG_HEADER:
		image_clear_offset();
		break;
	case IMAGE_SEG_BODY:
		addr += IMAGE_HEADER_SIZE;
		break;
	case IMAGE_SEG_TAILER:
		flash_read(&hdl->flash_handle, addr + offsetof(section_header_t, body_len),
				   &body_len, sizeof(body_len));
		addr += IMAGE_HEADER_SIZE + body_len;
		break;
	default :
		IMAGE_ERR("image write failed: invalid segment %d\n", seg);
		return 0;
	}

	addr += offset;

	IMAGE_DBG("image write: section ID %#010x, segment %d, addr %#010x\n", id, seg, addr);
	return flash_write(&hdl->flash_handle, addr, buf, size);
}

void image_close(image_handle_t *hdl)
{
	if (hdl == NULL) {
		IMAGE_ERR("image close failed: invalid parameter\n");
		return;
	}

	if (image_priv.deinit_cb) {
		image_priv.deinit_cb(&hdl->flash_handle);
	}

	image_free(hdl);
	hdl = NULL;
}

void image_deinit(void)
{
	image_clear_offset();
	image_priv.boot_offset	= 0;
	image_priv.image_offset[IMAGE_SEQ_1ST] = 0;
	image_priv.image_offset[IMAGE_SEQ_2ND] = 0;
	image_priv.image_seq	= 0;
	image_priv.init_cb		= NULL;
	image_priv.deinit_cb	= NULL;
}
