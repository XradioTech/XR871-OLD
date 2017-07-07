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
#include "sys/fdcm.h"
#include "image_debug.h"

#define fdcm_malloc(l)			malloc(l)
#define fdcm_free(p)			free(p)
#define fdcm_memcpy(d, s, n)	memcpy(d, s, n)
#define fdcm_memset(s, c, n) 	memset(s, c, n)

typedef OS_Mutex_t		fdcm_mutex_t;

static inline fdcm_status fdcm_mutex_create(fdcm_mutex_t *mutex)
{
	return OS_MutexCreate(mutex) == OS_OK ? FDCM_OK : FDCM_ERROR;
}

static inline fdcm_status fdcm_mutex_delete(fdcm_mutex_t *mutex)
{
	return OS_MutexDelete(mutex) == OS_OK ? FDCM_OK : FDCM_ERROR;
}

static inline fdcm_status fdcm_mutex_lock(fdcm_mutex_t *mutex, uint32_t msec)
{
	return OS_MutexLock(mutex, msec) == OS_OK ? FDCM_OK : FDCM_ERROR;
}

static inline fdcm_status fdcm_mutex_unlock(fdcm_mutex_t *mutex)
{
	return OS_MutexUnlock(mutex) == OS_OK ? FDCM_OK : FDCM_ERROR;
}

#define FDCM_ID_CODE		(0xA55A5AA5)
#define FDCM_INDEX_MAX		(1 << 10)
#define FDCM_WAIT_TIME		(10000)
#define FDCM_BYTE_BITS		(8)

typedef struct fdcm_header {
	uint32_t	id_code;
	uint16_t	bitmap_size;
	uint16_t	data_size;
} fdcm_header_t;

#define FDCM_HEADER_SIZE	sizeof(fdcm_header_t)

typedef struct {
	uint32_t				cnt;
	fdcm_mutex_t			mutex;
	SF_Handler				flash_hdl;
	fdcm_flash_init_cb		init_cb;
	fdcm_flash_deinit_cb	deinit_cb;
} fdcm_priv_t;

fdcm_priv_t	fdcm_priv;

static uint32_t	fdcm_bit_count(uint8_t *bitmap, uint16_t size)
{
	uint32_t	byte_cnt = 0;
	uint32_t	bit_cnt = 0;
	uint8_t		val;

	while (byte_cnt < size) {
		if (*bitmap == 0) {
			bit_cnt += FDCM_BYTE_BITS;
		}
		else {
			val = ~(*bitmap);
			while (val) {
				val = val >> 1;
				bit_cnt++;
			}
			break;
		}
		bitmap++;
		byte_cnt++;
	}

	return bit_cnt;
}

static uint32_t fdcm_rewrite(fdcm_handle_t *hdl, void *data, uint16_t data_size)
{
	uint8_t			bitmap_byte = 0xFE;
	fdcm_header_t	header;

	if (flash_erase(&fdcm_priv.flash_hdl, hdl->addr, hdl->size) != hdl->size) {
		IMAGE_ERR("fdcm rewrite failed: failed to erase flash\n");
		return 0;
	}

	header.id_code = FDCM_ID_CODE;
	header.bitmap_size = (hdl->size - FDCM_HEADER_SIZE - 1) / (data_size * FDCM_BYTE_BITS + 1) + 1;
	if (header.bitmap_size > FDCM_INDEX_MAX)
		header.bitmap_size = FDCM_INDEX_MAX;
	header.data_size = data_size;

	if (flash_write(&fdcm_priv.flash_hdl, hdl->addr, &header, FDCM_HEADER_SIZE)
		&& flash_write(&fdcm_priv.flash_hdl, hdl->addr + FDCM_HEADER_SIZE, &bitmap_byte, 1)
		&& flash_write(&fdcm_priv.flash_hdl, hdl->addr + FDCM_HEADER_SIZE + header.bitmap_size, data, data_size)) {
		IMAGE_DBG("fdcm rewrite OK\n");
		return data_size;
	} else {
		IMAGE_ERR("fdcm rewrite failed: failed to write flash\n");
		return 0;
	}
}

fdcm_status fdcm_init(fdcm_flash_init_cb init_cb, fdcm_flash_deinit_cb deinit_cb)
{
	if ((init_cb == NULL) || (deinit_cb == NULL)) {
		IMAGE_ERR("fdcm init failed: invalid parameter\n");
		return FDCM_ERROR;
	}

	if (fdcm_mutex_create(&fdcm_priv.mutex) != FDCM_OK) {
		IMAGE_ERR("fdcm init failed: failed to create mutex\n");
		return FDCM_ERROR;
	}

	fdcm_priv.cnt		= 0;
	fdcm_priv.init_cb	= init_cb;
	fdcm_priv.deinit_cb	= deinit_cb;

	return FDCM_OK;
}

fdcm_handle_t *fdcm_open(uint32_t addr, uint32_t size)
{
	fdcm_handle_t *hdl;

	if ((fdcm_priv.init_cb == NULL) || (fdcm_priv.deinit_cb == NULL)) {
		IMAGE_ERR("fdcm open failed: without fdcm init\n");
		return NULL;
	}

	hdl = (fdcm_handle_t *)fdcm_malloc(sizeof(fdcm_handle_t));
	if (hdl == NULL) {
		IMAGE_ERR("fdcm open failed: malloc failed\n");
		return NULL;
	}

	fdcm_mutex_lock(&fdcm_priv.mutex, FDCM_WAIT_TIME);
	if (fdcm_priv.cnt == 0)
		fdcm_priv.init_cb(&fdcm_priv.flash_hdl);
	fdcm_priv.cnt++;
	fdcm_mutex_unlock(&fdcm_priv.mutex);

	if (flash_erase_check(&fdcm_priv.flash_hdl, addr, size) != 0) {
		IMAGE_ERR("fdcm open failed: invalid parameter, addr %#010x, size %#010x\n", addr, size);
		fdcm_mutex_lock(&fdcm_priv.mutex, FDCM_WAIT_TIME);
		fdcm_priv.cnt--;
		if (fdcm_priv.cnt == 0)
			fdcm_priv.deinit_cb(&fdcm_priv.flash_hdl);
		fdcm_mutex_unlock(&fdcm_priv.mutex);
		fdcm_free(hdl);
		return NULL;
	}

	hdl->addr = addr;
	hdl->size = size;

	return hdl;
}

uint32_t fdcm_read(fdcm_handle_t *hdl, void *data, uint16_t data_size)
{
	fdcm_header_t	header;
	uint8_t		   *bitmap;
	uint32_t		bit_cnt;
	uint32_t		addr;

	if ((hdl == NULL) || (data == NULL)) {
		IMAGE_ERR("fdcm read failed: invalid parameter\n");
		return 0;
	}

	flash_read(&fdcm_priv.flash_hdl, hdl->addr, &header, FDCM_HEADER_SIZE);

	IMAGE_DBG("fdcm header: id code %#010x, bitmap size %#06x, data size %#06x\n",
			  header.id_code, header.bitmap_size, header.data_size);

	if ((header.id_code != FDCM_ID_CODE)
		|| (header.data_size != data_size)
		|| (header.bitmap_size == 0)
		|| (header.bitmap_size + FDCM_HEADER_SIZE >= hdl->size)) {
		IMAGE_WARN("fdcm read failed: invalid fdcm header\n");
		return 0;
	}

	bitmap = (uint8_t *)fdcm_malloc(header.bitmap_size);
	if (bitmap == NULL) {
		IMAGE_ERR("fdcm read failed: malloc failed\n");
		return 0;
	}

	flash_read(&fdcm_priv.flash_hdl, hdl->addr + FDCM_HEADER_SIZE, bitmap, header.bitmap_size);
	bit_cnt = fdcm_bit_count(bitmap, header.bitmap_size);
	fdcm_free(bitmap);

	addr = hdl->addr + FDCM_HEADER_SIZE + header.bitmap_size + data_size * (bit_cnt - 1);
	return flash_read(&fdcm_priv.flash_hdl, addr, data, data_size);
}

uint32_t fdcm_write(fdcm_handle_t *hdl, void *data, uint16_t data_size)
{
	fdcm_header_t	header;
	uint8_t		   *bitmap;
	uint32_t		bit_cnt;
	uint32_t		addr;

	if ((hdl == NULL) || (data == NULL)) {
		IMAGE_ERR("fdcm write failed: invalid parameter\n");
		return 0;
	}

	flash_read(&fdcm_priv.flash_hdl, hdl->addr, &header, FDCM_HEADER_SIZE);

	IMAGE_DBG("fdcm header: id code %#010x, bitmap size %#06x, data size %#06x\n",
			  header.id_code, header.bitmap_size, header.data_size);

	if ((header.id_code != FDCM_ID_CODE)
		|| (header.data_size != data_size)
		|| (header.bitmap_size == 0)
		|| (header.bitmap_size + FDCM_HEADER_SIZE >= hdl->size)) {
		return fdcm_rewrite(hdl, data, data_size);
	}

	bitmap = (uint8_t *)fdcm_malloc(header.bitmap_size);
	if (bitmap == NULL) {
		IMAGE_ERR("fdcm write failed: malloc failed\n");
		return 0;
	}

	flash_read(&fdcm_priv.flash_hdl, hdl->addr + FDCM_HEADER_SIZE, bitmap, header.bitmap_size);
	bit_cnt = fdcm_bit_count(bitmap, header.bitmap_size);
	if ((bit_cnt == FDCM_BYTE_BITS * header.bitmap_size)
		|| (FDCM_HEADER_SIZE + header.bitmap_size + data_size * (bit_cnt + 1) > hdl->size)) {
		fdcm_free(bitmap);
		return fdcm_rewrite(hdl, data, data_size);
	}

	addr = hdl->addr + FDCM_HEADER_SIZE + bit_cnt / FDCM_BYTE_BITS;
	bitmap[bit_cnt / FDCM_BYTE_BITS] &= ~(0x1 << (bit_cnt % FDCM_BYTE_BITS));
	flash_write(&fdcm_priv.flash_hdl, addr, bitmap + bit_cnt / FDCM_BYTE_BITS, 1);
	fdcm_free(bitmap);

	addr = hdl->addr + FDCM_HEADER_SIZE + header.bitmap_size + data_size * bit_cnt;
	return flash_write(&fdcm_priv.flash_hdl, addr, data, data_size);
}

fdcm_status fdcm_close(fdcm_handle_t *hdl)
{
	if (hdl == NULL) {
		IMAGE_ERR("fdcm close failed: invalid parameter\n");
		return FDCM_ERROR;
	}

	fdcm_free(hdl);
	hdl = NULL;

	fdcm_mutex_lock(&fdcm_priv.mutex, FDCM_WAIT_TIME);
	fdcm_priv.cnt--;
	if (fdcm_priv.cnt == 0)
		fdcm_priv.deinit_cb(&fdcm_priv.flash_hdl);
	fdcm_mutex_unlock(&fdcm_priv.mutex);

	return 0;
}

fdcm_status fdcm_deinit(void)
{
	fdcm_mutex_lock(&fdcm_priv.mutex, FDCM_WAIT_TIME);
	if (fdcm_priv.cnt) {
		fdcm_priv.deinit_cb(&fdcm_priv.flash_hdl);
		fdcm_priv.cnt = 0;
	}
	fdcm_mutex_unlock(&fdcm_priv.mutex);

	if (fdcm_mutex_delete(&fdcm_priv.mutex) != FDCM_OK) {
		IMAGE_ERR("fdcm deinit failed: failed to delete mutex\n");
		return FDCM_ERROR;
	}

	fdcm_priv.init_cb	= NULL;
	fdcm_priv.deinit_cb	= NULL;

	return FDCM_OK;
}

