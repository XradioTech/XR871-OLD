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

#include "ota_i.h"
#include "ota_debug.h"
#include "ota_file.h"
#include "ota_http.h"
#include "sys/ota.h"
#include "sys/fdcm.h"
#include "sys/image.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/hal_wdg.h"

static ota_priv_t	ota_priv;

ota_status_t ota_init(image_ota_param_t *param)
{
	if (param == NULL) {
		OTA_ERR("param %p\n", param);
		return OTA_STATUS_ERROR;
	}

	ota_priv.flash[IMAGE_SEQ_1ST]	= param->flash[IMAGE_SEQ_1ST];
	ota_priv.addr[IMAGE_SEQ_1ST]	= param->addr[IMAGE_SEQ_1ST];
	ota_priv.flash[IMAGE_SEQ_2ND]	= param->flash[IMAGE_SEQ_2ND];
	ota_priv.addr[IMAGE_SEQ_2ND]	= param->addr[IMAGE_SEQ_2ND];
	ota_priv.image_size				= param->image_size;
	ota_priv.boot_size				= param->boot_size;

	OTA_DBG("%s(), %d, flash_1st %d, addr_1st %#010x, flash_2nd %d, "
			"addr_2nd %#010x, image_size %#010x, boot_size %#010x\n",
			__func__, __LINE__,
			ota_priv.flash[IMAGE_SEQ_1ST], ota_priv.addr[IMAGE_SEQ_1ST],
			ota_priv.flash[IMAGE_SEQ_2ND], ota_priv.addr[IMAGE_SEQ_2ND],
			ota_priv.image_size, ota_priv.boot_size);

	return OTA_STATUS_OK;
}

void ota_deinit(void)
{
	ota_priv.flash[IMAGE_SEQ_1ST]	= 0;
	ota_priv.addr[IMAGE_SEQ_1ST]	= IMAGE_INVALID_ADDR;
	ota_priv.flash[IMAGE_SEQ_2ND]	= 0;
	ota_priv.addr[IMAGE_SEQ_2ND]	= IMAGE_INVALID_ADDR;
	ota_priv.image_size				= 0;
	ota_priv.boot_size				= 0;
}

static ota_status_t ota_read_boot_cfg(ota_boot_cfg_t *boot_cfg)
{
	fdcm_handle_t *fdcm_hdl;

	fdcm_hdl = fdcm_open(ota_priv.flash[IMAGE_SEQ_2ND], ota_priv.addr[IMAGE_SEQ_2ND], ota_priv.boot_size);
	if (fdcm_hdl == NULL) {
		OTA_ERR("fdcm hdl %p\n", fdcm_hdl);
		return OTA_STATUS_ERROR;
	}

	if (fdcm_read(fdcm_hdl, boot_cfg, OTA_BOOT_CFG_SIZE) != OTA_BOOT_CFG_SIZE) {
		OTA_ERR("fdcm read failed\n");
		fdcm_close(fdcm_hdl);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, boot image %#06x, boot state %#06x\n",
			__func__, __LINE__, boot_cfg->boot_image, boot_cfg->boot_state);

	fdcm_close(fdcm_hdl);
	return OTA_STATUS_OK;
}

static ota_status_t ota_write_boot_cfg(ota_boot_cfg_t * boot_cfg)
{
	fdcm_handle_t *fdcm_hdl;

	fdcm_hdl = fdcm_open(ota_priv.flash[IMAGE_SEQ_2ND], ota_priv.addr[IMAGE_SEQ_2ND], ota_priv.boot_size);
	if (fdcm_hdl == NULL) {
		OTA_ERR("fdcm hdl %p\n", fdcm_hdl);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, boot image %#06x, boot state %#06x\n",
			__func__, __LINE__, boot_cfg->boot_image, boot_cfg->boot_state);

	if (fdcm_write(fdcm_hdl, boot_cfg, OTA_BOOT_CFG_SIZE) != OTA_BOOT_CFG_SIZE) {
		OTA_ERR("fdcm write failed\n");
		fdcm_close(fdcm_hdl);
		return OTA_STATUS_ERROR;
	}

	fdcm_close(fdcm_hdl);
	return OTA_STATUS_OK;
}

ota_status_t ota_read_cfg(ota_cfg_t *cfg)
{
	ota_boot_cfg_t	boot_cfg;

	if (cfg == NULL) {
		OTA_ERR("cfg %p\n", cfg);
		return OTA_STATUS_ERROR;
	}

	if (ota_read_boot_cfg(&boot_cfg) != OTA_STATUS_OK) {
		OTA_ERR("read boot cfg failed\n");
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, boot image %#06x, boot state %#06x\n",
			__func__, __LINE__, boot_cfg.boot_image, boot_cfg.boot_state);

	if (boot_cfg.boot_image == OTA_BOOT_IMAGE_1ST) {
		cfg->image = OTA_IMAGE_1ST;
	} else if (boot_cfg.boot_image == OTA_BOOT_IMAGE_2ND) {
		cfg->image = OTA_IMAGE_2ND;
	} else {
		OTA_ERR("boot image %#06x\n", boot_cfg.boot_image);
		return OTA_STATUS_ERROR;
	}

	if (boot_cfg.boot_state == OTA_BOOT_STATE_UNVERIFIED) {
		cfg->state = OTA_STATE_UNVERIFIED;
	} else if (boot_cfg.boot_state == OTA_BOOT_STATE_VERIFIED) {
		cfg->state = OTA_STATE_VERIFIED;
	} else {
		OTA_ERR("boot state %#06x\n", boot_cfg.boot_state);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, image %d, state %d\n", __func__, __LINE__, cfg->image, cfg->state);

	return OTA_STATUS_OK;
}

ota_status_t ota_write_cfg(ota_cfg_t *cfg)
{
	ota_boot_cfg_t	boot_cfg;

	if (cfg == NULL) {
		OTA_ERR("cfg %p\n", cfg);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, image %d, state %d\n", __func__, __LINE__, cfg->image, cfg->state);

	if (cfg->image == OTA_IMAGE_1ST) {
		boot_cfg.boot_image = OTA_BOOT_IMAGE_1ST;
	} else if (cfg->image == OTA_IMAGE_2ND) {
		boot_cfg.boot_image = OTA_BOOT_IMAGE_2ND;
	} else {
		OTA_ERR("image %d\n", cfg->image);
		return OTA_STATUS_ERROR;
	}

	if (cfg->state == OTA_STATE_UNVERIFIED) {
		boot_cfg.boot_state= OTA_BOOT_STATE_UNVERIFIED;
	} else if (cfg->state == OTA_STATE_VERIFIED) {
		boot_cfg.boot_state = OTA_BOOT_STATE_VERIFIED;
	} else {
		OTA_ERR("tate %d\n", cfg->state);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, boot image %#06x, boot state %#06x\n",
			__func__, __LINE__, boot_cfg.boot_image, boot_cfg.boot_state);

	if (ota_write_boot_cfg(&boot_cfg) != OTA_STATUS_OK) {
		OTA_ERR("write boot cfg failed\n");
		return OTA_STATUS_ERROR;
	}

	return OTA_STATUS_OK;
}

static ota_status_t ota_erase_flash(uint32_t flash, uint32_t addr, uint32_t size)
{
	uint32_t		start;
	uint32_t		multiples;
	FlashEraseMode	erase_size;
	HAL_Status		status;

	OTA_DBG("%s(), %d, flash %d, addr %#010x, size %#010x\n",
			__func__, __LINE__, flash, addr, size);

	if ((size >= (64 << 10))
		&& (HAL_Flash_MemoryOf(flash, FLASH_ERASE_64KB, addr, &start) == HAL_OK)
		&& (addr == start)
		&& ((size & 0xFFFF) == 0)) {
		multiples = size / (64 << 10);
		erase_size = FLASH_ERASE_64KB;
	} else if ((size >= (32 << 10))
			   && (HAL_Flash_MemoryOf(flash, FLASH_ERASE_32KB, addr, &start) == HAL_OK)
			   && (addr == start)
			   && ((size & 0x7FFF) == 0)) {
		multiples = size / (32 << 10);
		erase_size = FLASH_ERASE_32KB;
	} else if ((size >= (4 << 10))
			   && (HAL_Flash_MemoryOf(flash, FLASH_ERASE_4KB, addr, &start) == HAL_OK)
			   && (addr == start)
			   && ((size & 0xFFF) == 0)) {
		multiples = size / (4 << 10);
		erase_size = FLASH_ERASE_4KB;
	} else {
		OTA_ERR("flash %d, addr %#010x, size %#010x\n", flash, addr, size);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, erase size %#010x, multiples %d\n",
			__func__, __LINE__, erase_size, multiples);

	if (HAL_Flash_Open(flash, OTA_FLASH_TIMEOUT) != HAL_OK) {
		OTA_ERR("flash %d\n", flash);
		return OTA_STATUS_ERROR;
	}

	status = HAL_Flash_Erase(flash, erase_size, addr, multiples);
	if (status != HAL_OK) {
		OTA_ERR("status %d\n", status);
		HAL_Flash_Close(flash);
		return OTA_STATUS_ERROR;
	}

	HAL_Flash_Close(flash);
	return OTA_STATUS_OK;
}

static ota_status_t ota_update(void *url, ota_update_init_t init_cb, ota_update_get_t get_cb)
{
	ota_boot_cfg_t	boot_cfg;
	ota_status_t	status;
	image_seq_t		seq;
	uint32_t		flash;
	uint32_t		addr;
	uint32_t		boot_size;
	uint32_t		recv_size;
	uint32_t		image_size;
	uint8_t		   *ota_buf;
	uint8_t			eof_flag;
	uint8_t			cnt;

	if ((url == NULL) || (init_cb == NULL) || (get_cb == NULL)) {
		OTA_ERR("url %p, init cb %p, get cb %p\n", url, init_cb, get_cb);
		return OTA_STATUS_ERROR;
	}

	if (ota_read_boot_cfg(&boot_cfg) != OTA_STATUS_OK) {
		OTA_ERR("read boot cfg failed\n");
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, boot image %#06x, boot state %#06x\n",
			__func__, __LINE__, boot_cfg.boot_image, boot_cfg.boot_state);

	if (((boot_cfg.boot_image == OTA_BOOT_IMAGE_1ST) && (boot_cfg.boot_state == OTA_BOOT_STATE_UNVERIFIED))
		|| ((boot_cfg.boot_image == OTA_BOOT_IMAGE_2ND) && (boot_cfg.boot_state == OTA_BOOT_STATE_VERIFIED))) {
		seq = IMAGE_SEQ_1ST;
		flash = ota_priv.flash[IMAGE_SEQ_1ST];
		addr = ota_priv.addr[IMAGE_SEQ_1ST] + ota_priv.boot_size;
		boot_cfg.boot_image = OTA_BOOT_IMAGE_1ST;
	} else if (((boot_cfg.boot_image == OTA_BOOT_IMAGE_1ST) && (boot_cfg.boot_state == OTA_BOOT_STATE_VERIFIED))
			   || ((boot_cfg.boot_image == OTA_BOOT_IMAGE_2ND) && (boot_cfg.boot_state == OTA_BOOT_STATE_UNVERIFIED))) {
		seq = IMAGE_SEQ_2ND;
		flash = ota_priv.flash[IMAGE_SEQ_2ND];
		addr = ota_priv.addr[IMAGE_SEQ_2ND] + ota_priv.boot_size;
		boot_cfg.boot_image = OTA_BOOT_IMAGE_2ND;
	} else {
		OTA_ERR("boot image %#06x, boot state %#06x\n", boot_cfg.boot_image, boot_cfg.boot_state);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, seq %d, flash %d, addr %#010x\n", __func__, __LINE__, seq, flash, addr);

	if (ota_erase_flash(flash, addr, ota_priv.image_size - ota_priv.boot_size) != OTA_STATUS_OK) {
		OTA_ERR("flash %d, addr %#010x, image size %#010x\n", flash, addr, ota_priv.image_size);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, erase flash success\n", __func__, __LINE__);

	ota_buf = ota_malloc(OTA_BUF_SIZE);
	if (ota_buf == NULL) {
		OTA_ERR("ota buf %p\n", ota_buf);
		return OTA_STATUS_ERROR;
	}

	if (init_cb(url) != OTA_STATUS_OK) {
		OTA_ERR("ota update init failed\n");
		ota_free(ota_buf);
		return OTA_STATUS_ERROR;
	}

	OTA_DBG("%s(), %d, ota update init success\n", __func__, __LINE__);

	OTA_SYSLOG("OTA: loading image...\n");
	cnt = 0;

	boot_size = ota_priv.boot_size;
	while (boot_size > 0) {
		if (boot_size > OTA_BUF_SIZE)
			status = get_cb(ota_buf, OTA_BUF_SIZE, &recv_size, &eof_flag);
		else
			status = get_cb(ota_buf, boot_size, &recv_size, &eof_flag);
		if ((status != OTA_STATUS_OK) || eof_flag) {
			OTA_ERR("status %d, eof %d\n", status, eof_flag);
			ota_free(ota_buf);
			return OTA_STATUS_ERROR;
		}
		boot_size -= recv_size;

		cnt++;
		if (cnt == 100) {
			OTA_SYSLOG("OTA: loading image...\n");
			cnt = 0;
		}
	}

	OTA_DBG("%s(), %d, load bootloader success\n", __func__, __LINE__);

	if (HAL_Flash_Open(flash, OTA_FLASH_TIMEOUT) != HAL_OK) {
		OTA_ERR("flash %d\n", flash);
		return OTA_STATUS_ERROR;
	}

	image_size = ota_priv.image_size;
	while (image_size > 0) {
		if (image_size > OTA_BUF_SIZE)
			status = get_cb(ota_buf, OTA_BUF_SIZE, &recv_size, &eof_flag);
		else
			status = get_cb(ota_buf, image_size, &recv_size, &eof_flag);
		if (status != OTA_STATUS_OK) {
			OTA_ERR("status %d\n", status);
			HAL_Flash_Close(flash);
			ota_free(ota_buf);
			return OTA_STATUS_ERROR;
		}
		image_size -= recv_size;
		if (HAL_Flash_Write(flash, addr, ota_buf, recv_size) != HAL_OK) {
			OTA_ERR("flash %d, addr %#010x, size %#010x\n", flash, addr, recv_size);
			HAL_Flash_Close(flash);
			ota_free(ota_buf);
			return OTA_STATUS_ERROR;
		}
		addr += recv_size;
		if (eof_flag)
			break;

		cnt++;
		if (cnt == 100) {
			OTA_SYSLOG("OTA: loading image...\n");
			cnt = 0;
		}
	}

	HAL_Flash_Close(flash);
	ota_free(ota_buf);

	OTA_SYSLOG("OTA: finish loading image.\n");

	if (image_check_sections(seq) != IMAGE_VALID) {
		OTA_ERR("ota check image failed\n");
		return OTA_STATUS_ERROR;
	}

	OTA_SYSLOG("OTA: finish checking image.\n");

	boot_cfg.boot_state = OTA_BOOT_STATE_UNVERIFIED;
	OTA_DBG("%s(), %d, boot image %#06x, boot state %#06x\n",
			__func__, __LINE__, boot_cfg.boot_image, boot_cfg.boot_state);
	if (ota_write_boot_cfg(&boot_cfg) != OTA_STATUS_OK) {
		OTA_ERR("write boot cfg failed\n");
		return OTA_STATUS_ERROR;
	}

	OTA_SYSLOG("OTA: reboot.\n");

	HAL_WDG_Reboot();

	return OTA_STATUS_OK;
}

ota_status_t ota_update_file(void *url)
{
	return ota_update(url, ota_update_file_init, ota_update_file_get);
}

ota_status_t ota_update_http(void *url)
{
	return ota_update(url, ota_update_http_init, ota_update_http_get);
}

