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
#ifdef __CONFIG_ARCH_DUAL_CORE
#include "ota_http.h"
#endif /* __CONFIG_ARCH_DUAL_CORE */
#include "sys/ota.h"
#include "sys/fdcm.h"
#include "sys/image.h"
#include "driver/chip/hal_wdg.h"

static ota_priv_t	ota_priv;

static uint32_t ota_get_boot_size(void)
{
	SF_Handler	flash_hdl;
	uint32_t	next_addr;

	ota_priv.init_cb(&flash_hdl);
	HAL_SF_Read(&flash_hdl, ota_priv.boot_offset + offsetof(section_header_t, next_addr),
				(uint8_t *)&next_addr, sizeof(next_addr));
	ota_priv.deinit_cb(&flash_hdl);

	OTA_DBG("ota get boot size: %#010x\n", next_addr);

	return next_addr;
}

static ota_status ota_read_boot_cfg(ota_boot_cfg * boot_cfg)
{
	fdcm_handle_t *fdcm_hdl;
	uint32_t boot_cfg_size;

	boot_cfg_size = ota_get_boot_size();

	fdcm_hdl = fdcm_open(ota_priv.boot_cfg_offset, boot_cfg_size);
	if (fdcm_hdl == NULL) {
		OTA_ERR("ota read boot cfg failed: fdcm open failed\n");
		return OTA_STATUS_ERROR;
	}

	if (fdcm_read(fdcm_hdl, boot_cfg, OTA_BOOT_CFG_SIZE) != OTA_BOOT_CFG_SIZE) {
		OTA_ERR("ota read boot cfg failed: fdcm read failed\n");
		fdcm_close(fdcm_hdl);
		return OTA_STATUS_ERROR;
	}

	fdcm_close(fdcm_hdl);

	return OTA_STATUS_OK;
}

static ota_status ota_write_boot_cfg(ota_boot_cfg * boot_cfg)
{
	fdcm_handle_t *fdcm_hdl;
	uint32_t boot_cfg_size;

	boot_cfg_size = ota_get_boot_size();

	fdcm_hdl = fdcm_open(ota_priv.boot_cfg_offset, boot_cfg_size);
	if (fdcm_hdl == NULL) {
		OTA_ERR("ota write boot cfg failed: fdcm open failed\n");
		return OTA_STATUS_ERROR;
	}

	if (fdcm_write(fdcm_hdl, boot_cfg, OTA_BOOT_CFG_SIZE) != OTA_BOOT_CFG_SIZE) {
		OTA_ERR("ota write boot cfg failed: fdcm write failed\n");
		fdcm_close(fdcm_hdl);
		return OTA_STATUS_ERROR;
	}

	fdcm_close(fdcm_hdl);

	return OTA_STATUS_OK;
}

#ifdef __CONFIG_ARCH_DUAL_CORE
static ota_status ota_erase_flash(SF_Handler *hdl, uint32_t addr, uint32_t size)
{
	uint32_t		start;
	uint32_t		multiples;
	SF_Erase_Size	erase_size;
	HAL_Status		status;

	if ((size >= (64 << 10))
		&& (HAL_SF_MemoryOf(hdl, SF_ERASE_SIZE_64KB, addr, &start) == HAL_OK)
		&& (addr == start)
		&& ((size & 0xFFFF) == 0)) {
		multiples = size / (64 << 10);
		erase_size = SF_ERASE_SIZE_64KB;
	} else if ((size >= (32 << 10))
			   && (HAL_SF_MemoryOf(hdl, SF_ERASE_SIZE_32KB, addr, &start) == HAL_OK)
			   && (addr == start)
			   && ((size & 0x7FFF) == 0)) {
		multiples = size / (32 << 10);
		erase_size = SF_ERASE_SIZE_32KB;
	} else if ((size >= (4 << 10))
			   && (HAL_SF_MemoryOf(hdl, SF_ERASE_SIZE_4KB, addr, &start) == HAL_OK)
			   && (addr == start)
			   && ((size & 0xFFF) == 0)) {
		multiples = size / (32 << 10);
		erase_size = SF_ERASE_SIZE_32KB;
	} else {
		OTA_ERR("ota erase flash failed: addr %#010x, size %#010x\n", addr, size);
		return OTA_STATUS_ERROR;
	}

	status = HAL_SF_Erase(hdl, erase_size, addr, multiples);
	if (status != HAL_OK) {
		OTA_ERR("flash erase failed: status %d\n", status);
		return OTA_STATUS_ERROR;
	}

	return OTA_STATUS_OK;
}

static ota_status ota_check_image(image_seq_t seq)
{
	image_handle_t *image_hdl;

	image_hdl = image_open();
	if (image_hdl == NULL) {
		OTA_ERR("ota check image failed: image open failed\n");
		return OTA_STATUS_ERROR;
	}

	if (image_check_sections(image_hdl, seq, IMAGE_APP_ID) == IMAGE_INVALID) {
		OTA_ERR("ota check image failed\n");
		return OTA_STATUS_ERROR;
	}

	image_close(image_hdl);

	return OTA_STATUS_OK;
}
#endif /* __CONFIG_ARCH_DUAL_CORE */

ota_status ota_init(uint32_t boot_offset, uint32_t boot_cfg_offset,
					uint32_t image_offset_1st, uint32_t image_offset_2nd,
					ota_flash_init_cb init_cb, ota_flash_deinit_cb deinit_cb)
{
	if ((init_cb == NULL) || (deinit_cb == NULL)) {
		OTA_ERR("ota init failed: invalid parameter\n");
		return OTA_STATUS_ERROR;
	}

	ota_priv.boot_offset		= boot_offset;
	ota_priv.boot_cfg_offset	= boot_cfg_offset;
	ota_priv.image_offset_1st	= image_offset_1st;
	ota_priv.image_offset_2nd	= image_offset_2nd;
	ota_priv.init_cb			= init_cb;
	ota_priv.deinit_cb			= deinit_cb;

	return OTA_STATUS_OK;
}

ota_status ota_read_cfg(ota_cfg *cfg)
{
	ota_boot_cfg	boot_cfg;

	if ((ota_priv.init_cb == NULL) || (ota_priv.deinit_cb == NULL)) {
		OTA_ERR("ota read cfg failed: without ota init\n");
		return OTA_STATUS_ERROR;
	}

	if (cfg == NULL) {
		OTA_ERR("ota read cfg failed: invalid parameter\n");
		return OTA_STATUS_ERROR;
	}

	if (ota_read_boot_cfg(&boot_cfg) == OTA_STATUS_ERROR) {
		OTA_ERR("ota read cfg failed: ota read boot cfg failed\n");
		return OTA_STATUS_ERROR;
	}

	if (boot_cfg.boot_image == OTA_BOOT_IMAGE_1ST) {
		cfg->image = OTA_IMAGE_1ST;
	} else if (boot_cfg.boot_image == OTA_BOOT_IMAGE_2ND) {
		cfg->image = OTA_IMAGE_2ND;
	} else {
		OTA_ERR("ota read cfg failed: boot cfg image %#06x\n", boot_cfg.boot_image);
		return OTA_STATUS_ERROR;
	}

	if (boot_cfg.boot_state == OTA_BOOT_STATE_UNVERIFIED) {
		cfg->state = OTA_STATE_UNVERIFIED;
	} else if (boot_cfg.boot_state == OTA_BOOT_STATE_VERIFIED) {
		cfg->state = OTA_STATE_VERIFIED;
	} else {
		OTA_ERR("ota read cfg failed: boot cfg state %#06x\n", boot_cfg.boot_state);
		return OTA_STATUS_ERROR;
	}

	return OTA_STATUS_OK;
}

ota_status ota_write_cfg(ota_cfg *cfg)
{
	ota_boot_cfg	boot_cfg;

	if ((ota_priv.init_cb == NULL) || (ota_priv.deinit_cb == NULL)) {
		OTA_ERR("ota write cfg failed: without ota init\n");
		return OTA_STATUS_ERROR;
	}

	if (cfg == NULL) {
		OTA_ERR("ota write cfg failed: invalid parameter\n");
		return OTA_STATUS_ERROR;
	}

	if (cfg->image == OTA_IMAGE_1ST) {
		boot_cfg.boot_image = OTA_BOOT_IMAGE_1ST;
	} else if (cfg->image == OTA_IMAGE_2ND) {
		boot_cfg.boot_image = OTA_BOOT_IMAGE_2ND;
	} else {
		OTA_ERR("ota write cfg failed: cfg image %d\n", cfg->image);
		return OTA_STATUS_ERROR;
	}

	if (cfg->state == OTA_STATE_UNVERIFIED) {
		boot_cfg.boot_state= OTA_BOOT_STATE_UNVERIFIED;
	} else if (cfg->state == OTA_STATE_VERIFIED) {
		boot_cfg.boot_state = OTA_BOOT_STATE_VERIFIED;
	} else {
		OTA_ERR("ota write cfg failed: cfg state %d\n", cfg->state);
		return OTA_STATUS_ERROR;
	}

	if (ota_write_boot_cfg(&boot_cfg) == OTA_STATUS_ERROR) {
		OTA_ERR("ota write cfg failed: ota write boot cfg failed\n");
		return OTA_STATUS_ERROR;
	}

	return OTA_STATUS_OK;
}

#ifdef __CONFIG_ARCH_DUAL_CORE
ota_status ota_update(void *cmd)
{
	ota_status		status;
	ota_boot_cfg	boot_cfg;
	uint32_t		boot_size;
	uint32_t		addr;
	image_seq_t		seq;

	SF_Handler		flash_hdl;
	uint32_t		flash_size;
	uint32_t		recv_size;
	uint8_t		   *ota_buf;
	uint8_t			eof_flag;

	if ((ota_priv.init_cb == NULL) || (ota_priv.deinit_cb == NULL)) {
		OTA_ERR("ota update failed: without init\n");
		return OTA_STATUS_ERROR;
	}

	ota_read_boot_cfg(&boot_cfg);
	boot_size = ota_get_boot_size();

	if (((boot_cfg.boot_image == OTA_BOOT_IMAGE_1ST) && (boot_cfg.boot_state == OTA_BOOT_STATE_UNVERIFIED))
		|| ((boot_cfg.boot_image == OTA_BOOT_IMAGE_2ND) && (boot_cfg.boot_state == OTA_BOOT_STATE_VERIFIED))) {
		addr = ota_priv.image_offset_1st + boot_size;
		boot_cfg.boot_image = OTA_BOOT_IMAGE_1ST;
		seq = IMAGE_SEQ_1ST;
	} else if (((boot_cfg.boot_image == OTA_BOOT_IMAGE_1ST) && (boot_cfg.boot_state == OTA_BOOT_STATE_VERIFIED))
			   || ((boot_cfg.boot_image == OTA_BOOT_IMAGE_2ND) && (boot_cfg.boot_state == OTA_BOOT_STATE_UNVERIFIED))) {
		addr = ota_priv.image_offset_2nd + boot_size;
		boot_cfg.boot_image = OTA_BOOT_IMAGE_2ND;
		seq = IMAGE_SEQ_2ND;
	} else {
		OTA_ERR("ota update failed: image %#06x, state %#06x\n", boot_cfg.boot_image, boot_cfg.boot_state);
		return OTA_STATUS_ERROR;
	}

	ota_priv.init_cb(&flash_hdl);

	if (ota_erase_flash(&flash_hdl, addr, ota_priv.image_offset_2nd - ota_priv.image_offset_1st - boot_size) != OTA_STATUS_OK) {
		OTA_ERR("ota update failed: erase flash failed\n");
		return OTA_STATUS_ERROR;
	}

	ota_buf = (uint8_t *)ota_malloc(OTA_BUF_SIZE);
	if (ota_buf == NULL) {
		OTA_ERR("ota updata failed: malloc failed\n");
		return OTA_STATUS_ERROR;
	}

	if (ota_update_http_init(cmd) != OTA_STATUS_OK) {
		OTA_ERR("ota update failed: ota update http init failed\n");
		ota_free(ota_buf);
		return OTA_STATUS_ERROR;
	}

	while (boot_size > 0) {
		if (boot_size > OTA_BUF_SIZE)
			status = ota_update_http_get(ota_buf, OTA_BUF_SIZE, &recv_size, &eof_flag);
		else
			status = ota_update_http_get(ota_buf, boot_size, &recv_size, &eof_flag);
		if ((status != OTA_STATUS_OK) || eof_flag) {
			OTA_ERR("ota update failed: ota update http get failed, eof %d\n", eof_flag);
			ota_free(ota_buf);
			return OTA_STATUS_ERROR;
		}
		boot_size -= recv_size;
	}

	flash_size = ota_priv.image_offset_2nd - ota_priv.image_offset_1st - boot_size;
	while (flash_size > 0) {
		if (flash_size > OTA_BUF_SIZE)
			status = ota_update_http_get(ota_buf, OTA_BUF_SIZE, &recv_size, &eof_flag);
		else
			status = ota_update_http_get(ota_buf, flash_size, &recv_size, &eof_flag);
		if (status != OTA_STATUS_OK) {
			OTA_ERR("ota update failed: ota update http get failed\n");
			ota_free(ota_buf);
			return OTA_STATUS_ERROR;
		}
		flash_size -= recv_size;
		if (HAL_SF_Write(&flash_hdl, addr, ota_buf, recv_size) != HAL_OK) {
			OTA_ERR("ota update failed: flash write failed\n");
			ota_free(ota_buf);
			return OTA_STATUS_ERROR;
		}
		addr += recv_size;
		if (eof_flag)
			break;
	}
	ota_free(ota_buf);

	OTA_DBG("ota update: finish getting image\n");

	if (ota_check_image(seq) != OTA_STATUS_OK) {
		OTA_ERR("ota update failed: ota check image failed\n");
		return OTA_STATUS_ERROR;
	}

	boot_cfg.boot_state = OTA_BOOT_STATE_UNVERIFIED;
	ota_write_boot_cfg(&boot_cfg);
	HAL_WDG_Reboot();

	return OTA_STATUS_OK;
}
#endif /* __CONFIG_ARCH_DUAL_CORE */

void ota_deinit(void)
{
	ota_priv.boot_offset		= IMAGE_INVALID_OFFSET;
	ota_priv.boot_cfg_offset	= IMAGE_INVALID_OFFSET;
	ota_priv.image_offset_1st	= IMAGE_INVALID_OFFSET;
	ota_priv.image_offset_2nd	= IMAGE_INVALID_OFFSET;
	ota_priv.init_cb			= NULL;
	ota_priv.deinit_cb			= NULL;
}
