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

#include <stdint.h>

#include "img_ctrl.h"
#include "img_ctrl_debug.h"

#include "common/board/board.h"
#include "driver/chip/hal_wdg.h"
#include "sys/image.h"
#include "sys/ota.h"

static __inline void img_ctrl_reboot(void)
{
	HAL_WDG_Reboot();
}

static void img_ctrl_ota_init(image_ota_param_t *param)
{
	image_seq_t	seq;
	ota_cfg_t	cfg;

	ota_init(param);
	if (ota_read_cfg(&cfg) != OTA_STATUS_OK)
		IMG_CTRL_ERR("%s(), %d, ota read cfg failed\n", __func__, __LINE__);

	if (cfg.image == OTA_IMAGE_1ST) {
		seq = IMAGE_SEQ_1ST;
	} else if (cfg.image == OTA_IMAGE_2ND) {
		seq = IMAGE_SEQ_2ND;
	} else {
		IMG_CTRL_ERR("%s(), %d, invalid image %d\n", __func__, __LINE__, cfg.image);
		seq = IMAGE_SEQ_1ST;
	}
	image_set_running_seq(seq);

	if (cfg.state != OTA_STATE_VERIFIED) {
		if (image_check_sections(seq) != IMAGE_VALID) {
			IMG_CTRL_WRN("%s(), %d, seq %d, invalid image\n", __func__, __LINE__, seq);
			if (seq == IMAGE_SEQ_1ST)
				cfg.image = OTA_IMAGE_2ND;
			else if (seq == IMAGE_SEQ_2ND)
				cfg.image = OTA_IMAGE_1ST;
			cfg.state = OTA_STATE_UNVERIFIED;
			ota_write_cfg(&cfg);
			img_ctrl_reboot();
		}
		if (seq == IMAGE_SEQ_1ST)
			cfg.image = OTA_IMAGE_1ST;
		else if (seq == IMAGE_SEQ_2ND)
			cfg.image = OTA_IMAGE_2ND;
		cfg.state = OTA_STATE_VERIFIED;
		ota_write_cfg(&cfg);
	}
}

void img_ctrl_init(uint32_t flash, uint32_t addr, uint32_t size)
{
	image_ota_param_t	param;

	image_init(flash, addr, size);
	image_get_ota_param(&param);

	if (param.addr[IMAGE_SEQ_2ND] == IMAGE_INVALID_ADDR)
		image_set_running_seq(IMAGE_SEQ_1ST);
	else
		img_ctrl_ota_init(&param);
}

