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

#include "ctrl_img.h"
#include "ctrl_img_debug.h"

#include "common/board/board.h"
#include "driver/chip/hal_wdg.h"
#include "sys/image.h"
#include "sys/fdcm.h"
#include "sys/ota.h"

void ctrl_img_init(uint32_t boot_offset,
				   uint32_t boot_cfg_offset,
				   uint32_t image_offset_1st,
				   uint32_t image_offset_2nd)
{
	image_seq_t		seq;
#ifdef __CONFIG_OTA_UPDATE
	ota_cfg			cfg;
	image_handle_t *image_hdl;
#endif /* __CONFIG_OTA_UPDATE */

	fdcm_init(board_flash_init, board_flash_deinit);

#ifdef __CONFIG_OTA_UPDATE
	ota_init(boot_offset, boot_cfg_offset, image_offset_1st,
			 image_offset_2nd, board_flash_init, board_flash_deinit);
	ota_read_cfg(&cfg);

	if (cfg.image == OTA_IMAGE_1ST) {
		CTRL_IMG_DBG("ctrl image: image 1st\n");
		seq = IMAGE_SEQ_1ST;
	} else if (cfg.image == OTA_IMAGE_2ND) {
		CTRL_IMG_DBG("ctrl image: image 2nd\n");
		seq = IMAGE_SEQ_2ND;
	} else {
		CTRL_IMG_ERR("image init failed: cfg image %d\n", cfg.image);
		return;
	}
#else /* __CONFIG_OTA_UPDATE */
	seq = IMAGE_SEQ_1ST;
#endif /* __CONFIG_OTA_UPDATE */
	image_init(boot_offset, image_offset_1st, image_offset_2nd,
			   seq, board_flash_init, board_flash_deinit);

#ifdef __CONFIG_OTA_UPDATE
	if (cfg.state != OTA_STATE_VERIFIED) {
		image_hdl = image_open();
		if (image_check_sections(image_hdl, seq, IMAGE_APP_ID) == IMAGE_INVALID) {
			CTRL_IMG_WARN("check sections invalid: seq %d\n", seq);
			if (cfg.image == OTA_IMAGE_1ST)
				cfg.image = OTA_IMAGE_2ND;
			else if (cfg.image == OTA_IMAGE_2ND)
				cfg.image = OTA_IMAGE_1ST;
			cfg.state = OTA_STATE_UNVERIFIED;
			ota_write_cfg(&cfg);
			HAL_WDG_Reboot();
		}
		CTRL_IMG_DBG("check sections OK: seq %d\n", seq);
		cfg.state = OTA_STATE_VERIFIED;
		ota_write_cfg(&cfg);
		image_close(image_hdl);
	}
#endif /* __CONFIG_OTA_UPDATE */
}

