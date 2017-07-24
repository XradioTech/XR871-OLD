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

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sys/io.h"
#include "errno.h"
#include "driver/chip/hal_chip.h"
#include "sys/list.h"
#include "sys/image.h"
#include "sys/fdcm.h"
#include "sys/ota.h"
#include "kernel/os/os_time.h"

#include "sys/xr_debug.h"
#include "common/board/board.h"

#define DBG_BOOTLODER  0

#define BOOT_DBG(fmt, arg...) printf("[BOOT] "fmt, ##arg)
#define BOOT_WRN BOOT_DBG
#define BOOT_ERR BOOT_DBG

#if (DBG_BOOTLODER == 1)
#define boot_hex_dump_bytes(...) print_hex_dump_bytes(__VA_ARGS__)
#define BOOT_INF BOOT_DBG
#else
#define boot_hex_dump_bytes(...)
#define BOOT_INF(...)
#endif

#define BOOT_ABORT()	do { } while (1)

typedef enum {
	NORMAL_BOOT = 0,
	RESUME_BOOT = 1,
	RECOVERY_BOOT = 3
} boot_type;

#if (defined(__CONFIG_CHIP_XR871))
#define IMAGE_BOOT_OFFSET		(0x00000000)
#endif
#define IMAGE_BOOT_CFG_OFFSET	(IMAGE_BOOT_OFFSET + (1 << 20))

#define IMAGE_OFFSET_1ST		IMAGE_BOOT_OFFSET
#define IMAGE_OFFSET_2ND		(IMAGE_BOOT_OFFSET + (1 << 20))

static section_header_t section_header;

static void recovery(void)
{
	BOOT_INF("%s\n", __func__);

	//image_header_t *ih = &image_head;
	//SF_Handler hdl;

	/* step 1: init spi and flash. */

	/* step 2: erase boot area. */

	/* step 3: copy back area to boot area. */
	/* copy by list for each back section if back area is not the same as boot area. */
	//image_list_for_each_section(&hdl, IMAGE_BACK_BOOT_OFFSET, ih) {
		/* copy back area data to boot area data. */

		/* recalculate ih and write to image head area. */
		//ih->ih_nxtsecaddr -= IMAGE_BACK_BOOT_OFFSET;
		//ih->ih_hchksum = ;
		//...
	//}
}

static void reboot(void)
{
	BOOT_INF("%s\n", __func__);

	HAL_WDG_Reboot();
}

typedef void (*bl_goto)(void);

void bootloader(void)
{
#ifdef __CONFIG_OTA_UPDATE
	ota_cfg	cfg;
#endif /* __CONFIG_OTA_UPDATE */
	section_header_t *sh = &section_header;
	image_handle_t *hdl;

	BOOT_INF("%s,%d\n", __func__, __LINE__);

	if (HAL_PRCM_GetCPUABootFlag() == NORMAL_BOOT) {
		if (board_init() != 0) {
			BOOT_ERR("board init failed\n");
			BOOT_ABORT();
		}

#ifdef __CONFIG_OTA_UPDATE
		fdcm_init(board_flash_init, board_flash_deinit);
		ota_init(IMAGE_BOOT_OFFSET, IMAGE_BOOT_CFG_OFFSET, IMAGE_OFFSET_1ST,
				 IMAGE_OFFSET_2ND, board_flash_init, board_flash_deinit);
		ota_read_cfg(&cfg);

		if (cfg.image == OTA_IMAGE_1ST) {
			BOOT_DBG("bootloader: image 1st\n");
			image_init(IMAGE_BOOT_OFFSET, IMAGE_OFFSET_1ST, IMAGE_OFFSET_2ND,
				   IMAGE_SEQ_1ST, board_flash_init, board_flash_deinit);
		} else if (cfg.image == OTA_IMAGE_2ND) {
			BOOT_DBG("bootloader: image 2nd\n");
			image_init(IMAGE_BOOT_OFFSET, IMAGE_OFFSET_1ST, IMAGE_OFFSET_2ND,
				   IMAGE_SEQ_2ND, board_flash_init, board_flash_deinit);
		} else {
			BOOT_ERR("image init failed: cfg image %d\n", cfg.image);
			BOOT_ABORT();
		}
#else /* __CONFIG_OTA_UPDATE */
		image_init(IMAGE_BOOT_OFFSET, IMAGE_OFFSET_1ST, IMAGE_OFFSET_2ND,
				   IMAGE_SEQ_1ST, board_flash_init, board_flash_deinit);
#endif /* __CONFIG_OTA_UPDATE */
		hdl = image_open();
		if (hdl == NULL) {
			BOOT_ERR("image open failed\n");
			BOOT_ABORT();
		}

		if ((image_read(hdl, IMAGE_APP_ID, IMAGE_SEG_HEADER, 0, sh, IMAGE_HEADER_SIZE) != IMAGE_HEADER_SIZE)
			|| (image_check_header(sh) == IMAGE_INVALID)
			|| (image_read(hdl, IMAGE_APP_ID, IMAGE_SEG_BODY, 0, (void *)sh->load_addr, sh->body_len) != sh->body_len)
			|| (image_check_data(sh, (void *)sh->load_addr, sh->body_len, NULL, 0) == IMAGE_INVALID)) {
			BOOT_ERR("load app failed\n");
#ifdef __CONFIG_OTA_UPDATE
			if (cfg.image == OTA_IMAGE_1ST) {
				cfg.image = OTA_IMAGE_2ND;
				cfg.state = OTA_STATE_UNVERIFIED;
			} else if (cfg.image == OTA_IMAGE_2ND) {
				cfg.image = OTA_IMAGE_1ST;
				cfg.state = OTA_STATE_UNVERIFIED;
			}
			ota_write_cfg(&cfg);
			reboot();
#else /* __CONFIG_OTA_UPDATE */
			BOOT_ABORT();
#endif /* __CONFIG_OTA_UPDATE */
		}

		bl_goto ep = (bl_goto)sh->entry;
		BOOT_INF("%s,%d goto %p\n", __func__, __LINE__, ep);
//		boot_hex_dump_bytes((const void *)0x010000, 64);
		image_close(hdl);
#ifdef __CONFIG_OTA_UPDATE
		ota_deinit();
		fdcm_deinit();
#endif
		image_deinit();
		board_uart_deinit(BOARD_MAIN_UART_ID);
		board_deinit();
		SystemDeInit();
		__disable_fault_irq();
		__disable_irq();
		__set_CONTROL(0); /* reset to Privileged Thread mode and use MSP */
		ep(); /* never return */

		/* recovery image if run here */
		HAL_PRCM_SetCPUABootFlag(RECOVERY_BOOT);
	}

	if (HAL_PRCM_GetCPUABootFlag() == RECOVERY_BOOT) {
		recovery();
		HAL_PRCM_SetCPUABootFlag(NORMAL_BOOT);
	}

	reboot();
}
