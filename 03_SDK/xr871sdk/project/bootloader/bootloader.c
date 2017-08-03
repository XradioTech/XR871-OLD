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

#include "prj_config.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sys/io.h"
#include "driver/chip/system_chip.h"
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

typedef void (*boot_entry)(void);

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

static __inline void bl_hw_init(void)
{
	if (board_spi_init(BOARD_FLASH_SPI_PORT) != HAL_OK) {
		BOOT_ERR("spi init failed\n");
	}
}

static __inline void bl_hw_deinit(void)
{
	board_spi_deinit(BOARD_FLASH_SPI_PORT);
#if PRJCONF_UART_EN
	board_uart_deinit(BOARD_MAIN_UART_ID);
#endif
	SystemDeInit();
}

static __inline void bl_image_init(void)
{
	image_seq_t seq;

#ifdef __PRJ_CONFIG_OTA
	ota_cfg cfg;
	fdcm_init(board_flash_init, board_flash_deinit);
	ota_init(PRJCONF_IMG_BOOT_OFFSET, PRJCONF_IMG_BOOT_CFG_OFFSET,
	         PRJCONF_IMG_OFFSET_1ST, PRJCONF_IMG_OFFSET_2ND,
	         board_flash_init, board_flash_deinit);
	ota_read_cfg(&cfg);

	if (cfg.image == OTA_IMAGE_1ST) {
		seq = IMAGE_SEQ_1ST;
	} else if (cfg.image == OTA_IMAGE_2ND) {
		seq = IMAGE_SEQ_2ND;
	} else {
		BOOT_ERR("image init failed: cfg image %d\n", cfg.image);
		BOOT_ABORT();
	}
#else /* __PRJ_CONFIG_OTA */
	seq = IMAGE_SEQ_1ST;
#endif /* __PRJ_CONFIG_OTA */
	BOOT_INF("image seq %d\n", seq);
	image_init(PRJCONF_IMG_BOOT_OFFSET, PRJCONF_IMG_OFFSET_1ST,
	           PRJCONF_IMG_OFFSET_2ND, seq,
	           board_flash_init, board_flash_deinit);
}

static __inline void bl_image_deinit(void)
{
#ifdef __PRJ_CONFIG_OTA
	ota_deinit();
	fdcm_deinit();
#endif
	image_deinit();
}

static __inline boot_entry bl_load_bin(void)
{
	image_handle_t *hdl;
	section_header_t *sh = &section_header;

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
#ifdef __PRJ_CONFIG_OTA
		ota_cfg cfg;
		if (cfg.image == OTA_IMAGE_1ST) {
			cfg.image = OTA_IMAGE_2ND;
			cfg.state = OTA_STATE_UNVERIFIED;
		} else if (cfg.image == OTA_IMAGE_2ND) {
			cfg.image = OTA_IMAGE_1ST;
			cfg.state = OTA_STATE_UNVERIFIED;
		}
		ota_write_cfg(&cfg);
		reboot();
#else /* __PRJ_CONFIG_OTA */
		BOOT_ABORT();
#endif /* __PRJ_CONFIG_OTA */
	}

	image_close(hdl);
	return (boot_entry)sh->entry;
}

void bootloader(void)
{
	BOOT_INF("bl start\n");

	if (HAL_PRCM_GetCPUABootFlag() == NORMAL_BOOT) {
		bl_hw_init();
		bl_image_init();
		boot_entry ep = bl_load_bin();
		BOOT_INF("bl goto %p\n", ep);
//		boot_hex_dump_bytes((const void *)0x010000, 64);
		bl_image_deinit();
		bl_hw_deinit();

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
