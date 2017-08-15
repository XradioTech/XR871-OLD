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
#include "driver/chip/system_chip.h"
#include "driver/chip/hal_chip.h"
#include "sys/list.h"
#include "sys/image.h"
#include "sys/fdcm.h"
#include "sys/ota.h"
#include "kernel/os/os_time.h"

#include "common/board/board.h"
#include "bl_debug.h"


static __inline void bl_reboot(void)
{
	HAL_WDG_Reboot();
}

static __inline void bl_hw_init(void)
{
	if (board_spi_init(BOARD_FLASH_SPI_PORT) != HAL_OK) {
		BL_ERR("spi init failed\n");
	}
}

static __inline void bl_hw_deinit(void)
{
	board_spi_deinit(BOARD_FLASH_SPI_PORT);
#if PRJCONF_UART_EN
#if BL_DBG_ON
	while (!HAL_UART_IsTxEmpty(HAL_UART_GetInstance(BOARD_MAIN_UART_ID))) { }
#endif
	board_uart_deinit(BOARD_MAIN_UART_ID);
#endif
	SystemDeInit(0);
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
		seq = IMAGE_SEQ_1ST;
		BL_ERR("image init failed: cfg image %d\n", cfg.image);
	}
#else /* __PRJ_CONFIG_OTA */
	seq = IMAGE_SEQ_1ST;
#endif /* __PRJ_CONFIG_OTA */
	BL_DBG("image seq %d\n", seq);
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

static __inline uint32_t bl_load_bin(void)
{
	extern const unsigned char __RAM_BASE[];	/* SRAM start address */
	image_handle_t *hdl;
	uint32_t len;
	section_header_t sh;

	hdl = image_open();
	if (hdl == NULL) {
		BL_ERR("image open failed\n");
	}

	len = image_read(hdl, IMAGE_APP_ID, IMAGE_SEG_HEADER, 0, &sh, IMAGE_HEADER_SIZE);
	if (len != IMAGE_HEADER_SIZE) {
		BL_WRN("app header size %u, read %u\n", IMAGE_HEADER_SIZE, len);
		goto err;
	}

	if (image_check_header(&sh) == IMAGE_INVALID) {
		BL_WRN("invalid app bin header\n");
		goto err;
	}

	if (sh.load_addr + sh.body_len > (uint32_t)__RAM_BASE) {
		BL_WRN("app overlap with bl, %#x + %#x > %p\n", sh.load_addr, sh.body_len, __RAM_BASE);
	}

	len = image_read(hdl, IMAGE_APP_ID, IMAGE_SEG_BODY, 0, (void *)sh.load_addr, sh.body_len);
	if (len != sh.body_len) {
		BL_WRN("app body size %u, read %u\n", sh.body_len, len);
		goto err;
	}

	if (image_check_data(&sh, (void *)sh.load_addr, sh.body_len, NULL, 0) == IMAGE_INVALID) {
		BL_WRN("invalid app bin body\n");
		goto err;
	}
	goto out;

err:
#ifdef __PRJ_CONFIG_OTA
		BL_DBG("load app failed\n");
		ota_cfg cfg;
		if (cfg.image == OTA_IMAGE_1ST) {
			cfg.image = OTA_IMAGE_2ND;
			cfg.state = OTA_STATE_UNVERIFIED;
		} else if (cfg.image == OTA_IMAGE_2ND) {
			cfg.image = OTA_IMAGE_1ST;
			cfg.state = OTA_STATE_UNVERIFIED;
		}
		ota_write_cfg(&cfg);
		bl_reboot();
#else /* __PRJ_CONFIG_OTA */
		BL_ERR("load app failed\n");
#endif /* __PRJ_CONFIG_OTA */

out:
	image_close(hdl);
	return sh.entry;
}

int main(void)
{
	uint32_t boot_flag, entry;

	BL_DBG("start\n");

	boot_flag = HAL_PRCM_GetCPUABootFlag();
	if (boot_flag == PRCM_CPUA_BOOT_FROM_COLD_RESET) {
		bl_hw_init();
		bl_image_init();
		entry = bl_load_bin();
#ifdef __CONFIG_CHIP_XR871
		entry |= 0x1; /* set thumb bit */
#endif
		BL_DBG("goto %#x\n", entry);
		bl_image_deinit();
		bl_hw_deinit();

		__disable_fault_irq();
		__disable_irq();
		__set_CONTROL(0); /* reset to Privileged Thread mode and use MSP */
		__DSB();
		__ISB();
		((NVIC_IRQHandler)entry)(); /* never return */
		BL_ERR("unreachable\n");
	} else {
		BL_ERR("boot flag %#x\n", boot_flag);
	}

	return -1;
}
