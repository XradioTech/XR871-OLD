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
#include "driver/chip/system_chip.h"
#include "driver/chip/hal_chip.h"
#include "sys/image.h"

#include "common/board/board.h"
#include "bl_debug.h"

#define BL_INVALID_APP_ENTRY	0xFFFFFFFFU
#define BL_SYS2_SRAM_ADDR_MAX	0x60060000

/* return values for bl_load_bin_by_id() */
#define BL_LOAD_BIN_OK      (0)     /* success */
#define BL_LOAD_BIN_NO_SEC  (-1)    /* no section */
#define BL_LOAD_BIN_INVALID (-2)    /* invalid section */
#define BL_LOAD_BIN_TOOBIG  (-3)    /* section too big */

static __inline void bl_upgrade(void)
{
//	HAL_PRCM_SetCPUABootFlag(PRCM_CPUA_BOOT_FROM_COLD_RESET);
	HAL_PRCM_SetCPUABootFlag(PRCM_CPUA_BOOT_FROM_SYS_UPDATE);
	HAL_WDG_Reboot();
}

static __inline void bl_hw_init(void)
{
#ifdef __CONFIG_CHIP_XR32
	HAL_PRCM_DisableSys2();
	HAL_PRCM_DisableSys2Power();
	HAL_PRCM_EnableSys2Power();
#endif
	if (HAL_Flash_Init(PRJCONF_IMG_FLASH) != HAL_OK) {
		BL_ERR("flash init fail\n");
	}
#ifdef __CONFIG_CHIP_XR32
	HAL_PRCM_DisableSys2Isolation();
	HAL_PRCM_ReleaseSys2Reset();
#endif
}

static __inline void bl_hw_deinit(void)
{
	HAL_Flash_Deinit(PRJCONF_IMG_FLASH);
#if PRJCONF_UART_EN
#if BL_DBG_ON
	while (!HAL_UART_IsTxEmpty(HAL_UART_GetInstance(BOARD_MAIN_UART_ID))) { }
#endif
	board_uart_deinit(BOARD_MAIN_UART_ID);
#endif
	SystemDeInit(0);
}

static int bl_load_bin_by_id(uint32_t id, uint32_t addr_max, uint32_t *entry)
{
	uint32_t len;
	section_header_t sh;

	BL_DBG("%s(), id %#x\n", __func__, id);

	len = image_read(id, IMAGE_SEG_HEADER, 0, &sh, IMAGE_HEADER_SIZE);
	if (len != IMAGE_HEADER_SIZE) {
		BL_WRN("bin header size %u, read %u\n", IMAGE_HEADER_SIZE, len);
		return BL_LOAD_BIN_NO_SEC;
	}

	if (image_check_header(&sh) == IMAGE_INVALID) {
		BL_WRN("invalid bin header\n");
		return BL_LOAD_BIN_INVALID;
	}

	if (sh.body_len == 0) {
		BL_WRN("body_len %u\n", sh.body_len);
		return BL_LOAD_BIN_NO_SEC;
	}

	if (sh.load_addr + sh.body_len > addr_max) {
		BL_WRN("bin too big, %#x + %#x > %x\n", sh.load_addr, sh.body_len,
		       addr_max);
		return BL_LOAD_BIN_TOOBIG;
	}

	len = image_read(id, IMAGE_SEG_BODY, 0, (void *)sh.load_addr, sh.body_len);
	if (len != sh.body_len) {
		BL_WRN("bin body size %u, read %u\n", sh.body_len, len);
		return BL_LOAD_BIN_INVALID;
	}

	if (image_check_data(&sh, (void *)sh.load_addr, sh.body_len,
	                     NULL, 0) == IMAGE_INVALID) {
		BL_WRN("invalid bin body\n");
		return BL_LOAD_BIN_INVALID;
	}

	if (entry) {
		*entry = sh.entry;
	}
	return BL_LOAD_BIN_OK;
}

static uint32_t bl_load_app_bin(void)
{
	extern const unsigned char __RAM_BASE[]; /* SRAM start address of bl */
	uint32_t entry;
	int ret;

	ret = bl_load_bin_by_id(IMAGE_APP_ID, (uint32_t)__RAM_BASE, &entry);
	if (ret != BL_LOAD_BIN_OK) {
		return BL_INVALID_APP_ENTRY;
	}

#ifdef __CONFIG_CHIP_XR32
	HAL_PRCM_SetSys2SramClk(BOARD_CPU_CLK_SRC, BOARD_CPU_CLK_FACTOR);
	ret = bl_load_bin_by_id(IMAGE_APP_EXT_ID, BL_SYS2_SRAM_ADDR_MAX, NULL);
	if (ret == BL_LOAD_BIN_NO_SEC || ret == BL_LOAD_BIN_OK) {
		return entry;
	} else {
		return BL_INVALID_APP_ENTRY;
	}
#else
	return entry;
#endif

}

static uint32_t bl_load_bin(void)
{
	/* init image */
	if (image_init(PRJCONF_IMG_FLASH, PRJCONF_IMG_ADDR,
	               PRJCONF_IMG_MAX_SIZE) != 0) {
		BL_ERR("img init fail\n");
		return BL_INVALID_APP_ENTRY;
	}

	const image_ota_param_t *iop = image_get_ota_param();
	if (iop->ota_addr == IMAGE_INVALID_ADDR) {
		/* ota is disable */
		image_set_running_seq(0);
		return bl_load_app_bin();
	}

	/* ota is enabled */
	uint32_t entry;
	image_cfg_t cfg;
	image_seq_t cfg_seq, load_seq, i;

	if (image_get_cfg(&cfg) == 0) {
		BL_DBG("img seq %d, state %d\n", cfg.seq, cfg.state);
		if (cfg.state == IMAGE_STATE_VERIFIED) {
			cfg_seq = cfg.seq;
		} else {
			BL_WRN("invalid img state %d, seq %d\n", cfg.state, cfg.seq);
			cfg_seq = IMAGE_SEQ_NUM; /* set to invalid sequence */
		}
	} else {
		BL_WRN("ota read cfg fail\n");
		cfg_seq = IMAGE_SEQ_NUM; /* set to invalid sequence */
	}

	/* load app bin */
	load_seq = (cfg_seq == IMAGE_SEQ_NUM) ? 0 : cfg_seq;
	for (i = 0; i < IMAGE_SEQ_NUM; ++i) {
		image_set_running_seq(load_seq);
		entry = bl_load_app_bin();
		if (entry != BL_INVALID_APP_ENTRY) {
			if (load_seq != cfg_seq) {
				BL_WRN("boot from seq %u, cfg_seq %u\n", load_seq, cfg_seq);
				cfg.seq = load_seq;
				cfg.state = IMAGE_STATE_VERIFIED;
				if (image_set_cfg(&cfg) != 0) {
					BL_ERR("write img cfg fail\n");
				}
			}
			return entry;
		} else {
			BL_WRN("load app bin fail, seq %u\n", load_seq);
			load_seq = (load_seq + 1) % IMAGE_SEQ_NUM;
		}
	}

	return BL_INVALID_APP_ENTRY;
}

int main(void)
{
	uint32_t boot_flag;
	register uint32_t entry;

	BL_DBG("start\n");

	boot_flag = HAL_PRCM_GetCPUABootFlag();
	if (boot_flag == PRCM_CPUA_BOOT_FROM_COLD_RESET) {
		bl_hw_init();
		entry = bl_load_bin();
		if (entry == BL_INVALID_APP_ENTRY) {
			BL_ERR("load app bin fail, enter upgrade mode\n");
#ifdef __CONFIG_CHIP_XR32
			HAL_PRCM_DisableSys2();
			HAL_PRCM_DisableSys2Power();
#endif
			bl_upgrade();
		}
#ifdef __CONFIG_CPU_CM4F
		entry |= 0x1; /* set thumb bit */
#endif
		BL_DBG("goto %#x\n", entry);
		bl_hw_deinit();

		__disable_fault_irq();
		__disable_irq();
		__set_CONTROL(0); /* reset to Privileged Thread mode and use MSP */
		__DSB();
		__ISB();
		((NVIC_IRQHandler)entry)(); /* never return */
		BL_ERR("unreachable\n");
		BL_ABORT();
	} else {
		BL_ERR("boot flag %#x\n", boot_flag);
		BL_ABORT();
	}

	return -1;
}
