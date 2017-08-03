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

#include <stdio.h>
#include "compiler.h"
#include "version.h"

#include "common/board/board.h"
#include "sysinfo.h"
#include "img_ctrl.h"
#include "net_ctrl.h"
#include "sysinfo.h"
#include "sys_ctrl.h"
#include "fwk_debug.h"

#if (PRJCONF_SOUNDCARD0_EN || PRJCONF_SOUNDCARD1_EN)
#include "audio/manager/audio_manager.h"
#endif
#if PRJCONF_CONSOLE_EN
#include "console/console.h"
#include "command.h"
#endif
#ifdef __PRJ_CONFIG_XIP
#include "driver/chip/hal_flashctrl.h"
#include "sys/image.h"
#endif

#define PLATFORM_SHOW_INFO	1	/* for internal debug only */

#if PLATFORM_SHOW_INFO
static void platform_show_info(void)
{
	extern uint8_t	__data_end__[];
	extern uint8_t	_edata[];
	extern uint8_t	__bss_start__[];
	extern uint8_t	__bss_end__[];
	extern uint8_t	__end__[];
	extern uint8_t	end[];
	extern uint8_t	__HeapLimit[];
	extern uint8_t	__StackLimit[];
	extern uint8_t	__StackTop[];
	extern uint8_t	__stack[];
	extern uint8_t	_estack[];

	printf("__data_end__  %p\n", __data_end__);
	printf("_edata        %p\n", _edata);
	printf("__bss_start__ %p\n", __bss_start__);
	printf("__bss_end__   %p\n", __bss_end__);
	printf("__end__       %p\n", __end__);
	printf("end           %p\n", end);
	printf("__HeapLimit   %p\n", __HeapLimit);
	printf("__StackLimit  %p\n", __StackLimit);
	printf("__StackTop    %p\n", __StackTop);
	printf("__stack       %p\n", __stack);
	printf("_estack       %p\n", _estack);
	printf("\n");

	printf("heap space [%p, %p), size %u\n\n",
	       __end__, _estack - PRJCONF_MSP_STACK_SIZE,
	       _estack - __end__ - PRJCONF_MSP_STACK_SIZE);

	printf("cpu  clock %u Hz\n", HAL_GetCPUClock());
	printf("ahb1 clock %u Hz\n", HAL_GetAHB1Clock());
	printf("ahb2 clock %u Hz\n", HAL_GetAHB2Clock());
	printf("apb  clock %u Hz\n", HAL_GetAPBClock());
	printf("HF   clock %u Hz\n", HAL_GetHFClock());
	printf("LF   clock %u Hz\n", HAL_GetLFClock());
	printf("\n");
}
#endif /* PLATFORM_SHOW_INFO */

#ifdef __PRJ_CONFIG_XIP
#define PLATFORM_XIP_FREQ	(64 * 1000 * 1000)

static void platform_xip_init(void)
{
	XIP_Config xip;
	image_handle_t *hdl;
	uint32_t addr;

	hdl = image_open();
	if (hdl == NULL) {
		FWK_ERR("image open failed\n");
		return;
	}

	addr = image_get_section_addr(hdl, IMAGE_APP_XIP_ID);
	if (addr == IMAGE_INVALID_OFFSET) {
		FWK_ERR("no xip section\n");
		image_close(hdl);
		return;
	}

	/* TODO: check section's validity */

	image_close(hdl);

	if (addr != IMAGE_INVALID_OFFSET) {
		xip.addr = addr + IMAGE_HEADER_SIZE;
		xip.freq = PLATFORM_XIP_FREQ;
		FWK_DBG("xip enable, addr 0x%x, freq %u\n", xip.addr, xip.freq);
		HAL_XIP_Init(&xip);
	}
}
#endif /* __PRJ_CONFIG_XIP */

/* init system hardware independent of system service */
__weak void platform_hw_init_level0(void)
{
#if PRJCONF_UART_EN
	if (BOARD_SUB_UART_ID != BOARD_MAIN_UART_ID) {
		board_uart_init(BOARD_SUB_UART_ID);
	}
#endif
	board_spi_init(BOARD_FLASH_SPI_PORT);
#if PRJCONF_EFUSE_EN
	HAL_EFUSE_Init();
#endif
#if PRJCONF_CE_EN
	HAL_CE_Init();
#endif
}

/* init system hardware which is depends on system service */
__weak void platform_hw_init_level1(void)
{
#if PRJCONF_MMC_EN
	SDC_InitTypeDef sdc_param;
  #ifdef CONFIG_DETECT_CARD
	sdc_param.cd_mode = PRJCONF_MMC_DETECT_MODE;
  #endif
	HAL_SDC_Init(0, &sdc_param);
#endif

#if PRJCONF_SOUNDCARD0_EN
	board_soundcard0_init();
#endif

#if PRJCONF_SOUNDCARD1_EN
	board_soundcard1_init();
#endif
}

/* init basic system services independent of any hardware */
__weak void platform_service_init_level0(void)
{
	sys_ctrl_init();
#if (PRJCONF_SOUNDCARD0_EN || PRJCONF_SOUNDCARD1_EN)
	aud_mgr_init();
#endif
}

/* init system standard services */
__weak void platform_service_init_level1(void)
{
	sysinfo_init();
	img_ctrl_init(PRJCONF_IMG_BOOT_OFFSET, PRJCONF_IMG_BOOT_CFG_OFFSET,
	              PRJCONF_IMG_OFFSET_1ST, PRJCONF_IMG_OFFSET_2ND);

#if (defined(__PRJ_CONFIG_XIP) && PRJCONF_XIP_INIT_EARLIEST)
	platform_xip_init();
#endif

#if PRJCONF_CONSOLE_EN
	console_param_t cparam;
	cparam.uartID = BOARD_MAIN_UART_ID;
	cparam.cmd_exec = main_cmd_exec;
	console_start(&cparam);
#endif

#if PRJCONF_PM_MODE
	pm_mode_platform_select(PRJCONF_PM_MODE);
#endif

#if PRJCONF_NET_EN
	net_sys_start(sysinfo_get_wlan_mode());
  #if PRJCONF_NET_ONOFF_PM_MODE
	pm_register_wlan_power_onoff(net_sys_onoff, PRJCONF_NET_ONOFF_PM_MODE);
  #endif
#endif

#if (defined(__PRJ_CONFIG_XIP) && !PRJCONF_XIP_INIT_EARLIEST)
	platform_xip_init();
#endif
}

void platform_init(void)
{
	printf("XRadio IoT WLAN SDK %s\n\n", SDK_VERSION_STR);
#if PLATFORM_SHOW_INFO
	platform_show_info();
#endif

	platform_hw_init_level0();
	platform_service_init_level0();
	platform_hw_init_level1();
	platform_service_init_level1();
}
