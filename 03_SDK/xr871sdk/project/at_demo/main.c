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

#include <stdio.h>
#include <string.h>
#include "types.h"
#include "kernel/os/os.h"
#include "version.h"

#include "common/board/board.h"
#include "common/net_ctrl/net_ctrl.h"
#include "common/sysinfo/sysinfo.h"

#include "sys_ctrl.h"
#include "ctrl_debug.h"

#include "driver/chip/hal_clock.h"
#include "pm/pm.h"
#include "driver/chip/hal_wakeup.h"
#include "efpg/efpg.h"

static void show_info(void)
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
	       __end__, _estack - 1024,
	       _estack - __end__ - 1024);

	printf("cpu  clock %u Hz\n", HAL_GetCPUClock());
	printf("ahb1 clock %u Hz\n", HAL_GetAHB1Clock());
	printf("ahb2 clock %u Hz\n", HAL_GetAHB2Clock());
	printf("apb  clock %u Hz\n", HAL_GetAPBClock());
	printf("HF   clock %u Hz\n", HAL_GetHFClock());
	printf("LF   clock %u Hz\n", HAL_GetLFClock());
	printf("\n");
}

#define MAIN_THREAD_STACK_SIZE		(1 * 1024)
static OS_Thread_t g_main_thread;

static const char *g_efpg_key = "efpgtest";

static void main_task(void *arg)
{
	show_info();
	printf("XRadio IoT WLAN SDK %s\n\n", SDK_VERSION_STR);

	sysinfo_init();
	board_init();
	sys_ctrl_init();

	efpg_init((uint8_t *)g_efpg_key, strlen(g_efpg_key));

#ifdef __CONFIG_ARCH_DUAL_CORE
	net_sys_start(sysinfo_get_wlan_mode());
	pm_register_wlan_power_onoff(&net_sys_onoff, PM_SUPPORT_HIBERNATION | PM_SUPPORT_POWEROFF);
#endif
#ifdef __CONFIG_XIP_ENABLE
	board_xip_init();
	CTRL_DBG("XIP enable\n");
#endif

	OS_ThreadDelete(&g_main_thread);
}

int main(void)
{
	if (OS_ThreadCreate(&g_main_thread,
		                "",
		                main_task,
		                NULL,
		                OS_THREAD_PRIO_APP,
		                MAIN_THREAD_STACK_SIZE) != OS_OK) {
		CTRL_ERR("create main task failed\n");
	}

	OS_ThreadStartScheduler();

	while (1) {
		CTRL_ERR("error\n");
	}

	return 0;
}
