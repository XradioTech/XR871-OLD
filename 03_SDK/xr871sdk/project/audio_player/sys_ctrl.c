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

#include "types.h"
#include "kernel/os/os.h"
#include "console/console.h"
#include "sys/image.h"

#include "common/board/board.h"
#include "common/ctrl_msg/ctrl_msg.h"
#include "common/ctrl_img/ctrl_img.h"
#include "common/net_ctrl/net_ctrl.h"

#include "command.h"
#include "ctrl_debug.h"
#include "audio_player.h"


#define SYS_CTRL_MSG_QUEUE_LEN		16

#define SYS_CTRL_THREAD_STACK_SIZE	(2 * 1024)
static OS_Thread_t g_sys_ctrl_thread;

#if (defined(__CONFIG_CHIP_XR871))
#define IMAGE_BOOT_OFFSET		(0x00000000)
#endif
#define IMAGE_BOOT_CFG_OFFSET	(IMAGE_BOOT_OFFSET + (1 << 20))

#define IMAGE_OFFSET_1ST		IMAGE_BOOT_OFFSET
#define IMAGE_OFFSET_2ND		(IMAGE_BOOT_OFFSET + (1 << 20))

static void sys_ctrl_task(void *arg)
{
	int ret;
	struct ctrl_msg msg;

	while (1) {
		ret = ctrl_msg_recv(&msg, OS_WAIT_FOREVER);
		if (ret != 0) {
			continue;
		}

		CTRL_DBG("recv msg (%u, %u, %u)\n", ctrl_msg_get_type(&msg),
		         ctrl_msg_get_subtype(&msg), ctrl_msg_get_data(&msg));

		switch (ctrl_msg_get_type(&msg)) {
		case CTRL_MSG_TYPE_SYSTEM:
			break;
		case CTRL_MSG_TYPE_NETWORK:
#ifdef __CONFIG_ARCH_DUAL_CORE
			net_ctrl_msg_process(ctrl_msg_get_subtype(&msg),
			                     ctrl_msg_get_data(&msg));
#endif
			break;
		case CTRL_MSG_TYPE_VKEY:
			if (ctrl_msg_get_subtype(&msg) == CTRL_MSG_SUB_TYPE_AD_BUTTON)
				player_set_ad_button_cmd((AD_Button_Cmd_Info *)ctrl_msg_get_data(&msg));
			else
				player_set_gpio_button_cmd((GPIO_Button_Cmd_Info *)ctrl_msg_get_data(&msg));
			break;
		case CTRL_MSG_VOLUME:
			printf("%s(), volume value is %d\n", __func__, ctrl_msg_get_data(&msg));
			break;
		default:
			CTRL_WARN("unknown msg\n");
			break;
		}
	}

	CTRL_DBG("%s() end\n", __func__);
	OS_ThreadDelete(&g_sys_ctrl_thread);
}

void sys_ctrl_init(void)
{
	/* init control image */
	ctrl_img_init(IMAGE_BOOT_OFFSET, IMAGE_BOOT_CFG_OFFSET, IMAGE_OFFSET_1ST, IMAGE_OFFSET_2ND);

	/* init console */
	console_param_t cparam;
	cparam.uartID = BOARD_MAIN_UART_ID;
	cparam.cmd_exec = main_cmd_exec;
	console_start(&cparam);

	/* int control message */
	ctrl_msg_init(SYS_CTRL_MSG_QUEUE_LEN);

	/* start system control task */
	if (OS_ThreadCreate(&g_sys_ctrl_thread,
	                    "",
	                    sys_ctrl_task,
	                    NULL,
	                    OS_THREAD_PRIO_SYS_CTRL,
	                    SYS_CTRL_THREAD_STACK_SIZE) != OS_OK) {
		CTRL_ERR("create sys ctrl task failed\n");
	}
}
