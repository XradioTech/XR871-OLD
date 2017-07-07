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

#include "kernel/os/os_queue.h"
#include "ctrl_msg.h"
#include "ctrl_msg_debug.h"

OS_Queue_t g_ctrl_msg_queue;

int ctrl_msg_init(uint32_t queue_len)
{
#if CTRL_MSG_VALIDITY_CHECK
	if (OS_QueueIsValid(&g_ctrl_msg_queue)) {
		CTRL_MSG_WARN("control message queue already inited\n");
		return -1;
	}
#endif

	if (OS_QueueCreate(&g_ctrl_msg_queue, queue_len, sizeof(struct ctrl_msg)) != OS_OK) {
		CTRL_MSG_ERR("%s() failed!\n", __func__);
		return -1;
	}
	CTRL_MSG_DBG("%s()\n", __func__);
	return 0;
}

int ctrl_msg_deinit(void)
{
	OS_Status ret;

#if CTRL_MSG_VALIDITY_CHECK
	if (!OS_QueueIsValid(&g_ctrl_msg_queue)) {
		CTRL_MSG_WARN("%s(), invalid queue %p\n", __func__, g_ctrl_msg_queue.handle);
		return 0;
	}
#endif

	ret = OS_QueueDelete(&g_ctrl_msg_queue);
	if (ret != OS_OK) {
		CTRL_MSG_ERR("%s() failed, err 0x%x\n", __func__, ret);
		return -1;
	}
	CTRL_MSG_DBG("%s()\n", __func__);
	return 0;
}

int ctrl_msg_send(struct ctrl_msg *msg, uint32_t wait_ms)
{
	OS_Status ret;

#if CTRL_MSG_VALIDITY_CHECK
	if (!OS_QueueIsValid(&g_ctrl_msg_queue)) {
		CTRL_MSG_WARN("%s(), invalid queue %p\n", __func__, g_ctrl_msg_queue.handle);
		return -1;
	}
#endif

	ret = OS_QueueSend(&g_ctrl_msg_queue, msg, wait_ms);
	if (ret != OS_OK) {
		CTRL_MSG_ERR("%s() failed, err 0x%x\n", __func__, ret);
		return -1;
	}
	return 0;
}

int ctrl_msg_recv(struct ctrl_msg *msg, uint32_t wait_ms)
{
	OS_Status ret;

#if CTRL_MSG_VALIDITY_CHECK
	if (!OS_QueueIsValid(&g_ctrl_msg_queue)) {
		CTRL_MSG_WARN("%s(), invalid queue %p\n", __func__, g_ctrl_msg_queue.handle);
		return -1;
	}
#endif

	ret = OS_QueueReceive(&g_ctrl_msg_queue, msg, wait_ms);
	if (ret != OS_OK) {
		CTRL_MSG_ERR("%s() failed, err 0x%x\n", __func__, ret);
		return -1;
	}
	return 0;
}
