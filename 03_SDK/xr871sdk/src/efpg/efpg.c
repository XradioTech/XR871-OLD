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

#include "efpg_i.h"
#include "efpg_debug.h"
#include "efpg/efpg.h"
#include "console/console.h"
#include "driver/chip/hal_uart.h"

static efpg_priv_t g_efpg;
static OS_Thread_t g_efpg_thread;

static void efpg_rx_callback(void *arg)
{
	efpg_priv_t *efpg = &g_efpg;

	efpg->frame = (uint8_t *)efpg_malloc(efpg->expt_len);
	if (efpg->frame == NULL) {
		EFPG_ERR("efpg rx callback: malloc failed\n");
		return;
	}

	efpg->recv_len = HAL_UART_Receive_Poll(efpg->uart_id, efpg->frame, efpg->expt_len, EFPG_RECV_TIMEOUT_MS);

	OS_SemaphoreRelease(&efpg->sem);
}

static void efpg_task(void *arg)
{
	efpg_priv_t *efpg = &g_efpg;

	while (1) {
		if (OS_SemaphoreWait(&efpg->sem, OS_WAIT_FOREVER) != OS_OK)
			continue;

		if (efpg->recv_len != efpg->expt_len) {
			EFPG_WARN("efpg recv len %d, expt len %d\n", efpg->recv_len, efpg->expt_len);
			efpg_reset();
			continue;
		}

		if (efpg->is_cmd)
			efpg_cmd_frame_process(efpg);
		else
			efpg_data_frame_process(efpg);
	}
}

int efpg_init(uint8_t *key, uint8_t len)
{
	efpg_priv_t *efpg = &g_efpg;

	/* parameter */
	if ((key == NULL) || (len == 0)) {
		EFPG_ERR("efpg init failed: invalid parameter\n");
		return -1;
	}

	/* key */
	efpg->key = (uint8_t *)efpg_malloc(len);
	if (efpg->key == NULL) {
		EFPG_ERR("efpg init failed: malloc failed\n");
		return -1;
	}
	efpg_memcpy(efpg->key, key, len);
	efpg->key_len = len;

	/* semaphore */
	if (OS_SemaphoreCreateBinary(&efpg->sem) != OS_OK) {
		EFPG_ERR("efpg init failed: semaphore create failed\n");
		return -1;
	}

	return 0;
}

void efpg_deinit(void)
{
	efpg_priv_t *efpg = &g_efpg;

	/* key */
	if (efpg->key)
		efpg_free(efpg->key);
	efpg->key_len = 0;

	/* semaphore */
	if (OS_SemaphoreDelete(&efpg->sem) != OS_OK)
		EFPG_ERR("efpg deinit: semaphore delete failed\n");

	return;
}

int efpg_start(void)
{
	UART_T		   *uart;
	efpg_priv_t	   *efpg = &g_efpg;

	efpg->is_cmd	= 1;
	efpg->cmd_frame	= NULL;
	efpg->frame		= NULL;
	efpg->expt_len	= EFPG_CMD_FRAME_LEN;
	efpg->recv_len	= 0;
	efpg->op		= EFPG_OP_NUM;
	efpg->area		= EFPG_AREA_NUM;

	efpg->uart_id = console_get_uart_id();
	if (efpg->uart_id == UART_NUM) {
		EFPG_ERR("efpg start failed: get uart id failed\n");
		return -1;
	}

	if (OS_ThreadCreate(&g_efpg_thread,
		                "",
		                efpg_task,
		                NULL,
		                OS_THREAD_PRIO_CONSOLE,
		                EFPG_THREAD_STACK_SIZE) != OS_OK) {
		EFPG_ERR("create efpg task failed\n");
		return -1;
	}

	console_disable();

	uart = HAL_UART_GetInstance(efpg->uart_id);
	HAL_UART_EnableRxCallback(efpg->uart_id, efpg_rx_callback, uart);

	return 0;
}

void efpg_stop(void)
{
	efpg_priv_t *efpg = &g_efpg;

	HAL_UART_DisableRxCallback(efpg->uart_id);

	console_enable();

	if (efpg->cmd_frame)
		efpg_free(efpg->cmd_frame);

	if (efpg->frame)
		efpg_free(efpg->frame);

	OS_ThreadDelete(&g_efpg_thread);
}

void efpg_reset(void)
{
	efpg_priv_t *efpg = &g_efpg;

	if (efpg->cmd_frame) {
		efpg_free(efpg->cmd_frame);
		efpg->cmd_frame = NULL;
	}

	if (efpg->frame) {
		efpg_free(efpg->frame);
		efpg->frame = NULL;
	}

	efpg->is_cmd	= 1;
	efpg->expt_len	= EFPG_CMD_FRAME_LEN;
	efpg->recv_len	= 0;
	efpg->op		= EFPG_OP_NUM;
	efpg->area		= EFPG_AREA_NUM;
}

int efpg_read(efpg_block_t block, uint8_t *data)
{
	efpg_area_t	area;
	uint16_t	ack;

	if (data == NULL) {
		EFPG_ERR("efpg read failed: invalid parameter\n");
		return -1;
	}

	switch (block) {
	case EFPG_BLOCK_HOSC:
		area = EFPG_AREA_HOSC;
		break;
	case EFPG_BLOCK_BOOT:
		area = EFPG_AREA_BOOT;
		break;
	case EFPG_BLOCK_DCXO:
		area = EFPG_AREA_DCXO;
		break;
	case EFPG_BLOCK_POUT:
		area = EFPG_AREA_POUT;
		break;
	case EFPG_BLOCK_MAC:
		area = EFPG_AREA_MAC;
		break;
	default :
		EFPG_ERR("efpg read failed, invalid block %d\n", block);
		return -1;
	}

	ack = efpg_read_area(area, data);
	if (ack != EFPG_ACK_OK) {
		EFPG_ERR("efpg read failed, ack %d\n", ack);
		return -1;
	}

	return 0;
}

int efpg_read_hosc(efpg_hosc_t *hosc)
{
	uint8_t		data = 0;
	uint16_t	ack;

	if (hosc == NULL) {
		EFPG_ERR("efpg read hosc failed: invalid parameter\n");
		return -1;
	}

	ack = efpg_read_area(EFPG_AREA_HOSC, &data);
	if (ack != EFPG_ACK_OK) {
		EFPG_ERR("efpg read hosc failed, ack %d\n", ack);
		return -1;
	}

	switch (data) {
	case 0x03:
		*hosc = EFPG_HOSC_26M;
	case 0x0C:
		*hosc = EFPG_HOSC_40M;
	case 0x06:
		*hosc = EFPG_HOSC_24M;
	case 0x09:
		*hosc = EFPG_HOSC_52M;
	default :
		*hosc = EFPG_HOSC_INVALID;
	}

	return 0;
}
