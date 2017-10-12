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
#include "driver/chip/hal_uart.h"

static efpg_priv_t g_efpg;
static OS_Thread_t g_efpg_thread;

static void efpg_task(void *arg)
{
	uint8_t *buf;
	int32_t recv_len;
	efpg_priv_t *efpg = &g_efpg;
	efpg_state_t state = EFPG_STATE_NUM;

	if (efpg->start_cb)
		efpg->start_cb();

	efpg->cmd_frame = efpg_malloc(EFPG_CMD_FRAME_LEN);
	efpg->frame = efpg_malloc(EFPG_DATA_FRAME_LEN_MAX);
	if ((efpg->cmd_frame == NULL) || (efpg->frame == NULL)) {
		EFPG_ERR("cmd frame %p, frame %p\n", efpg->cmd_frame, efpg->frame);
		goto efpg_stop;
	}

efpg_reset:
	efpg->is_cmd	= 1;
	efpg->expt_len	= EFPG_CMD_FRAME_LEN;
	efpg->recv_len	= 0;
	efpg->op		= EFPG_OP_NUM;
	efpg->area		= EFPG_AREA_NUM;

	while (1) {
efpg_continue:
		if (efpg->is_cmd)
			buf = efpg->cmd_frame;
		else
			buf = efpg->frame;
		recv_len = 0;
		while (recv_len == 0) {
			recv_len = HAL_UART_Receive_IT(efpg->uart_id, buf, efpg->expt_len, EFPG_RECV_TIMEOUT_MS);
		}

		if (recv_len == -1) {
			EFPG_ERR("UART receive failed\n");
			goto efpg_stop;
		}

		if ((uint16_t)recv_len != efpg->expt_len) {
			EFPG_WARN("%s(), %d, recv len %d, expt len %d\n",
					  __func__, __LINE__, recv_len, efpg->expt_len);
			goto efpg_reset;
		}

		efpg->recv_len = (uint16_t)recv_len;

		if (efpg->is_cmd)
			state = efpg_cmd_frame_process(efpg);
		else
			state = efpg_data_frame_process(efpg);

		switch (state) {
		case EFPG_STATE_CONTINUE:
			goto efpg_continue;
		case EFPG_STATE_RESET:
			goto efpg_reset;
		case EFPG_STATE_STOP:
			goto efpg_stop;
		default:
			EFPG_ERR("invalid state %d\n", state);
			goto efpg_stop;
		}
	}

efpg_stop:
	if (efpg->stop_cb)
		efpg->stop_cb();

	if (efpg->cmd_frame)
		efpg_free(efpg->cmd_frame);

	if (efpg->frame)
		efpg_free(efpg->frame);

	OS_ThreadDelete(&g_efpg_thread);
}

int efpg_init(uint8_t *key, uint8_t len)
{
	efpg_priv_t *efpg = &g_efpg;

	if ((key == NULL) || (len == 0)) {
		EFPG_ERR("key %p, len %d\n", key, len);
		return -1;
	}

	efpg->key = efpg_malloc(len);
	if (efpg->key == NULL) {
		EFPG_ERR("malloc failed\n");
		return -1;
	}
	efpg_memcpy(efpg->key, key, len);
	efpg->key_len = len;

	return 0;
}

void efpg_deinit(void)
{
	efpg_priv_t *efpg = &g_efpg;

	if (efpg->key)
		efpg_free(efpg->key);
	efpg->key_len = 0;
}

int efpg_start(UART_ID uart_id, efpg_cb_t start_cb, efpg_cb_t stop_cb)
{
	efpg_priv_t *efpg = &g_efpg;

	if (uart_id == UART_NUM) {
		EFPG_ERR("uart_id %d\n", uart_id);
		return -1;
	}

	efpg->uart_id	= uart_id;
	efpg->start_cb	= start_cb;
	efpg->stop_cb	= stop_cb;

	if (OS_ThreadCreate(&g_efpg_thread,
		                "",
		                efpg_task,
		                NULL,
		                OS_THREAD_PRIO_CONSOLE,
		                EFPG_THREAD_STACK_SIZE) != OS_OK) {
		EFPG_ERR("create efpg task failed\n");
		return -1;
	}

	return 0;
}

int efpg_read(efpg_area_t area, uint8_t *data)
{
	if (data == NULL) {
		EFPG_ERR("data %p\n", data);
		return -1;
	}

	uint16_t ack = efpg_read_area(area, data);
	if (ack != EFPG_ACK_OK) {
		EFPG_WARN("%s(), %d, ack %d\n", __func__, __LINE__, ack);
		return -1;
	}

	return 0;
}

int efpg_read_hosc(efpg_hosc_t *hosc)
{
	uint8_t		data = 0;
	uint16_t	ack;

	if (hosc == NULL) {
		EFPG_ERR("hosc %p\n", hosc);
		return -1;
	}

	ack = efpg_read_area(EFPG_AREA_HOSC, &data);
	if (ack != EFPG_ACK_OK) {
		EFPG_WARN("%s(), %d, ack %d\n", __func__, __LINE__, ack);
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
