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
#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_crypto.h"

static uint8_t efpg_checksum8(uint8_t *data, uint32_t len)
{
	uint8_t cs = 0;

	while (len > 0) {
		cs += *data++;
		len--;
	}

	return cs;
}

static int efpg_check_msg_dgst(efpg_priv_t *efpg)
{
	CE_SHA256_Handler hdl;
	uint8_t msg_dgst_cal[EFPG_MSG_DGST_LEN];
	uint8_t *msg_dgst = efpg->frame + efpg->recv_len - EFPG_MSG_DGST_LEN;

	if ((HAL_SHA256_Init(&hdl, CE_CTL_IVMODE_SHA_MD5_FIPS180, NULL) != HAL_OK)
		|| (HAL_SHA256_Append(&hdl, efpg->cmd_frame, EFPG_CMD_FRAME_LEN) != HAL_OK)
		|| (HAL_SHA256_Append(&hdl, efpg->frame, efpg->recv_len - EFPG_MSG_DGST_LEN) != HAL_OK)
		|| (HAL_SHA256_Append(&hdl, efpg->key, efpg->key_len) != HAL_OK)
		|| (HAL_SHA256_Finish(&hdl, (uint32_t *)msg_dgst_cal) != HAL_OK)) {
		EFPG_ERR("failed to calculate msg dgst\n");
		return -1;
	}

	if (efpg_memcmp(msg_dgst, msg_dgst_cal, EFPG_MSG_DGST_LEN)) {
		EFPG_WARN("efpg check msg dgst failed\n");
		return -1;
	}

	return 0;
}

static int efpg_send_ack(efpg_priv_t *efpg, uint16_t status)
{
	uint8_t cs;
	int32_t send_len;

	uint8_t *ack_frame = (uint8_t *)efpg_malloc(EFPG_ACK_FRAME_LEN);
	if (ack_frame == NULL) {
		EFPG_ERR("efpg send ack: malloc failed\n");
		return -1;
	}

	efpg_memset(ack_frame, 0, EFPG_ACK_FRAME_LEN);
	efpg_memcpy(ack_frame, &status, 2);
	cs = 0xFF - efpg_checksum8(ack_frame, EFPG_ACK_FRAME_LEN);
	efpg_memcpy(ack_frame + 3, &cs, sizeof(cs));

	send_len = HAL_UART_Transmit_Poll(efpg->uart_id, ack_frame, EFPG_ACK_FRAME_LEN);
	efpg_free(ack_frame);

	if (send_len != EFPG_ACK_FRAME_LEN) {
		EFPG_WARN("efpg send ack failed: send len %d\n", send_len);
		return -1;
	} else {
		return 0;
	}
}

static int efpg_parse_cmd(efpg_priv_t *efpg)
{
	uint16_t	op_code;
	uint16_t	type;
	uint16_t	len;
	uint8_t	   *p = efpg->frame;

	op_code = *((uint16_t *)p);
	p += 2;
	type = *((uint16_t *)p);
	p += 2;
	len = *((uint16_t *)p);

	switch (op_code) {
	case EFPG_OP_CODE_READ:
		efpg->op = EFPG_OP_READ;
		break;
	case EFPG_OP_CODE_WRITE:
		efpg->op = EFPG_OP_WRITE;
		break;
	case EFPG_OP_CODE_EXIT:
		efpg->op = EFPG_OP_EXIT;
		return 0;
	default:
		EFPG_WARN("efpg parse cmd failed: op_code %#06x\n", op_code);
		return -1;
	}

	switch (type) {
	case EFPG_TYPE_HOSC:
		efpg->area = EFPG_AREA_HOSC;
		efpg->expt_len = EFPG_HOSC_FRAME_LEN;
		break;
	case EFPG_TYPE_BOOT:
		efpg->area = EFPG_AREA_BOOT;
		efpg->expt_len = EFPG_BOOT_FRAME_LEN;
		break;
	case EFPG_TYPE_DCXO:
		efpg->area = EFPG_AREA_DCXO;
		efpg->expt_len = EFPG_DCXO_FRAME_LEN;
		break;
	case EFPG_TYPE_POUT:
		efpg->area = EFPG_AREA_POUT;
		efpg->expt_len = EFPG_POUT_FRAME_LEN;
		break;
	case EFPG_TYPE_MAC:
		efpg->area = EFPG_AREA_MAC;
		efpg->expt_len = EFPG_MAC_FRAME_LEN;
		break;
	default:
		EFPG_WARN("efpg parse cmd failed: type %#06x\n", type);
		return -1;
	}

	if (len != efpg->expt_len) {
		EFPG_WARN("efpg parse cmd failed: len %d, expt %d\n", len, efpg->expt_len);
		return -1;
	}

	return 0;
}

static void efpg_read_process(efpg_priv_t *efpg)
{
	uint16_t status;
	uint8_t *data;
	uint8_t *msg_dgst;
	int32_t send_len;
	CE_SHA256_Handler hdl;

	uint8_t *frame = (uint8_t *)efpg_malloc(efpg->expt_len);
	if (frame == NULL) {
		EFPG_ERR("efpg read process failed: malloc failed\n");
		efpg_reset();
		return;
	}

	efpg_memset(frame, 0, efpg->expt_len);
	data = frame;
	msg_dgst = frame + efpg->expt_len - EFPG_MSG_DGST_LEN;

	status = efpg_read_area(efpg->area, data);
	if ((HAL_SHA256_Init(&hdl, CE_CTL_IVMODE_SHA_MD5_FIPS180, NULL) != HAL_OK)
		|| (HAL_SHA256_Append(&hdl, efpg->frame, EFPG_CMD_FRAME_LEN) != HAL_OK)
		|| (HAL_SHA256_Append(&hdl, data, efpg->expt_len - EFPG_MSG_DGST_LEN) != HAL_OK)
		|| (HAL_SHA256_Append(&hdl, efpg->key, efpg->key_len) != HAL_OK)
		|| (HAL_SHA256_Finish(&hdl, (uint32_t *)msg_dgst) != HAL_OK)) {
		EFPG_WARN("efpg read process: SHA256 failed\n");
		status = EFPG_ACK_RW_ERR;
	}

	if (status != EFPG_ACK_OK) {
		EFPG_WARN("efpg read process: status %d\n", status);
		efpg_free(frame);
		efpg_reset();
		efpg_send_ack(efpg, status);
		return;
	}

	if (efpg_send_ack(efpg, EFPG_ACK_OK) < 0) {
		EFPG_WARN("efpg read send ack failed\n");
		efpg_free(frame);
		efpg_reset();
		return;
	}

	send_len = HAL_UART_Transmit_Poll(efpg->uart_id, frame, efpg->expt_len);
	if (send_len != efpg->expt_len)
		EFPG_WARN("efpg read process: send len %d, expt %d\n", send_len, efpg->expt_len);

	efpg_free(frame);
	efpg_reset();
}

static void efpg_write_process(efpg_priv_t *efpg)
{
	efpg->is_cmd = 0;
	efpg->cmd_frame = efpg->frame;
	efpg->frame = NULL;
	efpg->recv_len = 0;

	if (efpg_send_ack(efpg, EFPG_ACK_OK) < 0) {
		EFPG_WARN("efpg write send ack failed\n");
		efpg_reset();
	}
}

static void efpg_stop_process(efpg_priv_t *efpg)
{
	if (efpg_send_ack(efpg, EFPG_ACK_OK) < 0) {
		EFPG_WARN("efpg exit send ack failed\n");
		efpg_reset();
	} else {
		efpg_stop();
	}
}

void efpg_cmd_frame_process(efpg_priv_t *efpg)
{
	/* checksum */
	if (efpg_checksum8(efpg->frame, efpg->recv_len) != 0xFF) {
		EFPG_WARN("efpg cmd frame process: efpg checksum failed\n");
		efpg_reset();
		efpg_send_ack(efpg, EFPG_ACK_CS_ERR);
		return;
	}

	/* parse frame */
	if (efpg_parse_cmd(efpg) < 0) {
		EFPG_WARN("efpg cmd frame process: efpg parse failed\n");
		efpg_reset();
		efpg_send_ack(efpg, EFPG_ACK_PARSE_ERR);
		return;
	}

	if (efpg->op == EFPG_OP_READ) {
		efpg_read_process(efpg);
	} else if (efpg->op == EFPG_OP_WRITE) {
		efpg_write_process(efpg);
	} else if (efpg->op == EFPG_OP_EXIT) {
		efpg_stop_process(efpg);
	}
}

void efpg_data_frame_process(efpg_priv_t *efpg)
{
	uint16_t status;
	uint8_t *data;

	/* message digest */
	if (efpg_check_msg_dgst(efpg) < 0) {
		EFPG_WARN("efpg data frame process: efpg check msg dgst failed\n");
		efpg_reset();
		efpg_send_ack(efpg, EFPG_ACK_MD_ERR);
		return;
	}

	/* write data */
	data = efpg->frame;
	status = efpg_write_area(efpg->area, data);
	EFPG_DBG("efpg data frame process: write status %d\n", status);

	efpg_reset();
	efpg_send_ack(efpg, status);
}

