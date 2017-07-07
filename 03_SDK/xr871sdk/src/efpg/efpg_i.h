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

#ifndef _EFPG_I_H_
#define _EFPG_I_H_

#include <stdlib.h>
#include <string.h>
#include "kernel/os/os.h"
#include "driver/chip/hal_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EFPG_WR_PROTECT_BIT_NUM	(1600)
#define EFPG_RECV_TIMEOUT_MS	(3000)
#define EFPG_THREAD_STACK_SIZE	(3 * 1024)

#define EFPG_CMD_FRAME_LEN		(8)
#define EFPG_ACK_FRAME_LEN		(4)
#define EFPG_MSG_DGST_LEN		(32)
#define EFPG_HOSC_FRAME_LEN		((1)  + EFPG_MSG_DGST_LEN)
#define EFPG_BOOT_FRAME_LEN		((32) + EFPG_MSG_DGST_LEN)
#define EFPG_DCXO_FRAME_LEN		((1)  + EFPG_MSG_DGST_LEN)
#define EFPG_POUT_FRAME_LEN		((3)  + EFPG_MSG_DGST_LEN)
#define EFPG_MAC_FRAME_LEN		((6)  + EFPG_MSG_DGST_LEN)

#define EFPG_ACK_OK				(200)
#define EFPG_ACK_CS_ERR			(400)
#define EFPG_ACK_MD_ERR			(401)
#define EFPG_ACK_PARSE_ERR		(402)
#define EFPG_ACK_RW_ERR			(403)
#define EFPG_ACK_DI_ERR			(404)
#define EFPG_ACK_NODATA_ERR		(405)

#define EFPG_OP_CODE_READ		(0x0FF0)
#define EFPG_OP_CODE_WRITE		(0xF00F)
#define EFPG_OP_CODE_EXIT		(0xFFFF)

#define EFPG_TYPE_HOSC			(0xFF00)
#define EFPG_TYPE_BOOT			(0xFE01)
#define EFPG_TYPE_DCXO			(0xFD02)
#define EFPG_TYPE_POUT			(0xFC03)
#define EFPG_TYPE_MAC			(0xFB04)

#define EFPG_HOSC_TYPE_START	(328)
#define EFPG_HOSC_TYPE_NUM		(4)
#define EFPG_BOOT_FLAG_START	(332)
#define EFPG_BOOT_FLAG_NUM		(2)
#define EFPG_BOOT_1ST_EN_START	(334)
#define EFPG_BOOT_1ST_EN_NUM	(1)
#define EFPG_BOOT_1ST_NO_START	(335)
#define EFPG_BOOT_1ST_NO_NUM	(8)
#define EFPG_BOOT_2ND_EN_START	(343)
#define EFPG_BOOT_2ND_EN_NUM	(1)
#define EFPG_BOOT_2ND_NO_START	(344)
#define EFPG_BOOT_2ND_NO_NUM	(8)
#define EFPG_BOOT_HASH_START	(352)
#define EFPG_BOOT_HASH_NUM		(256)
#define EFPG_DCXO_FLAG_START	(898)
#define EFPG_DCXO_FLAG_NUM		(6)
#define EFPG_DCXO_TRIM_START	(904)
#define EFPG_DCXO_TRIM_NUM		(8)
#define EFPG_POUT_FLAG_START	(928)
#define EFPG_POUT_FLAG_NUM		(6)
#define EFPG_POUT_CAL_START		(934)
#define EFPG_POUT_CAL_NUM		(21)
#define EFPG_MAC_FLAG_START		(997)
#define EFPG_MAC_FLAG_NUM		(18)
#define EFPG_MAC_ADDR_START		(1015)
#define EFPG_MAC_ADDR_NUM		(48)

#define EFPG_HOSC_BUF_LEN		(1)
#define EFPG_BOOT_BUF_LEN		(32)
#define EFPG_DCXO_BUF_LEN		(1)
#define EFPG_POUT_BUF_LEN		(3)
#define EFPG_MAC_BUF_LEN		(6)

#define EFPG_DCXO_IDX_MAX		(3)
#define EFPG_POUT_IDX_MAX		(3)
#define EFPG_MAC_IDX_MAX		(9)

#define efpg_malloc(l)			malloc(l)
#define efpg_free(p)			free(p)
#define efpg_memcpy(d, s, n)	memcpy(d, s, n)
#define efpg_memset(s, c, n)	memset(s, c, n)
#define efpg_memcmp(s1, s2, n)	memcmp(s1, s2, n)

typedef enum efpg_op {
	EFPG_OP_READ	= 0,
	EFPG_OP_WRITE	= 1,
	EFPG_OP_EXIT	= 2,
	EFPG_OP_NUM		= 3,
} efpg_op_t;

typedef enum efpg_area {
	EFPG_AREA_HOSC	= 0,
	EFPG_AREA_BOOT	= 1,
	EFPG_AREA_DCXO	= 2,
	EFPG_AREA_POUT	= 3,
	EFPG_AREA_MAC	= 4,
	EFPG_AREA_NUM	= 5,
} efpg_area_t;

typedef struct efpg_priv {
	uint8_t		   *key;
	uint8_t			key_len;

	uint8_t			is_cmd;
	uint8_t		   *cmd_frame;
	uint8_t		   *frame;
	uint16_t		expt_len;
	uint16_t		recv_len;

	efpg_op_t		op;
	efpg_area_t		area;

	UART_ID			uart_id;
	OS_Semaphore_t	sem;
} efpg_priv_t;

void efpg_stop(void);
void efpg_reset(void);

void efpg_cmd_frame_process(efpg_priv_t *efpg);
void efpg_data_frame_process(efpg_priv_t *efpg);

uint16_t efpg_read_area(efpg_area_t area, uint8_t *data);
uint16_t efpg_write_area(efpg_area_t area, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* _EFPG_I_H_ */