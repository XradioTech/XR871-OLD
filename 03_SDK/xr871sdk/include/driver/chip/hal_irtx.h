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

#ifndef _DRIVER_CHIP_HAL_IRTX_H
#define _DRIVER_CHIP_HAL_IRTX_H

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* NOTE: IR_CLK_32K_USED can be used only external 32768 crystal enabled */
//#define IR_CLK_32K_USED         1       /* keep the same define in hal_irrx.h */

#define IRTX_MULTIPROTOS_SUPPORT 1              /* support multiply protocals */

/*
 *
 * irrx use example:
 * 1 define IR_CLK_32K_USED as ir clk source if 32k can be used when suspend.
 *

#include "driver/chip/hal_irtx.h"
#include "driver/chip/ir_nec.h"

extern HAL_Status board_irtx_cfg(uint32_t id, HAL_BoardReq req, void *arg);

void main(const void *arg)
{
	IRTX_InitTypeDef irtx_param;

	irtx_param.PulsePolarity = IRTX_TPPI_NONE;
	irtx_param.ModulateDutyLevel = IRTX_DRMC_TIME_1;
	irtx_param.SendModeType = IRTX_TTS_NONE; //IRTX_TTS_CYCLICAL for cycle mode
	irtx_param.CyclicalCnt = 3; //send 3 times if cycle mode
#if defined (IR_CLK_32K_USED)
	irtx_param.IdleDurationCnt = IRTX_32K_NEC_IDC_VALUE;
#else
	uint32_t clk = HAL_GetHFClock();

	if (clk == HOSC_CLOCK_26M) {
		irtx_param.IdleDurationCnt = IRTX_26M_NEC_IDC_VALUE;
	} else if (clk == HOSC_CLOCK_24M) {
		irtx_param.IdleDurationCnt = IRTX_24M_NEC_IDC_VALUE;
	} else {
		printf("%s unknow clk type(%d)!\n", __func__, clk);
	}
#endif

	irtx_param.InternalModulation = IRTX_IMS_ENABLE;
	irtx_param.boardCfg = &board_irtx_cfg;
	HAL_IRTX_Init(&irtx_param);

	while (1) {
		OS_Sleep(5);
		// add=0x59, key=0x16, add|~addr|key|~key
		HAL_IRTX_Transmit(IRTX_NEC_PROTO, 0x59A616E9);
	}
}
 *
 */

/* NOTE: add a new protocal in IRTX_ProtocalsDef and IRTX_PROTOS_FUN_INIT */
typedef enum
{
	IRTX_NEC_PROTO		= 0,
	//IRTX_ITT_PROTO	  = 1,
	IRTX_PROTO_NUM			/* keep last */
} IRTX_ProtocalsDef;

#ifdef IRTX_MULTIPROTOS_SUPPORT
typedef uint32_t (*IRTX_Proto_Code)(uint8_t *txBuff, uint32_t ir_tx_code);
typedef uint32_t (*IRTX_Proto_RepeatCode)(uint8_t *txBuff);

#define IRTX_PROTOS_FUN_INIT(handler) \
	do { \
		(handler)->Protos[IRTX_NEC_PROTO] = IRTX_NECPacket_Code; \
		/*(handler)->Protos[IRTX_ITT_PROTO] = IRTX_ITTPacket_Code;*/ \
	} while (0)
#else
#define IRTX_PROTOS_FUN_INIT(handler) \
	do { } while (0)
#endif

/* IRTX Transmit Global register definition, bit5~6,
 *  the high level/low level of modulated carrier duty ratio.
 * DRMC_TIME_1 can trasmit longer, DRMC_TIME_3 has less power.
 */
typedef enum
{
	IRTX_DRMC_TIME_1        = (0),          /* 1:1 */
	IRTX_DRMC_TIME_2        = (1<<5),       /* 1:2 */
	IRTX_DRMC_TIME_3        = (2<<5),       /* 1:3 */
	IRTX_DRMC_TIME_MASK     = (3<<5)
} IRTX_DRMC_TIME;

/* IRTX Transmit Global register definition, bit2, transmit pulse polarity. */
typedef enum
{
	IRTX_TPPI_NONE          = (0),
	IRTX_TPPI_INVERT        = (1<<2),
	IRTX_TPPI_MASK          = (1<<2)
} IRTX_TPPI_Type;

/* IRTX internal modulation signal enable, select disable when connect rx directly for test */
typedef enum
{
	IRTX_IMS_DISABLE        = (0),
	IRTX_IMS_ENABLE         = (1<<7)
} IRTX_IMS_Type;

/* IRTX Transmit Control register definition, bit0, transmit type: signal or
 * cyclical mode. NOTE: the repeat code start with S0(4.5mS not 2.25mS) for hardware not support change S0. */
typedef enum
{
	IRTX_TTS_NONE           = (0),
	IRTX_TTS_CYCLICAL       = (1),
	IRTX_TTS_MASK           = (1)
} IRTX_TTS_Type;

/* IRTX Init Structure definition */
typedef struct
{
	IRTX_DRMC_TIME          ModulateDutyLevel;      /* 1/3(less power cosumption) ~ 1/1(more tx lenght) */
	IRTX_TTS_Type           SendModeType;           /* signal or cyclical mode */
	uint32_t                CyclicalCnt;            /* count of cyclical mode */
	IRTX_TPPI_Type          PulsePolarity;          /* pulse polarity */
	IRTX_IMS_Type           InternalModulation;     /* internal modulation signal enable */
	uint32_t                IdleDurationCnt;        /* idle time for cyclical */
	HAL_BoardCfg            boardCfg;
} IRTX_InitTypeDef;

extern void HAL_IRTX_Transmit(uint32_t protos_sel, uint32_t ir_tx_code);
extern void HAL_IRTX_Init(IRTX_InitTypeDef *param);
extern void HAL_IRTX_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_IRTX_H */
