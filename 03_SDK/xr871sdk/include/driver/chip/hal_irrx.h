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

#ifndef _DRIVER_CHIP_HAL_IRRX_H
#define _DRIVER_CHIP_HAL_IRRX_H

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* NOTE: only support NEC protocal now, rewrite IRRX_XXXPacket_Handler if you want
 *        support other protocals.
 */

/* NOTE: IR_CLK_32K_USED can be used only external 32768 crystal enabled */
//#define IR_CLK_32K_USED         1       /* keep the same define in hal_irtx.h */

/* IR Key Match Config, only addr code is IRRX_ADDR_CODE will be reported. */
//#define IRRX_CHECK_ADDR_CODE    1
#define IRRX_ADDR_CODE          (0x7f80)        /* (addr|~addr) */

/**
 * irrx use example:
 * 1. define IR_CLK_32K_USED as ir clk source if 32k can be used when suspend.
 * 2. define IRRX_CHECK_ADDR_CODE if you want to support only one ir remoter which addr is defined by IRRX_ADDR_CODE.
 *
#include "driver/chip/hal_irrx.h"

static GPIO_PinMuxParam g_pinmux_irrx[] = {
//        port         pin            mode                      driving               pull
	{ GPIO_PORT_A, GPIO_PIN_14, { GPIOA_P14_F5_IR_RX,       GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

extern HAL_Status board_irrx_cfg(HAL_BoardReq req, void *arg);

static void irrx_rxcplt_callback(uint32_t addr, uint32_t key)
{
	printf("received ir code addr:0x%02x key:0x%02x\n", addr, key);
}

void main(const void *arg)
{
	IRRX_InitTypeDef irrx_param;

	irrx_param.PulsePolariyInvert = IRRX_RPPI_INVERT;
	irrx_param.boardCfg = &board_irrx_cfg;
	irrx_param.rxCpltCallback = &irrx_rxcplt_callback;
	HAL_IRRX_Init(&irrx_param);

	while (1) {
		OS_Sleep(10);
	}
}
 *
 */

/* IRRX configure register definition, receiver pulse polarity. */
typedef enum
{
	IRRX_RPPI_NONE          = 0,
	IRRX_RPPI_INVERT        = HAL_BIT(2)
} IRRX_RppiTypeDef;

typedef void (*IRRX_RxCpltCallback)(uint32_t addr, uint32_t key);

/* IRRX Init Structure definition */
typedef struct
{
	IRRX_RppiTypeDef        PulsePolariyInvert;
	HAL_BoardCfg            boardCfg;
	IRRX_RxCpltCallback     rxCpltCallback;
} IRRX_InitTypeDef;

/**
 * Initializes the IRRX mode according to the specified parameters in the param.
 *
 * @param: Pointer to the configuration information for the specified IRRX module.
 */
extern void HAL_IRRX_Init(IRRX_InitTypeDef *param);

/**
 * DeInitializes the IRRX peripheral.
 */
extern void HAL_IRRX_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_IRRX_H */
