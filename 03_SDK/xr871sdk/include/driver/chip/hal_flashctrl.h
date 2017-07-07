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

#ifndef __HAL_FLASHCTRL_H_
#define __HAL_FLASHCTRL_H_

#include <stdbool.h>
#include <stdlib.h>
#include "driver/chip/hal_def.h"
#include "driver/chip/device.h"
#include "sys/xr_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	__IO uint32_t COMMON_CFG;   			/* ,                                       Address offset:        */
	__IO uint32_t CMD_CFG;
	__IO uint32_t DUMMY_H;  /* ,                           Address offset:        */
	__IO uint32_t DUMMY_L;  	/* ,                      Address offset:        */
	__IO uint32_t CS_WAIT;   			/* ,                  Address offset:        */
	__IO uint32_t CMD_WAIT;   			/* ,                  Address offset:        */
	__IO uint32_t RESERVE[2];   			/* ,                  Address offset:        */
	__IO uint32_t FLASH_COMMON_CFG; /* ,                  Address offset:        */
	__I  uint32_t XIP_EXEC;   			/* ,                  Address offset:        */

} FLASH_CTRL_T;



#define FLASH_CTRL ((FLASH_CTRL_T *)FLASH_CTRL_BASE)

/*
 * @brief
 */
#define FLASH_CTRL_COMMON_CFG_CONT_EN_SHIFT				(20)
#define FLASH_CTRL_COMMON_CFG_CONT_EN_MASK					(0x1U << FLASH_CTRL_COMMON_CFG_CONT_EN_SHIFT)

#define FLASH_CTRL_COMMON_CFG_IO1_SHIFT				(16)
#define FLASH_CTRL_COMMON_CFG_IO1_MASK					(0x3U << FLASH_CTRL_COMMON_CFG_IO1_SHIFT)

#define FLASH_CTRL_COMMON_CFG_IO2_SHIFT				(12)
#define FLASH_CTRL_COMMON_CFG_IO2_MASK					(0x3U << FLASH_CTRL_COMMON_CFG_IO2_SHIFT)

#define FLASH_CTRL_COMMON_CFG_IO3_SHIFT				(8)
#define FLASH_CTRL_COMMON_CFG_IO3_MASK					(0x3U << FLASH_CTRL_COMMON_CFG_IO3_SHIFT)

typedef enum {
	FLASH_CTRL_IO_OUTPUT_0,
	FLASH_CTRL_IO_OUTPUT_1,
	FLASH_CTRL_IO_OUTPUT_Z
} Flash_Ctrl_Io_Output;

#define FLASH_CTRL_COMMON_CFG_PREFETCH_EN_SHIFT				(4)
#define FLASH_CTRL_COMMON_CFG_PREFETCH_EN_MASK					(0x1U << FLASH_CTRL_COMMON_CFG_PREFETCH_EN_SHIFT)

#define FLASH_CTRL_COMMON_CFG_IBUS_EN_SHIFT				(0)
#define FLASH_CTRL_COMMON_CFG_IBUS_EN_MASK					(0x1U << FLASH_CTRL_COMMON_CFG_IBUS_EN_SHIFT)

#define FLASH_CTRL_CMD_CFG_CMD_SHIFT				(24)
#define FLASH_CTRL_CMD_CFG_CMD_MASK					(0xFFU << FLASH_CTRL_CMD_CFG_CMD_SHIFT)

#define FLASH_CTRL_CMD_CFG_CMD_BIT_SHIFT				(20)
#define FLASH_CTRL_CMD_CFG_CMD_BIT_MASK					(0x3U << FLASH_CTRL_CMD_CFG_CMD_BIT_SHIFT)

#define FLASH_CTRL_CMD_CFG_ADDR_BIT_SHIFT				(16)
#define FLASH_CTRL_CMD_CFG_ADDR_BIT_MASK					(0x3U << FLASH_CTRL_CMD_CFG_ADDR_BIT_SHIFT)

#define FLASH_CTRL_CMD_CFG_DUM_BIT_SHIFT				(12)
#define FLASH_CTRL_CMD_CFG_DUM_BIT_MASK					(0x3U << FLASH_CTRL_CMD_CFG_DUM_BIT_SHIFT)

#define FLASH_CTRL_CMD_CFG_DUM_WIDTH_SHIFT				(4)
#define FLASH_CTRL_CMD_CFG_DUM_WIDTH_MASK					(0x7FU << FLASH_CTRL_CMD_CFG_DUM_WIDTH_SHIFT)

#define FLASH_CTRL_CMD_CFG_DATA_BIT_SHIFT				(0)
#define FLASH_CTRL_CMD_CFG_DATA_BIT_MASK					(0x3U << FLASH_CTRL_CMD_CFG_DATA_BIT_SHIFT)

typedef enum {
	FLASH_CTRL_CYCLEBITS_0,
	FLASH_CTRL_CYCLEBITS_1,
	FLASH_CTRL_CYCLEBITS_2,
	FLASH_CTRL_CYCLEBITS_4
} Flash_Ctrl_CycleBits;

#define FLASH_CTRL_DUMMY_H_SHIFT				(0)
#define FLASH_CTRL_DUMMY_H_MASK					(0xFFFFFFFFU << FLASH_CTRL_DUMMY_H_SHIFT)

#define FLASH_CTRL_DUMMY_L_SHIFT				(0)
#define FLASH_CTRL_DUMMY_L_MASK					(0xFFFFFFFFU << FLASH_CTRL_DUMMY_L_SHIFT)

#define FLASH_CTRL_CS_WAIT_BEGIN_SHIFT				(16)
#define FLASH_CTRL_CS_WAIT_BEGIN_MASK					(0xFFU << FLASH_CTRL_CS_WAIT_BEGIN_SHIFT)

#define FLASH_CTRL_CS_WAIT_OVER_SHIFT				(8)
#define FLASH_CTRL_CS_WAIT_OVER_MASK					(0xFFU << FLASH_CTRL_CS_WAIT_OVER_SHIFT)

#define FLASH_CTRL_CS_WAIT_DESEL_SHIFT				(0)
#define FLASH_CTRL_CS_WAIT_DESEL_MASK					(0xFFU << FLASH_CTRL_CS_WAIT_DESEL_SHIFT)

#define FLASH_CTRL_CMD_WAIT_CMD_SHIFT				(16)
#define FLASH_CTRL_CMD_WAIT_CMD_MASK					(0xFFU << FLASH_CTRL_CMD_WAIT_CMD_SHIFT)

#define FLASH_CTRL_CMD_WAIT_ADDR_SHIFT				(8)
#define FLASH_CTRL_CMD_WAIT_ADDR_MASK					(0xFFU << FLASH_CTRL_CMD_WAIT_ADDR_SHIFT)

#define FLASH_CTRL_CMD_WAIT_DUM_SHIFT				(0)
#define FLASH_CTRL_CMD_WAIT_DUM_MASK					(0xFFU << FLASH_CTRL_CMD_WAIT_DUM_SHIFT)

#define FLASH_CTRL_FLASH_COMMON_CFG_WAIT_DATA_SHIFT				(12)
#define FLASH_CTRL_FLASH_COMMON_CFG_WAIT_DATA_MASK					(0x3U << FLASH_CTRL_FLASH_COMMON_CFG_WAIT_DATA_SHIFT)

#define FLASH_CTRL_FLASH_COMMON_CFG_CS_SHIFT				(8)
#define FLASH_CTRL_FLASH_COMMON_CFG_CS_MASK					(0x1U << FLASH_CTRL_FLASH_COMMON_CFG_CS_SHIFT)

typedef enum {
	FLASH_CTRL_TCTRL_CS_LOW_ENABLE = 0 << FLASH_CTRL_FLASH_COMMON_CFG_CS_SHIFT,
	FLASH_CTRL_TCTRL_CS_HIGH_ENABLE = 1 << FLASH_CTRL_FLASH_COMMON_CFG_CS_SHIFT
} Flash_Ctrl_Cs;

#define FLASH_CTRL_FLASH_COMMON_CFG_FBS_SHIFT				(4)
#define FLASH_CTRL_FLASH_COMMON_CFG_FBS_MASK					(0x1U << FLASH_CTRL_FLASH_COMMON_CFG_FBS_SHIFT)

typedef enum {
	FLASH_CTRL_TCTRL_FBS_MSB = 0 << FLASH_CTRL_FLASH_COMMON_CFG_FBS_SHIFT,
	FLASH_CTRL_TCTRL_FBS_LSB = 1 << FLASH_CTRL_FLASH_COMMON_CFG_FBS_SHIFT
} Flash_Ctrl_TCTRL_Fbs;

#define FLASH_CTRL_FLASH_COMMON_CFG_CPOL_SHIFT				(1)
#define FLASH_CTRL_FLASH_COMMON_CFG_CPOL_MASK					(0x1U << FLASH_CTRL_FLASH_COMMON_CFG_CPOL_SHIFT)

#define FLASH_CTRL_FLASH_COMMON_CFG_CPHA_SHIFT				(0)
#define FLASH_CTRL_FLASH_COMMON_CFG_CPHA_MASK					(0x1U << FLASH_CTRL_FLASH_COMMON_CFG_CPHA_SHIFT)

typedef enum {
	FLASH_CTRL_SCLK_Mode0 = 0 << FLASH_CTRL_FLASH_COMMON_CFG_CPHA_SHIFT,
	FLASH_CTRL_SCLK_Mode1 = 1 << FLASH_CTRL_FLASH_COMMON_CFG_CPHA_SHIFT,
	FLASH_CTRL_SCLK_Mode2 = 2 << FLASH_CTRL_FLASH_COMMON_CFG_CPHA_SHIFT,
	FLASH_CTRL_SCLK_Mode3 = 3 << FLASH_CTRL_FLASH_COMMON_CFG_CPHA_SHIFT
} Flash_Ctrl_Sclk_Mode;

#define FLASH_CTRL_XIP_EXEC_SHIFT				(0)
#define FLASH_CTRL_XIP_EXEC_MASK					(0x1U << FLASH_CTRL_XIP_EXEC_SHIFT)

typedef struct {
	uint8_t cs_begin;
	uint8_t cs_over;
	uint8_t cs_deselect;
	uint8_t cmd_over;
	uint8_t addr_over;
	uint8_t dummy_over;
	uint8_t data;	//delay n half cycle
} Flash_Ctrl_DelayCycle;

typedef enum {
	XIP_MODE_DUAL_IO,
	XIP_MODE_QUAD_IO
} XIP_Mode;

typedef struct {
	uint32_t addr;
	uint32_t freq;
	HAL_BoardCfg cb;
	Flash_Ctrl_DelayCycle delay;
#ifdef FLASH_QUAD_READ
	XIP_Mode mode;
	HAL_BoardCfg spi_cb;
#endif
} XIP_Config;

HAL_Status HAL_XIP_Init(XIP_Config *cfg);
HAL_Status HAL_XIP_Deinit();



#ifdef __cplusplus
}
#endif


#endif
