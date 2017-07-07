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

#ifndef __HAL_NORFLASH_H_
#define __HAL_NORFLASH_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "driver/chip/hal_spi.h"
#include "sys/xr_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	SF_ERASE_SIZE_NOSUPPORT,
	SF_ERASE_SIZE_4KB		= 0x20,
	SF_ERASE_SIZE_32KB 		= 0x52,
	SF_ERASE_SIZE_64KB 		= 0xD8,
	SF_ERASE_SIZE_CHIP		= 0xC7
} SF_Erase_Size;

typedef enum {
//	SF_ATTRIBUTION_PP_MODE,
	SF_ATTRIBUTION_READ_MODE
} SF_Attribution;

/*
// expanded config example
typedef enum {
	SF_PP_MODE_NORMAL_PP,
	SF_PP_MODE_QUAD_PP,
} SF_PP_Mode;
*/

typedef enum {
	SF_READ_MODE_NORMAL_IO,
	SF_READ_MODE_DUAL_OUTPUT,
	SF_READ_MODE_DUAL_IO,
	SF_READ_MODE_QUAD_OUTPUT,
	SF_READ_MODE_QUAD_IO
} SF_Read_Mode;

#define SF_MAX_WAIT_TIME (5000)

#define FLASH_M25P64 /* norflash for code */
#define FLASH_W25Q16FW
#define FLASH_PN25F08
#define FLASH_MX25L1636E
#define FLASH_MX25L1633E
#define FLASH_MX25L1606E
#define FLASH_PN25F08B
#define FLASH_PN25F16B
#define FLASH_EN25QH16A
#define FLASH_PN25F16

typedef void * SF_Handler;

typedef struct {
	SPI_Port spi;
	SPI_CS cs;
	uint32_t sclk;
	bool dma_en;
} SF_Config;

HAL_Status HAL_SF_Init(SF_Handler *hdl, SF_Config *config);
HAL_Status HAL_SF_Deinit(SF_Handler *hdl);
HAL_Status HAL_SF_Config(SF_Handler *hdl, SF_Attribution attr, uint32_t arg);
HAL_Status HAL_SF_Write(SF_Handler *hdl, uint32_t addr, uint8_t *data, uint32_t size);
HAL_Status HAL_SF_Read(SF_Handler *hdl, uint32_t addr, uint8_t *data, uint32_t size);
HAL_Status HAL_SF_Erase(SF_Handler *hdl, SF_Erase_Size blk_size, uint32_t addr, uint32_t blk_cnt);
HAL_Status HAL_SF_MemoryOf(SF_Handler *hdl, SF_Erase_Size size, uint32_t addr, uint32_t *start);
void HAL_SF_Test(void);



/**************************** debug *************************************/

#define SF_MODULE (DBG_OFF | XR_LEVEL_DEBUG)

#if ((SF_MODULE & DBG_ON) != 0)
#define SF_MODULE_DEBUG
#endif

#define SF_ASSERT(condition) XR_ASSERT(condition, SF_MODULE, #condition " failed\n")

#define SF_DEBUG(msg, arg...) XR_DEBUG(SF_MODULE, NOEXPAND, "[SPI NorFlash Debug] " msg, ##arg)

#define SF_ALERT(msg, arg...) XR_ALERT(SF_MODULE, NOEXPAND, "[SPI NorFlash Alert] " msg, ##arg)

#define SF_ENTRY() XR_ENTRY(SF_MODULE, "[SF(Spi NorFlash)]")
#define SF_EXIT() XR_RET_NOVAL(SF_MODULE, "[SF(Spi NorFlash)]")
#define SF_EXIT_VAL(val) XR_RET(SF_MODULE, "[SF(Spi NorFlash)]", val)

#ifdef __cplusplus
}
#endif

#endif
