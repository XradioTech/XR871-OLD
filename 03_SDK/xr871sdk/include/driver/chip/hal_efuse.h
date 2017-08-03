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

#ifndef _DRIVER_CHIP_HAL_EFUSE_H_
#define _DRIVER_CHIP_HAL_EFUSE_H_

#include "driver/chip/hal_def.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
		 uint32_t RESERVED0[16];	/* 0x0000 reserved								*/
	__IO uint32_t CTRL;				/* 0x0040 EFUSE program/read control register	*/
		 uint32_t RESERVED1[3];		/* 0x0044 reserved								*/
	__IO uint32_t PROGRAM_VALUE;	/* 0x0050 EFUSE program key value register		*/
		 uint32_t RESERVED2[3];		/* 0x0054 reserved								*/
	__I  uint32_t READ_VALUE;		/* 0x0060 EFUSE read key value register			*/
		 uint32_t RESERVED3[11];	/* 0x0064 reserved								*/
	__IO uint32_t TIMING_CTRL;		/* 0x0090 EFUSE burned timing control			*/
	__IO uint32_t DEBUG;			/* 0x0094 EFUSE debug register					*/
} EFUSE_T;

#define EFUSE	((EFUSE_T *)SID_BASE)

/* EFUSE->CTRL */
#define EFUSE_CLK_GATE_EN_BIT	HAL_BIT(28)

#define EFUSE_INDEX_SHIFT		16
#define EFUSE_INDEX_MASK		((0xFFU) << EFUSE_INDEX_SHIFT)
#define EFUSE_INDEX_CHECK_VMASK	0xC0U

#define EFUSE_OPERA_LOCK_SHIFT	8
#define EFUSE_OPERA_LOCK_MASK	((0xFFU) << EFUSE_OPERA_LOCK_SHIFT)
#define EFUSE_OPERA_UNLOCK_VAL	0xACU

#define EFUSE_HW_READ_STATUS_BIT	HAL_BIT(2)
#define EFUSE_SW_READ_START_BIT		HAL_BIT(1)
#define EFUSE_SW_PROG_START_BIT		HAL_BIT(0)

/* EFUSE->TIMING_CTRL */
typedef enum {
	EFUSE_TIMING_PARAM_24M	= 0x63321190U,
	EFUSE_TIMING_PARAM_26M	= 0x63321190U,
} EFUSE_TimingParam;

/******************************************************************************/

#define EFUSE_BIT_NUM	(2048)

HAL_Status HAL_EFUSE_Init(void);

/*
 * index:	0~63
 * bit:		64 * 32 = 2048
 */
HAL_Status HAL_EFUSE_Program(uint8_t index, uint32_t data);
HAL_Status HAL_EFUSE_Read(uint8_t index, uint32_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_EFUSE_H_ */
