/**
  * @file  hal_global.c
  * @author  XRADIO IOT WLAN Team
  */

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

#include "hal_base.h"

static uint8_t g_chip_version = 0;

/**
 * @brief Global initialization for HAL module
 * @return None
 */
void HAL_GlobalInit(void)
{
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_DEFAULT);
	HAL_CCM_BusForceAllPeriphReset();
	HAL_CCM_BusDisableAllPeriphClock();
}

static uint8_t GlobalGetBitValue(uint32_t start_bit, uint8_t bit_cnt)
{
	volatile uint8_t *reg = (volatile uint8_t *)0x40043D00;

	uint32_t reg0_idx = start_bit >> 3;	/* start_bit / 8 */
	uint32_t reg1_idx = (start_bit + bit_cnt - 1) >> 3;	/* end_bit / 8 */
	uint32_t bit_idx = start_bit & 0x7;	/* start_bit % 8 */

	if (reg0_idx == reg1_idx) {
		return HAL_GET_BIT_VAL(reg[reg0_idx], bit_idx, (1 << bit_cnt) - 1);
	} else {
		uint16_t value = (((uint16_t)reg[reg1_idx]) << 8) | reg[reg0_idx];
		return HAL_GET_BIT_VAL(value, bit_idx, (1 << bit_cnt) - 1);
	}
}

/**
 * @brief Get chip version
 * @return Chip version, 0 on failure
 */
uint32_t HAL_GlobalGetChipVer(void)
{
	uint32_t start_bit;

	if (g_chip_version == 0) {
		start_bit = GlobalGetBitValue(608, 2);
		start_bit = (start_bit == 0 ? 200 : 610) + 22;
		g_chip_version = GlobalGetBitValue(start_bit, 6);
	}

	return g_chip_version;
}

uint8_t HAL_GlobalGetSmpsBgtr(void)
{
	uint8_t val = GlobalGetBitValue(0, 4);

	if (val == 0) {
		val = GlobalGetBitValue(72, 5);
		val = (val << 1) & 0x1f;
		return val;
	}
	return 0xff;
}
