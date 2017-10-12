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

#include "driver/chip/hal_efuse.h"
#include "hal_base.h"

typedef enum {
	EFUSE_STATE_READY	= 0,
	EFUSE_STATE_BUSY	= 1
} EFUSE_State;

EFUSE_State gEfuseState = EFUSE_STATE_READY;

__STATIC_INLINE void EFUSE_EnableClkGate(void)
{
	HAL_SET_BIT(EFUSE->CTRL, EFUSE_CLK_GATE_EN_BIT);
}

__STATIC_INLINE void EFUSE_DisableClkGate(void)
{
	HAL_CLR_BIT(EFUSE->CTRL, EFUSE_CLK_GATE_EN_BIT);
}

__STATIC_INLINE void EFUSE_SetIndex(uint32_t index)
{
	HAL_MODIFY_REG(EFUSE->CTRL, EFUSE_INDEX_MASK,
				   HAL_GET_BIT(index << EFUSE_INDEX_SHIFT, EFUSE_INDEX_MASK));
}

__STATIC_INLINE uint32_t EFUSE_GetHwReadStatus(void)
{
	return !!HAL_GET_BIT(EFUSE->CTRL, EFUSE_HW_READ_STATUS_BIT);
}

__STATIC_INLINE void EFUSE_StartRead(void)
{
	HAL_MODIFY_REG(EFUSE->CTRL, EFUSE_OPERA_LOCK_MASK,
				   (EFUSE_OPERA_UNLOCK_VAL << EFUSE_OPERA_LOCK_SHIFT) | EFUSE_SW_READ_START_BIT);
}

__STATIC_INLINE uint32_t EFUSE_GetSwReadStatus(void)
{
	return !!HAL_GET_BIT(EFUSE->CTRL, EFUSE_SW_READ_START_BIT);
}

__STATIC_INLINE void EFUSE_StartProgram(void)
{
	HAL_MODIFY_REG(EFUSE->CTRL, EFUSE_OPERA_LOCK_MASK,
				   (EFUSE_OPERA_UNLOCK_VAL << EFUSE_OPERA_LOCK_SHIFT) | EFUSE_SW_PROG_START_BIT);
}

__STATIC_INLINE uint32_t EFUSE_GetSwProgStatus(void)
{
	return !!HAL_GET_BIT(EFUSE->CTRL, EFUSE_SW_PROG_START_BIT);
}

__STATIC_INLINE void EFUSE_ClrCtrlReg(void)
{
	HAL_CLR_BIT(EFUSE->CTRL, EFUSE_INDEX_MASK
							 | EFUSE_OPERA_LOCK_MASK
							 | EFUSE_SW_READ_START_BIT
							 | EFUSE_SW_PROG_START_BIT);
}

__STATIC_INLINE void EFUSE_SetProgValue(uint32_t value)
{
	EFUSE->PROGRAM_VALUE = value;
}

__STATIC_INLINE uint32_t EFUSE_GetReadValue(void)
{
	return EFUSE->READ_VALUE;
}

__STATIC_INLINE void EFUSE_SetTimingParam(EFUSE_TimingParam timingParam)
{
	EFUSE->TIMING_CTRL = timingParam;
}

/**
 * @brief Initialize the EFUSE
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_EFUSE_Init(void)
{
	HAL_Status ret;
	unsigned long flags;
	EFUSE_TimingParam timingParam;
	uint32_t clk;

	clk = HAL_GetHFClock();
	switch (clk) {
	case HOSC_CLOCK_24M:
		timingParam = EFUSE_TIMING_PARAM_24M;
		break;
	case HOSC_CLOCK_26M:
		timingParam = EFUSE_TIMING_PARAM_26M;
		break;
	default:
		HAL_WRN("unsupport HOSC %u\n", clk);
		return HAL_ERROR;
	}

	flags = HAL_EnterCriticalSection();
	if (gEfuseState == EFUSE_STATE_READY) {
		EFUSE_SetTimingParam(timingParam);
		ret = HAL_OK;
	} else {
		HAL_WRN("EFUSE is busy.\n");
		ret = HAL_BUSY;
	}
	HAL_ExitCriticalSection(flags);
	return ret;
}

static HAL_Status EFUSE_ReadData(uint8_t index, uint32_t *pData)
{
	unsigned long flags;

	flags = HAL_EnterCriticalSection();
	if (gEfuseState == EFUSE_STATE_READY) {
		gEfuseState = EFUSE_STATE_BUSY;
		HAL_ExitCriticalSection(flags);
	} else {
		HAL_ExitCriticalSection(flags);
		HAL_WRN("EFUSE is busy.\n");
		return HAL_BUSY;
	}

	EFUSE_EnableClkGate();
	EFUSE_SetIndex(index << 2);
	EFUSE_StartRead();

	while (EFUSE_GetSwReadStatus())
		;

	while (EFUSE_GetHwReadStatus())
		;

	*pData = EFUSE_GetReadValue();
	EFUSE_ClrCtrlReg();
	EFUSE_DisableClkGate();

	flags = HAL_EnterCriticalSection();
	gEfuseState = EFUSE_STATE_READY;
	HAL_ExitCriticalSection(flags);

	return HAL_OK;
}

/**
 * @brief Read an amount of data from EFUSE
 * @param[in] start_bit The first bit to be read on EFUSE
 * @param[in] bit_num Number of bits to be read
 * @param[in] data Pointer to the data buffer
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_EFUSE_Read(uint32_t start_bit, uint32_t bit_num, uint8_t *data)
{
	if ((data == NULL)
		|| (start_bit >= HAL_EFUSE_BIT_NUM)
		|| (bit_num == 0)
		|| (bit_num > HAL_EFUSE_BIT_NUM)
		|| (start_bit + bit_num > HAL_EFUSE_BIT_NUM)) {
		HAL_ERR("start bit %d, bit num %d, data %p\n", start_bit, bit_num, data);
		return HAL_ERROR;
	}

	uint8_t	   *p_data = data;
	uint32_t	bit_shift = start_bit & (32 - 1);
	uint32_t	word_idx = start_bit >> 5;

	uint64_t	buf = 0;
	uint32_t   *efuse_word = (uint32_t *)&buf;
	uint32_t	byte_num = (bit_num + 7) >> 3;
	uint32_t	byte_cnt = byte_num;
	uint32_t	bit_cnt;
	uint32_t	copy_size;
	HAL_Status	status;

	HAL_Memset(data, 0, byte_num);

	while (byte_cnt > 0) {
		status = EFUSE_ReadData((uint8_t)word_idx, &efuse_word[0]);
		if (status != HAL_OK) {
			HAL_ERR("status %d\n", status);
			return status;
		}
		if (word_idx + 1 < 64) {
			status = EFUSE_ReadData((uint8_t)word_idx + 1, &efuse_word[1]);
			if (status != HAL_OK) {
				HAL_ERR("status %d\n", status);
				return status;
			}
		} else {
			efuse_word[1] = 0;
		}
		buf = buf >> bit_shift;

		copy_size = (byte_cnt > sizeof(efuse_word[0])) ? sizeof(efuse_word[0]) : byte_cnt;
		HAL_Memcpy(p_data, &efuse_word[0], copy_size);
		byte_cnt -= copy_size;
		p_data += copy_size;
		word_idx++;
	}

	bit_cnt = bit_num & (8 - 1);
	if (bit_cnt > 0)
		data[byte_num - 1] &= ((1 << bit_cnt) - 1);

	return HAL_OK;
}

static HAL_Status EFUSE_WriteData(uint8_t index, uint32_t data)
{
	unsigned long flags;

	flags = HAL_EnterCriticalSection();
	if (gEfuseState == EFUSE_STATE_READY) {
		gEfuseState = EFUSE_STATE_BUSY;
		HAL_ExitCriticalSection(flags);
	} else {
		HAL_ExitCriticalSection(flags);
		HAL_WRN("EFUSE is busy.\n");
		return HAL_BUSY;
	}

	EFUSE_EnableClkGate();
	EFUSE_SetIndex(index << 2);
	EFUSE_SetProgValue(data);
	EFUSE_StartProgram();

	while (EFUSE_GetSwProgStatus())
		;

	EFUSE_ClrCtrlReg();
	EFUSE_DisableClkGate();

	flags = HAL_EnterCriticalSection();
	gEfuseState = EFUSE_STATE_READY;
	HAL_ExitCriticalSection(flags);

	return HAL_OK;
}

/**
 * @brief Write an amount of data to EFUSE
 * @param[in] start_bit The first bit to be written on EFUSE
 * @param[in] bit_num Number of bits to be written
 * @param[in] data Pointer to the data buffer
 * @retval HAL_Status, HAL_OK on success
 */
HAL_Status HAL_EFUSE_Write(uint32_t start_bit, uint32_t bit_num, uint8_t *data)
{
	if ((data == NULL)
		|| (start_bit >= HAL_EFUSE_BIT_NUM)
		|| (bit_num == 0)
		|| (bit_num > HAL_EFUSE_BIT_NUM)
		|| (start_bit + bit_num > HAL_EFUSE_BIT_NUM)) {
		HAL_ERR("start bit %d, bit num %d, data %p\n", start_bit, bit_num, data);
		return HAL_ERROR;
	}

	uint8_t	   *p_data = data;
	uint32_t	bit_shift = start_bit & (32 - 1);
	uint32_t	word_idx = start_bit >> 5;

	uint64_t	buf = 0;
	uint32_t   *efuse_word = (uint32_t *)&buf;
	uint32_t	bit_cnt = bit_num;
	HAL_Status	status;

	HAL_Memcpy(&efuse_word[1], p_data, sizeof(efuse_word[1]));
	if (bit_cnt < 32)
		efuse_word[1] &= (1 << bit_cnt) - 1;
	efuse_word[1] = efuse_word[1] << bit_shift;

	status = EFUSE_WriteData((uint8_t)word_idx, efuse_word[1]);
	if (status != HAL_OK) {
		HAL_ERR("status %d\n", status);
		return status;
	}

	word_idx++;
	bit_cnt -= (bit_cnt <= 32 - bit_shift) ? bit_cnt : 32 - bit_shift;

	while (bit_cnt > 0) {
		HAL_Memcpy(&buf, p_data, sizeof(buf));
		buf = buf << bit_shift;
		if (bit_cnt < 32)
		efuse_word[1] &= (1 << bit_cnt) - 1;

		status = EFUSE_WriteData((uint8_t)word_idx, efuse_word[1]);
		if (status != HAL_OK) {
			HAL_ERR("status %d\n", status);
			return status;
		}

		word_idx++;
		p_data += 4;
		bit_cnt -= (bit_cnt <= 32) ? bit_cnt : 32;
	}

	return HAL_OK;
}

