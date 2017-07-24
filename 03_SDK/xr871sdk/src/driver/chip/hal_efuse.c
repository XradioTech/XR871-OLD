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

HAL_Status HAL_EFUSE_SetTimingParam(EFUSE_TimingParam timingParam)
{
	unsigned long flags;

	flags = HAL_EnterCriticalSection();
	if (gEfuseState == EFUSE_STATE_READY) {
		EFUSE_SetTimingParam(timingParam);
		HAL_ExitCriticalSection(flags);
		return HAL_OK;
	} else {
		HAL_ExitCriticalSection(flags);
		HAL_WRN("EFUSE is busy.\n");
		return HAL_BUSY;
	}
}

HAL_Status HAL_EFUSE_Program(uint8_t index, uint32_t data)
{
	unsigned long flags;

	if ((index & EFUSE_INDEX_CHECK_VMASK) != 0) {
		HAL_ERR("invalid eFuse index: %d\n", index);
		return HAL_ERROR;
	}

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

HAL_Status HAL_EFUSE_Read(uint8_t index, uint32_t *pData)
{
	unsigned long flags;

	if ((index & EFUSE_INDEX_CHECK_VMASK) != 0) {
		HAL_ERR("invalid eFuse index: %d\n", index);
		return HAL_ERROR;
	}

	if (pData == NULL) {
		HAL_ERR("pData is NULL.\n");
		return HAL_ERROR;
	}

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

