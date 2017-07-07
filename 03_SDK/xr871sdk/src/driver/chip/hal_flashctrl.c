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

#include <stdbool.h>
#include "hal_inc.h"
#include "driver/chip/hal_flashctrl.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_def.h"
#include "driver/chip/device.h"
#include "driver/chip/hal_flashcache.h"
#include "driver/chip/hal_norflash.h"
#include "pm/pm.h"


#include "sys/xr_debug.h"

#define FC_XIP_ONE_LINE_MODE (1)

#define FLASH_QUAD_READ (0)


#define FC_DEBUG(msg, arg...) XR_DEBUG((DBG_OFF | XR_LEVEL_ALL), NOEXPAND, "[FC Debug] " msg, ##arg)

#define FC_REG(reg) FC_DEBUG("register " #reg "(addr:0x%x): 0x%x.\n", (uint32_t)&(reg), reg);

#define FC_REG_ALL() { \
		FC_DEBUG("flash controller register:\n"); \
		FC_REG(FLASH_CTRL->COMMON_CFG); \
		FC_REG(FLASH_CTRL->CMD_CFG); \
		FC_REG(FLASH_CTRL->DUMMY_H); \
		FC_REG(FLASH_CTRL->DUMMY_L); \
		FC_REG(FLASH_CTRL->CS_WAIT); \
		FC_REG(FLASH_CTRL->CMD_WAIT); \
		FC_REG(FLASH_CTRL->RESERVE[0]); \
		FC_REG(FLASH_CTRL->FLASH_COMMON_CFG); \
		FC_REG(FLASH_CTRL->XIP_EXEC); \
	}



typedef enum {
	FLASH_CTRL_EN_CONTINUE = 1 << FLASH_CTRL_COMMON_CFG_CONT_EN_SHIFT,
	FLASH_CTRL_EN_PREFETCH = 1 << FLASH_CTRL_COMMON_CFG_PREFETCH_EN_SHIFT,
	FLASH_CTRL_EN_IBUS = 1 << FLASH_CTRL_COMMON_CFG_IBUS_EN_SHIFT
} Flash_Ctrl_En;

void FlashCtrl_Enable(uint32_t flash_ctrl_en)
{
	HAL_SET_BIT(FLASH_CTRL->COMMON_CFG, flash_ctrl_en);
}

void FlashCtrl_Disable(uint32_t flash_ctrl_en)
{
	HAL_CLR_BIT(FLASH_CTRL->COMMON_CFG, flash_ctrl_en);
}

bool FlashCtrl_IsXIP()
{
	return HAL_GET_BIT(FLASH_CTRL->XIP_EXEC, FLASH_CTRL_XIP_EXEC_MASK);
}

/*typedef enum {
	FLASH_CTRL_READMODE_NORMAL_IO = () | () | (),
	FLASH_CTRL_READMODE_DUAL_OUTPUT,
	FLASH_CTRL_READMODE_DUAL_IO,
	FLASH_CTRL_READMODE_QUAD_OUTPUT,
	FLASH_CTRL_READMODE_QUAD_IO
} Flash_Ctrl_ReadMode;*/

uint32_t FlashCtrl_DefOutput(uint8_t io_num, Flash_Ctrl_Io_Output io)
{
	uint32_t mask, shift;

	if (io_num == 1) {
		mask = FLASH_CTRL_COMMON_CFG_IO1_MASK;
		shift = FLASH_CTRL_COMMON_CFG_IO1_SHIFT;
	} else if (io_num == 2) {
		mask = FLASH_CTRL_COMMON_CFG_IO2_MASK;
		shift = FLASH_CTRL_COMMON_CFG_IO2_SHIFT;
	} else if (io_num == 3) {
		mask = FLASH_CTRL_COMMON_CFG_IO3_MASK;
		shift = FLASH_CTRL_COMMON_CFG_IO3_SHIFT;
	} else
		return -1;

	HAL_MODIFY_REG(FLASH_CTRL->COMMON_CFG, mask, io << shift);

	return 0;
}

void FlashCtrl_ReadConfig(uint8_t read_cmd,
						  Flash_Ctrl_CycleBits cmd,
						  Flash_Ctrl_CycleBits addr,
						  Flash_Ctrl_CycleBits dummy,
						  Flash_Ctrl_CycleBits data,
						  uint8_t dummy_cycle)
{
	uint8_t dummy_width;
	if (dummy == FLASH_CTRL_CYCLEBITS_4)
		dummy_width = dummy_cycle * 4;
/*	else if (dummy == FLASH_CTRL_CYCLEBITS_2)
		dummy_width = dummy_cycle * 2;*/
	else
		dummy_width = dummy_cycle * dummy;

	HAL_MODIFY_REG(FLASH_CTRL->CMD_CFG,
				   FLASH_CTRL_CMD_CFG_CMD_MASK
				   | FLASH_CTRL_CMD_CFG_CMD_BIT_MASK
				   | FLASH_CTRL_CMD_CFG_ADDR_BIT_MASK
				   | FLASH_CTRL_CMD_CFG_DUM_BIT_MASK
				   | FLASH_CTRL_CMD_CFG_DATA_BIT_MASK
				   | FLASH_CTRL_CMD_CFG_DUM_WIDTH_MASK,
				   (read_cmd << FLASH_CTRL_CMD_CFG_CMD_SHIFT)
				   | (cmd << FLASH_CTRL_CMD_CFG_CMD_BIT_SHIFT)
				   | (addr << FLASH_CTRL_CMD_CFG_ADDR_BIT_SHIFT)
				   | (dummy << FLASH_CTRL_CMD_CFG_DUM_BIT_SHIFT)
				   | (data << FLASH_CTRL_CMD_CFG_DATA_BIT_SHIFT)
				   | (dummy_width << FLASH_CTRL_CMD_CFG_DUM_WIDTH_SHIFT));
}

void FlashCtrl_DummyData(uint32_t dummyh, uint32_t dummyl)
{
	FLASH_CTRL->DUMMY_H	= dummyh;
	FLASH_CTRL->DUMMY_L = dummyl;
}

void FlashCtrl_TransmitDelay(Flash_Ctrl_DelayCycle *delay)
{
	HAL_MODIFY_REG(FLASH_CTRL->CS_WAIT,
				   FLASH_CTRL_CS_WAIT_BEGIN_MASK
				   | FLASH_CTRL_CS_WAIT_OVER_MASK
				   | FLASH_CTRL_CS_WAIT_DESEL_MASK,
				   (delay->cs_begin << FLASH_CTRL_CS_WAIT_BEGIN_SHIFT)
				   | (delay->cs_over << FLASH_CTRL_CS_WAIT_OVER_SHIFT)
				   | (delay->cs_deselect << FLASH_CTRL_CS_WAIT_DESEL_SHIFT));
	HAL_MODIFY_REG(FLASH_CTRL->CMD_WAIT,
				   FLASH_CTRL_CMD_WAIT_CMD_MASK
				   | FLASH_CTRL_CMD_WAIT_ADDR_MASK
				   | FLASH_CTRL_CMD_WAIT_DUM_MASK,
				   (delay->cmd_over << FLASH_CTRL_CMD_WAIT_CMD_SHIFT)
				   | (delay->addr_over << FLASH_CTRL_CMD_WAIT_ADDR_SHIFT)
				   | (delay->dummy_over << FLASH_CTRL_CMD_WAIT_DUM_SHIFT));
	HAL_MODIFY_REG(FLASH_CTRL->FLASH_COMMON_CFG,
				   FLASH_CTRL_FLASH_COMMON_CFG_WAIT_DATA_MASK,
				   delay->data << FLASH_CTRL_FLASH_COMMON_CFG_WAIT_DATA_SHIFT);
}

void FlashCtrl_SetFlash(Flash_Ctrl_Cs cs, Flash_Ctrl_TCTRL_Fbs fbs, Flash_Ctrl_Sclk_Mode mode)
{
	HAL_MODIFY_REG(FLASH_CTRL->FLASH_COMMON_CFG,
				   FLASH_CTRL_FLASH_COMMON_CFG_CS_MASK
				   | FLASH_CTRL_FLASH_COMMON_CFG_FBS_MASK
				   | FLASH_CTRL_FLASH_COMMON_CFG_CPOL_MASK
				   | FLASH_CTRL_FLASH_COMMON_CFG_CPHA_MASK,
				   cs | fbs | mode);
}

/*
 * @brief
 */
static void HAL_FlashCtrl_EnableCCMU()
{
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_FLASHC);
	HAL_CCM_FLASHC_EnableMClock();
}


/*
 * @brief
 */
static void HAL_FlashCtrl_DisableCCMU()
{
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_FLASHC);
	HAL_CCM_FLASHC_DisableMClock();
}

static inline void HAL_FlashCtrl_ResetCCMU()
{
	CCM_BusPeriphBit ccm_flashc = CCM_BUS_PERIPH_BIT_FLASHC;	/* spi_port translate to ccm_bit */
	HAL_CCM_BusForcePeriphReset(ccm_flashc);
	HAL_CCM_BusReleasePeriphReset(ccm_flashc);
}


/*
 * @brief
 */
static bool HAL_FlashCtrl_ConfigCCMU(uint32_t clk)
{
	CCM_AHBPeriphClkSrc src;
	uint32_t mclk;
	uint32_t div;

	if (clk > HAL_GetHFClock())
	{
		mclk = HAL_PRCM_GetDevClock();
		src = CCM_AHB_PERIPH_CLK_SRC_DEVCLK;
//		SPI_DEBUG("CCMU src = CCM_AHB_PERIPH_CLK_SRC_DEVCLK.\n");
	}
	else
	{
		mclk = HAL_GetHFClock();
		src = CCM_AHB_PERIPH_CLK_SRC_HFCLK;
//		SPI_DEBUG("CCMU src = CCM_AHB_PERIPH_CLK_SRC_HFCLK.\n");
	}

	div = (mclk + clk - 1) / clk;
	div = div==0 ? 1 : div;

//	_DEBUG("CCMU div = %d\n", div);
	if (div > (16 * 8))
		return 0;

	if (div > 64)
		HAL_CCM_FLASHC_SetMClock(src, CCM_PERIPH_CLK_DIV_N_8, (CCM_PeriphClkDivM)((div >> 3) - 1));
	else if (div > 32)
		HAL_CCM_FLASHC_SetMClock(src, CCM_PERIPH_CLK_DIV_N_4, (CCM_PeriphClkDivM)((div >> 2) - 1));
	else if (div > 16)
		HAL_CCM_FLASHC_SetMClock(src, CCM_PERIPH_CLK_DIV_N_2, (CCM_PeriphClkDivM)((div >> 1) - 1));
	else
		HAL_CCM_FLASHC_SetMClock(src, CCM_PERIPH_CLK_DIV_N_1, (CCM_PeriphClkDivM)((div >> 0) - 1));

	return 1;
}

static HAL_BoardCfg pinmux_cb;
static XIP_Config xip_cfg;
static uint8_t xip_prepared = 0;
static uint8_t xip_on = 0;

#ifdef CONFIG_PM
static int hal_flashc_suspending = 0;

static int flashc_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	hal_flashc_suspending = 1;

	HAL_XIP_Deinit();
	FC_REG_ALL();

	return 0;
}

static int flashc_resume(struct soc_device *dev, enum suspend_state_t state)
{
	HAL_XIP_Init(&xip_cfg);
	FC_REG_ALL();

	hal_flashc_suspending = 0;

	return 0;
}

static struct soc_device_driver flashc_drv = {
	.name = "flashc",
	.suspend_noirq = flashc_suspend,
	.resume_noirq = flashc_resume,
};

static struct soc_device flashc_dev = {
	.name = "flashc",
	.driver = &flashc_drv,
};

#define FLASHC_DEV (&flashc_dev)
#else
#define FLASHC_DEV NULL
#endif


HAL_Status HAL_XIP_Init(XIP_Config *cfg)
{
	uint32_t mode;
	/* spi flash doing */

	memcpy(&xip_cfg, cfg, sizeof(xip_cfg));

#if FLASH_QUAD_READ
	if (xip_prepared == 0) {
		HAL_Status ret;

		SF_Handler hdl;
		SF_Config config;
		config.sclk = 12 * 1000 * 1000;
		config.spi = SPI0;
		config.cs = SPI_TCTRL_SS_SEL_SS0;
		config.dma_en = 1;

		if ((ret = HAL_SF_Init(&hdl, &config)) != HAL_OK) {
			HAL_SPI_Deinit((SPI_Port)SPI0);
			return ret;
		}

		HAL_SF_Config(&hdl, SF_ATTRIBUTION_READ_MODE, SF_READ_MODE_QUAD_IO);

		HAL_SF_Deinit(&hdl);
	}
	mode = 1;

#else
	//PN25F08 dual io should be no config
	mode = 0;
#endif
	xip_prepared = 1;

	/* enable ccmu */
	HAL_FlashCtrl_DisableCCMU();
	HAL_FlashCtrl_ResetCCMU();
	HAL_FlashCtrl_ConfigCCMU(cfg->freq);
	HAL_FlashCtrl_EnableCCMU();

	/* open io */
	pinmux_cb = cfg->cb;
	pinmux_cb(0, HAL_BR_PINMUX_INIT, &mode);

	FlashCtrl_Disable(FLASH_CTRL_EN_CONTINUE | FLASH_CTRL_EN_IBUS | FLASH_CTRL_EN_PREFETCH);

	/* config flash controller */
	Flash_Ctrl_DelayCycle delay = {1, 1, 8, 0, 0, 0, 1};
	FlashCtrl_TransmitDelay(&delay);	// restart system bug here now.

	FlashCtrl_SetFlash(FLASH_CTRL_TCTRL_CS_LOW_ENABLE, FLASH_CTRL_TCTRL_FBS_MSB, FLASH_CTRL_SCLK_Mode0);

	unsigned long flags = HAL_EnterCriticalSection();

#if FLASH_QUAD_READ
	FlashCtrl_ReadConfig(0xEB,
						 FLASH_CTRL_CYCLEBITS_1,
						 FLASH_CTRL_CYCLEBITS_4,
						 FLASH_CTRL_CYCLEBITS_4,
						 FLASH_CTRL_CYCLEBITS_4,
						 6);

	FlashCtrl_DummyData(0x20000000, 0);
	FlashCtrl_Enable(FLASH_CTRL_EN_CONTINUE | FLASH_CTRL_EN_IBUS/* | FLASH_CTRL_EN_PREFETCH */);


#else
#if FC_XIP_ONE_LINE_MODE
	FlashCtrl_ReadConfig(0x3B,
						 FLASH_CTRL_CYCLEBITS_1,
						 FLASH_CTRL_CYCLEBITS_1,
						 FLASH_CTRL_CYCLEBITS_1,
						 FLASH_CTRL_CYCLEBITS_2,
						 8);
	FlashCtrl_DummyData(0, 0);
	FlashCtrl_Enable(FLASH_CTRL_EN_IBUS);

#else /* FC_XIP_ONE_LINE_MODE */
	FlashCtrl_ReadConfig(0xBB,
						 FLASH_CTRL_CYCLEBITS_1,
						 FLASH_CTRL_CYCLEBITS_2,
						 FLASH_CTRL_CYCLEBITS_2,
						 FLASH_CTRL_CYCLEBITS_2,
						 4);
	FlashCtrl_DummyData(0x20000000, 0);
	FlashCtrl_Enable(FLASH_CTRL_EN_CONTINUE | FLASH_CTRL_EN_IBUS/* | FLASH_CTRL_EN_PREFETCH */);

#endif /* FC_XIP_ONE_LINE_MODE */
#endif

	xip_on = 1;
	HAL_ExitCriticalSection(flags);


	//config flash cache
	FlashCache_Config cache_cfg = {cfg->addr};
	Hal_FlashCache_Init(&cache_cfg);

#ifdef CONFIG_PM
	if (!hal_flashc_suspending)
		pm_register_ops(FLASHC_DEV);
#endif

	FC_REG_ALL();

	return HAL_OK;
}

HAL_Status HAL_XIP_Deinit()
{
#ifdef CONFIG_PM
	if (!hal_flashc_suspending)
		pm_unregister_ops(FLASHC_DEV);
#endif

	unsigned long flags = HAL_EnterCriticalSection();
	//config flash controller
	FlashCtrl_Disable(FLASH_CTRL_EN_CONTINUE | FLASH_CTRL_EN_IBUS | FLASH_CTRL_EN_PREFETCH);
	xip_on = 0;
	HAL_ExitCriticalSection(flags);

	//close io
#if	FLASH_QUAD_READ
	pinmux_cb(0, HAL_BR_PINMUX_DEINIT, 1);
#else
	pinmux_cb(0, HAL_BR_PINMUX_DEINIT, 0);
#endif

	//deinit flash cache
	Hal_FlashCache_Deinit();

	//disable ccmu
	HAL_FlashCtrl_DisableCCMU();

	return HAL_OK;
}

bool HAL_XIP_IsRunning()
{
	return xip_on;
}

HAL_Status HAL_XIP_Start()
{
	if (xip_prepared == 0)
		return HAL_OK;
	return HAL_XIP_Init(&xip_cfg);
}

HAL_Status HAL_XIP_Stop()
{
	if (xip_prepared == 0)
		return HAL_OK;
	return HAL_XIP_Deinit();
}

