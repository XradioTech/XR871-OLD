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

#include "driver/chip/hal_csi.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_dma.h"
#include "hal_inc.h"

#define CSI_DEBUG 1

#define HAL_CSI_LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)


#define HAL_CSI_DBG(fmt, arg...)	\
			HAL_CSI_LOG(CSI_DEBUG, "[HAL_CSI] "fmt, ##arg)


uint8_t csi_is_run = 0;

void CSI_ModuleEnable()
{
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_CSI);
	HAL_CCM_BusForcePeriphReset(CCM_BUS_PERIPH_BIT_CSI);
	HAL_CCM_BusReleasePeriphReset(CCM_BUS_PERIPH_BIT_CSI);
	HAL_CCM_BusEnablePeriphClock(CCM_BUS_PERIPH_BIT_CSI);
}

void CSI_ModuleDisable()
{
	HAL_CCM_BusDisablePeriphClock(CCM_BUS_PERIPH_BIT_CSI);
}

void CSI_InputFormat() //raw
{
	HAL_CLR_BIT(CSI->CSI_CFG_REG , CSI_CFG_INPUT_FORMAT);
}

HAL_Status HAL_CSI_Config(CSI_Config *csi)
{
	if (csi_is_run) {
		HAL_WARN("%s, %d csi is busy\n", __func__, __LINE__);
		return HAL_BUSY;
	}
	CSI_ModuleEnable();
	HAL_CCM_CSI_SetMClock(csi->src_Clk.clk, csi->src_Clk.divN, csi->src_Clk.divM);
	HAL_CCM_CSI_EnableMClock();

	csi->board_Cfg(0, HAL_BR_PINMUX_INIT, NULL);
	csi_is_run = 1;
	CSI_InputFormat();
	return HAL_OK;
}

void HAL_CSI_OutClk_Ctrl(CSI_CTRL ctrl)
{
	if (CSI_ENABLE == ctrl)
		HAL_CCM_CSI_EnableMClock();
	else
		HAL_CCM_CSI_EnableMClock();
}

void HAL_CSI_DeInit(HAL_BoardCfg board_cfg)
{
	HAL_CLR_BIT(CSI->CSI_EN_REG, CSI_EN);
	HAL_CCM_CSI_DisableMClock();
	CSI_ModuleDisable();
	board_cfg(0, HAL_BR_PINMUX_DEINIT, NULL);
	csi_is_run = 0;
}

void HAL_CSI_Ctrl(CSI_CTRL ctrl)
{
	if (CSI_ENABLE == ctrl)
		HAL_SET_BIT(CSI->CSI_EN_REG, CSI_EN);
	else
		HAL_CLR_BIT(CSI->CSI_EN_REG, CSI_EN);
}

void HAL_CSI_SYNC_Signal_Polarity(CSI_Sync_Signal *signal)
{
	uint32_t csi_sync_pol;
	csi_sync_pol = signal->vsync + (signal->herf << 1) + (signal->p_Clk << 2);
	HAL_CLR_BIT(CSI->CSI_CFG_REG, CSI_CFG_SYNC_SIGNAL_POL);
	HAL_SET_BIT(CSI->CSI_CFG_REG, csi_sync_pol);
}

void HAL_CSI_Capture_Mode(CSI_CAPTURE_MODE mode , CSI_CTRL ctrl)
{
	HAL_CLR_BIT(CSI->CSI_CAP_REG, CSI_CAP_MODE);

	if (CSI_ENABLE == ctrl) {
		if (mode == CSI_STILL_MODE) {
			HAL_SET_BIT(CSI->CSI_CAP_REG, HAL_BIT(0));
		} else if (mode == CSI_VIDEO_MODE)
			HAL_SET_BIT(CSI->CSI_CAP_REG, HAL_BIT(1));
	}
}

void HAL_CSI_Interval_Capture(uint8_t ver_mask, uint16_t hor_mask)
{
	HAL_CLR_BIT(CSI->CSI_SCALE_REG, CSI_VER_MASK);
	HAL_CLR_BIT(CSI->CSI_SCALE_REG, CSI_HER_MASK);

	HAL_SET_BIT(CSI->CSI_SCALE_REG, ver_mask << 24);
	HAL_SET_BIT(CSI->CSI_SCALE_REG, hor_mask);
}

void HAL_CSI_Selection_Next_FIFO (CSI_FIFO fifo_num)
{
	HAL_CLR_BIT(CSI->CSI_BUF_CTL_REG ,HAL_BIT(2));
	HAL_SET_BIT(CSI->CSI_BUF_CTL_REG , fifo_num << 2);
}

CSI_FIFO HAL_CSI_Current_Enable_FIFO()
{
	return  (HAL_GET_BIT(CSI->CSI_BUF_CTL_REG, HAL_BIT(1)) >> 1);
}

void HAL_CSI_Double_FIFO_Mode(CSI_CTRL ctrl)
{
	HAL_CLR_BIT(CSI->CSI_BUF_CTL_REG, HAL_BIT(0));

	if (CSI_ENABLE == ctrl)
		HAL_SET_BIT(CSI->CSI_BUF_CTL_REG, HAL_BIT(0));
}

CSI_Status HAL_CSI_Status()
{
	CSI_Status sta;
	sta.luminance = HAL_GET_BIT(CSI->CSI_BUF_STA_REG, LUM_STATIS) >> 8;
	sta.still_Mode = HAL_GET_BIT(CSI->CSI_BUF_STA_REG, HAL_BIT(1)) >> 1;
	sta.video_Mode = HAL_GET_BIT(CSI->CSI_BUF_STA_REG, HAL_BIT(0));
	return sta;
}

void CSI_Irq_Enable()
{
	HAL_NVIC_SetPriority(CSI_IRQn, 0);//NVIC_PERIPHERAL_PRIORITY_DEFAULT);
	HAL_NVIC_EnableIRQ(CSI_IRQn);
}

void CSI_Irq_Disable()
{
	HAL_NVIC_DisableIRQ(CSI_IRQn);
}

void HAL_CSI_Interrupt_Ctrl(CSI_INTERRUPT_SIGNAL irq_signel, CSI_CTRL ctrl)
{
	if (ctrl == CSI_ENABLE)
		HAL_SET_BIT(CSI->CSI_INT_EN_REG, irq_signel);
	else
		HAL_CLR_BIT(CSI->CSI_INT_EN_REG, irq_signel);
}

__IO uint32_t HAL_CSI_Interrupt_Sta()
{
	return CSI->CSI_INT_STA_REG;
}

void HAL_CSI_Interrupt_Clear()
{
	HAL_SET_BIT(CSI->CSI_INT_STA_REG, CSI->CSI_INT_STA_REG);
}

HAL_Status HAL_CSI_Set_Picture_Size(CSI_Picture_Size *size)
{
	if (size->hor_start > (HAL_BIT(14) - 1)) {
		HAL_WARN("%s, %d csi Picture size error hor_start = %d\n",
					__func__, __LINE__, size->hor_start);
		return HAL_ERROR;
	}

	if (size->hor_len > (HAL_BIT(14) - 1)) {
		HAL_WARN("%s, %d csi Picture size error hor_len = %d\n",
					__func__, __LINE__, size->hor_len);
		return HAL_ERROR;
	}

	HAL_CLR_BIT(CSI->CSI_HSIZE_REG, CSI_SIZE_REG);

	HAL_SET_BIT(CSI->CSI_HSIZE_REG, size->hor_len << 16);
	HAL_SET_BIT(CSI->CSI_HSIZE_REG, size->hor_start);
	return HAL_OK;
}

CSI_FIFO_Data_Len HAL_CSI_FIFO_Data_Len()
{
	CSI_FIFO_Data_Len len;
	len.FIFO_0_B_Data_Len = HAL_GET_BIT(CSI->CSI_TRUE_DATA_NUM, (CSI_VALID_DATA_LEN << 16)) >> 16;
	len.FIFO_0_A_Data_Len = HAL_GET_BIT(CSI->CSI_TRUE_DATA_NUM, CSI_VALID_DATA_LEN);
	return len;
}

void HAL_CSI_Set_Y_Length_In_Line(uint16_t length)
{
	HAL_CLR_BIT(CSI->CSI_BF_LEN_REG, CSI_LUM_LEN);
	HAL_SET_BIT(CSI->CSI_BF_LEN_REG, CSI_LUM_LEN & length);
}

void HAL_CIS_JPEG_Mode_Ctrl(CSI_CTRL ctrl)
{
	if (CSI_ENABLE == ctrl)
		HAL_SET_BIT(CSI->CSI_JPEG_MOD_SEL,  HAL_BIT(0));
	else
		HAL_CLR_BIT(CSI->CSI_JPEG_MOD_SEL, HAL_BIT(0));
}

CSI_Call_Back csi_cb;

void HAL_CSI_CallBack(CSI_Call_Back *cb, CSI_CTRL ctrl)
{
	if (CSI_ENABLE == ctrl) {
		if (cb == NULL) {
			csi_cb.callBack = NULL;
			csi_cb.arg = NULL;
		} else
			csi_cb = *cb;
		CSI_Irq_Enable();
	} else {
		csi_cb.callBack = NULL;
		csi_cb.arg = NULL;
		CSI_Irq_Disable();
	}
}

void CSI_IRQHandler()
{
	if(csi_cb.callBack != NULL)
		csi_cb.callBack(csi_cb.arg);
}

void CSI_Printf()
{
	printf("CSI_EN_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_EN_REG, CSI->CSI_EN_REG);
	printf("CSI_CFG_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_CFG_REG, CSI->CSI_CFG_REG);
	printf("CSI_CAP_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_CAP_REG, CSI->CSI_CAP_REG);
	printf("CSI_SCALE_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_SCALE_REG, CSI->CSI_SCALE_REG);
	printf("CSI_BUF_CTL_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_BUF_CTL_REG, CSI->CSI_BUF_CTL_REG);
	printf("CSI_BUF_STA_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_BUF_STA_REG, CSI->CSI_BUF_STA_REG);
	printf("CSI_INT_EN_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_INT_EN_REG, CSI->CSI_INT_EN_REG);
	printf("CSI_INT_STA_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_INT_STA_REG, CSI->CSI_INT_STA_REG);
	printf("CSI_HSIZE_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_HSIZE_REG, CSI->CSI_HSIZE_REG);
	printf("CSI_BF_LEN_REG 0x%x 0x%x\n", (uint32_t)&CSI->CSI_BF_LEN_REG, CSI->CSI_BF_LEN_REG);
	printf("CSI_TRUE_DATA_NUM 0x%x 0x%x\n", (uint32_t)&CSI->CSI_TRUE_DATA_NUM, CSI->CSI_TRUE_DATA_NUM);
	printf("CSI_JPEG_MOD_SEL 0x%x 0x%x\n", (uint32_t)&CSI->CSI_JPEG_MOD_SEL, CSI->CSI_JPEG_MOD_SEL);

}
