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

#ifndef _DRIVER_CHIP_HAL_CSI_H_
#define _DRIVER_CHIP_HAL_CSI_H_

#include "driver/chip/hal_def.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	__IO uint32_t CSI_EN_REG;		/*CSI enable register*/
	__IO uint32_t CSI_CFG_REG;		/*CSI configuration register*/
	__IO uint32_t CSI_CAP_REG;		/*CSI capture control register*/
	__IO uint32_t CSI_SCALE_REG;	/*CSI scale */
	uint32_t RESERVED1[6];
	__IO uint32_t CSI_BUF_CTL_REG;	/*CSI output buffer control register*/
	__IO uint32_t CSI_BUF_STA_REG;	/*CSI status register*/
	__IO uint32_t CSI_INT_EN_REG;	/*CSI interrupt enable register*/
	__IO uint32_t CSI_INT_STA_REG;	/*CSI interrupt size register*/
	uint32_t RESERVED2[2];
	__IO uint32_t CSI_HSIZE_REG;	/*CSI horizontal size register*/
	uint32_t RESERVED3[1];
	__IO uint32_t CSI_BF_LEN_REG;	/*CSI line buffer length register*/
	__IO uint32_t CSI_TRUE_DATA_NUM;/*CSI true data number of fifo sram register*/
	__IO uint32_t CSI_JPEG_MOD_SEL; /*CSI JPEG Mode select register*/
}CSI_T ;


#define	CSI	((CSI_T *)DCMI_BASE)

#define CSI_FIFO_A (DCMI_BASE + 0X800)
#define CSI_FIFO_B (DCMI_BASE + 0XA00)

#define	CSI_EN HAL_BIT(0)
#define	CSI_CFG_INPUT_FORMAT ((uint32_t)0x00700000)
#define	CSI_CFG_SYNC_SIGNAL_POL ((uint32_t)0x00000007)
#define	CSI_CAP_MODE ((uint32_t)0x00000003)

#define	CSI_VER_MASK ((uint32_t)0x0F000000)
#define	CSI_HER_MASK ((uint32_t)0x000000FF)

#define	LUM_STATIS	((uint32_t)0xFFFFFF00)

#define CSI_INTERRUPT_CLEAR ((uint32_t)0x000003FF)

#define CSI_SIZE_REG ((uint32_t)0xFFFFFFFF)

#define CSI_LUM_LEN ((uint32_t)0x00001FFF)
#define CSI_VALID_DATA_LEN ((uint32_t)0x000003FF)


typedef struct {
	CCM_AHBPeriphClkSrc clk;
	CCM_PeriphClkDivN divN;
	CCM_PeriphClkDivM divM;
} CSI_Clk;

typedef struct {
	CSI_Clk src_Clk;
	CSI_Clk out_Clk;
	HAL_BoardCfg board_Cfg;
} CSI_Config;

typedef enum {
	CSI_DISABLE,
	CSI_ENABLE,
}CSI_CTRL;

typedef enum {
	CSI_NEGATIVE,
	CSI_POSITIVE,
}CSI_SYNC_SIGNAL_POLARITY;

typedef struct {
	CSI_SYNC_SIGNAL_POLARITY vsync;
	CSI_SYNC_SIGNAL_POLARITY herf;
	CSI_SYNC_SIGNAL_POLARITY p_Clk;
} CSI_Sync_Signal;

typedef enum {
	CSI_STILL_MODE,
	CSI_VIDEO_MODE,
} CSI_CAPTURE_MODE;

typedef enum {
	CSI_FIFO_0_A,
	CSI_FIFO_0_B,
}CSI_FIFO;

typedef enum {
	CSI_FREE,
	CSI_BUSY,
} CSI_RUN_STA;

typedef struct {
	uint32_t luminance;
	CSI_RUN_STA video_Mode;
	CSI_RUN_STA still_Mode;
} CSI_Status;

typedef struct {
	uint16_t hor_start;
	uint16_t hor_len; //the picture one row contains the number of data(not number of pixel), unit byte.
} CSI_Picture_Size;

typedef enum {
	CSI_CAPTURE_DONE_IRQ = HAL_BIT(0),
	CSI_FRAME_DONE_IRQ = HAL_BIT(1),
	CSI_FIFO_0_OVERFLOW_IRQ = HAL_BIT(2),
	CSI_ALL_FIFO_OVERFLOW_IRQ = HAL_BIT(6),
	CSI_VSYNC_IRQ =  HAL_BIT(7),
	CSI_FIFO_0_A_READY_IRQ = HAL_BIT(8),
	CSI_FIFO_0_B_READY_IRQ = HAL_BIT(9),
} CSI_INTERRUPT_SIGNAL;

typedef struct {
	uint16_t FIFO_0_A_Data_Len;
	uint16_t FIFO_0_B_Data_Len;
}CSI_FIFO_Data_Len;

typedef struct {
	void *arg;
	void(*callBack)(void *arg);
}CSI_Call_Back;

HAL_Status HAL_CSI_Config(CSI_Config *csi);
void HAL_CSI_OutClk_Ctrl(CSI_CTRL ctrl);


void HAL_CSI_DeInit(HAL_BoardCfg board_cfg);
void HAL_CSI_Ctrl(CSI_CTRL ctrl);

void HAL_CSI_SYNC_Signal_Polarity(CSI_Sync_Signal *signal);
void HAL_CSI_Capture_Mode(CSI_CAPTURE_MODE mode , CSI_CTRL ctrl);
void HAL_CSI_Interval_Capture(uint8_t ver_mask, uint16_t hor_mask);

void HAL_CSI_Selection_Next_FIFO (CSI_FIFO fifo_num);
CSI_FIFO HAL_CSI_Current_Enable_FIFO();
void HAL_CSI_Double_FIFO_Mode(CSI_CTRL ctrl);

CSI_Status HAL_CSI_Status();
void HAL_CSI_Interrupt_Ctrl(CSI_INTERRUPT_SIGNAL irq_signel, CSI_CTRL ctrl);
__IO uint32_t HAL_CSI_Interrupt_Sta();
void HAL_CSI_Interrupt_Clear();

void HAL_CSI_Set_Y_Length_In_Line(uint16_t length);
HAL_Status HAL_CSI_Set_Picture_Size(CSI_Picture_Size *size);
CSI_FIFO_Data_Len HAL_CSI_FIFO_Data_Len();
void HAL_CIS_JPEG_Mode_Ctrl(CSI_CTRL ctrl);
void HAL_CSI_CallBack(CSI_Call_Back *cb, CSI_CTRL ctrl);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_CSI_H_ */

