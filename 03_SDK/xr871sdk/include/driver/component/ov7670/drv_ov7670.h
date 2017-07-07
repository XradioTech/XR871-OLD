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
#ifndef __OV7670_H__
#define __OV7670_H__

#include "driver/component/component_def.h"
#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
	 extern "C" {
#endif

#define OV7670_PICTURE_SEND_UART UART0_ID

typedef enum {
	LIGHT_AUTO,  //The default
	LIGHT_SUNNY,
	LIGHT_COLUDY,
	LIGHT_OFFICE,
	LIGHT_HOME,
} OV7670_LIGHT_MODE;

typedef enum {
	COLOR_SATURATION_0, // -2
	COLOR_SATURATION_1, // -1
	COLOR_SATURATION_2, //The default
	COLOR_SATURATION_3, // 1
	COLOR_SATURATION_4, // 2
} OV7670_COLOR_SATURATION;

typedef enum {
	BRIGHT_0,  // birghtness -2
	BRIGHT_1,  // -1
	BRIGHT_2,  //The default
	BRIGHT_3,  // 1
	BRIGHT_4,  // 2
} OV7670_BRIGHTNESS;

typedef enum {
	CONTARST_0, // -2
	CONTARST_1, // -1
	CONTARST_2, //The default
	CONTARST_3, // 1
	CONTARST_4, // 2
} OV7670_CONTARST;

typedef enum {
	IMAGE_NOMAL,
	IMAGE_NEGATIVE,
	IMAGE_BLACK_WHITE,
	IMAGE_SLANT_RED,
	IMAGE_SLANT_GREEN,
	IMAGE_SLANT_BLUE,
	IMAGE_VINTAGE,
} OV7670_SPECAIL_EFFECTS;

typedef struct {
	GPIO_Port Ov7670_Reset_Port;
	GPIO_Pin Ov7670_Reset_Pin;
	GPIO_Port Ov7670_Pwdn_Port;
	GPIO_Pin Ov7670_Pwdn_Pin;
}Ov7670_PowerCtrlCfg;

void Drv_Ov7670_PowerInit(Ov7670_PowerCtrlCfg *cfg);
void Drv_Ov7670_Reset_Pin_Ctrl(GPIO_PinState state);
void Drv_Ov7670_Pwdn_Pin_Ctrl(GPIO_PinState state);
Component_Status Drv_Ov7670_Init();

void Drv_Ov7670_Set_SaveImage_Buff(uint32_t image_buff_addr);
void Drv_Ov7670_Reset_Image_buff();

void Drv_OV7670_Window_Set(uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);
void Drv_OV7670_Special_Effects(uint8_t eft);
void Drv_Drv_OV7670_Contrast(OV7670_CONTARST contrast);
void Drv_OV7670_Light_Mode(OV7670_LIGHT_MODE light_mode);
void Drv_OV7670_Color_Saturation(OV7670_COLOR_SATURATION sat);
void Drv_OV7670_Brightness(OV7670_BRIGHTNESS bright);

void Drv_Ov7670_Uart_Send_Picture();
void Drv_Ov7670_DeInit();

Component_Status Ov7670_Demo();

#ifdef __cplusplus
	 }
#endif

#endif /* _OV7670_H_ */

