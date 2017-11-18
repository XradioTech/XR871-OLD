/**
  * @file  drv_oled.h
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

#ifndef _OLED_H_
#define _OLED_H_

#include "driver/component/component_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief Config oled.
  */
typedef struct {
	 SPI_Port oled_SPI_ID;			/*!< The spi id */
	 SPI_TCTRL_SS_Sel oled_SPI_CS;	/*!< The spi cs seclet */
	 uint32_t oled_SPI_MCLK;		/*!< The spi clk freq */
	 GPIO_Port oled_dsPort;			/*!< The spi ds seclet */
	 GPIO_Pin oled_dsPin;			/*!< The spi ds seclet */
	 GPIO_Port oled_reset_Port;		/*!< The spi reset io seclet */
	 GPIO_Pin oled_reset_Pin;		/*!< The spi reset io seclet */
}Oled_Config;

Component_Status DRV_Oled_Pnxm_Bmp(uint8_t column, uint8_t page, uint8_t width, uint8_t hight, const uint8_t *bmp);
Component_Status  DRV_Oled_Showchar_1608(uint8_t x, uint8_t y, uint8_t chr);
Component_Status DRV_Oled_Show_Str_1608(uint8_t column, uint8_t page, const char* str);
int DRV_Oled_P8xnstr(uint8_t column, uint8_t page, const uint8_t* str, uint8_t len);

Component_Status  DRV_Oled_Init(Oled_Config *cfg);
Component_Status DRV_Oled_DeInit();

void DRV_Oled_Reset();
void DRV_Oled_Set_Brightness(uint8_t brightness);
void DRV_Oled_Power_Off();
void DRV_Oled_Power_On();
void DRV_Oled_OnOff(int onoff);
void DRV_Oled_Clear_Screen();


#ifdef __cplusplus
}
#endif

#endif /* _KERNEL_OS_FREERTOS_OS_COMMON_H_ */

