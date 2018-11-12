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

#include <stdio.h>
#include "ssd1306.h"
#include "driver/chip/hal_def.h"
#include "kernel/os/os_mutex.h"

#define SSD1306_DBG 0
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_SSD1306_DBG(fmt, arg...)	\
			LOG(SSD1306_DBG, "[HAL SSD1306] "fmt, ##arg)


GPIO_Port SSD1306_dsPort;
GPIO_Pin SSD1306_dsPin;

#define SPI_NULL_PORT 3
SPI_Port SSD1306_SPI_ID = SPI_NULL_PORT;


OS_Mutex_t SSD1306_SPI_WR_LOCK;

HAL_Status SSD1306_SPI_Write(uint8_t data, SSD1306_WR_MODE mode)
{
	OS_MutexLock(&SSD1306_SPI_WR_LOCK, 10000000);
	HAL_Status sta;
	if(mode == SSD1306_CMD) {
		HAL_GPIO_WritePin(SSD1306_dsPort, SSD1306_dsPin, GPIO_PIN_LOW);
	} else if(mode == SSD1306_DATA) {
		HAL_GPIO_WritePin(SSD1306_dsPort, SSD1306_dsPin, GPIO_PIN_HIGH);
	}
	sta = HAL_SPI_Transmit(SSD1306_SPI_ID, &data, 1);
	if (sta != HAL_OK)
		COMPONENT_WARN("spi write error error %d\n", sta);

	OS_MutexUnlock(&SSD1306_SPI_WR_LOCK);
	return sta;
}

HAL_Status SSD1306_SPI_Init(SSD1306_t *SSD1306config)
{
	SPI_Global_Config gconfig;
	gconfig.cs_level = 0;
	gconfig.mclk = SSD1306config->SSD1306_SPI_MCLK;
	HAL_Status sta = HAL_SPI_Init(SSD1306config->SSD1306_SPI_ID, &gconfig);
	if (sta != HAL_OK)
		COMPONENT_WARN("spi init error %d\n", sta);

	if(OS_MutexCreate(&SSD1306_SPI_WR_LOCK) != OS_OK)
		COMPONENT_WARN("OS_MutexCreate error\n");

	SPI_Config spi_Config;
	spi_Config.firstBit = SPI_TCTRL_FBS_MSB;
	spi_Config.mode = SPI_CTRL_MODE_MASTER;
	spi_Config.opMode = SPI_OPERATION_MODE_POLL;
	spi_Config.sclk = SSD1306config->SSD1306_SPI_MCLK;
	spi_Config.sclkMode = SPI_SCLK_Mode0;

	sta = HAL_SPI_Open(SSD1306config->SSD1306_SPI_ID,
						SSD1306config->SSD1306_SPI_CS, &spi_Config, 1);
	SSD1306_SPI_ID = SSD1306config->SSD1306_SPI_ID;

	if (sta != HAL_OK)
		COMPONENT_WARN("spi open error error %d\n", sta);

	GPIO_InitParam io_Param;
	io_Param.driving = GPIO_DRIVING_LEVEL_3;
	io_Param.mode = GPIOx_Pn_F1_OUTPUT;
	io_Param.pull = GPIO_PULL_NONE;
	HAL_GPIO_Init(SSD1306config->SSD1306_dsPort,
					SSD1306config->SSD1306_dsPin, &io_Param);

	SSD1306_dsPort = SSD1306config->SSD1306_dsPort;
	SSD1306_dsPin = SSD1306config->SSD1306_dsPin;

	SSD1306config->SSD1306_Write = SSD1306_SPI_Write;
	sta = HAL_SPI_CS(SSD1306_SPI_ID, 1);
	if (sta != HAL_OK)
		COMPONENT_WARN("spi cs init error %d\n", sta);
	COMPONENT_TRACK("end\n");
	return sta;
}

HAL_Status SSD1306_SPI_DeInit()
{
	HAL_Status sta = HAL_ERROR;
	if(SSD1306_SPI_ID != SPI_NULL_PORT) {
		OS_MutexDelete(&SSD1306_SPI_WR_LOCK);
		sta = HAL_SPI_Close(SSD1306_SPI_ID);
		if (sta != HAL_OK)
			COMPONENT_WARN("spi close error error %d\n", sta);

		sta = HAL_SPI_Deinit(SSD1306_SPI_ID);
		if(sta != HAL_OK)
			COMPONENT_WARN("spi deinit error error %d\n", sta);
	}
	COMPONENT_TRACK("end\n");
	return sta;
}

void SSD1306_set_brightness(uint8_t brightness)
{
	SSD1306_SPI_Write(SSD1306_SETCONTRAST, SSD1306_CMD);
	SSD1306_SPI_Write(brightness, SSD1306_CMD); //set brightness
}

void SSD1306_Init(SSD1306_t *SSD1306config)
{
	SSD1306_SPI_Write(SSD1306_DISPLAYOFF, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_SETDISPLAYCLOCKDIV, SSD1306_CMD);
	SSD1306_SPI_Write(80, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_SETMULTIPLEX, SSD1306_CMD);
	SSD1306_SPI_Write(0x3F, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_SETDISPLAYOFFSET, SSD1306_CMD);
	SSD1306_SPI_Write(0x00, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_SETSTARTLINE, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_ENABLE_CHARGE_PUMP, SSD1306_CMD);
	SSD1306_SPI_Write(0x14, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_MEMORYMODE, SSD1306_CMD);
	SSD1306_SPI_Write(0x02, SSD1306_CMD);
	SSD1306_SPI_Write(0xA1, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_COMSCANINC, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_SETCOMPINS, SSD1306_CMD);
	SSD1306_SPI_Write(0X12, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_SETCONTRAST, SSD1306_CMD);
	SSD1306_SPI_Write(0x1F, SSD1306_CMD); //set brightness
	SSD1306_SPI_Write(SSD1306_SETPRECHARGE, SSD1306_CMD);
	SSD1306_SPI_Write(0xF1, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_SETVCOMDETECT, SSD1306_CMD);
	SSD1306_SPI_Write(0x30, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_DISPLAYALLON_RESUME, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_NORMALDISPLAY, SSD1306_CMD);
	SSD1306_SPI_Write(SSD1306_DISPLAYON, SSD1306_CMD);
	COMPONENT_TRACK("end\n");
}

