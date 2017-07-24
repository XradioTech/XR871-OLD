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

#include "board_debug.h"
#include "board.h"
#include "driver/chip/hal_codec.h"
#include <string.h>

/* uart */
__weak HAL_Status board_uart_init(UART_ID uart_id)
{
	static const UART_InitParam board_uart_param = {
		.baudRate		  = BOARD_UART_BAUD_RATE,
		.parity 		  = BOARD_UART_PARITY,
		.stopBits		  = BOARD_UART_STOP_BITS,
		.dataBits		  = BOARD_UART_DATA_BITS,
		.isAutoHwFlowCtrl = BOARD_UART_HW_FLOW_CTRL
	};

	return HAL_UART_Init(uart_id, &board_uart_param);
}

__weak HAL_Status board_uart_deinit(UART_ID uart_id)
{
	return HAL_UART_DeInit(uart_id);
}

__weak int32_t board_uart_write(UART_ID uart_id, char *buf, int count)
{
	return HAL_UART_Transmit_Poll(uart_id, (uint8_t *)buf, count);
}

/* spi */
__weak HAL_Status board_spi_init(SPI_Port spi)
{
	static const SPI_Global_Config board_spi_param = {
		.mclk	  = BOARD_SPI_MCLK,
		.cs_level = BOARD_SPI_CS_LEVEL
	};

	return HAL_SPI_Init(spi, &board_spi_param);
}

__weak HAL_Status board_spi_deinit(SPI_Port spi)
{
	return HAL_SPI_Deinit(spi);
}

/* sound card0 */
#ifdef __CONFIG_SOUNDCARD0_ENABLE
__weak HAL_Status board_soundcard0_init(void)
{
	static const I2C_InitParam i2c_param = {
		.addrMode	= BOARD_SOUNDCARD0_I2C_ADDR_MODE,
		.clockFreq	= BOARD_SOUNDCARD0_I2C_CLK
	};

	static const CODEC_Param acodec_param = {
		.name	= (uint8_t *)BOARD_SOUNDCARD0_CODEC_NAME,
		.write	= BOARD_SOUNDCARD0_CODEC_WRITE,
		.read	= BOARD_SOUNDCARD0_CODEC_READ,
		.i2cId	= BOARD_SOUNDCARD0_I2C_ID,
		.param 	= NULL
	};

	I2S_Param i2s_param;
	HAL_Status ret;

	ret = HAL_I2C_Init(BOARD_SOUNDCARD0_I2C_ID, &i2c_param);
	if (ret != HAL_OK) {
		BOARD_ERR("I2C %d init failed\n", BOARD_SOUNDCARD0_I2C_ID);
		return ret;
	}

	memset(&i2s_param, 0, sizeof(i2s_param));
	ret = HAL_I2S_Init(&i2s_param);
	if (ret != HAL_OK) {
		BOARD_ERR("I2S init failed\n");
		HAL_I2C_DeInit(BOARD_SOUNDCARD0_I2C_ID);
		return ret;
	}

	ret = HAL_CODEC_Init(&acodec_param);
	if (ret != HAL_OK) {
		BOARD_ERR("acodec init failed\n");
		HAL_I2S_DeInit();
		HAL_I2C_DeInit(BOARD_SOUNDCARD0_I2C_ID);
		return ret;
	}

	return ret;
}

__weak HAL_Status board_soundcard0_deinit(void)
{
	HAL_CODEC_DeInit();
	HAL_I2S_DeInit();
	return HAL_I2C_DeInit(BOARD_SOUNDCARD0_I2C_ID);
}
#endif /* __CONFIG_SOUNDCARD0_ENABLE */

/* sound card1 */
#ifdef __CONFIG_SOUNDCARD1_ENABLE
__weak HAL_Status board_soundcard1_init(void)
{
	DMIC_Param dmic_param;
	memset(&dmic_param, 0, sizeof(dmic_param));

	return HAL_DMIC_Init(&dmic_param);
}

__weak HAL_Status board_soundcard1_deinit(void)
{
	HAL_DMIC_DeInit();
	return HAL_OK;
}
#endif /* __CONFIG_SOUNDCARD1_ENABLE */

#ifdef __CONFIG_XIP_ENABLE
/* TODO: temp implementation, put it to system init level later */
#include "driver/chip/hal_flashctrl.h"
#include "sys/image.h"

void board_xip_init(void)
{
	XIP_Config xip;
	image_handle_t *hdl;
	uint32_t addr;

	hdl = image_open();
	if (hdl == NULL) {
		BOARD_ERR("image open failed\n");
		return;
	}

	addr = image_get_section_addr(hdl, IMAGE_APP_XIP_ID);
	if (addr == IMAGE_INVALID_OFFSET) {
		BOARD_ERR("no xip section\n");
		image_close(hdl);
		return;
	}

	/* TODO: check section's validity */

	image_close(hdl);

	if (addr != IMAGE_INVALID_OFFSET) {
		xip.addr = addr + IMAGE_HEADER_SIZE;
		xip.freq = 64 * 1000 * 1000;
		BOARD_DBG("xip addr 0x%x, freq %u\n", xip.addr, xip.freq);
		HAL_XIP_Init(&xip);
	}
}
#endif /* __CONFIG_XIP_ENABLE */

#if 1 /* TODO: implement in flash driver */
static HAL_Status HAL_Flash_Init(SF_Handler *hdl)
{
	const SF_Config *flash_cfg;
	HAL_Status ret;

	ret = HAL_BoardIoctl(HAL_BIR_GET_CFG, HAL_MKDEV(HAL_DEV_MAJOR_FLASH, 0),
	                     (uint32_t)&flash_cfg);
	if (ret != HAL_OK) {
		BOARD_ERR("get flash config failed\n");
		return ret;
	}
	return HAL_SF_Init(hdl, flash_cfg);
}

static HAL_Status HAL_Flash_DeInit(SF_Handler *hdl)
{
	if (hdl && *hdl) {
		return HAL_SF_Deinit(hdl);
	} else {
		return HAL_INVALID;
	}
}

int board_flash_init(SF_Handler *hdl)
{
	return (HAL_Flash_Init(hdl) == HAL_OK ? 0 : -1);
}

void board_flash_deinit(SF_Handler *hdl)
{
	HAL_Flash_DeInit(hdl);
}
#endif
