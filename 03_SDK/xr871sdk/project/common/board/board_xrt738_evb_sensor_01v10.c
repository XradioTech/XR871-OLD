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

#if (defined(__CONFIG_BOARD_XRT738_EVB_SENSOR_01V10))

#include "board_debug.h"
#include "board_common.h"
#include "board_xrt738_evb_sensor_01v10.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_i2c.h"
#include "driver/chip/hal_adc.h"
#include "driver/chip/hal_spi.h"
#include "driver/chip/hal_norflash.h"
#include "driver/chip/hal_pwm.h"
#include "driver/chip/sdmmc/hal_sdhost.h"
#include "driver/chip/hal_codec.h"
#include "pm/pm.h"

static GPIO_PinMuxParam g_pinmux_uart0[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_B, GPIO_PIN_0,  { GPIOB_P0_F2_UART0_TX,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },	/* TX */
	{ GPIO_PORT_B, GPIO_PIN_1,  { GPIOB_P1_F2_UART0_RX,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },	/* RX */
};

static GPIO_PinMuxParam g_pinmux_uart1[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_17, { GPIOA_P17_F5_UART1_TX,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },	/* TX */
	{ GPIO_PORT_A, GPIO_PIN_18, { GPIOA_P18_F5_UART1_RX,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },	/* RX */
};

static GPIO_PinMuxParam g_pinmux_dmic[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_21,  { GPIOA_P21_F3_DMIC_CLK,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_22,  { GPIOA_P22_F3_DMIC_DATA,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

static GPIO_PinMuxParam g_pinmux_i2s[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_12,  { GPIOA_P12_F4_I2S_MCLK,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_13,  { GPIOA_P13_F4_I2S_BCLK,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_14,  { GPIOA_P14_F4_I2S_DI,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_15,  { GPIOA_P15_F4_I2S_DO,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_16,  { GPIOA_P16_F4_I2S_LRCLK,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

static GPIO_PinMuxParam g_pinmux_irrx[] = {
//        port         pin            mode                      driving               pull
	{ GPIO_PORT_A, GPIO_PIN_17, { GPIOA_P17_F3_IR_RX,       GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

static GPIO_PinMuxParam g_pinmux_irtx[] = {
//        port         pin            mode                      driving               pull
	{ GPIO_PORT_A, GPIO_PIN_18, { GPIOA_P18_F3_IR_TX,       GPIO_DRIVING_LEVEL_1, GPIO_PULL_DOWN } },
};

static GPIO_PinMuxParam g_pinmux_i2c0[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_4,  { GPIOA_P4_F4_I2C0_SCL,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
	{ GPIO_PORT_A, GPIO_PIN_5,  { GPIOA_P5_F4_I2C0_SDA,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

static GPIO_PinMuxParam g_pinmux_i2c1[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_17, { GPIOA_P17_F4_I2C1_SCL,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
	{ GPIO_PORT_A, GPIO_PIN_18, { GPIOA_P18_F4_I2C1_SDA,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

static GPIO_PinMuxParam g_pinmux_spk[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_17, { GPIOx_Pn_F1_OUTPUT,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};


static GPIO_PinMuxParam g_pinmux_adc[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_8,  { GPIOA_P8_F2_ADC_CH0,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_9,  { GPIOA_P9_F2_ADC_CH1,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_10, { GPIOA_P10_F2_ADC_CH2, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_11, { GPIOA_P11_F2_ADC_CH3, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_12, { GPIOA_P12_F2_ADC_CH4, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_13, { GPIOA_P13_F2_ADC_CH5, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_14, { GPIOA_P14_F2_ADC_CH6, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_15, { GPIOA_P15_F2_ADC_CH7, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

static GPIO_PinMuxParam g_pinmux_spi0[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_B, GPIO_PIN_4,  { GPIOB_P4_F2_SPI0_MOSI,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_5,  { GPIOB_P5_F2_SPI0_MISO,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_7,  { GPIOB_P7_F2_SPI0_CLK,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

static GPIO_PinMuxParam g_pinmux_spi0_cs0[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_B, GPIO_PIN_6,  { GPIOB_P6_F2_SPI0_CS0,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

static GPIO_PinMuxParam g_pinmux_spi1[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_0,  { GPIOA_P0_F2_SPI1_MOSI,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
	{ GPIO_PORT_A, GPIO_PIN_2,  { GPIOA_P2_F2_SPI1_CLK,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } },
};

static GPIO_PinMuxParam g_pinmux_spi1_cs0[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_3,  { GPIOA_P3_F2_SPI1_CS0,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

static GPIO_PinMuxParam g_pinmux_spi1_cs1[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_6,  { GPIOA_P6_F3_SPI1_CS1,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

static GPIO_PinMuxParam g_pinmux_spi1_cs2[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_7,  { GPIOA_P7_F3_SPI1_CS2,   GPIO_DRIVING_LEVEL_1, GPIO_PULL_UP } },
};

static GPIO_PinMuxParam g_pinmux_flash[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_B, GPIO_PIN_4,  { GPIOB_P4_F5_FLASH_MOSI,  GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_5,  { GPIOB_P5_F5_FLASH_MISO,  GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_7,  { GPIOB_P7_F5_FLASH_CLK,   GPIO_DRIVING_LEVEL_3, GPIO_PULL_NONE } },
	{ GPIO_PORT_B, GPIO_PIN_6,  { GPIOB_P6_F5_FLASH_CS,   GPIO_DRIVING_LEVEL_3, GPIO_PULL_UP } },
	{ GPIO_PORT_B, GPIO_PIN_2,  { GPIOB_P2_F5_FLASH_WP,   GPIO_DRIVING_LEVEL_3, GPIO_PULL_UP } },
	{ GPIO_PORT_B, GPIO_PIN_3,  { GPIOB_P3_F5_FLASH_HOLD, GPIO_DRIVING_LEVEL_3, GPIO_PULL_UP } },
};


static GPIO_PinMuxParam g_pinmux_pwm_ch0 =
/*   port         pin            mode                    driving               pull          */
{ GPIO_PORT_A, GPIO_PIN_8,  { GPIOA_P8_F3_PWM0_ECT0,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } };
static GPIO_PinMuxParam g_pinmux_pwm_ch1 =
/*   port         pin            mode                    driving               pull          */
{ GPIO_PORT_A, GPIO_PIN_9,  { GPIOA_P9_F3_PWM1_ECT1,  GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } };
static GPIO_PinMuxParam g_pinmux_pwm_ch2 =
/*   port         pin            mode                    driving               pull          */
{ GPIO_PORT_A, GPIO_PIN_10, { GPIOA_P10_F3_PWM2_ECT2, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } };
static GPIO_PinMuxParam g_pinmux_pwm_ch3 =
/*   port         pin            mode                    driving               pull          */
{ GPIO_PORT_A, GPIO_PIN_11, { GPIOA_P11_F3_PWM3_ECT3, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } };
static GPIO_PinMuxParam g_pinmux_pwm_ch4 =
/*   port         pin            mode                    driving               pull          */
{ GPIO_PORT_A, GPIO_PIN_12, { GPIOA_P12_F3_PWM4_ECT4, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } };
static GPIO_PinMuxParam g_pinmux_pwm_ch5 =
/*   port         pin            mode                    driving               pull          */
{ GPIO_PORT_A, GPIO_PIN_13, { GPIOA_P13_F3_PWM5_ECT5, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } };
static GPIO_PinMuxParam g_pinmux_pwm_ch6 =
/*   port         pin            mode                    driving               pull          */
{ GPIO_PORT_A, GPIO_PIN_14, { GPIOA_P14_F3_PWM6_ECT6, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } };
static GPIO_PinMuxParam g_pinmux_pwm_ch7 =
/*   port         pin            mode                    driving               pull          */
{ GPIO_PORT_A, GPIO_PIN_15, { GPIOA_P15_F3_PWM7_ECT7, GPIO_DRIVING_LEVEL_1, GPIO_PULL_NONE } };

static GPIO_PinMuxParam *g_pinmux_pwm[] = {
	&g_pinmux_pwm_ch0,
	&g_pinmux_pwm_ch1,
	&g_pinmux_pwm_ch2,
	&g_pinmux_pwm_ch3,
	&g_pinmux_pwm_ch4,
	&g_pinmux_pwm_ch5,
	&g_pinmux_pwm_ch6,
	&g_pinmux_pwm_ch7,
};

static GPIO_PinMuxParam g_pinmux_sd0[] = {
/*	  port         pin            mode                    driving               pull          */
	{ GPIO_PORT_A, GPIO_PIN_0,  { GPIOA_P0_F3_SD_CMD,     GPIO_DRIVING_LEVEL_2, GPIO_PULL_UP } },	/* CMD */
	{ GPIO_PORT_A, GPIO_PIN_2,  { GPIOA_P2_F3_SD_CLK,     GPIO_DRIVING_LEVEL_2, GPIO_PULL_UP } },	/* CLK */
	{ GPIO_PORT_A, GPIO_PIN_1,  { GPIOA_P1_F3_SD_DATA0,   GPIO_DRIVING_LEVEL_2, GPIO_PULL_UP } },	/* D0 */
	{ GPIO_PORT_A, GPIO_PIN_3,  { GPIOA_P3_F3_SD_DATA1,   GPIO_DRIVING_LEVEL_2, GPIO_PULL_UP } },	/* D1 */
	{ GPIO_PORT_A, GPIO_PIN_4,  { GPIOA_P4_F3_SD_DATA2,   GPIO_DRIVING_LEVEL_2, GPIO_PULL_UP } },	/* D2 */
	{ GPIO_PORT_A, GPIO_PIN_5,  { GPIOA_P5_F3_SD_DATA3,   GPIO_DRIVING_LEVEL_2, GPIO_PULL_NONE } },	/* D3 */
};

#define FLASH_SPI_ID SPI0
#define FLASH_CS_ID SPI_TCTRL_SS_SEL_SS0

HAL_Status board_uart_cfg(uint32_t uart_id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;

	if (uart_id == UART0_ID) {
		pinmux = g_pinmux_uart0;
		count = HAL_ARRAY_SIZE(g_pinmux_uart0);
	} else if (uart_id == UART1_ID) {
		pinmux = g_pinmux_uart1;
		count = HAL_ARRAY_SIZE(g_pinmux_uart1);
	} else {
		return HAL_INVALID;
	}

	return board_pinmux_cfg(req, pinmux, count);
}

void board_uart_init(UART_ID uart_id)
{
	UART_InitParam uart_param;

	uart_param.boardCfg = board_uart_cfg;
	uart_param.baudRate = 115200;
	uart_param.parity = UART_PARITY_NONE;
	uart_param.stopBits = UART_STOP_BITS_1;
	uart_param.dataBits = UART_DATA_BITS_8;
	uart_param.isAutoHwFlowCtrl = 0;

	HAL_UART_Init(uart_id, &uart_param);
}

void board_uart_deinit(UART_ID uart_id)
{
	HAL_UART_DeInit(uart_id);
}

int board_uart_write(UART_ID uart_id, char *buf, int count)
{
	return HAL_UART_Transmit_Poll(uart_id, (uint8_t *)buf, count);
}

int board_uart_getc(UART_ID uart_id)
{
	UART_T *uart;

	uart = HAL_UART_GetInstance(uart_id);
	if (HAL_UART_IsRxReady(uart)) {
		return (int)HAL_UART_GetRxData(uart);
	} else {
		return -1;
	}
}

HAL_Status board_irrx_cfg(uint32_t id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;

	pinmux = g_pinmux_irrx;
	count = HAL_ARRAY_SIZE(g_pinmux_irrx);

	return board_pinmux_cfg(req, pinmux, count);
}

HAL_Status board_spi_cfg(uint32_t spi_id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;
	GPIO_PinMuxParam *cspin;

	if (spi_id == (uint32_t)SPI0) {
		pinmux = g_pinmux_spi0;
		count = HAL_ARRAY_SIZE(g_pinmux_spi0);
		cspin = g_pinmux_spi0_cs0;
	} else if (spi_id == (uint32_t)SPI1) {
		pinmux = g_pinmux_spi1;
		count = HAL_ARRAY_SIZE(g_pinmux_spi1);
		switch (*(SPI_CS *)arg) {
		case SPI_TCTRL_SS_SEL_SS0:
			cspin = g_pinmux_spi1_cs0;
			break;
		case SPI_TCTRL_SS_SEL_SS1:
			cspin = g_pinmux_spi1_cs1;
			break;
		case SPI_TCTRL_SS_SEL_SS2:
			cspin = g_pinmux_spi1_cs2;
			break;
		default:
			return HAL_INVALID;
		}
	} else
		return HAL_INVALID;

	switch (req) {
	case HAL_BR_PINMUX_INIT:
		HAL_GPIO_PinMuxConfig(pinmux, count);
		HAL_GPIO_PinMuxConfig(cspin, 1);
		break;
	case HAL_BR_PINMUX_DEINIT:
		HAL_GPIO_PinMuxDeConfig(pinmux, count);
		HAL_GPIO_PinMuxDeConfig(cspin, 1);
		break;
	default:
		return HAL_INVALID;
	}

	return HAL_OK;
}

HAL_Status board_irtx_cfg(uint32_t id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;

	pinmux = g_pinmux_irtx;
	count = HAL_ARRAY_SIZE(g_pinmux_irtx);

	return board_pinmux_cfg(req, pinmux, count);
}

HAL_Status board_i2c_cfg(uint32_t i2c_id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;

	if (i2c_id == I2C0_ID) {
		pinmux = g_pinmux_i2c0;
		count = HAL_ARRAY_SIZE(g_pinmux_i2c0);
	} else if (i2c_id == I2C1_ID) {
		pinmux = g_pinmux_i2c1;
		count = HAL_ARRAY_SIZE(g_pinmux_i2c1);
	} else {
		return HAL_INVALID;
	}

	return board_pinmux_cfg(req, pinmux, count);
}

HAL_Status board_adc_cfg(uint32_t id, HAL_BoardReq req, void *chan)
{
	GPIO_PinMuxParam *pinmux;

	if ((ADC_Channel)chan == ADC_CHANNEL_8)
		return HAL_OK;

	pinmux = &g_pinmux_adc[(ADC_Channel)chan];

	return board_pinmux_cfg(req, pinmux, 1);
}

HAL_Status board_dmic_cfg(uint32_t id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;

	pinmux = g_pinmux_dmic;
	count = HAL_ARRAY_SIZE(g_pinmux_dmic);

	return board_pinmux_cfg(req, pinmux, count);
}

HAL_Status board_i2s_cfg(uint32_t id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;

	pinmux = g_pinmux_i2s;
	count = HAL_ARRAY_SIZE(g_pinmux_i2s);

	return board_pinmux_cfg(req, pinmux, count);
}

HAL_Status board_pwm_cfg(uint32_t pwm_id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;
	pinmux = g_pinmux_pwm[(PWM_CHID)arg];
	count = 1;

	return board_pinmux_cfg(req, pinmux, count);
}

HAL_Status board_spk_cfg(uint32_t id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;

	SPK_Param *argu = (SPK_Param *)arg;
	pinmux = g_pinmux_spk;
	count = HAL_ARRAY_SIZE(g_pinmux_spk);

	switch (req) {
	case HAL_BR_PINMUX_INIT:
		HAL_GPIO_PinMuxConfig(pinmux, count);
		break;
	case HAL_BR_PINMUX_DEINIT:
		HAL_GPIO_PinMuxDeConfig(pinmux, count);
		break;
	case HAL_BR_PINMUX_GETINFO:
		argu->pinmux = g_pinmux_spk;
		break;
	default:
		return HAL_INVALID;
	}

	return HAL_OK;
}

HAL_Status board_sdc_cfg(uint32_t id, HAL_BoardReq req, void *arg)
{
	HAL_SDCGPIOArg *arg_board = (HAL_SDCGPIOArg *)arg;

	switch (req) {
	case HAL_BR_PINMUX_INIT:
		HAL_GPIO_PinMuxConfig(arg_board->pinmux, arg_board->count);
		break;
	case HAL_BR_PINMUX_DEINIT:
		HAL_GPIO_PinMuxDeConfig(arg_board->pinmux, arg_board->count);
		break;
	case HAL_BR_PINMUX_GETINFO:
		if (SDCGPIO_BAS == arg_board->index) {
			arg_board->pinmux = g_pinmux_sd0;
			arg_board->count = HAL_ARRAY_SIZE(g_pinmux_sd0);
		} else if (SDCGPIO_DET == arg_board->index) {
			BOARD_ERR("%s not support gpio detect!\n", __func__);
			return HAL_ERROR;
		}
		break;
	default:
		return HAL_INVALID;
	}

	return HAL_OK;
}

int board_flash_init(SF_Handler *hdl)
{
	SPI_Port port = FLASH_SPI_ID;
	SF_Config flash_config;

	flash_config.spi = FLASH_SPI_ID;
	flash_config.cs = FLASH_CS_ID;
	flash_config.dma_en = 0;
	flash_config.sclk = BOARD_FLASH_CLK;
	if (HAL_SF_Init(hdl, &flash_config) != HAL_OK) {
		BOARD_ERR("%s %d flash init failed\n", __func__, __LINE__);
		HAL_SPI_Deinit(port);
		return -1;
	}
	return 0;
}

void board_flash_deinit(SF_Handler *hdl)
{
	if (hdl && *hdl) {
		HAL_SF_Deinit(hdl);
	}
}

HAL_Status board_flash_cfg(uint32_t id, HAL_BoardReq req, void *arg)
{
	GPIO_PinMuxParam *pinmux;
	uint32_t count;
	uint32_t *mode = arg;

	pinmux = g_pinmux_flash;
//	count = HAL_ARRAY_SIZE(g_pinmux_flash);
	if (*mode == 1)
		count = 6;
	else
		count = 4;

	return board_pinmux_cfg(req, pinmux, count);
}

HAL_Status board_spi_init(uint32_t spi_id)
{
	SPI_Global_Config spi_gconfig;
	SPI_Port port = spi_id;

	spi_gconfig.cb = board_spi_cfg;
	spi_gconfig.cs_level = 0;

	spi_gconfig.mclk = BOARD_FLASH_CLK;
	if (HAL_SPI_Init(port, &spi_gconfig) != HAL_OK) {
		BOARD_ERR("%s %d spi init failed\n", __func__, __LINE__);
		return -1;
	}

	return 0;
}

HAL_Status board_spi_deinit(uint32_t spi_id)
{
	return HAL_SPI_Deinit(spi_id);
}

HAL_Status board_init(void)
{
	HAL_Status ret;

	if ((ret = board_spi_init(FLASH_SPI_ID)) != HAL_OK)
		return ret;

#ifndef __CONFIG_BOOTLOADER
	SDC_InitTypeDef sdc_param;
#ifdef CONFIG_DETECT_CARD
	sdc_param.cd_mode = CARD_ALWAYS_PRESENT;
#endif
	sdc_param.boardCfgCb = board_sdc_cfg;
	HAL_SDC_Init(0, &sdc_param);

	if (BOARD_SUB_UART_ID != BOARD_MAIN_UART_ID) {
		board_uart_init(BOARD_SUB_UART_ID);
	}

	pm_mode_platform_select(PM_SUPPORT_SLEEP|PM_SUPPORT_STANDBY|PM_SUPPORT_POWEROFF);
#endif /* __CONFIG_BOOTLOADER */

	return HAL_OK;
}

HAL_Status board_deinit(void)
{
	HAL_Status ret;

	if ((ret = board_spi_deinit(FLASH_SPI_ID)) != HAL_OK)
		return ret;

#ifndef __CONFIG_BOOTLOADER
	if (BOARD_SUB_UART_ID != BOARD_MAIN_UART_ID) {
		board_uart_deinit(BOARD_SUB_UART_ID);
	}

	HAL_SDC_Deinit(0);
#endif /* __CONFIG_BOOTLOADER */

	return HAL_OK;
}

#ifdef __CONFIG_XIP_ENABLE
/* NB: temp implementation, don't put it here */
#include "driver/chip/hal_flashctrl.h"
#include "sys/image.h"

void board_xip_init(void)
{
	XIP_Config xip;
	xip.addr = IMAGE_APP_XIP_OFFSET + IH_SIZE;
	xip.freq = 64 * 1000 * 1000;
	xip.cb = board_flash_cfg;
	HAL_XIP_Init(&xip);
}
#endif /* __CONFIG_XIP_ENABLE */

#endif /* __CONFIG_BOARD_XRT738_EVB_01V10 */
