/**
  * @file  drv_matrix_led.c
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

#include "stdio.h"
#include "kernel/os/os.h"
#include "driver/chip/hal_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/component/matrix_led/drv_matrix_led.h"

#define MATRIX_LED_DBG 0

#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)


#define DRV_MATRIX_LED_DBG(fmt, arg...)	\
			LOG(MATRIX_LED_DBG, "[MATRIX LED] "fmt, ##arg)

typedef struct {
	GPIO_Port matrix_Port;
	GPIO_Pin matrix_Pin;
	GPIO_PullType pull;
}Matrix_Led_IO;

#define MATRIX_LIST_NUM 6
#define MATRIX_ROW_NUM 5

#if (defined(__CONFIG_CHIP_XRT871))
Matrix_Led_IO Matrx_List[MATRIX_LIST_NUM] = {
	{GPIO_PORT_A, GPIO_PIN_22, GPIO_PULL_NONE},
	{GPIO_PORT_B, GPIO_PIN_1, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_11, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_6, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_7, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_19, GPIO_PULL_NONE},
};

Matrix_Led_IO Matrx_Row[MATRIX_ROW_NUM] = {
	{GPIO_PORT_A, GPIO_PIN_3, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_4, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_5, GPIO_PULL_NONE},
	{GPIO_PORT_B, GPIO_PIN_13, GPIO_PULL_NONE},
	{GPIO_PORT_B, GPIO_PIN_15, GPIO_PULL_NONE},
};
#else
Matrix_Led_IO Matrx_List[MATRIX_LIST_NUM] = {
	{GPIO_PORT_B, GPIO_PIN_3, GPIO_PULL_NONE},
	{GPIO_PORT_B, GPIO_PIN_1, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_11, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_6, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_7, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_19, GPIO_PULL_NONE},
};

Matrix_Led_IO Matrx_Row[MATRIX_ROW_NUM] = {
	{GPIO_PORT_A, GPIO_PIN_3, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_4, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_5, GPIO_PULL_NONE},
	{GPIO_PORT_A, GPIO_PIN_22, GPIO_PULL_NONE},
	{GPIO_PORT_B, GPIO_PIN_2, GPIO_PULL_NONE},
};
#endif

void Matrix_Io_Init(Matrix_Led_IO *pin)
{
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.pull = pin->pull;
	param.mode = GPIOx_Pn_F1_OUTPUT;
	HAL_GPIO_Init(pin->matrix_Port, pin->matrix_Pin, &param);
}

void Matrix_Io_DeInit(Matrix_Led_IO *pin)
{
	HAL_GPIO_DeInit(pin->matrix_Port, pin->matrix_Pin);
}

typedef struct {
	uint16_t list;
	uint16_t row;
}List_Row_Data;

List_Row_Data Matrix_Data_Analysis(MAXTRIX_LED_DATA_FORMAT format, uint16_t data)
{
	uint16_t list = 0;
	uint16_t row = 0;
	List_Row_Data matrix_data;
	if (format == LIST_DATA_IS_LOW){
		uint16_t temp = (1 << MATRIX_LIST_NUM)- 1;
		list = data & temp;
		temp = (1 << (MATRIX_LIST_NUM + MATRIX_ROW_NUM)) - 1;
		DRV_MATRIX_LED_DBG("%s() %d, temp %d\n", __func__, __LINE__, temp);
		row = (data & temp ) >> MATRIX_LIST_NUM;
	} else if (format == LIST_DATA_IS_HIGH) {
		uint16_t temp = (1 << MATRIX_ROW_NUM) - 1;
		row = data & temp;
		temp = (1 << (MATRIX_LIST_NUM + MATRIX_ROW_NUM)) - 1;
		DRV_MATRIX_LED_DBG("%s() %d, temp %d\n", __func__, __LINE__, temp);
		list = (data & temp) >> MATRIX_ROW_NUM;
	}
	DRV_MATRIX_LED_DBG("%s(), list %d, row %d\n", __func__, list, row);
	matrix_data.list = list;
	matrix_data.row = row;
	return matrix_data;
}

void Matrix_List_Output(uint16_t list_data)
{
	int i = 0;
	uint16_t temp = 0;
	Matrix_Led_IO *io = Matrx_List;
	for (i = 0; i < MATRIX_LIST_NUM; i ++) {
		temp = list_data & (1 << i);
		if (temp > 0) {
			DRV_MATRIX_LED_DBG("%s(), list is 1\n", __func__);
			HAL_GPIO_WritePin(io->matrix_Port, io->matrix_Pin, GPIO_PIN_HIGH);
		}else {
			DRV_MATRIX_LED_DBG("%s(), list is 0\n", __func__);
			HAL_GPIO_WritePin(io->matrix_Port, io->matrix_Pin, GPIO_PIN_LOW);
		}
		io++;
	}
}

void Matrix_Row_Output(uint16_t row_data)
{
	int i = 0;
	uint16_t temp = 0;
	Matrix_Led_IO *io = Matrx_Row;
	for (i = 0; i < MATRIX_ROW_NUM; i ++) {
		temp = row_data & (1 << i);
		if (temp > 0) {
			DRV_MATRIX_LED_DBG("%s(), row is 1\n", __func__);
			HAL_GPIO_WritePin(io->matrix_Port, io->matrix_Pin, GPIO_PIN_HIGH);
		} else {
			DRV_MATRIX_LED_DBG("%s(), row is 0\n", __func__);
			HAL_GPIO_WritePin(io->matrix_Port, io->matrix_Pin, GPIO_PIN_LOW);
		}
		io++;
	}
}

/**
  * @brief Init matrix led pins.
  * @retval None.
  */
void DRV_Matrix_Init()
{
	int i = 0;
	for (i = 0; i < MATRIX_LIST_NUM; i ++) {
		Matrix_Led_IO *p = &Matrx_List[i];
		Matrix_Io_Init(p);
	}

	for (i = 0; i < MATRIX_ROW_NUM; i ++) {
		Matrix_Led_IO *p = &Matrx_Row[i];
		Matrix_Io_Init(p);
	}
	COMPONENT_TRACK("%s(), %d end\n", __func__, __LINE__);
}

/**
  * @brief Deinit matrix led pins.
  * @retval None.
  */
void DRV_Matrix_DeInit()
{
	int i = 0;
	for (i = 0; i < MATRIX_LIST_NUM; i ++) {
		Matrix_Led_IO *p = &Matrx_List[i];
		Matrix_Io_DeInit(p);
	}

	for (i = 0; i < MATRIX_ROW_NUM; i ++) {
		Matrix_Led_IO *p = &Matrx_Row[i];
		Matrix_Io_DeInit(p);
	}
}

/**
  * @brief Clear matrix led display.
  * @retval None.
  */
void DRV_Matrix_Clear()
{
	Matrix_List_Output(0);
	Matrix_Row_Output(0);
}

/**
  * @brief Matrix led display.
  * @param format: display data format.
  * @param data: display data.
  * @retval None.
  */
void DRV_Matrix_Display(MAXTRIX_LED_DATA_FORMAT format, uint16_t data)
{
	List_Row_Data matrix_data = Matrix_Data_Analysis(format, data);
	Matrix_List_Output(matrix_data.list);
	Matrix_Row_Output(matrix_data.row);
}

uint16_t matrix_data = 0x7c1;

void Matrix_Test()
{
	DRV_Matrix_Init();
	DRV_Matrix_Display(LIST_DATA_IS_HIGH, matrix_data);
}
