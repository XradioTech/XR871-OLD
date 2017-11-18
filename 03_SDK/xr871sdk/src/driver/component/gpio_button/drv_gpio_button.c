/**
  * @file  drv_gpio_button.c
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
#include "stdlib.h"
#include "kernel/os/os.h"
#include "driver/chip/hal_def.h"
#include "driver/component/gpio_button/drv_gpio_button.h"

#define GPIO_BUTTON_DBG 1

#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)


#define DRV_GPIO_BUTTON_DBG(fmt, arg...)	\
			//LOG(GPIO_BUTTON_DBG, "[GPIO BUTTON] "fmt, ##arg)

#define IO_EINTA 6U


static GPIO_Button_Irq *Button_Irq = NULL;
static GPIO_Button_IO *Button_Reg = NULL;
static uint8_t *Button_Irq_Edge = NULL;
static uint32_t Button_Num = 0;

static void GPIO_Button_Cb(void *arg)
{
	uint32_t id = (uint32_t)arg;
	if (id >= Button_Num)
		return;
	if (Button_Irq_Edge[id] == GPIO_IRQ_EVT_FALLING_EDGE) {
		Button_Irq[id].buttonCallBack(Button_Irq[id].arg, GPIO_IRQ_EVT_FALLING_EDGE);
		Button_Irq_Edge[id] = GPIO_IRQ_EVT_RISING_EDGE;
	} else {
		Button_Irq[id].buttonCallBack(Button_Irq[id].arg, GPIO_IRQ_EVT_RISING_EDGE);
		Button_Irq_Edge[id] = GPIO_IRQ_EVT_FALLING_EDGE;
	}

	GPIO_IrqParam Irq_param;
	Irq_param.event = Button_Irq_Edge[id];
	Irq_param.callback = GPIO_Button_Cb;
	Irq_param.arg = (void *)id;
	GPIO_Button_IO button = Button_Reg[id];
	HAL_GPIO_EnableIRQ(button.button_Port,
					   button.button_Pin, &Irq_param);
}

static Component_Status Enable_GPIO_Button(uint32_t id, GPIO_BUTTON_DIFAULT_STA default_Potential)
{
	if (id >= Button_Num)
		return COMP_ERROR;

	GPIO_IrqParam Irq_param;
	if(default_Potential == LOW_LEVEL)
		Irq_param.event = GPIO_IRQ_EVT_RISING_EDGE;
	else if(default_Potential == HIGH_LEVEL)
		Irq_param.event = GPIO_IRQ_EVT_FALLING_EDGE;

	Irq_param.callback = GPIO_Button_Cb;
	Irq_param.arg = (void *)id;
	GPIO_Button_IO button = Button_Reg[id];
	HAL_GPIO_EnableIRQ(button.button_Port,
					   button.button_Pin, &Irq_param);
	return COMP_OK;
}

static Component_Status GPIO_ButtonInit(uint32_t id)
{
	DRV_GPIO_BUTTON_DBG("%s() id %d\n", __func__, id);
	if (id >= Button_Num)
		return COMP_ERROR;
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	GPIO_Button_IO button = Button_Reg[id];
	if (button.default_Potential == HIGH_LEVEL) {
		param.pull = GPIO_PULL_UP;
		Button_Irq_Edge[id] = GPIO_IRQ_EVT_FALLING_EDGE;
	} else {
		param.pull = GPIO_PULL_DOWN;
		Button_Irq_Edge[id] = GPIO_IRQ_EVT_RISING_EDGE;
	}
	param.mode = IO_EINTA;
	HAL_GPIO_Init(button.button_Port,
				  button.button_Pin, &param);

	Enable_GPIO_Button(id, button.default_Potential);
	return COMP_OK;
}

static Component_Status Disable_GPIO_Button(uint32_t id)
{
	if (id >= Button_Num)
		return COMP_ERROR;
	GPIO_Button_IO button = Button_Reg[id];
	HAL_GPIO_DisableIRQ(button.button_Port,
					    button.button_Pin);
	return COMP_OK;
}

static Component_Status DeInit_GPIO_Button(uint32_t id)
{
	if (id >= Button_Num)
		return COMP_ERROR;
	Disable_GPIO_Button(id);
	GPIO_Button_IO button = Button_Reg[id];
	HAL_GPIO_DeInit(button.button_Port,
					button.button_Pin);

	return COMP_OK;
}

/**
  * @brief Init or deinit gpio button pins.
  * @param req: ctrl init or deinit pins.
  * @param button_reg_ruff: pins list.
  * @param reg_buff_len: buff len.
  * @retval Component_Status: The status of driver.
  */
Component_Status DRV_Board_GPIO_Button_Cfg(GPIO_Board_Button_Req req,
												    GPIO_Button_IO *button_reg_ruff, uint32_t reg_buff_len)
{
	if (req == BUTTON_INIT) {
		Button_Irq = malloc(sizeof(GPIO_Button_Irq) * reg_buff_len);
		Button_Irq_Edge = malloc(reg_buff_len);
		Button_Reg = button_reg_ruff;
		Button_Num = reg_buff_len;
	}

	int i = 0;
	for (i = 0; i < Button_Num; i++) {
		if (req == BUTTON_INIT)
			GPIO_ButtonInit(i);
		else
			DeInit_GPIO_Button(i);
	}

	OS_MSleep(2);

	if (req == BUTTON_DEINIT) {
		free(Button_Irq);
		Button_Irq = NULL;
		free(Button_Irq_Edge);
		Button_Irq_Edge = NULL;
		Button_Reg = NULL;
		Button_Num = 0;
	}

	return COMP_OK;
}

/**
  * @brief register the callback for gpio button.
  * @note This function is used to rigister the callback,when you push the button
  *           and the button is config, the callback will be run.
  * @param button_id: button id.
  * @param irq: Config callback.
  * @retval None
  */
void DRV_GPIO_ButtonCallBackRegister(uint32_t button_id, GPIO_Button_Irq *irq)
{
	 Button_Irq [button_id].buttonCallBack = irq->buttonCallBack;
	 Button_Irq [button_id].arg = irq->arg;
}

/**
  * @brief Read the button status.
  * @note This function is used to read the button status.
  * @param id: button id.
  * @retval GPIO_BUTTON_STA: button status, BUTTON_PRESS,BUTTON_RELEASE,
	BUTTON_NORMAL.
  */
GPIO_BUTTON_STA DRV_GPIO_ButtonStatus(uint32_t id)
{
	if (id >= Button_Num)
		return NULL_LEVEL;
	GPIO_Button_IO button = Button_Reg[id];
	GPIO_PinState value =  HAL_GPIO_ReadPin
							(button.button_Port,
							 button.button_Pin);
	if (value == GPIO_PIN_HIGH) {
		if (button.default_Potential == HIGH_LEVEL)
			return BUTTON_RELEASE;
		else
			return BUTTON_PRESS;
	} else {
		if (button.default_Potential == LOW_LEVEL)
			return BUTTON_RELEASE;
		else
			return BUTTON_PRESS;
	}
}
