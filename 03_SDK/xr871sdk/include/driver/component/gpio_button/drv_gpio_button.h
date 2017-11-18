/**
  * @file  drv_gpio_button.h
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

#ifndef _GPIO_BUTTON_H_
#define _GPIO_BUTTON_H_

#include "driver/component/component_def.h"
#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief The button difault status.
  */
typedef enum {
	LOW_LEVEL,
	HIGH_LEVEL,
	NULL_LEVEL,
} GPIO_BUTTON_DIFAULT_STA;

/**
  * @brief The button status.
  */
typedef enum {
	BUTTON_PRESS,
	BUTTON_RELEASE,
	BUTTON_NORMAL,
} GPIO_BUTTON_STA;

/**
  * @brief Init or deinit button.
  */
typedef enum {
	BUTTON_INIT,
	BUTTON_DEINIT,
} GPIO_Board_Button_Req;

/**
  * @brief  Gpio button callback set and irq mode set.
  */
typedef struct {
	void (*buttonCallBack) (void *arg, GPIO_IrqEvent edge);
	void *arg;
} GPIO_Button_Irq;

/**
  * @brief Pins init.
  */
typedef struct {
	GPIO_Port button_Port;
	GPIO_Pin button_Pin;
	GPIO_BUTTON_DIFAULT_STA default_Potential;
} GPIO_Button_IO;

Component_Status DRV_Board_GPIO_Button_Cfg(GPIO_Board_Button_Req req, GPIO_Button_IO *button_reg_ruff,  uint32_t reg_buff_len);
GPIO_BUTTON_STA DRV_GPIO_ButtonStatus(uint32_t button_id);
void DRV_GPIO_ButtonCallBackRegister(uint32_t button_id, GPIO_Button_Irq *irq);

#ifdef __cplusplus
}
#endif
#endif /* _DRIVER_CHIP_HAL_PRCM_H_ */
