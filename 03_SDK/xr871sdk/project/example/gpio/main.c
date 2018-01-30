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
#include "kernel/os/os.h"
#include "driver/chip/hal_gpio.h"

#define LED_PORT GPIO_PORT_A
#define LED_PIN GPIO_PIN_6

#define BUTTON_1_PORT GPIO_PORT_A
#define BUTTON_1_PIN GPIO_PIN_19

#define BUTTON_2_PORT GPIO_PORT_A
#define BUTTON_2_PIN GPIO_PIN_20

void gpio_led_pin_init()
{
	/*PA0 output*/
	GPIO_InitParam param;
	/*set pin driver capability*/
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOx_Pn_F1_OUTPUT;
	param.pull = GPIO_PULL_NONE;
	HAL_GPIO_Init(LED_PORT, LED_PIN, &param);
}

void gpio_led_on()
{
	/*set pin output low level*/
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_LOW);
}

void gpio_led_off()
{
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_HIGH);
}

void gpio_button1_init()
{
	/*PA1 input mode*/
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOx_Pn_F0_INPUT;
	param.pull = GPIO_PULL_NONE;
	HAL_GPIO_Init(BUTTON_1_PORT, BUTTON_1_PIN, &param);
}

void gpio_button1_read()
{
	/*read pin*/
	GPIO_PinState sta = HAL_GPIO_ReadPin(BUTTON_1_PORT, BUTTON_1_PIN);
	if (sta == GPIO_PIN_LOW)
		printf("gpio button1 is low level.\n");
}

void gpio_button2_callback()
{
	printf("gpio button2 irq.\n");
}


void gpio_button2_irq_init()
{
	/*PA2 Interrupt mode*/
	GPIO_InitParam param;
	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOA_P2_F6_EINTA2;
	param.pull = GPIO_PULL_NONE;
	HAL_GPIO_Init(BUTTON_2_PORT, BUTTON_2_PIN, &param);

	/*delay some time, wait gpio config over*/
	OS_MSleep(1);

	/*gpio irq config*/
	GPIO_IrqParam irq_param;
	irq_param.arg = NULL;
	irq_param.callback = gpio_button2_callback;
	/*set pin irq low level trigger*/
	irq_param.event = GPIO_IRQ_EVT_LOW_LEVEL;
	HAL_GPIO_EnableIRQ(BUTTON_2_PORT, BUTTON_2_PIN, &irq_param);

}

/*Run this demo, please connect the sensor board.*/
int main(void)
{
	printf("gpio demo started.\n");
	printf("please press DK1_button or DK2_button.\n");
	printf("The led D1 will be flash.\n\n");

	gpio_led_pin_init();
	gpio_button1_init();
	gpio_button2_irq_init();

	static int i = 0;
	while (1) {
		if (i <= 100)
			gpio_led_on();
		else
			gpio_led_off();

		gpio_button1_read();

		i++;
		if(i > 200)
			i = 0;

		OS_MSleep(10);
	}

	/*deinit gpio*/
	/*PA0 deinit*/
	HAL_GPIO_DeInit(GPIO_PORT_A, GPIO_PIN_0);
	/*PA1 deinit*/
	HAL_GPIO_DeInit(GPIO_PORT_A, GPIO_PIN_1);
	/*PA2 deinit*/
	HAL_GPIO_DisableIRQ(GPIO_PORT_A, GPIO_PIN_2);
	HAL_GPIO_DeInit(GPIO_PORT_A, GPIO_PIN_2);

	printf("gpio demo over.\n");
	return 0;
}

