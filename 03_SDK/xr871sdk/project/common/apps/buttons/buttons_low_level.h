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

#ifndef _APPS_BUTTONS_LOW_LEVEL_H_
#define _APPS_BUTTONS_LOW_LEVEL_H_

#include "driver/chip/hal_adc.h"
#include "driver/chip/hal_gpio.h"

typedef struct {
	char *name;
	uint32_t mask;
	ADC_Channel channel;
	int value;
	int debounce_time;
} ad_button;

typedef struct {
	char *name;
	uint32_t mask;
	uint8_t active_low;
	GPIO_PinMuxParam gpio_param;
	int debounce_time;
} gpio_button;

typedef struct {
	ad_button *ad_buttons_p;
	uint8_t count;
} ad_button_info;

typedef struct {
	const gpio_button *gpio_buttons_p;
	uint8_t count;
} gpio_button_info;

int buttons_low_level_wait_semaphore(uint32_t waitms);
int buttons_low_level_release_semaphore(void);
int buttons_low_level_get_state(void);
int buttons_low_level_init(void);
int buttons_low_level_deinit(void);

#endif /* _APPS_BUTTONS_LOW_LEVEL_H_ */
