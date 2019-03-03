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

#include "common/apps/buttons/buttons.h"
#include "common/apps/buttons/buttons_low_level.h"
#include "component_manage.h"

/* the buttons in board, need see board_config.c */
#define AK1 KEY0
#define AK2 KEY1
#define AK3 KEY2
#define DK1 KEY3
#define DK2 KEY4

static button_handle *ak1_button;
static button_handle *ak2_button;
static button_handle *ak3_button;
static button_handle *dk1_button;
static button_handle *dk2_button;

extern COMPONENT_CTRL sensor_button_cmd;

static void ak1_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS)
		sensor_button_cmd = COMPONENT_CTRL_INTO;
}

static void ak2_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS || sta == REPEAT_PRESS)
		sensor_button_cmd = COMPONENT_CTRL_DOWN;
}

static void ak3_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS || sta == REPEAT_PRESS)
		sensor_button_cmd = COMPONENT_CTRL_UP;
}

static void dk1_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS)
		sensor_button_cmd = COMPONENT_CTRL_SLEEP;
}

static void dk2_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS)
		sensor_button_cmd = COMPONENT_CTRL_BREAK;
}

int sensor_buttons_init(void)
{
	int ret;

	/* register the low level button interface */
	button_impl_t impl = {
		buttons_low_level_init,
		buttons_low_level_get_state,
		buttons_low_level_wait_semaphore,
		buttons_low_level_release_semaphore,
		buttons_low_level_deinit
	};
	/* set buttons thread priority and stack size */
	buttons_set_thread_priority(4);
	buttons_set_thread_stack_size(256);

	/* init buttons, will init the low level buttons, ad buttons and gpio buttons */
	ret = buttons_init(&impl);

	/* create buttons object, after a long press of 50ms, buttons will trigger */
	ak1_button = create_long_button(AK1, 50);
	ak2_button = create_repeat_long_button(AK2, 50, 200);
	ak3_button = create_repeat_long_button(AK3, 50, 200);
	dk1_button = create_long_button(DK1, 50);
	dk2_button = create_long_button(DK2, 50);

	/* set buttons callback */
	ak1_button->cb = ak1_button_cb;
	ak2_button->cb = ak2_button_cb;
	ak3_button->cb = ak3_button_cb;
	dk1_button->cb = dk1_button_cb;
	dk2_button->cb = dk2_button_cb;

	/* start buttons object */
	ak1_button->start(ak1_button);
	ak2_button->start(ak2_button);
	ak3_button->start(ak3_button);
	dk1_button->start(dk1_button);
	dk2_button->start(dk2_button);

	return ret;
}

void sensor_buttons_deinit(void)
{
	/* destroy buttons object */
	ak1_button->destroy(ak1_button);
	ak2_button->destroy(ak2_button);
	ak3_button->destroy(ak3_button);
	dk1_button->destroy(dk1_button);
	dk2_button->destroy(dk2_button);

	/* buttons deinit */
	buttons_deinit();
}

