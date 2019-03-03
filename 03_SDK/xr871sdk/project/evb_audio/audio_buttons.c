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
#include "audio_player.h"

/* the buttons in board, need see board_config.c */
#define AK1 KEY0
#define AK2 KEY1
#define AK3 KEY2
#define DK1 KEY3
#define DK2 KEY4

static button_handle *vol_up_button;
static button_handle *vol_down_button;
static button_handle *next_button;
static button_handle *prev_button;
static button_handle *pause_button;

extern PLAYER_CMD player_button_cmd;

static void vol_up_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS)
		player_button_cmd = CMD_PLAYER_VOLUME_UP;
}

static void vol_down_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS)
		player_button_cmd = CMD_PLAYER_VOLUME_DOWN;
}

static void next_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS)
		player_button_cmd = CMD_PLAYER_NEXT;
}

static void prev_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS)
		player_button_cmd = CMD_PLAYER_PERV;
}

static void pause_button_cb(BUTTON_STATE sta, void *arg)
{
	if (sta == PRESS)
		player_button_cmd = CMD_PLAYER_PAUSE;
}

int audio_buttons_init(void)
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
	vol_up_button = create_long_button(AK1, 50);
	vol_down_button = create_long_button(DK2, 50);
	next_button = create_long_button(AK3, 50);
	prev_button = create_long_button(AK2, 50);
	pause_button = create_long_button(DK1, 50);

	/* set buttons callback */
	vol_up_button->cb = vol_up_button_cb;
	vol_down_button->cb = vol_down_button_cb;
	next_button->cb = next_button_cb;
	prev_button->cb = prev_button_cb;
	pause_button->cb = pause_button_cb;

	/* start buttons object */
	vol_up_button->start(vol_up_button);
	vol_down_button->start(vol_down_button);
	next_button->start(next_button);
	prev_button->start(prev_button);
	pause_button->start(pause_button);

	return ret;
}

void audio_buttons_deinit(void)
{
	/* destroy buttons object */
	vol_up_button->destroy(vol_up_button);
	vol_down_button->destroy(vol_down_button);
	next_button->destroy(next_button);
	prev_button->destroy(prev_button);
	pause_button->destroy(pause_button);

	/* buttons deinit */
	buttons_deinit();
}

