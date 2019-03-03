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
#include <string.h>
#include "kernel/os/os.h"
#include "common/framework/fs_ctrl.h"
#include "common/apps/player_app.h"
#include "common/framework/platform_init.h"

#define PLAYER_THREAD_STACK_SIZE    (1024 * 2)

static player_base *bgm_player = NULL;
static OS_Thread_t player_thread;
static OS_Semaphore_t sem;

static void bgm_task_callback(player_events event, void *data, void *arg)
{
	switch (event) {
	case PLAYER_EVENTS_MEDIA_PREPARED:
		break;
	case PLAYER_EVENTS_MEDIA_STOPPED:
		break;
	case PLAYER_EVENTS_MEDIA_ERROR:
		printf("error occur\n");
		OS_SemaphoreRelease(&sem);
		break;
	case PLAYER_EVENTS_MEDIA_PLAYBACK_COMPLETE:
		printf("media play is complete\n");
		OS_SemaphoreRelease(&sem);
		break;
	default:
		break;
	}
}

static void bgm_play_task(void *arg)
{
	OS_Status ret;
	player_base *player;

	ret = OS_SemaphoreCreate(&sem, 0, OS_SEMAPHORE_MAX_COUNT);
	if (ret != OS_OK) {
		printf("sem create fail\n");
		return;
	}

	player = player_create();
	if (player == NULL) {
		printf("player create fail.\n");
		OS_SemaphoreDelete(&sem);
		return;
	}
	bgm_player = player;

	while (1) {
		player->set_callback(player, bgm_task_callback, NULL);
		player->play(player, "file://music/1.mp3");

		OS_SemaphoreWait(&sem, OS_WAIT_FOREVER);

		player->stop(player);
	}

	OS_SemaphoreDelete(&sem);
	player_destroy(player);

	OS_ThreadDelete(&player_thread);
}

int background_music_pause()
{
	if (bgm_player) {
		bgm_player->pause(bgm_player);
	}
	return 0;
}

int background_music_resume()
{
	if (bgm_player) {
		bgm_player->resume(bgm_player);
	}
	return 0;
}

int background_music_start()
{
	if (OS_ThreadCreate(&player_thread,
                        "player_task",
                        bgm_play_task,
                        NULL,
                        OS_THREAD_PRIO_APP,
                        PLAYER_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create fail.exit\n");
		return -1;
	}
	return 0;
}

