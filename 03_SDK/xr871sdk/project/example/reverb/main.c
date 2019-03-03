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
#include "audio/manager/audio_manager.h"
#include "audio/reverb/mixer.h"
#include "soundStreamControl.h"
#include "common/framework/fs_ctrl.h"
#include "common/framework/platform_init.h"
#include "bgm.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/reverb/reverb.h"

#define KAROK_THREAD_STACK_SIZE    (2 * 1024)

#define MICROPHONE_GPIO_PORT      GPIO_PORT_A
#define MICROPHONE_GPIO_PIN       GPIO_PIN_17

#define AUDIO_CARD                AUDIO_CARD0
#define AUDIO_SAMPLERATE          (16000)
#define AUDIO_CHL                 (1)

static struct pcm_config au_config = {
	.channels       = AUDIO_CHL,
	.format         = PCM_FORMAT_S16_LE,
	.period_count   = 2,
	.period_size    = 1024,
	.rate           = AUDIO_SAMPLERATE,
	.mix_mode       = 0
};

static SoundStreamCtrl ssc;

typedef struct {
	uint8_t      prv_state;
	uint8_t      run;
	OS_Timer_t   timer;
	OS_Thread_t  thread;
} karaok;

static karaok gKaraok;

static int karaok_open()
{
	int block = 1;

	background_music_pause();

	if (snd_pcm_open(&au_config, AUDIO_CARD, PCM_OUT) != 0) {
		printf("sound card open err\n");
		return -1;
	}

	snd_stream_open(ssc, STREAM_TYPE_REVERB_PCM);
	snd_stream_control(ssc, STREAM_TYPE_REVERB_PCM, STREAM_CMD_SET_BLOCK_MODE, &block);

	background_music_resume();

	if (snd_pcm_open(&au_config, AUDIO_CARD, PCM_IN) != 0) {
		printf("sound card open err\n");
		return -1;
	}

	return 0;
}

static void karaok_close()
{
	int block = 0;

	snd_stream_control(ssc, STREAM_TYPE_REVERB_PCM, STREAM_CMD_SET_BLOCK_MODE, &block);

	background_music_pause();

	snd_pcm_close(AUDIO_CARD, PCM_OUT);
	snd_stream_close(ssc, STREAM_TYPE_REVERB_PCM);

	background_music_resume();

	snd_pcm_close(AUDIO_CARD, PCM_IN);
}

static void karaok_task()
{
	int ret;
	float d_time;
	short *record_buf = NULL;
	unsigned char *bgm_buf = NULL;
	unsigned int pcm_read_size;
	reverb_info rev_info;

	ret = karaok_open();
	if (ret != 0) {
		printf("karaok open fail.\n");
		goto err0;
	}

	d_time = 0.25f;
	rev_info.BitsPerSample = 16;
	rev_info.SampleRate = AUDIO_SAMPLERATE;
	rev_info.feed_ratio = 3;
	rev_info.NumChannels = AUDIO_CHL;
	rev_info.delay_sample = (int)(d_time * rev_info.SampleRate * rev_info.NumChannels);
	ret = reverb_init(&rev_info);
	if (ret != 0) {
		printf("reverb init fail.\n");
		goto err1;
	}

	pcm_read_size = (au_config.channels) * 2 *(au_config.period_size);
	record_buf = malloc(pcm_read_size);
	if (record_buf == NULL) {
		printf("malloc fail\n");
		goto err2;
	}
	memset(record_buf, 0, pcm_read_size);

	bgm_buf = (unsigned char *)malloc(pcm_read_size);
	if (bgm_buf == NULL) {
		printf("malloc fail\n");
		goto err2;
	}

	while (gKaraok.run) {
		/* get record data, and create reverd data by record data */
		snd_pcm_read(&au_config, AUDIO_CARD, record_buf, pcm_read_size);
		reverb_comb(&rev_info, record_buf, pcm_read_size);

		/* get bgm data */
		memset(bgm_buf, 0, pcm_read_size);
		ret = snd_stream_read(ssc, STREAM_TYPE_REVERB_PCM, bgm_buf, pcm_read_size);

		/* mix reverb data and bgm data */
		if (ret != 0) {
			char *src_data[2];
			src_data[0] = (char*)record_buf;
			src_data[1] = (char*)bgm_buf;
			mixer_process(src_data, 2, (char*)record_buf, pcm_read_size);
		}

		/* write to sound card */
		snd_pcm_write(&au_config, AUDIO_CARD, record_buf, pcm_read_size);
	}

err2:
	free(bgm_buf);
	free(record_buf);
	reverb_release(&rev_info);
err1:
	karaok_close();
err0:
	OS_ThreadDelete(&gKaraok.thread);
}

static int karaok_thread_create()
{
	if (OS_ThreadIsValid(&gKaraok.thread)) {
		printf("task is still running");
		return -1;
	}

	if (OS_ThreadCreate(&gKaraok.thread,
                        "karaok",
                        karaok_task,
                        NULL,
                        OS_THREAD_PRIO_APP,
                        KAROK_THREAD_STACK_SIZE) != OS_OK) {
		printf("thread create fail.\n");
		return -1;
	}
	return 0;
}

static void karaok_timer_cb(void *arg)
{
	uint8_t level = HAL_GPIO_ReadPin(MICROPHONE_GPIO_PORT, MICROPHONE_GPIO_PIN);

	if (gKaraok.prv_state == level)
		return;

	gKaraok.prv_state = level;
	if(level == 1) {
		printf("microphone remove...\n");
		gKaraok.run = 0;
	} else {
		printf("microphone insert...\n");
		gKaraok.run = 1;
		karaok_thread_create();
	}
}

static void microphone_det_cb(void* arg)
{
	OS_TimerStart(&gKaraok.timer);
}

static int microphone_det_init()
{
	GPIO_InitParam param = {0};
	GPIO_IrqParam irq_param = {0};

	param.driving = GPIO_DRIVING_LEVEL_1;
	param.mode = GPIOx_Pn_F6_EINT;
	param.pull = GPIO_PULL_UP;
	HAL_GPIO_Init(MICROPHONE_GPIO_PORT, MICROPHONE_GPIO_PIN, &param);

	irq_param.event = GPIO_IRQ_EVT_BOTH_EDGE;
	irq_param.callback = microphone_det_cb;
	irq_param.arg = NULL;
	HAL_GPIO_EnableIRQ(MICROPHONE_GPIO_PORT, MICROPHONE_GPIO_PIN, &irq_param);

	return 0;
}

static void karaok_init()
{
	ssc = snd_stream_create(STREAM_TYPE_REVERB_PCM);
	OS_TimerCreate(&gKaraok.timer, OS_TIMER_ONCE, karaok_timer_cb, NULL, 300);
	microphone_det_init();
}

int main(void)
{
	platform_init();

	printf("reverb demo start.\n");

	if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("mount fail\n");
		return -1;
	}

	background_music_start();
	karaok_init();

	return 0;
}
