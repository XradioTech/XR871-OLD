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
#include "common/framework/fs_ctrl.h"
#include "common/apps/recorder_app.h"
#include "common/framework/platform_init.h"

static void record_demo(void)
{
	recorder_base *recorder;
	rec_cfg cfg;

	if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
		printf("mount fail\n");
		return;
	}

	recorder = recorder_create();
	if (recorder == NULL) {
		printf("recorder create fail, exit\n");
		return;
	}

	/* record a 15s amr media */
	cfg.type = XRECODER_AUDIO_ENCODE_AMR_TYPE;
	printf("start record amr now, last for 15s\n");
	recorder->start(recorder, "file://record/1.amr", &cfg);
	OS_Sleep(15);
	recorder->stop(recorder);
	printf("record amr over.\n");

	/* record a 15s pcm media */
	cfg.type = XRECODER_AUDIO_ENCODE_PCM_TYPE;
	cfg.sample_rate = 8000;
	cfg.chan_num = 1;
	cfg.bitrate = 12200;
	cfg.sampler_bits = 16;
	printf("start record pcm now, last for 15s\n");
	recorder->start(recorder, "file://record/1.pcm", &cfg);
	OS_Sleep(15);
	recorder->stop(recorder);
	printf("record pcm over.\n");

	recorder_destroy(recorder);
}

int main(void)
{
	platform_init();

	printf("record demo start.\n");

	record_demo();

	printf("record demo over.\n");
	return 0;
}
