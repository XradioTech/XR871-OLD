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
#ifndef BBC_MAIN_H
#define BBC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bbc_audio_upload{
	unsigned char audio_play_sta;
	unsigned char audio_play_vol;
}audio_upload;

typedef enum {
	AUD_CALL_BACK 		= 2,
	AUD_CALL_DI_BACK	= 3
} AUD_CALL_STATUS;

typedef enum {
	AUD_PLAY_PUSH 		= 0,
	AUD_PLAY_START		= 1
} AUD_PLAY_STATUS;

typedef struct AudioCallBack{
	unsigned char NONE_CALLBACK_FLAG;
	unsigned char STA_CALLBACK_FLAG;
	unsigned char VOL_CALLBACK_FLAG;
}Audio_DATA_CALLBACK;

extern audio_upload AudUpload;
extern Audio_DATA_CALLBACK AudioDataCall;

int bbc_audio_task_init();
extern void pub_audio_callback(void);

#ifdef __cplusplus
}
#endif 

#endif