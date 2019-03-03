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

#ifndef _REVERB_PCM_H_
#define _REVERB_PCM_H_

#include "audio/pcm/audio_pcm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void * ReverbPcmCtrl;

#define STREAM_DEFAULT_SR       (16000)
#define STREAM_DEFAULT_CHL      (1)

typedef enum {
	REVERB_CMD_SET_BLOCK,
} ReverbPcmCmd;

ReverbPcmCtrl reverb_pcm_create(void);
void reverb_pcm_destroy(ReverbPcmCtrl rpc);
int reverb_pcm_open(ReverbPcmCtrl rpc);
int reverb_pcm_close(ReverbPcmCtrl rpc);
int reverb_pcm_write(ReverbPcmCtrl rpc, struct pcm_config *config, void *buffer, unsigned int size);
int reverb_pcm_read(ReverbPcmCtrl rpc, struct pcm_config *config, void *buffer, unsigned int size);
int reverb_pcm_flush(ReverbPcmCtrl rpc);
int reverb_pcm_ioctl(ReverbPcmCtrl rpc, ReverbPcmCmd cmd, void *param);

#ifdef __cplusplus
}
#endif

#endif /* _AUDIO_STREAM_MGR_H_ */
