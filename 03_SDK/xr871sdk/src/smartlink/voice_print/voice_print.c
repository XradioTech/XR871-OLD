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
#include <stdlib.h>
#include <fs/fatfs/ff.h>

#include "kernel/os/os.h"

//#include "audio_player.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"
#include "driver/chip/hal_gpio.h"

//#define _MEASURE_TIME

#ifdef _MEASURE_TIME
#include "driver/chip/hal_rtc.h"
#endif
#include "sys/xr_debug.h"

#include "smartlink/sc_assistant.h"
#include "smartlink/voice_print/voice_print.h"
#include "adt.h"

#define TYPE 1

#if TYPE == 1
#define SAMPLE_RATE 32000
#define SAMPLE_TYPE FREQ_TYPE_MIDDLE
#elif TYPE == 2
#define SAMPLE_RATE 44100
#define SAMPLE_TYPE FREQ_TYPE_HIGH
#else
#define SAMPLE_RATE 16000
#define SAMPLE_TYPE FREQ_TYPE_LOW
#endif

#define SOUND_CARD_ID_IN AUDIO_CARD0
#define PCM_BUF_SIZE (1024 * 4)

#define VP_ACK_TIME_OUT_MS 3000

enum loglevel {
	OFF = 0,
	ERROR = 1,
	INFO = 2,
};

#define g_debuglevel ERROR

#define VP_DBG(level, fmt, args...) \
	do {		\
		if (level <= g_debuglevel)	\
			printf("[VP]"fmt,##args);	\
	} while (0)

#define VP_HEX_DUMP(level, addr, len) \
	do {		\
		if (level <= g_debuglevel)	\
			print_hex_dump_bytes(addr, len);	\
	} while (0)

#define VP_BUG_ON(v) do {if(v) {printf("BUG at %s:%d!\n", __func__, __LINE__); \
		      while (1);}} while (0)

#define VP_TASK_RUN     (1 << 0)
#define VP_TASK_STOP    (1 << 1)

typedef struct vp_priv {
	struct netif *nif;
	wlan_voiceprint_result_t result;
	voiceprint_status_t status;
	uint32_t decoder_bitsize;
	uint32_t waiting;
	uint8_t ref;
	void *handle;
	struct pcm_config config;
	uint8_t pcm_data_buf[PCM_BUF_SIZE];
	uint8_t result_str[200];
} vp_priv_t;

static vp_priv_t *voice_print = NULL;

int voice_print_start(struct netif *nif, const char *key)
{
	int ret = 0;
	vp_priv_t *priv = voice_print;
	config_decoder_t decode_config;

	VP_BUG_ON(!nif);

	if (priv) {
		VP_DBG(ERROR, "%s has already started!\n", __func__);
		return -1;
	}

	priv = malloc(sizeof(vp_priv_t));
	if (!priv) {
		VP_DBG(ERROR, "%s malloc failed!\n", __func__);
		return -1;
	}
	memset(priv, 0, sizeof(vp_priv_t));

	decode_config.error_correct = 0;
	decode_config.error_correct_num = 0;
	decode_config.freq_type = SAMPLE_TYPE;
	decode_config.group_symbol_num = 10;
	decode_config.max_strlen = 256;
	decode_config.sample_rate = SAMPLE_RATE;

	priv->handle = decoder_create(&decode_config);
	if (priv->handle == NULL) {
		VP_DBG(ERROR, "decoder_create error\n");
		return -1;
	}

	priv->decoder_bitsize = decoder_getbsize(priv->handle);

	VP_DBG(INFO, "decoder gitsize:%u\n", priv->decoder_bitsize);
	if (PCM_BUF_SIZE % (priv->decoder_bitsize * 2)) {
		VP_DBG(INFO, "pcm recoder buffer size(%d) is not aligned to %d\n",
		       PCM_BUF_SIZE, priv->decoder_bitsize);
	}

	priv->config.channels = 1;
	priv->config.format = PCM_FORMAT_S16_LE;
	priv->config.period_count = 2;
	priv->config.period_size = 1024 * 2;
	priv->config.rate = SAMPLE_RATE;
	priv->nif = nif;

	voice_print = priv;

	return ret;
}

voiceprint_status_t voice_print_wait_once(void)
{
	int ret;
	int i = 0;
	voiceprint_status_t status = VP_STATUS_DEC_ERROR;
	vp_priv_t *priv = voice_print;
	uint32_t bufsize;
	int finish = 0;

	if (!priv)
		return VP_STATUS_DEC_ERROR;

	status = priv->status;

	if (priv->waiting & VP_TASK_STOP) {
		finish = 1;
		VP_DBG(ERROR, "voiceprint has stoped wt:%x\n", priv->waiting);
		goto out;
	}

	if (priv->ref == 0) {
		OS_ThreadSuspendScheduler();
		priv->waiting |= VP_TASK_RUN;
		OS_ThreadResumeScheduler();
		VP_DBG(INFO, "recoder start\n");
		/* NOTE: time between open and read should not too long. */
		ret = snd_pcm_open(&priv->config, SOUND_CARD_ID_IN, PCM_IN);
		if (ret != 0) {
			finish = 1;
			status = VP_STATUS_DEC_ERROR;
			VP_DBG(ERROR, "pcm open error\n");
			goto out;
		}
		priv->ref++;
	}
	ret = snd_pcm_read(&priv->config, SOUND_CARD_ID_IN, priv->pcm_data_buf,
	                   PCM_BUF_SIZE);
	if (ret == -1) {
		finish = 1;
		status = VP_STATUS_DEC_ERROR;
		VP_DBG(ERROR, "pcm read error\n");
		goto out;
	}

	bufsize = priv->decoder_bitsize * 2;

	for (i = 0; i < (PCM_BUF_SIZE / bufsize); i++) {
#ifdef _MEASURE_TIME
		int start_time, stop_time;
		start_time = HAL_RTC_GetFreeRunTime();
#endif
		ret = decoder_fedpcm(priv->handle,
		                     (short *)&priv->pcm_data_buf[i * bufsize]);
#ifdef _MEASURE_TIME
		stop_time = HAL_RTC_GetFreeRunTime();
		if (i % 100 == 0)
			VP_DBG(INFO, ":%d\n", stop_time - start_time);
#endif

		switch (ret) {
		case RET_DEC_ERROR:
			VP_DBG(ERROR, "decoder error\n");
			status = VP_STATUS_DEC_ERROR;
			finish = 1;
			goto out;
		case RET_DEC_NORMAL:
			status = VP_STATUS_NORMAL;
			break;
		case RET_DEC_NOTREADY:
			VP_DBG(INFO, "wait to decoder\n");
			status = VP_STATUS_NOTREADY;
			break;
		case RET_DEC_END:
			status = VP_STATUS_COMPLETE;
			VP_DBG(INFO, "decoder end\n");
			ret = decoder_getstr(priv->handle, priv->result_str);
			if (ret == RET_DEC_NOTREADY) {
				VP_DBG(ERROR, "decoder result error\n");
				status = VP_STATUS_NOTREADY;
			} else {
				int len = strlen((char *)priv->result_str);
				VP_DBG(INFO, "result:%s, len:%u\n", priv->result_str, len);
				VP_HEX_DUMP(INFO, priv->result_str, len);
				decoder_reset(priv->handle);
				sc_assistant_newstatus(SCA_STATUS_COMPLETE, NULL, NULL);
				finish = 1;
				goto out;
			}
			break;
		default:
			VP_DBG(ERROR, "ret is invalid\n");
			break;
		}
	}

out:
	if (finish || (priv->waiting & VP_TASK_STOP)) {
		if (priv->ref > 0) {
			snd_pcm_close(SOUND_CARD_ID_IN, PCM_IN);
			VP_DBG(INFO, "%s,%d get data end\n", __func__, __LINE__);
			priv->ref--;
		}
		OS_ThreadSuspendScheduler();
		priv->waiting &= ~VP_TASK_RUN;
		OS_ThreadResumeScheduler();
	}
	priv->status = status;

	return status;
}

voiceprint_ret_t voice_print_wait(uint32_t timeout_ms)
{
	uint32_t end_time;
	vp_priv_t *priv = voice_print;
	voiceprint_status_t status = WLAN_VOICEPRINT_FAIL;

	if (!priv)
		return WLAN_VOICEPRINT_FAIL;

	OS_ThreadSuspendScheduler();
	priv->waiting |= VP_TASK_RUN;
	OS_ThreadResumeScheduler();
	end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + timeout_ms;

	while (!(priv->waiting & VP_TASK_STOP) &&
	       OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {

		if (sc_assistant_get_status() >= SCA_STATUS_CHANNEL_LOCKED) {
			VP_DBG(INFO, "voice wait end, because sc_assistant locked!\n");
			break;
		}

		status = voice_print_wait_once();
		if (status == VP_STATUS_DEC_ERROR || status == VP_STATUS_COMPLETE)
			break;
	}

	OS_ThreadSuspendScheduler();
	priv->waiting = 0;
	OS_ThreadResumeScheduler();

	if (status == VP_STATUS_COMPLETE && priv->result_str[0] != 0) {
		VP_DBG(INFO, "%s,%d recoader end\n", __func__, __LINE__);
		return WLAN_VOICEPRINT_SUCCESS;
	}

	VP_DBG(INFO, "%s,%d recoader faild\n", __func__, __LINE__);
	return WLAN_VOICEPRINT_FAIL;
}

voiceprint_status_t voiceprint_get_status(void)
{
	vp_priv_t *priv = voice_print;

	if (!priv)
		return VP_STATUS_DEC_ERROR;

	return priv->status;
}

voiceprint_ret_t wlan_voiceprint_get_raw_result(char *buf_result, int *len)
{
	voiceprint_status_t ret;
	vp_priv_t *priv = voice_print;
	int len_result;

	if (!priv) {
		VP_DBG(INFO, "voiceprint has exit\n");
		return WLAN_VOICEPRINT_FAIL;
	}

	ret = voiceprint_get_status();
	if (ret != VP_STATUS_COMPLETE) {
		VP_DBG(ERROR, "voiceprint is not complete\n");
		return WLAN_VOICEPRINT_FAIL;
	}

	if (priv->result_str[0] == '\0') {
		*len = 0;
		VP_DBG(ERROR, "voiceprint invalid result\n");
		return WLAN_VOICEPRINT_FAIL;
	}

	len_result = strlen((char *)priv->result_str);
	if (len_result >= *len) {
		VP_DBG(ERROR, "voiceprint buffer overflow\n");
		return WLAN_VOICEPRINT_OVERFLOW;
	}

	*len = len_result;
	memcpy(buf_result, (char *)priv->result_str, len_result);
	return WLAN_VOICEPRINT_SUCCESS;
}

voiceprint_ret_t voiceprint_ack_start(vp_priv_t *priv, uint32_t random_num,
                                      uint32_t timeout_ms)
{
	//TODO: ack
	voiceprint_ret_t ret = WLAN_VOICEPRINT_SUCCESS;

	return ret;
}

voiceprint_ret_t
wlan_voiceprint_connect_ack(struct netif *nif, uint32_t timeout_ms,
                            wlan_voiceprint_result_t *result)
{
	VP_DBG(ERROR, "Do not deal with SSID and PSK string here!\n");
	return WLAN_VOICEPRINT_FAIL;
}

int voice_print_stop(uint32_t wait)
{
	vp_priv_t *priv = voice_print;
	void *decoder_handle;

	if (!priv) {
		VP_DBG(INFO, "%s has already stoped!\n", __func__);
		return -1;
	}

	voice_print = NULL;

	sc_assistant_stop_connect_ap();

	if (wait) {
		OS_ThreadSuspendScheduler();
		priv->waiting |= VP_TASK_STOP;
		OS_ThreadResumeScheduler();

		while (priv->waiting & VP_TASK_RUN) {
			OS_MSleep(10);
		}
	}

	decoder_handle = priv->handle;
	priv->handle = NULL;
	decoder_destroy(decoder_handle);

	if (priv->ref) {
		VP_DBG(INFO, "%s,%d close pcm\n", __func__, __LINE__);
		snd_pcm_close(SOUND_CARD_ID_IN, PCM_IN);
		priv->ref--;
	}

	free(priv);

	VP_DBG(INFO, "%s voice stoped!\n", __func__);

	return 0;
}
