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

#include "smartlink/sc_assistant.h"
#include "smartlink/voice_print/voice_print.h"
#include "adt.h"

#define SOUND_CARD_ID_IN AUDIO_CARD0
#define PCM_BUF_SIZE (1024 * 4)

#define VP_ACK_TIME_OUT_MS 3000

enum loglevel {
	OFF = 0,
	ERROR = 1,
	INFO = 2,
};

#define g_debuglevel INFO

#define VP_DBG(level, fmt, args...) \
	do {		\
		if (level <= g_debuglevel)	\
			printf("[voice_print]"fmt,##args);	\
	} while (0)


#define VP_BUG_ON(v) do {if(v) {printf("BUG at %s:%d!\n", __func__, __LINE__); \
		      while (1);}} while (0)

#define VP_TASK_RUN     (1 << 0)
#define VP_TASK_STOP    (1 << 1)

#define DEC_BSIZE 256

typedef struct vp_priv {
	struct netif *nif;
	wlan_voiceprint_result_t result;
	voiceprint_status_t status;
	uint32_t waiting;
	void *handle;
	struct pcm_config config;
	uint8_t pcm_data_buf[PCM_BUF_SIZE];
	uint8_t str_buf[200];
	short dec_buf[DEC_BSIZE/2];
} vp_priv_t;

static vp_priv_t *voice_print = NULL;

int voice_print_start(struct netif *nif, const char *key)
{
	int ret;
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
	voice_print = priv;

	priv->config.channels = 1;
	priv->config.format = PCM_FORMAT_S16_LE;
	priv->config.period_count = 2;
	priv->config.period_size = 1024 * 2;
	priv->config.rate = 16000;
	priv->nif = nif;

	ret = snd_pcm_open(&priv->config, SOUND_CARD_ID_IN, PCM_IN);
	if (ret != 0) {
		VP_DBG(ERROR, "pcm open error\n");
		return -1;
	}

	VP_DBG(INFO, "recoader start\n");

	decode_config.error_correct = 0;
	decode_config.error_correct_num = 0;
	decode_config.freq_type = FREQ_TYPE_LOW;
	decode_config.group_symbol_num = 10;
	decode_config.max_strlen = 200;
	decode_config.sample_rate = 16000;

	priv->handle = decoder_create(&decode_config);
	if (priv->handle == NULL) {
		VP_DBG(ERROR, "decoder_create error\n");
		return -1;
	}

	ret = decoder_getbsize(priv->handle);

	VP_DBG(INFO, "decoder_getbsize bytes %u\n", 2 * ret);
	return ret;
}

voiceprint_ret_t voice_print_wait(uint32_t timeout_ms)
{
	voiceprint_ret_t ret = WLAN_VOICEPRINT_SUCCESS;
	int i = 0;
	int decode_times = 0;
	uint32_t end_time;
	vp_priv_t *priv = voice_print;

	if (!priv)
		return WLAN_VOICEPRINT_FAIL;

	priv->waiting |= VP_TASK_RUN;
	priv->str_buf[0] = 0;
	end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + timeout_ms; /*sc_assistant_get_status() < SCA_STATUS_CHANNEL_LOCKED && \*/

	while (!(priv->waiting & VP_TASK_STOP) && \
	       OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {

		if ( !(sc_assistant_get_status() < SCA_STATUS_CHANNEL_LOCKED) ) {
			printf("voice wait end, because sc_assistant!!!!!\n");
			break;
		}

		ret = snd_pcm_read(&priv->config, SOUND_CARD_ID_IN, priv->pcm_data_buf, PCM_BUF_SIZE);
		if (ret == -1)
			VP_DBG(ERROR, "pcm read error\n");

		for (i = 0; i < (PCM_BUF_SIZE / DEC_BSIZE); i++) {
			memcpy(priv->dec_buf, &priv->pcm_data_buf[i * DEC_BSIZE], DEC_BSIZE);
			int status = decoder_fedpcm(priv->handle, priv->dec_buf);

			switch (status) {
			case RET_DEC_ERROR:
				VP_DBG(ERROR, "decoder error\n");
				goto err_out;
			case RET_DEC_NORMAL:
				break;
			case RET_DEC_NOTREADY:
				VP_DBG(INFO, "wait to decoder\n");
				break;
			case RET_DEC_END:
				VP_DBG(INFO, "decoder end\n");
				ret = decoder_getstr(priv->handle, priv->str_buf);
                if (ret == RET_DEC_NOTREADY)
                	VP_DBG(ERROR, "decoder result error\n");
                else if (ret == 0) {
					VP_DBG(INFO, "str_buf :%s, len %u\n", priv->str_buf, strlen((char *)priv->str_buf));

					int i = 0;
					while(1) {
						if (g_debuglevel == INFO) {
							printf("%x ", priv->str_buf[i++]);
							if (i > strlen((char *)priv->str_buf)) {
								printf("\n");
								break;
							}
						}
						break;
					}

					VP_DBG(INFO, "%s, %d, get data +%d\n", __func__, __LINE__, decode_times);
                    decode_times ++;
                }

                if (decode_times < 3) {
					priv->str_buf[0] = 0;
                    decoder_reset(priv->handle);
				} else {
					sc_assistant_newstatus(SCA_STATUS_COMPLETE, NULL, NULL);
					priv->waiting = 0;
					VP_DBG(INFO, "%s, %d, voice get data end\n", __func__, __LINE__);
					goto out;
				}
				break;
			default:
				VP_DBG(ERROR, "ret is invalid\n");
				break;
			}
		}
	}

out:
	priv->waiting = 0;

	if (priv->str_buf[0] != 0) {
		snd_pcm_close(SOUND_CARD_ID_IN, PCM_IN);
		VP_DBG(INFO, "%s, %d, recoader end\n", __func__, __LINE__);
		return WLAN_VOICEPRINT_SUCCESS;
	}

err_out:
	priv->waiting = 0;
	snd_pcm_close(SOUND_CARD_ID_IN, PCM_IN);
	VP_DBG(INFO, "%s, %d, recoader end\n", __func__, __LINE__);
	return WLAN_VOICEPRINT_FAIL;
}

voiceprint_status_t wlan_voiceprint_get_result(wlan_voiceprint_result_t *result)
{
	voiceprint_status_t ret = RET_DEC_END;
	vp_priv_t *priv = voice_print;;
	uint32_t ssid_len = 0;

	if (!priv) {
		VP_DBG(INFO, "voiceprint has exit\n");
		return RET_DEC_ERROR;
	}

	sscanf((char *)priv->str_buf, "%u", &ssid_len);
	memcpy(result->ssid, &priv->str_buf[2], ssid_len);
	strcpy((char *)result->passphrase, (char *)&priv->str_buf[2 + ssid_len]);
	result->ssid_len = ssid_len;

	VP_DBG(INFO,"ssid_len = %u\n", ssid_len);
	VP_DBG(INFO, "SSID: %.*s\n", ssid_len, result->ssid);
	VP_DBG(INFO, "PSK: %s\n", result->passphrase);

	return ret;
}

voiceprint_ret_t voiceprint_ack_start(vp_priv_t *priv, uint32_t random_num,
                                      uint32_t timeout_ms)
{
	//TODO: ack
	voiceprint_ret_t ret = WLAN_VOICEPRINT_SUCCESS;

	return ret;
}

voiceprint_ret_t
wlan_voiceprint_connect_ack(struct netif *nif, uint32_t timeout_ms, wlan_voiceprint_result_t *result)
{
	voiceprint_ret_t ret = WLAN_VOICEPRINT_FAIL;
	uint8_t *psk;
	vp_priv_t *priv = voice_print;;

	if (!priv) {
		VP_DBG(ERROR, "voiceprint has exit\n");
		return WLAN_VOICEPRINT_FAIL;
	}

	if (priv->str_buf[0] == 0) {
		VP_DBG(ERROR, "voiceprint invalid result\n");
		return WLAN_VOICEPRINT_INVALID;
	}

	VP_DBG(INFO,"str_buf :%s\n", priv->str_buf);
	VP_DBG(INFO,"str_buf len :%u\n", strlen((char *)priv->str_buf));

	if (wlan_voiceprint_get_result(result) != RET_DEC_END) {
		return WLAN_VOICEPRINT_FAIL;
	}

	priv->nif = sc_assistant_open_sta();

	if (result->passphrase[0] != '\0') {
		psk = result->passphrase;
	} else {
		psk = NULL;
	}

	ret = sc_assistant_connect_ap(result->ssid, result->ssid_len, psk, timeout_ms);
	if (ret < 0) {
		VP_DBG(ERROR, " ap connect time out\n");
		return WLAN_VOICEPRINT_TIMEOUT;
	}

#if 0
	ret = voiceprint_ack_start(priv, result->random_num, VP_ACK_TIME_OUT_MS);
	if (ret < 0)
		VP_DBG(ERROR, "voice ack error, ap connect time out\n");
#endif
	return WLAN_VOICEPRINT_SUCCESS;
}

int voice_print_stop(void)
{
	vp_priv_t *priv = voice_print;

	if (!priv) {
		VP_DBG(ERROR, "%s has already stoped!\n", __func__);
		return -1;
	}

	priv->waiting |= VP_TASK_STOP;
	while (priv->waiting & VP_TASK_RUN) {
		OS_MSleep(10);
	}

	decoder_destroy(priv->handle);

	voice_print = NULL;

	free(priv);

	VP_DBG(INFO, "%s voice stoped!\n", __func__);

	return 0;
}
