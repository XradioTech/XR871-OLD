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

#include "kernel/os/os.h"
#include "audio/pcm/audio_pcm.h"
#include "audio/manager/audio_manager.h"
#include "smartlink/sc_assistant.h"
#include "smartlink/voice_print/voice_print.h"

#include "adt.h"


#define VP_DBG_ON       0
#define VP_INF_ON       0
#define VP_WRN_ON       0
#define VP_ERR_ON       1
#define VP_ABORT_ON     0

#define VP_DBG_TIME_ON  1

#define VP_SYSLOG       printf
#define VP_ABORT()      do { } while (0)

#define VP_LOG(flags, fmt, arg...) \
    do {                           \
        if (flags)                 \
            VP_SYSLOG(fmt, ##arg); \
    } while (0)

#define VP_DBG(fmt, arg...) VP_LOG(VP_DBG_ON, "[VP D] "fmt, ##arg)
#define VP_INF(fmt, arg...) VP_LOG(VP_INF_ON, "[VP I] "fmt, ##arg)
#define VP_WRN(fmt, arg...) VP_LOG(VP_WRN_ON, "[VP W] "fmt, ##arg)
#define VP_ERR(fmt, arg...)                            \
    do {                                               \
        VP_LOG(VP_ERR_ON, "[VP E] %s():%d, "fmt,       \
               __func__, __LINE__, ##arg);             \
        if (VP_ABORT_ON)                               \
            VP_ABORT();                                \
    } while (0)


#define VP_TYPE 1

#if VP_TYPE == 1
#define VP_SAMPLE_RATE 32000
#define VP_SAMPLE_TYPE FREQ_TYPE_MIDDLE
#elif VP_TYPE == 2
#define VP_SAMPLE_RATE 44100
#define VP_SAMPLE_TYPE FREQ_TYPE_HIGH
#else
#define VP_SAMPLE_RATE 16000
#define VP_SAMPLE_TYPE FREQ_TYPE_LOW
#endif

#if (VOICE_PRINT_POLICY == 1)
/* group symbol number for decoder, MUST be the same as encoder */
#define VP_GROUP_SYMBOL_NUM     10

/* decoded result length */
#define VP_DEC_RESULT_MAX_LEN   101 /* 2 + 32 + 2 + 63 + 2*/
#define VP_DEC_RESULT_MIN_LEN   6

/* max decoded result number */
#define VP_DEC_RESULT_MAX_NUM   0x1

#elif (VOICE_PRINT_POLICY == 2)
/* group symbol number for decoder, MUST be the same as encoder */
#define VP_GROUP_SYMBOL_NUM     13

/* decoded result length: [4, 13] */
#define VP_DEC_RESULT_MAX_LEN   13
#define VP_DEC_RESULT_MIN_LEN   4

/* max decoded result number */
#define VP_DEC_RESULT_MAX_NUM   0x10
#endif /* VOICE_PRINT_POLICY */

/* buffer size for reading pcm data, unit is 2-byte */
#define VP_PCM_BUF_SIZE         (1024 * 2)

#define VP_TASK_RUN     (1 << 0)
#define VP_TASK_STOP    (1 << 1)

typedef struct vp_priv {
	struct netif *nif;
	voiceprint_status_t status;
	uint8_t audio_card;
	uint8_t ref;
	uint32_t dec_input_size; /* decode input data size, unit is 2-byte */
	uint32_t waiting;
	void *handle;
	struct pcm_config config;
#if (VP_DBG_ON && VP_DBG_TIME_ON)
	uint32_t start_tm;
#endif

	int16_t pcm_buf[VP_PCM_BUF_SIZE];

	uint8_t data[VP_DEC_RESULT_MAX_LEN + 1]; /* tmp for data process */
//	uint8_t data[VP_DEC_RESULT_MAX_LEN * 2]; /* tmp for data process */
	uint8_t data_len;

	uint8_t result_total;
	uint8_t result_cnt;

	uint8_t result_len[VP_DEC_RESULT_MAX_NUM];
	uint8_t result[VP_DEC_RESULT_MAX_NUM][VP_DEC_RESULT_MAX_LEN];
} vp_priv_t;

static vp_priv_t *voice_print = NULL;

int voiceprint_start(voiceprint_param_t *param)
{
	vp_priv_t *priv;
	config_decoder_t decode_config;

	VP_DBG("%s()\n", __func__);

	if (param == NULL || param->nif == NULL) {
		VP_WRN("invalid param\n");
		return -1;
	}

	if (voice_print != NULL) {
		VP_WRN("vp started!\n");
		return -1;
	}

	priv = malloc(sizeof(vp_priv_t));
	if (priv == NULL) {
		VP_ERR("no mem\n");
		return -1;
	}
	memset(priv, 0, sizeof(vp_priv_t));

	decode_config.max_strlen = VP_DEC_RESULT_MAX_LEN;
	decode_config.sample_rate = VP_SAMPLE_RATE;
	decode_config.freq_type = VP_SAMPLE_TYPE;
	decode_config.group_symbol_num = VP_GROUP_SYMBOL_NUM;
	decode_config.error_correct = 0;
	decode_config.error_correct_num = 0;
	priv->handle = decoder_create(&decode_config);
	if (priv->handle == NULL) {
		VP_ERR("decoder_create() fail\n");
		goto err;
	}

	priv->dec_input_size = decoder_getbsize(priv->handle);
	VP_DBG("dec input size %u\n", priv->dec_input_size);

	if (VP_PCM_BUF_SIZE % priv->dec_input_size) {
		VP_ERR("pcm buf size %d not aligned to %d\n", VP_PCM_BUF_SIZE,
		       priv->dec_input_size);
		goto err;
	}

	priv->config.channels = 1;
	priv->config.rate = VP_SAMPLE_RATE;
	priv->config.period_size = VP_PCM_BUF_SIZE;
	priv->config.period_count = 2;
	priv->config.format = PCM_FORMAT_S16_LE;

	priv->nif = param->nif;
	priv->audio_card = param->audio_card;
	priv->status = VP_STATUS_NORMAL;
	voice_print = priv;

	return 0;

err:
	if (priv->handle != NULL) {
		decoder_destroy(priv->handle);
	}

	if (priv) {
		free(priv);
	}

	return -1;
}

static int hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

#if (VOICE_PRINT_POLICY == 1)

static int8_t vp_process_result(vp_priv_t *priv)
{
	int len, tmp, i;
	char *data;
	uint8_t chksum;

	data = (char *)priv->data;
	len = priv->data_len;
	VP_DBG("data (%d): %s\n", len, data);

	if (len < VP_DEC_RESULT_MIN_LEN || len > VP_DEC_RESULT_MAX_LEN) {
		VP_DBG("invalid str len %d\n", len);
		return -1;
	}

	/* get checksum (last 2 byte) */
	tmp = hex2num(data[--len]);
	if (tmp < 0) {
		VP_DBG("invalid chksum\n");
		return -1;
	}
	chksum = (uint8_t)tmp;

	tmp = hex2num(data[--len]);
	if (tmp < 0) {
		VP_DBG("invalid chksum\n");
		return -1;
	}
	chksum += (uint8_t)tmp << 4;

	/* do checksum */
	tmp = 0;
	for (i = 0; i < len; ++i) {
		tmp += data[i];
	}
	if (((tmp + chksum) & 0xff) != 0xff) {
		VP_DBG("chksum err, (0x%02x + 0x%02x) != 0xff\n", tmp, chksum);
		return -1;
	}

	VP_DBG("get all result: %s\n", data);
	priv->result_len[0] = len + 2;
	memcpy(priv->result[0], data, priv->result_len[0]);
	priv->result_total = 1;
	return 0;
}

#elif (VOICE_PRINT_POLICY == 2)

static int8_t vp_process_result(vp_priv_t *priv)
{
	int len, tmp, i, idx;
	char *data;
	uint8_t chksum;

	data = (char *)priv->data;
	len = priv->data_len;
	VP_DBG("data (%d): %s\n", len, data);

	if (len < VP_DEC_RESULT_MIN_LEN || len > VP_DEC_RESULT_MAX_LEN) {
		VP_DBG("invalid str len %d\n", len);
		return -1;
	}

	/* get checksum (last 2 byte) */
	tmp = hex2num(data[--len]);
	if (tmp < 0) {
		VP_DBG("invalid chksum\n");
		return -1;
	}
	chksum = (uint8_t)tmp;

	tmp = hex2num(data[--len]);
	if (tmp < 0) {
		VP_DBG("invalid chksum\n");
		return -1;
	}
	chksum += (uint8_t)tmp << 4;

	/* do checksum */
	tmp = 0;
	for (i = 0; i < len; ++i) {
		tmp += data[i];
	}
	if (((tmp + chksum) & 0xff) != 0xff) {
		VP_DBG("chksum err, (0x%02x + 0x%02x) != 0xff\n", tmp, chksum);
		return -1;
	}

	VP_DBG("result idx %c (%d)\n", *data, priv->result_total);
	idx = hex2num(*data);
	if (idx < 0 || idx >= VP_DEC_RESULT_MAX_NUM) {
		return -1;
	}

	if (priv->result_len[idx] != 0) {
		return -1; /* get this result already, skip it */
	}

	++data; --len;
	if (idx == 0) {
		VP_DBG("result max idx %c\n", *data);
		++data; --len; /* skip version, useless now */
		tmp = hex2num(*data);
		if (tmp < 0 || tmp >= VP_DEC_RESULT_MAX_NUM) {
			return -1;
		}
		priv->result_total = tmp + 1;

		++data; --len;
		memcpy(priv->result[idx], data, len);
		priv->result_len[idx] = len;

		for (i = 0; i < priv->result_total; ++i) {
			if (priv->result_len[i] != 0) {
				++priv->result_cnt;
			}
		}
	} else {
		memcpy(priv->result[idx], data, len);
		priv->result_len[idx] = len;
		if (priv->result_total > 0) {
			++priv->result_cnt;
		}
	}

	VP_DBG("result cnt %d (%d)\n", priv->result_cnt, priv->result_total);
	if (priv->result_total > 0 && priv->result_cnt == priv->result_total) {
#if VP_DBG_ON
		VP_DBG("get all result: ");
		for (i = 0; i < priv->result_total; ++i) {
			VP_LOG(1, "%s", priv->result[i]);
		}
		VP_LOG(1, "\n");
#endif
		return 0;
	}

	return -1;
}

#endif /* VOICE_PRINT_POLICY */

voiceprint_status_t voiceprint_wait_once(void)
{
	int ret, i, finish;
	voiceprint_status_t status;
	vp_priv_t *priv;
	uint32_t input_size; /* unit is 2-byte*/

	if (voice_print == NULL)
		return VP_STATUS_DEC_ERROR;

	priv = voice_print;
	finish = 0;
	status = VP_STATUS_NORMAL;

	if (priv->waiting & VP_TASK_STOP) {
		finish = 1;
		VP_DBG("vp stopped, wait: %x\n", priv->waiting);
		goto out;
	}

	if (priv->ref == 0) {
		OS_ThreadSuspendScheduler();
		priv->waiting |= VP_TASK_RUN;
		OS_ThreadResumeScheduler();
		VP_DBG("rec start\n");
#if (VP_DBG_ON && VP_DBG_TIME_ON)
		priv->start_tm = OS_TicksToMSecs(OS_GetTicks());
#endif
		/* NOTE: time between open and read should not too long. */
		ret = snd_pcm_open(&priv->config, priv->audio_card, PCM_IN);
		if (ret != 0) {
			finish = 1;
			status = VP_STATUS_DEC_ERROR;
			VP_ERR("snd open fail\n");
			goto out;
		}
		priv->ref++;
	}
	ret = snd_pcm_read(&priv->config, priv->audio_card, priv->pcm_buf,
	                   sizeof(priv->pcm_buf));
	if (ret != sizeof(priv->pcm_buf)) {
		finish = 1;
		status = VP_STATUS_DEC_ERROR;
		VP_ERR("pcm read %d, err %d\n", sizeof(priv->pcm_buf), ret);
		goto out;
	}

	input_size = priv->dec_input_size;
	for (i = 0; i < VP_PCM_BUF_SIZE / input_size; i++) {
		ret = decoder_fedpcm(priv->handle, &priv->pcm_buf[i * input_size]);
		switch (ret) {
		case RET_DEC_ERROR:
			VP_DBG("decoder error\n");
			decoder_reset(priv->handle);
			break;
		case RET_DEC_NORMAL:
			break;
		case RET_DEC_NOTREADY:
			VP_DBG("decoder not ready\n");
			break;
		case RET_DEC_END:
			VP_DBG("decoder end\n");
			ret = decoder_getstr(priv->handle, priv->data);
			if (ret == RET_DEC_NOTREADY) {
				VP_DBG("decoder result not ready\n");
			} else {
				decoder_reset(priv->handle);

				/* process result */
				priv->data_len = strlen((char *)priv->data);
				ret = vp_process_result(priv);
				if (ret == 0) {
					finish = 1;
					status = VP_STATUS_COMPLETE;
#if (VP_DBG_ON && VP_DBG_TIME_ON)
					VP_DBG("decode cost %u ms\n",
					       OS_TicksToMSecs(OS_GetTicks()) - priv->start_tm);
#endif
					sc_assistant_newstatus(SCA_STATUS_COMPLETE, NULL, NULL);
					goto out;
				}
			}
			break;
		default:
			break;
		}
	}

out:
	if (finish || (priv->waiting & VP_TASK_STOP)) {
		if (priv->ref > 0) {
			snd_pcm_close(priv->audio_card, PCM_IN);
			VP_DBG("%s() get data end\n", __func__);
			priv->ref--;
		}
		OS_ThreadSuspendScheduler();
		priv->waiting &= ~VP_TASK_RUN;
		OS_ThreadResumeScheduler();
	}
	priv->status = status;

	return status;
}

voiceprint_ret_t voiceprint_wait(uint32_t timeout_ms)
{
	uint32_t end_time;
	vp_priv_t *priv = voice_print;
	voiceprint_status_t status = WLAN_VOICEPRINT_FAIL;

	if (!priv)
		return WLAN_VOICEPRINT_FAIL;

	OS_ThreadSuspendScheduler();
	priv->waiting |= VP_TASK_RUN;
	OS_ThreadResumeScheduler();
	end_time = OS_TicksToMSecs(OS_GetTicks()) + timeout_ms;

	while (!(priv->waiting & VP_TASK_STOP) &&
	       OS_TimeBefore(OS_TicksToMSecs(OS_GetTicks()), end_time)) {

		if (sc_assistant_get_status() >= SCA_STATUS_CHANNEL_LOCKED) {
			VP_DBG("voice wait end, because sc_assistant locked!\n");
			break;
		}

		status = voiceprint_wait_once();
		if (status == VP_STATUS_DEC_ERROR || status == VP_STATUS_COMPLETE)
			break;
	}

	OS_ThreadSuspendScheduler();
	priv->waiting = 0;
	OS_ThreadResumeScheduler();

	if (status == VP_STATUS_COMPLETE) {
		VP_DBG("dec ok\n");
		return WLAN_VOICEPRINT_SUCCESS;
	}

	VP_DBG("dec fail\n");
	return WLAN_VOICEPRINT_FAIL;
}

voiceprint_status_t voiceprint_get_status(void)
{
	vp_priv_t *priv = voice_print;

	if (!priv)
		return VP_STATUS_DEC_ERROR;

	return priv->status;
}

int voiceprint_stop(uint32_t wait)
{
	vp_priv_t *priv = voice_print;
	void *decoder_handle;

	VP_DBG("%s()\n", __func__);

	if (!priv) {
		return 0;
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
		VP_DBG("%s() close pcm\n", __func__);
		snd_pcm_close(priv->audio_card, PCM_IN);
		priv->ref--;
	}

	free(priv);
	VP_DBG("vp stopped!\n");

	return 0;
}

voiceprint_ret_t wlan_voiceprint_get_raw_result(char *result, int *len)
{
	voiceprint_status_t ret;
	vp_priv_t *priv = voice_print;
	int i, total_len;
	char *ptr;

	if (!result || !len || !priv) {
		return WLAN_VOICEPRINT_FAIL;
	}

	ret = voiceprint_get_status();
	if (ret != VP_STATUS_COMPLETE) {
		VP_DBG("voiceprint is not complete\n");
		return WLAN_VOICEPRINT_FAIL;
	}

	total_len = 0;
	for (i = 0; i < priv->result_total; ++i) {
		total_len += priv->result_len[i];
	}

	if (total_len == 0) {
		VP_DBG("result len 0\n");
		return WLAN_VOICEPRINT_FAIL;
	}

	if (total_len > *len) {
		VP_WRN("result buf %d < %d\n", *len, total_len);
		return WLAN_VOICEPRINT_OVERFLOW;
	}

	ptr = result;
	for (i = 0; i < priv->result_total; ++i) {
		memcpy(ptr, priv->result[i], priv->result_len[i]);
		ptr += priv->result_len[i];
	}
	*len = total_len;
	VP_DBG("raw result (%d): %.*s\n", total_len, total_len, result);

	return WLAN_VOICEPRINT_SUCCESS;
}

voiceprint_ret_t voiceprint_ack_start(vp_priv_t *priv, uint32_t random_num,
                                      uint32_t timeout_ms)
{
	return WLAN_VOICEPRINT_FAIL;
}

voiceprint_ret_t
wlan_voiceprint_connect_ack(struct netif *nif, uint32_t timeout_ms,
                            wlan_voiceprint_result_t *result)
{
	return WLAN_VOICEPRINT_FAIL;
}

