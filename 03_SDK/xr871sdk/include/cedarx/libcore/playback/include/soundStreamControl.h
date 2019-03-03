/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : soundStreamControl.h
 * Description :
 * History :
 *
 */

#ifndef SOUND_STREAM_CONTROL_H
#define SOUND_STREAM_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void * SoundStreamCtrl;

typedef enum {
	STREAM_TYPE_SOUND_CARD   = 0,
	STREAM_TYPE_REVERB_PCM   = 1,
	STREAM_TYPE_MAX_VALUE    = 2,
} SoundStreamType;

typedef enum {
	STREAM_CMD_SET_CONFIG,
	STREAM_CMD_SET_BLOCK_MODE,
} SoundStreamCmd;

struct SscPcmConfig {
	unsigned int  channels;
	unsigned int  rate;
};

SoundStreamCtrl snd_stream_create(SoundStreamType type);
void snd_stream_destroy(SoundStreamCtrl ssc, SoundStreamType type);
int snd_stream_control(SoundStreamCtrl ssc, SoundStreamType type, SoundStreamCmd cmd, void *param);
int snd_stream_open(SoundStreamCtrl ssc, SoundStreamType type);
int snd_stream_close(SoundStreamCtrl ssc, SoundStreamType type);
int snd_stream_flush(SoundStreamCtrl ssc, SoundStreamType type);
int snd_stream_write(SoundStreamCtrl ssc, SoundStreamType type, void* pData, int nDataSize);
int snd_stream_read(SoundStreamCtrl ssc, SoundStreamType type, void* pData, int nDataSize);

#ifdef __cplusplus
}
#endif

#endif