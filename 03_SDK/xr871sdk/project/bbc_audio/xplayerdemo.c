/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : xplayerdemo.c
 * Description : xplayerdemo in linux, H3-tv2next
 *               video write to DE, audio write with alsa
 * History :
 *
 */

#ifdef __PRJ_CONFIG_XPLAYER

//#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pthread.h"
//#include <ctype.h>
//#include "errno.h"
//#include <sys/select.h>

#include "iniparserapi.h"

//#include "cdx_config.h"
#include <cdx_log.h>
#include "xplayer.h"
#include "CdxTypes.h"
#include "fs/fatfs/ff.h"
//#include "memoryAdapter.h"
//#include "deinterlace.h"
//typedef unsigned long uintptr_t ;
extern SoundCtrl* SoundDeviceCreate();

static const int STATUS_STOPPED   = 0;
static const int STATUS_PREPARING = 1;
static const int STATUS_PREPARED  = 2;
static const int STATUS_PLAYING   = 3;
static const int STATUS_PAUSED    = 4;
static const int STATUS_SEEKING   = 5;

typedef struct DemoPlayerContext
{
    XPlayer*       mAwPlayer;
    int             mPreStatus;
    int             mStatus;
    int             mSeekable;
    int             mError;
    pthread_mutex_t mMutex;
    sem_t			mStoped;
	sem_t			mPrepared;
//    int             mVideoFrameNum;
}DemoPlayerContext;


//* define commands for user control.
typedef struct Command
{
    const char* strCommand;
    int         nCommandId;
    const char* strHelpMsg;
}Command;

#define COMMAND_HELP            0x1     //* show help message.
#define COMMAND_QUIT            0x2     //* quit this program.

#define COMMAND_SET_SOURCE      0x101   //* set url of media file.
#define COMMAND_PLAY            0x102   //* start playback.
#define COMMAND_PAUSE           0x103   //* pause the playback.
#define COMMAND_STOP            0x104   //* stop the playback.
#define COMMAND_SEEKTO          0x105   //* seek to posion, in unit of second.
#define COMMAND_SHOW_MEDIAINFO  0x106   //* show media information.
#define COMMAND_SHOW_DURATION   0x107   //* show media duration, in unit of second.
#define COMMAND_SHOW_POSITION   0x108   //* show current play position, in unit of second.
#define COMMAND_SWITCH_AUDIO    0x109   //* switch autio track.
#define COMMAND_SETSPEED        0x10a


void set_source(DemoPlayerContext *demoPlayer, char* pUrl)
{
    demoPlayer->mSeekable = 1;

    //* set url to the AwPlayer.
    if(XPlayerSetDataSourceUrl(demoPlayer->mAwPlayer,
                 (const char*)pUrl, NULL, NULL) != 0)
    {
        printf("error:\n");
        printf("    AwPlayer::setDataSource() return fail.\n");
        return;
    }
     printf("setDataSource end\n");

    //* start preparing.
    pthread_mutex_lock(&demoPlayer->mMutex);

	if (!strncmp(pUrl, "http://", 7)) {
	    if(XPlayerPrepareAsync(demoPlayer->mAwPlayer) != 0)
	    {
	        printf("error:\n");
	        printf("    AwPlayer::prepareAsync() return fail.\n");
	        pthread_mutex_unlock(&demoPlayer->mMutex);
	        return;
	    }
		sem_wait(&demoPlayer->mPrepared);
	}

    demoPlayer->mPreStatus = STATUS_STOPPED;
    demoPlayer->mStatus    = STATUS_PREPARING;
    printf("preparing...\n");
    pthread_mutex_unlock(&demoPlayer->mMutex);
}

void play(DemoPlayerContext *demoPlayer)
{
    if(XPlayerStart(demoPlayer->mAwPlayer) != 0)
    {
        printf("error:\n");
        printf("    AwPlayer::start() return fail.\n");
        return;
    }
    demoPlayer->mPreStatus = demoPlayer->mStatus;
    demoPlayer->mStatus    = STATUS_PLAYING;
    printf("playing.\n");
}

void pause(DemoPlayerContext *demoPlayer)
{
    if(XPlayerPause(demoPlayer->mAwPlayer) != 0)
    {
        printf("error:\n");
        printf("    AwPlayer::pause() return fail.\n");
        return;
    }
    demoPlayer->mPreStatus = demoPlayer->mStatus;
    demoPlayer->mStatus    = STATUS_PAUSED;
    printf("paused.\n");
}

void stop(DemoPlayerContext *demoPlayer)
{
    if(XPlayerReset(demoPlayer->mAwPlayer) != 0)
    {
        printf("error:\n");
        printf("    AwPlayer::reset() return fail.\n");
        return;
    }
    demoPlayer->mPreStatus = demoPlayer->mStatus;
    demoPlayer->mStatus    = STATUS_STOPPED;
    printf("stopped.\n");
}


//* a callback for awplayer.
int CallbackForAwPlayer(void* pUserData, int msg, int ext1, void* param)
{
    DemoPlayerContext* pDemoPlayer = (DemoPlayerContext*)pUserData;

    switch(msg)
    {
        case AWPLAYER_MEDIA_INFO:
        {
            switch(ext1)
            {
                case AW_MEDIA_INFO_NOT_SEEKABLE:
                {
                    pDemoPlayer->mSeekable = 0;
                    printf("info: media source is unseekable.\n");
                    break;
                }
                case AW_MEDIA_INFO_RENDERING_START:
                {
                    printf("info: start to show pictures.\n");
                    break;
                }
            }
            break;
        }

        case AWPLAYER_MEDIA_ERROR:
        {
            pthread_mutex_lock(&pDemoPlayer->mMutex);
            pDemoPlayer->mStatus = STATUS_STOPPED;
            pDemoPlayer->mPreStatus = STATUS_STOPPED;
            printf("error: open media source fail.\n");
            pthread_mutex_unlock(&pDemoPlayer->mMutex);
            pDemoPlayer->mError = 1;

            loge(" error : how to deal with it");
            break;
        }

        case AWPLAYER_MEDIA_PREPARED:
        {
            logd("info : preared");
            pDemoPlayer->mPreStatus = pDemoPlayer->mStatus;
            pDemoPlayer->mStatus = STATUS_PREPARED;
			sem_post(&pDemoPlayer->mPrepared);
            printf("info: prepare ok.\n");
            break;
        }

        case AWPLAYER_MEDIA_PLAYBACK_COMPLETE:
        {
            sem_post(&pDemoPlayer->mStoped);//* stop the player.
            //* TODO
            break;
        }

        case AWPLAYER_MEDIA_SEEK_COMPLETE:
        {
            pthread_mutex_lock(&pDemoPlayer->mMutex);
            pDemoPlayer->mStatus = pDemoPlayer->mPreStatus;
            printf("info: seek ok.\n");
            pthread_mutex_unlock(&pDemoPlayer->mMutex);
            break;
        }

        default:
        {
            //printf("warning: unknown callback from AwPlayer.\n");
            break;
        }
    }

    return 0;
}

static uint8_t cedarx_inited = 0;
void AwParserInit(void);
void AwStreamInit(void);

uint32_t g_stop_xplayer = 0;

int cedarx_test()
{
	FRESULT res;
	FATFS fs;
    DemoPlayerContext demoPlayer;

    printf_lock_init();

	if (!cedarx_inited) {
		cedarx_inited = 1;
		AwParserInit();
		AwStreamInit();
	}

    if ((res = f_mount(&fs, "0:/", 1)) != FR_OK) {
    	printf("can not mount\n");
		return -1;
    } else {
    	printf("mount success\n");
    }

    //* create a player.
    memset(&demoPlayer, 0, sizeof(DemoPlayerContext));
    pthread_mutex_init(&demoPlayer.mMutex, NULL);
    sem_init(&demoPlayer.mStoped, 0, 0);
	sem_init(&demoPlayer.mPrepared, 0, 0);

    demoPlayer.mAwPlayer = XPlayerCreate();
    if(demoPlayer.mAwPlayer == NULL)
    {
        printf("can not create AwPlayer, quit.\n");
        return -1;
    } else {
    	printf("create AwPlayer success.\n");
    }

    //* set callback to player.
    XPlayerSetNotifyCallback(demoPlayer.mAwPlayer, CallbackForAwPlayer, (void*)&demoPlayer);

    //* check if the player work.
    if(XPlayerInitCheck(demoPlayer.mAwPlayer) != 0)
    {
        printf("initCheck of the player fail, quit.\n");
        XPlayerDestroy(demoPlayer.mAwPlayer);
        demoPlayer.mAwPlayer = NULL;
        return -1;
    } else
    	printf("AwPlayer check success.\n");

    SoundCtrl* sound = SoundDeviceCreate();

    XPlayerSetAudioSink(demoPlayer.mAwPlayer, (void*)sound);

	uint32_t idx = 6;
	char music_file[64];

	while (!g_stop_xplayer) {
#if 1
		sprintf(music_file, "file://music/%02d.mp3", ((idx % 11) + 1));
		printf("****** play %s\n", music_file);
		set_source(&demoPlayer, music_file);
		play(&demoPlayer);
		sem_wait(&demoPlayer.mStoped);
		stop(&demoPlayer);
		msleep(5);
#endif
#if 1
		sprintf(music_file, "file://music/%02d.amr", ((idx % 8) + 1));
		printf("****** play %s\n", music_file);
		set_source(&demoPlayer, music_file);
		play(&demoPlayer);
		sem_wait(&demoPlayer.mStoped);
		stop(&demoPlayer);
		msleep(5);
#endif
		idx++;
	}


    printf("destroy AwPlayer.\n");

    if(demoPlayer.mAwPlayer != NULL)
    {
        XPlayerDestroy(demoPlayer.mAwPlayer);
        demoPlayer.mAwPlayer = NULL;
    }

    printf("destroy AwPlayer 1.\n");
    pthread_mutex_destroy(&demoPlayer.mMutex);

    if ((res = f_mount(NULL, "", 1)) != FR_OK)
    	printf("failed to unmount\n");

	sem_destroy(&demoPlayer.mPrepared);
	sem_destroy(&demoPlayer.mStoped);

    printf_lock_deinit();

    return 0;
}

int cedarx_http_test()
{
    DemoPlayerContext demoPlayer;

    printf_lock_init();

	if (!cedarx_inited) {
		cedarx_inited = 1;
		AwParserInit();
		AwStreamInit();
	}

    //* create a player.
    memset(&demoPlayer, 0, sizeof(DemoPlayerContext));
    pthread_mutex_init(&demoPlayer.mMutex, NULL);
    sem_init(&demoPlayer.mStoped, 0, 0);
	sem_init(&demoPlayer.mPrepared, 0, 0);

    demoPlayer.mAwPlayer = XPlayerCreate();
    if(demoPlayer.mAwPlayer == NULL)
    {
        printf("can not create AwPlayer, quit.\n");
        return -1;
    } else {
    	printf("create AwPlayer success.\n");
    }

    //* set callback to player.
    XPlayerSetNotifyCallback(demoPlayer.mAwPlayer, CallbackForAwPlayer, (void*)&demoPlayer);

    //* check if the player work.
    if(XPlayerInitCheck(demoPlayer.mAwPlayer) != 0)
    {
        printf("initCheck of the player fail, quit.\n");
        XPlayerDestroy(demoPlayer.mAwPlayer);
        demoPlayer.mAwPlayer = NULL;
        return -1;
    } else
    	printf("AwPlayer check success.\n");

    SoundCtrl* sound = SoundDeviceCreate();

    XPlayerSetAudioSink(demoPlayer.mAwPlayer, (void*)sound);

	uint32_t idx = 9;
	char music_file[64];
	while (!g_stop_xplayer) {
		sprintf(music_file, "http://192.168.51.102/%02d.mp3", ((++idx) % 10) + 2);
		printf("****** play %s\n", music_file);

	    set_source(&demoPlayer, music_file);
	    play(&demoPlayer);
	    sem_wait(&demoPlayer.mStoped);
	    stop(&demoPlayer);

		msleep(5);
	}


    printf("destroy AwPlayer.\n");

    if(demoPlayer.mAwPlayer != NULL)
    {
        XPlayerDestroy(demoPlayer.mAwPlayer);
        demoPlayer.mAwPlayer = NULL;
    }

    printf("destroy AwPlayer 1.\n");
    pthread_mutex_destroy(&demoPlayer.mMutex);

	sem_destroy(&demoPlayer.mPrepared);
	sem_destroy(&demoPlayer.mStoped);

    printf_lock_deinit();

    return 0;
}

#endif /* __PRJ_CONFIG_XPLAYER */
