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

#include "stdio.h"
#include "string.h"

#include "kernel/os/os.h"
#include "driver/component/oled/drv_oled.h"

#include "audio_display.h"
#include "audio_player.h"
#include "common/cmd/cmd_defs.h"
#include <fs/fatfs/ff.h>

#include <stdlib.h>

#include "pthread.h"
#include "iniparserapi.h"

#include <cdx_log.h>
#include "xplayer.h"
#include "CdxTypes.h"
#include "driver/chip/hal_codec.h"
#include "audio/manager/audio_manager.h"

#define AUDIO_PLAYER_DBG 1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define AUDIO_PLAYER_DEBUG(fmt, arg...)	\
			LOG(AUDIO_PLAYER_DBG, "[AUDIO_PLAYER] "fmt, ##arg)

FRESULT Player_Open_Dir(DIR *dirs, char *path)
{
	FRESULT res;
	AUDIO_PLAYER_DEBUG("[%s entry] open dir: %s\n", __func__, path);
	if (strlen(path) + 14 > 50)
		return FR_INVALID_PARAMETER;

	if ((res = f_opendir(dirs, path)) == FR_OK)
		return res;
	else {
		COMPONENT_WARN("open dir error %d\n", res);
		return res;
	}
}

int Read_Songs_Name(FIL* fp, char *buff)
{
	uint32_t len = 0;
	FRESULT res;
	char data;
	uint8_t i = 0;
	while(1) {
		res = f_read (fp, &data, 1, &len);
		if (res != FR_OK) {
			COMPONENT_WARN("read file error %d\n", res);
			return -1;
		}
		if (len == 0) {
			AUDIO_PLAYER_DEBUG("file read end\n");
			return 0;
		}

		buff[i++] = data;

		if (data == '\0') {
			AUDIO_PLAYER_DEBUG("read one song\n");
			break;
		}
	}
	return i;
}

uint32_t Mp3_Position[100];
uint32_t Mp3_File_Num = 0;
int Create_Mp3_List(FIL *fp,DIR *dirs, char *path)	/* Pointer to the path name working buffer */
{
	FRESULT res;
	char *mp3;
	char *MP3;
	FILINFO finfo;
	char f_path[100];

	sprintf(f_path,"%s/mp3_list.txt", path);
	f_unlink(f_path);

	res = f_open(fp, f_path, FA_CREATE_NEW | FA_WRITE | FA_READ);
	if(res != FR_OK) {
		COMPONENT_WARN("open file error %d\n", res);
		return 0;
	}
	uint8_t Mp3_File_count = 1;
	uint32_t File_Position = 0;
	while(1) {
		mp3 = MP3 = NULL;
		if ((res = f_readdir(dirs, &finfo)) == FR_OK && finfo.fname[0]) {
			if (finfo.fattrib & AM_DIR) {
				AUDIO_PLAYER_DEBUG("[%s]find a dir.\n", __func__);
			} else {
				mp3 = strstr(finfo.fname, ".mp3");
				MP3 = strstr(finfo.fname, ".MP3");
				if (mp3 || MP3) {
					AUDIO_PLAYER_DEBUG("find a mp3: %s\n",finfo.fname);
					uint32_t write_len = 0;
					res = f_write (fp, finfo.fname, strlen(finfo.fname) + 1, &write_len);
					File_Position += write_len;
					Mp3_Position[Mp3_File_count++] = File_Position;
					Mp3_File_Num ++;
					if (res != FR_OK) {
						f_close (fp);
						COMPONENT_WARN("write file error %d\n", res);
						return 0;
					}

					if (write_len != strlen(finfo.fname) + 1) {
						f_close (fp);
						COMPONENT_WARN("write file error\n");
						return 0;
					}
				}
			}
		} else {
			AUDIO_PLAYER_DEBUG("scan mp3 end\n");
			break;
		}
	}
	f_close (fp);
	COMPONENT_TRACK("end\n");
	return 1;
}

FATFS fs;
DIR dirs;
FIL fp;

void Read_Songs_Init (FATFS *fs, DIR *dirs, FIL *fp)
{
	FRESULT res;
	char *music_dir = "0:/music";

	//memset(&fs, 0, sizeof(fs));
	if ((res = f_mount(fs, "0:/", 1)) != FR_OK)
		COMPONENT_WARN("failed to mount: error\n");

	res =  Player_Open_Dir(dirs, music_dir);
	if(res != FR_OK)
		COMPONENT_WARN("open dir error %d\n", res);

	Create_Mp3_List(fp, dirs, music_dir);

	res = f_open(fp, "music/mp3_list.txt", FA_READ);
	if (res != FR_OK) {
		COMPONENT_WARN("open file error %d\n", res);
		return;
	}
}

/****************************************************************/
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

DemoPlayerContext demoPlayer;
uint8_t play_song_flag = 0;

static const int STATUS_STOPPED   = 0;
static const int STATUS_PREPARING = 1;
static const int STATUS_PREPARED  = 2;
static const int STATUS_PLAYING   = 3;
static const int STATUS_PAUSED    = 4;
static const int STATUS_SEEKING   = 5;

extern SoundCtrl* SoundDeviceCreate();
extern void AwParserInit();
extern void AwStreamInit();

extern int XPlayerSetNotifyCallback(XPlayer* p,
                                    XPlayerNotifyCallback notifier,
                                    void* pUserData);

static int CallbackForAwPlayer(void* pUserData, int msg, int ext1, void* param)
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
                    AUDIO_PLAYER_DEBUG("info: media source is unseekable.\n");
                    break;
                }
                case AW_MEDIA_INFO_RENDERING_START:
                {
                    AUDIO_PLAYER_DEBUG("info: start to show pictures.\n");
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
            COMPONENT_WARN("error: open media source fail.\n");
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
            AUDIO_PLAYER_DEBUG("info: prepare ok.\n");
            break;
        }

        case AWPLAYER_MEDIA_PLAYBACK_COMPLETE:
        {
            sem_post(&pDemoPlayer->mStoped);//* stop the player.
            //* TODO
            play_song_flag = 0;
            break;
        }

        case AWPLAYER_MEDIA_SEEK_COMPLETE:
        {
            pthread_mutex_lock(&pDemoPlayer->mMutex);
            pDemoPlayer->mStatus = pDemoPlayer->mPreStatus;
            AUDIO_PLAYER_DEBUG("info: seek ok.\n");
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

static void set_source(DemoPlayerContext *demoPlayer, char* pUrl)
{
    demoPlayer->mSeekable = 1;

    //* set url to the AwPlayer.
    if(XPlayerSetDataSourceUrl(demoPlayer->mAwPlayer,
                 (const char*)pUrl, NULL, NULL) != 0)
    {
        COMPONENT_WARN("error:\n");
        COMPONENT_WARN("    AwPlayer::setDataSource() return fail.\n");
        return;
    }
     COMPONENT_TRACK("setDataSource end\n");

    //* start preparing.
    pthread_mutex_lock(&demoPlayer->mMutex);

	if (!strncmp(pUrl, "http://", 7)) {
	    if(XPlayerPrepareAsync(demoPlayer->mAwPlayer) != 0)
	    {
	        COMPONENT_WARN("error:\n");
	        COMPONENT_WARN("    AwPlayer::prepareAsync() return fail.\n");
	        pthread_mutex_unlock(&demoPlayer->mMutex);
	        return;
	    }
		sem_wait(&demoPlayer->mPrepared);
	}

    demoPlayer->mPreStatus = STATUS_STOPPED;
    demoPlayer->mStatus    = STATUS_PREPARING;
    AUDIO_PLAYER_DEBUG("preparing...\n");
    pthread_mutex_unlock(&demoPlayer->mMutex);
}

static void play(DemoPlayerContext *demoPlayer)
{
    if(XPlayerStart(demoPlayer->mAwPlayer) != 0)
    {
        COMPONENT_WARN("error:\n");
        COMPONENT_WARN("    AwPlayer::start() return fail.\n");
        return;
    }
    demoPlayer->mPreStatus = demoPlayer->mStatus;
    demoPlayer->mStatus    = STATUS_PLAYING;
    AUDIO_PLAYER_DEBUG("playing.\n");
}

static void stop(DemoPlayerContext *demoPlayer)
{
    if(XPlayerReset(demoPlayer->mAwPlayer) != 0)
    {
        COMPONENT_WARN("error:\n");
        COMPONENT_WARN("    AwPlayer::reset() return fail.\n");
        return;
    }
    demoPlayer->mPreStatus = demoPlayer->mStatus;
    demoPlayer->mStatus    = STATUS_STOPPED;
    AUDIO_PLAYER_DEBUG("stopped.\n");
}

int player_init()
{
	Read_Songs_Init (&fs, &dirs, &fp);
    printf_lock_init();

	AwParserInit();
	AwStreamInit();

    //* create a player.
    memset(&demoPlayer, 0, sizeof(DemoPlayerContext));
    pthread_mutex_init(&demoPlayer.mMutex, NULL);
    sem_init(&demoPlayer.mStoped, 0, 0);
	sem_init(&demoPlayer.mPrepared, 0, 0);

    demoPlayer.mAwPlayer = XPlayerCreate();
    if(demoPlayer.mAwPlayer == NULL) {
        COMPONENT_WARN("can not create AwPlayer, quit.\n");
        return -1;
    } else {
    	AUDIO_PLAYER_DEBUG("create AwPlayer success.\n");
    }

    //* set callback to player.
    XPlayerSetNotifyCallback(demoPlayer.mAwPlayer, CallbackForAwPlayer, (void*)&demoPlayer);

    //* check if the player work.
    if(XPlayerInitCheck(demoPlayer.mAwPlayer) != 0) {
        COMPONENT_WARN("initCheck of the player fail, quit.\n");
        XPlayerDestroy(demoPlayer.mAwPlayer);
        demoPlayer.mAwPlayer = NULL;
        return -1;
    } else
    	AUDIO_PLAYER_DEBUG("AwPlayer check success.\n");

    SoundCtrl* sound = SoundDeviceCreate();

    XPlayerSetAudioSink(demoPlayer.mAwPlayer, (void*)sound);
    return 0;
}

void play_songs(char *song_name)
{
	char music_file[400];
	sprintf(music_file, "file://music/%s", song_name);
	stop(&demoPlayer);
	set_source(&demoPlayer, music_file);
	play_song_flag = 1;
	play(&demoPlayer);
}

void player_stop()
{
	stop(&demoPlayer);
}

void player_deinit()
{
	FRESULT res;
	f_close(&fp);

	if(demoPlayer.mAwPlayer != NULL) {
		XPlayerDestroy(demoPlayer.mAwPlayer);
		demoPlayer.mAwPlayer = NULL;
	}

	pthread_mutex_destroy(&demoPlayer.mMutex);

	if ((res = f_mount(NULL, "", 1)) != FR_OK)
		COMPONENT_WARN("failed to unmount\n");

	sem_destroy(&demoPlayer.mPrepared);
	sem_destroy(&demoPlayer.mStoped);

	printf_lock_deinit();
}

typedef enum {
	PLAYER_PREV,
	PLAYER_NEXT,
}PLAYER_READ_SONG;

char* player_read_songs(PLAYER_READ_SONG ctrl, char *buff)
{
	static int count = -1;
	uint32_t position = 0;
	int ret = 0;
	if (ctrl == PLAYER_NEXT) {
		count ++;
		if (count >= Mp3_File_Num)
			count = 0;
		ret = Read_Songs_Name(&fp, buff);
		if (ret == 0) {
			position = Mp3_Position[count];
			f_lseek (&fp, position);
			f_lseek (&fp, 0);
			ret = Read_Songs_Name(&fp, buff);
			count = 0;
		}
	} else {
		count --;
		if (count < 0)
			count = Mp3_File_Num - 1;
		position = Mp3_Position[count];
		f_lseek (&fp, position);
		ret = Read_Songs_Name(&fp, buff);
	}
	return buff;
}

void player_volume_ctrl(int volume)
{
	aud_mgr_handler(0, volume);
}

static void pause(DemoPlayerContext *demoPlayer)
{
    if(XPlayerPause(demoPlayer->mAwPlayer) != 0)
    {
        COMPONENT_WARN("error:\n");
        COMPONENT_WARN("    AwPlayer::pause() return fail.\n");
        return;
    }
    demoPlayer->mPreStatus = demoPlayer->mStatus;
    demoPlayer->mStatus    = STATUS_PAUSED;
}


typedef enum {
	PLAYER_PAUSE_EN,
	PLAYER_PAUSE_DIS,
}PLAYER_PAUSE_CTRL;

void player_pause(PLAYER_PAUSE_CTRL ctrl)
{
	if (ctrl == PLAYER_PAUSE_EN)
		pause(&demoPlayer);
	else
		play(&demoPlayer);;
}

typedef enum {
	CMD_PLAYER_NEXT, //K1
	CMD_PLAYER_PERV, //K2
	CMD_PLAYER_VOLUME_DOWN, // K1 REPEAT
	CMD_PLAYER_VOLUME_UP,  //K2 REPEAT
	CMD_PLAYER_PAUSE, //K3 SHORT
	CMD_PLAYER_STOP, //K4
	CMD_PLAYER_NULL,
}PLAYER_CMD;

GPIO_Button_Cmd_Info Gpio_Button_Cmd = {GPIO_BUTTON_CMD_NULL, GPIO_BUTTON_NUM};
AD_Button_Cmd_Info AD_Button_Cmd = {AD_BUTTON_CMD_NULL, AD_BUTTON_NUM};

void player_set_gpio_button_cmd(GPIO_Button_Cmd_Info *cmd)
{
	Gpio_Button_Cmd = *cmd;
}

void player_set_ad_button_cmd(AD_Button_Cmd_Info *cmd)
{
	AD_Button_Cmd = *cmd;
}

PLAYER_CMD read_payer_ctrl_cmd()
{
	PLAYER_CMD cmd = CMD_PLAYER_NULL;
	if (Gpio_Button_Cmd.cmd != GPIO_BUTTON_CMD_NULL) {
		switch(Gpio_Button_Cmd.id) {
			case GPIO_BUTTON_0:
				if (Gpio_Button_Cmd.cmd == GPIO_BUTTON_CMD_SHORT_PRESS)
					cmd = CMD_PLAYER_PAUSE;
				break;
			case GPIO_BUTTON_1:
				if (Gpio_Button_Cmd.cmd == GPIO_BUTTON_CMD_SHORT_PRESS)
					cmd = CMD_PLAYER_VOLUME_DOWN;
				else if (Gpio_Button_Cmd.cmd == GPIO_BUTTON_CMD_REPEAT)
					cmd = CMD_PLAYER_VOLUME_DOWN;
				break;
			default :
				break;
		}
		Gpio_Button_Cmd.cmd = GPIO_BUTTON_CMD_NULL;
		Gpio_Button_Cmd.id = GPIO_BUTTON_NUM;
	} else if (AD_Button_Cmd.cmd != AD_BUTTON_CMD_NULL) {
		switch (AD_Button_Cmd.id) {
			case AD_BUTTON_0:
				if (AD_Button_Cmd.cmd == AD_BUTTON_CMD_SHORT_PRESS)
					cmd = CMD_PLAYER_NEXT;
				break;
			case AD_BUTTON_1:
				if (AD_Button_Cmd.cmd == AD_BUTTON_CMD_SHORT_PRESS)
					cmd = CMD_PLAYER_PERV;
				break;
			case AD_BUTTON_2:
				if (AD_Button_Cmd.cmd == AD_BUTTON_CMD_SHORT_PRESS)
					cmd = CMD_PLAYER_VOLUME_UP;
				else if (AD_Button_Cmd.cmd == AD_BUTTON_CMD_REPEAT)
					cmd = CMD_PLAYER_VOLUME_UP;
				break;
			default:
				break;
		}
		AD_Button_Cmd.cmd = AD_BUTTON_CMD_NULL;
		AD_Button_Cmd.id = AD_BUTTON_NUM;
	}
	return cmd;
}


#define PLAYER_THREAD_STACK_SIZE	1024*2
OS_Thread_t player_task_thread;

#define DISPLAY_SONG_PERIOD 4
static uint8_t player_task_run = 0;
static char read_songs_buf[400];
static uint8_t count = DISPLAY_SONG_PERIOD;
static int volume = 15;
PLAYER_PAUSE_CTRL pause_ctrl = PLAYER_PAUSE_DIS;

void player_task(void *arg)
{
	player_init();
	player_volume_ctrl(volume);

	while (player_task_run) {
		PLAYER_CMD cmd = read_payer_ctrl_cmd();
		switch (cmd) {
			case CMD_PLAYER_NEXT:
				pause_ctrl = PLAYER_PAUSE_DIS;
				ui_reset_time();
				player_read_songs(PLAYER_NEXT, read_songs_buf);
				play_songs(read_songs_buf);
				ui_set_songs_name(read_songs_buf);
				goto end;
			case CMD_PLAYER_PERV:
				pause_ctrl = PLAYER_PAUSE_DIS;
				ui_reset_time();
				player_read_songs(PLAYER_PREV, read_songs_buf);
				play_songs(read_songs_buf);
				ui_set_songs_name(read_songs_buf);
				goto end;
			case CMD_PLAYER_VOLUME_UP:
				volume++;
				if (volume > 31)
					volume = 31;

				player_volume_ctrl(volume);
				break;
			case CMD_PLAYER_VOLUME_DOWN:
				volume--;
				if (volume < 0)
					volume = 0;
				player_volume_ctrl(volume);
				break;
			case CMD_PLAYER_PAUSE:
				if (pause_ctrl == PLAYER_PAUSE_DIS)
					pause_ctrl = PLAYER_PAUSE_EN;
				else
					pause_ctrl = PLAYER_PAUSE_DIS;

				player_pause(pause_ctrl);
				break;
			default:
				break;
		}

		if (!play_song_flag) {
			ui_reset_time();
			player_read_songs(PLAYER_NEXT, read_songs_buf);
			play_songs(read_songs_buf);
			ui_set_songs_name(read_songs_buf);
		}

		end :
		if (pause_ctrl == PLAYER_PAUSE_DIS) {
			ui_draw_time(40);

			count --;
			if (count == 0) {
				count = DISPLAY_SONG_PERIOD;
				ui_show_songs_name(read_songs_buf);
			}
		}
		ui_show_volume(volume);
		OS_MSleep(40);
	}
	COMPONENT_TRACK("end\n");
	OS_ThreadDelete(&player_task_thread);
}

Component_Status player_task_deinit()
{
	player_task_run = 0;
	while((OS_ThreadIsValid(&player_task_thread)))
		OS_MSleep(10);
	DRV_Oled_DeInit();
	return COMP_OK;
}

Component_Status player_task_init()
{
	ui_init();

	player_task_run = 1;
	if (OS_ThreadCreate(&player_task_thread,
		                "",
		                player_task,
		               	NULL,
		                OS_THREAD_PRIO_APP,
		                PLAYER_THREAD_STACK_SIZE) != OS_OK) {
		COMPONENT_WARN("thread create error\n");
		return COMP_ERROR;
	}

	return COMP_OK;
}

