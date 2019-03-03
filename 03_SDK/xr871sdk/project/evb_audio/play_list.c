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
#include <fs/fatfs/ff.h>
#include <stdlib.h>
#include "common/framework/fs_ctrl.h"
#include "play_list.h"

#define PLAYER_SONGS_DIR  "0:/music"
#define PLAY_LIST_FILE_NAME  "/mp3_list.txt"

#define LIST_LOGE(fmt, arg...)  printf("[LIST_ERR][F:%s][L:%d] " fmt, __func__, __LINE__, ##arg)
#define LIST_LOGD(fmt, arg...)  printf("[LIST_DBG][F:%s][L:%d] " fmt, __func__, __LINE__, ##arg)

struct play_list_info {
    FIL fp;
    int count;
    uint32_t Mp3_Position[100];
    uint32_t Mp3_File_Num;
};

static struct play_list_info list_info;

static int player_open_dir(DIR *dirs, char *path)
{
    if (strlen(path) + 14 > 50)
        return -1;

    if (f_opendir(dirs, path) != FR_OK) {
        return -1;
    }
    return 0;
}

static int read_songs_name(FIL* fp, char *buff)
{
    uint32_t len = 0;
    FRESULT res;
    char data;
    uint8_t i = 0;

    while(1) {
        res = f_read (fp, &data, 1, &len);
        if (res != FR_OK) {
            LIST_LOGE("read file error %d\n", res);
            return -1;
        }
        if (len == 0) {
            LIST_LOGD("file read end\n");
            return 0;
        }

        buff[i++] = data;

        if (data == '\0') {
            LIST_LOGD("read one song\n");
            break;
        }
    }
    return i;
}

int player_read_song(PLAYER_READ_SONG ctrl, char *buff)
{
    uint32_t position = 0;
    int ret = 0;
    struct play_list_info *info = &list_info;

    if (ctrl == PLAYER_NEXT) {
        info->count++;
        if (info->count >= info->Mp3_File_Num)
            info->count = 0;
        ret = read_songs_name(&info->fp, buff);
        if (ret == 0) {
            f_lseek (&info->fp, 0);
            ret = read_songs_name(&info->fp, buff);
            info->count = 0;
        }
    } else {
        info->count--;
        if (info->count < 0)
            info->count = info->Mp3_File_Num - 1;
        position = info->Mp3_Position[info->count];
        f_lseek (&info->fp, position);
        ret = read_songs_name(&info->fp, buff);
    }
    return 0;
}

static int create_play_list(char *music_dir, char *file_name)  /* Pointer to the path name working buffer */
{
    FRESULT res;
    int ret;
    char *mp3;
    char *MP3;
    DIR dirs;
    FILINFO finfo;
    char f_path[100];
    uint8_t Mp3_File_count = 1;
    uint32_t File_Position = 0;
    struct play_list_info *info = &list_info;

    ret =  player_open_dir(&dirs, music_dir);
    if(ret != 0) {
        LIST_LOGE("open dir error\n");
        return -1;
    }

    sprintf(f_path, "%s%s", music_dir, file_name);
    f_unlink(f_path);

    res = f_open(&info->fp, f_path, FA_CREATE_NEW | FA_WRITE | FA_READ);
    if(res != FR_OK) {
        LIST_LOGE("open file error %d\n", res);
        return -1;
    }

    info->Mp3_File_Num = 0;

    while(1) {
        mp3 = MP3 = NULL;
        if ((res = f_readdir(&dirs, &finfo)) == FR_OK && finfo.fname[0]) {
            if (finfo.fattrib & AM_DIR) {
                LIST_LOGD("find a dir.\n");
            } else {
                mp3 = strstr(finfo.fname, ".mp3");
                MP3 = strstr(finfo.fname, ".MP3");
                if (mp3 || MP3) {
                    LIST_LOGD("find a mp3: %s\n",finfo.fname);
                    uint32_t write_len = 0;
                    sprintf(f_path, "file://%s/%s", music_dir, finfo.fname);
                    res = f_write (&info->fp, f_path, strlen(f_path) + 1, &write_len);
                    if ((res != FR_OK) || (write_len != strlen(f_path) + 1)) {
                        goto err;
                    }
                    File_Position += write_len;
                    info->Mp3_Position[Mp3_File_count++] = File_Position;
                    info->Mp3_File_Num++;
                }
            }
        } else {
            LIST_LOGD("scan mp3 end\n");
            break;
        }
    }
    f_close (&info->fp);

    if (info->Mp3_File_Num == 0) {
        LIST_LOGD("no song found\n");
        return -1;
    }

    return 0;
err:
    f_close (&info->fp);
    LIST_LOGE("write file fail\n");
    return -1;
}

int play_list_init ()
{
    int ret;
    char *music_dir = PLAYER_SONGS_DIR;
    char *list_name = PLAY_LIST_FILE_NAME;
    char list_path[64];
    struct play_list_info *info = &list_info;

    info->count = -1;

    sprintf(list_path, "%s%s", music_dir, list_name);

    if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
        LIST_LOGE("mount fail\n");
        return -1;
    }

    ret = create_play_list(music_dir, list_name);
    if (ret != 0) {
        LIST_LOGE("create play list fail\n");
        return -1;
    }

    if (f_open(&info->fp, list_path, FA_READ) != FR_OK) {
        LIST_LOGE("open file fail\n");
        return -1;
    }
    return 0;
}

void play_list_deinit()
{
    struct play_list_info *info = &list_info;

    f_close(&info->fp);
    if (fs_ctrl_unmount(FS_MNT_DEV_TYPE_SDCARD, 0) != 0) {
        LIST_LOGE("unmount fail\n");
    }
}

