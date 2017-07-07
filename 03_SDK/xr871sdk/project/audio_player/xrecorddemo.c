/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : recoderdemo.c
 * Description : recoderdemo
 * History :
 *
 */

#define LOG_TAG "recoderdemo"

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
//#include <ctype.h>
//#include <errno.h>

//#include <cdx_config.h>
//#include <cdx_log.h>

#include <CdxQueue.h>
#include <AwPool.h>
//#include <CdxBinary.h>
#include <CdxMuxer.h>

//#include "memoryAdapter.h"
#include "awencoder.h"

#include "RecoderWriter.h"
#include "xrecord.h"

#define SAVE_VIDEO_FRAME (0)
#define AUDIO_INPUT (1)
#define VIDEO_INPUT (0)

//* the main method.
int xrecord_test()
{
	FRESULT res;
	FATFS fs;

	printf_lock_init();

	if ((res = f_mount(&fs, "0:/", 1)) != FR_OK) {
    	printf("can not mount\n");
		return -1;
    } else {
    	printf("mount success\n");
    }

void AwMuxerInit();
	AwMuxerInit();

    XRecord *xrecord = XRecordCreate();
	if (xrecord == NULL)
		printf("create success\n");
//    XRecordSetNotifyCallback(xrecord, , );

/*    if(XRecordInitCheck(xrecord) != 0)
    {
        printf("initCheck of the recoder fail, quit.\n");
        XRecordDestroy(xrecord);
        xrecord = NULL;
        return -1;
    } else
        printf("AwRecoder check success.\n");*/

    CaptureCtrl* cap = CaptureDeviceCreate();
	if (!cap)
		printf("cap device create failed\n");
    XRecordSetAudioCap(xrecord, cap);
			AudioEncodeConfig audioConfig;

			audioConfig.nType = AUDIO_ENCODE_AMR_TYPE;
			audioConfig.nInChan = 1;
			audioConfig.nInSamplerate = 8000;
			audioConfig.nOutChan = 1;
			audioConfig.nOutSamplerate = 8000;
			audioConfig.nSamplerBits = 16;
			audioConfig.nBitrate = 12200;
			audioConfig.nFrameStyle = 0;


    XRecordSetDataDstUrl(xrecord, "file://record/wechat.amr", NULL, NULL);
	printf("seturl\n");
    XRecordSetAudioEncodeType(xrecord, XRECODER_AUDIO_ENCODE_AMR_TYPE, &audioConfig);
	printf("set encode type\n");

    XRecordPrepare(xrecord);
    XRecordStart(xrecord);
	printf("record start\n");
    msleep(10000);
    XRecordStop(xrecord);
	printf("record stop\n");
	XRecordDestroy(xrecord);
	printf("record destroy\n");

	printf_lock_deinit();

	if ((res = f_mount(NULL, "", 1)) != FR_OK)
    	printf("failed to unmount\n");

    printf("\n");
    printf("**************************************************************\n");
    printf("* Quit the program, goodbye!\n");
    printf("**************************************************************\n");
    printf("\n");

    return 0;
}
