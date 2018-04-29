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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
//#include <ctype.h>
//#include <errno.h>

//#include <cdx_log.h>

#include "xrecord.h"
#include "common/framework/fs_ctrl.h"

#define SAVE_VIDEO_FRAME (0)
#define AUDIO_INPUT (1)
#define VIDEO_INPUT (0)

//* the main method.
int xrecord_test()
{
	if (fs_mount_request(FS_MNT_DEV_TYPE_SDCARD, 0, FS_MNT_MODE_MOUNT) != 0) {
		printf("mount fail\n");
		return -1;
	} else {
		printf("mount success\n");
	}

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

	XRecordConfig audioConfig;
	audioConfig.nChan = 1;
	audioConfig.nSamplerate = 8000;
	audioConfig.nSamplerBits = 16;
	audioConfig.nBitrate = 12200;

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

	if (fs_mount_request(FS_MNT_DEV_TYPE_SDCARD, 0, FS_MNT_MODE_UNMOUNT) != 0) {
		printf("unmount fail\n");
	}

    printf("\n");
    printf("**************************************************************\n");
    printf("* Quit the program, goodbye!\n");
    printf("**************************************************************\n");
    printf("\n");

    return 0;
}
