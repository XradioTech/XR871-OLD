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
#include "fs_ctrl.h"
#include "driver/chip/sdmmc/hal_sdhost.h"
#include "common/framework/sys_ctrl/sys_ctrl.h"
#include "fs/fatfs/ff.h"
#include "fs/fatfs/diskio.h"
#include "driver/chip/sdmmc/sdmmc.h"
#include "sys/interrupt.h"

#define	_FS_WARN			1
#define	_FS_ERROR			1
#define	_FS_INFO			1

#define FS_LOG(flags, fmt, arg...)		\
		do {							\
			if (flags)					\
				printf(fmt, ##arg); 	\
		} while (0)

#define FS_WARN(fmt, arg...)	\
		FS_LOG(_FS_WARN, "[FS WRN] %s():%d "fmt, __func__, __LINE__, ##arg)

#define FS_ERROR(fmt, arg...)	\
		FS_LOG(_FS_ERROR, "[FS ERR] %s():%d "fmt, __func__, __LINE__, ##arg)

#define FS_INFO(fmt, arg...)	\
		FS_LOG(_FS_INFO, "[FS INF]  "fmt, ##arg)

#define FS_CRTL_TIME_OUT_MAX			(0x80000000)

typedef struct
{
	OS_Mutex_t mutex;
	FATFS *fs;
	int8_t mount_auto;	/* mount automatically or manually */
	enum fs_mnt_status mount_status;
} fs_ctrl_private;

static fs_ctrl_private fs_ctrl;

#define FS_CTRL_LOCK()   OS_RecursiveMutexLock(&fs_ctrl.mutex, OS_WAIT_FOREVER)
#define FS_CTRL_UNLOCK() OS_RecursiveMutexUnlock(&fs_ctrl.mutex)

/*
 * @brief try to scan device and mount FATFS.
 * @param dev_type:
 *        type of the device,like SD card
 * @param dev_id:
 *        the ID of the device.
 * @retval  0 if mount success or other if failed.
 */
static int fs_ctrl_mount(enum fs_mnt_dev_type dev_type, uint32_t dev_id)
{
	struct mmc_card *card;
	FRESULT fs_ret;
	int mmc_ret;
	int ret = -1;

	if (mmc_card_create(dev_id) != 0) {
		FS_ERROR("card create fail\n");
		goto out;
	}
	card = mmc_card_open(dev_id);
	if (card == NULL) {
		FS_ERROR("card open fail\n");
		goto err_card;
	}
	if (!mmc_card_present(card)) {
		mmc_ret = mmc_rescan(card, 0);
		if (mmc_ret != 0) {
			FS_ERROR("scan fail\n");
			mmc_card_close(dev_id);
			goto err_card;
		} else {
			FS_INFO("sd card init\n");
		}
	}
	mmc_card_close(dev_id);

	if (fs_ctrl.fs == NULL) {
		FATFS *fs = malloc(sizeof(FATFS));
		if (fs == NULL) {
			FS_ERROR("no mem\n");
			goto err_fs;
		}
		fs_ret = f_mount(fs, "", 0);
		if (fs_ret != FR_OK) {
			FS_ERROR("mount fail ret:%d\n", fs_ret);
			free(fs);
			goto err_fs;
		} else {
			FS_INFO("mount success\n");
			fs_ctrl.fs = fs;
		}
	}
	ret = 0;
	goto out;

err_fs:
	if (mmc_card_present(card))
		mmc_card_deinit(card);

err_card:
	mmc_card_delete(dev_id, 0);

out:
	if (ret == 0) {
		fs_ctrl.mount_status = FS_MNT_STATUS_MOUNT_OK;
	} else {
		fs_ctrl.mount_status = FS_MNT_STATUS_MOUNT_FAIL;
	}
	return ret;
}

/*
 * @brief try to deinitialize device and unmount FATFS.
 * @param dev_type:
 *		  type of the device,like SD card
 * @param dev_id:
 *		  the ID of the device.
 * @retval  0 if unmount success or other if dev not exist or FATFS not mount before.
 */
static int fs_ctrl_unmount(enum fs_mnt_dev_type dev_type, uint32_t dev_id)
{
	struct mmc_card *card;
	FRESULT fs_ret;
	int ret = 0;

	if (fs_ctrl.fs != NULL) {
		fs_ret = f_mount(NULL, "", 0);
		if (fs_ret != FR_OK) {
			FS_ERROR("unmount fail ret:%d\n", fs_ret);
		}
		free(fs_ctrl.fs);
		fs_ctrl.fs = NULL;
	} else {
		ret = -1;
	}

	card = mmc_card_open(dev_id);
	if (card == NULL) {
		FS_ERROR("card open fail\n");
	} else {
		if (mmc_card_present(card)) {
			mmc_card_deinit(card);
		}
		mmc_card_close(dev_id);
		mmc_card_delete(dev_id, 0);
	}

	fs_ctrl.mount_status = FS_MNT_STATUS_UNMOUNT;
	return ret;
}

static void fs_ctrl_msg_process(uint32_t event, uint32_t data, void *arg)
{
	int ret;
	uint16_t msg = FS_CTRL_MSG_NULL;
	uint32_t param = 0;

	FS_CTRL_LOCK();

	switch (EVENT_SUBTYPE(event)) {
		case FS_CTRL_MSG_SDCARD_INSERT:
			if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) < 0) {
				param = FS_MNT_MSG_PARAM(FS_MNT_DEV_TYPE_SDCARD,
				                         0, FS_MNT_STATUS_MOUNT_FAIL);
			} else {
				param = FS_MNT_MSG_PARAM(FS_MNT_DEV_TYPE_SDCARD,
				                         0, FS_MNT_STATUS_MOUNT_OK);
			}
			msg = FS_CTRL_MSG_FS_MNT;
			break;
		case FS_CTRL_MSG_SDCARD_REMOVE:
			ret = fs_ctrl_unmount(FS_MNT_DEV_TYPE_SDCARD, 0);
			if (ret == 0) {
				msg  = FS_CTRL_MSG_FS_MNT;
				param = FS_MNT_MSG_PARAM(FS_MNT_DEV_TYPE_SDCARD,
				                         0, FS_MNT_STATUS_UNMOUNT);
			}
			break;
		default:
			break;
	}

	if (msg != FS_CTRL_MSG_NULL) {
		ret = sys_event_send(CTRL_MSG_TYPE_FS, msg, param, 0);
		if (ret != 0)
			FS_ERROR("send fail\n");
	}

	FS_CTRL_UNLOCK();
}

/*
 * @brief create the sys_ctrl events.
 * @param
 *        @arg none
 * @retval  0 if create success or other if failed.
 */
int fs_ctrl_init(void)
{
	observer_base *base = sys_callback_observer_create(CTRL_MSG_TYPE_SDCARD,
	                                                   ALL_SUBTYPE,
	                                                   fs_ctrl_msg_process,
	                                                   NULL);
	if (base == NULL) {
		FS_ERROR("create fail\n");
		return -1;
	}

	if (sys_ctrl_attach(base) != 0) {
		FS_ERROR("attach fail\n");
		return -1;
	}

	if (OS_RecursiveMutexCreate(&fs_ctrl.mutex) != OS_OK) {
		FS_ERROR("create mtx fail\n");
		return -1;
	}

	return 0;
}

/*
 * @brief requset to mount or unmount FATFS.
 * @param dev_type: type of the device,like SD card.
 * @param dev_id: the ID of the device.
 * @param mode:
 *        @arg FS_MNT_MODE_MOUNT: requset to mount FATFS.
 *		  @arg FS_MNT_MODE_UNMOUNT: requset to unmount FATFS.
 * @retval 0 on success, -1 on failure
 */
int fs_mount_request(enum fs_mnt_dev_type dev_type, uint32_t dev_id, enum fs_mnt_mode mode)
{
	int ret = 0;
	uint16_t subtype;

	FS_CTRL_LOCK();

	do {
		if (mode == FS_MNT_MODE_MOUNT) {
			fs_ctrl.mount_auto = 1;
			if (fs_ctrl.mount_status == FS_MNT_STATUS_MOUNT_OK) {
				break;
			}
			subtype = FS_CTRL_MSG_SDCARD_INSERT;
		} else {
			fs_ctrl.mount_auto = 0;
			if (fs_ctrl.mount_status == FS_MNT_STATUS_UNMOUNT) {
				break;
			}
			subtype = FS_CTRL_MSG_SDCARD_REMOVE;
		}

		fs_ctrl.mount_status = FS_MNT_STATUS_INVALID;
		fs_ctrl_msg_process(MK_EVENT(CTRL_MSG_TYPE_SDCARD, subtype), 0, 0);
		if (fs_ctrl.mount_status != FS_MNT_STATUS_MOUNT_OK &&
			fs_ctrl.mount_status != FS_MNT_STATUS_UNMOUNT) {
			ret = -1;
		}
	} while (0);

	FS_CTRL_UNLOCK();
	return ret;
}

/*
 * @brief sdcard detect callback.
 * @param present:
 *        @arg present 1-card insert, 0-card remove
 * @retval  none
 */
void sdcard_detect_callback(uint32_t present)
{
	int ret;
	uint16_t subtype;

	FS_CTRL_LOCK();

	if (!fs_ctrl.mount_auto) {
		FS_CTRL_UNLOCK();
		return;
	}

	if (present) {
		FS_INFO("card insert\n");
		subtype = FS_CTRL_MSG_SDCARD_INSERT;
	} else {
		FS_INFO("card remove\n");
		subtype = FS_CTRL_MSG_SDCARD_REMOVE;
	}

	ret = sys_event_send(CTRL_MSG_TYPE_SDCARD, subtype, 0, 0);
	if (ret != 0) {
		FS_ERROR("send fail\n");
	}

	FS_CTRL_UNLOCK();
}
