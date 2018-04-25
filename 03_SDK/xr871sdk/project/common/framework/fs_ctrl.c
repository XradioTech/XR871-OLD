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
	uint8_t mount_auto_en;
	enum fs_mnt_status mount_status;
} fs_ctrl_private;

static FATFS *fs;
static fs_ctrl_private fs_ctrl;

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
	FRESULT ret;
	int mmc_ret;
	struct mmc_card *card;

	if (mmc_card_create(dev_id) == -1) {
		FS_ERROR("card create fail\n");
		return -1;
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

	if (fs == NULL) {
		fs = malloc(sizeof(FATFS));
		if (fs == NULL) {
			FS_ERROR("no mem\n");
			goto err_fs;
		}
		ret = f_mount(fs, "", 0);
		if(ret != FR_OK) {
			FS_ERROR("mount fail ret:%d\n", ret);
			free(fs);
			fs = NULL;
			goto err_fs;
		} else {
			FS_INFO("mount success\n");
		}
	}

	return 0;

err_fs:
	if (mmc_card_present(card))
		mmc_card_deinit(card);

err_card:
	mmc_card_delete(dev_id, 0);

	return -1;
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
	FRESULT ret;
	int fs_ret = 0;
	struct mmc_card *card;

	if (fs != NULL) {
		ret = f_mount(NULL, "", 0);
		if(ret != FR_OK)
			FS_ERROR("unmount fail ret:%d\n", ret);
		free(fs);
		fs = NULL;
	} else {
		fs_ret = -1;
	}

	card = mmc_card_open(dev_id);
	if (card == NULL) {
		FS_ERROR("card open fail\n");
		if(fs_ret == -1)
			return -1;
		return 0;
	}
	if (mmc_card_present(card)) {
		mmc_card_deinit(card);
	}
	mmc_card_close(dev_id);
	mmc_card_delete(dev_id, 0);

	return fs_ret;
}

static void fs_ctrl_msg_process(uint32_t event, uint32_t data, void *arg)
{
	int ret;
	uint16_t msg = FS_CTRL_MSG_NULL;
	uint32_t param = 0;
	unsigned long flags;

	switch (EVENT_SUBTYPE(event)) {
		case FS_CTRL_MSG_SDCARD_INSERT:
			if (fs_ctrl_mount(FS_MNT_DEV_TYPE_SDCARD, 0) < 0) {
				flags = arch_irq_save();
				fs_ctrl.mount_status = FS_MNT_STATUS_MOUNT_FAIL;
				arch_irq_restore(flags);
				param = FS_MNT_MSG_PARAM(FS_MNT_DEV_TYPE_SDCARD, \
				                         0, FS_MNT_STATUS_MOUNT_FAIL);
			} else {
				flags = arch_irq_save();
				fs_ctrl.mount_status = FS_MNT_STATUS_MOUNT_OK;
				arch_irq_restore(flags);
				param = FS_MNT_MSG_PARAM(FS_MNT_DEV_TYPE_SDCARD, \
				                         0, FS_MNT_STATUS_MOUNT_OK);
			}
			msg = FS_CTRL_MSG_FS_MNT;
			break;
		case FS_CTRL_MSG_SDCARD_REMOVE:
			ret = fs_ctrl_unmount(FS_MNT_DEV_TYPE_SDCARD, 0);
			flags = arch_irq_save();
			fs_ctrl.mount_status = FS_MNT_STATUS_UNMOUNT;
			arch_irq_restore(flags);
			if (ret == 0) {
				msg  = FS_CTRL_MSG_FS_MNT;
				param = FS_MNT_MSG_PARAM(FS_MNT_DEV_TYPE_SDCARD, \
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
}

/*
 * @brief create the sys_ctrl events.
 * @param
 *        @arg none
 * @retval  0 if create success or other if failed.
 */
int fs_ctrl_init(void)
{
	observer_base *base = sys_callback_observer_create(CTRL_MSG_TYPE_SDCARD, \
	                                                   ALL_SUBTYPE, \
	                                                   fs_ctrl_msg_process, \
	                                                   NULL);
	if (base == NULL) {
		FS_ERROR("create fail\n");
		return -1;
	}
	if (sys_ctrl_attach(base) != 0) {
		FS_ERROR("attach fail\n");
		return -1;
	}

	return 0;
}

/*
 * @brief set auto_mount,and requset to mount or unmount FATFS.
 * @param dev_type: type of the device,like SD card.
 * @param dev_id: the ID of the device.
 * @param mode:
 *        @arg FS_MNT_MODE_MOUNT: requset to mount FATFS.
 *		  @arg FS_MNT_MODE_UNMOUNT: requset to unmount FATFS.
 * @param waitMS:to get the result of requesting among waitMS ms.
 * @retval  0 if success or other if failed.
 */
int fs_mount_request(enum fs_mnt_dev_type dev_type, uint32_t dev_id, enum fs_mnt_mode mode, uint32_t waitMS)
{
	int ret;
	uint16_t subtype;
	uint32_t timeout;
	unsigned long flags;

	if (OS_MSecsToTicks(waitMS) > FS_CRTL_TIME_OUT_MAX) {
		FS_ERROR("unvalid argument\n");
		return -1;
	}

	if (mode == FS_MNT_MODE_MOUNT) {
		fs_ctrl.mount_auto_en = 1;
		if (fs_ctrl.mount_status == FS_MNT_STATUS_MOUNT_OK)
			return 0;
		subtype = FS_CTRL_MSG_SDCARD_INSERT;
	} else {
		fs_ctrl.mount_auto_en = 0;
		if (fs_ctrl.mount_status == FS_MNT_STATUS_UNMOUNT)
			return 0;
		subtype = FS_CTRL_MSG_SDCARD_REMOVE;
	}
	flags = arch_irq_save();
	fs_ctrl.mount_status = FS_MNT_STATUS_INVALID;
	arch_irq_restore(flags);

	ret = sys_event_send(CTRL_MSG_TYPE_SDCARD, subtype, 0, 0);
	if (ret != 0) {
		FS_ERROR("send fail\n");
		if (fs_ctrl.mount_status == FS_MNT_STATUS_MOUNT_OK || \
		    fs_ctrl.mount_status == FS_MNT_STATUS_UNMOUNT) {
			return 0;
		}
		return -1;
	}

	timeout = OS_GetTicks() + OS_MSecsToTicks(waitMS);
	while (OS_TimeBefore(OS_GetTicks(), timeout)) {
		if (fs_ctrl.mount_status != FS_MNT_STATUS_INVALID) {
			if (fs_ctrl.mount_status != FS_MNT_STATUS_MOUNT_FAIL) {
				return 0;
			} else {
				return -1;
			}
		}
		OS_MSleep(10);
	}

	return -1;
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

	if (!fs_ctrl.mount_auto_en)
		return;

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
}
