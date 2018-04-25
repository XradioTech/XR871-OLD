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

#ifndef _FS_CTRL_H_
#define _FS_CTRL_H_

#include "common/framework/sys_ctrl/sys_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

enum fs_ctrl_msg_type {
	FS_CTRL_MSG_NULL,
	FS_CTRL_MSG_SDCARD_INSERT,
	FS_CTRL_MSG_SDCARD_REMOVE,
	FS_CTRL_MSG_FS_MNT,

	FS_CTRL_MSG_ALL = ALL_SUBTYPE,
};

enum fs_mnt_dev_type {
	FS_MNT_DEV_TYPE_SDCARD,
};

enum fs_mnt_mode {
	FS_MNT_MODE_MOUNT,
	FS_MNT_MODE_UNMOUNT,
};

enum fs_mnt_status {
	FS_MNT_STATUS_INVALID = 0,
	FS_MNT_STATUS_MOUNT_OK,
	FS_MNT_STATUS_MOUNT_FAIL,
	FS_MNT_STATUS_UNMOUNT,
};

#define FS_MNT_MSG_PARAM(dev_type, dev_id, status) \
    ((((dev_type) & 0xFF) << 16) | (((dev_id) & 0xFF) << 8) | ((status) & 0xFF))
#define FS_MNT_DEV_TYPE(param)  (((param) >> 16) & 0xFF)
#define FS_MNT_DEV_ID(param)    (((param) >> 8) & 0xFF)
#define FS_MNT_STATUS(param)    ((param) & 0xFF)

/**
 * @brief create the sys_ctrl events.
 * @param
 *        @arg none
 * @retval  0 if create success or other if failed.
 */
int fs_ctrl_init(void);

/**
 * @brief set auto_mount,and requset to mount or unmount FATFS.
 * @param dev_type: type of the device,like SD card.
 * @param dev_id: the ID of the device.
 * @param mode:
 *        @arg FS_MNT_MODE_MOUNT: requset to mount FATFS.
 *		  @arg FS_MNT_MODE_UNMOUNT: requset to unmount FATFS.
 * @param waitMS:to get the result of requesting among waitMS ms.
 * @retval  0 if success or other if failed.
 */
int fs_mount_request(enum fs_mnt_dev_type dev_type, uint32_t dev_id, enum fs_mnt_mode mode, uint32_t waitMS);

/**
 * @brief sdcard detect callback.
 * @param present:
 *        @arg present 1-card insert, 0-card remove
 * @retval  none
 */
void sdcard_detect_callback(uint32_t present);

#ifdef __cplusplus
}
#endif

#endif /* _FS_CTRL_H_ */
