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

#if (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE))

#include "wlan_debug.h"
#include "wpa_ctrl_req.h"

#include "pm/pm.h"
#include "sys/image.h"
#include "efpg/efpg.h"
#include "lwip/netif.h"
#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"
#include "driver/chip/hal_ccm.h"
#include "driver/chip/hal_prcm.h"
#include "kernel/os/os_semaphore.h"

int wpa_ctrl_request(wpa_ctrl_cmd_t cmd, void *data)
{
	struct ducc_param_wlan_wpa_ctrl_req param;
	param.cmd = cmd;
	param.data = data;

	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_WPA_CTRL_REQUEST, &param);
}

static __inline int wpa_ctrl_open(void)
{
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_WPA_CTRL_OPEN, NULL);
}

static __inline void wpa_ctrl_close(void)
{
	ducc_app_ioctl(DUCC_APP_CMD_WLAN_WPA_CTRL_CLOSE, NULL);
}

int wlan_start(struct netif *nif)
{
	if (ducc_app_ioctl(DUCC_APP_CMD_WLAN_START, nif->state) != 0) {
		return -1;
	}

	return wpa_ctrl_open();
}

/* Note: make sure wlan is disconnect before calling wlan_stop() */
int wlan_stop(void)
{
	if (ducc_app_ioctl(DUCC_APP_CMD_WLAN_STOP, NULL) != 0) {
		return -1;
	}
	wpa_ctrl_close();
	return 0;
}

int wlan_set_mac_addr(uint8_t *mac_addr, int mac_len)
{
	struct ducc_param_wlan_set_mac_addr param;
	param.mac_addr = mac_addr;
	param.mac_len = mac_len;
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_SET_MAC_ADDR, &param);
}

int wlan_set_ip_addr(void *ifp, uint8_t *ip_addr, int ip_len)
{
	struct ducc_param_wlan_set_ip_addr param;
	param.ifp = ifp;
	param.ip_addr = ip_addr;
	param.ip_len = ip_len;
	return ducc_app_ioctl(DUCC_APP_CMD_WLAN_SET_IP_ADDR, &param);
}

/* PM */
#ifdef CONFIG_PM
static int wlan_suspending;

static int wlan_power_notify(enum suspend_state_t state)
{
	return ducc_app_ioctl(DUCC_APP_CMD_POWER_NOTIFY, (void *)state);
}

int wlan_wakeup_net(void)
{
	return wlan_power_notify(PM_MODE_ON);
}

static int wlan_power_callback(int state)
{
	wlan_suspending = 0;

	return 0;
}

static int wlan_sys_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	int err = 0;
	uint32_t _timeout = OS_GetTicks() + OS_SecsToJiffies(3);

	switch (state) {
	case PM_MODE_STANDBY:
		wlan_suspending = 1;
		wlan_power_notify(PM_MODE_ON);
		wlan_power_notify(state);
		while (!HAL_PRCM_IsCPUNDeepSleep() && wlan_suspending &&
		       OS_TimeBefore(OS_GetTicks(), _timeout)) {
			OS_MSleep(5);
		}
		if (OS_TimeAfterEqual(OS_GetTicks(), _timeout) || !wlan_suspending) {
			err = -1;
			break;
		}
		OS_MSleep(2);
		break;
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
		/* step1: notify net cpu to switch to HOSC, turn off SYSCLK2 and enter WFI state. */
		wlan_suspending = 1;
		wlan_power_notify(PM_MODE_POWEROFF);
		while (!HAL_PRCM_IsCPUNSleep() && wlan_suspending) {
			OS_MSleep(5);
		}
		if (!wlan_suspending)
			WLAN_WARN("wlan poweroff faild!\n");
		OS_MSleep(5); /* wait net cpu enter wfi */

		/* step2: writel(0x00, GPRCM_SYS2_CRTL) to reset and isolation network system. */
		HAL_PRCM_EnableSys2Isolation();
		HAL_PRCM_ForceCPUNReset();
		HAL_PRCM_ForceSys2Reset();
		OS_MSleep(5);
		HAL_PRCM_DisableSys2Power();
		WLAN_DBG("%s okay\n", __func__);
		break;
	default:
		break;
	}

	return err;
}

static int wlan_sys_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_STANDBY:
		/* maybe wakeup net at this time better than later by other cmds */
		pm_set_sync_magic();
		wlan_power_notify(PM_MODE_ON);
		wlan_suspending = 0;
		WLAN_DBG("%s okay\n", __func__);
		break;
	default:
		break;
	}

	return 0;
}

static struct soc_device_driver wlan_sys_drv = {
	.name = "wlan_sys",
	.suspend = wlan_sys_suspend,
	.resume = wlan_sys_resume,
};

static struct soc_device wlan_sys_dev = {
	.name = "wlan_sys",
	.driver = &wlan_sys_drv,
};

#define WLAN_SYS_DEV (&wlan_sys_dev)

#else /* CONFIG_PM */

static int wlan_power_callback(int state)
{
	return 0;
}

#define WLAN_SYS_DEV NULL

#endif /* CONFIG_PM */

static OS_Semaphore_t g_ducc_sync_sem; /* use to sync with net system */
static ducc_cb_func g_wlan_net_sys_cb = NULL;

static int wlan_load_net_bin(enum wlan_mode mode)
{
	section_header_t section_header;
	section_header_t *sh = &section_header;
	uint32_t image_id;

	image_id = (mode == WLAN_MODE_HOSTAP) ? IMAGE_NET_AP_ID : IMAGE_NET_ID;

	if ((image_read(image_id, IMAGE_SEG_HEADER, 0, sh, IMAGE_HEADER_SIZE) != IMAGE_HEADER_SIZE)
	    || (image_check_header(sh) == IMAGE_INVALID)
	    || (image_read(image_id, IMAGE_SEG_BODY, 0, (void *)sh->load_addr, sh->body_len) != sh->body_len)
	    || (image_check_data(sh, (void *)sh->load_addr, sh->data_size, NULL, 0) == IMAGE_INVALID)) {
		WLAN_ERR("%s: failed to load net section\n", __func__);
		return -1;
	}

	return 0;
}

static int wlan_get_wlan_bin(int type, int offset, uint8_t *buf, int len)
{
	uint32_t id;
	section_header_t section_header;
	section_header_t *sh = &section_header;

	switch (type) {
	case DUCC_WLAN_BIN_TYPE_BL:
		id = IMAGE_WLAN_BL_ID;
		break;
	case DUCC_WLAN_BIN_TYPE_FW:
		id = IMAGE_WLAN_FW_ID;
		break;
	case DUCC_WLAN_BIN_TYPE_SDD:
		id = IMAGE_WLAN_SDD_ID;
		break;
	default:
		WLAN_ERR("invalid wlan bin type\n");
		return 0;
	}

	if (offset == 0) {
		if (image_read(id, IMAGE_SEG_HEADER, 0, sh, IMAGE_HEADER_SIZE) != IMAGE_HEADER_SIZE) {
			WLAN_ERR("load section (id: %#08x) header failed\n", id);
			return 0;
		}
		if (image_check_header(sh) == IMAGE_INVALID) {
			WLAN_ERR("check section (id: %#08x) header failed\n", id);
			return 0;
		}

		if (len > sh->data_size)
			len = sh->data_size;
	}

	if (image_read(id, IMAGE_SEG_BODY, offset, buf, len) != len) {
		WLAN_ERR("load section (id: %#010x) body failed\n", id);
		return 0;
	}

	if (offset == 0)
		return sh->data_size;
	else
		return len;
}

static int wlan_sys_callback(uint32_t param0, uint32_t param1)
{
	struct ducc_param_wlan_bin *p;
	struct ducc_param_efuse *efuse;

	switch (param0) {
	case DUCC_NET_CMD_SYS_EVENT:
		switch (param1) {
		case DUCC_NET_SYS_READY:
			OS_SemaphoreRelease(&g_ducc_sync_sem);
			break;
		default:
			break;
		}
		break;
	case DUCC_NET_CMD_POWER_NOTIFY:
		wlan_power_callback(param1);
		break;
	case DUCC_NET_CMD_BIN_READ:
		p = (struct ducc_param_wlan_bin *)param1;
		return wlan_get_wlan_bin(p->type, p->index, p->buf, p->len);
	case DUCC_NET_CMD_EFUSE_READ:
		efuse = (struct ducc_param_efuse *)param1;
		return efpg_read_efuse(efuse->data, efuse->start_bit, efuse->bit_num);
	case DUCC_NET_CMD_EFUSE_WRITE:
		efuse = (struct ducc_param_efuse *)param1;
		return efpg_write_efuse(efuse->data, efuse->start_bit, efuse->bit_num);
	default:
		if (g_wlan_net_sys_cb) {
			return g_wlan_net_sys_cb(param0, param1);
		}
		break;
	}
	return 0;
}

int wlan_sys_init(enum wlan_mode mode, ducc_cb_func cb)
{
#ifndef __CONFIG_ARCH_MEM_PATCH
	HAL_PRCM_DisableSys2();
	HAL_PRCM_DisableSys2Power();
	HAL_PRCM_EnableSys2Power();
#endif

	g_wlan_net_sys_cb = cb;
	OS_SemaphoreCreateBinary(&g_ducc_sync_sem);

#ifndef __CONFIG_ARCH_MEM_PATCH
	HAL_PRCM_DisableSys2Isolation();
	HAL_PRCM_ReleaseSys2Reset();
#endif

	if (wlan_load_net_bin(mode) != 0) {
		WLAN_ERR("%s: wlan load net bin failed\n", __func__);

#ifndef __CONFIG_ARCH_MEM_PATCH
		HAL_PRCM_ForceSys2Reset();
		HAL_PRCM_EnableSys2Isolation();
		HAL_PRCM_DisableSys2Power();
		HAL_PRCM_DisableSys2();
#endif

		return -1;
	}

	struct ducc_app_param param = { wlan_sys_callback };
	ducc_app_start(&param);

	HAL_PRCM_ReleaseCPUNReset();
	OS_SemaphoreWait(&g_ducc_sync_sem, OS_WAIT_FOREVER);
	OS_SemaphoreDelete(&g_ducc_sync_sem);
	WLAN_DBG("wlan sys init done\n");

#ifdef CONFIG_PM
	pm_register_ops(WLAN_SYS_DEV);
#endif

	return 0;
}

int wlan_sys_deinit(void)
{
#ifdef CONFIG_PM
	pm_unregister_ops(WLAN_SYS_DEV);
#endif

	HAL_PRCM_ForceCPUNReset();
	WLAN_DBG("wlan sys deinit done\n");

	ducc_app_stop();
	g_wlan_net_sys_cb = NULL;

#ifndef __CONFIG_ARCH_MEM_PATCH
	HAL_PRCM_DisableSys2Isolation();
	HAL_PRCM_ReleaseSys2Reset();
#endif

#ifndef __CONFIG_ARCH_MEM_PATCH
	HAL_PRCM_DisableSys2Power();
	HAL_PRCM_DisableSys2();
#endif

	return 0;
}

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_APP_CORE)) */
