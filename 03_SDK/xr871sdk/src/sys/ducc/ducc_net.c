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

#if (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_NET_CORE))

#include "sys/ducc/ducc_addr.h"
#include "sys/ducc/ducc_net.h"
#include "sys/ducc/ducc_app.h"
#include "net/wpa_supplicant/wpas.h"
#include "net/wpa_supplicant/wpa_ctrl_req.h"
#include "net/wlan/wlan.h"
#include "net/wlan/wlan_defs.h"
#include "pm/pm.h"

#include "ducc_debug.h"
#include "ducc_os.h"
#include "ducc_wlan_event.h"
#include "ducc_mbox.h"
#include "ducc.h"


/* resouce for normal functions */
#define DUCC_NET_NORMAL_THREAD_PRIO			OS_PRIORITY_ABOVE_NORMAL
#define DUCC_NET_NORMAL_THREAD_STACK_SIZE	(2 * 1024)
static ducc_thread_t g_ducc_net_normal_thread;
static ducc_mutex_t g_ducc_net_normal_mutex;

/* resouce for functions about TX/RX */
#define DUCC_NET_DATA_THREAD_PRIO			OS_PRIORITY_NORMAL
#define DUCC_NET_DATA_THREAD_STACK_SIZE		(2 * 1024)
static ducc_thread_t g_ducc_net_data_thread;
ducc_mutex_t g_ducc_net_data_mutex;

/* marcos for request */
#define DUCC_NET_REQ_SEND(id, r)	ducc_mbox_send(id, r)
#define DUCC_NET_REQ_RECV(id)		ducc_mbox_recv(id, DUCC_WAIT_FOREVER)

#define DUCC_NET_REQ_WAIT(id)		ducc_req_wait(id)
#define DUCC_NET_REQ_RELEASE(id)	ducc_req_release(id)

/* convert pointer from app core to net core */
#define DUCC_NET_PTR(p)	((void *)DUCC_APPMEM_APP2NET(p))

static ducc_cb_func ducc_net_cb = NULL;

#ifdef CONFIG_PM

static uint32_t g_ducc_hw_mbox_suspending = 0;

static int ducc_hw_mbox_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	g_ducc_hw_mbox_suspending = 1;

	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
		ducc_mbox_deinit(DUCC_ID_APP2NET_DATA, 0, 1);
		ducc_mbox_deinit(DUCC_ID_NET2APP_DATA, 1, 1);

		ducc_mbox_deinit(DUCC_ID_APP2NET_NORMAL, 0, 1);
		ducc_mbox_deinit(DUCC_ID_NET2APP_NORMAL, 1, 1);

		DUCC_DBG("%s okay\n", __func__);
		break;
	default:
		break;
	}

	return 0;
}

static int ducc_hw_mbox_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
		break;
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		ducc_mbox_init(DUCC_ID_NET2APP_NORMAL, 1, 1);
		ducc_mbox_init(DUCC_ID_APP2NET_NORMAL, 0, 1);

		ducc_mbox_init(DUCC_ID_NET2APP_DATA, 1, 1);
		ducc_mbox_init(DUCC_ID_APP2NET_DATA, 0, 1);

		__asm(" dsb \n");
		__asm(" isb \n");
		DUCC_DBG("%s okay\n", __func__);
		break;
	default:
		break;
	}

	g_ducc_hw_mbox_suspending = 0;

	return 0;
}

void hw_mbox_print_regs(void)
{
}

static struct soc_device_driver ducc_hw_mbox_drv = {
	.name = "hw_mbox",
	.suspend = ducc_hw_mbox_suspend,
	.resume = ducc_hw_mbox_resume,
};

static struct soc_device ducc_hw_mbox_dev = {
	.name = "hw_mbox",
	.driver = &ducc_hw_mbox_drv,
};

#define DUCC_HW_MBOX_DEV (&ducc_hw_mbox_dev)
#else
#define DUCC_HW_MBOX_DEV NULL
#endif

int ducc_net_ioctl(enum ducc_net_cmd cmd, void *param)
{
	struct ducc_req req;
	ducc_mutex_t *mutex;
	uint32_t send_id, wait_id;

	DUCC_NET_DBG("send req %d\n", cmd);
#ifdef CONFIG_PM
	if (g_ducc_hw_mbox_suspending) {
		DUCC_ERR("send req %d when suspending\n", cmd);
		return -1;
	}
#endif

	if (DUCC_NET_IS_DATA_CMD(cmd)) {
		mutex = &g_ducc_net_data_mutex;
		send_id = DUCC_ID_NET2APP_DATA;
		wait_id = DUCC_ID_APP2NET_DATA;
	} else {
		mutex = &g_ducc_net_normal_mutex;
		send_id = DUCC_ID_NET2APP_NORMAL;
		wait_id = DUCC_ID_APP2NET_NORMAL;
	}

	ducc_mutex_lock(mutex);

#if DUCC_SIMULATE_HW_MBOX
	req.id = send_id;
#endif
	req.cmd = (uint32_t)cmd;
	req.param = (uint32_t)param;
	req.result = -1;

	do {
		if (DUCC_NET_REQ_SEND(send_id, &req) < 0) {
			DUCC_WARN("send req %d failed\n", cmd);
			break;
		}

		DUCC_NET_DBG("wait req %d\n", cmd);
		if (DUCC_NET_REQ_WAIT(wait_id) < 0) {
			DUCC_WARN("wait req %d failed\n", cmd);
			break;
		}
	} while (0);

	DUCC_NET_DBG("wait req %d done\n", cmd);

	ducc_mutex_unlock(mutex);

	return req.result;
}

static int ducc_net_wpa_ctrl_request(struct ducc_param_wlan_wpa_ctrl_req *req)
{
	wpa_ctrl_cmd_t ctrl_cmd = (wpa_ctrl_cmd_t)(req->cmd);
	void *data;
	int ret;

	switch (ctrl_cmd) {
	case WPA_CTRL_CMD_STA_SCAN_RESULTS:
	case WPA_CTRL_CMD_STA_SET:
	case WPA_CTRL_CMD_STA_GET:
	case WPA_CTRL_CMD_STA_STATE:
	case WPA_CTRL_CMD_STA_AP:
	case WPA_CTRL_CMD_STA_WPS_GET_PIN:
	case WPA_CTRL_CMD_STA_WPS_SET_PIN:

	case WPA_CTRL_CMD_AP_SET:
	case WPA_CTRL_CMD_AP_GET:
	case WPA_CTRL_CMD_AP_STA_NUM:
	case WPA_CTRL_CMD_AP_STA_INFO:

		data = DUCC_NET_PTR(req->data); /* data is pointer */
		break;

	case WPA_CTRL_CMD_STA_SCAN:
	case WPA_CTRL_CMD_STA_SCAN_INTERVAL:
	case WPA_CTRL_CMD_STA_REASSOCIATE:
	case WPA_CTRL_CMD_STA_REATTACH:
	case WPA_CTRL_CMD_STA_RECONNECT:
	case WPA_CTRL_CMD_STA_TERMINATE:
	case WPA_CTRL_CMD_STA_DISCONNECT:
	case WPA_CTRL_CMD_STA_ENABLE:
	case WPA_CTRL_CMD_STA_DISABLE:
	case WPA_CTRL_CMD_STA_AUTOCONNECT:
	case WPA_CTRL_CMD_STA_BSS_EXPIRE_AGE:
	case WPA_CTRL_CMD_STA_BSS_EXPIRE_COUNT:
	case WPA_CTRL_CMD_STA_BSS_FLUSH:
	case WPA_CTRL_CMD_STA_WPS_PBC:

	case WPA_CTRL_CMD_AP_ENABLE:
	case WPA_CTRL_CMD_AP_RELOAD:
	case WPA_CTRL_CMD_AP_DISABLE:

	default:
		data = req->data; /* data is not pointer */
		break;
	}

	if (ctrl_cmd == WPA_CTRL_CMD_STA_SCAN_RESULTS) {
		wlan_sta_scan_results_t *results = data;
		if (results->ap) {
			results->ap = (void *)DUCC_APPMEM_APP2NET(results->ap);
			ret = wpa_ctrl_request(ctrl_cmd, data);
			results->ap = (void *)DUCC_APPMEM_NET2APP(results->ap);
		} else {
			return -1;
		}
	} else if (ctrl_cmd == WPA_CTRL_CMD_AP_STA_INFO) {
		wlan_ap_stas_t *stas = data;
		if (stas->sta) {
			stas->sta = (void *)DUCC_APPMEM_APP2NET(stas->sta);
			ret = wpa_ctrl_request(ctrl_cmd, data);
			stas->sta = (void *)DUCC_APPMEM_NET2APP(stas->sta);
		} else {
			return -1;
		}
	} else {
		ret = wpa_ctrl_request(ctrl_cmd, data);
	}

	return ret;
}

static void ducc_net_normal_task(void *arg)
{
	uint32_t recv_id = DUCC_ID_APP2NET_NORMAL;
	uint32_t send_id = DUCC_ID_NET2APP_NORMAL;
	void *app_req;
	struct ducc_req *req;
	uint32_t pm_enter = 0, pm_state = 0;

	while (1) {
		app_req = DUCC_NET_REQ_RECV(recv_id);

		if (app_req == NULL) {
			DUCC_WARN("invalid app req\n");
			continue;
		}

		req = DUCC_NET_PTR(app_req);
#if DUCC_SIMULATE_HW_MBOX
		if (req->id != recv_id) {
			DUCC_WARN("invalid app req, id 0x%x\n", req->id);
			continue;
		}
#endif
		DUCC_NET_DBG("exec req %d\n", req->cmd);

		switch (req->cmd) {
#if (__CONFIG_MBUF_IMPL_MODE == 0)
		case DUCC_APP_CMD_MBUF_GET:
		{
			struct ducc_param_mbuf_get *p = DUCC_NET_PTR(req->param);
			struct mbuf *m = mb_get(p->len, p->tx);
			if (m) {
				MBUF_NET2APP(m);
				p->mbuf = m;
				req->result = 0;
			} else {
				req->result = -1;
			}
			break;
		}
		case DUCC_APP_CMD_MBUF_FREE:
		{
			struct mbuf *m = (struct mbuf *)req->param;
			MBUF_APP2NET(m);
			mb_free(m);
			req->result = 0;
			break;
		}
#endif /* (__CONFIG_MBUF_IMPL_MODE == 0) */
		case DUCC_APP_CMD_CONSOLE_EXEC:
		{
			if (ducc_net_cb)
				ducc_net_cb(req->cmd, (uint32_t)DUCC_NET_PTR(req->param));
			req->result = 0;
			break;
		}
		case DUCC_APP_CMD_POWER_NOTIFY:
			/* never return, so send result here. */
			req->result = 0;
			pm_enter = 1;
			pm_state = req->param;
			break;
		case DUCC_APP_CMD_WLAN_ATTACH:
			req->result = wlan_attach(ducc_wlan_event);
			break;
		case DUCC_APP_CMD_WLAN_DETACH:
			req->result = wlan_detach();
			break;
		case DUCC_APP_CMD_WLAN_IF_CREATE:
		{
			struct ducc_param_wlan_create *p = DUCC_NET_PTR(req->param);
			p->ifp = wlan_if_create(p->mode, p->nif, DUCC_NET_PTR(p->name));
			req->result = p->ifp ? 0 : -1;
			break;
		}
		case DUCC_APP_CMD_WLAN_IF_DELETE:
			req->result = wlan_if_delete((void *)(req->param));
			break;
		case DUCC_APP_CMD_WLAN_START:
			req->result = wlan_start((void *)(req->param));
			break;
		case DUCC_APP_CMD_WLAN_STOP:
			req->result = wlan_stop();
			break;
		case DUCC_APP_CMD_WLAN_GET_MAC_ADDR:
		{
			struct ducc_param_wlan_get_mac_addr *p = DUCC_NET_PTR(req->param);
			req->result = wlan_get_mac_addr(p->ifp,
			                                DUCC_NET_PTR(p->buf),
			                                p->buf_len);
			break;
		}
		case DUCC_APP_CMD_WLAN_SET_MAC_ADDR:
		{
			struct ducc_param_wlan_set_mac_addr *p = DUCC_NET_PTR(req->param);
			req->result = wlan_set_mac_addr(DUCC_NET_PTR(p->mac_addr),
			                                p->mac_len);
			break;
		}
		case DUCC_APP_CMD_WLAN_SET_IP_ADDR:
		{
			struct ducc_param_wlan_set_ip_addr *p = DUCC_NET_PTR(req->param);
			req->result = wlan_set_ip_addr(p->ifp,
			                               DUCC_NET_PTR(p->ip_addr),
			                               p->ip_len);
			break;
		}
		case DUCC_APP_CMD_WLAN_WPA_CTRL_OPEN:
			req->result = wpa_ctrl_open();
			break;
		case DUCC_APP_CMD_WLAN_WPA_CTRL_CLOSE:
			wpa_ctrl_close();
			req->result = 0;
			break;
		case DUCC_APP_CMD_WLAN_WPA_CTRL_REQUEST:
			req->result = ducc_net_wpa_ctrl_request(DUCC_NET_PTR(req->param));
			break;
#ifdef __CONFIG_WLAN_STA
		case DUCC_APP_CMD_WLAN_SMART_CONFIG_START:
			req->result = wlan_smart_config_start((void *)(req->param), WLAN_SMART_CONFIG_CONNECT);
			break;
		case DUCC_APP_CMD_WLAN_SMART_CONFIG_STOP:
			wlan_smart_config_stop(WLAN_SMART_CONFIG_CONNECT);
			req->result = 0;
			break;
		case DUCC_APP_CMD_WLAN_SMART_CONFIG_SET_KEY:
			wlan_smart_config_set_key(DUCC_NET_PTR(req->param), WLAN_SMART_CONFIG_CONNECT);
			req->result = 0;
			break;
		case DUCC_APP_CMD_WLAN_AIRKISS_START:
			req->result = wlan_smart_config_start((void *)(req->param), WLAN_AIRKISS_CONNECT);
			break;
		case DUCC_APP_CMD_WLAN_AIRKISS_STOP:
			wlan_smart_config_stop(WLAN_AIRKISS_CONNECT);
			req->result = 0;
			break;
		case DUCC_APP_CMD_WLAN_AIRKISS_SET_KEY:
			wlan_smart_config_set_key(DUCC_NET_PTR(req->param), WLAN_AIRKISS_CONNECT);
			req->result = 0;
			break;
#endif /* __CONFIG_WLAN_STA */
		default:
			DUCC_WARN("invalid command %d\n", req->cmd);
			break;
		};

		DUCC_NET_DBG("exec req %d done\n", req->cmd);

		DUCC_NET_REQ_SEND(send_id, DUCC_RELEASE_REQ_VAL(send_id));
		if (pm_enter) {
			pm_enter_mode(pm_state);
			pm_enter = 0;
		}
	}

	ducc_thread_exit(&g_ducc_net_normal_thread);
}

static void ducc_net_data_task(void *arg)
{
	uint32_t recv_id = DUCC_ID_APP2NET_DATA;
	uint32_t send_id = DUCC_ID_NET2APP_DATA;
	void *app_req;
	struct ducc_req *req;

	while (1) {
		app_req = DUCC_NET_REQ_RECV(recv_id);

		if (app_req == NULL) {
			DUCC_WARN("invalid app req\n");
			continue;
		}

		req = DUCC_NET_PTR(app_req);
#if DUCC_SIMULATE_HW_MBOX
		if (req->id != recv_id) {
			DUCC_WARN("invalid app req, id 0x%x\n", req->id);
			continue;
		}
#endif
		DUCC_NET_DBG("exec req %d\n", req->cmd);

		switch (req->cmd) {
		case DUCC_APP_CMD_WLAN_LINKOUTPUT:
		{
			struct ducc_param_wlan_linkoutput *p = DUCC_NET_PTR(req->param);
			struct mbuf *m = p->mbuf;
			MBUF_APP2NET(m);
			req->result = wlan_linkoutput(p->ifp, m);
			break;
		}
		default:
			DUCC_WARN("invalid command %d\n", req->cmd);
			break;
		};

		DUCC_NET_DBG("exec req %d done\n", req->cmd);

		DUCC_NET_REQ_SEND(send_id, DUCC_RELEASE_REQ_VAL(send_id));
	}

	ducc_thread_exit(&g_ducc_net_data_thread);
}

int ducc_net_start(struct ducc_net_param *param)
{
	ducc_net_cb = param->cb;

	ducc_mutex_create(&g_ducc_net_normal_mutex);
	ducc_req_init(DUCC_ID_APP2NET_NORMAL);
	ducc_mbox_init(DUCC_ID_NET2APP_NORMAL, 1, 0);
	ducc_mbox_init(DUCC_ID_APP2NET_NORMAL, 0, 0);

	ducc_mutex_create(&g_ducc_net_data_mutex);
	ducc_req_init(DUCC_ID_APP2NET_DATA);
	ducc_mbox_init(DUCC_ID_NET2APP_DATA, 1, 0);
	ducc_mbox_init(DUCC_ID_APP2NET_DATA, 0, 0);

	if (ducc_thread_create(&g_ducc_net_normal_thread,
	                       ducc_net_normal_task,
	                       NULL,
	                       DUCC_NET_NORMAL_THREAD_PRIO,
	                       DUCC_NET_NORMAL_THREAD_STACK_SIZE) != 0) {
		DUCC_ERR("create thread failed\n");
		return -1;
	}

	if (ducc_thread_create(&g_ducc_net_data_thread,
	                       ducc_net_data_task,
	                       NULL,
	                       DUCC_NET_DATA_THREAD_PRIO,
	                       DUCC_NET_DATA_THREAD_STACK_SIZE) != 0) {
		DUCC_ERR("create thread failed\n");
		return -1;
	}
#ifdef CONFIG_PM
	pm_register_ops(DUCC_HW_MBOX_DEV);
#endif

	return 0;
}

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_NET_CORE)) */
