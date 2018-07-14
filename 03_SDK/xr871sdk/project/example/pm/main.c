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
#include <string.h>

#include "kernel/os/os.h"
#include "driver/chip/hal_wakeup.h"
#include "pm/pm.h"

#include "common/framework/net_ctrl.h"
#include "common/framework/platform_init.h"

/* cmds:
 * 1. net sta config ap_ssid [ap_psk]
 * 2. net sta enable
 * 3. netcmd lmac vif0_set_pm_dtim 8
 *    note: 8 should AP DTIM times(if AP DTIM is 3, set vif0_set_pm_dtim to 9)
 * 4. wifi will goto standby in 3S, and goto standby again if wakeup.
 */
#define TEST_STANDBY_DTIM

#ifndef TEST_STANDBY_DTIM
#define TEST_SLEEP
#define TEST_STANDBY
#define TEST_POWEROFF
#endif

#ifdef TEST_STANDBY_DTIM
static uint16_t wlan_event = NET_CTRL_MSG_WLAN_DISCONNECTED;

static void wlan_msg_recv(uint32_t event, uint32_t data, void *arg)
{
	uint16_t type = EVENT_SUBTYPE(event);
	printf("%s msg type:%d\n", __func__, type);

	switch (type) {
	case NET_CTRL_MSG_WLAN_CONNECTED:
		wlan_event = NET_CTRL_MSG_WLAN_CONNECTED;
		break;
	case NET_CTRL_MSG_WLAN_DISCONNECTED:
		wlan_event = NET_CTRL_MSG_WLAN_DISCONNECTED;
		break;
	case NET_CTRL_MSG_WLAN_SCAN_SUCCESS:
	case NET_CTRL_MSG_WLAN_SCAN_FAILED:
	case NET_CTRL_MSG_WLAN_4WAY_HANDSHAKE_FAILED:
	case NET_CTRL_MSG_WLAN_CONNECT_FAILED:
		break;
	case NET_CTRL_MSG_CONNECTION_LOSS:
		wlan_event = WLAN_EVENT_CONNECTION_LOSS;
		break;
	case NET_CTRL_MSG_NETWORK_UP:
		wlan_event = NET_CTRL_MSG_NETWORK_UP;
		break;
	case NET_CTRL_MSG_NETWORK_DOWN:
		wlan_event = NET_CTRL_MSG_NETWORK_DOWN;
		break;
#if (!defined(__CONFIG_LWIP_V1) && LWIP_IPV6)
	case NET_CTRL_MSG_NETWORK_IPV6_STATE:
		break;
#endif
	default:
		printf("unknown msg (%u, %u)\n", type, data);
		break;
	}
}

static int wlan_msg_init(void)
{
	observer_base *ob = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
	                                                 NET_CTRL_MSG_ALL,
	                                                 wlan_msg_recv,
	                                                 NULL);
	if (ob == NULL)
		return -1;
	if (sys_ctrl_attach(ob) != 0)
		return -1;

	return 0;
}
#endif

#define BUTTON_WAKEUP_PORT_DEF GPIO_PORT_A
#define BUTTON_WAKEUP_PIN_DEF  GPIO_PIN_6

#define WAKEUP_IO_PIN_DEF  2    /* PA6 */
#define WAKEUP_IO_MODE_DEF  0
#define WAKEUP_IO_PULL_DEF  1

static void button_irq_cb(void *arg)
{
	printf("button event!\r\n");
}

static void button_init(void)
{
	GPIO_InitParam param;
	GPIO_IrqParam Irq_param;

	param.driving = GPIO_DRIVING_LEVEL_1;
	param.pull = GPIO_PULL_UP;
	param.mode = GPIOx_Pn_F6_EINT;
	HAL_GPIO_Init(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF, &param);

	Irq_param.event = GPIO_IRQ_EVT_FALLING_EDGE;
	Irq_param.callback = button_irq_cb;
	Irq_param.arg = (void *)NULL;
	HAL_GPIO_EnableIRQ(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF, &Irq_param);
}

static void button_deinit(void)
{
	HAL_GPIO_DisableIRQ(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF);
	HAL_GPIO_DeInit(BUTTON_WAKEUP_PORT_DEF, BUTTON_WAKEUP_PIN_DEF);
}

int main(void)
{
	platform_init();

	button_init();

	OS_MSleep(2000);

#ifdef TEST_STANDBY_DTIM
	wlan_msg_init();
#endif

	printf("\n\nPlease connect AP\n");

#if PRJCONF_NET_EN
	while (!(g_wlan_netif && netif_is_up(g_wlan_netif) &&
	         netif_is_link_up(g_wlan_netif))) {
		OS_MSleep(1000);
	}

	wlan_set_ps_mode(g_wlan_netif, 1);
#endif
	OS_MSleep(3000);

#ifdef TEST_STANDBY_DTIM
	wlan_sta_scan_interval(4);
	HAL_Wakeup_SetIO(WAKEUP_IO_PIN_DEF, WAKEUP_IO_MODE_DEF, WAKEUP_IO_PULL_DEF);
	OS_MSleep(3000);
	while (1) {
		uint32_t wakeup_event = HAL_Wakeup_GetEvent();
		uint32_t end_time;

		printf("wakeup_event:%x\n", wakeup_event);
		if (wakeup_event & PM_WAKEUP_SRC_NETCPU) {
			/* Wait wlan event or process recived data.
			 * Need't wait 15ms after data has been processed, if
			 *  you kwow this wakeup by recived data.
			 * Or else wait 15mS to get detail wlan event( maybe
			 *  CONNECTION_LOSS or DISCONNECTED).
			 */
			OS_MSleep(15);
			end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + 180;
			/* maybe disconnect event, wait this event */
			while (((wlan_event != NET_CTRL_MSG_NETWORK_UP) ||
			        ((g_wlan_netif && netif_is_up(g_wlan_netif) &&
			         netif_is_link_up(g_wlan_netif))))&&
			       OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {
				OS_MSleep(2);
			}
		}
		OS_MSleep(2);
		while ((wlan_event != NET_CTRL_MSG_NETWORK_UP) ||
		       (!(g_wlan_netif && netif_is_up(g_wlan_netif) &&
		        netif_is_link_up(g_wlan_netif)))) {
			OS_MSleep(1);
		}
		pm_enter_mode(PM_MODE_STANDBY);
	}
#endif

#ifdef TEST_SLEEP
	printf("System will go to Sleep Mode with wifi connect, "
	       "and wakeup after 10S.\n"
	       " Prepare to test power consumption now.\n");
	HAL_Wakeup_SetTimer_mS(10000);
	//HAL_Wakeup_SetIO(WAKEUP_IO_PIN_DEF, WAKEUP_IO_MODE_DEF, WAKEUP_IO_PULL_DEF);
	pm_enter_mode(PM_MODE_SLEEP);
	printf("wakeup event:%d\n", HAL_Wakeup_GetEvent());
#endif

#ifdef TEST_STANDBY
	OS_MSleep(2000);
	printf("set DTIM to 4/7/10 to get a low power consumption\n");
	printf("System will go to Standby Mode with wifi connect, "
	       "and wakeup after 10S.\n"
	       " Prepare to test power consumption now.\n");
	HAL_Wakeup_SetTimer_mS(10000);
	//HAL_Wakeup_SetIO(WAKEUP_IO_PIN_DEF, WAKEUP_IO_MODE_DEF, WAKEUP_IO_PULL_DEF);
	pm_enter_mode(PM_MODE_STANDBY);
	printf("wakeup event:%d\n", HAL_Wakeup_GetEvent());
#endif

#ifdef TEST_POWEROFF
	OS_MSleep(2000);
	printf("System will go to Poweroff Mode.\n"
	       " Prepare to test power consumption now.\n");
	//HAL_Wakeup_SetTimer_mS(10000);
	//HAL_Wakeup_SetIO(WAKEUP_IO_PIN_DEF, WAKEUP_IO_MODE_DEF, WAKEUP_IO_PULL_DEF);
	pm_enter_mode(PM_MODE_POWEROFF);
#endif

	button_deinit();

	return 0;
}
