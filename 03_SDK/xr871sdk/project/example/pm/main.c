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
//#define TEST_STANDBY_DTIM

#ifndef TEST_STANDBY_DTIM
#define TEST_SLEEP
#define TEST_STANDBY
#define TEST_POWEROFF
#endif

int main(void)
{
	platform_init();

	OS_MSleep(2000);

	printf("\n\nPlease connect AP\n");

	while (!(g_wlan_netif && netif_is_up(g_wlan_netif) && \
	         netif_is_link_up(g_wlan_netif))) {
		OS_MSleep(1000);
	}

#ifdef TEST_STANDBY_DTIM
	OS_MSleep(3000);
	while (1) {
		uint32_t wakeup_event = HAL_Wakeup_GetEvent();
		uint32_t end_time;

		printf("wakeup_event:%x\n", wakeup_event);
		if (wakeup_event & PM_WAKEUP_SRC_NETCPU) {
			end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + 60;
			/* maybe disconnect event, wait this event */
			while ((g_wlan_netif && netif_is_up(g_wlan_netif) && \
			        netif_is_link_up(g_wlan_netif))&& \
			        OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {
				OS_MSleep(2);
			}
		}
		while (!(g_wlan_netif && netif_is_up(g_wlan_netif) && \
		        netif_is_link_up(g_wlan_netif))) {
			OS_MSleep(1);
		}
		OS_MSleep(2);
		pm_enter_mode(PM_MODE_STANDBY);
	}
#endif

#ifdef TEST_SLEEP
	printf("System will go to Sleep Mode with wifi connect, "
	       "and wakeup after 10S.\n"
	       " Prepare to test power consumption now.\n");
	HAL_Wakeup_SetTimer_mS(10000);
	//HAL_Wakeup_SetIO(2, 0, 1);
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
	//HAL_Wakeup_SetIO(2, 0, 1);
	pm_enter_mode(PM_MODE_STANDBY);
	printf("wakeup event:%d\n", HAL_Wakeup_GetEvent());
#endif

#ifdef TEST_POWEROFF
	OS_MSleep(2000);
	printf("System will go to Poweroff Mode.\n"
	       " Prepare to test power consumption now.\n");
	//HAL_Wakeup_SetTimer_mS(10000);
	//HAL_Wakeup_SetIO(2, 0, 1);
	pm_enter_mode(PM_MODE_POWEROFF);
#endif

	return 0;
}
