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

#include "sys/ducc/ducc_net.h"
#include "net/wlan/wlan.h"

#include "ducc_os.h"
#include "ducc_debug.h"

void ducc_wlan_event(uint32_t param0, uint32_t param1)
{
	DUCC_WLAN_DBG("%s(), (0x%x, 0x%x)\n", __func__, param0, param1);

	switch (param0) {
	case WLAN_EVENT_CONNECTED:
	case WLAN_EVENT_DISCONNECTED:
	case WLAN_EVENT_SCAN_SUCCESS:
	case WLAN_EVENT_SCAN_FAILED:
	case WLAN_EVENT_4WAY_HANDSHAKE_FAILED:
	case WLAN_EVENT_CONNECT_FAILED:
		ducc_net_ioctl(DUCC_NET_CMD_WLAN_EVENT, (void *)param0);
		break;
	case WLAN_EVENT_SMART_CONFIG_RESULT:
		ducc_net_ioctl(DUCC_NET_CMD_WLAN_SMART_CONFIG_RESULT, (void *)param1);
		break;
	case WLAN_EVENT_AIRKISS_RESULT:
		ducc_net_ioctl(DUCC_NET_CMD_WLAN_AIRKISS_RESULT, (void *)param1);
		break;
	case WLAN_EVENT_DEV_HANG:
		ducc_net_ioctl(DUCC_NET_CMD_WLAN_EVENT, (void *)WLAN_EVENT_DISCONNECTED);
		wlan_dev_reset((void *)param1);
		break;
	default:
		DUCC_WARN("unknown wlan event 0x%x\n", param0);
		break;
	}
}

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE) && defined(__CONFIG_ARCH_NET_CORE)) */
