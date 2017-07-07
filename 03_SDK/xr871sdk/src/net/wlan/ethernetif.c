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

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "lwip/tcpip.h"
#include "netif/etharp.h"
#include <string.h>
#include "sys/mbuf.h"
#ifdef __CONFIG_ARCH_DUAL_CORE
#include "sys/ducc/ducc_app.h"
#else
#include "net80211/net80211_ifnet.h"
#endif


#define ETHER_MTU_MAX		1500
#define NETIF_LINK_SPEED_BPS	(100 * 1000 * 1000)
#define NETIF_ATTACH_FLAGS	(NETIF_FLAG_BROADCAST	| \
				 NETIF_FLAG_ETHARP	| \
				 NETIF_FLAG_ETHERNET	| \
				 NETIF_FLAG_IGMP)

struct ethernetif {
	struct netif nif;
	enum wlan_mode mode;
};

#define ethernetif2netif(eth)	((struct netif *)(eth))
#define netif2ethernetif(nif)	((struct ethernetif *)(nif))

static struct ethernetif g_eth_netif;

#if LWIP_NETIF_HOSTNAME
#define NETIF_HOSTNAME_MAX_LEN		32
static char g_netif_hostname[NETIF_HOSTNAME_MAX_LEN];

void ethernetif_set_hostname(char *hostname)
{
	if (hostname != NULL) {
		strlcpy(g_netif_hostname, hostname, NETIF_HOSTNAME_MAX_LEN);
	}
}
#endif /* LWIP_NETIF_HOSTNAME */

static err_t tcpip_null_input(struct pbuf *p, struct netif *nif)
{
	LWIP_DEBUGF(LWIP_XR_DEBUG, ("%s() should not be called\n", __func__));
	pbuf_free(p);
	return ERR_OK;
}

static err_t ethernetif_null_output(struct netif *nif, struct pbuf *p, ip_addr_t *ipaddr)
{
	LWIP_DEBUGF(LWIP_XR_DEBUG, ("%s() should not be called\n", __func__));
	return ERR_IF;
}

static err_t ethernetif_null_linkoutput(struct netif *nif, struct pbuf *p)
{
	LWIP_DEBUGF(LWIP_XR_DEBUG, ("%s() should not be called\n", __func__));
	return ERR_IF;
}

#ifdef __CONFIG_ARCH_DUAL_CORE
/* NB: @p is freed by Lwip. */
static err_t ethernetif_linkoutput(struct netif *nif, struct pbuf *p)
{
	int ret;

#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif
	struct mbuf *m = mb_pbuf2mbuf(p);
#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
	if (m == NULL) {
		LWIP_DEBUGF(LWIP_XR_DEBUG, ("mb_pbuf2mbuf() failed\n"));
		return ERR_MEM;
	}

	struct ducc_param_wlan_linkoutput param;
	param.ifp = nif->state;
	param.mbuf = m;
	ret = ducc_app_ioctl(DUCC_APP_CMD_WLAN_LINKOUTPUT, &param);
	if (ret != 0) {
		LWIP_DEBUGF(LWIP_XR_DEBUG, ("linkoutput failed (%d)\n", ret));
	} else {
		LINK_STATS_INC(link.xmit);
	}
	return ERR_OK;
}

static err_t ethernetif_output(struct netif *nif, struct pbuf *p, ip_addr_t *ipaddr)
{
	if (!netif_is_link_up(nif)) {
		LWIP_DEBUGF(LWIP_XR_DEBUG, ("netif %p is link down\n", nif));
		return ERR_IF;
	}

	return etharp_output(nif, p, ipaddr);
}
#endif /* __CONFIG_ARCH_DUAL_CORE */

/* NB: call by RX task to process received data */
err_t ethernetif_input(struct netif *nif, struct pbuf *p)
{
	err_t err = ERR_MEM;

	do {
		if (p == NULL) {
			LWIP_DEBUGF(LWIP_XR_DEBUG, ("pbuf is NULL\n"));
			LINK_STATS_INC(link.memerr);
			break;
		}
#if ETH_PAD_SIZE
		if (pbuf_header(p, ETH_PAD_SIZE) != 0) {
			/* add padding word for LwIP */
			LWIP_DEBUGF(LWIP_XR_DEBUG, ("pbuf_header(%d) failed!\n", ETH_PAD_SIZE));
			LINK_STATS_INC(link.memerr);
			break;
		}
#endif /* ETH_PAD_SIZE */

		/* send data to LwIP, nif->input() == tcpip_input() */
		err = nif->input(p, nif);
		if (err != ERR_OK) {
			LWIP_DEBUGF(LWIP_XR_DEBUG, ("lwip process data failed, err %d!\n", err));
			LINK_STATS_INC(link.err);
		} else {
			p = NULL; /* pbuf will be freed by LwIP */
		}
	} while (0);

	if (p) {
		pbuf_free(p);
	}

#if LINK_STATS
	if (err == ERR_OK)
		LINK_STATS_INC(link.recv);
	else
		LINK_STATS_INC(link.drop);
#endif /* LINK_STATS */
	return err;
}

static err_t ethernetif_hw_init(struct netif *nif, enum wlan_mode mode)
{
#ifdef __CONFIG_ARCH_DUAL_CORE
	char name[4];
	name[0] = nif->name[0];
	name[1] = nif->name[1];
	name[2] = nif->num + '0';
	name[3] = '\0';

	struct ducc_param_wlan_create param;
	param.mode = mode;
	param.nif = nif;
	param.name = name;
	param.ifp = NULL;
	if (ducc_app_ioctl(DUCC_APP_CMD_WLAN_IF_CREATE, &param) != 0) {
		LWIP_DEBUGF(LWIP_XR_DEBUG, ("wlan interface create failed\n"));
		return ERR_IF;
	}
	nif->state = param.ifp;

	struct ducc_param_wlan_get_mac_addr param2;
	param2.ifp = nif->state;
	param2.buf = nif->hwaddr;
	param2.buf_len = ETHARP_HWADDR_LEN;

	if (ducc_app_ioctl(DUCC_APP_CMD_WLAN_GET_MAC_ADDR, &param2) != ETHARP_HWADDR_LEN) {
		LWIP_DEBUGF(LWIP_XR_DEBUG, ("get mac addr failed\n"));
		ducc_app_ioctl(DUCC_APP_CMD_WLAN_IF_DELETE, nif->state);
		nif->state = NULL;
		return ERR_IF;
	}
#else /* __CONFIG_ARCH_DUAL_CORE */
	nif->state = net80211_ifnet_create(mode, nif);
	if (nif->state == NULL) {
		LWIP_DEBUGF(LWIP_XR_DEBUG, ("wlan interface create failed\n"));
		return ERR_IF;
	}
#endif /* __CONFIG_ARCH_DUAL_CORE */
	return ERR_OK;
}

static void ethernetif_hw_deinit(struct netif *nif)
{
#ifdef __CONFIG_ARCH_DUAL_CORE
	ducc_app_ioctl(DUCC_APP_CMD_WLAN_IF_DELETE, nif->state);
#else
	net80211_ifnet_delete(nif->state);
#endif
}

static err_t ethernetif_init(struct netif *nif, enum wlan_mode mode)
{
	if (mode == WLAN_MODE_STA || mode == WLAN_MODE_HOSTAP) {
		nif->input = tcpip_input;
#ifdef __CONFIG_ARCH_DUAL_CORE
		nif->output = ethernetif_output;
		nif->linkoutput = ethernetif_linkoutput;
#else /* __CONFIG_ARCH_DUAL_CORE */
		nif->output = etharp_output;
		nif->linkoutput = net80211_linkoutput;
#endif /* __CONFIG_ARCH_DUAL_CORE */
	} else if (mode == WLAN_MODE_MONITOR) {
		nif->input = tcpip_null_input;
		nif->output = ethernetif_null_output;
		nif->linkoutput = ethernetif_null_linkoutput;
	} else {
		LWIP_DEBUGF(LWIP_XR_DEBUG, ("mode %d is not support!\n", mode));
		return ERR_ARG;
	}

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	nif->hostname = g_netif_hostname;
#endif /* LWIP_NETIF_HOSTNAME */

	/*
	* Initialize the snmp variables and counters inside the struct netif.
	* The last argument should be replaced with your link speed, in units
	* of bits per second.
	*/
	NETIF_INIT_SNMP(nif, snmp_ifType_ethernet_csmacd, NETIF_LINK_SPEED_BPS);

	nif->name[0] = 'e';
	nif->name[1] = 'n';
	nif->mtu = ETHER_MTU_MAX;
	nif->hwaddr_len = ETHARP_HWADDR_LEN;
	nif->flags |= NETIF_ATTACH_FLAGS;

	/* initialize the hardware */
	return ethernetif_hw_init(nif, mode);
}

static err_t ethernetif_sta_init(struct netif *nif)
{
	return ethernetif_init(nif, WLAN_MODE_STA);
}

static err_t ethernetif_hostap_init(struct netif *nif)
{
	return ethernetif_init(nif, WLAN_MODE_HOSTAP);
}

static err_t ethernetif_monitor_init(struct netif *nif)
{
	return ethernetif_init(nif, WLAN_MODE_MONITOR);
}

struct netif *ethernetif_create(enum wlan_mode mode)
{
	netif_init_fn init_fn;
	netif_input_fn input_fn;
	struct netif *nif;

	if (mode == WLAN_MODE_STA) {
		init_fn = ethernetif_sta_init;
		input_fn = tcpip_input;
	} else if (mode == WLAN_MODE_HOSTAP) {
		init_fn = ethernetif_hostap_init;
		input_fn = tcpip_input;
	} else if (mode == WLAN_MODE_MONITOR) {
		init_fn = ethernetif_monitor_init;
		input_fn = tcpip_null_input;
	} else {
		LWIP_DEBUGF(LWIP_XR_DEBUG, ("mode %d is not support!\n", mode));
		return NULL;
	}

	nif = ethernetif2netif(&g_eth_netif);
	memset(nif, 0, sizeof(*nif));
	g_eth_netif.mode = mode;

	/* add netif */
#if LWIP_NETIF_API
	netifapi_netif_add(nif, NULL, NULL, NULL, NULL, init_fn, input_fn);
	netifapi_netif_set_default(nif);
#else
	netif_add(nif, NULL, NULL, NULL, NULL, init_fn, input_fn);
	netif_set_default(nif);
#endif
	return nif;

}

void ethernetif_delete(struct netif *nif)
{
	/* remove netif from LwIP stack */
	netif_remove(nif);
	dhcp_cleanup(nif);
	nif->flags &= ~NETIF_ATTACH_FLAGS;
	ethernetif_hw_deinit(nif);
}

enum wlan_mode ethernetif_get_mode(struct netif *nif)
{
	if (nif == ethernetif2netif(&g_eth_netif)) {
		return g_eth_netif.mode;
	} else {
		return WLAN_MODE_INVALID;
	}
}
