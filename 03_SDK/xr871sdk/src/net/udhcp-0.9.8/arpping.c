/*
 * arpping.c
 *
 * Mostly stolen from: dhcpcd - DHCP client daemon
 * by Yoichi Hariguchi <yoichi@fore.com>
 */
#ifdef DHCPD_LWIP
#include <lwip/sockets.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/if_ether.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#endif

#ifndef DHCPD_TIMEALT
#include <sys/time.h>
#else
#include "dhcp_time.h"
#endif

#include <stdio.h>
#include <string.h>
//#include <unistd.h>

#include "dhcpd.h"
#include "debug.h"
#include "arpping.h"

/* args:	yiaddr - what IP to ping
 *		ip - our ip
 *		mac - our arp address
 *		interface - interface to use
 * retn: 	1 addr free
 *		0 addr used
 *		-1 error
 */

#ifdef DHCPD_LWIP
#define ETH_P_ARP		ETHTYPE_ARP
#define ARPHRD_ETHER		1
#define ETH_P_IP		ETHTYPE_IP

#define ARPOP_REQUEST		ARP_REQUEST
#endif


/* FIXME: match response against chaddr */
int arpping(u_int32_t yiaddr, u_int32_t ip, unsigned char *mac, char *interface)
{

	int	timeout = 2;
	int 	optval = 1;
	int	s;			/* socket */
	int	rv = 1;			/* return value */
	//struct sockaddr addr;		/* for interface name */

	struct arpMsg	arp;
	fd_set		fdset;
	struct timeval	tm;
	time_t		prevTime;

	struct sockaddr_ll sl;

	if ((s = socket (AF_PACKET, SOCK_RAW, htons(ETH_P_ARP))) == -1) {
		DHCPD_LOG(LOG_ERR, "arp:Could not open raw socket");
		return -1;
	}

	if (setsockopt(s, SOL_SOCKET, SO_BROADCAST, &optval, sizeof(optval)) == -1) {
		DHCPD_LOG(LOG_ERR, "arp:Could not setsocketopt on raw socket");
#ifdef DHCPD_LWIP
		closesocket(s);
#else
		close(s);
#endif
		return -1;
	}
	DHCPD_LOG(LOG_INFO, "arp : check ip.");
	/* send arp request */
	memset(&arp, 0, sizeof(arp));
#ifdef DHCPD_LWIP
	memcpy(arp.ethhdr.dest.addr, MAC_BCAST_ADDR, 6);	/* MAC DA */
	memcpy(arp.ethhdr.src.addr, mac, 6);		/* MAC SA */
	arp.ethhdr.type = htons(ETH_P_ARP);		/* protocol type (Ethernet) */
#else
	memcpy(arp.ethhdr.h_dest, MAC_BCAST_ADDR, 6);	/* MAC DA */
	memcpy(arp.ethhdr.h_source, mac, 6);		/* MAC SA */
	arp.ethhdr.h_proto = htons(ETH_P_ARP);		/* protocol type (Ethernet) */
#endif

	arp.htype = htons(ARPHRD_ETHER);		/* hardware type */
	arp.ptype = htons(ETH_P_IP);			/* protocol type (ARP message) */
	arp.hlen = 6;					/* hardware address length */
	arp.plen = 4;					/* protocol address length */
	arp.operation = htons(ARPOP_REQUEST);		/* ARP op code */

#ifndef DHCPD_LWIP
	*((u_int *) arp.sInaddr) = ip;			/* source IP address */
#else
	memcpy(arp.sInaddr,(char *)&ip, 4);
#endif
	memcpy(arp.sHaddr, mac, 6);			/* source hardware address */
#ifndef DHCPD_LWIP
	*((u_int *) arp.tInaddr) = yiaddr;		/* target IP address */
#else
	memcpy(arp.tInaddr,(char *)&yiaddr, 4);
#endif
	//memset(&addr, 0, sizeof(addr));
	//strcpy(addr.sa_data, interface);
	//if (sendto(s, &arp, sizeof(arp), 0, &addr, sizeof(addr)) < 0)
	memset(&sl, 0, sizeof(sl));
	sl.sll_family = AF_PACKET;
    	//sl.sll_addr = MAC_SOURCE;
    	//sl.sll_halen = ETH_ALEN;
	sl.sll_ifindex = 0x2;
	if (sendto(s, &arp, sizeof(arp), 0, (struct sockaddr*)&sl, sizeof(sl)) < 0)
		rv = 0;

	/* wait arp reply, and check it */
	tm.tv_usec = 0;
	time(&prevTime);

	while (timeout > 0) {
		FD_ZERO(&fdset);
		FD_SET(s, &fdset);
		tm.tv_sec = timeout;
		if (select(s + 1, &fdset, (fd_set *) NULL, (fd_set *) NULL, &tm) < 0) {
			DEBUG(LOG_ERR, "Error on ARPING request: %s", strerror(errno));
			if (errno != EINTR) rv = 0;
		} else if (FD_ISSET(s, &fdset)) {
			if (recv(s, &arp, sizeof(arp), 0) < 0 ) rv = 0;
			u_int *tsInaddr = (u_int *)arp.sInaddr;

			if (arp.operation == htons(ARP_REPLY) &&
			    bcmp(arp.tHaddr, mac, 6) == 0 &&
			    *(/*(u_int *) arp.sInaddr*/tsInaddr) == yiaddr) {

				DEBUG(LOG_INFO, "Valid arp reply receved for this address");
				rv = 0;
				break;
			}
		}
		timeout -= time(NULL) - prevTime;
		time(&prevTime);

	}
#ifdef DHCPD_LWIP
	closesocket(s);
#else
	close(s);
#endif
	DEBUG(LOG_INFO, "%salid arp replies for this address", rv ? "No v" : "V");
	return rv;
}
