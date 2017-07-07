/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *
 *******************************************************************************/
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "MQTTXrRTOS.h"
#include "MQTTDebug.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "errno.h"

#ifdef XR_MQTT_PLATFORM_UTEST
static unsigned int tick;

#ifdef OS_GetTicks
#undef OS_GetTicks
#endif /* OS_GetTicks */
#define OS_GetTicks() (tick)

#endif /* XR_MQTT_PLATFORM_UTEST */

/** countdown_ms - set timeout value in mil seconds
 * @param timer - timeout timer where the timeout value save
 * @param timeout_ms - timeout in timeout_ms mil seconds
 */
void countdown_ms(Timer* timer, unsigned int timeout_ms)
{
	timer->end_time = OS_TicksToMSecs(OS_GetTicks()) + timeout_ms;
}

/** countdown - set timeout value in seconds
 * @param timer - timeout timer where the timeout value save
 * @param timeout - timeout in timeout seconds
 */
void countdown(Timer* timer, unsigned int timeout)
{
	countdown_ms(timer, timeout * 1000);
}

/** left_ms - calculate how much time left before timeout
 * @param timer - timeout timer
 * @return the time left befor timeout, or 0 if it has expired
 */
int left_ms(Timer* timer)
{
	int diff = (int)(timer->end_time) - (int)(OS_TicksToMSecs(OS_GetTicks()));
	return (diff < 0) ? 0 : diff;
}

/** expired - has it already timeouted
 * @param timer - timeout timer
 * @return 0 if it has already timeout, or otherwise.
 */
char expired(Timer* timer)
{
	return 0 <= (int)OS_TicksToMSecs(OS_GetTicks()) - (int)(timer->end_time); /* is time_now over than end time */
}

/** InitTimer - initialize the timer
 * @param timer - timeout timer
 */
void InitTimer(Timer* timer)
{
	timer->end_time = 0;
}

#ifdef PACKET_SPLICE_SIMULATE
static int pkt_splice_force = 100;
#endif

/** xr_rtos_read - read data from network with TCP/IP based on xr_rtos platform
 * @param n - the network has been connected
 * @param buffer - where the data will buffer in
 * @param len - the data length hoped to receive
 * @param timeout_ms - timeouted value to abandon this reading
 * @return the read size, or 0 if timeouted, or -1 if network has been disconnected,
 * @       or -2 if error occured.
 */
static int xr_rtos_read(Network* n, unsigned char *buffer, int len, int timeout_ms)
{
	/* it's a bug fixed version which may cause a blocking even it has been timeouted */
	int recvLen = 0;
	int leftms;
	int rc = -1;
	struct timeval tv;
	Timer timer;
	fd_set fdset;

	MQTT_PLATFORM_ENTRY();

	countdown_ms(&timer, timeout_ms);
	
#ifdef PACKET_SPLICE_SIMULATE
	if ((pkt_splice_force-- < 0) && (len != 1)) {
		pkt_splice_force = 300;
		
		leftms = left_ms(&timer);
		tv.tv_sec = leftms / 1000;
		tv.tv_usec = (leftms % 1000) * 1000;
		
		FD_ZERO(&fdset);
		FD_SET(n->my_socket, &fdset);
		
		rc = select(n->my_socket + 1, &fdset, NULL, NULL, &tv);
		if (rc > 0) {
			rc = recv(n->my_socket, buffer + recvLen, (len - recvLen) / 2, 0);
			if (rc > 0) {
				/* received normally */
				recvLen += rc;
			} else if (rc == 0) {
				/* has disconnected with server */
				recvLen = -1;
			} else {
				/* network error */
				MQTT_PLATFORM_WARN("recv return %d, errno = %d\n", rc, errno);
				recvLen = -2;
			}
		} else if (rc == 0) {
			if (recvLen != 0)
				MQTT_PLATFORM_WARN("received timeout and length had received is %d\n", recvLen);
			/* timeouted and return the length received */
		} else {
			/* network error */
			MQTT_PLATFORM_WARN("select return %d, errno = %d\n", rc, errno);
					recvLen = -2;
				}
	} else 
#endif
	do {
		leftms = left_ms(&timer);
		tv.tv_sec = leftms / 1000;
		tv.tv_usec = (leftms % 1000) * 1000;

		FD_ZERO(&fdset);
		FD_SET(n->my_socket, &fdset);

		rc = select(n->my_socket + 1, &fdset, NULL, NULL, &tv);
		if (rc > 0) {
			rc = recv(n->my_socket, buffer + recvLen, len - recvLen, 0);
			if (rc > 0) {
				/* received normally */
				recvLen += rc;
			} else if (rc == 0) {
				/* has disconnected with server */
				recvLen = -1;
				break;
			} else {
				/* network error */
				MQTT_PLATFORM_WARN("recv return %d, errno = %d\n", rc, errno);
				recvLen = -2;
				break;
			}
		} else if (rc == 0) {
			if (recvLen != 0)
				MQTT_PLATFORM_WARN("received timeout and length had received is %d\n", recvLen);
			/* timeouted and return the length received */
			break;
		} else {
			/* network error */
			MQTT_PLATFORM_WARN("select return %d, errno = %d\n", rc, errno);
			recvLen = -2;
			break;
		}
	} while (recvLen < len && !expired(&timer)); /* expired() is redundant? */
	
	MQTT_PLATFORM_EXIT(recvLen);

	return recvLen;
}

/** xr_rtos_write - write data throught TCP/IP network based on xr_rtos platform
 * @param n - the network has been connected
 * @param buffer - data which need to be written out
 * @param len - the data length hoped to send
 * @param timeout_ms - timeouted value to abandon this writing
 * @return the writed size, or 0 if timeouted, or -1 if network has been disconnected,
 * @       or -2 if error occured.
 */
static int xr_rtos_write(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
	int rc = -1;
	int sentLen = 0;
	fd_set fdset;
	struct timeval tv;

	MQTT_PLATFORM_ENTRY();

	FD_ZERO(&fdset);
	FD_SET(n->my_socket, &fdset);

	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	rc = select(n->my_socket + 1, NULL, &fdset, NULL, &tv);
	if (rc > 0) {
		if ((rc = send(n->my_socket, buffer, len, 0)) > 0)
			sentLen = rc;
		else if (rc == 0)
			sentLen = -1; /* disconnected with server */
		else {
			MQTT_PLATFORM_WARN("send return %d, errno = %d\n", rc, errno);
			sentLen = -2; /* network error */
		}
	} else if (rc == 0)
		sentLen = 0; /* timeouted and sent 0 bytes */
	 else {
	 	MQTT_PLATFORM_WARN("select return %d, errno = %d\n", rc, errno);
		sentLen = -2; /* network error */
	 }

	MQTT_PLATFORM_EXIT(sentLen);

	return sentLen;
}

/** xr_rtos_disconnect - disconnect the nectwork
 * @param n - the network has been connected
 */
static void xr_rtos_disconnect(Network* n)
{
	closesocket(n->my_socket);
}

/** NewNetwork - initialize the network
 * @param n - the network hoped to be connected
 */
void NewNetwork(Network* n)
{
	n->my_socket = 0;
	n->mqttread = xr_rtos_read;
	n->mqttwrite = xr_rtos_write;
	n->disconnect = xr_rtos_disconnect;
}

/** ConnectNetwork - connect the network with destination
 * @param n - the network need to be connected
 * @param addr - the host name
 * @param port - the TCP port
 */
int ConnectNetwork(Network* n, char* addr, int port)
{
	int type = SOCK_STREAM;
	int family = AF_INET;
	struct addrinfo hints = {0, family, type, IPPROTO_TCP, 0, NULL, NULL, NULL};

	int rc = -1;
	struct sockaddr_in address;
	struct addrinfo *result = NULL;

	if ((rc = getaddrinfo(addr, NULL, &hints, &result)) == 0) {
		struct addrinfo *res = result;

		while (res) {
			if (res->ai_family == family)
			{
				result = res;
				break;
			}
			res = res->ai_next;
		}

		if (result->ai_family == family)
		{
			address.sin_port = htons(port);
			address.sin_family = family;
			address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
		}
		else
			rc = -1;

		freeaddrinfo(result);
	}

	if (rc == 0) {
		n->my_socket = socket(family, type, 0);
		if (n->my_socket < 0)
			return -2;

		rc = connect(n->my_socket, (struct sockaddr *)&address, sizeof(address));
		if (rc < 0) {
			MQTT_PLATFORM_WARN("lwip_connect failed, error code = %d\n", errno);
			closesocket(n->my_socket);
			return -3;
		}
	}

	return rc;
}


#if 0
int NetworkConnectTLS(Network *n, char* addr, int port, SlSockSecureFiles_t* certificates, unsigned char sec_method, unsigned int cipher, char server_verify)
{
	SlSockAddrIn_t sAddr;
	int addrSize;
	int retVal;
	unsigned long ipAddress;

	retVal = sl_NetAppDnsGetHostByName(addr, strlen(addr), &ipAddress, AF_INET);
	if (retVal < 0) {
		return -1;
	}

	sAddr.sin_family = AF_INET;
	sAddr.sin_port = sl_Htons((unsigned short)port);
	sAddr.sin_addr.s_addr = sl_Htonl(ipAddress);

	addrSize = sizeof(SlSockAddrIn_t);

	n->my_socket = sl_Socket(SL_AF_INET, SL_SOCK_STREAM, SL_SEC_SOCKET);
	if (n->my_socket < 0) {
		return -1;
	}

	SlSockSecureMethod method;
	method.secureMethod = sec_method;
	retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECMETHOD, &method, sizeof(method));
	if (retVal < 0) {
		return retVal;
	}

	SlSockSecureMask mask;
	mask.secureMask = cipher;
	retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &mask, sizeof(mask));
	if (retVal < 0) {
		return retVal;
	}

	if (certificates != NULL) {
		retVal = sl_SetSockOpt(n->my_socket, SL_SOL_SOCKET, SL_SO_SECURE_FILES, certificates->secureFiles, sizeof(SlSockSecureFiles_t));
		if (retVal < 0)
		{
			return retVal;
		}
	}

	retVal = sl_Connect(n->my_socket, (SlSockAddr_t *)&sAddr, addrSize);
	if (retVal < 0) {
		if (server_verify || retVal != -453) {
			sl_Close(n->my_socket);
			return retVal;
		}
	}

	SysTickIntRegister(SysTickIntHandler);
	SysTickPeriodSet(80000);
	SysTickEnable();

	return retVal;
}
#endif

