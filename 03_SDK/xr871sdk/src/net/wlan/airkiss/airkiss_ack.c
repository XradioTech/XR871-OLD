#include "string.h"

#include "kernel/os/os.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip.h"

#include "airkiss.h"
#include "airkiss_ack.h"

static struct netif *wlan_netif;

#define MAX_TIME 4294967295
#define AIRKISS_CONNECT_WIFI_TIME_OUT 30000

#define AIRKISS_ACK_UDP_PORT 10000
#define AIRKISS_ONLINE_ACK_UDP_PORT 12467

static const airkiss_config_t ak_config = {
								(airkiss_memset_fn)&memset,
								(airkiss_memcpy_fn)&memcpy,
								(airkiss_memcmp_fn)&memcmp,
								(airkiss_printf_fn)&printf
							};

char *wechat_public_id = "abcdefg";
char* device_id = "abcedfg";

static void airkiss_device_online_ack(void *arg)
{
	uint8_t ak_online_ack_buf[200];
	uint16_t ak_online_buf_len = 200;
	int socket_id = (int)arg;
	int  ret = airkiss_lan_pack(AIRKISS_LAN_SSDP_NOTIFY_CMD,
								wechat_public_id, device_id,
								0, 0,
								ak_online_ack_buf, &ak_online_buf_len,
								&ak_config);

	if (ret == AIRKISS_LAN_PAKE_READY) {
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));

		addr.sin_port = htons(AIRKISS_ACK_UDP_PORT);
		addr.sin_family = AF_INET;
		if (inet_aton("255.255.255.255", &addr.sin_addr) < 0) {
			printf("%s(), %d, inet_aton error\n", __func__, __LINE__);
			airkiss_online_ack_stop();
			return;
		}

		int tmp = 1;
		int ret = setsockopt(socket_id, SOL_SOCKET, SO_BROADCAST, &tmp, sizeof(int));
		if (ret != 0) {
			printf("%s(), %d, setsockopt error\n", __func__, __LINE__);
			airkiss_online_ack_stop();
			return;
		}
		ret = lwip_sendto(socket_id, ak_online_ack_buf, ak_online_buf_len,
						  0,(const struct sockaddr *)&addr, sizeof(addr));
		if (ret == -1) {
			printf("%s(), %d, udp send error\n", __func__, __LINE__);
			airkiss_online_ack_stop();
		}
	}
}

static OS_Timer_t *ak_online_timer;
#define AK_ONLINE_ACK_PERIOD 5000		//MS
static int ak_online_socket_id = 0;

void airkiss_online_ack_start()
{
	ak_online_socket_id = lwip_socket(AF_INET, SOCK_DGRAM, IP_PROTO_UDP);
	if (ak_online_socket_id < 0) {
		printf("create socket fail.\n");
		return;
	}
	lwip_fcntl(ak_online_socket_id, F_SETFL, O_NONBLOCK);  /* set noblocking */

	OS_Status ret = OS_TimerCreate(ak_online_timer, OS_TIMER_PERIODIC,
                        		   airkiss_device_online_ack,
                        		   (void *)ak_online_socket_id, AK_ONLINE_ACK_PERIOD);
	if (ret != OS_OK) {
		printf("%s(), %d, create os_time error \n", __func__, __LINE__);
		OS_TimerDelete(ak_online_timer);
		return;
	}

	OS_TimerStart(ak_online_timer);
}

void airkiss_online_ack_stop()
{
	if (OS_TimerIsValid(ak_online_timer)) {
	 	OS_TimerStop(ak_online_timer);
		OS_TimerDelete(ak_online_timer);
		lwip_close(ak_online_socket_id);
	}
}

static uint32_t airkiss_d_time(uint32_t start_time, uint32_t os_time)
{
	uint32_t d_time = 0;
	if (start_time <= os_time)
		d_time = os_time - start_time;
	else
		d_time = MAX_TIME - start_time + os_time;
	return d_time;
}

static int airkiss_ack_successful(uint32_t random_num)
{
	uint8_t num[1];
	num[0] = (uint8_t)random_num;

	struct netif *nif = wlan_netif;

	if (nif == NULL)
		return 0;
	if (netif_is_up(nif) && netif_is_link_up(nif)) {
		printf("airkiss ack start\n");
		int ak_ack_socket_id = lwip_socket(AF_INET, SOCK_DGRAM, 0);
		if (ak_ack_socket_id < 0) {
			printf("%s(), %d, create socket fail\n", __func__, __LINE__);
			return 0;
		}

		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));

		addr.sin_port = htons(AIRKISS_ACK_UDP_PORT);
		addr.sin_family = AF_INET;
		if (inet_aton("255.255.255.255", &addr.sin_addr) < 0) {
			printf("%s(), %d, inet_aton error\n", __func__, __LINE__);
			return 0;
		}

		int tmp = 1;
		int ret = setsockopt(ak_ack_socket_id, SOL_SOCKET, SO_BROADCAST, &tmp, sizeof(int));
		if (ret != 0) {
			printf("%s(), %d, setsockopt error\n", __func__, __LINE__);
			lwip_close(ak_ack_socket_id);
			return 0;
		}

		int i = 0;
		for (i = 0; i < 300; i ++) {
			int ret = lwip_sendto(ak_ack_socket_id, num,
								  1, 0,(struct sockaddr *)&addr, sizeof(addr));
			if (ret == -1) {
				printf("%s(), %d, udp send error, %d\n", __func__, __LINE__, OS_GetErrno());
				lwip_close(ak_ack_socket_id);
				return 0;
			}
			OS_MSleep(1);
		}
		lwip_close(ak_ack_socket_id);
		return 1;
	}
	return 0;
}

static OS_Thread_t g_airkiss_ack_thread;
#define AIRKISS_ACK_THREAD_STACK_SIZE	512

void airkiss_ack_thread(void *arg)
{
	uint32_t start_time = OS_JiffiesToMSecs(OS_GetJiffies());
	uint32_t os_time = 0;
	uint32_t random_num = (uint32_t)arg;
	while (1) {
		if (airkiss_ack_successful(random_num))
			break;

		os_time = OS_JiffiesToMSecs(OS_GetJiffies());
		uint32_t d_time = airkiss_d_time(start_time, os_time);
		if (d_time >= AIRKISS_CONNECT_WIFI_TIME_OUT)
			break;

		OS_MSleep(100);
	}
	printf("airkiss ack end\n");
	OS_ThreadDelete(&g_airkiss_ack_thread);
}

void airkiss_ack_start(uint32_t airkiss_random_num, struct netif *netif)
{
	printf("%s(), %d airkiss ask start\n", __func__, __LINE__);
	wlan_netif = netif;
	if (OS_ThreadCreate(&g_airkiss_ack_thread,
						"",
						airkiss_ack_thread,
						(void *)airkiss_random_num,
						OS_THREAD_PRIO_APP,
						AIRKISS_ACK_THREAD_STACK_SIZE) != OS_OK) {
		printf("%s(), %d create airkiss ask thread failed\n", __func__, __LINE__);
	}
}

