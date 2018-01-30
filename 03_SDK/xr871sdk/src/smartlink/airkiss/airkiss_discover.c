#include "string.h"

#include "kernel/os/os.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/ip.h"

#include "smartlink/airkiss/wlan_airkiss.h"
#include "airkiss.h"

#define g_debuglevel  ERROR

#define AIRKISS_LAN_PORT 12476

static void airkiss_device_online_ack(airkiss_priv_t *priv, int socket_id)
{
	uint16_t ak_online_buf_len = AIRKISS_ONLINE_BUF_LEN;
	int ret;
	Airkiss_Online_Ack_Info *info = &priv->ack_info;

	AIRKISS_DBG(INFO, "into %s\n", __func__);

	ret = airkiss_lan_pack(AIRKISS_LAN_SSDP_NOTIFY_CMD, \
	                       info->app_id, info->device_id, \
	                       0, 0, \
	                       priv->online_ack_buf, &ak_online_buf_len, \
	                       &priv->func);

	if (ret == AIRKISS_LAN_PAKE_READY) {
		struct sockaddr_in addr;
		int tmp = 1;

		memset(&addr, 0, sizeof(addr));
		addr.sin_port = htons(AIRKISS_LAN_PORT);
		addr.sin_family = AF_INET;
		if (inet_aton("255.255.255.255", &addr.sin_addr) < 0) {
			AIRKISS_DBG(ERROR, "%s,%d, inet_aton error!\n", __func__, __LINE__);
			return;
		}

		ret = setsockopt(socket_id, SOL_SOCKET, SO_BROADCAST, &tmp, sizeof(int));
		if (ret != 0) {
			AIRKISS_DBG(ERROR, "%s,%d, setsockopt error!\n", __func__, __LINE__);
			return;
		}
		ret = lwip_sendto(socket_id, priv->online_ack_buf, ak_online_buf_len,
		                  0,(const struct sockaddr *)&addr, sizeof(addr));
		if (ret == -1)
			AIRKISS_DBG(ERROR, "%s,%d, udp send error!\n", __func__, __LINE__);
	}
}

static void airkiss_cycle_ack_task(void *param)
{
	airkiss_priv_t *priv = (airkiss_priv_t *)param;
	Airkiss_Online_Ack_Info *info = &priv->ack_info;
	int socketfd;
	struct sockaddr_in addr;

	priv->cycle_ack_run |= AK_TASK_RUN;

	socketfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketfd < 0) {
		AIRKISS_DBG(ERROR, "%s,%d create sock error!\n", __func__, __LINE__);
		goto err_out;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family=AF_INET;
	addr.sin_port= htons(AIRKISS_LAN_PORT);
	addr.sin_addr.s_addr=htonl(INADDR_ANY);

	if (bind(socketfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		AIRKISS_DBG(ERROR, "%s,%d bind sock error!\n", __func__, __LINE__);
		goto out;
	}

	while (!(priv->cycle_ack_run & AK_TASK_STOP)) {
		airkiss_device_online_ack(priv, socketfd);
		OS_MSleep(info->ack_period_ms);
	}

out:
	closesocket(socketfd);
err_out:
	priv->cycle_ack_run = 0;
	/* since lan discovery should be send always, so we will never reach this */
	OS_ThreadDelete(&priv->cycle_ack_thread);
}

static int
airkiss_send_active_lan_discovery_packets(airkiss_priv_t *priv, int client_socket_fd)
{
	int ret = -1;
	struct sockaddr_in to_addr;
	uint16_t lan_buf_len = AIRKISS_LAN_BUF_LEN;
	int tmp;
	Airkiss_Online_Ack_Info *info = &priv->ack_info;

	ret = airkiss_lan_pack(AIRKISS_LAN_SSDP_NOTIFY_CMD, \
	                       info->app_id, info->device_id, \
	                       0, 0, \
	                       priv->lan_buf, &lan_buf_len, \
	                       &priv->func);
	if (ret != AIRKISS_LAN_PAKE_READY) {
		AIRKISS_DBG(ERROR, "airkiss pack lan packet error!\n");
		return -1;
	}

	FD_ZERO(&to_addr);
	to_addr.sin_family = AF_INET;
	to_addr.sin_port = htons(AIRKISS_LAN_PORT);
	to_addr.sin_addr.s_addr =inet_addr("255.255.255.255");

	tmp = 1;
	ret = setsockopt(client_socket_fd, SOL_SOCKET, SO_BROADCAST, &tmp, sizeof(int));
	if (ret != 0) {
		AIRKISS_DBG(ERROR, "%s,%d, setsockopt error!\n", __func__, __LINE__);
		return -1;
	}

	ret = sendto(client_socket_fd, (unsigned char *)priv->lan_buf, lan_buf_len, 0, \
	             (struct sockaddr *) &to_addr, sizeof(struct sockaddr));
	if (ret == -1) {
		AIRKISS_DBG(ERROR, "%s,%d UDP send error!\n", __func__, __LINE__);
		return -1;
	}

	return 0;
}

static void
airkiss_lan_server_reply(airkiss_priv_t *priv, int client_socket_fd, struct sockaddr_in addr, \
                         char *pdata, unsigned short len)
{
	airkiss_lan_ret_t ret = -1;
	airkiss_lan_ret_t pack_ret;
	uint16_t lan_buf_len = AIRKISS_LAN_BUF_LEN;
	Airkiss_Online_Ack_Info *info = &priv->ack_info;

	ret = airkiss_lan_recv(pdata, len, &priv->func);

	switch (ret) {
	case AIRKISS_LAN_SSDP_REQ:
		addr.sin_port = htons(AIRKISS_LAN_PORT);
		lan_buf_len = AIRKISS_LAN_BUF_LEN;
		pack_ret = airkiss_lan_pack(AIRKISS_LAN_SSDP_RESP_CMD,
		                            info->app_id, info->device_id, \
		                            0, 0, \
		                            priv->lan_buf, &lan_buf_len, \
		                            &priv->func);
		if (pack_ret != AIRKISS_LAN_PAKE_READY) {
			AIRKISS_DBG(ERROR, "%s,%d Pack lan packet error!\n",
			            __func__, __LINE__);
			return;
		}

		AIRKISS_DBG(ERROR, "AIRKISS_LAN_SSDP_REQ !\n");
		ret = sendto(client_socket_fd, \
		             (unsigned char *)priv->lan_buf, lan_buf_len, \
		             0, (struct sockaddr *) &addr, sizeof(struct sockaddr));
		if (ret != 0) {
			AIRKISS_DBG(ERROR, "lan_server_reply sendto ret=%d\n", ret);
		}
		break;
	default:
		AIRKISS_DBG(ERROR,"Pack is not ssdq req!\n");
		break;
	}
}

static void airkiss_online_dialog_task(void *arg)
{
	airkiss_priv_t *priv = (airkiss_priv_t *)arg;
	Airkiss_Online_Ack_Info *info = &priv->ack_info;
	int server_sock_fd,len;
	struct sockaddr_in addr;
	int sock_timeout_val = 1000; /* 1000 ms */
	int addr_len = sizeof(struct sockaddr_in);
	char buffer[256] = {0};

	priv->dialog_run |= AK_TASK_RUN;

	if ((server_sock_fd = socket(AF_INET,SOCK_DGRAM,0)) < 0){
		AIRKISS_DBG(ERROR, "create sock error!\n");
		goto err_out;
	}

	memset(&addr, 0, sizeof(addr));
	addr.sin_family=AF_INET;
	addr.sin_port = htons(AIRKISS_LAN_PORT);
	addr.sin_addr.s_addr=htonl(INADDR_ANY) ;

	if (bind(server_sock_fd, (struct sockaddr *)&addr, sizeof(addr))<0) {
		AIRKISS_DBG(ERROR, "bind sock error!\n");
		goto out;
	}

	int ret = setsockopt(server_sock_fd, SOL_SOCKET, SO_RCVTIMEO, \
	                     &sock_timeout_val, sizeof(sock_timeout_val));
	if (ret != 0) {
		AIRKISS_DBG(ERROR, "%s,%d, setsockopt error!\n", __func__, __LINE__);
	}

	memset(&buffer, 0, sizeof(buffer));
	len = recvfrom(server_sock_fd, buffer, sizeof(buffer), 0, \
	               (struct sockaddr *)&addr , (socklen_t *)&addr_len);

	if (len != -1) {
		airkiss_lan_server_reply(priv, server_sock_fd, addr, buffer, len);
	}

	while (!(priv->dialog_run & AK_TASK_STOP)) {
		if (airkiss_send_active_lan_discovery_packets(priv, server_sock_fd) == -1)
			break;
		OS_MSleep(info->ack_period_ms);
	}

out:
	closesocket(server_sock_fd);
err_out:
	priv->dialog_run = 0;
	OS_ThreadDelete(&priv->online_dialog_thread);
}

int wlan_airkiss_online_cycle_ack_start(char *app_id, char *drv_id, uint32_t period_ms)
{
	airkiss_priv_t *priv = airkiss_priv;

	priv->ack_info.app_id = app_id;
	priv->ack_info.device_id = drv_id;
	priv->ack_info.ack_period_ms = period_ms;
	priv->cycle_ack_run |= AK_TASK_RUN;

	if (OS_ThreadCreate(&priv->cycle_ack_thread,
			    "",
			    airkiss_cycle_ack_task,
			    (void *)priv,
			    OS_THREAD_PRIO_APP,
			    AIRKISS_CYCLE_ACK_THREAD_STACK_SIZE) != OS_OK) {
		AIRKISS_DBG(ERROR, "%s,%d create airkiss ask thread failed!\n",
		            __func__, __LINE__);
		return -1;
	}

	return 0;
}

int wlan_airkiss_online_cycle_ack_stop(void)
{
	airkiss_priv_t *priv = airkiss_priv;

	priv->cycle_ack_run |= AK_TASK_STOP;

	while (priv->cycle_ack_run & AK_TASK_RUN)
		OS_MSleep(10);
	return 0;
}

int wlan_airkiss_online_dialog_mode_start(char *app_id, char *drv_id, uint32_t period_ms)
{
	airkiss_priv_t *priv = airkiss_priv;

	priv->ack_info.app_id = app_id;
	priv->ack_info.device_id = drv_id;
	priv->ack_info.ack_period_ms = period_ms;

	priv->dialog_run |= AK_TASK_RUN;

	if (OS_ThreadCreate(&priv->online_dialog_thread,
	                    "",
	                    airkiss_online_dialog_task,
	                    (void *)priv,
	                    OS_THREAD_PRIO_APP,
	                    AIRKISS_ONLINE_DIALOG_THREAD_STACK_SIZE) != OS_OK) {
		AIRKISS_DBG(ERROR, "%s,%d create airkiss ask thread failed!\n",
		            __func__, __LINE__);
		return -1;
	}

	return 0;
}

int wlan_airkiss_online_dialog_mode_stop(void)
{
	airkiss_priv_t *priv = airkiss_priv;

	priv->dialog_run |= AK_TASK_STOP;

	while (priv->dialog_run & AK_TASK_RUN)
		OS_MSleep(10);

	return 0;
}
