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




/************************demo********************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <lwip/sockets.h>
#include <lwip/err.h>
#include <lwip/sys.h>

#include "kernel/os/os.h"
#include "driver/chip/hal_i2c.h"
#include "driver/chip/hal_dma.h"
#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_csi.h"

#include "common/framework/net_ctrl.h"
#include "common/framework/platform_init.h"


#include "driver/component/csi_camera/camera_csi.h"

#include "driver/component/csi_camera/gc0308/drv_gc0308.h"



#define IMAGE_BUFFSIZE       153600

#define CAM_RESET_PIN        GPIO_PIN_13
#define CAM_RESET_PORT       GPIO_PORT_A

#define CAM_POWERDOWN_PIN    GPIO_PIN_12
#define CAM_POWERDOWN_PORT   GPIO_PORT_A

enum serverstatus_type {
	SERVER_CONNECTED_IDLE,
	SERVER_CONNECTED_OK
};

#define	SERVER_PORT		50000
#define SERVER_IP       "192.168.1.3"



uint8_t *image_buff;

#define THREAD_STACK_SIZE_WlanControl	(3 * 1024)
static OS_Thread_t threadWlanControl;


static uint16_t wlan_event = NET_CTRL_MSG_WLAN_DISCONNECTED;




int i_socketFd;
int i_wlanstatus = SERVER_CONNECTED_IDLE;


void Cam_Hardware_Reset(void)
{
	printf("--%s---%d----\n", __func__, __LINE__);

	Drv_GC0308_Pwdn_Pin_Ctrl(GPIO_PIN_LOW);
	Drv_GC0308_Reset_Pin_Ctrl(GPIO_PIN_LOW);
	OS_MSleep(3);
	Drv_GC0308_Reset_Pin_Ctrl(GPIO_PIN_HIGH);
	OS_MSleep(100);
}


void Cam_PowerInit(void)
{
	Cam_PowerCtrlCfg PowerCtrlcfg;
	PowerCtrlcfg.Cam_Pwdn_Port = CAM_POWERDOWN_PORT;
	PowerCtrlcfg.Cam_Reset_Port = CAM_RESET_PORT;

	PowerCtrlcfg.Cam_Pwdn_Pin = CAM_POWERDOWN_PIN; //开发板
	PowerCtrlcfg.Cam_Reset_Pin = CAM_RESET_PIN;


	Drv_GC0308_PowerInit(&PowerCtrlcfg);

	Drv_GC0308_EnvironmentInit();
}


int Cam_Init(uint8_t *imagebuf)
{
	HAL_CSI_Moudle_Enalbe(CSI_DISABLE);
	if (Drv_GC0308_Init() == COMP_ERROR)
		return COMP_ERROR;
	else
		OS_MSleep(500);

	Drv_GC0308_Set_SaveImage_Buff((uint32_t)imagebuf);
	HAL_CSI_Moudle_Enalbe(CSI_ENABLE);
	return COMP_OK;
}







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



static void TaskWlanControl(void *arg)
{
	uint16_t old_event = WLAN_EVENT_DISCONNECTED;

	wlan_msg_init();

	printf("--%s---%d----\n", __func__, __LINE__);
    while (1) {
		if(old_event != wlan_event)
		{
			old_event = wlan_event;

			printf("%s msg type:%d\n", __func__, old_event);
			if(old_event == NET_CTRL_MSG_NETWORK_UP)
			{
				struct sockaddr_in server_addr;
			    int revel;

			    i_socketFd = socket(AF_INET, SOCK_STREAM, 0);
			    if (i_socketFd == -1) {
			        printf("failed to create sock_fd!\n");
			    }

				printf("--%s---%d----\n", __func__, __LINE__);
			    server_addr.sin_family = AF_INET;
			    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
			    server_addr.sin_port = htons(SERVER_PORT);

			    while(1) {
					printf("--%s---%d----\n", __func__, __LINE__);
			        revel = connect(i_socketFd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr));
			        if(revel < 0)
			            printf("Cennect failed! revel = %d\n", revel);

			        else {
			            printf("Cennect OK!\n");
						i_wlanstatus = SERVER_CONNECTED_OK;
			            break;
			        }
					if(wlan_event == NET_CTRL_MSG_NETWORK_DOWN)
						break;
			        OS_MSleep(1000);

			    }
			}
			else if(old_event == NET_CTRL_MSG_NETWORK_DOWN)
			{
				if (i_socketFd != -1) {
			        closesocket(i_socketFd);
					i_socketFd = -1;
					i_wlanstatus = SERVER_CONNECTED_IDLE;
			    }
			}
		}
		OS_MSleep(10);
    }
}



int main(void)
{
	uint32_t image_size = 0;
	uint32_t len, count;
	uint8_t *pbuf;

	platform_init();


	if (OS_ThreadCreate(&threadWlanControl,
                        "TaskWlanControl",
                        TaskWlanControl,
                        NULL,
                        OS_THREAD_PRIO_APP,
                        THREAD_STACK_SIZE_WlanControl) != OS_OK) {
        printf("create TaskEnterStandby failed\n");
        return -1;
    }


	image_buff = (uint8_t *) malloc(IMAGE_BUFFSIZE);
	if (image_buff == NULL) {
		COMPONENT_WARN("image buff malloc error \r\n");
		return COMP_ERROR;
	}
	memset(image_buff, 0, IMAGE_BUFFSIZE);



	Cam_PowerInit();
	Cam_Hardware_Reset();
	Cam_Init(image_buff);


	while(1)
	{
		Drv_GC0308_Capture_Enable(CSI_STILL_MODE, CSI_ENABLE);
		image_size = Drv_GC0308_Capture_Componemt(10000);
		//printf("[GC0308] image_size %u\n", image_size);
		if(image_size == 320*240*2) {
			if((i_wlanstatus == SERVER_CONNECTED_OK) && (i_socketFd != -1)){
				count = 0;
				len = 0;
				pbuf = image_buff;
				while(1)
				{
					len = send(i_socketFd, pbuf, 1024, 0);
					count += len;
					if (count >= image_size) {
						printf("Socket Send OK!\r\n");
						break;
					}
					pbuf += len;
				}
			}
		}
	}

	Drv_GC0308_DeInit();
	free(image_buff);

	return COMP_OK;
}
