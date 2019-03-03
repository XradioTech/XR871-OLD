/**
  * @file  drv_gc0308.h
  * @author  XRADIO IOT WLAN Team
  */

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

#ifndef __GC0308_H__
#define __GC0308_H__

#include "driver/component/component_def.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_csi.h"



#ifdef __cplusplus
	 extern "C" {
#endif



#define GC0308_DBG 1
#define LOG(flags, fmt, arg...)	\
	do {								\
		if (flags) 						\
			printf(fmt, ##arg);		\
	} while (0)

#define DRV_GC0308_DBG(fmt, arg...)	\
			LOG(GC0308_DBG, "[GC0308] "fmt, ##arg)


int Drv_GC0308_EnvironmentInit(void);
void Drv_GC0308_Set_SaveImage_Buff(uint32_t image_buff_addr);
void Drv_GC0308_PowerInit(Cam_PowerCtrlCfg *cfg);
void Drv_GC0308_Reset_Pin_Ctrl(GPIO_PinState state);
void Drv_GC0308_Pwdn_Pin_Ctrl(GPIO_PinState state);
Component_Status Drv_GC0308_Init();
Component_Status Drv_GC0308_Capture_Enable(CSI_CAPTURE_MODE mode , CSI_CTRL ctrl);

uint32_t Drv_GC0308_Capture_Componemt(uint32_t timeout_ms);
void Drv_GC0308_DeInit();

#ifdef __cplusplus
	 }
#endif

#endif /* _OV7670_H_ */
