/**
  * @file  drv_tcs4056.h
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

#ifndef _TCS4056_H_
#define _TCS4056_H_

#include "driver/component/component_def.h"

#ifdef __cplusplus
	 extern "C" {
#endif

/**
  * @brief The status of chrg interrupt.
  */
typedef enum {
	CHRG_LOW_LEVEL,
	CHRG_HIGH_LEVEL,
	CHRG_NULL,
}CHRG_IRQ_STA;

/**
  * @brief The callback for chrg interrupt.
  */
typedef struct {
	void (*chrgCallBack) (void *arg, CHRG_IRQ_STA sta);
	void *arg;
}ChrgIrq;

/**
  * @brief The io that uset for chrg.
  */
typedef struct {
	GPIO_Port chrgPort;
	GPIO_Pin chrgPin;
}ChrgIo;

Component_Status DRV_Tcs4056_Init(ChrgIo *chrg_io, ADC_Channel ch);
void DRV_ChrgCallBackRegister(ChrgIrq *cb);
void DRV_Enable_Chrg_Irq();
void DRV_Disable_Chrg_Irq();
uint32_t DRV_Read_Bat_Voltage();
void DRV_Tcs4056_DeInit();

void Tcs4056_Test();

#ifdef __cplusplus
	 }
#endif

#endif /* _TCS4056_H_ */
