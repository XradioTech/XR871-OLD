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

#ifndef _DRIVER_CHIP_HAL_WAKEUP_H_
#define _DRIVER_CHIP_HAL_WAKEUP_H_

#include "driver/chip/hal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ignore the second set if the second set time is longer than the first time */
//#define WAKEUP_TIMER_CHECK_TIME 1

/* the minimum time of wakeup timer support, based on ms */
#define WAKEUP_TIMER_MIN_TIME   (1)

/* wakeup events.
 * NOTE: WKIO0~9 should define from 1<<0 to 1<<9 */
#ifdef __CONFIG_ARCH_APP_CORE
#define PM_WAKEUP_SRC_WKIO0     (1<<0)
#define PM_WAKEUP_SRC_WKIO1     (1<<1)
#define PM_WAKEUP_SRC_WKIO2     (1<<2)
#define PM_WAKEUP_SRC_WKIO3     (1<<3)
#define PM_WAKEUP_SRC_WKIO4     (1<<4)
#define PM_WAKEUP_SRC_WKIO5     (1<<5)
#define PM_WAKEUP_SRC_WKIO6     (1<<6)
#define PM_WAKEUP_SRC_WKIO7     (1<<7)
#define PM_WAKEUP_SRC_WKIO8     (1<<8)
#define PM_WAKEUP_SRC_WKIO9     (1<<9)
#endif
#define PM_WAKEUP_SRC_WKTIMER   (1<<10)
#define PM_WAKEUP_SRC_WKSEV     (1<<11)
#define PM_WAKEUP_SRC_NETCPU    (PM_WAKEUP_SRC_WKSEV)
#define PM_WAKEUP_SRC_DEVICES   (1<<12)

extern uint32_t HAL_Wakeup_GetEevent(void);
#ifdef __CONFIG_ARCH_APP_CORE
extern int32_t HAL_Wakeup_SetIOHold(uint32_t hold_io);
extern void HAL_Wakeup_SetIO(uint32_t pn, uint32_t mode); /* pn:0~9, mode: 0:negative edge, 1:positive edge */
extern void HAL_Wakeup_ClrIO(uint32_t pn);
#endif
extern int32_t HAL_Wakeup_SetTimer(uint32_t count_32k);
#define HAL_Wakeup_SetTimer_mS(ms) HAL_Wakeup_SetTimer(ms*32)
extern int32_t HAL_Wakeup_SetSrc(void);
extern void HAL_Wakeup_ClrSrc(void);
extern void HAL_Wakeup_Init(void);
extern void HAL_Wakeup_DeInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _DRIVER_CHIP_HAL_WAKEUP_H_ */
