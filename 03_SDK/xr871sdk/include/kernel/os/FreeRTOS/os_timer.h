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

#ifndef _KERNEL_OS_FREERTOS_OS_TIMER_H_
#define _KERNEL_OS_FREERTOS_OS_TIMER_H_

#include "kernel/os/FreeRTOS/os_common.h"
#include "timers.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (defined(configUSE_TIMER_ID_AS_CALLBACK_ARG) && configUSE_TIMER_ID_AS_CALLBACK_ARG == 1)
#define OS_TIMER_USE_FREERTOS_ORIG_CALLBACK	0
#else
#define OS_TIMER_USE_FREERTOS_ORIG_CALLBACK	1
#endif

typedef enum {
	OS_TIMER_ONCE		= 0,
	OS_TIMER_PERIODIC	= 1
} OS_TimerType;

typedef void (*OS_TimerCallback_t)(void *arg);
typedef TimerHandle_t OS_TimerHandle_t;

#if OS_TIMER_USE_FREERTOS_ORIG_CALLBACK
typedef struct OS_TimerCallbackData {
	OS_TimerCallback_t	callback;
	void				*argument;
} OS_TimerCallbackData_t;
#endif

typedef struct OS_Timer {
	TimerHandle_t	handle;
#if OS_TIMER_USE_FREERTOS_ORIG_CALLBACK
	OS_TimerCallbackData_t *priv;
#endif
} OS_Timer_t;


OS_Status OS_TimerCreate(OS_Timer_t *timer, OS_TimerType type,
                         OS_TimerCallback_t cb, void *arg, OS_Time_t periodMS);
OS_Status OS_TimerDelete(OS_Timer_t *timer);
OS_Status OS_TimerStart(OS_Timer_t *timer);
OS_Status OS_TimerChangePeriod(OS_Timer_t *timer, OS_Time_t periodMS);
OS_Status OS_TimerStop(OS_Timer_t *timer);

static __inline int OS_TimerIsValid(OS_Timer_t *timer)
{
	return (timer->handle != OS_INVALID_HANDLE);
}

static __inline void OS_TimerSetInvalid(OS_Timer_t *timer)
{
	timer->handle = OS_INVALID_HANDLE;
}

static __inline int OS_TimerIsActive(OS_Timer_t *timer)
{
	return (xTimerIsTimerActive(timer->handle) != pdFALSE);
}

#ifdef __cplusplus
}
#endif

#endif /* _KERNEL_OS_FREERTOS_OS_TIMER_H_ */
