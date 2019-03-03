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

#include "kernel/FreeRTOS/FreeRTOS.h"
#include "kernel/FreeRTOS/cpuusage.h"

#if (configDEBUG_CPU_USAGE_EN == 1)

#include <stdio.h>

#include "kernel/FreeRTOS/task.h"

#define ABS(x) ((x) >= 0 ? (x) : (-x))

#define tskCPUUSAGE_STACK_SIZE ((unsigned short)1024)
#define tskCPUUSAGE_PRIORITY (tskIDLE_PRIORITY + 1) /* priority only higher than idle */

static uint32_t print_interval;
static uint32_t OSCPUUsage;

#if ((configUSE_PREEMPTION == 1) && (configIDLE_SHOULD_YIELD == 0))

static volatile uint32_t OSIdleCtr; /* count or time */

#if ((configUSE_IDLE_HOOK == 1) && (configUSE_TICKLESS_IDLE == 0))

static uint32_t OSIdleCtrMax;

void vApplicationIdleHook(void)
{
	taskENTER_CRITICAL();
	OSIdleCtr++;
	taskEXIT_CRITICAL();
}

#elif ((configUSE_IDLE_HOOK == 0) && (configUSE_TICKLESS_IDLE == 1))

#include "kernel/os/FreeRTOS/os_common.h"

static uint32_t OSIdleTimeBegin;

void vPortTraceLOW_Power_Idle_Begin(void)
{
	OSIdleTimeBegin = (uint32_t)xTaskGetTickCount();
}

void vPortTraceLOW_Power_Idle_End(void)
{
	/* no protection is ok */
	OSIdleCtr += xTaskGetTickCount() - OSIdleTimeBegin;
}
#endif

static void OS_CPUUsage_Task(void *pvParameters)
{
	uint32_t idle_cnt;
	uint32_t print_cnt = 0;
	uint32_t cpu_usage = 0;

	for (;;) {
		uint32_t this_usage;

		taskENTER_CRITICAL();
		idle_cnt = OSIdleCtr;
		OSIdleCtr = 0;
		taskEXIT_CRITICAL();

		/* 100 - (OSIdleTimeRun/1000)*100 */
#if ((configUSE_IDLE_HOOK == 1) && (configUSE_TICKLESS_IDLE == 0))
		this_usage = 100 - (idle_cnt / OSIdleCtrMax);
#elif ((configUSE_IDLE_HOOK == 0) && (configUSE_TICKLESS_IDLE == 1))
		this_usage = 100 - (idle_cnt / (configTICK_RATE_HZ/100));
#endif
		cpu_usage += ABS(this_usage);
		if (print_interval && (print_interval <= ++print_cnt)) {
			OSCPUUsage = cpu_usage/print_interval;
			printf("[%8d]: cpua usage:%d%%\n", xTaskGetTickCount(), OSCPUUsage);
			print_cnt = 0;
			cpu_usage = 0;
		} else {
			OSCPUUsage = cpu_usage / print_cnt;
		}
		vTaskDelay(1000);
	}
}

#endif

void OSCpuUsageInit(uint32_t print_s)
{
	print_interval = print_s;

#if ((configUSE_PREEMPTION == 1) && (configIDLE_SHOULD_YIELD == 0))
#if ((configUSE_IDLE_HOOK == 1) && (configUSE_TICKLESS_IDLE == 0))

	taskENTER_CRITICAL();
	OSIdleCtr = 0;
	taskEXIT_CRITICAL();

	vTaskDelay(1000);

	taskENTER_CRITICAL();
	OSIdleCtrMax = OSIdleCtr / 100;
	taskEXIT_CRITICAL();
#endif
	xTaskCreate(OS_CPUUsage_Task, "", tskCPUUSAGE_STACK_SIZE, (void *)NULL,
		    tskCPUUSAGE_PRIORITY, NULL);
#else
	(void)OSIdleCtr;
	printf("cpu usage unsupported !\n");
#endif
}

uint32_t OSGetCpuUsage(void)
{
	return OSCPUUsage;
}

#endif /* configDEBUG_CPU_USAGE_EN */
