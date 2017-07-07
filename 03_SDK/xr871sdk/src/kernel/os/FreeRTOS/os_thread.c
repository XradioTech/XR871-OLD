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

#include "kernel/os/FreeRTOS/os_thread.h"
#include "os_util.h"

OS_Status OS_ThreadCreate(OS_Thread_t *thread, const char *name,
                          OS_ThreadEntry_t entry, void *arg,
                          OS_Priority priority, uint32_t stackSize)
{
	BaseType_t ret;

	OS_HANDLE_ASSERT(!OS_ThreadIsValid(thread), thread->handle);

	ret = xTaskCreate(entry, name, stackSize / sizeof(StackType_t), arg,
	                  priority, &thread->handle);
	if (ret != pdPASS) {
		OS_ERR("err %"OS_BASETYPE_F"\n", ret);
		OS_ThreadSetInvalid(thread);
		return OS_FAIL;
	}
	return OS_OK;
}

OS_Status OS_ThreadDelete(OS_Thread_t *thread)
{
	TaskHandle_t handle;
	TaskHandle_t curHandle;

	if (thread == NULL) {
		vTaskDelete(NULL); /* delete self */
		return OS_OK;
	}

	OS_HANDLE_ASSERT(OS_ThreadIsValid(thread), thread->handle);

	handle = thread->handle;
	curHandle = xTaskGetCurrentTaskHandle();
	if (handle == curHandle) {
		/* delete self */
		OS_ThreadSetInvalid(thread);
		vTaskDelete(NULL);
	} else {
		/* delete other thread */
		OS_WARN("thread %"OS_HANDLE_F" delete %"OS_HANDLE_F"\n", curHandle, handle);
		vTaskDelete(handle);
		OS_ThreadSetInvalid(thread);
	}

	return OS_OK;
}

uint32_t OS_ThreadGetStackMinFreeSize(OS_Thread_t *thread)
{
	TaskHandle_t handle;

	handle =  thread ? thread->handle : NULL;
	return (uxTaskGetStackHighWaterMark(handle) * sizeof(StackType_t));
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	OS_ERR("task %p stack over flow\n", xTask);
	OS_ABORT();
}
#endif
