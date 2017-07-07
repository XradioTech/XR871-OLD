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

#include "kernel/os/FreeRTOS/os_mutex.h"
#include "os_util.h"


OS_Status OS_MutexCreate(OS_Mutex_t *mutex)
{
	OS_HANDLE_ASSERT(!OS_MutexIsValid(mutex), mutex->handle);

	mutex->handle = xSemaphoreCreateMutex();
	if (mutex->handle == NULL) {
		OS_ERR("err %"OS_HANDLE_F"\n", mutex->handle);
		return OS_FAIL;
	}

	return OS_OK;
}

OS_Status OS_MutexDelete(OS_Mutex_t *mutex)
{
	OS_HANDLE_ASSERT(OS_MutexIsValid(mutex), mutex->handle);

	vSemaphoreDelete(mutex->handle);
	OS_MutexSetInvalid(mutex);
	return OS_OK;
}

OS_Status OS_MutexLock(OS_Mutex_t *mutex, OS_Time_t waitMS)
{
	BaseType_t ret;

	OS_HANDLE_ASSERT(OS_MutexIsValid(mutex), mutex->handle);

	ret = xSemaphoreTake(mutex->handle, OS_CalcWaitTicks(waitMS));
	if (ret != pdPASS) {
		OS_DBG("%s() fail @ %d, %"OS_TIME_F" ms\n", __func__, __LINE__, waitMS);
		return OS_FAIL;
	}

	return OS_OK;
}

OS_Status OS_MutexUnlock(OS_Mutex_t *mutex)
{
	BaseType_t ret;

	OS_HANDLE_ASSERT(OS_MutexIsValid(mutex), mutex->handle);

	ret = xSemaphoreGive(mutex->handle);
	if (ret != pdPASS) {
		OS_DBG("%s() fail @ %d\n", __func__, __LINE__);
		return OS_FAIL;
	}

	return OS_OK;
}

OS_Status OS_RecursiveMutexCreate(OS_Mutex_t *mutex)
{
	OS_HANDLE_ASSERT(!OS_MutexIsValid(mutex), mutex->handle);

	mutex->handle = xSemaphoreCreateRecursiveMutex();
	if (mutex->handle == NULL) {
		OS_ERR("err %"OS_HANDLE_F"\n", mutex->handle);
		return OS_FAIL;
	}

	return OS_OK;
}

OS_Status OS_RecursiveMutexLock(OS_Mutex_t *mutex, OS_Time_t waitMS)
{
	BaseType_t ret;

	OS_HANDLE_ASSERT(OS_MutexIsValid(mutex), mutex->handle);

	ret = xSemaphoreTakeRecursive(mutex->handle, OS_CalcWaitTicks(waitMS));
	if (ret != pdPASS) {
		OS_DBG("%s() fail @ %d, %"OS_TIME_F" ms\n", __func__, __LINE__, waitMS);
		return OS_FAIL;
	}

	return OS_OK;
}

OS_Status OS_RecursiveMutexUnlock(OS_Mutex_t *mutex)
{
	BaseType_t ret;

	OS_HANDLE_ASSERT(OS_MutexIsValid(mutex), mutex->handle);

	ret = xSemaphoreGiveRecursive(mutex->handle);
	if (ret != pdPASS) {
		OS_DBG("%s() fail @ %d\n", __func__, __LINE__);
		return OS_FAIL;
	}

	return OS_OK;
}
