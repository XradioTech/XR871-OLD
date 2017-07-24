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

#include <sys/unistd.h> /* for STDOUT_FILENO and STDERR_FILENO */

#include "compiler.h"
#include "kernel/os/os_mutex.h"
#include "driver/chip/hal_cmsis.h"
#include "common/board/board.h"


/*
 * retarget for standard output/error
 */

static OS_Mutex_t g_stdout_mutex;
static uint8_t g_stdout_enable = 1;

static __always_inline int stdout_is_isr_context(void)
{
	return __get_IPSR();
}

static void stdout_mutex_lock(void)
{
	if (stdout_is_isr_context() || !OS_ThreadIsSchedulerRunning()) {
		return;
	}

	if (OS_MutexIsValid(&g_stdout_mutex)) {
		OS_RecursiveMutexLock(&g_stdout_mutex, OS_WAIT_FOREVER);
	} else {
		OS_RecursiveMutexCreate(&g_stdout_mutex);
		OS_RecursiveMutexLock(&g_stdout_mutex, OS_WAIT_FOREVER);
	}
}

static void stdout_mutex_unlock(void)
{
	if (stdout_is_isr_context() || !OS_ThreadIsSchedulerRunning()) {
		return;
	}

	if (OS_MutexIsValid(&g_stdout_mutex)) {
		OS_RecursiveMutexUnlock(&g_stdout_mutex);
	}
}

void stdout_enable(uint8_t en)
{
	g_stdout_enable = en;
}

UART_ID g_serial_uart_id = UART_NUM;

int _write(int fd, char *buf, int count)
{
	int ret;

	if (fd != STDOUT_FILENO && fd != STDERR_FILENO) {
		return -1;
	}

	if (!g_stdout_enable)
		return -1;

	if (g_serial_uart_id >= UART_NUM) {
		board_uart_init(BOARD_MAIN_UART_ID);
		g_serial_uart_id = BOARD_MAIN_UART_ID;
	}

	stdout_mutex_lock();
	ret = board_uart_write(g_serial_uart_id, buf, count);
	stdout_mutex_unlock();

	return ret;
}
