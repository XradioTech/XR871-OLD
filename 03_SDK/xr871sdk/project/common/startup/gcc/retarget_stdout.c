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

#include <stdio.h>
#include "common/board/board.h"
#include "pm/pm.h"
#include "driver/chip/hal_util.h"

/*
 * retarget for standard output/error
 */

#define STDOUT_WAIT_UART_TX_DONE 1

static uint8_t g_stdout_enable = 1;
static UART_ID g_stdout_uart_id = UART_NUM;

#if PRJCONF_UART_EN

#ifdef CONFIG_PM

#include <string.h>
#include <stdlib.h>

static uint32_t pm_print_index;
static uint32_t pm_print_len;
static char *pm_print_buf;

static int8_t g_stdio_suspending = 0;

static int stdio_suspend(struct soc_device *dev, enum suspend_state_t state)
{
	g_stdio_suspending = 1;

#if STDOUT_WAIT_UART_TX_DONE
	if (g_stdout_enable && g_stdout_uart_id < UART_NUM) {
		UART_T *uart = HAL_UART_GetInstance(g_stdout_uart_id);
		while (!HAL_UART_IsTxEmpty(uart)) { }
		HAL_UDelay(100); /* wait tx done, 100 us for baudrate 115200 */
	}
#endif

	switch (state) {
	case PM_MODE_SLEEP:
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
	case PM_MODE_POWEROFF:
		printf("a%s ok\n", __func__);
		break;
	default:
		break;
	}
	return 0;
}

/* BESURE app cpu has run now for nuart use app gpio */
static int stdio_resume(struct soc_device *dev, enum suspend_state_t state)
{
	switch (state) {
	case PM_MODE_SLEEP:
	case PM_MODE_STANDBY:
	case PM_MODE_HIBERNATION:
		printf("a%s ok\n", __func__);
		break;
	default:
		break;
	}

	g_stdio_suspending = 0;

	if (pm_print_len > 0 && pm_print_index > 0) {
		pm_print_buf[pm_print_index] = '\0';
		puts(pm_print_buf);
		pm_print_index = 0;
	}

	return 0;
}

static const struct soc_device_driver stdio_drv = {
	.name = "astdio",
	.suspend_noirq = stdio_suspend,
	.resume_noirq = stdio_resume,
};

static struct soc_device stdio_dev = {
	.name = "astdio",
	.driver = &stdio_drv,
};

#define STDIO_DEV (&stdio_dev)

void uart_set_suspend_record(unsigned int len)
{
	char *buf;

	if (pm_print_len == len) {
		return;
	}

	if (pm_print_len > 0) {
		buf = pm_print_buf;
		pm_print_len = 0;
		pm_print_buf = NULL;
		free(buf);
	}

	if (len > 0) {
		buf = malloc(len);
		if (buf) {
			pm_print_buf = buf;
			pm_print_len = len;
		}
	}
	pm_print_index = 0;
}

#endif /* CONFIG_PM */

void stdout_enable(uint8_t en)
{
	g_stdout_enable = en;
}

static int stdout_write(const char *buf, int len)
{
	if (!g_stdout_enable || g_stdout_uart_id >= UART_NUM || len <= 0) {
		return 0;
	}

#ifdef CONFIG_PM
	if (g_stdio_suspending) {
		if (pm_print_len > 0 && pm_print_index + len < (pm_print_len - 1)) {
			memcpy(pm_print_buf + pm_print_index, buf, len);
			pm_print_index += len;
			return len;
		} else {
			return 0;
		}
	}
#endif

	return board_uart_write(g_stdout_uart_id, buf, len);
}

int stdout_init(void)
{
	if (g_stdout_uart_id < UART_NUM) {
		return 0;
	}

	if (board_uart_init(BOARD_MAIN_UART_ID) == HAL_OK) {
		g_stdout_uart_id = BOARD_MAIN_UART_ID;
#ifdef __CONFIG_LIBC_WRAP_STDIO
		stdio_set_write(stdout_write);
#endif
#ifdef CONFIG_PM
		if (!g_stdio_suspending) {
			pm_register_ops(STDIO_DEV);
		}
#endif
		return 0;
	}

	return -1;
}

int stdout_deinit(void)
{
	if (g_stdout_uart_id >= UART_NUM) {
		return 0;
	}

#ifdef CONFIG_PM
	if (!g_stdio_suspending) {
		pm_unregister_ops(STDIO_DEV);
	}
#endif

	if (board_uart_deinit(g_stdout_uart_id) == HAL_OK) {
		g_stdout_uart_id = UART_NUM;
		return 0;
	}

	return -1;
}

#ifndef __CONFIG_LIBC_WRAP_STDIO

#include <sys/unistd.h> /* for STDOUT_FILENO and STDERR_FILENO */
#include "compiler.h"
#include "driver/chip/hal_cmsis.h"
#include "kernel/os/os_mutex.h"

static OS_Mutex_t g_stdout_mutex;

/* case of critical context
 *    - IRQ disabled
 *    - FIQ disabled
 *    - Execute in ISR context
 *    - Scheduler is not running
 */
static int stdout_is_critical_context(void)
{
    return (__get_PRIMASK()   ||
            __get_FAULTMASK() ||
            __get_IPSR()      ||
            !OS_ThreadIsSchedulerRunning());
}

static void stdout_mutex_lock(void)
{
	if (stdout_is_critical_context()) {
		return;
	}

	if (!OS_MutexIsValid(&g_stdout_mutex)) {
		OS_RecursiveMutexCreate(&g_stdout_mutex);
	}
	OS_RecursiveMutexLock(&g_stdout_mutex, OS_WAIT_FOREVER);
}

static void stdout_mutex_unlock(void)
{
	if (stdout_is_critical_context()) {
		return;
	}

	if (OS_MutexIsValid(&g_stdout_mutex)) {
		OS_RecursiveMutexUnlock(&g_stdout_mutex);
	}
}

int _write(int fd, const char *buf, int len)
{
	int ret;

	if (fd != STDOUT_FILENO && fd != STDERR_FILENO) {
		return -1;
	}

	stdout_mutex_lock();
	ret = stdout_write(buf, len);
	stdout_mutex_unlock();

	return ret;
}

#endif /* __CONFIG_LIBC_WRAP_STDIO */

#endif /* PRJCONF_UART_EN */
