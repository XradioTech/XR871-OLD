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

#include <string.h>
#include <ctype.h>
#include "kernel/os/os.h"
#include "serial.h"
#include "serial_debug.h"

#define SERIAL_CACHE_BUF_NUM	16
#define SERIAL_CACHE_BUF_SIZE	(64/2)

typedef enum {
	SERIAL_STATE_STOP = 0,
	SERIAL_STATE_START,
	SERIAL_STATE_TERMINATE,	/* to be terminated */
} serial_state;

typedef struct serial_priv {
	UART_ID			uartID;
	serial_state	state;

	OS_Semaphore_t	 cmd_sem;

	serial_cmd_exec_func cmd_exec;

	struct {
		volatile uint8_t cnt;
		uint8_t widx;
		uint8_t ridx;
		uint8_t len[SERIAL_CACHE_BUF_NUM];
		uint8_t buf[SERIAL_CACHE_BUF_NUM][SERIAL_CACHE_BUF_SIZE];
	} cache;
} serial_priv_t;

static serial_priv_t g_serial;

/* Note: only support line end with "\r\n" or "\n", not support "\r" */
static void serial_rx_callback(void *arg)
{
	serial_priv_t *serial;
	UART_T *uart;
	uint32_t cnt;
	uint32_t idx;
	uint32_t len;
	uint8_t data;

	uart = (UART_T *)arg;
	serial = &g_serial;

	cnt = serial->cache.cnt;

	if(cnt < SERIAL_CACHE_BUF_NUM) {
		idx = serial->cache.widx;
		len = 0;
		while (1) {
			if (HAL_UART_IsRxReady(uart)) {
				data = HAL_UART_GetRxData(uart);

				serial->cache.buf[idx][len++] = data;

				if (len >= SERIAL_CACHE_BUF_SIZE) {
					serial->cache.len[idx] = len;
					idx++;
					if (idx >= SERIAL_CACHE_BUF_NUM) {
						idx = 0;
					}
					cnt++;
					serial->cache.widx = idx;
					serial->cache.cnt = cnt;
					OS_SemaphoreRelease(&serial->cmd_sem);

					if (cnt >= SERIAL_CACHE_BUF_NUM) {
						if (HAL_UART_IsRxReady(uart)) {
							/* discard data */
							while (HAL_UART_IsRxReady(uart)) {
								data = HAL_UART_GetRxData(uart);
							}

							SERIAL_WARN("no buf for rx, discard received data\n");
						}

						break;
					}

					len = 0;
				}
			}
			else {
				if (len > 0) {
					serial->cache.len[idx] = len;
					idx++;
					if (idx >= SERIAL_CACHE_BUF_NUM) {
						idx = 0;
					}
					cnt++;
					serial->cache.widx = idx;
					serial->cache.cnt = cnt;
					OS_SemaphoreRelease(&serial->cmd_sem);

					break;
				}
			}
		}
	}
	else {
		/* discard data */
		while (HAL_UART_IsRxReady(uart)) {
			data = HAL_UART_GetRxData(uart);
		}
		SERIAL_WARN("no buf for rx, discard received data\n");
	}
}

#define SERIAL_THREAD_STACK_SIZE	(2 * 1024)
static OS_Thread_t g_serial_thread;

static void serial_task(void *arg)
{
#if 0
	serial_priv_t *serial;
	uint32_t cnt;
	uint32_t idx;

	SERIAL_DBG("%s() start...\n", __func__);

	serial = &g_serial;

	idx = serial->cache.ridx;

	while (1) {
		if (OS_SemaphoreWait(&serial->cmd_sem, OS_WAIT_FOREVER) != OS_OK)
			continue;

		if (serial->state != SERIAL_STATE_START)
			break;

		arch_irq_disable();
		cnt = serial->cache.cnt;
		arch_irq_enable();

		if (cnt > 0) {
			if (serial->cmd_exec) {
				serial->cmd_exec((char *)serial->cache.buf[idx], serial->cache.len[idx]);
			}

			idx++;
			if (idx >= SERIAL_CACHE_BUF_NUM) {
				idx = 0;
			}

			serial->cache.ridx = idx;

			arch_irq_disable();
			serial->cache.cnt--;
			arch_irq_enable();
		}
		else {
			SERIAL_WARN("no valid command\n");
		}
	}
#else
	serial_priv_t *serial;

	SERIAL_DBG("%s() start...\n", __func__);

	serial = &g_serial;

	while (1) {
		serial->cmd_exec();
	}
#endif
	SERIAL_DBG("%s() exit\n", __func__);
	OS_ThreadDelete(&g_serial_thread);
}

int serial_config(UART_ID uart_id, int baudrate, int data_bits, int parity, int stop_bits, int hwfc)
{
	UART_InitParam uart_param;

	uart_param.baudRate = baudrate;
	uart_param.parity = UART_PARITY_NONE;
	uart_param.stopBits = UART_STOP_BITS_1;
	uart_param.dataBits = UART_DATA_BITS_8;
	uart_param.isAutoHwFlowCtrl = hwfc;

	HAL_UART_Init(uart_id, &uart_param);

	return 0;
}

int serial_init(UART_ID uart_id, int baudrate, int data_bits, int parity, int stop_bits, int hwfc)
{
	serial_priv_t *serial;

	serial_config(uart_id, baudrate, data_bits, parity, stop_bits, hwfc);

	serial = &g_serial;
	if (serial->state != SERIAL_STATE_STOP) {
		SERIAL_ERR("serial state %d\n", serial->state);
		return -1;
	}

	memset(serial, 0, sizeof(*serial));
	serial->uartID = uart_id;

	if (OS_SemaphoreCreate(&serial->cmd_sem, 0, OS_SEMAPHORE_MAX_COUNT) != OS_OK) {
		SERIAL_ERR("create semaphore failed\n");
		return -1;
	}

	return 0;
}

int serial_deinit(UART_ID uart_id)
{
	serial_priv_t *serial;

	serial = &g_serial;

	HAL_UART_DeInit(uart_id);

	OS_SemaphoreDelete(&serial->cmd_sem);


	return 0;
}

/* NB: Make sure uart is inited before calling this function. */
int serial_start(void)
{
	UART_T *uart;
	serial_priv_t *serial;

	serial = &g_serial;

	uart = HAL_UART_GetInstance(serial->uartID);
	HAL_UART_EnableRxCallback(serial->uartID, serial_rx_callback, uart);
	serial->state = SERIAL_STATE_START;

	return 0;
}

void serial_stop(void)
{
	serial_priv_t *serial;

	serial = &g_serial;
	HAL_UART_DisableRxCallback(serial->uartID);
	serial->state = SERIAL_STATE_STOP;
}

int serial_read(uint8_t *buf, int32_t size)
{
	serial_priv_t *serial;
	uint32_t cnt;
	uint32_t idx;
	int rlen = 0;

	SERIAL_DBG("%s() start...\n", __func__);

	serial = &g_serial;

	idx = serial->cache.ridx;

	while (1) {
/*
		if (OS_SemaphoreWait(&serial->cmd_sem, OS_WAIT_FOREVER) != OS_OK)
			continue;
*/
		if (OS_SemaphoreWait(&serial->cmd_sem, 10) != OS_OK)
			break;

		if (serial->state != SERIAL_STATE_START)
			break;

		arch_irq_disable();
		cnt = serial->cache.cnt;
		arch_irq_enable();

		if (cnt > 0) {
			rlen = serial->cache.len[idx];
			if (rlen > size) {
				return -1; /* buffer size is too small */
			}

			memcpy(buf, serial->cache.buf[idx], rlen);

			idx++;
			if (idx >= SERIAL_CACHE_BUF_NUM) {
				idx = 0;
			}

			serial->cache.ridx = idx;

			arch_irq_disable();
			serial->cache.cnt--;
			arch_irq_enable();

			break;
		}
		else {
			SERIAL_WARN("no valid command\n");
			return -2; /* no data  */
		}
	}

	return rlen;
}

int serial_write(uint8_t *buf, int32_t len)
{
	serial_priv_t *serial;

	serial = &g_serial;
	return HAL_UART_Transmit_Poll(serial->uartID, buf, len);
}

void serial_disable(void)
{
	serial_priv_t *serial;

	serial = &g_serial;
	if (serial->state == SERIAL_STATE_START) {
		HAL_UART_DisableRxCallback(serial->uartID);
	}
}

void serial_enable(void)
{
	serial_priv_t *serial;
	UART_T *uart;

	serial = &g_serial;
	if (serial->state == SERIAL_STATE_START) {
		uart = HAL_UART_GetInstance(serial->uartID);
		HAL_UART_EnableRxCallback(serial->uartID, serial_rx_callback, uart);
	}
}

UART_ID serial_get_uart_id(void)
{
	serial_priv_t *serial;

	serial = &g_serial;
	if (serial->state == SERIAL_STATE_START) {
		return serial->uartID;
	} else {
		return UART_INVALID_ID;
	}
}
