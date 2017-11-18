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

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "driver/chip/hal_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_UART_ID      UART0_ID    /* debug and console */

typedef void (*serial_cmd_exec_func)(void);

typedef struct serial_param {
	UART_ID	uartID;
	serial_cmd_exec_func cmd_exec;
} serial_param_t;

/* NB: Make sure uart is inited before calling this function. */
extern int serial_init(UART_ID uart_id, int baudrate, int data_bits, int parity, int stop_bits, int hwfc);
extern int serial_config(UART_ID uart_id, int baudrate, int data_bits, int parity, int stop_bits, int hwfc);
extern int serial_deinit(UART_ID uart_id);

extern int serial_start(void);
extern void serial_stop(void);

extern int serial_read(uint8_t *buf, int32_t size);

extern int serial_write(uint8_t *buf, int32_t len);

extern void serial_disable(void);
extern void serial_enable(void);
extern UART_ID serial_get_uart_id(void);

#ifdef __cplusplus
}
#endif

#endif /* _SERIAL_H_ */
