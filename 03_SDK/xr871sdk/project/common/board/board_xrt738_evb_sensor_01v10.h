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

#ifndef _BOARD_XRT738_EVB_SENSOR_01V10_H_
#define _BOARD_XRT738_EVB_SENSOR_01V10_H_

#ifdef __CONFIG_BOARD_XRT738_EVB_SENSOR_01V10

#include "driver/chip/hal_uart.h"
#include "driver/chip/hal_norflash.h"

#include "board_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_MAIN_UART_ID	UART0_ID	/* debug and console */
#define BOARD_SUB_UART_ID	UART0_ID	/* debug for netos */

#define BOARD_FLASH_CLK		(12 * 1000 * 1000)
#define BOARD_FLASH_HWDELAY (Flash_Ctrl_DelayCycle){1, 1, 8, 0, 0, 0, 1}

void board_uart_init(UART_ID uart_id);
void board_uart_deinit(UART_ID uart_id);
int board_uart_write(UART_ID uart_id, char *buf, int count);
int board_uart_getc(UART_ID uart_id);

int board_flash_init(SF_Handler *hdl);
void board_flash_deinit(SF_Handler *hdl);

HAL_Status board_deinit(void);
HAL_Status board_init(void);

#ifdef __CONFIG_XIP_ENABLE
void board_xip_init(void);
#endif /* __CONFIG_XIP_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_BOARD_XRT738_EVB_SENSOR_01V10 */
#endif /* _BOARD_XRT738_EVB_SENSOR_01V10_H_ */
