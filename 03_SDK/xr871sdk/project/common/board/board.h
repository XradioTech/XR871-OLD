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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "board_config.h"
#include "driver/chip/hal_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_Status board_ioctl(HAL_BoardIoctlReq req, uint32_t param0, uint32_t param1);

#if PRJCONF_UART_EN
/* uart */
HAL_Status board_uart_init(UART_ID uart_id);

static __inline HAL_Status board_uart_deinit(UART_ID uart_id)
{
	return HAL_UART_DeInit(uart_id);
}

static __inline int32_t board_uart_write(UART_ID uart_id, const char *buf, int len)
{
	return HAL_UART_Transmit_Poll(uart_id, (uint8_t *)buf, len);
}
#endif

#if PRJCONF_SPI_EN
/* spi */
HAL_Status board_spi_init(SPI_Port spi);
HAL_Status board_spi_deinit(SPI_Port spi);
#endif

#if PRJCONF_SOUNDCARD0_EN
/* sound card0 */
HAL_Status board_soundcard0_init(codec_detect_cb cb);
HAL_Status board_soundcard0_deinit(void);
#endif

#if PRJCONF_SOUNDCARD1_EN
/* sound card1 */
HAL_Status board_soundcard1_init(void);
HAL_Status board_soundcard1_deinit(void);
#endif

#if PRJCONF_MMC_EN
/* mmc card */
HAL_Status board_sdcard_init(card_detect_cb cb);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H_ */
