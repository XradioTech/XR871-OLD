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

#ifndef _EFPG_EFPG_H_
#define _EFPG_EFPG_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum efpg_block {
	EFPG_BLOCK_HOSC	= 0,		/* data size: 1  byte  */
	EFPG_BLOCK_BOOT	= 1,		/* data size: 32 bytes */
	EFPG_BLOCK_DCXO	= 2,		/* data size: 1  byte  */
	EFPG_BLOCK_POUT	= 3,		/* data size: 3  bytes */
	EFPG_BLOCK_MAC	= 4,		/* data size: 6  bytes */
	EFPG_BLOCK_NUM	= 5,
} efpg_block_t;

typedef enum efpg_hosc {
	EFPG_HOSC_24M		= 0,
	EFPG_HOSC_26M		= 1,
	EFPG_HOSC_40M		= 2,
	EFPG_HOSC_52M		= 3,
	EFPG_HOSC_INVALID	= 4,
} efpg_hosc_t;

int efpg_init(uint8_t *key, uint8_t len);
void efpg_deinit(void);

/* The rest bit(s) in data will be cleared to be 0. */
int efpg_read(efpg_block_t block, uint8_t *data);
int efpg_read_hosc(efpg_hosc_t *hosc);

/* The rest bit(s) in data will be cleared to be 0. */
int efpg_read_efuse(uint8_t *data, uint32_t start_bit, uint32_t bit_num);
int efpg_write_efuse(uint8_t *data, uint32_t start_bit, uint32_t bit_num);

/* Just for OEM programming tool to entry eFuse programming mode. */
int efpg_start(void);

#ifdef __cplusplus
}
#endif

#endif /* _EFPG_EFPG_H_ */