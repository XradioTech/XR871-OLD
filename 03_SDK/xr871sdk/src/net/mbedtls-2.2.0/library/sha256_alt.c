/*
 *  FIPS-180-2 compliant SHA-256 implementation
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
/*
 *  The SHA-256 Secure Hash Standard was published by NIST in 2002.
 *
 *  http://csrc.nist.gov/publications/fips/fips180-2/fips180-2.pdf
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_SHA256_C)

#include "mbedtls/sha256.h"

#include <string.h>

#if defined(MBEDTLS_SELF_TEST)
#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf printf
#endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_SHA256_ALT)

/* Implementation that should never be optimized out by the compiler */
static void mbedtls_zeroize( void *v, size_t n ) {
    volatile unsigned char *p = v; while( n-- ) *p++ = 0;
}

void mbedtls_sha256_init( mbedtls_sha256_context *ctx )
{
    memset( ctx, 0, sizeof( mbedtls_sha256_context ) );
}

void mbedtls_sha256_free( mbedtls_sha256_context *ctx )
{
    if( ctx == NULL )
        return;

    mbedtls_zeroize( ctx, sizeof( mbedtls_sha256_context ) );
}

void mbedtls_sha256_clone( mbedtls_sha256_context *dst,
                           const mbedtls_sha256_context *src )
{
    *dst = *src;
}

/*
 * SHA-256 context setup
 */
void mbedtls_sha256_starts( mbedtls_sha256_context *ctx, int is224 )
{
	uint32_t iv[8];

	if (is224 == 0) {
		/* SHA-256 */
		HAL_SHA256_Init(&ctx->sha256, CE_CTL_IVMODE_SHA_MD5_FIPS180, iv);
	} else {
		/* SHA-224 */
		iv[0] = 0xC1059ED8;
		iv[1] = 0x367CD507;
		iv[2] = 0x3070DD17;
		iv[3] = 0xF70E5939;
		iv[4] = 0xFFC00B31;
		iv[5] = 0x68581511;
		iv[6] = 0x64F98FA7;
		iv[7] = 0xBEFA4FA4;
		HAL_SHA256_Init(&ctx->sha256, CE_CTL_IVMODE_SHA_MD5_INPUT, iv);
	}

    ctx->is224 = is224;
}

#if !defined(MBEDTLS_SHA256_PROCESS_ALT)
void mbedtls_sha256_process( mbedtls_sha256_context *ctx, const unsigned char data[64] )
{

}
#endif /* !MBEDTLS_SHA256_PROCESS_ALT */

/*
 * SHA-256 process buffer
 */
void mbedtls_sha256_update( mbedtls_sha256_context *ctx, const unsigned char *input,
                    size_t ilen )
{
	HAL_SHA256_Append(&ctx->sha256, (uint8_t*)input, ilen);
}

/*
 * SHA-256 final digest
 */
void mbedtls_sha256_finish( mbedtls_sha256_context *ctx, unsigned char output[32] )
{
	uint32_t output_224[8];

	if (ctx->is224 == 0) {
		HAL_SHA256_Finish(&ctx->sha256, (uint32_t*)output);
	} else {
		HAL_SHA256_Finish(&ctx->sha256, output_224);
		*(uint32_t*)&output[0] = output_224[0];
		*(uint32_t*)&output[4] = output_224[1];
		*(uint32_t*)&output[8] = output_224[2];
		*(uint32_t*)&output[12] = output_224[3];
		*(uint32_t*)&output[16] = output_224[4];
		*(uint32_t*)&output[20] = output_224[5];
		*(uint32_t*)&output[24] = output_224[6];
	}
}

#endif /* !MBEDTLS_SHA256_ALT */

#endif /* MBEDTLS_SHA256_C */
