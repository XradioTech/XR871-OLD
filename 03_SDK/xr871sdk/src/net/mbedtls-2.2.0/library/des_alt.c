/*
 *  FIPS-46-3 compliant Triple-DES implementation
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
 *  DES, on which TDES is based, was originally designed by Horst Feistel
 *  at IBM in 1974, and was adopted as a standard by NIST (formerly NBS).
 *
 *  http://csrc.nist.gov/publications/fips/fips46-3/fips46-3.pdf
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_DES_C)

#include "mbedtls/des.h"

#include <string.h>

#if defined(MBEDTLS_SELF_TEST)
#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf printf
#endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_DES_ALT)

/* Implementation that should never be optimized out by the compiler */
static void mbedtls_zeroize( void *v, size_t n ) {
    volatile unsigned char *p = v; while( n-- ) *p++ = 0;
}

#define SWAP(a,b) { uint32_t t = a; a = b; b = t; t = 0; }

void mbedtls_des_init( mbedtls_des_context *ctx )
{
	memset( ctx, 0, sizeof( mbedtls_des_context ) );
}

void mbedtls_des_free( mbedtls_des_context *ctx )
{
	if( ctx == NULL )
		return;

	mbedtls_zeroize( ctx, sizeof( mbedtls_des_context ) );
}

void mbedtls_des3_init( mbedtls_des3_context *ctx )
{
	memset( ctx, 0, sizeof( mbedtls_des3_context ) );
}

void mbedtls_des3_free( mbedtls_des3_context *ctx )
{
	if( ctx == NULL )
		return;

	mbedtls_zeroize( ctx, sizeof( mbedtls_des3_context ) );
}

#if !defined(MBEDTLS_DES_SETKEY_ALT)
void mbedtls_des_setkey( uint32_t SK[32], const unsigned char key[MBEDTLS_DES_KEY_SIZE] )
{

}
#endif /* !MBEDTLS_DES_SETKEY_ALT */

/*
 * DES key schedule (56-bit, encryption)
 */
int mbedtls_des_setkey_enc( mbedtls_des_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE] )
{
	memcpy(ctx->des.key, key, MBEDTLS_DES_KEY_SIZE);
	ctx->mode = MBEDTLS_DES_ENCRYPT;
	return( 0 );
}

/*
 * DES key schedule (56-bit, decryption)
 */
int mbedtls_des_setkey_dec( mbedtls_des_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE] )
{
	memcpy(ctx->des.key, key, MBEDTLS_DES_KEY_SIZE);
	ctx->mode = MBEDTLS_DES_DECRYPT;

	return( 0 );
}

static void des3_set2key( uint32_t esk[96],
                          uint32_t dsk[96],
                          const unsigned char key[MBEDTLS_DES_KEY_SIZE*2] )
{

}

/*
 * Triple-DES key schedule (112-bit, encryption)
 */
int mbedtls_des3_set2key_enc( mbedtls_des3_context *ctx,
                      const unsigned char key[MBEDTLS_DES_KEY_SIZE * 2] )
{
	memcpy(ctx->des3.key, key, MBEDTLS_DES_KEY_SIZE * 2);
	memcpy(&ctx->des3.key[MBEDTLS_DES_KEY_SIZE*2], key, MBEDTLS_DES_KEY_SIZE);
	ctx->mode = MBEDTLS_DES_ENCRYPT;

	return( 0 );
}

/*
 * Triple-DES key schedule (112-bit, decryption)
 */
int mbedtls_des3_set2key_dec( mbedtls_des3_context *ctx,
                      const unsigned char key[MBEDTLS_DES_KEY_SIZE * 2] )
{
	memcpy(ctx->des3.key, key, MBEDTLS_DES_KEY_SIZE * 2);
	memcpy(&ctx->des3.key[MBEDTLS_DES_KEY_SIZE*2], key, MBEDTLS_DES_KEY_SIZE);
	ctx->mode = MBEDTLS_DES_DECRYPT;

	return( 0 );
}

static void des3_set3key( uint32_t esk[96],
                          uint32_t dsk[96],
                          const unsigned char key[24] )
{

}

/*
 * Triple-DES key schedule (168-bit, encryption)
 */
int mbedtls_des3_set3key_enc( mbedtls_des3_context *ctx,
                      const unsigned char key[MBEDTLS_DES_KEY_SIZE * 3] )
{
	memcpy(ctx->des3.key, key, MBEDTLS_DES_KEY_SIZE * 3);
	ctx->mode = MBEDTLS_DES_ENCRYPT;

	return( 0 );
}

/*
 * Triple-DES key schedule (168-bit, decryption)
 */
int mbedtls_des3_set3key_dec( mbedtls_des3_context *ctx,
                      const unsigned char key[MBEDTLS_DES_KEY_SIZE * 3] )
{
	memcpy(ctx->des3.key, key, MBEDTLS_DES_KEY_SIZE * 3);
	ctx->mode = MBEDTLS_DES_DECRYPT;

	return( 0 );
}

/*
 * DES-ECB block encryption/decryption
 */
#if !defined(MBEDTLS_DES_CRYPT_ECB_ALT)
int mbedtls_des_crypt_ecb( mbedtls_des_context *ctx,
                    const unsigned char input[8],
                    unsigned char output[8] )
{
	ctx->des.mode = CE_CRYPT_MODE_ECB;

	if (ctx->mode == MBEDTLS_DES_ENCRYPT)
		HAL_DES_Encrypt(&ctx->des, (uint8_t*)input, (uint8_t*)output, 8);
	else
		HAL_DES_Decrypt(&ctx->des, (uint8_t*)input, (uint8_t*)output, 8);

	return( 0 );
}
#endif /* !MBEDTLS_DES_CRYPT_ECB_ALT */

#if defined(MBEDTLS_CIPHER_MODE_CBC)
/*
 * DES-CBC buffer encryption/decryption
 */
int mbedtls_des_crypt_cbc( mbedtls_des_context *ctx,
                    int mode,
                    size_t length,
                    unsigned char iv[8],
                    const unsigned char *input,
                    unsigned char *output )
{
	ctx->des.mode = CE_CRYPT_MODE_CBC;
	ctx->des.src = CE_CTL_KEYSOURCE_INPUT;
	memcpy(ctx->des.iv, iv, 8);

	if(mode == MBEDTLS_DES_ENCRYPT)
		HAL_DES_Encrypt(&ctx->des, (uint8_t*)input, (uint8_t*)output, length);
	else
		HAL_DES_Decrypt(&ctx->des, (uint8_t*)input, (uint8_t*)output, length);
	memcpy(iv, ctx->des.iv, 8);

	return( 0 );
}
#endif /* MBEDTLS_CIPHER_MODE_CBC */

/*
 * 3DES-ECB block encryption/decryption
 */
#if !defined(MBEDTLS_DES3_CRYPT_ECB_ALT)
int mbedtls_des3_crypt_ecb( mbedtls_des3_context *ctx,
                     const unsigned char input[8],
                     unsigned char output[8] )
{
	ctx->des3.mode = CE_CRYPT_MODE_ECB;

	if (ctx->mode == MBEDTLS_DES_ENCRYPT)
		HAL_3DES_Encrypt(&ctx->des3, (uint8_t*)input, (uint8_t*)output, 8);
	else
		HAL_3DES_Decrypt(&ctx->des3, (uint8_t*)input, (uint8_t*)output, 8);

	return( 0 );
}
#endif /* !MBEDTLS_DES3_CRYPT_ECB_ALT */

#if defined(MBEDTLS_CIPHER_MODE_CBC)
/*
 * 3DES-CBC buffer encryption/decryption
 */
int mbedtls_des3_crypt_cbc( mbedtls_des3_context *ctx,
                     int mode,
                     size_t length,
                     unsigned char iv[8],
                     const unsigned char *input,
                     unsigned char *output )
{
	ctx->des3.mode = CE_CRYPT_MODE_CBC;
	ctx->des3.src = CE_CTL_KEYSOURCE_INPUT;
	memcpy(ctx->des3.iv, iv, 8);

	if (mode == MBEDTLS_DES_ENCRYPT)
		HAL_3DES_Encrypt(&ctx->des3, (uint8_t*)input, (uint8_t*)output, length);
	else
		HAL_3DES_Decrypt(&ctx->des3, (uint8_t*)input, (uint8_t*)output, length);
	memcpy(iv, ctx->des3.iv, 8);

	return( 0 );
}
#endif /* MBEDTLS_CIPHER_MODE_CBC */

#endif /* !MBEDTLS_DES_ALT */

#endif /* MBEDTLS_DES_C */
