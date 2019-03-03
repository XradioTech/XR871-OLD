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

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "compiler.h"

#if (defined(__ARM_ARCH_7EM__) && defined(__ARM_FEATURE_UNALIGNED))
#define MEM_OPT_UNALIGNED_ACCESS	1
#else
#define MEM_OPT_UNALIGNED_ACCESS	0
#endif

#define MEM_ALIGN_MOD(addr)	((uint32_t)(addr) & 0x3)
#define MEM_UNALIGNED(addr)	MEM_ALIGN_MOD(addr)

static __always_inline
void *memcpy_backward(void *dst0, const void *src0, size_t len)
{
	uint8_t *dst = (uint8_t *)dst0 + len;
	const uint8_t *src = (const uint8_t *)src0 + len;

#if MEM_OPT_UNALIGNED_ACCESS
	if (len < 8) {
		goto byte_copy;
	}

	if (MEM_UNALIGNED(src)) {
		while (MEM_UNALIGNED(dst)) {
			*(--dst) = *(--src);
			--len;
		}
	}
#else /* MEM_OPT_UNALIGNED_ACCESS */
	if (len < 12) {
		goto byte_copy;
	}

	if (MEM_ALIGN_MOD(dst) != MEM_ALIGN_MOD(src)) {
		/* cannot be aligned */
		goto byte_copy;
	} else {
		/* can be aligned */
		while (MEM_UNALIGNED(dst)) {
			*(--dst) = *(--src);
			--len;
		}
	}
#endif /* MEM_OPT_UNALIGNED_ACCESS */

	uint32_t *dst_aligned = (uint32_t *)dst;
	const uint32_t *src_aligned = (const uint32_t *)src;

	while (len >= 4 * 16) {
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		len -= 4 * 16;
	}

	while (len >= 4 * 4) {
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		*(--dst_aligned) = *(--src_aligned);
		len -= 4 * 4;
	}

	while (len >= 4) {
		*(--dst_aligned) = *(--src_aligned);
		len -= 4;
	}

	dst = (uint8_t *)dst_aligned;
	src = (const uint8_t *)src_aligned;

byte_copy:
	while (len) {
		*(--dst) = *(--src);
		--len;
	}

	return dst0;
}

__attribute__ ((__optimize__ ("-O2")))
void *__wrap_memmove(void *dst0, const void *src0, size_t len)
{
	uint8_t *dst = (uint8_t *)dst0;
	const uint8_t *src = (const uint8_t *)src0;

	if (dst > src && dst < src + len) {
		/* copy backwards */
		return memcpy_backward(dst, src, len);
	} else {
		/* copy forwards */
		return memcpy(dst, src, len);
	}
}
