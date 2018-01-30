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

#ifndef _UTIL_ATOMIC_H_
#define _UTIL_ATOMIC_H_

#include <stdint.h>
#include "compiler.h"

static __inline int arch_atomic_read(volatile int *v)
{
	return (*v);
}

static __inline int arch_atomic_set(volatile int *v, int i)
{
	*v = i;
	return i;
}

int arch_atomic_add_return(int *v, int i);
int arch_atomic_sub_return(int *v, int i);
int arch_atomic_and_return(int *v, int i);
int arch_atomic_or_return(int *v, int i);
int arch_atomic_xor_return(int *v, int i);
int arch_atomic_nand_return(int *v, int i);

static __inline int arch_atomic_inc_return(int *v)
{
	return arch_atomic_add_return(v, 1);
}

static __inline int arch_atomic_dec_return(int *v)
{
	return arch_atomic_sub_return(v, 1);
}

static __inline void arch_atomic_add(int *v, int i)
{
	arch_atomic_add_return(v, i);
}

static __inline void arch_atomic_sub(int *v, int i)
{
	arch_atomic_sub_return(v, i);
}

static __inline void arch_atomic_and(int *v, int i)
{
	arch_atomic_and_return(v, i);
}

static __inline void arch_atomic_or(int *v, int i)
{
	arch_atomic_or_return(v, i);
}

static __inline void arch_atomic_xor(int *v, int i)
{
	arch_atomic_xor_return(v, i);
}

static __inline void arch_atomic_nand(int *v, int i)
{
	arch_atomic_nand_return(v, i);
}

static __inline void arch_atomic_inc(int *v)
{
	arch_atomic_add_return(v, 1);
}

static __inline void arch_atomic_dec(int *v)
{
	arch_atomic_sub_return(v, 1);
}

int arch_atomic_xchg(int *v, int i);
int arch_atomic_cmpxchg(int *v, int old, int new_v);

void arch_atomic_clear_mask(uint32_t *addr, uint32_t mask);
void arch_atomic_set_mask(uint32_t *addr, uint32_t mask);

#endif /* _UTIL_ATOMIC_H_ */
