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

#include <stdlib.h>
#include "sys/interrupt.h"
#include "util/atomic.h"

#define ENTER_CRITICAL() arch_irq_disable()

#define EXIT_CRITICAL() arch_irq_enable()


int arch_atomic_add_return(int *v, int i)
{
	int val;

	ENTER_CRITICAL();
	val = *v;
	*v = val += i;
	EXIT_CRITICAL();

	return val;
}

int arch_atomic_sub_return(int *v, int i)
{
	int val;

	ENTER_CRITICAL();
	val = *v;
	*v = val -= i;
	EXIT_CRITICAL();

	return val;
}

int arch_atomic_and_return(int *v, int i)
{
	int val;

	ENTER_CRITICAL();
	val = *v;
	*v = val &= i;
	EXIT_CRITICAL();

	return val;
}

int arch_atomic_or_return(int *v, int i)
{
	int val;

	ENTER_CRITICAL();
	val = *v;
	*v = val |= i;
	EXIT_CRITICAL();

	return val;
}

int arch_atomic_xor_return(int *v, int i)
{
	int val;

	ENTER_CRITICAL();
	val = *v;
	*v = val ^= i;
	EXIT_CRITICAL();

	return val;
}

int arch_atomic_nand_return(int *v, int i)
{
	int val;

	ENTER_CRITICAL();
	val = *v;
	val &= i;
	val = ~val;
	*v = val;
	EXIT_CRITICAL();

	return val;
}

int arch_atomic_cmpxchg(int *v, int old, int new_v)
{
	int ret;

	ENTER_CRITICAL();
	ret = *v;
	if (ret == old)
		*v = new_v;
	EXIT_CRITICAL();

	return ret;
}

void arch_atomic_clear_mask(uint32_t *addr, uint32_t mask)
{
	ENTER_CRITICAL();
	*addr &= ~mask;
	EXIT_CRITICAL();
}

void arch_atomic_set_mask(uint32_t *addr, uint32_t mask)
{
	ENTER_CRITICAL();
	*addr |= mask;
	EXIT_CRITICAL();
}

int arch_atomic_xchg(int *v, int i)
{
	int val;

	ENTER_CRITICAL();
	val = *v;
	*v = i;
	EXIT_CRITICAL();

	return val;
}
