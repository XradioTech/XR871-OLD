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

#include <sys/reent.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __CONFIG_MALLOC_USE_STDLIB

#ifdef __CONFIG_OS_FREERTOS
#include "kernel/os/os_thread.h"
static __inline void malloc_mutex_lock(void)
{
	OS_ThreadSuspendScheduler();
}

static __inline void malloc_mutex_unlock(void)
{
	OS_ThreadResumeScheduler();
}
#endif /* __CONFIG_OS_FREERTOS */

void *__real__malloc_r(struct _reent *reent, size_t size);
void *__real__realloc_r(struct _reent *reent, void *ptr, size_t size);
void __real__free_r(struct _reent *reent, void *ptr);

#define WRAP_MALLOC_MEM_TRACE	defined(__CONFIG_MALLOC_TRACE)

#if WRAP_MALLOC_MEM_TRACE

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define HEAP_MEM_DBG_ON         0
#define HEAP_MEM_ERR_ON         1

#define HEAP_MEM_DBG_MIN_SIZE   100
#define HEAP_MEM_MAX_CNT        1024
#define HEAP_SYSLOG             printf

#define HEAP_MEM_IS_TRACED(size)    (size > HEAP_MEM_DBG_MIN_SIZE)

#define HEAP_MEM_LOG(flags, fmt, arg...)    \
    do {                                    \
        if (flags)                          \
            HEAP_SYSLOG(fmt, ##arg);        \
    } while (0)

#define HEAP_MEM_DBG(fmt, arg...) \
	HEAP_MEM_LOG(HEAP_MEM_DBG_ON, "[heap] "fmt, ##arg)

#define HEAP_MEM_ERR(fmt, arg...) \
	HEAP_MEM_LOG(HEAP_MEM_ERR_ON, "[heap ERR] %s():%d, "fmt, \
	                              __func__, __LINE__, ##arg);

struct heap_mem {
	void *ptr;
	size_t size;
};

static struct heap_mem g_mem[HEAP_MEM_MAX_CNT];

static int g_mem_entry_cnt = 0;
static int g_mem_entry_cnt_max = 0;
static int g_mem_empty_idx = 0; /* beginning idx to do the search for new one */

static size_t g_mem_sum = 0;
static size_t g_mem_sum_max = 0;

static uint8_t g_do_reallocing = 0; /* flag for realloc() */

#define WRAP_MEM_MAGIC_LEN	4

#if (WRAP_MEM_MAGIC_LEN)
static const char g_mem_magic[WRAP_MEM_MAGIC_LEN] = {0x4a, 0x5b, 0x6c, 0x7f};
#define WRAP_MEM_SET_MAGIC(p, l)	memcpy((((char *)(p)) + (l)), g_mem_magic, 4)
#define WRAP_MEM_CHK_MAGIC(p, l)	memcmp((((char *)(p)) + (l)), g_mem_magic, 4)
#else
#define WRAP_MEM_SET_MAGIC(p, l)	do { } while (0)
#define WRAP_MEM_CHK_MAGIC(p, l)	0
#endif

uint32_t wrap_malloc_heap_info(int verbose)
{
	malloc_mutex_lock();

	HEAP_SYSLOG("<<< heap info >>>\n"
	            "g_mem_sum       %u (%u KB)\n"
	            "g_mem_sum_max   %u (%u KB)\n"
	            "g_mem_entry_cnt %u, max %u\n",
	            g_mem_sum, g_mem_sum / 1024,
	            g_mem_sum_max, g_mem_sum_max / 1024,
	            g_mem_entry_cnt, g_mem_entry_cnt_max);

	int i, j = 0;
	for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
		if (g_mem[i].ptr != NULL) {
			if (verbose) {
				HEAP_SYSLOG("%03d. %03d, %p, %u\n",
				            ++j, i, g_mem[i].ptr, g_mem[i].size);
			}

			if (WRAP_MEM_CHK_MAGIC(g_mem[i].ptr, g_mem[i].size)) {
				HEAP_MEM_ERR("mem (%p) corrupt\n", g_mem[i].ptr);
			}
		}
	}

	uint32_t ret = g_mem_sum;
	malloc_mutex_unlock();

	return ret;
}

/* Note: @ptr != NULL */
static void wrap_malloc_add_entry(void *ptr, size_t size)
{
	int i;

	WRAP_MEM_SET_MAGIC(ptr, size);

	for (i = g_mem_empty_idx; i < HEAP_MEM_MAX_CNT; ++i) {
		if (g_mem[i].ptr == NULL) {
			g_mem[i].ptr = ptr;
			g_mem[i].size = size;
			g_mem_entry_cnt++;
			g_mem_empty_idx = i + 1;
			g_mem_sum += size;
			if (g_mem_sum > g_mem_sum_max)
				g_mem_sum_max = g_mem_sum;
			if (g_mem_entry_cnt > g_mem_entry_cnt_max)
				g_mem_entry_cnt_max = g_mem_entry_cnt;
			break;
		}
	}

	if (i >= HEAP_MEM_MAX_CNT) {
		HEAP_MEM_ERR("heap mem count exceed %d\n", HEAP_MEM_MAX_CNT);
	}
}

/* Note: @ptr != NULL */
static ssize_t wrap_malloc_delete_entry(void *ptr)
{
	int i;
	ssize_t size;

	for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
		if (g_mem[i].ptr == ptr) {
			size = g_mem[i].size;
			if (WRAP_MEM_CHK_MAGIC(ptr, size)) {
				HEAP_MEM_ERR("mem f (%p, %u) corrupt\n", ptr, size);
			}
			g_mem_sum -= size;
			g_mem[i].ptr = NULL;
			g_mem[i].size = 0;
			g_mem_entry_cnt--;
			if (i < g_mem_empty_idx)
				g_mem_empty_idx = i;
			break;
		}
	}

	if (i >= HEAP_MEM_MAX_CNT) {
		HEAP_MEM_ERR("heap mem entry (%p) missed\n", ptr);
		size = -1;
	}

	return size;
}

/* Note: @old_ptr != NULL, @new_ptr != NULL, @new_size != 0 */
static ssize_t wrap_malloc_update_entry(void *old_ptr,
                                        void *new_ptr, size_t new_size)
{
	int i;
	ssize_t old_size;

	WRAP_MEM_SET_MAGIC(new_ptr, new_size);

	for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
		if (g_mem[i].ptr == old_ptr) {
			old_size = g_mem[i].size;
			g_mem_sum = g_mem_sum - old_size + new_size;
			g_mem[i].ptr = new_ptr;
			g_mem[i].size = new_size;
			if (g_mem_sum > g_mem_sum_max)
				g_mem_sum_max = g_mem_sum;
			break;
		}
	}

	if (i >= HEAP_MEM_MAX_CNT) {
		HEAP_MEM_ERR("heap mem entry (%p) missed\n", new_ptr);
		old_size = -1;
	}

	return old_size;
}


void *__wrap__malloc_r(struct _reent *reent, size_t size)
{
	malloc_mutex_lock();

	size_t real_size = size;

	if (!g_do_reallocing) {
		real_size += WRAP_MEM_MAGIC_LEN;
	}

	void *ptr = __real__malloc_r(reent, real_size);

	if (!g_do_reallocing) {
		if (HEAP_MEM_IS_TRACED(size)) {
			HEAP_MEM_DBG("m (%p, %u)\n", ptr, size);
		}

		if (ptr) {
			wrap_malloc_add_entry(ptr, size);
		} else {
			HEAP_MEM_ERR("heap mem exhausted (%u)\n", size);
		}
	}

	malloc_mutex_unlock();

	return ptr;
}

void *__wrap__realloc_r(struct _reent *reent, void *ptr, size_t size)
{
	void *new_ptr;
	ssize_t old_size;

	malloc_mutex_lock();
	g_do_reallocing = 1;

	size_t real_size = size;

	if (size != 0) { /* (size == 0) means free it */
		real_size += WRAP_MEM_MAGIC_LEN;
	}

	new_ptr = __real__realloc_r(reent, ptr, real_size);

	if (ptr == NULL) {
		old_size = 0;
		if (new_ptr != NULL) {
			wrap_malloc_add_entry(new_ptr, size);
		} else {
			if (size != 0) {
				HEAP_MEM_ERR("heap mem exhausted (%p, %u)\n", ptr, size);
				goto out;
			}
		}
	} else {
		if (size == 0) {
			if (new_ptr != NULL) {
				HEAP_MEM_ERR("realloc (%p, %u) return %p\n", ptr, size, new_ptr);
			}
			old_size = wrap_malloc_delete_entry(ptr);
		} else {
			if (new_ptr != NULL) {
				old_size = wrap_malloc_update_entry(ptr, new_ptr, size);
			} else {
				HEAP_MEM_ERR("heap mem exhausted (%p, %u)\n", ptr, size);
				goto out;
			}
		}
	}

	if (HEAP_MEM_IS_TRACED(size) || HEAP_MEM_IS_TRACED(old_size)) {
		HEAP_MEM_DBG("r (%p, %u) <- (%p, %u)\n", new_ptr, size, ptr, old_size);
	}

out:
	g_do_reallocing = 0;
	malloc_mutex_unlock();

	return new_ptr;
}

void __wrap__free_r(struct _reent *reent, void *ptr)
{
	malloc_mutex_lock();

	if (!g_do_reallocing && ptr) {
		ssize_t size = wrap_malloc_delete_entry(ptr);
		if (HEAP_MEM_IS_TRACED(size)) {
			HEAP_MEM_DBG("f (%p, %u)\n", ptr, size);
		}
	}

	__real__free_r(reent, ptr);

	malloc_mutex_unlock();
}

#else /* WRAP_MALLOC_MEM_TRACE */

void *__wrap__malloc_r(struct _reent *reent, size_t size)
{
	void *ptr;

	malloc_mutex_lock();
	ptr = __real__malloc_r(reent, size);
	malloc_mutex_unlock();

	return ptr;
}

void *__wrap__realloc_r(struct _reent *reent, void *ptr, size_t size)
{
	void *new_ptr;

	malloc_mutex_lock();
	new_ptr = __real__realloc_r(reent, ptr, size);
	malloc_mutex_unlock();

	return new_ptr;
}

void __wrap__free_r(struct _reent *reent, void *ptr)
{
	malloc_mutex_lock();
	__real__free_r(reent, ptr);
	malloc_mutex_unlock();
}

#endif /* WRAP_MALLOC_MEM_TRACE */

#elif (defined(__CONFIG_OS_FREERTOS))

void *pvPortMalloc( size_t xWantedSize );
void vPortFree( void *pv );

void *__wrap_malloc(size_t size)
{
	return pvPortMalloc(size);
}

void *__wrap_realloc(void *ptr, size_t size)
{
#error "realloc() not support"
	vPortFree(ptr);
	return pvPortMalloc(size);
}

void __wrap_free(void *ptr)
{
	vPortFree(ptr);
}

void *__wrap__malloc_r(struct _reent *reent, size_t size)
{
	return pvPortMalloc(size);
}

void *__wrap__realloc_r(struct _reent *reent, void *ptr, size_t size)
{
#error "realloc() not support"
	vPortFree(ptr);
	return pvPortMalloc(size);
}

void __wrap__free_r(struct _reent *reent, void *ptr)
{
	vPortFree(ptr);
}

#endif /* __CONFIG_MALLOC_USE_STDLIB */
