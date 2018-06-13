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

void *__real_malloc(size_t size);
void *__real_realloc(void *ptr, size_t size);
void __real_free(void *ptr);

void *__real__malloc_r(struct _reent *reent, size_t size);
void *__real__realloc_r(struct _reent *reent, void *ptr, size_t size);
void __real__free_r(struct _reent *reent, void *ptr);

#define WRAP_MALLOC_MEM_TRACE	defined(__CONFIG_MALLOC_TRACE)

#if WRAP_MALLOC_MEM_TRACE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define HEAP_MEM_DBG_ON 		0
#define HEAP_MEM_ERR_ON 		1

#define HEAP_MEM_DBG_MIN_SIZE	100
#define HEAP_MEM_MAX_CNT		1024
#define HEAP_SYSLOG 			printf

#define HEAP_MEM_IS_TRACED(size)	(size > HEAP_MEM_DBG_MIN_SIZE)

#define HEAP_MEM_LOG(flags, fmt, arg...)	\
	do {									\
		if (flags) 							\
			HEAP_SYSLOG(fmt, ##arg);		\
	} while (0)

#define HEAP_MEM_DBG(fmt, arg...)	\
	HEAP_MEM_LOG(HEAP_MEM_DBG_ON, "[malloc] "fmt, ##arg)

#define HEAP_MEM_ERR(fmt, arg...)	\
	HEAP_MEM_LOG(HEAP_MEM_ERR_ON, "[malloc ERR] %s():%d, "fmt,	\
		__func__, __LINE__, ##arg);

struct heap_mem {
	void *ptr;
	size_t size;
};

static struct heap_mem g_mem[HEAP_MEM_MAX_CNT];

static int g_mem_entry_cnt = 0;
static int g_mem_entry_cnt_max = 0;
static int g_mem_empty_idx = 0; // beginning idx to do the search for new one

static size_t g_mem_sum = 0;
static size_t g_mem_sum_max = 0;

#define WRAP_MEM_MAGIC_LEN	4
static const char g_mem_magic[WRAP_MEM_MAGIC_LEN] = {0x4a, 0x5b, 0x6c, 0x7f};
#define WRAP_MEM_SET_MAGIC(p, l)	memcpy((((char *)(p)) + (l)), g_mem_magic, 4)
#define WRAP_MEM_CHK_MAGIC(p, l)	memcmp((((char *)(p)) + (l)), g_mem_magic, 4)

uint32_t wrap_malloc_heap_info(int check_only)
{
	HEAP_SYSLOG("<<< malloc heap info >>>\n"
		    "g_mem_sum       %u (%u KB)\n"
		    "g_mem_sum_max   %u (%u KB)\n"
		    "g_mem_entry_cnt %u, max %u\n",
		    g_mem_sum, g_mem_sum / 1024,
		    g_mem_sum_max, g_mem_sum_max / 1024,
		    g_mem_entry_cnt, g_mem_entry_cnt_max);

	int i, j = 0;
	for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
		if (g_mem[i].ptr != 0) {
			if (!check_only) {
				HEAP_SYSLOG("%03d. %03d, %p, %u\n",
					    ++j, i, g_mem[i].ptr, g_mem[i].size);
			}

			if (WRAP_MEM_CHK_MAGIC(g_mem[i].ptr, g_mem[i].size)) {
				HEAP_MEM_ERR("memory (%p) corrupt\n", g_mem[i].ptr);
			}
		}
	}
	return g_mem_sum;
}

void *__wrap_malloc(size_t size)
{
	malloc_mutex_lock();
	void *ptr = __real_malloc(size + WRAP_MEM_MAGIC_LEN);

	if (ptr) {
		WRAP_MEM_SET_MAGIC(ptr, size);
		if (HEAP_MEM_IS_TRACED(size)) {
			HEAP_MEM_DBG("m (%p, %u)\n", ptr, size);
		}
		int i;
		for (i = g_mem_empty_idx; i < HEAP_MEM_MAX_CNT; ++i) {
			if (g_mem[i].ptr == 0) {
				/* add new entry */
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
			HEAP_MEM_ERR("heap memory count exceed %d\n",
				     HEAP_MEM_MAX_CNT);
		}
	} else {
		HEAP_MEM_ERR("heap memory exhausted!\n");
	}
	malloc_mutex_unlock();

	return ptr;
}

void *__wrap_realloc(void *ptr, size_t size)
{
	malloc_mutex_lock();
	void *p = __real_realloc(ptr, size + WRAP_MEM_MAGIC_LEN);

	if (p) {
		WRAP_MEM_SET_MAGIC(p, size);
		int i;
		for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
			if (g_mem[i].ptr != ptr)
				continue;

			/* update the old entry */
			if (HEAP_MEM_IS_TRACED(size)) {
				HEAP_MEM_DBG("r (%p, %u) <- (%p, %u)\n",
					     p, size,
					     g_mem[i].ptr, g_mem[i].size);
			}
			g_mem_sum -= g_mem[i].size;
			g_mem[i].ptr = p;
			g_mem[i].size = size;
			g_mem_sum += size;
			if (g_mem_sum > g_mem_sum_max)
				g_mem_sum_max = g_mem_sum;
			if (ptr == NULL) {
				g_mem_entry_cnt++;
				g_mem_empty_idx = i + 1;
				if (g_mem_entry_cnt > g_mem_entry_cnt_max)
					g_mem_entry_cnt_max = g_mem_entry_cnt;
			}
			break;
		}
		if (i >= HEAP_MEM_MAX_CNT) {
			HEAP_MEM_ERR("heap memory entry (%p) missed\n", p);
		}
	} else {
		HEAP_MEM_ERR("heap memory exhausted!\n");
	}
	malloc_mutex_unlock();

	return p;
}

void __wrap_free(void *ptr)
{
	malloc_mutex_lock();
	if (ptr) {
		int i;
		for (i = 0; i < HEAP_MEM_MAX_CNT; ++i) {
			if (g_mem[i].ptr != ptr)
				continue;

			/* delete the old entry */
			if (HEAP_MEM_IS_TRACED(g_mem[i].size)) {
				HEAP_MEM_DBG("f (%p, %u)\n",
					     g_mem[i].ptr, g_mem[i].size);
			}
			if (WRAP_MEM_CHK_MAGIC(g_mem[i].ptr, g_mem[i].size)) {
				HEAP_MEM_ERR("memory f (%p) corrupt\n", ptr);
			}
			g_mem_sum -= g_mem[i].size;
			g_mem[i].ptr = 0;
			g_mem[i].size = 0;
			g_mem_entry_cnt--;
			if (i < g_mem_empty_idx)
				g_mem_empty_idx = i;
			break;
		}
		if (i >= HEAP_MEM_MAX_CNT) {
			HEAP_MEM_ERR("heap memory entry (%p) missed\n", ptr);
		}
	}

	__real_free(ptr);
	malloc_mutex_unlock();
}

void *__wrap_calloc(size_t cnt, size_t size)
{
	void *ptr = malloc(cnt * size);
	if (ptr) {
		memset(ptr, 0, cnt * size);
	}
	return ptr;
}

char *__wrap_strdup(const char *s)
{
	char *res;
	size_t len;
	if (s == NULL)
		return NULL;
	len = strlen(s);
	res = malloc(len + 1);
	if (res)
		memcpy(res, s, len + 1);
	return res;
}

#else /* WRAP_MALLOC_MEM_TRACE */

void *__wrap_malloc(size_t size)
{
	void *ret;

	malloc_mutex_lock();
	ret = __real_malloc(size);
	malloc_mutex_unlock();

	return ret;
}

void *__wrap_realloc(void *ptr, size_t size)
{
	void *ret;

	malloc_mutex_lock();
	ret = __real_realloc(ptr, size);
	malloc_mutex_unlock();

	return ret;
}

void __wrap_free(void *ptr)
{
	malloc_mutex_lock();
	__real_free(ptr);
	malloc_mutex_unlock();
}

#endif /* WRAP_MALLOC_MEM_TRACE */

void *__wrap__malloc_r(struct _reent *reent, size_t size)
{
	void *ret;

	malloc_mutex_lock();
	ret = __real__malloc_r(reent, size);
	malloc_mutex_unlock();

	return ret;
}

void *__wrap__realloc_r(struct _reent *reent, void *ptr, size_t size)
{
	void *ret;

	malloc_mutex_lock();
	ret = __real__realloc_r(reent, ptr, size);
	malloc_mutex_unlock();

	return ret;
}

void __wrap__free_r(struct _reent *reent, void *ptr)
{
	malloc_mutex_lock();
	__real__free_r(reent, ptr);
	malloc_mutex_unlock();
}

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
