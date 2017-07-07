#include <stdlib.h>
#include <stdio.h>
#include <mbedtls/xr_mem.h>

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(XR_MEM_DBG)
#define MEM_TRACE_SIZE 1
#define OS_HEAP_MEM_MAX_CNT 128
struct os_heap_mem g_mem[OS_HEAP_MEM_MAX_CNT];
int g_mem_entry_cnt = 0;
int g_mem_entry_cnt_max = 0;
int g_mem_empty_idx = 0; 

size_t g_mem_sum = 0;
size_t g_mem_sum_max = 0;

void heap_usage()
{
        int i, j = 0;
        printf("===== https heap usage =====\n"
                                "g_mem_sum: %u (%u KB)\n"
                                "g_mem_sum_max: %u (%u KB)\n"
                                "g_mem_entry_cnt: %u, max %u\n",
                                g_mem_sum, g_mem_sum / 1024,
                                g_mem_sum_max, g_mem_sum_max / 1024,
                                g_mem_entry_cnt, g_mem_entry_cnt_max);

        for (i = 0; i < OS_HEAP_MEM_MAX_CNT; ++i) {
                if (g_mem[i].ptr != 0) {
                        ++j;
                        printf("%03d. %03d, %p, %u (%u KB)\n",
                                                j, i, g_mem[i].ptr,
                                                g_mem[i].size, g_mem[i].size / 1024);
                }
        }
}

void * XR_MALLOC(size_t size)
{
        void *p = malloc(size);

        if (p) {
                if (size > MEM_TRACE_SIZE) {
                       printf("malloc (%p, %u)\n", p, size);
                }
                int i;
                for (i = g_mem_empty_idx; i < OS_HEAP_MEM_MAX_CNT; ++i) {
                        if (g_mem[i].ptr == 0) {
                                /* add new entry */
                                g_mem[i].ptr = p;
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
                if (i >= OS_HEAP_MEM_MAX_CNT) {
                        printf("heap memory count exceed %d\n",
                                                OS_HEAP_MEM_MAX_CNT);
                }
        } else {
                printf("heap memory exhausted!\n");
        }

        return p;
}
void *XR_CALLOC(size_t nmemb, size_t size)
{
        void *p = calloc(nmemb, size);

        if (p) {
                if (size > MEM_TRACE_SIZE) {
                        printf("calloc (%p, %u)\n", p, size);
                }
                int i;
                for (i = g_mem_empty_idx; i < OS_HEAP_MEM_MAX_CNT; ++i) {
                        if (g_mem[i].ptr == 0) {
                                /* add new entry */
                                g_mem[i].ptr = p;
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
                if (i >= OS_HEAP_MEM_MAX_CNT) {
                        printf("heap memory count exceed %d\n",
                                                OS_HEAP_MEM_MAX_CNT);
                }
        } else {
                printf("heap memory exhausted!\n");
        }

        return p;
}


void * XR_REALLOC(void *ptr, size_t size)
{
        void *p = realloc(ptr, size);

        if (p) {
                int i;
                for (i = 0; i < OS_HEAP_MEM_MAX_CNT; ++i) {
                        if (g_mem[i].ptr != ptr)
                                continue;

                        /* update the old entry */
                        if (size > MEM_TRACE_SIZE) {
                               printf("r (%p, %u) <- (%p, %u)\n",
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
                if (i >= OS_HEAP_MEM_MAX_CNT) {
                       printf("heap memory entry (%p) missed\n", ptr);
                }
        } else {
                printf("heap memory exhausted!\n");
        }

        return p;
}

void XR_FREE(void *ptr)
{
        if (ptr) {
                int i;
                for (i = 0; i < OS_HEAP_MEM_MAX_CNT; ++i) {
                        if (g_mem[i].ptr != ptr)
                                continue;

                        /* delete the old entry */
                        if (g_mem[i].size > MEM_TRACE_SIZE) {
                                printf("f (%p, %u)\n",
                                                g_mem[i].ptr, g_mem[i].size);
                        }
                        g_mem_sum -= g_mem[i].size;
                        g_mem[i].ptr = 0;
                        g_mem[i].size = 0;
                        g_mem_entry_cnt--;
                        if (i < g_mem_empty_idx)
                                g_mem_empty_idx = i;
                        break;
                }
                if (i >= OS_HEAP_MEM_MAX_CNT) {
                        printf("heap memory entry (%p) missed\n", ptr);
                }
        }

        free(ptr);
}

#endif
