
#ifndef CDX_MALLOC_H
#define CDX_MALLOC_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CDX_MEM_TRACE   0
#define CDX_MEM_TRACE_SAVE_FILE_LINE  0

#if CDX_MEM_TRACE
void *cdx_malloc(size_t size, const char *file, int line);
void *cdx_realloc(void *ptr, size_t size, const char *file, int line);
void *cdx_calloc(size_t cnt, size_t size, const char *file, int line);
char *cdx_strdup(const char *s, const char *file, int line);
void cdx_free(void *ptr, const char *file, int line);

#define malloc(s) cdx_malloc(s, __FILE__, __LINE__)
#define calloc(n,s) cdx_calloc(n, s, __FILE__, __LINE__)
#define free(p) cdx_free(p, __FILE__, __LINE__)
#define strdup(s) cdx_strdup(s, __FILE__, __LINE__)
#define realloc(p,s) cdx_realloc(p, s, __FILE__, __LINE__)
#endif

#endif
