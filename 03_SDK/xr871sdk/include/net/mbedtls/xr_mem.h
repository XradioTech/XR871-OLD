#ifndef XR_MEM__H
#define XR_MEM__H

struct os_heap_mem {
        void *ptr;
        size_t size;
};

void heap_usage();
void * XR_MALLOC(size_t size);
void *XR_CALLOC(size_t nmemb, size_t size);
void * XR_REALLOC(void *ptr, size_t size);
void XR_FREE(void *ptr);

#endif
