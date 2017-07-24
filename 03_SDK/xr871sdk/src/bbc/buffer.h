#ifndef BUFFER_H
#define BUFFER_H

#include <stdlib.h>
#include <strings.h>
#include <stdarg.h>

struct Buffer {
    char *contents;
    int bytes_used;
    int total_size;
};

typedef struct Buffer Buffer;

Buffer * buffer_alloc(int initial_size);
int buffer_strlen(Buffer *buf);
void buffer_free(Buffer *buf);
int buffer_append(Buffer *buf, char *append, int length);
int buffer_appendf(Buffer *buf, const char *format, ...);
int buffer_nappendf(Buffer *buf, size_t length, const char *format, ...);
char *buffer_to_s(Buffer *buf);

#endif