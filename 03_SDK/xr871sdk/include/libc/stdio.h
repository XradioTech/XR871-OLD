#ifndef _LIBC_STDIO_H_
#define _LIBC_STDIO_H_

#include_next <stdio.h>

#ifdef __CONFIG_LIBC_WRAP_STDIO

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*stdio_write_fn)(const char *buf, int len);

void stdio_set_write(stdio_write_fn fn);

void stdout_mutex_lock(void);
void stdout_mutex_unlock(void);

#undef putc
#undef putchar

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_LIBC_WRAP_STDIO */

#endif /* _LIBC_STDIO_H_ */
