#ifndef _LIBC_SYS_TIME_H_
#define _LIBC_SYS_TIME_H_

#include_next <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif

void timeofday_save(void);
void timeofday_restore(void);

#ifdef __cplusplus
}
#endif

#endif /* _LIBC_SYS_TIME_H_ */
