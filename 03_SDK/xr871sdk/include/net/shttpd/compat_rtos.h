#ifndef __COMPAT_RTOS_H_H
#define __COMPAT_RTOS_H_H

#include "lwip/sockets.h"
#include "kernel/os/os.h"

#define	DIRSEP		'/'
#define	IS_DIRSEP_CHAR(c)		((c) == '/')
#define USE_HEAPMEM
#define NO_CGI
#define NO_SSL
#define NO_AUTH
#define HAVE_MD5
#define NO_FS
#define NO_RANGE

//#define NO_SSI
#define NO_SSI_INCLUDE
#define NO_SSI_CONDITION
#define NO_SSI_EXEC
//#define NO_SSI_CALL

#define NO_UID
#define NO_ACL
#define NO_INETD
#define NO_HTPASSWD
#define NO_PUTDEL
#define NO_COMMAND
#define NO_MCON
#define NO_THREADS

#define CUSTOM_LOG
#define CUSTOM_LOG_ON
#define DEBUG_ON

#if !defined(O_NONBLOCK)
#define O_NONBLOCK		1
#endif

#include "kernel/os/os_time.h"
#include <time.h>
time_t TIME(time_t *timer);

#include <errno.h>
#define ERRNO		errno

struct _stat {
	unsigned long st_size;
	int st_mode;
};

struct usrfile {
	char *name;
	char *body;
};

#define stat _stat

#define _S_IFMT		(0x0170000)
#define _S_IFDIR		(0x0040000)
#define _S_IFREG		(0x0100000)
#define _S_IEXEC		(0x0000100)
#define _S_IWRITE		(0x0000200)
#define _S_IREAD		(0x0000400)

#define _S_ISDIR(x)		((x) & _S_IFDIR)

#define ARRAY_SIZE(arr) (sizeof((arr))/sizeof((arr)[0]))

void *zalloc(size_t size);
void _shttpd_init_local(struct usrfile *filelist,int count);

#if !defined(NO_THREADS)

#define HTTP_THREAD_STACK_SIZE	(4 * 1024)

#define _beginthread(a, b, c) do { \
	OS_Thread_t http_thread;\
	OS_ThreadCreate(&http_thread,\
                                "",\
                                a,\
                                (void *)c,\
                                OS_THREAD_PRIO_APP,\
                                HTTP_THREAD_STACK_SIZE);} while (0)
#endif /* !NO_THREADS */

#endif


