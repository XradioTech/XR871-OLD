/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/sys.h"
#include "lwip/stats.h"

#include "arch/sys_arch.h"

#if LWIP_RESOURCE_TRACE
int g_lwip_mutex_cnt = 0;
int g_lwip_sem_cnt = 0;
int g_lwip_mbox_cnt = 0;
#endif

#if (NO_SYS == 0)

#if !LWIP_COMPAT_MUTEX
/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
err_t sys_mutex_new(sys_mutex_t *mutex)
{
#if LWIP_RESOURCE_TRACE
	g_lwip_mutex_cnt++;
#endif
	err_t err = OS_MutexCreate(mutex);
	if (err == OS_OK) {
		SYS_STATS_INC_USED(mutex);
		return ERR_OK;
	} else {
		SYS_STATS_INC(mutex.err);
		return ERR_MEM;
	}
}

/** Lock a mutex
 * @param mutex the mutex to lock */
void sys_mutex_lock(sys_mutex_t *mutex)
{
	OS_MutexLock(mutex, OS_WAIT_FOREVER);
}

/** Unlock a mutex
 * @param mutex the mutex to unlock */
void sys_mutex_unlock(sys_mutex_t *mutex)
{
	OS_MutexUnlock(mutex);
}

/** Delete a semaphore
 * @param mutex the mutex to delete */
void sys_mutex_free(sys_mutex_t *mutex)
{
	if (OS_MutexDelete(mutex) == OS_OK) {
		SYS_STATS_DEC(mutex.used);
	} else {
		LWIP_ERROR("sys_mutex_free: failed", 0, while(1));
		SYS_STATS_INC(mutex.err);
	}
#if LWIP_RESOURCE_TRACE
	g_lwip_mutex_cnt--;
#endif
}
#endif /* LWIP_COMPAT_MUTEX */

/** Create a new semaphore
 * @param sem pointer to the semaphore to create
 * @param count initial count of the semaphore
 * @return ERR_OK if successful, another err_t otherwise */
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
#if LWIP_RESOURCE_TRACE
	g_lwip_sem_cnt++;
#endif
	err_t err = OS_SemaphoreCreate(sem, count, OS_SEMAPHORE_MAX_COUNT);
	if (err == OS_OK) {
		SYS_STATS_INC_USED(sem);
		return ERR_OK;
	} else {
		SYS_STATS_INC(sem.err);
		return ERR_MEM;
	}
}

/** Signals a semaphore
 * @param sem the semaphore to signal */
void sys_sem_signal(sys_sem_t *sem)
{
	OS_SemaphoreRelease(sem);
}

/** Wait for a semaphore for the specified timeout
 * @param sem the semaphore to wait for
 * @param timeout timeout in milliseconds to wait (0 = wait forever)
 * @return time (in milliseconds) waited for the semaphore
 *         or SYS_ARCH_TIMEOUT on timeout */
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
	u32_t start_ticks = OS_GetTicks();
	if (OS_SemaphoreWait(sem, timeout == 0 ? OS_WAIT_FOREVER : timeout) != OS_OK)
		return SYS_ARCH_TIMEOUT;

	return OS_TicksToMSecs(OS_GetTicks() - start_ticks);
}

/** Delete a semaphore
 * @param sem semaphore to delete */
void sys_sem_free(sys_sem_t *sem)
{
	if (OS_SemaphoreDelete(sem) == OS_OK) {
		SYS_STATS_DEC(sem.used);
	} else {
		LWIP_ERROR("sys_sem_free: failed", 0, while(1));
		SYS_STATS_INC(sem.err);
	}
#if LWIP_RESOURCE_TRACE
	g_lwip_sem_cnt--;
#endif
}

#if LWIP_MBOX_TRACE
#define MBOX2Q(m)		(&((m)->q))
#define MBOX_INC_USED(m)							\
	do {									\
		if (++(m)->used > (m)->max) {					\
			(m)->max = (m)->used;					\
			LWIP_PLATFORM_DIAG(("mbox %p, avail %u, max %u\n",	\
				(m), (m)->avail, (m)->max));			\
		}								\
	} while (0)
#define MBOX_DEC_USED(m)	do { --(m)->used; } while (0)
#define MBOX_INC_ERR(m)		do { ++(m)->err; } while (0)
#else /* LWIP_MBOX_TRACE */
#define MBOX2Q(m)		(m)
#define MBOX_INC_USED(m)	do { } while (0)
#define MBOX_DEC_USED(m)	do { } while (0)
#define MBOX_INC_ERR(m)		do { } while (0)
#endif /* LWIP_MBOX_TRACE */

/** Create a new mbox of specified size
 * @param mbox pointer to the mbox to create
 * @param size (miminum) number of messages in this mbox
 * @return ERR_OK if successful, another err_t otherwise */
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
#if LWIP_RESOURCE_TRACE
	g_lwip_mbox_cnt++;
#endif
	err_t err = OS_MsgQueueCreate(MBOX2Q(mbox), size);
	if (err == OS_OK) {
		SYS_STATS_INC_USED(mbox);
#if LWIP_MBOX_TRACE
		mbox->avail = size;
		mbox->used = 0;
		mbox->max = 0;
		mbox->err = 0;
#endif
		return ERR_OK;
	} else {
		SYS_STATS_INC(mbox.err);
		return ERR_MEM;
	}
}

/** Post a message to an mbox - may not fail
 * -> blocks if full, only used from tasks not from ISR
 * @param mbox mbox to posts the message
 * @param msg message to post (ATTENTION: can be NULL) */
void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
	OS_MsgQueueSend(MBOX2Q(mbox), msg, OS_WAIT_FOREVER);
	MBOX_INC_USED(mbox);
}

/** Try to post a message to an mbox - may fail if full or ISR
 * @param mbox mbox to posts the message
 * @param msg message to post (ATTENTION: can be NULL) */
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	err_t err = OS_MsgQueueSend(MBOX2Q(mbox), msg, 0);
	if (err == OS_OK) {
		MBOX_INC_USED(mbox);
		return ERR_OK;
	} else {
		MBOX_INC_ERR(mbox);
		return ERR_TIMEOUT;
	}
}

/** Wait for a new message to arrive in the mbox
 * @param mbox mbox to get a message from
 * @param msg pointer where the message is stored
 * @param timeout maximum time (in milliseconds) to wait for a message
 * @return time (in milliseconds) waited for a message, may be 0 if not waited
           or SYS_ARCH_TIMEOUT on timeout
 *         The returned time has to be accurate to prevent timer jitter! */
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	u32_t start_ticks = OS_GetTicks();
	if (OS_MsgQueueReceive(MBOX2Q(mbox), msg, timeout == 0 ? OS_WAIT_FOREVER : timeout) != OS_OK)
		return SYS_ARCH_TIMEOUT;

	MBOX_DEC_USED(mbox);
	return OS_TicksToMSecs(OS_GetTicks() - start_ticks);
}

/** Try to wait for a new message to arrive in the mbox
 * @param mbox mbox to get a message from
 * @param msg pointer where the message is stored
 * @return 0 (milliseconds) if a message has been received
 *         or SYS_MBOX_EMPTY if the mailbox is empty */
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	if (OS_MsgQueueReceive(MBOX2Q(mbox), msg, 0) != OS_OK) {
		return SYS_MBOX_EMPTY;
	} else {
		MBOX_DEC_USED(mbox);
		return 0;
	}
}

/** Delete an mbox
 * @param mbox mbox to delete */
void sys_mbox_free(sys_mbox_t *mbox)
{
#if LWIP_MBOX_TRACE
	LWIP_PLATFORM_DIAG(("free mbox %p, avail %u, used %u, max %u, err %u\n",
		mbox, mbox->avail, mbox->used, mbox->max, mbox->err));
#endif
	if (OS_MsgQueueDelete(MBOX2Q(mbox)) == OS_OK) {
		SYS_STATS_DEC(mbox.used);
	} else {
		LWIP_ERROR("sys_mbox_free: mbox not empty", 0, while(1));
		SYS_STATS_INC(mbox.err);
	}
#if LWIP_RESOURCE_TRACE
	g_lwip_mbox_cnt--;
#endif
}

/* Support only one tcpip thread to save space */
static OS_Thread_t g_lwip_tcpip_thread;

/** The only thread function:
 * Creates a new thread
 * @param name human-readable name for the thread (used for debugging purposes)
 * @param thread thread-function
 * @param arg parameter passed to 'thread'
 * @param stacksize stack size in bytes for the new thread (may be ignored by ports)
 * @param prio priority of the new thread (may be ignored by ports) */
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
	if (OS_ThreadIsValid(&g_lwip_tcpip_thread))
		return NULL;

	if (OS_ThreadCreate(&g_lwip_tcpip_thread, name, thread, arg, prio, stacksize) != OS_OK) {
		return NULL;
	}
	return &g_lwip_tcpip_thread;
}

#if LWIP_XR_DEINIT

/** Thread terminate itself */
void sys_thread_exit(const char *name)
{
	OS_ThreadDelete(&g_lwip_tcpip_thread);
}

/** Free a thread, or wait for thread termination */
void sys_thread_free(const char *name)
{
	while (OS_ThreadIsValid(&g_lwip_tcpip_thread)) {
		OS_MSleep(1); /* wait for thread termination */
	}
}
#endif /* */

#endif /* (NO_SYS == 0) */

#if (SYS_LIGHTWEIGHT_PROT && SYS_ARCH_PROTECT_USE_MUTEX)
/** mutex for SYS_ARCH_PROTECT */
OS_Mutex_t g_lwip_sys_mutex;
#endif

/** sys_init() must be called before anthing else. */
void sys_init(void)
{
#if (SYS_LIGHTWEIGHT_PROT && SYS_ARCH_PROTECT_USE_MUTEX)
	OS_RecursiveMutexCreate(&g_lwip_sys_mutex);
#endif
}

#if LWIP_XR_DEINIT
/** sys_deinit() must be called after anthing else. */
void sys_deinit(void)
{
#if (SYS_LIGHTWEIGHT_PROT && SYS_ARCH_PROTECT_USE_MUTEX)
	OS_RecursiveMutexDelete(&g_lwip_sys_mutex);
#endif
}
#endif /* LWIP_XR_DEINIT */

/** Returns the current time in milliseconds,
 * may be the same as sys_jiffies or at least based on it. */
u32_t sys_now(void)
{
	return OS_TicksToMSecs(OS_GetTicks());
}

#if LWIP_RESOURCE_TRACE
extern int g_lwip_socket_cnt;

void lwip_resource_info(void)
{
	LWIP_PLATFORM_DIAG(("<<< lwip resource info >>>\n"));
	LWIP_PLATFORM_DIAG(("g_lwip_socket_cnt %d\n", g_lwip_socket_cnt));
	LWIP_PLATFORM_DIAG(("g_lwip_mutex_cnt  %d\n", g_lwip_mutex_cnt));
	LWIP_PLATFORM_DIAG(("g_lwip_sem_cnt    %d\n", g_lwip_sem_cnt));
	LWIP_PLATFORM_DIAG(("g_lwip_mbox_cnt   %d\n", g_lwip_mbox_cnt));
}
#endif /* LWIP_RESOURCE_TRACE */
