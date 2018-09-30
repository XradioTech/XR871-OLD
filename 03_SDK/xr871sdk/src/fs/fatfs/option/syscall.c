/*------------------------------------------------------------------------*/
/* Sample code of OS dependent controls for FatFs                         */
/* (C)ChaN, 2014                                                          */
/*------------------------------------------------------------------------*/


#include "fs/fatfs/ff.h"
#include <stdlib.h>
#include <string.h>


#if _FS_REENTRANT
/*------------------------------------------------------------------------*/
/* Create a Synchronization Object										  */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount() function to create a new
/  synchronization object, such as semaphore and mutex. When a 0 is returned,
/  the f_mount() function fails with FR_INT_ERR.
*/
#define FF_USE_STATIC_MUTEX

#ifdef FF_USE_STATIC_MUTEX
static OS_Mutex_t ff_mutex;
static BYTE ff_mutex_init;
#endif
int ff_cre_syncobj (	/* 1:Function succeeded, 0:Could not create the sync object */
	BYTE vol,			/* Corresponding volume (logical drive number) */
	_SYNC_t *sobj		/* Pointer to return the created sync object */
)
{
#ifdef FF_USE_STATIC_MUTEX
	if (!ff_mutex_init) {
		ff_mutex_init = 1;
		if (OS_MutexCreate(&ff_mutex) != OS_OK) {
			return 0;
		}
	}
	return 1;
#else
	int ret = 1;


//	*sobj = CreateMutex(NULL, FALSE, NULL);		/* Win32 */
//	ret = (int)(*sobj != INVALID_HANDLE_VALUE);

//	*sobj = SyncObjects[vol];			/* uITRON (give a static sync object) */
//	ret = 1;							/* The initial value of the semaphore must be 1. */

//	*sobj = OSMutexCreate(0, &err);		/* uC/OS-II */
//	ret = (int)(err == OS_NO_ERR);

//	*sobj = xSemaphoreCreateMutex();	/* FreeRTOS */
//	ret = (int)(*sobj != NULL);

	*sobj = malloc(sizeof(**sobj));
	memset(*sobj, 0, sizeof(**sobj));

	if (OS_MutexCreate(*sobj) != OS_OK)
		ret = 0;

	return ret;
#endif
}



/*------------------------------------------------------------------------*/
/* Delete a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount() function to delete a synchronization
/  object that created with ff_cre_syncobj() function. When a 0 is returned,
/  the f_mount() function fails with FR_INT_ERR.
*/

int ff_del_syncobj (	/* 1:Function succeeded, 0:Could not delete due to any error */
	_SYNC_t sobj		/* Sync object tied to the logical drive to be deleted */
)
{
#ifdef FF_USE_STATIC_MUTEX
	return 1;
#else
	int ret = 1;


//	ret = CloseHandle(sobj);	/* Win32 */

//	ret = 1;					/* uITRON (nothing to do) */

//	OSMutexDel(sobj, OS_DEL_ALWAYS, &err);	/* uC/OS-II */
//	ret = (int)(err == OS_NO_ERR);

//  vSemaphoreDelete(sobj);		/* FreeRTOS */
//	ret = 1;

	if (OS_MutexDelete(sobj) != OS_OK)
		ret = 0;
	else
		free(sobj);

	return ret;
#endif
}



/*------------------------------------------------------------------------*/
/* Request Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on entering file functions to lock the volume.
/  When a 0 is returned, the file function fails with FR_TIMEOUT.
*/

int ff_req_grant (	/* 1:Got a grant to access the volume, 0:Could not get a grant */
	_SYNC_t sobj	/* Sync object to wait */
)
{
	int ret = 1;
	OS_Status status;

//	ret = (int)(WaitForSingleObject(sobj, _FS_TIMEOUT) == WAIT_OBJECT_0);	/* Win32 */

//	ret = (int)(wai_sem(sobj) == E_OK);			/* uITRON */

//	OSMutexPend(sobj, _FS_TIMEOUT, &err));		/* uC/OS-II */
//	ret = (int)(err == OS_NO_ERR);

//	ret = (int)(xSemaphoreTake(sobj, _FS_TIMEOUT) == pdTRUE);	/* FreeRTOS */

#ifdef FF_USE_STATIC_MUTEX
	status = OS_MutexLock(&ff_mutex, _FS_TIMEOUT);
#else
	status = OS_MutexLock(sobj, _FS_TIMEOUT);
#endif
	if (status != OS_OK) {
		ret = 0;
	}

	return ret;
}



/*------------------------------------------------------------------------*/
/* Release Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on leaving file functions to unlock the volume.
*/

void ff_rel_grant (
	_SYNC_t sobj	/* Sync object to be signaled */
)
{
//	ReleaseMutex(sobj);		/* Win32 */

//	sig_sem(sobj);			/* uITRON */

//	OSMutexPost(sobj);		/* uC/OS-II */

//	xSemaphoreGive(sobj);	/* FreeRTOS */

#ifdef FF_USE_STATIC_MUTEX
	OS_MutexUnlock(&ff_mutex);
#else
	OS_MutexUnlock(sobj);
#endif
}

#endif




#if _USE_LFN == 3	/* LFN with a working buffer on the heap */
/*------------------------------------------------------------------------*/
/* Allocate a memory block                                                */
/*------------------------------------------------------------------------*/
/* If a NULL is returned, the file function fails with FR_NOT_ENOUGH_CORE.
*/

void* ff_memalloc (	/* Returns pointer to the allocated memory block */
	UINT msize		/* Number of bytes to allocate */
)
{
	return malloc(msize);	/* Allocate a new memory block with POSIX API */
}


/*------------------------------------------------------------------------*/
/* Free a memory block                                                    */
/*------------------------------------------------------------------------*/

void ff_memfree (
	void* mblock	/* Pointer to the memory block to free */
)
{
	free(mblock);	/* Discard the memory block with POSIX API */
}

#endif

/* RTC function */
#if !_FS_READONLY && !_FS_NORTC
#include <time.h>

DWORD get_fattime (void)
{
	time_t sec;
	struct tm t;

	sec = time(NULL);
	localtime_r(&sec, &t);

	return (DWORD)(((t.tm_year - 80)        << 25) | /* Year origin from the 1980 (0..127) */
	               (((t.tm_mon + 1) &  0xf) << 21) | /* Month (1..12) */
	               ((t.tm_mday      & 0x1f) << 16) | /* Day of the month (1..31) */
	               ((t.tm_hour      & 0x1f) << 11) | /* Hour (0..23) */
	               ((t.tm_min       & 0x3f) <<  5) | /* Minute (0..59) */
	               ((t.tm_sec >> 1) & 0x1f));        /* Second / 2 (0..29) */
}
#endif
