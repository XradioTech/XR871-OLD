/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/


#include <stdint.h>

extern uint32_t SystemCoreClock;	/* Global variable of CMSIS */

/* Clock related definitions. */
#define configCPU_CLOCK_HZ                      ( SystemCoreClock )
//#define configSYSTICK_CLOCK_HZ
#define configTICK_RATE_HZ                      ( 1000 )
#define configUSE_16_BIT_TICKS                  0

/* Task related definitions. */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 1
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                128 /* 512-byte */
#define configMAX_TASK_NAME_LEN                 16
#define configIDLE_SHOULD_YIELD                 0 // ?
#define configUSE_TIME_SLICING                  1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 1

#define configUSE_TASK_NOTIFICATIONS            1 // ?

/* Mutex related definitions. */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1

/* Semaphore related definitions. */
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_ALTERNATIVE_API               0 /* Deprecated! */

/* Queue related definitions. */
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_QUEUE_SETS                    1

/* Misc */
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     1

#ifndef __CONFIG_MALLOC_USE_STDLIB
/* Memory allocation related definitions. */
//#define configSUPPORT_STATIC_ALLOCATION         1
//#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configLINKER_ALLOCATED_HEAP				1 /* heap size defined by linker */

#if ( configLINKER_ALLOCATED_HEAP == 1)
	#define configMSP_STACK_SIZE				1024
//	#define configTOTAL_HEAP_SIZE				0 /* calc automatically */
	#define configAPPLICATION_ALLOCATED_HEAP	0 /* useless */
#else
	#define configTOTAL_HEAP_SIZE				( 64 * 1024 )
	#define configAPPLICATION_ALLOCATED_HEAP	0
#endif
#endif /* __CONFIG_MALLOC_USE_STDLIB */

/* Hook function related definitions. */
#define configUSE_IDLE_HOOK                     0 // ?
#define configUSE_TICK_HOOK                     0 // ?
#define configCHECK_FOR_STACK_OVERFLOW          2 // ?
#define configUSE_MALLOC_FAILED_HOOK            0 // ?
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0 // ?

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1 // ?
#define configUSE_STATS_FORMATTING_FUNCTIONS    0 // ?

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         2

/* Software timer related definitions. */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 ) // highest priority
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            512 /* 2048-byte */
#define configUSE_TIMER_ID_AS_CALLBACK_ARG      1 /* use timer ID as timer callback function argument */

#define configDEBUG_CPU_USAGE_EN                0 /* use cpu usage statistic */

#define configDEBUG_TRACE_TASK_MOREINFO         1 /* add some info to trace tasks, use configUSE_STATS_FORMATTING_FUNCTIONS */

/* Interrupt nesting behaviour configuration. */
#define configKERNEL_INTERRUPT_PRIORITY 		( 7 << 5 )	/* Priority 7 as only the top three bits are implemented.  This is the lowest priority. */
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( 1 << 5 )  /* Priority 1 as only the top three bits are implemented. */

/* Define to trap errors during development. */
//#define configASSERT( ( x ) ) if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )

/* FreeRTOS MPU specific definitions. */
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS 0

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
//#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          1
//#define INCLUDE_xTaskAbortDelay                 0
//#define INCLUDE_xTaskGetHandle                  0
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xTimerGetTimerDaemonTaskHandle  0
#define INCLUDE_pcTaskGetTaskName               1
#define INCLUDE_xSemaphoreGetMutexHolder        1

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
   standard names. */
#if 1
#define vPortSVCHandler		SVC_Handler
#define xPortPendSVHandler	PendSV_Handler
#define xPortSysTickHandler	SysTick_Handler
#endif

/* A header file that defines trace macro can be included here. */

////////////////////////////////////////////////////////////////////////////////

/* disable some features for bootloader to reduce code size */
#ifdef __CONFIG_BOOTLOADER

#undef  configUSE_TICKLESS_IDLE
#define configUSE_TICKLESS_IDLE                 0

#undef  configUSE_TASK_NOTIFICATIONS
#define configUSE_TASK_NOTIFICATIONS            0

#undef  configUSE_QUEUE_SETS
#define configUSE_QUEUE_SETS                    0

//#undef  configCHECK_FOR_STACK_OVERFLOW
//#define configCHECK_FOR_STACK_OVERFLOW          0

#undef  configUSE_TRACE_FACILITY
#define configUSE_TRACE_FACILITY                0

#undef  configUSE_STATS_FORMATTING_FUNCTIONS
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

#undef  configUSE_TIMERS
#define configUSE_TIMERS                        0

#undef  INCLUDE_xTimerPendFunctionCall
#define INCLUDE_xTimerPendFunctionCall          0

#undef  INCLUDE_xEventGroupSetBitFromISR
#define INCLUDE_xEventGroupSetBitFromISR        0

#undef  configTIMER_TASK_STACK_DEPTH
#define configTIMER_TASK_STACK_DEPTH            256 /* 1024-byte */

#undef  configDEBUG_TRACE_TASK_MOREINFO
#define configDEBUG_TRACE_TASK_MOREINFO         0

#undef  INCLUDE_vTaskPrioritySet
#define INCLUDE_vTaskPrioritySet                0

#undef  INCLUDE_uxTaskPriorityGet
#define INCLUDE_uxTaskPriorityGet               0

#undef  INCLUDE_vTaskSuspend
#define INCLUDE_vTaskSuspend                    0

#undef  INCLUDE_vTaskDelayUntil
#define INCLUDE_vTaskDelayUntil                 0

#undef  INCLUDE_uxTaskGetStackHighWaterMark
#define INCLUDE_uxTaskGetStackHighWaterMark     0

#undef  INCLUDE_xTaskGetIdleTaskHandle
#define INCLUDE_xTaskGetIdleTaskHandle          0

#undef  INCLUDE_eTaskGetState
#define INCLUDE_eTaskGetState                   0

#undef  INCLUDE_xTaskResumeFromISR
#define INCLUDE_xTaskResumeFromISR              0

#undef  INCLUDE_xTimerGetTimerDaemonTaskHandle
#define INCLUDE_xTimerGetTimerDaemonTaskHandle  0

#undef  INCLUDE_pcTaskGetTaskName
#define INCLUDE_pcTaskGetTaskName               0

#undef  INCLUDE_xSemaphoreGetMutexHolder
#define INCLUDE_xSemaphoreGetMutexHolder        0

#endif /* __CONFIG_BOOTLOADER */

#endif /* FREERTOS_CONFIG_H */
