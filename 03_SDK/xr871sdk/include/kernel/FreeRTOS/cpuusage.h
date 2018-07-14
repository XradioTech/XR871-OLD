#ifndef __CPUUSAGE_H__
#define __CPUUSAGE_H__

#include "FreeRTOS.h"

#if (configDEBUG_CPU_USAGE_EN == 1)

/*
 * print_s: 0: not print, other: print cpu usage every print_s seconds.
 * Note: use IDLE_HOOK mode when cpuusage is more than 50% to get a precise result.
 *       called after idle thread run and no other thread are runing if used IDLE_HOOK!
 */
extern void OSCpuUsageInit(uint32_t print_s);
extern uint32_t OSGetCpuUsage(void);

#else

static inline void OSCpuUsageInit(uint32_t print_s) { ; }
static inline uint32_t OSGetCpuUsage(void) { return 0; }

#endif
#endif /* __CPUUSAGE_H__ */
