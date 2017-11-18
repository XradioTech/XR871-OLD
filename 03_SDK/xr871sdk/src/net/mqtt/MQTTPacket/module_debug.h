#ifndef __MODULE_DEBUG_H__
#define __MODULE_DEBUG_H__

#include "sys/xr_util.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define MDEBUG_ON


/*
 * @brief Debug level
 */
#define DBG_LEVEL_MASK (0x0F)


#define MLEVEL_EMERG 0

#define MLEVEL_ALERT 1

#define MLEVEL_CRIT 2

#define MLEVEL_ERROR 3

#define MLEVEL_WARNING 4

#define MLEVEL_NOTICE 5

#define MLEVEL_INFO 6

#define MLEVEL_DEBUG 7

#define MLEVEL_ALL 0x00


/*
 * No expanded condition
 */
#define NOEXPAND 1


/*
 * module ON/OFF
 */
#define DBG_ON (1 << 4)

#define DBG_OFF (0)


/*
 * Always show message
 */
#define MOD_DBG_ALW_ON (DBG_ON | MLEVEL_ALL)


/************************************************************
 * MDEBUG INTERFACE
 ************************************************************/
#ifdef MDEBUG_ON

#define MDEBUG_PRINT(msg, arg...) printf(msg, ##arg)

#define MDEBUG_ABORT()	do { \
							printf("system aborted!"); \
							sys_abort(); \
						} while (0)


/*
 * @brief	The realization of showing debug messages.
 * @param	module: Contained a module On/Off and a module debug level.
 * @param	dlevel: Debug message showed level for seal like MDEBUG.
 * @param	expand: Expanded condition if level param and module ON/OFF are not
 * 			        enough for developer.
 * @param	msg: The debug message.
 * @param	arg: Arguement shown in debug message which like printf arguement.
 * @retval	None
 */
#define _MDEBUG(module, dlevel, expand, msg, arg...)	\
		do { \
			if ( \
				((module) & DBG_ON) && \
				(((module) & DBG_LEVEL_MASK) >= dlevel) && \
				(expand)) { \
				MDEBUG_PRINT(msg, ##arg); \
			} \
		} while(0)

/*
 * @brief	The realization of showing debug messages and it can't be turn off by
 * 			module ON/OFF.
 * @param	module: Contained a module On/Off and a module debug level.
 * @param	dlevel: Debug message showed level for seal.
 * @param	expand: Expanded condition if level param is not enough for developer.
 * @param	msg: The debug message.
 * @param	arg: Arguement shown in debug message which like printf arguement.
 * @retval	None
 */
#define _MINFO(module, dlevel, expand, msg, arg...)	\
		do { \
			if ( \
				(((int16_t)(module) & DBG_LEVEL_MASK) >= dlevel) && \
				(expand)) { \
				MDEBUG_PRINT(msg, ##arg); \
			} \
		} while(0)

/*
 * @brief	The realization of assert debug messages shown the assert position
 * @param	module: Contained a module On/Off and a module debug level.
 * @param	dlevel: Debug message showed level for seal.
 * @param	msg: The debug message.
 * @param	arg: Arguement shown in debug message which like printf arguement.
 * @retval	None
 */
#define _MASSERT(assert, module, dlevel, msg, arg...)	\
		_MDEBUG(module, dlevel, assert, \
				"[Assert] At %s line %d fun %s: " msg, \
				__FILE__, __LINE__, __func__, ##arg)

/*
 * @brief	The realization of assert debug messages shown the assert position,
 * 			and abort.
 * @param	module: Contained a module On/Off and a module debug level.
 * @param	dlevel: Debug message showed level for seal.
 * @param	msg: The debug message.
 * @param	arg: Arguement shown in debug message which like printf arguement.
 * @retval	None
 */
#define _MASSERT_ABORT(assert, module, dlevel, msg, arg...)	\
		do { \
			if (((int16_t)(module) & DBG_LEVEL_MASK) >= dlevel) { \
				MDEBUG_PRINT("[Assert] At %s line %d fun %s: " msg, \
							 __FILE__, __LINE__, __func__, ##arg); \
				MDEBUG_ABORT(); \
			} \
		} while(0)


/*
 * @brief	a level debug message
 * @param	module: Contained a module On/Off and a module debug level.
 * @param	expand: Expanded condition if level param and module ON/OFF are not
 * 			        enough for developer.
 * @param	msg: The debug message.
 * @param	arg: Arguement shown in debug message which like printf arguement.
 * @retval	None
 */
#define MERROR(module, expand, msg, arg...) \
		_MDEBUG(module, MLEVEL_ERROR, expand, msg, ##arg)

#define MALERT(module, expand, msg, arg...) \
		_MDEBUG(module, MLEVEL_ALERT, expand, msg, ##arg)

#define MCRIT(module, expand, msg, arg...) \
		_MDEBUG(module, MLEVEL_CRIT, expand, msg, ##arg)

#define MEMERG(module, expand, msg, arg...) \
		_MDEBUG(module, MLEVEL_EMERG, expand, msg, ##arg)

#define MWARN(module, expand, msg, arg...) \
		_MDEBUG(module, MLEVEL_WARNING, expand, msg, ##arg)

#define MNOTICE(module, expand, msg, arg...) \
		_MDEBUG(module, MLEVEL_NOTICE, expand, msg, ##arg)

#define MINFO(module, expand, msg, arg...) \
		_MDEBUG(module, MLEVEL_INFO, expand, msg, ##arg)

#define MDEBUG(module, expand, msg, arg...) \
		_MDEBUG(module, MLEVEL_DEBUG, expand, msg, ##arg)


/*
 * @brief	assert a full debug message with position(file, line, etc.) without level.
 * @param	assert: debug condition
 * @param	module: Contained a module On/Off at least.
 * @param	msg: The debug message.
 * @param	arg: Arguement shown in debug message which like printf arguement.
 * @retval	None
 */
#define MASSERT(assert, module, msg, arg...) \
		_MASSERT(assert, module, MLEVEL_ALL, msg, ##arg)

#define MASSERT_ABORT(assert, module, msg, arg...) \
		_MASSERT_ABORT(assert, module, MLEVEL_ALL, msg, ##arg)

#ifndef assert
#define assert(condition) MASSERT(condition, MOD_DBG_ALW_ON, "condition %s is fault. errno is %d.\n", #condition, xr_thread_errno);
#endif

/*
// THIS REALIZATION DO NOT SEAL
#define MASSERT(assert, module, msg, arg...) \
		_MASSERT(assert, module, MLEVEL_ALL, "[%s]" msg, #module, ##arg)

#define MASSERT_ABORT(assert, module, msg, arg...) \
		_MASSERT_ABORT(assert, module, MLEVEL_ALL, "[%s]" msg, #module, ##arg)
*/


/*
 * @brief	notify the function entry and exit/return in the debug level
 * @param	module: Contained a module On/Off at least.
 * @param	mname: module name in string
 * @param	ret: return value
 * @retval	None
 */
#define MENTRY(module, mname) \
		MDEBUG(module, NOEXPAND, mname "entry %s().\n", __func__)

#define MRET(module, mname, ret) \
		MDEBUG(module, NOEXPAND, mname "exit %s() with return %d.\n", __func__, ret)

#define MRET_NOVAL(module, mname) \
		MDEBUG(module, NOEXPAND, mname "exit %s().\n", __func__)

#else /* MDEBUG_ON */

#define MDEBUG_PRINT(msg, arg...)

#define MDEBUG_ABORT()


#define _MDEBUG(module, dlevel, expand, msg, arg...)

#define _MINFO(module, dlevel, expand, msg, arg...)

#define _MASSERT(assert, module, dlevel, msg, arg...)

#define _MASSERT_ABORT(assert, module, dlevel, msg, arg...)


#define MERROR(module, expand, msg, arg...)

#define MALERT(module, expand, msg, arg...)

#define MCRIT(module, expand, msg, arg...)

#define MEMERG(module, expand, msg, arg...)

#define MWARN(module, expand, msg, arg...)

#define MNOTICE(module, expand, msg, arg...)

#define MINFO(module, expand, msg, arg...)

#define MDEBUG(module, expand, msg, arg...)


#define MASSERT(assert, module, msg, arg...)


#define MENTRY(module, mname)

#define MRET(module, mname, ret)

#define MRET_NOVAL(module, mname)

#endif /* MDEBUG_ON */

#endif /* __LWIP_DEBUG_H__ */
