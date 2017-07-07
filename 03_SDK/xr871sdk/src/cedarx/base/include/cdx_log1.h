/*
 * Copyright (c) 2008-2016 Allwinner Technology Co. Ltd.
 * All rights reserved.
 *
 * File : cdx_log.h
 * Description : Log
 * History :
 *
 */

#ifndef CDX_LOG1_H
#define CDX_LOG1_H

#ifndef LOG_TAG
#define LOG_TAG "awplayer"
#endif

#ifndef CDX_DEBUG1
#define CDX_DEBUG1 0
#endif

#include "cdx_malloc_dbg.h"

/* temp define to develop */
/*
#define VIDEO_SUPPORT 0
#define SUBTITLE_SUPPORT 0
#define ONLINE_SUPPORT 0
#define FD_SUPPORT 0
*/

/*enum CDX_LOG_LEVEL_TYPE {
    LOG_LEVEL_VERBOSE = 2,
    LOG_LEVEL_DEBUG = 3,
    LOG_LEVEL_INFO = 4,
    LOG_LEVEL_WARNING = 5,
    LOG_LEVEL_ERROR = 6,
};*/

extern enum CDX_LOG_LEVEL_TYPE GLOBAL_LOG_LEVEL1;

#ifndef CONFIG_LOG_LEVEL
#define CONFIG_LOG_LEVEL    (0xFFFF)
#endif


#include <stdarg.h>
int printf_lock_init();
int wrap_printf(const char *fmt, ...);
int printf_lock_deinit();

int log_file_reset(const char *path);
int log_file(const char *path, unsigned char *buf, unsigned int len);

#define printf wrap_printf

#if CDX_DEBUG1
//#if 0
#define PRINTF(fmt, arg...) printf(fmt, ##arg)
#else
#define PRINTF(fmt, arg...)
#endif


#ifdef __ANDROID__
#include <cutils/log.h>

#define CDX_LOG_ORDER \
    ((unsigned)LOG_LEVEL_ERROR   ==  (unsigned)ANDROID_LOG_ERROR) && \
    ((unsigned)LOG_LEVEL_WARNING ==  (unsigned)ANDROID_LOG_WARN) && \
    ((unsigned)LOG_LEVEL_INFO    ==  (unsigned)ANDROID_LOG_INFO) && \
    ((unsigned)LOG_LEVEL_DEBUG   ==  (unsigned)ANDROID_LOG_DEBUG) && \
    ((unsigned)LOG_LEVEL_VERBOSE ==  (unsigned)ANDROID_LOG_VERBOSE)

typedef char CHECK_LOG_LEVEL_EQUAL_TO_ANDROID[CDX_LOG_ORDER > 0 ? 1 : -1];

#define AWLOG(level, fmt, arg...)  \
    do { \
        if (level >= GLOBAL_LOG_LEVEL || level >= CONFIG_LOG_LEVEL) \
            LOG_PRI(level, LOG_TAG, "<%s:%u>: " fmt, __FUNCTION__, __LINE__, ##arg); \
    } while (0)

#define CDX_TRACE() \
    CDX_LOGI("<%s:%u> tid(%d)", __FUNCTION__, __LINE__, gettid())

/*check when realease version*/
#define CDX_FORCE_CHECK1(e) \
        LOG_ALWAYS_FATAL_IF1(                        \
                !(e),                               \
                "<%s:%d>CDX_CHECK(%s) failed.",     \
                __FUNCTION__, __LINE__, #e)

#define CDX_TRESPASS1() \
        LOG_ALWAYS_FATAL1("Should not be here.")

#define CDX_LOG_FATAL1(fmt, arg...)                          \
        LOG_ALWAYS_FATAL1("<%s:%d>" fmt,                      \
            __FUNCTION__, __LINE__, ##arg)

#define CDX_LOG_CHECK1(e, fmt, arg...)                           \
    LOG_ALWAYS_FATAL_IF1(                                        \
            !(e),                                               \
            "<%s:%d>check (%s) failed:" fmt,                    \
            __FUNCTION__, __LINE__, #e, ##arg)

#ifdef AWP_DEBUG
#define CDX_CHECK1(e)                                            \
    LOG_ALWAYS_FATAL_IF1(                                        \
            !(e),                                               \
            "<%s:%d>CDX_CHECK(%s) failed.",                     \
            __FUNCTION__, __LINE__, #e)

#else
#define CDX_CHECK1(e)
#endif

#else

#include <stdio.h>
#include <string.h>
#include <assert.h>

extern const char *CDX_LOG_LEVEL_NAME[];

#if CDX_DEBUG1
//#if 0
#define AWLOG1(level, fmt, arg...)  \
    do { \
        if ((level >= GLOBAL_LOG_LEVEL1 || level >= CONFIG_LOG_LEVEL) && \
                level <= LOG_LEVEL_ERROR) \
            printf("%s: %s <%s:%u>: " fmt "\n", \
                    CDX_LOG_LEVEL_NAME[level], LOG_TAG, __FUNCTION__, __LINE__, ##arg); \
    } while (0)
#else
#define AWLOG1(level, fmt, arg...)
#endif

#define CDX_TRESPASS1()

#define CDX_FORCE_CHECK1(e) CDX_CHECK1(e)

#define CDX_LOG_CHECK1(e, fmt, arg...)                           \
    do {                                                        \
        if (!(e))                                               \
        {                                                       \
            CDX_LOGE1("check (%s) failed:"fmt, #e, ##arg);       \
        }                                                       \
    } while (0)

#ifdef AWP_DEBUG
#define CDX_CHECK1(e)                                            \
    do {                                                        \
        if (!(e))                                               \
        {                                                       \
            CDX_LOGE1("check (%s) failed.", #e);                 \
            assert(0);                                          \
        }                                                       \
    } while (0)
#else
#define CDX_CHECK1(e) (void)(e)
#endif

#endif

#define CDX_LOGV1(fmt, arg...) logv1(fmt, ##arg)
#define CDX_LOGD1(fmt, arg...) logd1(fmt, ##arg)
#define CDX_LOGI1(fmt, arg...) logi1(fmt, ##arg)
#define CDX_LOGW1(fmt, arg...) logw1(fmt, ##arg)
#define CDX_LOGE1(fmt, arg...) loge1(fmt, ##arg)

#if CDX_DEBUG1
#define CDX_BUF_DUMP1(buf, len) \
    do { \
        char *_buf = (char *)buf;\
        char str[1024] = {0};\
        unsigned int index = 0, _len;\
        _len = (unsigned int)len;\
        snprintf(str, 1024, ":%d:[", _len);\
        for (index = 0; index < _len; index++)\
        {\
            snprintf(str + strlen(str), 1024 - strlen(str), "%02hhx ", _buf[index]);\
        }\
        str[strlen(str) - 1] = ']';\
        CDX_LOGD1("%s", str);\
    }while (0)
#else
#define CDX_BUF_DUMP1(buf, len)
#endif

#define CDX_ITF_CHECK1(base, itf)    \
    CDX_CHECK1(base);                \
    CDX_CHECK1(base->ops);           \
    CDX_CHECK1(base->ops->itf)

#define CDX_UNUSE1(param) (void)param
#define CEDARX_UNUSE1(param) (void)param

#define logd1(fmt, arg...) //AWLOG1(LOG_LEVEL_DEBUG, fmt, ##arg)
#define loge1(fmt, arg...) AWLOG1(LOG_LEVEL_ERROR, "\033[40;31m" fmt "\033[0m", ##arg)
#define logw1(fmt, arg...) //AWLOG1(LOG_LEVEL_WARNING, fmt, ##arg)
#define logi1(fmt, arg...) //AWLOG1(LOG_LEVEL_INFO, fmt, ##arg)
#define logv1(fmt, arg...) //AWLOG1(LOG_LEVEL_VERBOSE, fmt, ##arg)


#define CDX_NOTDEAD() { \
	while (1) { \
		logd1("not dead\n"); \
		sleep(3); \
	} \
}

#define CDX_ENTRY1() //PRINTF("[%s entry] line %d\n", __func__, __LINE__)
#define CDX_EXIT1(ret) //PRINTF("[%s exit] line %d, return %d\n", __func__, __LINE__, (int)ret)

#define CDX_FAILED() //PRINTF("[%s failed] line %d\n", __func__, __LINE__)


#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif

#endif
