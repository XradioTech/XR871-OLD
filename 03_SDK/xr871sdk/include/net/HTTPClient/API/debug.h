#ifndef __HTTPC_H_H
#define __HTTPC_H_H

#include "HTTPClientWrapper.h"

//#define _HTTP_DEBUGGING_
//#define HTTPCLIENT_DEBUG

#define HC_LOGD(fmt, arg...) printf("[HTTPC]"fmt"\n", ##arg)
#define HC_LOGE(fmt, arg...) printf("[HTTPC][ERR]"fmt"\n", ##arg)

#ifdef HTTPCLIENT_DEBUG
#define	HC_DBG(x)	do { HC_LOGD x; } while (0)
#else
#define	HC_DBG(x)	do { } while (0)
#endif /* HTTPCLIENT_DEBUG */

#define	HC_ERR(x)	do { HC_LOGE x; } while (0)

#endif /* __HTTPC_H_H */
