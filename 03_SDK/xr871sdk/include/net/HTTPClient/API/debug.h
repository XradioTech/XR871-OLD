#ifndef __HTTPC_H_H
#define __HTTPC_H_H

#include "HTTPClientWrapper.h"

//#define HTTPCLIENT_DEBUG

#ifdef HTTPCLIENT_DEBUG
#define	HC_DBG(x)	do {printf("[HTTPC]"); printf x ; putchar('\n'); fflush(stdout); } while (0)
#else
#define	HC_DBG(x) NULL
#endif /* DEBUG */

#define	HC_ERR(x)	do {printf("[HTTPC][ERR]"); printf x ; putchar('\n'); fflush(stdout); } while (0)
#endif