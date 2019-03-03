
#ifndef HTTP_CLIENT_WRAPPER
#define HTTP_CLIENT_WRAPPER


///////////////////////////////////////////////////////////////////////////////
//
// Section      : Microsoft Windows Support
// Last updated : 01/09/2005
//
///////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32

#pragma warning (disable: 4996) // 'function': was declared deprecated (VS 2005)
#include	<stdlib.h>
#include	<string.h>
#include	<memory.h>
#include	<stdio.h>
#include	<ctype.h>
#include	<time.h>
#include	<winsock.h>

// Sockets (Winsock wrapper)
#define		HTTP_ECONNRESET     (WSAECONNRESET)
#define		HTTP_EINPROGRESS    (WSAEINPROGRESS)
#define		HTTP_EWOULDBLOCK    (WSAEWOULDBLOCK)

// Kluge alert: redefining strncasecmp() as memicmp() for Windows.
//
#define		strncasecmp			memicmp
#define		strcasecmp			stricmp

#elif defined _LINUX  // Non Win32 : GCC Linux

#include	<unistd.h>
#include	<errno.h>
#include	<pthread.h>
#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<ctype.h>
#include	<time.h>
#include	<sys/socket.h>
#include	<sys/un.h>
#include	<netinet/in.h>
#include	<netinet/tcp.h>
#include	<netdb.h>
#include	<arpa/inet.h>
#include	<sys/ioctl.h>
#include	<errno.h>
#include	<stdarg.h>

#define		SOCKET_ERROR			-1

#define		HTTP_EINPROGRESS    (EINPROGRESS)
#define		HTTP_EWOULDBLOCK    (EWOULDBLOCK)

typedef unsigned long                UINT32;
typedef long                         INT32;
#else

#include	<stdlib.h>
#include	<stdio.h>
#include	<string.h>
#include	<ctype.h>
#include	<time.h>
#include	"lwip/sockets.h"
#include	"lwip/netdb.h"
#include	<errno.h>

#include	<stdarg.h>
#include	"debug.h"

#define		SOCKET_ERROR			-1

#define		HTTP_EINPROGRESS    (EINPROGRESS)
#define		HTTP_EWOULDBLOCK    (EWOULDBLOCK)

#define		HTTPC_SSL

// Generic types
typedef unsigned long                UINT32;
typedef long                         INT32;


#ifndef BOOL
#define BOOL  int
#endif

#define HTTPC_ERRNO errno

#define HTTPC_AUTH_CLOSE_CONNEC
#define HTTPC_LWIP
#define HTTPC_LITTLE_STACK
#define HTTPC_SEND_TOGTHER  // send http request header together, this will malloc an additional buffer
#define HTTP_GET_REDIRECT_URL
#define HTTP_GET_HANDLE_FLAGS

#endif

// Note: define this to prevent timeouts while debugging.
// #define							 NO_TIMEOUTS

///////////////////////////////////////////////////////////////////////////////
//
// Section      : Functions that are not supported by the AMT stdc framework
//                So they had to be specificaly added.
// Last updated : 01/09/2005
//
///////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
extern "C" {
#endif

// STDC Wrapper implimentation
int                                 HTTPWrapperIsAscii              (int c);
int                                 HTTPWrapperToUpper              (int c);
int                                 HTTPWrapperToLower              (int c);
int                                 HTTPWrapperIsAlpha              (int c);
int                                 HTTPWrapperIsAlNum              (int c);
char*                               HTTPWrapperItoa                 (char *buff,int i);
void                                HTTPWrapperInitRandomeNumber    ();
long                                HTTPWrapperGetUpTime            ();
int                                 HTTPWrapperGetRandomeNumber     ();
int                                 HTTPWrapperGetSocketError       (int s);
unsigned long                       HTTPWrapperGetHostByName        (char *name,unsigned long *address);
int                                 HTTPWrapperShutDown             (int s,int in);
// SSL Wrapper prototypes
int                                 HTTPWrapperSSLConnect           (int s,const struct sockaddr *name,int namelen,char *hostname);
int                                 HTTPWrapperSSLNegotiate         (int s,const struct sockaddr *name,int namelen,char *hostname);
int                                 HTTPWrapperSSLSend              (int s,char *buf, int len,int flags);
int                                 HTTPWrapperSSLRecv              (int s,char *buf, int len,int flags);
int                                 HTTPWrapperSSLClose             (int s);
int                                 HTTPWrapperSSLRecvPending       (int s);

typedef void* (*HTTPC_USR_CERTS)(void);
void*                               HTTPC_obtain_user_certs();
unsigned char                       HTTPC_get_ssl_verify_mode();

        // Global wrapper Functions
#define                             IToA                            HTTPWrapperItoa
#define                             GetUpTime                       HTTPWrapperGetUpTime
#define                             SocketGetErr                    HTTPWrapperGetSocketError
#define                             HostByName                      HTTPWrapperGetHostByName
#define                             InitRandomeNumber               HTTPWrapperInitRandomeNumber
#define                             GetRandomeNumber                HTTPWrapperGetRandomeNumber

#ifdef __cplusplus
}
#endif

///////////////////////////////////////////////////////////////////////////////
//
// Section      : Global type definitions
// Last updated : 01/09/2005
//
///////////////////////////////////////////////////////////////////////////////

#define VOID                         void
#ifndef NULL
#define NULL                         0
#endif
#define TRUE                         1
#define FALSE                        0
typedef char                         CHAR;
typedef unsigned short               UINT16;
//typedef int                          BOOL;
//typedef unsigned long                ULONG;

// Global socket structures and definitions
#define                              HTTP_INVALID_SOCKET (-1)
typedef struct sockaddr_in           HTTP_SOCKADDR_IN;
typedef struct timeval               HTTP_TIMEVAL;
typedef struct hostent               HTTP_HOSTNET;
typedef struct sockaddr              HTTP_SOCKADDR;
typedef struct in_addr               HTTP_INADDR;


#endif // HTTP_CLIENT_WRAPPER
