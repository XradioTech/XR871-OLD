
#ifndef HTTPC_USR_H_H
#define HTTPC_USR_H_H

#define HTTP_CLIENT_BUFFER_SIZE     4096

#include <stdio.h>
#include "API/HTTPClient.h"
#include "API/debug.h"

typedef struct _HTTPParameters
{
        CHAR                    Uri[256]; /*in*/
	HTTP_VERB		HttpVerb; /*in*/
        UINT32                  Verbose; /*in*/
        CHAR                    UserName[16]; /*in*/
        CHAR                    Password[16]; /*in*/
        HTTP_AUTH_SCHEMA        AuthType; /*in*/
        BOOL			isTransfer; /*out*/
        HTTP_SESSION_HANDLE     pHTTP; /*out*/
	UINT32              	Flags; /*in*/
	VOID			*pData; /*in*/
	UINT32              	pLength; /*in*/
} HTTPParameters;

int HTTPC_open(HTTPParameters *ClientParams);
int HTTPC_request(HTTPParameters *ClientParams,INT32 *conLength);
int HTTPC_write(HTTPParameters *ClientParams,VOID *pBuffer, UINT32 toWrite);
int HTTPC_read(HTTPParameters *ClientParams,VOID *pBuffer, UINT32 toRead, UINT32 *recived);
int HTTPC_close( HTTPParameters *ClientParams);


int HTTPC_get(HTTPParameters *ClientParams,CHAR *Buffer, INT32 bufSize, INT32 *recvSize );


#endif // HTTP_CLIENT_SAMPLE


