/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HTTPC_USR_H_H
#define HTTPC_USR_H_H

#define HTTP_CLIENT_BUFFER_SIZE     4096

#include <stdio.h>
#include "API/HTTPClient.h"
#include "API/debug.h"
#include "API/HTTPClientCommon.h"

typedef struct _HTTPParameters
{
	CHAR Uri[HTTP_CLIENT_MAX_URL_LENGTH]; /*in, this uri will be overwritten if there is a redirect */
	HTTP_VERB HttpVerb; /*in*/
	UINT32 Verbose; /*in*/
	CHAR UserName[HTTP_CLIENT_MAX_USERNAME_LENGTH]; /*in*/
	CHAR Password[HTTP_CLIENT_MAX_PASSWORD_LENGTH]; /*in*/
	HTTP_AUTH_SCHEMA AuthType; /*in*/
	BOOL isTransfer; /*out*/
	HTTP_SESSION_HANDLE pHTTP; /*out*/
	UINT32 Flags; /*in*/
	VOID *pData; /*in*/
	UINT32 pLength; /*in*/
} HTTPParameters;

int HTTPC_open(HTTPParameters *ClientParams);
int HTTPC_request(HTTPParameters *ClientParams, HTTP_CLIENT_GET_HEADER Callback);
int HTTPC_get_request_info(HTTPParameters *ClientParams, void *HttpClient);
int HTTPC_write(HTTPParameters *ClientParams, VOID *pBuffer, UINT32 toWrite);
int HTTPC_read(HTTPParameters *ClientParams, VOID *pBuffer, UINT32 toRead, UINT32 *recived);
int HTTPC_close(HTTPParameters *ClientParams);
int HTTPC_reset_session(HTTPParameters *ClientParams);
int HTTPC_get(HTTPParameters *ClientParams,CHAR *Buffer, INT32 bufSize, INT32 *recvSize);
void HTTPC_Register_user_certs(HTTPC_USR_CERTS certs);

#endif // HTTPC_USR_H_H
