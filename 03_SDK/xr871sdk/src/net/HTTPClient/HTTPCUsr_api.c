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
#include "net/HTTPClient/HTTPCUsr_api.h"

static HTTPC_USR_CERTS httpc_user_certs = NULL;
static unsigned char httpc_ssl_verify_mode = 0;

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPDebug
// Purpose      : HTTP API Debugging callback
// Gets         : arguments
// Returns      : UINT32
// Last updated : 01/09/2005
//
///////////////////////////////////////////////////////////////////////////////

void HTTPDebug(const CHAR* FunctionName,const CHAR *DebugDump,UINT32 iLength,CHAR *DebugDescription,...) // Requested operation
{
    va_list pArgp;
    va_start(pArgp, DebugDescription);
	if (FunctionName)
		printf("%s ", FunctionName);
	if (DebugDump)
		printf("%s ", DebugDump);
	vprintf(DebugDescription, pArgp);
	printf("\n");
    va_end(pArgp);
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_open
// Purpose      : open session.
// Returns      : 0: success other: fail
//
// Last updated : 02/15/2017
//
///////////////////////////////////////////////////////////////////////////////

int HTTPC_open(HTTPParameters *ClientParams)
{

	INT32 nRetCode;
	HTTP_SESSION_HANDLE pHTTP;
	do
	{
		ClientParams->isTransfer = TRUE;
		HC_DBG(("HTTP OPEN."));
		// Open the HTTP request handle
		pHTTP = HTTPClientOpenRequest(ClientParams->Flags);
		// Set the Verb
		if((nRetCode = HTTPClientSetVerb(pHTTP,ClientParams->HttpVerb)) != HTTP_CLIENT_SUCCESS)
		{
			break;
		}
		// Set authentication
		if(ClientParams->AuthType != AuthSchemaNone)
		{
			HC_DBG(("OPEN:auth type:%ld.\n", (INT32)(ClientParams->AuthType)));
			if((nRetCode = HTTPClientSetAuth(pHTTP,ClientParams->AuthType,NULL)) != HTTP_CLIENT_SUCCESS)
			{
				break;
			}
			// Set authentication
			if((nRetCode = HTTPClientSetCredentials(pHTTP,ClientParams->UserName,ClientParams->Password)) != HTTP_CLIENT_SUCCESS)
			{
				break;
			}
		}
		//P_HTTP_SESSION pHTTPSession = (P_HTTP_SESSION)pHTTP;
		ClientParams->pHTTP = pHTTP;
	} while(0);

	if (nRetCode != HTTP_CLIENT_SUCCESS)
		HTTPClientCloseRequest (&pHTTP);

	return nRetCode;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_request
// Purpose      : send request ..
// Returns      : 0: success other: fail
// Last updated : 02/15/2017
//
///////////////////////////////////////////////////////////////////////////////

HTTP_GET_HEADER gHttpcGetHeader;

int HTTPC_request(HTTPParameters *ClientParams, HTTP_CLIENT_GET_HEADER Callback)
{
	HTTP_CLIENT httpClient;
	INT32 nRetCode;

	memset(&gHttpcGetHeader, 0, sizeof(gHttpcGetHeader));
	if (Callback != NULL)
		gHttpcGetHeader.callback = Callback;
sendrequest:
	do
	{
		if (ClientParams->HttpVerb == VerbPost)
		{
			if (ClientParams->pLength > 0 && ClientParams->pData != NULL)
			{
				// Send a request for the home page
				if((nRetCode = HTTPClientSendRequest(ClientParams->pHTTP,ClientParams->Uri,ClientParams->pData,ClientParams->pLength,TRUE,0,0)) != HTTP_CLIENT_SUCCESS)
				{
					HC_ERR(("POST:Send Request failed..\n"));
					break;
				}
			}
			else
			{
				return -1;
			}
		}
		else
		{
			// Send a request for the home page
			if((nRetCode = HTTPClientSendRequest(ClientParams->pHTTP,ClientParams->Uri,NULL,0,FALSE,0,0)) != HTTP_CLIENT_SUCCESS)
			{
				HC_ERR(("Get: Send Request failed..\n"));
				break;
			}
		}
		// Retrieve the the headers and analyze them
		if((nRetCode = HTTPClientRecvResponse(ClientParams->pHTTP,0)) != HTTP_CLIENT_SUCCESS)
		{
			HC_ERR(("Recv Response failed..\n"));
			break;
		}
		// Get the request info for determine whether it is redirection
		if((nRetCode = HTTPC_get_request_info(ClientParams, &httpClient)) != 0)
		{
			HC_ERR(("Get request info err..\n"));
			break;
		}
		//if the HTTPStatusCode is not HTTP_STATUS_OK, it may be a redirect url
		if(httpClient.HTTPStatusCode != HTTP_STATUS_OK)
		{
			//if the HTTPStatusCode is 302/301
			if((httpClient.HTTPStatusCode == HTTP_STATUS_OBJECT_MOVED) ||
				(httpClient.HTTPStatusCode == HTTP_STATUS_OBJECT_MOVED_PERMANENTLY))
			{
				HC_DBG(("Get a redirect url, this request will be closed and create a new request.."));
				//copy the redirect url
				if(httpClient.RedirectUrl->nLength < sizeof(ClientParams->Uri))
				{
					strncpy(ClientParams->Uri, httpClient.RedirectUrl->pParam, httpClient.RedirectUrl->nLength);
					if((nRetCode = HTTPC_close(ClientParams)) != 0)
					{
						HC_ERR(("http close err..\n"));
						break;
					}
					if((nRetCode = HTTPC_open(ClientParams)) != 0)
					{
						HC_ERR(("http open err..\n"));
						break;
					}
				}
				else
				{
					HC_ERR(("The redirect url too long..\n"));
					break;
				}
				HC_DBG(("Send request to redirect url.."));
				goto sendrequest;
			}
		}
	} while(0);

	if (nRetCode != HTTP_CLIENT_SUCCESS) {
		HTTP_SESSION_HANDLE pSession = ClientParams->pHTTP;
		HTTPClientCloseRequest(&pSession);
		ClientParams->pHTTP = 0;
	}
	return nRetCode;
}
///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_get_request_info
// Purpose      : get request result..
// Returns      : 0: success other: fail
// Last updated : 02/15/2017
//
///////////////////////////////////////////////////////////////////////////////

int HTTPC_get_request_info(HTTPParameters *ClientParams, void *HttpClient)
{
	INT32 nRetCode = HTTP_CLIENT_SUCCESS;
	HTTP_SESSION_HANDLE pSession = ClientParams->pHTTP;
	if ((nRetCode = HTTPClientGetInfo(pSession, HttpClient)) != HTTP_CLIENT_SUCCESS)
		HC_ERR(("get info failed."));
	return nRetCode;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_reset_session
// Purpose      : reset last http session..
// Returns      : 0: success other: fail
// Last updated : 02/15/2017
//
///////////////////////////////////////////////////////////////////////////////

int HTTPC_reset_session(HTTPParameters *ClientParams)
{

	INT32 nRetCode = HTTP_CLIENT_SUCCESS;
	HTTP_SESSION_HANDLE pSession = ClientParams->pHTTP;
	if ((nRetCode = HTTPClientReset(pSession)) != HTTP_CLIENT_SUCCESS)
		HC_ERR(("Client reset failed."));

	return nRetCode;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_write
// Purpose      : write data to server.
// Returns      : 0: success other: fail
// Last updated : 02/15/2017
//
///////////////////////////////////////////////////////////////////////////////

//int HTTPC_write( HTTP_SESSION_HANDLE pSession,VOID *pBuffer, UINT32 toWrite)
int HTTPC_write( HTTPParameters *ClientParams, VOID *pBuffer, UINT32 toWrite)
{
	UINT32 nRetCode = 0;
	UINT32 wSize = toWrite;
	HTTP_SESSION_HANDLE pSession = ClientParams->pHTTP;

	nRetCode = HTTPClientWriteData (pSession, pBuffer, wSize, 0);
	return nRetCode;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_read
// Purpose      : receive the server data.
// Returns      : 0: success other: fail
// Last updated : 02/15/2017
//
///////////////////////////////////////////////////////////////////////////////
//int HTTPC_read( HTTP_SESSION_HANDLE pSession,VOID *pBuffer, UINT32 toRead, UINT32 *recived)

int HTTPC_read(HTTPParameters *ClientParams,VOID *pBuffer, UINT32 toRead, UINT32 *recived)
{
	UINT32 nRetCode = 0;
	UINT32 *rBytes = recived;
	UINT32 rSize = toRead;
	HTTP_SESSION_HANDLE pSession = ClientParams->pHTTP;

	nRetCode = HTTPClientReadData(pSession,pBuffer,rSize,0,rBytes);
	*recived = *rBytes;
	return nRetCode;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_close
// Purpose      : close session.
// Returns      : 0: success other: fail
// Last updated : 02/15/2017
//
///////////////////////////////////////////////////////////////////////////////
//int HTTPC_close( HTTP_SESSION_HANDLE pSession)
int HTTPC_close(HTTPParameters *ClientParams)
{
	UINT32 nRetCode = 0 ;
	HTTP_SESSION_HANDLE pSession = ClientParams->pHTTP;
	nRetCode = HTTPClientCloseRequest(&pSession);
	return nRetCode;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_get
// Purpose      : get the server data.
// Returns      : 0: success other: fail
// Last updated : 02/15/2017
//
///////////////////////////////////////////////////////////////////////////////
int HTTPC_get(HTTPParameters *ClientParams,CHAR *Buffer, INT32 bufSize, INT32 *recvSize)
{
	INT32 nRetCode;
	UINT32 nSize;
	nSize = bufSize;

	HTTP_SESSION_HANDLE pHTTP;
	HTTP_CLIENT httpClient;
	do
	{
		if (ClientParams->isTransfer)
		{
			HC_DBG(("HTTP GET Recving."));
			nRetCode = HTTPClientReadData(ClientParams->pHTTP,Buffer,nSize,0,&nSize);
			*recvSize = nSize;
			break;
		}
openrequest:
		ClientParams->isTransfer = TRUE;
		HC_DBG(("HTTP GET."));
		// Open the HTTP request handle
		pHTTP = HTTPClientOpenRequest(ClientParams->Flags);
		ClientParams->pHTTP = pHTTP;
#ifdef _HTTP_DEBUGGING_
        HTTPClientSetDebugHook(pHTTP,&HTTPDebug);
#endif
		// Set the Verb
		if((nRetCode = HTTPClientSetVerb(pHTTP,VerbGet)) != HTTP_CLIENT_SUCCESS)
		{
			break;
		}
		// Set authentication
		if(ClientParams->AuthType != AuthSchemaNone)
		{
			if((nRetCode = HTTPClientSetAuth(pHTTP,ClientParams->AuthType,NULL)) != HTTP_CLIENT_SUCCESS)
			{
				break;
			}
			// Set authentication
			if((nRetCode = HTTPClientSetCredentials(pHTTP,ClientParams->UserName,ClientParams->Password)) != HTTP_CLIENT_SUCCESS)
			{
				break;
			}
		}
		// Send a request for the home page
		if((nRetCode = HTTPClientSendRequest(pHTTP,ClientParams->Uri,NULL,0,FALSE,0,0)) != HTTP_CLIENT_SUCCESS)
		{
			HC_ERR(("HTTP Send Request failed.."));
			break;
		}
		// Retrieve the the headers and analyze them
		if((nRetCode = HTTPClientRecvResponse(pHTTP,0)) != HTTP_CLIENT_SUCCESS)
		{
			break;
		}
		// Get the Client info
		if ((nRetCode = HTTPClientGetInfo(pHTTP, &httpClient)) != HTTP_CLIENT_SUCCESS)
		{
			HC_ERR(("get info failed.."));
			break;
		}
		//if the HTTPStatusCode is not HTTP_STATUS_OK, it may be a redirect url
		if(httpClient.HTTPStatusCode != HTTP_STATUS_OK)
		{
			//if the HTTPStatusCode is 302/301
			if((httpClient.HTTPStatusCode == HTTP_STATUS_OBJECT_MOVED) ||
				(httpClient.HTTPStatusCode == HTTP_STATUS_OBJECT_MOVED_PERMANENTLY))
			{
				HC_DBG(("Get a redirect url, this request will be closed and create a new request.."));
				//copy the redirect url
				if(httpClient.RedirectUrl->nLength < sizeof(ClientParams->Uri))
				{
					strncpy(ClientParams->Uri, httpClient.RedirectUrl->pParam, httpClient.RedirectUrl->nLength);
					if((nRetCode = HTTPClientCloseRequest(&pHTTP)) != HTTP_CLIENT_SUCCESS)
					{
						HC_ERR(("close request failed.."));
						break;
					}
					goto openrequest;
				}
			}
			else
			{
				nRetCode = HTTP_CLIENT_EOS;
				break;
			}
		}
		// Get the data
		nRetCode = HTTPClientReadData(pHTTP,Buffer,nSize,0,&nSize);
		*recvSize = nSize;

	} while(0);

	if (nRetCode != HTTP_CLIENT_SUCCESS)
	{
		HC_DBG(("Close Request.."));
		ClientParams->isTransfer = 0;
		HTTPClientCloseRequest(&(ClientParams->pHTTP));
	}
	return nRetCode;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_Register_user_certs
// Purpose      : Register user certs callback.
// Returns      : none
// Last updated : 08/21/2017
//
///////////////////////////////////////////////////////////////////////////////
void HTTPC_Register_user_certs(HTTPC_USR_CERTS certs)
{
	httpc_user_certs = certs;
}

///////////////////////////////////////////////////////////////////////////////
//
// Function     : HTTPC_obtain_user_certs
// Purpose      : get user certs pointer.
// Returns      : pointer to the user certs
// Last updated : 08/21/2017
//
///////////////////////////////////////////////////////////////////////////////
void* HTTPC_obtain_user_certs()
{
	if (httpc_user_certs)
		return httpc_user_certs();
	else
		return NULL;
}

void HTTPC_set_ssl_verify_mode(unsigned char mode)
{
	httpc_ssl_verify_mode = mode;
}

unsigned char HTTPC_get_ssl_verify_mode()
{
	return httpc_ssl_verify_mode;
}

