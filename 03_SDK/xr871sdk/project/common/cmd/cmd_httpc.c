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

#if (defined(__CONFIG_ARCH_DUAL_CORE))

#include "cmd_debug.h"
#include "cmd_util.h"
#include "net/HTTPClient/HTTPCUsr_api.h"

static int HTTPC_checksum_check(void *url)
{
        if (strstr(url, "checksum") != NULL)
                return 0;
        return -1;
}
#if 0
static char HTTPC_cal_checksum(void *buffer, int length)
{
        int *cal = (char *)buffer;
        int result = 0;
        if ((length % 4) != 0)
                cal_length = length - (length % 4)
                        while (cal_length != 0)
                                result += cal[--cal_length];
        return result;
}
#else
static char HTTPC_cal_checksum(void *buffer, int length)
{
        unsigned char *cal = (unsigned char *)buffer;
        unsigned char result = 0;
        while (length != 0)
                result += cal[--length];
        return result;
}
#endif
static unsigned char checksum = 0;
static int checksum_flag = 0;


static int HTTPC_get_test(HTTPParameters *clientParams)
{
        int nRetCode = 0;
        int recvSize = 0;
        char *buf = NULL;
        if ((buf = malloc(HTTP_CLIENT_BUFFER_SIZE)) == NULL) {
                return -1;
        }

        do {
                nRetCode = HTTPC_get(clientParams,buf, 4096,(INT32 *)&recvSize);
                if (checksum_flag == 1)
                        checksum += HTTPC_cal_checksum(buf,recvSize);

                if (nRetCode != HTTP_CLIENT_SUCCESS)
                        break;
        } while(1);

        if (buf) {
                free(buf);
                buf = NULL;
        }

        if (nRetCode == HTTP_CLIENT_EOS) {
                CMD_DBG("HTTP GET SUCCESS\n");
                nRetCode = HTTP_CLIENT_SUCCESS;
        }

        return nRetCode;
}


static int HTTPC_get_test_fresh(HTTPParameters *clientParams)
{
        int contentLength = 0, ret = 0;
        unsigned int toReadLength = 4096;
        unsigned int Received = 0;

        char *buf = malloc(toReadLength);
        if (buf == NULL) {
                CMD_ERR("malloc pbuffer failed..\n");
                return -1;
        }

        clientParams->HttpVerb = VerbGet;

        if (HTTPC_open(clientParams) != 0) {
                CMD_ERR("http open err..\n");
        } else if (HTTPC_request(clientParams,(void *)&contentLength) != 0) {
                CMD_ERR("http request err..\n");
        } else if (contentLength != 0) {

                do {
                        if ((ret = HTTPC_read(clientParams, buf, toReadLength, (void *)&Received)) != 0) {
                                if (ret == 1000) {
                                        CMD_DBG("The end..\n");
                                } else
                                        CMD_ERR("Transfer err...ret:%d\n",ret);
                        } else {
                                CMD_DBG("get data :%d\n", Received);
                        }

                        if (checksum_flag == 1)
                                checksum += HTTPC_cal_checksum(buf,Received);

                        if (ret != 0)
                                break;

                } while (1);

        }

        free(buf);
        HTTPC_close(clientParams);
	return ret;
}


static int HTTPC_post_test(HTTPParameters *clientParams, char *credentials)
{
        int contentLength = 0, ret = 0;
        unsigned int toReadLength = 4096;
        unsigned int Received = 0;
        char *buf = malloc(toReadLength);
        if (buf == NULL)
                CMD_ERR("malloc pbuffer failed..\n");

        memset(buf, 0, toReadLength);
        clientParams->HttpVerb = VerbPost;
        clientParams->pData = buf;
        memcpy(buf, credentials, strlen(credentials));
        clientParams->pLength = strlen(credentials);

        if (HTTPC_open(clientParams) != 0) {

                CMD_ERR("http open err..\n");

        } else if (HTTPC_request(clientParams,(void *)&contentLength) != 0) {
                CMD_ERR("http request err..\n");
        } else if (contentLength != 0 || (((P_HTTP_SESSION)(clientParams->pHTTP))->HttpFlags & HTTP_CLIENT_FLAG_CHUNKED )) {

                do {

                        if ((ret = HTTPC_read(clientParams, buf, toReadLength, (void *)&Received)) != 0) {
                                if (ret == 1000) {
                                        ret = 0;
                                        CMD_DBG("The end..\n");
                                } else
                                        CMD_ERR("Transfer err...ret:%d\n",ret);

                                break;


                        } else {
                                // CMD_DBG("get data :%d\n", Received);
                        }

                } while (1);

        }
        free(buf);
        HTTPC_close(clientParams);
        return ret;
}

static int HTTPC_head_test(HTTPParameters *clientParams)
{
        return 0;
}
//extern uint32_t heap_free_size(void);
//extern void wrap_malloc_heap_info(void);
static int httpc_exec(char *cmd)
{
        CMD_DBG("HTTPC TEST %s\n",cmd);
        int ret = -1;
        int argc = 0;
        char *argv[6];

        HTTPParameters *clientParams;
        clientParams = malloc(sizeof(*clientParams));
        if (!clientParams) {
                return -1;
        } else{
                memset(clientParams,0,sizeof(HTTPParameters));
        }

        argc = cmd_parse_argv(cmd, argv, 6);
        if (argc < 2) {
                CMD_ERR("invalid httpc cmd, argc %d\n", argc);
                ret = -1;
                goto releaseParams;
        }

        if (HTTPC_checksum_check(argv[1]) == 0)
                checksum_flag = 1;
        else
                checksum_flag = 0;
        checksum = 0;

        strcpy(clientParams->Uri,argv[1]);
        if (cmd_strncmp(argv[0],"get",3) == 0) {
	//tryagain:
		//printf("#####heap:%x#####\n",heap_free_size());
		//wrap_malloc_heap_info();
                ret = HTTPC_get_test(clientParams);

		//printf("#####heap:%x#####\n",heap_free_size());
		//OS_Sleep(5);
		//goto tryagain;

        } else if (cmd_strncmp(argv[0], "post", 4) == 0) {
                if (argc < 3) {
                        CMD_ERR("invalid httpc cmd, argc %d\n", argc);
                        ret = -1;
                        goto releaseParams;
                }
                ret = HTTPC_post_test(clientParams, argv[2]);

        } else if (cmd_strncmp(argv[0], "__get", 5) == 0) {

                ret = HTTPC_get_test_fresh(clientParams);

        } else if (cmd_strncmp(argv[0], "head", 4) == 0) {
               ret = HTTPC_head_test(clientParams);
        }else if (cmd_strncmp(argv[0], "auth-get", 8) == 0) {
        	// net httpc auth-get(0) url(1) test(id)(2) 12345678(key)(3)
        	if (argc < 4) {
                        CMD_ERR("invalid httpc cmd, argc %d\n", argc);
                        ret = -1;
                        goto releaseParams;
                }
		cmd_memcpy(clientParams->UserName, argv[2], cmd_strlen(argv[2]));
		cmd_memcpy(clientParams->Password, argv[3], cmd_strlen(argv[3]));
		clientParams->AuthType = AuthSchemaDigest;
		ret = HTTPC_get_test_fresh(clientParams);

        }else if (cmd_strncmp(argv[0], "ssl-get", 7) == 0) {
		//net httpc ssl-get(0) url(1)
		ret = HTTPC_get_test_fresh(clientParams);

        }else if (cmd_strncmp(argv[0], "ssl-post", 8) == 0) {
		//net httpc ssl-post(0) url(1) data(2)
	        if (argc < 3) {
                        CMD_ERR("invalid httpc cmd, argc %d\n", argc);
                        ret = -1;
                        goto releaseParams;
                }
                ret = HTTPC_post_test(clientParams, argv[2]);

        }else if (cmd_strncmp(argv[0], "muti-get", 8) == 0) {
        }else if (cmd_strncmp(argv[0], "muti-post", 8) == 0) {
        }

releaseParams:
        if (clientParams != NULL) {
                free(clientParams);
                clientParams = NULL;
        }
        if (checksum_flag == 1) {
                printf("[httpc test]:checksum = %#x\n",checksum);
                if (checksum != 0xff)
                        ret = -1;
                else
                        ret = 0;
        }
        checksum_flag = 0;
        checksum = 0;
        return ret;
}

#define HTTPC_THREAD_STACK_SIZE		(4 * 1024)
static OS_Thread_t g_httpc_thread;

void httpc_cmd_task(void *arg)
{
        char *cmd = (char *)arg;
	int ret = httpc_exec(cmd);
        if (ret != 0 && ret != 1000)
                CMD_LOG(1, "[httpc cmd]:failed. %s\n",cmd);
        else {
                CMD_LOG(1, "[httpc cmd]:success. %s\n",cmd);
        }
        OS_ThreadDelete(&g_httpc_thread);
}

enum cmd_status cmd_httpc_exec(char *cmd)
{
        if (OS_ThreadIsValid(&g_httpc_thread)) {
                CMD_ERR("httpc task is running\n");
                return CMD_STATUS_FAIL;
        }

        if (OS_ThreadCreate(&g_httpc_thread,
                                "",
                                httpc_cmd_task,
                                (void *)cmd,
                                OS_THREAD_PRIO_APP,
                                HTTPC_THREAD_STACK_SIZE) != OS_OK) {
                CMD_ERR("httpc task create failed\n");
                return CMD_STATUS_FAIL;
        }

        return CMD_STATUS_OK;
}


#if 0
        if (cmd_strncmp(argv[0],"__post_m",8) == 0) {

                strcpy(ClientParams->Uri,argv[1]);
                int conLength = 0;
                unsigned int toReadLength = 4096;
                unsigned int Received = 0;
                char *pBuffer = malloc(toReadLength);
                if (pBuffer == NULL)
                        printf("malloc pbuffer failed..\n");

                memset(pBuffer, 0, toReadLength);
                ClientParams->HttpVerb = VerbPost;
                ClientParams->pData = pBuffer;
                ClientParams->sData = client_credentials;
                ClientParams->sLength = strlen(client_credentials);

                if (HTTPC_open(ClientParams) != 0) {

                        printf("http open err..\n");

                }
                unsigned int times = 0x2;
                while(times--) {
                        if (HTTPC_request(ClientParams,(void *)&conLength) != 0) {
                                printf("http request err..\n");

                        } else if (conLength != 0 || (((P_HTTP_SESSION)(ClientParams->pHTTP))->HttpFlags & HTTP_CLIENT_FLAG_CHUNKED )) {

                                do {
                                        int retValue = 0;
                                        if ((retValue = HTTPC_read(ClientParams, pBuffer, toReadLength, (void *)&Received)) != 0) {
                                                if (retValue == 1000) {
                                                        printf("The end..\n");
                                                } else
                                                        printf("Transfer err...retValue:%d\n",retValue);
                                                break;


                                        } else {
                                                printf("get data :%d\n", Received);
                                        }

                                } while (1);
                        }
                        if (retValue != 1000 && retValue != 0) {
                                printf("httpc post err...\n");
                                break;

                        } else {
                                printf("*******httpc post success********\n");
                                while(1){
                                        OS_Sleep(20);
                                }
                                ret = retValue;
                        }
                        OS_Sleep(2);
                }

                free(pBuffer);
                HTTPC_close(ClientParams);
        }
#endif

#endif /* (defined(__CONFIG_ARCH_DUAL_CORE)) */
