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

#include "atcmd/at_command.h"
#include "at_private.h"
#include "at_debug.h"

AT_ERROR_CODE at_get_newline(char *para, s32 size);

s32 at_get_value(char *strbuf, s32 pt, void *pvar, s32 vsize)
{
	s32 i;

	if (pvar == NULL) {
		return 2; /* null pointer */
	}

	if (pt == APT_DI) {
		sprintf(strbuf, "%d", *((at_di_t *)pvar));
	}
	else if (pt == APT_HI) {
		sprintf(strbuf, "0x%08X", *((at_hi_t *)pvar));
	}
	else if (pt == APT_HEX) {
		if (vsize > 0) {
			sprintf(strbuf, "%02X", ((at_hex_t *)pvar)[0]);
		}

		for (i = 1; i < vsize; i++) {
			sprintf(&strbuf[strlen(strbuf)], ":%02X", ((at_hex_t *)pvar)[i]);
		}
	}
	else if (pt == APT_IP) {
		if (vsize > 0) {
			sprintf(strbuf, "%d", ((unsigned char *)pvar)[0]);
		}

		for (i = 1; i < vsize; i++) {
			sprintf(&strbuf[strlen(strbuf)], ".%d", ((unsigned char *)pvar)[i]);
		}
	}
	else if (pt == APT_TEXT) {
		sprintf(strbuf, "%s", (char *)pvar);
	}
	else {
		return 1; /* error parameter type */
	}

	return 0; /* succeed */
}

s32 at_set_value(s32 pt, void *pvar, s32 vsize, at_value_t *value)
{
	switch (pt) {
	case APT_TEXT:

		strcpy(pvar, value->text);

		break;

	case APT_HEX:

		memcpy(pvar, value->hex, vsize);

		break;

	case APT_DI:

		*((at_di_t *)pvar) = value->di;

		break;

	case APT_HI:

		*((at_hi_t *)pvar) = value->hi;

		break;

	case APT_IP:

		memcpy(pvar,value->ip, vsize);

		break;

	default:

		return -1; /* error parameter type */

		break;
	}

	return 0; /* succeed */
}

int at_atoi(char *str)
{
	return atoi(str);
}

static int is_hex(char ch)
{
	if (((ch >= '0') && (ch <= '9')) || ((ch >= 'A') && (ch <= 'F')) || ((ch >= 'a') && (ch <= 'f'))) {
		return 1;
	}
	else {
		return 0;
	}
}

static int hex_to_num(char ch)
{
	if ((ch >= '0') && (ch <= '9')) {
		return ch - '0';
	}
	else if ((ch >= 'A') && (ch <= 'F')) {
		return ch - 'A' + 10;
	}
	else if ((ch >= 'a') && (ch <= 'f')) {
		return ch - 'a' + 10;
	}
	else {
		return -1;
	}
}

static int is_dec(char ch)
{
	if ((ch >= '0') && (ch <= '9')) {
		return 1;
	}
	else {
		return 0;
	}
}

/**
  * @brief  Get the text parameter.
  * @param	ppara: the pointer to command line string buffer
  * @param	pvar: the pointer to the variable
  * @param	opt: the option of the parameter
  * @retval Bit0: over flag,	Bit1: error flag
  */
static u32 get_text_para(char **ppara, void *pvar, u32 opt)
{
	char *para;
	int over = 0;
	int error = 0;
	char strbuf[AT_PARA_MAX_SIZE];
	int cnt;
	int size_limit;
	u32 end_type = 0;

	para = *ppara;

	cnt = 0;
	strbuf[0] = '\0';

	size_limit = SIZE_LIMIT(opt);

	while (cnt < size_limit) {
		if (*para == AT_SEPARATOR) {
			para++; /* skip separator */
			end_type = AET_PARA;
			break;
		}
		else if (*para == AT_CR) {
			para++; /* skip <CR> */

			if (*para == AT_LF) {
				para++; /* skip <LF> */
			}

			over = 1;
			end_type = AET_LINE;

			break;
		}
		else if (*para == AT_LF) {
			para++; /* skip <LF> */

			over = 1;
			end_type = AET_LINE;

			break;
		}

		strbuf[cnt++] = *para++;
	}

	if (!(end_type & opt)) {
		error = 1;
	}

	if (!error) {
		if (cnt > 0 && cnt < size_limit) {
			strbuf[cnt++] = '\0';

			memcpy(pvar,strbuf,cnt);
		}
		else {
			error = 1;
		}
	}

	*ppara = para;

	return (over | (error << 1));
}

/**
  * @brief  Get the hex parameter.
  * @param	ppara: the pointer to command line string buffer
  * @param	pvar: the pointer to the variable
  * @param	opt: the option of the parameter
  * @retval Bit0: over flag,	Bit1: error flag
  */
static u32 get_hex_para(char **ppara, void *pvar, u32 opt)
{
	char *para;
	int over = 0;
	int error = 0;
	char strbuf[AT_PARA_MAX_SIZE];
	int cnt;
	int size_limit;
	u32 end_type = 0;
	char hexbuf[2];
	int hexcnt = 0;

	para = *ppara;

	cnt = 0;
	strbuf[0] = '\0';

	size_limit = SIZE_LIMIT(opt);

	while (cnt < size_limit) {
		if (*para == AT_SEPARATOR) {
			para++; /* skip separator */
			end_type = AET_PARA;
			break;
		}
		else if (*para == AT_CR) {
			para++; /* skip <CR> */

			if (*para == AT_LF) {
				para++; /* skip <LF> */
			}

			over = 1;
			end_type = AET_LINE;

			break;
		}
		else if (*para == AT_LF) {
			para++; /* skip <LF> */

			over = 1;
			end_type = AET_LINE;

			break;
		}

		if (hexcnt < 2) {
			if (is_hex(*para)) {
				hexbuf[hexcnt++] = *para++;
			}
			else {
				error = 1;
				break;
			}
		}
		else if (hexcnt == 2) {
#if 0
			if (*para++ == AT_COLON) {
				strbuf[cnt++] = (hex_to_num(hexbuf[0]) << 4) + hex_to_num(hexbuf[1]);
				hexcnt = 0; /* reset counter */
			}
			else {
				error = 1;
				break;
			}
#else
			strbuf[cnt++] = (hex_to_num(hexbuf[0]) << 4) + hex_to_num(hexbuf[1]);
			hexcnt = 0; /* reset counter */
#endif
		}
		else {
			error = 1;
			break;
		}
	}

	if (end_type & opt) {
		if (hexcnt == 2) {
			para++; /* skip separator */
			strbuf[cnt++] = (hex_to_num(hexbuf[0]) << 4) + hex_to_num(hexbuf[1]);
		}
		else {
			error = 1;
		}
	}
	else {
		error = 1;
	}

	if (!error) {
		memset(pvar, 0, size_limit); /* clear hex */
		memcpy(pvar, strbuf, cnt);
	}

	*ppara = para;

	return (over | (error << 1));
}

/**
  * @brief  Get the decimal integer parameter.
  * @param	ppara: the pointer to command line string buffer
  * @param	pvar: the pointer to the variable
  * @param	opt: the option of the parameter
  * @retval Bit0: over flag,	Bit1: error flag
  */
static u32 get_di_para(char **ppara, void *pvar, u32 opt)
{
	char *para;
	int over = 0;
	int error = 0;
	char strbuf[AT_PARA_MAX_SIZE];
	int cnt;
	int size_limit;
	u32 end_type = 0;

	para = *ppara;

	cnt = 0;
	strbuf[0] = '\0';

	size_limit = 10 + 1; /* SIZE_LIMIT(opt) */

	while (cnt < size_limit) {
		if (*para == AT_SEPARATOR) {
			para++; /* skip separator */
			end_type = AET_PARA;
			break;
		}
		else if (*para == AT_CR) {
			para++; /* skip <CR> */

			if (*para == AT_LF) {
				para++; /* skip <LF> */
			}

			over = 1;
			end_type = AET_LINE;

			break;
		}
		else if (*para == AT_LF) {
			para++; /* skip <LF> */

			over = 1;
			end_type = AET_LINE;

			break;
		}

		if (is_dec(*para)) {
			strbuf[cnt++] = *para++;
		}
		else {
			error = 1;
			break;
		}
	}

	if (!(end_type & opt)) {
		error = 1;
	}
	else {
		if (!error) {
			if (cnt>0 && cnt < size_limit) {
				strbuf[cnt++] = '\0';
				*((at_di_t *)pvar) = at_atoi(strbuf);
			}
			else {
				error = 1;
			}
		}
	}

	*ppara = para;

	return (over | (error << 1));
}

/**
  * @brief  Get the hexadecimal integer parameter.
  * @param	ppara: the pointer to command line string buffer
  * @param	pvar: the pointer to the variable
  * @param	opt: the option of the parameter
  * @retval Bit0: over flag,	Bit1: error flag
  */
static u32 get_hi_para(char **ppara, void *pvar, u32 opt)
{
	char *para;
	int over = 0;
	int error = 0;
	char strbuf[AT_PARA_MAX_SIZE];
	int cnt;
	int size_limit;
	u32 end_type = 0;

	para = *ppara;

	cnt = 0;
	strbuf[0] = '\0';

	size_limit = 10 + 1; /* SIZE_LIMIT(opt) */

	while (cnt < size_limit) {
		if (*para == AT_SEPARATOR) {
			para++; /* skip separator */
			end_type = AET_PARA;
			break;
		}
		else if (*para == AT_CR) {
			para++; /* skip <CR> */

			if (*para == AT_LF) {
				para++; /* skip <LF> */
			}

			over = 1;
			end_type = AET_LINE;

			break;
		}
		else if (*para == AT_LF) {
			para++; /* skip <LF> */

			over = 1;
			end_type = AET_LINE;

			break;
		}

		if (is_hex(*para)) {
			strbuf[cnt++] = *para++;
		}
		else {
			error = 1;
			break;
		}
	}

	if (!(end_type & opt)) {
		error = 1;
	}
	else {
		if (!error) {
			if (cnt>0 && cnt < size_limit) {
				strbuf[cnt++] = '\0';
				sscanf(strbuf,"%x",(at_di_t *)pvar);
			}
			else {
				error = 1;
			}
		}
	}

	*ppara = para;

	return (over | (error << 1));
}

/**
  * @brief  Get the ip parameter.
  * @param	ppara: the pointer to command line string buffer
  * @param	pvar: the pointer to the variable
  * @param	opt: the option of the parameter
  * @retval Bit0: over flag,	Bit1: error flag
  */
static u32 get_ip_para(char **ppara, void *pvar, u32 opt)
{
	char *para;
	int over = 0;
	int error = 0;
	char strbuf[AT_PARA_MAX_SIZE];
	int cnt;
	int size_limit;
	u32 end_type = 0;
	char decbuf[3+1];
	int num;
	int deccnt = 0;

	para = *ppara;

	cnt = 0;
	strbuf[0] = '\0';

	size_limit = 4; /* SIZE_LIMIT(opt) */
	memset(decbuf, 0, sizeof(decbuf)); /* clear buffer */

	while (cnt < size_limit)
	{
		if (*para == AT_SEPARATOR) {
			para++; /* skip separator */
			end_type = AET_PARA;
			break;
		}
		else if (*para == AT_CR) {
			para++; /* skip <CR> */

			if (*para == AT_LF) {
				para++; /* skip <LF> */
			}

			over = 1;
			end_type = AET_LINE;

			break;
		}
		else if (*para == AT_LF) {
			para++; /* skip <LF> */

			over = 1;
			end_type = AET_LINE;

			break;
		}

		if (*para == AT_DOT) {
			if (deccnt >= 1 && deccnt <= 3) {
				num = at_atoi(decbuf);

				if (num >= 0 && num <= 255) {
					strbuf[cnt++] = num;
				}
				else {
					error = 1;
				}
			}
			else {
				error = 1;
			}

			para++; /* skip DOT */
			deccnt = 0; /* reset counter */
			memset(decbuf, 0, sizeof(decbuf)); /* clear buffer */

			if (error)
			{
				break;
			}
		}
		else if (is_dec(*para)) {
			if (deccnt < 3) {
				decbuf[deccnt++] = *para++;
			}
			else {
				error = 1;
				break;
			}
		}
		else {
			error = 1;
			break;
		}
	}

	if (!(end_type & opt)) {
		error = 1;
	}
	else {
		if (deccnt >= 1 && deccnt <= 3) {
			num = at_atoi(decbuf);

			if (num >= 0 && num <= 255) {
				strbuf[cnt++] = num;
			}
			else {
				error = 1;
			}
		}
		else {
			error = 1;
		}

		if (cnt != 4) {
			error = 1;
		}

		if (!error)	{
			memset(pvar, 0, size_limit); /* clear hex */
			memcpy(pvar, strbuf, cnt);
		}
	}

	*ppara = para;

	return (over | (error << 1));
}

/**
  * @brief  Get the parameters of the AT command.
  * @param	ppara: the pointer to command line string buffer
  * @param	list: the descriptor list of the parameters
  * @param	lsize: the size of the list
  * @param	pcnt: the pointer to parameter counter
  * @retval AEC_OK: succeed,	Other: fail
  */
AT_ERROR_CODE at_get_parameters(char **ppara, at_para_descriptor_t *list, s32 lsize, s32 *pcnt)
{
	char *para;
	int paracnt = 0;
	int over = 0;
	int error = 0;
	int i;
#ifdef _DEBUG
	char strbuf[AT_PARA_MAX_SIZE*4];
#endif
	int para_type;
	u32 res;

	para = *ppara;

	if ((list == NULL) || (lsize == 0)) {
		/* do not need para counter */

		return at_get_newline(para, 2);
	}

	for (i = 0; i < lsize; i++) {
#ifdef _DEBUG
/* display default */
		printf("default p%d=", i+1);
		at_get_value(strbuf,list[i].pt, list[i].pvar, SIZE_LIMIT(list[i].option));
		printf("%s", strbuf);
		printf("\n");
#endif

		para_type = list[i].pt;

		switch (para_type) {
		case APT_TEXT:

			res = get_text_para(&para, list[i].pvar, list[i].option);

			break;

		case APT_HEX:

			res = get_hex_para(&para, list[i].pvar, list[i].option);

			break;

		case APT_DI:

			res = get_di_para(&para, list[i].pvar, list[i].option);

			break;

		case APT_HI:

			res = get_hi_para(&para, list[i].pvar, list[i].option);

			break;

		case APT_IP:

			res = get_ip_para(&para, list[i].pvar, list[i].option);

			break;

		default:

			return AEC_PARA_ERROR;

			break;
		}

		over = res & 0x01;
		error = (res >> 1) & 0x01;

		if (!error) {
			paracnt++;
#ifdef _DEBUG
/* debug info */
			printf("current p%d=", i+1);
			at_get_value(strbuf,list[i].pt, list[i].pvar, SIZE_LIMIT(list[i].option));
			printf("%s", strbuf);
			printf("\n");
#endif
		}

		if (over || error) {
			break;
		}
	}

	*ppara = para; /* modify ppara pointer */
	*pcnt = paracnt;

	if (error) {
		return AEC_PARA_ERROR;
	}

	return AEC_OK;
}

AT_ERROR_CODE at_get_newline(char *para, s32 size)
{
	if (*para == AT_LF) {
		return AEC_OK;
	}
	else if (*para == AT_CR) {
		if ((size >= 2) && (*(para+1) == AT_LF)) {
			return AEC_OK;
		}
		else {
			return AEC_OK;
		}
	}
	else {
		return AEC_CMD_ERROR;
	}
}
