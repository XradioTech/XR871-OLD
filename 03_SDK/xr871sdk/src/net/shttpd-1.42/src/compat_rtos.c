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

#include "defs.h"

struct local {
	struct llhead	link;
	const char      *name;
	char            *body;
	unsigned long	mode;
};

struct llhead	registered_file;

time_t TIME(time_t *timer)
{
	if (!timer)
		return (time_t)OS_GetTime();
	*timer = (time_t)OS_GetTime();
	return *timer;
}

void _shttpd_free(void *ptr)
{
	if (!ptr)
		return;
	free(ptr);
	ptr = NULL;
}

void *_shttpd_zalloc(size_t size)
{
	void *buf = malloc(size);
	if (!buf)
		return NULL;
	memset(buf,0,size);
	return buf;
}

void _shttpd_init_local_file(const struct usr_file *list, int count)
{
	if (list == NULL || count == 0) {
		_shttpd_elog(E_LOG, NULL, "invalid param (%s)",__func__);
		return;
	}
	int i = 0;
	struct local *file = NULL;

	LL_INIT(&registered_file);
	for (i = 0; i < count; i++) {
		if ((file = _shttpd_zalloc(sizeof(*file))) == NULL)
		{
			_shttpd_elog(E_LOG, NULL, "init local file _shttpd_zalloc failed.");
			return;
		}
		if (list[i].name != NULL) {
			file->name = list[i].name;
			file->mode = (file->name[strlen(file->name)-1]
				             == '/')? _S_IFDIR : _S_IFREG;
			if (file->mode == _S_IFREG)
				file->body = list[i].body;
		} else
			break;
		LL_ADD(&registered_file, &file->link);
	}
}

void _shttpd_deinit_local_file()
{
	struct llhead *lp, *tmp;
	struct local *local_file = NULL;

	LL_FOREACH_SAFE(&registered_file, lp, tmp) {
		local_file = LL_ENTRY(lp, struct local, link);
		LL_DEL(&local_file->link);
		_shttpd_free(local_file);
	}
}

int _shttpd_add_file(struct local *file)
{
	struct llhead *lp;
	struct local *local_file = NULL;;

	LL_FOREACH(&registered_file, lp) {
		local_file = LL_ENTRY(lp, struct local, link);
		if (_shttpd_strncasecmp(local_file->name, file->name,
					 strlen(local_file->name) + 1) == 0)
			return -1;
	}
	LL_ADD(&registered_file, &file->link);
	return 0;
}

int _shttpd_delete_file(struct local *file)
{
	struct llhead *lp;
	struct local *local_file = NULL;;

	LL_FOREACH(&registered_file, lp) {
		local_file = LL_ENTRY(lp, struct local, link);
		if (_shttpd_strncasecmp(local_file->name, file->name,
	                                 strlen(local_file->name) + 1) == 0) {
			LL_DEL(&file->link);
			_shttpd_free(file);
			return 0;
		}
	}
	return -1;
}

int _shttpd_lookup_file(struct local **file, const char *name)
{
	struct llhead *lp;
	struct local *local_file = NULL;;

	LL_FOREACH(&registered_file, lp) {
		local_file = LL_ENTRY(lp, struct local, link);
		if (_shttpd_strncasecmp(local_file->name, name, strlen(local_file->name) +1) == 0) {
			*file = local_file;
			return 0;
		}
	}
	return -1;
}

void _shttpd_set_close_on_exec(int fd)
{
}

int _shttpd_stat(const char *path, struct stat *stp)
{
        struct local *file;
	 if (_shttpd_lookup_file(&file, path) != 0) {
	 	_shttpd_elog(E_LOG, NULL, "file path mismatch (%s).", __func__);
		return -1;
	 }
	stp->st_mode = file->mode;
	if (stp->st_mode == _S_IFREG)
		stp->st_size = strlen(file->body);
	else
		stp->st_size = 0;
	return 0;
}

char* _shttpd_open(const char *path, int flags, int mode)
{
	struct local *file;

	if (_shttpd_lookup_file(&file, path) != 0) {
		_shttpd_elog(E_LOG, NULL, "file path mismatch (%s).", __func__);
		return NULL;
	}
	if (file->mode == _S_IFREG)
			return file->body;
		else
			return NULL;
}

int _shttpd_remove(const char *path)
{
	return 0;

}

int _shttpd_rename(const char *path1, const char *path2)
{
	return 0;
}

int _shttpd_mkdir(const char *path, int mode)
{
	return 0;
}

char* _shttpd_getcwd(char *buffer, int maxlen)
{
	return NULL;
}

int _shttpd_set_non_blocking_mode(int fd)
{
	int ret = -1;

	int val = 1;
	if (ioctlsocket(fd, FIONBIO,  (void *)&val) != 0) {
		DBG(("nonblock: fcntl(F_SETFL): %d", ERRNO));
	} else {
		ret = 0;	/* Success */
	}
	return (ret);
}
