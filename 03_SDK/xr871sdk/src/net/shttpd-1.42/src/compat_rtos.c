/*
 * Copyright (c) 2017-2020 AW-IOT Team
 * All rights reserved
 *
 * "THE BEER-WARE LICENSE" (Revision 42):
 * Sergey Lyubka wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 */

#include "defs.h"

struct local {
	const char	*name;
	char *body;
	unsigned long	mode;
	struct local *next;
};

struct local *localfile;

void _shttpd_init_local(struct usrfile *filelist,int count)
{
	int i = 0;
	struct usrfile *uri_file;
	struct local *local_file;
	local_file = localfile = malloc(count*(sizeof(*local_file)));
	if (localfile == NULL)
		return ;
        for (i = 0; i < count; i++) {
		uri_file = &filelist[i];
		if (uri_file->name != NULL) {
			local_file->name = uri_file->name;

			local_file->mode = (local_file->name[strlen(local_file->name)-1] == '/')? _S_IFDIR : _S_IFREG;

			if (local_file->mode == _S_IFREG)
				local_file->body = uri_file->body;
			local_file->next = local_file + 1;
			local_file++;
		}
	}
	local_file = NULL;
}

void _shttpd_deinit_local()
{
	if (localfile != NULL)
		free(localfile);
}

void _shttpd_set_close_on_exec(int fd)
{
}

int _shttpd_stat(const char *path, struct stat *stp)
{
        struct local *lfile;

        for (lfile = localfile; lfile != NULL; lfile = lfile->next) {
                if (strcmp(path,lfile->name) == 0)
                        break;
        }
        if (!lfile)
                return (-1);
        else {
		stp->st_mode = lfile->mode;

		if (stp->st_mode == _S_IFREG)
			stp->st_size = strlen(lfile->body);
		else
			stp->st_size = 0;

                return 0;
        }
}

char* _shttpd_open(const char *path)
{
	struct local *lfile;

        for (lfile = localfile; lfile != NULL; lfile = lfile->next)
                if (strcmp(path,lfile->name) == 0)
                        break;
        if (!lfile)
                return (NULL);
        else {
		if (lfile->mode == _S_IFREG)
			return lfile->body;
		else
			return NULL;
        }


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

	int	ret = -1;

	if (lwip_fcntl(fd, F_SETFL, O_NONBLOCK) != 0) {
		DBG(("nonblock: fcntl(F_SETFL): %d", ERRNO));
	} else {
		ret = 0;	/* Success */
	}


	return (ret);

}

time_t TIME(time_t *timer)
{
	if (!timer)
		return (time_t)OS_GetTime();
	*timer = (time_t)OS_GetTime();
	return *timer;
}
void *zalloc(size_t size)
{
	void *buf = malloc(size);
	if (!buf)
		return NULL;
	memset(buf,0,size);
	return buf;
}

#ifndef NO_CGI
int _shttpd_spawn_process(struct conn *c, const char *prog, char *envblk,
		char *envp[], int sock, const char *dir)
{
	int		ret;
	pid_t		pid;
	const char	*p, *interp = c->ctx->options[OPT_CGI_INTERPRETER];

	envblk = NULL;	/* unused */

	if ((pid = vfork()) == -1) {

		ret = -1;
		_shttpd_elog(E_LOG, c, "redirect: fork: %s", strerror(ERRNO));

	} else if (pid == 0) {

		/* Child */

		(void) chdir(dir);
		(void) dup2(sock, 0);
		(void) dup2(sock, 1);
		(void) closesocket(sock);

		/* If error file is specified, send errors there */
		if (c->ctx->error_log)
			(void) dup2(fileno(c->ctx->error_log), 2);

		if ((p = strrchr(prog, '/')) != NULL)
			p++;
		else
			p = prog;

		/* Execute CGI program */
		if (interp == NULL) {
			(void) execle(p, p, NULL, envp);
			_shttpd_elog(E_FATAL, c, "redirect: exec(%s)", prog);
		} else {
			(void) execle(interp, interp, p, NULL, envp);
			_shttpd_elog(E_FATAL, c, "redirect: exec(%s %s)",
			    interp, prog);
		}

		/* UNREACHED */
		exit(EXIT_FAILURE);

	} else {

		/* Parent */
		ret = 0;
		(void) closesocket(sock);
	}

	return (ret);
}
#endif /* !NO_CGI */


