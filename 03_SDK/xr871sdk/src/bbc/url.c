
#include<stdlib.h>
#include<string.h>
#include "url.h"

int url_check(Url *url);

Url *url_parse(char *url_to_parse)
{
	Url *url = (Url*)malloc(sizeof(Url));
	memset(url,0, sizeof(Url));
	//scheme
	char* p = strstr(url_to_parse, "://");
	const size_t scheme_len = (p - url_to_parse);
	url->scheme = (char*)malloc(scheme_len*sizeof(char) + 1);
	strncpy(url->scheme, url_to_parse, scheme_len);
	url->scheme[scheme_len] = '\0';
	url_to_parse += scheme_len + 3;

	//host
	const size_t host_len = strcspn(url_to_parse, ":/?#");
	url->hostname = (char*)malloc(host_len*sizeof(char) + 1);
	strncpy(url->hostname, url_to_parse, host_len);
	url->hostname[host_len] = '\0';
	url_to_parse += host_len;
	//port
	if (url_to_parse[0] == ':')
	   {
	      const size_t port_len = strcspn(++url_to_parse, "/?#");
	      if (port_len)
	      {
	    	  url->port = (char*)malloc(port_len*sizeof(char) + 1);
	    	  strncpy(url->port, url_to_parse, port_len);
	          url_to_parse += port_len;
	      }
	   }
	else{
		char* default_port = "80";
		if (!strncmp(url->scheme, "http", strlen("http")))
			default_port = "80";
		      else if (!strncmp(url->scheme, "https", strlen("https")))
		    	  default_port = "443";
		      else if (!strncmp(url->scheme, "ftp", strlen("ftp")))
		    	  default_port = "21";
		      else if (!strncmp(url->scheme, "ws", strlen("ws")))
		    	  default_port = "80";
		      else if (!strncmp(url->scheme, "wss", strlen("wss")))
		    	  default_port = "443";
		url->port = (char*)malloc(strlen(default_port)*sizeof(char) + 1);
		strcpy(url->port, default_port);
	}
	//path
	url->path = (char*)malloc(strlen(url_to_parse)*sizeof(char) + 1);
	strcpy(url->path, url_to_parse);
	return url;
}


void add_path(char* url, const char* path){
	if(path){
		strcat(url, path);
		strcat(url, "/");
	}
}


int url_check(Url *url)
{
    int invalid = 0;

    invalid = invalid || url->scheme == NULL || strcmp(url->scheme, "http");
    invalid = invalid || url->hostname == NULL;

    return invalid;
}

void url_free(Url *url)
{
    url_free_part(url->scheme);
    url_free_part(url->hostname);
    url_free_part(url->port);
    url_free_part(url->path);
    free(url);
}
