#ifndef _BBC_URL_H
#define _BBC_URL_H

#define url_free_part(P) if(P) { free(P); }

typedef struct Url {
    char *scheme;
    char *hostname;
    char *port;
    char *path;
} Url;

Url *url_parse(char *url);
void url_free(Url *url);


#endif
