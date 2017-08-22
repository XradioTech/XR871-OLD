/*******************************************************************************
 * Copyright (c) 2014, 2015 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 * 
 *******************************************************************************/

#ifndef __MQTT_XR_RTOS__
#define __MQTT_XR_RTOS__

#include "kernel/os/os.h"

#include "net/mbedtls/net.h"
#include "net/mbedtls/ssl.h"
#include "net/mbedtls/certs.h"

typedef struct Timer Timer;

struct Timer 
{
	unsigned int end_time;
};

void InitTimer(Timer*);
char expired(Timer*);
void countdown_ms(Timer*, unsigned int);
void countdown(Timer*, unsigned int);
int left_ms(Timer*);

typedef struct Network Network;

/*
struct Network
{
	int my_socket;
	int (*mqttread) (Network*, unsigned char*, int, int);
	int (*mqttwrite) (Network*, unsigned char*, int, int);
	void (*disconnect) (Network*);
};
*/

struct Network {
	int my_socket;                                                
	int (*mqttread)(Network *, unsigned char *, int, int);      
	int (*mqttwrite)(Network *, unsigned char *, int, int);      
	void (*disconnect)(Network *);   
	
	mbedtls_ssl_context ssl;         
	mbedtls_net_context fd;          
	mbedtls_ssl_config conf;         
	mbedtls_x509_crt cacertl;        
	mbedtls_x509_crt clicert;        
	mbedtls_pk_context pkey;          
};

void NewNetwork(Network*);
int ConnectNetwork(Network*, char*, int);

int mqtt_ssl_establish(Network *n, const char *addr, const char *port, const char *ca_crt, size_t ca_crt_len) ;

#endif
