#ifndef __USR_DHCPD_H_H
#define __USR_DHCPD_H_H
void udhcpd_start(void * arg);
int udhcpd_stop(void * arg);
void dhcp_server_start(const uint8_t *arg);

#endif
