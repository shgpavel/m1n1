/* SPDX-License-Identifier: MIT */

#ifndef NETPROXY_H
#define NETPROXY_H

#include "types.h"

int netproxy_start(const char *spec);
bool netproxy_active(void);
bool netproxy_wait_host(u32 seconds);
void netproxy_vuart_setup(void);
void netproxy_shutdown(void);

#endif
