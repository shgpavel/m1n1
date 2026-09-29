/* SPDX-License-Identifier: MIT */

#ifndef NETPROXY_H
#define NETPROXY_H

#include "types.h"

int netproxy_start(const char *spec);
bool netproxy_active(void);
void netproxy_vuart_setup(void);
void netproxy_shutdown(void);

#endif
