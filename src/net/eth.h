/* SPDX-License-Identifier: MIT */

#ifndef ETH_H
#define ETH_H

#include "types.h"

int eth_init(void);
void eth_shutdown(void);
bool eth_link_poll(void);
const u8 *eth_mac(void);
void *eth_nic(void);
void eth_dump(void);

#endif
