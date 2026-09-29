/* SPDX-License-Identifier: MIT */

#ifndef __WDT_H__
#define __WDT_H__

#include "types.h"

void wdt_disable(void);
void wdt_arm(u32 seconds);
void wdt_kick(void);
void wdt_reboot(void);

#endif
