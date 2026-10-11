/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef PCIE_H
#define PCIE_H

#include "types.h"

int pcie_init(void);
int pcie_link_up(u32 port, u32 timeout_ms);
int pcie_shutdown(void);

#endif
