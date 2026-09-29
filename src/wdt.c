/* SPDX-License-Identifier: MIT */

#include "wdt.h"
#include "adt.h"
#include "types.h"
#include "utils.h"

#define WDT_COUNT 0x10
#define WDT_ALARM 0x14
#define WDT_CTL   0x1c

#define WDT_CLOCK 24000000

static u64 wdt_base = 0;
static bool wdt_armed = false;

void wdt_disable(void)
{
    int path[8];
    int node = adt_path_offset_trace(adt, "/arm-io/wdt", path);

    if (node < 0) {
        printf("WDT node not found!\n");
        return;
    }

    if (adt_get_reg(adt, path, "reg", 0, &wdt_base, NULL)) {
        printf("Failed to get WDT reg property!\n");
        return;
    }

    printf("Primary WDT register @ 0x%lx\n", wdt_base);
    write32(wdt_base + WDT_CTL, 0);
    wdt_armed = false;
    printf("Primary WDT disabled\n");

    // disable secondary watchdog if wdt-version is 2 or 3
    u32 wdt_version;
    if (ADT_GETPROP(adt, node, "wdt-version", &wdt_version) < 0)
        return;

    if (wdt_version == 2 || wdt_version == 3) {
        u64 wdt_2nd = 0;
        if (adt_get_reg(adt, path, "reg", 2, &wdt_2nd, NULL)) {
            printf("Failed to get WDT reg[2] property!\n");
            return;
        }

        printf("Secondary WDT register @ 0x%lx\n", wdt_2nd);
        write32(wdt_2nd, 0);
        printf("Secondary WDT disabled\n");
    }
}

void wdt_arm(u32 seconds)
{
    if (!wdt_base)
        return;

    if (!seconds) {
        write32(wdt_base + WDT_CTL, 0);
        wdt_armed = false;
        printf("Primary WDT disabled\n");
        return;
    }

    u64 ticks = min((u64)seconds * WDT_CLOCK, 0xffffffffUL);

    write32(wdt_base + WDT_ALARM, ticks);
    write32(wdt_base + WDT_COUNT, 0);
    write32(wdt_base + WDT_CTL, 4);
    wdt_armed = true;
    printf("Primary WDT armed: reset after %lu s without a kick\n", ticks / WDT_CLOCK);
}

void wdt_kick(void)
{
    if (wdt_armed)
        write32(wdt_base + WDT_COUNT, 0);
}

void wdt_reboot(void)
{
    if (!wdt_base)
        return;

    write32(wdt_base + WDT_ALARM, 0x100000);
    write32(wdt_base + WDT_COUNT, 0);
    write32(wdt_base + WDT_CTL, 4);
}
