/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "dptx_phy.h"
#include "malloc.h"

#include "../adt.h"
#include "../utils.h"

#define DPTX_MAX_LANES    4
#define DPTX_LANE0_OFFSET 0x5000
#define DPTX_LANE_STRIDE  0x1000
#define DPTX_LANE_END     (DPTX_LANE0_OFFSET + DPTX_MAX_LANES * DPTX_LANE_STRIDE)

enum dptx_type {
    DPTX_PHY_T8112,
    DPTX_PHY_T602X,
    DPTX_PHY_T8122,
};

typedef struct dptx_phy {
    u64 regs[2];
    enum dptx_type type;
    u32 dcp_index;
    u32 active_lanes;
    u32 dfp_endpoint;
    u32 link_rate;
    bool active;
} dptx_phy_t;

struct t8122_pll_cfg {
    u32 code;
    u32 f20b0;
    u32 f20b4;
    u32 f20b8;
    bool bypass;
    bool half;
};

static const struct t8122_pll_cfg t8122_pll_cfgs[] = {
    {0x06, 0x21c, 0x0000000, 0x156600, true, false},
    {0x0a, 0x1c2, 0x7fffffe, 0x155200, true, true},
    {0x14, 0x1c2, 0x7fffffe, 0x554800, true, true},
    {0x1e, 0x2a3, 0xbff7ffc, 0x564800, false, true},
};

static void t8122_field(dptx_phy_t *phy, u32 off, u32 field, u32 val)
{
    mask32(phy->regs[0] + off, field, FIELD_PREP(field, val));
}

static void t8122_bit(dptx_phy_t *phy, u32 off, u32 bit, bool val)
{
    mask32(phy->regs[0] + off, BIT(bit), val ? BIT(bit) : 0);
}

static void t8122_enable_device_power(dptx_phy_t *phy)
{
    static const u32 steps[][2] = {{0x10, 0x20}, {0x100, 0x200}, {0x1, 0x2}};

    mask32(phy->regs[1] + 0x10, GENMASK(3, 0), FIELD_PREP(GENMASK(3, 0), phy->dcp_index));

    for (size_t i = 0; i < ARRAY_SIZE(steps); i++) {
        set32(phy->regs[1] + 0x48, steps[i][0]);
        udelay(1);
        set32(phy->regs[1] + 0x48, steps[i][1]);
        udelay(1);
        clear32(phy->regs[1] + 0x48, steps[i][1] << 1);
        udelay(1);
    }
}

static void t8122_impedance_calibration(dptx_phy_t *phy)
{
    t8122_bit(phy, 0x4224, 0, false);
    mdelay(1);
    t8122_bit(phy, 0x4224, 0, true);
    udelay(1);
    t8122_field(phy, 0x4200, GENMASK(9, 4), 0x3c);
    udelay(1);
    t8122_bit(phy, 0x4200, 0, true);
    udelay(125);
    u32 status = read32(phy->regs[0] + 0x4204);
    udelay(10);
    t8122_bit(phy, 0x4200, 0, false);

    u32 code = FIELD_GET(GENMASK(24, 18), read32(phy->regs[0] + 0x4210));
    t8122_field(phy, 0x4228, GENMASK(22, 16), code);
    t8122_bit(phy, 0x4228, 15, true);

    if (!(status & 1))
        printf("DPTX-PHY: impedance calibration failed: status 0x%x code 0x%x\n", status, code);
    else
        dprintf("DPTX-PHY: impedance calibration status 0x%x code 0x%x\n", status, code);
}

static int t8122_set_active_lane_count(dptx_phy_t *phy, u32 num_lanes)
{
    if (num_lanes > DPTX_MAX_LANES)
        return -1;

    phy->active_lanes = num_lanes;
    if (!phy->active)
        return 0;

    t8122_bit(phy, 0x4000, 26, true);
    for (u32 l = 0; l < DPTX_MAX_LANES; l++)
        t8122_bit(phy, DPTX_LANE0_OFFSET + DPTX_LANE_STRIDE * l, 9, l >= num_lanes);
    udelay(1);
    for (u32 l = 0; l < DPTX_MAX_LANES; l++)
        t8122_bit(phy, DPTX_LANE0_OFFSET + DPTX_LANE_STRIDE * l, 8, l >= num_lanes);
    if (num_lanes > 0)
        t8122_bit(phy, 0x4000, 26, false);

    return 0;
}

static int t8122_activate(dptx_phy_t *phy)
{
    t8122_enable_device_power(phy);

    t8122_field(phy, 0x2014, GENMASK(23, 16), 0x30);
    t8122_field(phy, 0x20b8, GENMASK(19, 16), 5);
    t8122_field(phy, 0x2220, GENMASK(2, 0), 0);
    t8122_bit(phy, 0x222c, 11, true);
    t8122_bit(phy, 0x222c, 8, true);
    t8122_field(phy, 0x2230, GENMASK(21, 17), 0x10);
    t8122_bit(phy, 0x2278, 27, false);
    t8122_bit(phy, 0x22a4, 0, true);

    for (u32 loff = DPTX_LANE0_OFFSET; loff < DPTX_LANE_END; loff += DPTX_LANE_STRIDE) {
        t8122_field(phy, loff + 0x40, GENMASK(23, 20), 2);
        t8122_bit(phy, loff + 0x40, 19, true);
        t8122_bit(phy, loff + 0x244, 4, false);
    }

    t8122_bit(phy, 0x2218, 0, true);
    t8122_field(phy, 0x2234, GENMASK(1, 0), 0);
    t8122_bit(phy, 0x2200, 1, true);
    t8122_bit(phy, 0x1000, 1, false);
    t8122_bit(phy, 0x4004, 3, true);
    udelay(1);

    phy->active = true;
    t8122_set_active_lane_count(phy, phy->active_lanes);

    t8122_bit(phy, 0xd000, 0, false);
    t8122_impedance_calibration(phy);

    return 0;
}

static void t8122_phy_power_down(dptx_phy_t *phy)
{
    t8122_bit(phy, 0x4004, 0, true);
    t8122_bit(phy, 0x4000, 6, false);
    t8122_bit(phy, 0x4004, 3, false);
    t8122_bit(phy, 0x4000, 25, false);
    t8122_bit(phy, 0x4000, 24, false);
}

static void t8122_phy_power_up(dptx_phy_t *phy)
{
    for (u32 loff = DPTX_LANE0_OFFSET; loff < DPTX_LANE_END; loff += DPTX_LANE_STRIDE)
        t8122_field(phy, loff + 0x10, GENMASK(29, 27), 3);
    t8122_bit(phy, 0x4000, 24, true);
    udelay(1);
    t8122_bit(phy, 0x4000, 25, true);
    udelay(1);
    t8122_bit(phy, 0x4004, 3, true);
    t8122_bit(phy, 0x4000, 6, true);
    t8122_bit(phy, 0x4004, 0, false);
}

static void t8122_pll_power_down(dptx_phy_t *phy)
{
    if (!(read32(phy->regs[0] + 0x2200) & BIT(1)))
        return;

    t8122_bit(phy, 0x2200, 1, false);
    t8122_field(phy, 0x100c, GENMASK(2, 0), 0);
    t8122_bit(phy, 0x100c, 3, true);
    udelay(100);
    u32 ack = read32(phy->regs[0] + 0x1014) & 1;
    t8122_bit(phy, 0x100c, 3, false);

    if (!ack)
        printf("DPTX-PHY: no ack to PLL power down: 0x1008 = 0x%x\n",
               read32(phy->regs[0] + 0x1008));
}

static void t8122_pll_power_up(dptx_phy_t *phy)
{
    t8122_field(phy, 0x100c, GENMASK(2, 0), 7);
    t8122_bit(phy, 0x100c, 3, true);
    udelay(100);
    u32 ack = read32(phy->regs[0] + 0x1014) & 1;
    t8122_bit(phy, 0x100c, 3, false);
    u32 lock = read32(phy->regs[0] + 0x1008) & BIT(3);
    t8122_bit(phy, 0x2200, 1, true);

    if (!ack)
        printf("DPTX-PHY: no ack to PLL power up\n");
    if (!lock)
        printf("DPTX-PHY: PLL not locked\n");
}

static void t8122_shadow_update(dptx_phy_t *phy)
{
    set32(phy->regs[1] + 0xa0, 0x8);
    set32(phy->regs[1] + 0xa0, 0x4);
    set32(phy->regs[1] + 0xa0, 0x40000);
    clear32(phy->regs[1] + 0xa0, 0x40000);
    set32(phy->regs[1] + 0xa0, 0x80000);
    clear32(phy->regs[1] + 0xa0, 0x80000);
    clear32(phy->regs[1] + 0xa0, 0x4);
    clear32(phy->regs[1] + 0xa0, 0x8);
}

static int t8122_set_link_rate(dptx_phy_t *phy, u32 link_rate)
{
    const struct t8122_pll_cfg *cfg = NULL;
    u32 code = link_rate / 270;

    if (code) {
        for (size_t i = 0; i < ARRAY_SIZE(t8122_pll_cfgs); i++) {
            if (t8122_pll_cfgs[i].code == code)
                cfg = &t8122_pll_cfgs[i];
        }
        if (!cfg) {
            printf("DPTX-PHY: unsupported link rate %u\n", link_rate);
            return -1;
        }
    }

    if (code == phy->link_rate)
        return 0;

    t8122_phy_power_down(phy);
    t8122_pll_power_down(phy);

    if (cfg) {
        t8122_field(phy, 0x20b0, GENMASK(9, 0), cfg->f20b0);
        t8122_field(phy, 0x20b4, GENMASK(27, 0), cfg->f20b4);
        mask32(phy->regs[0] + 0x20b8, 0x7ffe00, cfg->f20b8);
        t8122_bit(phy, 0x222c, 1, cfg->bypass);
        t8122_shadow_update(phy);
        t8122_bit(phy, 0x2000, 29, !cfg->half);
        t8122_bit(phy, 0x201c, 5, !cfg->half);
        t8122_pll_power_up(phy);
        t8122_phy_power_up(phy);
    }

    phy->link_rate = code;

    return 0;
}

int dptx_phy_activate(dptx_phy_t *phy)
{
    if (phy->type == DPTX_PHY_T8122)
        return t8122_activate(phy);

    // MMIO: R.4   0x23c500010 (dptx-phy[1], offset 0x10) = 0x0
    // MMIO: W.4   0x23c500010 (dptx-phy[1], offset 0x10) = 0x0
    read32(phy->regs[1] + 0x10);
    write32(phy->regs[1] + 0x10, phy->dcp_index);

    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x444
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x454
    set32(phy->regs[1] + 0x48, 0x010);
    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x454
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x474
    set32(phy->regs[1] + 0x48, 0x020);
    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x474
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x434
    clear32(phy->regs[1] + 0x48, 0x040);
    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x434
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x534
    set32(phy->regs[1] + 0x48, 0x100);
    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x534
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x734
    set32(phy->regs[1] + 0x48, 0x200);
    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x734
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x334
    clear32(phy->regs[1] + 0x48, 0x400);
    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x334
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x335
    set32(phy->regs[1] + 0x48, 0x001);
    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x335
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x337
    set32(phy->regs[1] + 0x48, 0x002);
    // MMIO: R.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x337
    // MMIO: W.4   0x23c500048 (dptx-phy[1], offset 0x48) = 0x333
    clear32(phy->regs[1] + 0x48, 0x004);

    // MMIO: R.4   0x23c542014 (dptx-phy[0], offset 0x2014) = 0x80a0c
    u32 val_2014 = read32(phy->regs[0] + 0x2014);
    // MMIO: W.4   0x23c542014 (dptx-phy[0], offset 0x2014) = 0x300a0c
    write32(phy->regs[0] + 0x2014, (0x30 << 16) | (val_2014 & 0xffff));

    // MMIO: R.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x644800
    // MMIO: W.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x654800
    set32(phy->regs[0] + 0x20b8, 0x010000);

    // MMIO: R.4   0x23c542220 (dptx-phy[0], offset 0x2220) = 0x11090a2
    // MMIO: W.4   0x23c542220 (dptx-phy[0], offset 0x2220) = 0x11090a0
    clear32(phy->regs[0] + 0x2220, 0x0000002);

    // MMIO: R.4   0x23c54222c (dptx-phy[0], offset 0x222c) = 0x103003
    // MMIO: W.4   0x23c54222c (dptx-phy[0], offset 0x222c) = 0x103803
    set32(phy->regs[0] + 0x222c, 0x000800);
    // MMIO: R.4   0x23c54222c (dptx-phy[0], offset 0x222c) = 0x103803
    // MMIO: W.4   0x23c54222c (dptx-phy[0], offset 0x222c) = 0x103903
    set32(phy->regs[0] + 0x222c, 0x000100);

    // MMIO: R.4   0x23c542230 (dptx-phy[0], offset 0x2230) = 0x2308804
    // MMIO: W.4   0x23c542230 (dptx-phy[0], offset 0x2230) = 0x2208804
    clear32(phy->regs[0] + 0x2230, 0x0100000);

    // MMIO: R.4   0x23c542278 (dptx-phy[0], offset 0x2278) = 0x18300811
    // MMIO: W.4   0x23c542278 (dptx-phy[0], offset 0x2278) = 0x10300811
    clear32(phy->regs[0] + 0x2278, 0x08000000);

    // MMIO: R.4   0x23c5422a4 (dptx-phy[0], offset 0x22a4) = 0x1044200
    // MMIO: W.4   0x23c5422a4 (dptx-phy[0], offset 0x22a4) = 0x1044201
    set32(phy->regs[0] + 0x22a4, 0x0000001);

    // MMIO: R.4   0x23c544008 (dptx-phy[0], offset 0x4008) = 0x18030
    u32 val_4008 = read32(phy->regs[0] + 0x4008);
    // MMIO: W.4   0x23c544008 (dptx-phy[0], offset 0x4008) = 0x30030
    write32(phy->regs[0] + 0x4008, (0x6 << 15) | (val_4008 & 0x7fff));
    // MMIO: R.4   0x23c544008 (dptx-phy[0], offset 0x4008) = 0x30030
    // MMIO: W.4   0x23c544008 (dptx-phy[0], offset 0x4008) = 0x30010
    clear32(phy->regs[0] + 0x4008, 0x00020);

    // MMIO: R.4   0x23c54420c (dptx-phy[0], offset 0x420c) = 0x88e3
    // MMIO: W.4   0x23c54420c (dptx-phy[0], offset 0x420c) = 0x88c3
    clear32(phy->regs[0] + 0x420c, 0x0020);

    // MMIO: R.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x0
    // MMIO: W.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000000
    set32(phy->regs[0] + 0x4600, 0x8000000);

    // MMIO: R.4   0x23c545040 (dptx-phy[0], offset 0x5040) = 0x21780
    // MMIO: W.4   0x23c545040 (dptx-phy[0], offset 0x5040) = 0x221780
    // MMIO: R.4   0x23c546040 (dptx-phy[0], offset 0x6040) = 0x21780
    // MMIO: W.4   0x23c546040 (dptx-phy[0], offset 0x6040) = 0x221780
    // MMIO: R.4   0x23c547040 (dptx-phy[0], offset 0x7040) = 0x21780
    // MMIO: W.4   0x23c547040 (dptx-phy[0], offset 0x7040) = 0x221780
    // MMIO: R.4   0x23c548040 (dptx-phy[0], offset 0x8040) = 0x21780
    // MMIO: W.4   0x23c548040 (dptx-phy[0], offset 0x8040) = 0x221780
    for (u32 loff = DPTX_LANE0_OFFSET; loff < DPTX_LANE_END; loff += DPTX_LANE_STRIDE)
        set32(phy->regs[0] + loff + 0x40, 0x200000);

    // MMIO: R.4   0x23c545040 (dptx-phy[0], offset 0x5040) = 0x221780
    // MMIO: W.4   0x23c545040 (dptx-phy[0], offset 0x5040) = 0x2a1780
    // MMIO: R.4   0x23c546040 (dptx-phy[0], offset 0x6040) = 0x221780
    // MMIO: W.4   0x23c546040 (dptx-phy[0], offset 0x6040) = 0x2a1780
    // MMIO: R.4   0x23c547040 (dptx-phy[0], offset 0x7040) = 0x221780
    // MMIO: W.4   0x23c547040 (dptx-phy[0], offset 0x7040) = 0x2a1780
    // MMIO: R.4   0x23c548040 (dptx-phy[0], offset 0x8040) = 0x221780
    // MMIO: W.4   0x23c548040 (dptx-phy[0], offset 0x8040) = 0x2a1780
    for (u32 loff = DPTX_LANE0_OFFSET; loff < DPTX_LANE_END; loff += DPTX_LANE_STRIDE)
        set32(phy->regs[0] + loff + 0x40, 0x080000);

    // MMIO: R.4   0x23c545244 (dptx-phy[0], offset 0x5244) = 0x18
    // MMIO: W.4   0x23c545244 (dptx-phy[0], offset 0x5244) = 0x8
    // MMIO: R.4   0x23c546244 (dptx-phy[0], offset 0x6244) = 0x18
    // MMIO: W.4   0x23c546244 (dptx-phy[0], offset 0x6244) = 0x8
    // MMIO: R.4   0x23c547244 (dptx-phy[0], offset 0x7244) = 0x18
    // MMIO: W.4   0x23c547244 (dptx-phy[0], offset 0x7244) = 0x8
    // MMIO: R.4   0x23c548244 (dptx-phy[0], offset 0x8244) = 0x18
    // MMIO: W.4   0x23c548244 (dptx-phy[0], offset 0x8244) = 0x8
    for (u32 loff = DPTX_LANE0_OFFSET; loff < DPTX_LANE_END; loff += DPTX_LANE_STRIDE)
        clear32(phy->regs[0] + loff + 0x244, 0x10);

    // MMIO: R.4   0x23c542214 (dptx-phy[0], offset 0x2214) = 0x1e0
    // MMIO: W.4   0x23c542214 (dptx-phy[0], offset 0x2214) = 0x1e1
    set32(phy->regs[0] + 0x2214, 0x001);

    // MMIO: R.4   0x23c542224 (dptx-phy[0], offset 0x2224) = 0x20086001
    // MMIO: W.4   0x23c542224 (dptx-phy[0], offset 0x2224) = 0x20086000
    clear32(phy->regs[0] + 0x2224, 0x00000001);

    // MMIO: R.4   0x23c542200 (dptx-phy[0], offset 0x2200) = 0x2000
    // MMIO: W.4   0x23c542200 (dptx-phy[0], offset 0x2200) = 0x2002
    set32(phy->regs[0] + 0x2200, 0x0002);

    // MMIO: R.4   0x23c541000 (dptx-phy[0], offset 0x1000) = 0xe0000003
    // MMIO: W.4   0x23c541000 (dptx-phy[0], offset 0x1000) = 0xe0000001
    clear32(phy->regs[0] + 0x1000, 0x00000002);

    // MMIO: R.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x41
    // MMIO: W.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x49
    set32(phy->regs[0] + 0x4004, 0x08);

    /* TODO: no idea what happens here, supposedly setting/clearing some bits */
    // MMIO: R.4   0x23c544404 (dptx-phy[0], offset 0x4404) = 0x555d444
    read32(phy->regs[0] + 0x4404);
    // MMIO: W.4   0x23c544404 (dptx-phy[0], offset 0x4404) = 0x555d444
    write32(phy->regs[0] + 0x4404, 0x555d444);
    // MMIO: R.4   0x23c544404 (dptx-phy[0], offset 0x4404) = 0x555d444
    read32(phy->regs[0] + 0x4404);
    // MMIO: W.4   0x23c544404 (dptx-phy[0], offset 0x4404) = 0x555d444
    write32(phy->regs[0] + 0x4404, 0x555d444);

    dptx_phy_set_active_lane_count(phy, 0);

    // MMIO: R.4   0x23c544200 (dptx-phy[0], offset 0x4200) = 0x4002430
    // MMIO: W.4   0x23c544200 (dptx-phy[0], offset 0x4200) = 0x4002420
    clear32(phy->regs[0] + 0x4200, 0x0000010);

    // MMIO: R.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000000
    // MMIO: W.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000000
    clear32(phy->regs[0] + 0x4600, 0x0000001);
    // MMIO: R.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000000
    // MMIO: W.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000001
    set32(phy->regs[0] + 0x4600, 0x0000001);
    // MMIO: R.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000001
    // MMIO: W.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000003
    set32(phy->regs[0] + 0x4600, 0x0000002);
    // MMIO: R.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000043
    // MMIO: R.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000043
    // MMIO: W.4   0x23c544600 (dptx-phy[0], offset 0x4600) = 0x8000041
    /* TODO: read first to check if the previous set(...,0x2) sticked? */
    read32(phy->regs[0] + 0x4600);
    clear32(phy->regs[0] + 0x4600, 0x0000001);

    // MMIO: R.4   0x23c544408 (dptx-phy[0], offset 0x4408) = 0x482
    // MMIO: W.4   0x23c544408 (dptx-phy[0], offset 0x4408) = 0x482
    /* TODO: probably a set32 of an already set bit */
    u32 val_4408 = read32(phy->regs[0] + 0x4408);
    if (val_4408 != 0x482 && val_4408 != 0x483)
        printf("DPTX-PHY: unexpected initial value at regs[0] offset 0x4408: 0x%03x\n", val_4408);
    write32(phy->regs[0] + 0x4408, val_4408);
    // MMIO: R.4   0x23c544408 (dptx-phy[0], offset 0x4408) = 0x482
    // MMIO: W.4   0x23c544408 (dptx-phy[0], offset 0x4408) = 0x483
    set32(phy->regs[0] + 0x4408, 0x001);

    return 0;
}

int dptx_phy_set_active_lane_count(dptx_phy_t *phy, u32 num_lanes)
{
    u32 l;

    dprintf("DPTX-PHY: set_active_lane_count(%u) phy_regs = {0x%lx, 0x%lx}\n", num_lanes,
            phy->regs[0], phy->regs[1]);

    if (phy->type == DPTX_PHY_T8122)
        return t8122_set_active_lane_count(phy, num_lanes);

    if (num_lanes == 3 || num_lanes > DPTX_MAX_LANES)
        return -1;

    u32 ctrl = read32(phy->regs[0] + 0x4000);
    write32(phy->regs[0] + 0x4000, ctrl);

    for (l = 0; l < num_lanes; l++) {
        u64 offset = 0x5000 + 0x1000 * l;
        read32(phy->regs[0] + offset);
        write32(phy->regs[0] + offset, 0x100);
    }
    for (; l < DPTX_MAX_LANES; l++) {
        u64 offset = 0x5000 + 0x1000 * l;
        read32(phy->regs[0] + offset);
        write32(phy->regs[0] + offset, 0x300);
    }
    for (l = 0; l < num_lanes; l++) {
        u64 offset = 0x5000 + 0x1000 * l;
        read32(phy->regs[0] + offset);
        write32(phy->regs[0] + offset, 0x0);
    }
    for (; l < DPTX_MAX_LANES; l++) {
        u64 offset = 0x5000 + 0x1000 * l;
        read32(phy->regs[0] + offset);
        write32(phy->regs[0] + offset, 0x300);
    }

    if (num_lanes > 0) {
        // clear32(phy->regs[0] + 0x4000, 0x4000000);
        ctrl = read32(phy->regs[0] + 0x4000);
        ctrl &= ~0x4000000;
        write32(phy->regs[0] + 0x4000, ctrl);
    }
    phy->active_lanes = num_lanes;

    return 0;
}

int dptx_phy_set_link_rate(dptx_phy_t *phy, u32 link_rate)
{
    if (phy->type == DPTX_PHY_T8122)
        return t8122_set_link_rate(phy, link_rate);

    UNUSED(link_rate);
    u32 sts_1008, sts_1014;

    // MMIO: R.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x49
    // MMIO: W.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x49
    set32(phy->regs[0] + 0x4004, 0x08);

    // MMIO: R.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x41021ac
    // MMIO: W.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x41021ac
    clear32(phy->regs[0] + 0x4000, 0x0000040);

    // MMIO: R.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x49
    // MMIO: W.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x41
    clear32(phy->regs[0] + 0x4004, 0x08);

    // MMIO: R.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x41021ac
    // MMIO: W.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x41021ac
    clear32(phy->regs[0] + 0x4000, 0x2000000);
    // MMIO: R.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x41021ac
    // MMIO: W.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x41021ac
    set32(phy->regs[0] + 0x4000, 0x1000000);

    // MMIO: R.4   0x23c542200 (dptx-phy[0], offset 0x2200) = 0x2002
    // MMIO: R.4   0x23c542200 (dptx-phy[0], offset 0x2200) = 0x2002
    // MMIO: W.4   0x23c542200 (dptx-phy[0], offset 0x2200) = 0x2000
    /* TODO: what is this read checking for? */
    read32(phy->regs[0] + 0x2200);
    clear32(phy->regs[0] + 0x2200, 0x0002);

    // MMIO: R.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf000
    // MMIO: W.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf000
    // MMIO: R.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf000
    // MMIO: W.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf008
    /* TODO: what is the setting/clearing? */
    u32 val_100c = read32(phy->regs[0] + 0x100c);
    write32(phy->regs[0] + 0x100c, val_100c);
    set32(phy->regs[0] + 0x100c, 0x0008);

    // MMIO: R.4   0x23c541014 (dptx-phy[0], offset 0x1014) = 0x1
    sts_1014 = read32(phy->regs[0] + 0x1014);
    UNUSED(sts_1014); /* TODO: assert(sts_1014 == 0x1); */

    // MMIO: R.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf008
    // MMIO: W.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf000
    clear32(phy->regs[0] + 0x100c, 0x0008);

    // MMIO: R.4   0x23c541008 (dptx-phy[0], offset 0x1008) = 0x1
    sts_1008 = read32(phy->regs[0] + 0x1008);
    UNUSED(sts_1008); /* TODO: assert(sts_1008 == 0x1); */

    // MMIO: R.4   0x23c542220 (dptx-phy[0], offset 0x2220) = 0x11090a0
    // MMIO: W.4   0x23c542220 (dptx-phy[0], offset 0x2220) = 0x1109020
    clear32(phy->regs[0] + 0x2220, 0x0000080);

    // MMIO: R.4   0x23c5420b0 (dptx-phy[0], offset 0x20b0) = 0x1e0e01c2
    // MMIO: W.4   0x23c5420b0 (dptx-phy[0], offset 0x20b0) = 0x1e0e01c2
    u32 val_20b0 = read32(phy->regs[0] + 0x20b0);
    /* TODO: what happens on dptx-phy */
    if (phy->type == DPTX_PHY_T602X)
        val_20b0 = (val_20b0 & ~0x3ff) | 0x2a3;
    write32(phy->regs[0] + 0x20b0, val_20b0);

    // MMIO: R.4   0x23c5420b4 (dptx-phy[0], offset 0x20b4) = 0x7fffffe
    // MMIO: W.4   0x23c5420b4 (dptx-phy[0], offset 0x20b4) = 0x7fffffe
    u32 val_20b4 = read32(phy->regs[0] + 0x20b4);
    /* TODO: what happens on dptx-phy */
    if (phy->type == DPTX_PHY_T602X)
        val_20b4 = (val_20b4 | 0x4000000) & ~0x0008000;
    write32(phy->regs[0] + 0x20b4, val_20b4);

    // MMIO: R.4   0x23c5420b4 (dptx-phy[0], offset 0x20b4) = 0x7fffffe
    // MMIO: W.4   0x23c5420b4 (dptx-phy[0], offset 0x20b4) = 0x7fffffe
    val_20b4 = read32(phy->regs[0] + 0x20b4);
    /* TODO: what happens on dptx-phy */
    if (phy->type == DPTX_PHY_T602X)
        val_20b4 = (val_20b4 | 0x0000001) & ~0x0000004;
    write32(phy->regs[0] + 0x20b4, val_20b4);

    // MMIO: R.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x654800
    // MMIO: W.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x654800
    /* TODO: unclear */
    set32(phy->regs[0] + 0x20b8, 0);
    // MMIO: R.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x654800
    // MMIO: W.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x654800
    /* TODO: unclear */
    set32(phy->regs[0] + 0x20b8, 0);
    // MMIO: R.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x654800
    // MMIO: W.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x654800
    /* TODO: unclear */
    if (phy->type == DPTX_PHY_T602X)
        set32(phy->regs[0] + 0x20b8, 0x010000);
    else
        set32(phy->regs[0] + 0x20b8, 0);
    // MMIO: R.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x654800
    // MMIO: W.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x454800
    clear32(phy->regs[0] + 0x20b8, 0x200000);

    // MMIO: R.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x454800
    // MMIO: W.4   0x23c5420b8 (dptx-phy[0], offset 0x20b8) = 0x454800
    /* TODO: unclear */
    set32(phy->regs[0] + 0x20b8, 0);

    // MMIO: R.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x0
    // MMIO: W.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x8
    // MMIO: R.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x8
    // MMIO: W.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0xc
    // MMIO: R.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0xc
    // MMIO: W.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x4000c
    // MMIO: R.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x4000c
    // MMIO: W.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0xc
    // MMIO: R.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0xc
    // MMIO: W.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x8000c
    // MMIO: R.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x8000c
    // MMIO: W.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0xc
    // MMIO: R.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0xc
    // MMIO: W.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x8
    // MMIO: R.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x8
    // MMIO: W.4   0x23c5000a0 (dptx-phy[1], offset 0xa0) = 0x0
    set32(phy->regs[1] + 0xa0, 0x8);
    set32(phy->regs[1] + 0xa0, 0x4);
    set32(phy->regs[1] + 0xa0, 0x40000);
    clear32(phy->regs[1] + 0xa0, 0x40000);
    set32(phy->regs[1] + 0xa0, 0x80000);
    clear32(phy->regs[1] + 0xa0, 0x80000);
    clear32(phy->regs[1] + 0xa0, 0x4);
    clear32(phy->regs[1] + 0xa0, 0x8);

    // MMIO: R.4   0x23c542000 (dptx-phy[0], offset 0x2000) = 0x2
    // MMIO: W.4   0x23c542000 (dptx-phy[0], offset 0x2000) = 0x2
    /* TODO: unclear */
    set32(phy->regs[0] + 0x2000, 0x0);

    // MMIO: R.4   0x23c542018 (dptx-phy[0], offset 0x2018) = 0x0
    // MMIO: W.4   0x23c542018 (dptx-phy[0], offset 0x2018) = 0x0
    clear32(phy->regs[0] + 0x2018, 0x0);

    // MMIO: R.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf000
    // MMIO: W.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf007
    set32(phy->regs[0] + 0x100c, 0x0007);
    // MMIO: R.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf007
    // MMIO: W.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf00f
    set32(phy->regs[0] + 0x100c, 0x0008);

    // MMIO: R.4   0x23c541014 (dptx-phy[0], offset 0x1014) = 0x38f
    sts_1014 = read32(phy->regs[0] + 0x1014);
    /* TODO: assert(sts_1014 == 0x38f); */

    // MMIO: R.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf00f
    // MMIO: W.4   0x23c54100c (dptx-phy[0], offset 0x100c) = 0xf007
    clear32(phy->regs[0] + 0x100c, 0x0008);

    // MMIO: R.4   0x23c541008 (dptx-phy[0], offset 0x1008) = 0x9
    sts_1008 = read32(phy->regs[0] + 0x1008);
    /* TODO: assert(sts_1008 == 0x9); */

    // MMIO: R.4   0x23c542200 (dptx-phy[0], offset 0x2200) = 0x2000
    // MMIO: W.4   0x23c542200 (dptx-phy[0], offset 0x2200) = 0x2002
    set32(phy->regs[0] + 0x2200, 0x0002);

    // MMIO: R.4   0x23c545010 (dptx-phy[0], offset 0x5010) = 0x18003000
    // MMIO: W.4   0x23c545010 (dptx-phy[0], offset 0x5010) = 0x18003000
    // MMIO: R.4   0x23c546010 (dptx-phy[0], offset 0x6010) = 0x18003000
    // MMIO: W.4   0x23c546010 (dptx-phy[0], offset 0x6010) = 0x18003000
    // MMIO: R.4   0x23c547010 (dptx-phy[0], offset 0x7010) = 0x18003000
    // MMIO: W.4   0x23c547010 (dptx-phy[0], offset 0x7010) = 0x18003000
    // MMIO: R.4   0x23c548010 (dptx-phy[0], offset 0x8010) = 0x18003000
    // MMIO: W.4   0x23c548010 (dptx-phy[0], offset 0x8010) = 0x18003000
    write32(phy->regs[0] + 0x8010, 0x18003000);
    for (u32 loff = DPTX_LANE0_OFFSET; loff < DPTX_LANE_END; loff += DPTX_LANE_STRIDE) {
        u32 val_l010 = read32(phy->regs[0] + loff + 0x10);
        write32(phy->regs[0] + loff + 0x10, val_l010);
    }

    // MMIO: R.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x41021ac
    // MMIO: W.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x51021ac
    set32(phy->regs[0] + 0x4000, 0x1000000);
    // MMIO: R.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x51021ac
    // MMIO: W.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x71021ac
    set32(phy->regs[0] + 0x4000, 0x2000000);

    // MMIO: R.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x41
    // MMIO: W.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x49
    set32(phy->regs[0] + 0x4004, 0x08);

    // MMIO: R.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x71021ac
    // MMIO: W.4   0x23c544000 (dptx-phy[0], offset 0x4000) = 0x71021ec
    set32(phy->regs[0] + 0x4000, 0x0000040);

    // MMIO: R.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x49
    // MMIO: W.4   0x23c544004 (dptx-phy[0], offset 0x4004) = 0x48
    clear32(phy->regs[0] + 0x4004, 0x01);

    return 0;
}

u32 dptx_phy_dcp_output(dptx_phy_t *phy)
{
    switch (phy->type) {
        case DPTX_PHY_T8112:
            return 5;
        case DPTX_PHY_T602X:
            return 4;
        case DPTX_PHY_T8122:
            return phy->dfp_endpoint;
        default:
            return 5;
    }
}

dptx_phy_t *dptx_phy_init(const char *phy_node, u32 dcp_index)
{
    enum dptx_type type;
    int adt_phy_path[8];

    int node = adt_path_offset_trace(adt, phy_node, adt_phy_path);
    if (node < 0) {
        printf("DPtx-phy: Error getting phy node %s\n", phy_node);
        return NULL;
    }

    if (adt_is_compatible(adt, node, "dptx-phy,t8112"))
        type = DPTX_PHY_T8112;
    else if (adt_is_compatible(adt, node, "dptx-phy,t602x"))
        type = DPTX_PHY_T602X;
    else if (adt_is_compatible(adt, node, "dptx-phy,t8122"))
        type = DPTX_PHY_T8122;
    else {
        printf("DPtx-phy: dptx-phy node %s is not compatible\n", phy_node);
        return NULL;
    }

    dptx_phy_t *phy = calloc(sizeof *phy, 1);
    if (!phy)
        return NULL;

    phy->type = type;
    phy->dcp_index = dcp_index;

    if (type == DPTX_PHY_T8122) {
        u32 len;
        const u32 *dfp = adt_getprop(adt, node, "dp-switch-dfp-endpoint", &len);

        phy->dfp_endpoint = (dfp && len == sizeof(*dfp)) ? *dfp : 4;
    }

    if (adt_get_reg(adt, adt_phy_path, "reg", 0, &phy->regs[0], NULL) < 0) {
        printf("DPtx-phy: failed to get %s.reg[0]\n", phy_node);
        goto out_err;
    }

    if (adt_get_reg(adt, adt_phy_path, "reg", 1, &phy->regs[1], NULL) < 0) {
        printf("DPtx-phy: failed to get %s.reg[1]\n", phy_node);
        goto out_err;
    }

    return phy;

out_err:
    free(phy);
    return NULL;
}

void dptx_phy_shutdown(dptx_phy_t *phy)
{
    free(phy);
}
