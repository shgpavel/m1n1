/* SPDX-License-Identifier: MIT */

#include "eth.h"
#include "adt.h"
#include "bcm57762.h"
#include "dart.h"
#include "malloc.h"
#include "pcie.h"
#include "string.h"
#include "types.h"
#include "utils.h"

#define ETH_PCIE_PATH   "/arm-io/apcie"
#define ETH_BRIDGE_PATH "/arm-io/apcie/pci-bridge2"
#define ETH_NIC_PATH    "/arm-io/apcie/pci-bridge2/lan-1gb"
#define ETH_DART_PATH   "/arm-io/dart-apcie2"

#define ETH_BUS      1
#define ETH_NIC_ID   0x168214e4
#define ETH_WINDOW   SZ_1M
#define ETH_DMA_IOVA 0x100000
#define ETH_DMA_SIZE (2 * SZ_1M)

#define PCI_ID          0x00
#define PCI_COMMAND     0x04
#define PCI_BAR0        0x10
#define PCI_BAR1        0x14
#define PCI_BUS_NUMBERS 0x18
#define PCI_MEM_WINDOW  0x20
#define PCI_PREF_WINDOW 0x24
#define PCI_PREF_BASE32 0x28
#define PCI_PREF_LIM32  0x2c

#define PCI_COMMAND_MEMORY BIT(1)
#define PCI_COMMAND_MASTER BIT(2)

#define PCI_BAR_TYPE      GENMASK(2, 1)
#define PCI_BAR_TYPE_64   2
#define PCI_BAR_ADDR_MASK (~0xfu)

#define ADT_PCI_SPACE     GENMASK(25, 24)
#define ADT_PCI_SPACE_M32 2

static struct {
    bool up;
    u64 ecam;
    u32 port;
    u64 pci_base, cpu_base, win_size;
    u8 mac[6];
    u64 bar0;
    u64 iova;
    void *dma;
    dart_dev_t *dart[2];
    struct bcm57762 nic;
} eth;

static u64 cfg(u32 bus, u32 dev, u16 off)
{
    return eth.ecam + ((u64)bus << 20) + ((u64)dev << 15) + off;
}

static u32 nic_cfg_read32(void *ctx, u16 off)
{
    UNUSED(ctx);
    return read32(cfg(ETH_BUS, 0, off));
}

static void nic_cfg_write32(void *ctx, u16 off, u32 val)
{
    UNUSED(ctx);
    write32(cfg(ETH_BUS, 0, off), val);
}

static int eth_parse_adt(void)
{
    int path[8];
    int node = adt_path_offset_trace(adt, ETH_PCIE_PATH, path);
    if (node < 0 || adt_get_reg(adt, path, "reg", 0, &eth.ecam, NULL)) {
        printf("eth: %s not found\n", ETH_PCIE_PATH);
        return -1;
    }

    u32 len;
    const u32 *ranges = adt_getprop(adt, node, "ranges", &len);
    if (!ranges || len % 28) {
        printf("eth: bad ranges on %s\n", ETH_PCIE_PATH);
        return -1;
    }
    for (u32 i = 0; i < len / 4; i += 7) {
        if (FIELD_GET(ADT_PCI_SPACE, ranges[i]) != ADT_PCI_SPACE_M32)
            continue;
        eth.pci_base = ranges[i + 1] | ((u64)ranges[i + 2] << 32);
        eth.cpu_base = ranges[i + 3] | ((u64)ranges[i + 4] << 32);
        eth.win_size = ranges[i + 5] | ((u64)ranges[i + 6] << 32);
    }
    if (!eth.win_size) {
        printf("eth: no 32-bit memory window on %s\n", ETH_PCIE_PATH);
        return -1;
    }

    int bridge = adt_path_offset(adt, ETH_BRIDGE_PATH);
    if (bridge < 0 || ADT_GETPROP(adt, bridge, "apcie-port", &eth.port) < 0) {
        printf("eth: %s not found\n", ETH_BRIDGE_PATH);
        return -1;
    }

    int nic = adt_path_offset(adt, ETH_NIC_PATH);
    if (nic < 0 || ADT_GETPROP_ARRAY(adt, nic, "local-mac-address", eth.mac) < 0) {
        printf("eth: no MAC address in %s\n", ETH_NIC_PATH);
        return -1;
    }

    printf("eth: ECAM 0x%lx, root port %d, window PCI 0x%lx -> CPU 0x%lx (0x%lx)\n", eth.ecam,
           eth.port, eth.pci_base, eth.cpu_base, eth.win_size);
    printf("eth: MAC %02x:%02x:%02x:%02x:%02x:%02x\n", eth.mac[0], eth.mac[1], eth.mac[2],
           eth.mac[3], eth.mac[4], eth.mac[5]);
    return 0;
}

static int eth_setup_pci(void)
{
    u64 rp = cfg(0, eth.port, 0);
    u32 limit = eth.pci_base + ETH_WINDOW - 1;

    write32(rp + PCI_BUS_NUMBERS, (ETH_BUS << 16) | (ETH_BUS << 8));
    write32(rp + PCI_MEM_WINDOW, ((eth.pci_base >> 16) & 0xfff0) | ((limit >> 16) & 0xfff0) << 16);
    write32(rp + PCI_PREF_WINDOW, 0x0000fff0);
    write32(rp + PCI_PREF_BASE32, 0);
    write32(rp + PCI_PREF_LIM32, 0);
    set32(rp + PCI_COMMAND, PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

    u32 id = read32(cfg(ETH_BUS, 0, PCI_ID));
    printf("eth: 01:00.0 is %04x:%04x\n", id & 0xffff, id >> 16);
    if (id != ETH_NIC_ID)
        return -1;

    u64 bar = cfg(ETH_BUS, 0, PCI_BAR0);
    write32(bar, 0xffffffff);
    u32 probe = read32(bar);
    u32 size = ~(probe & PCI_BAR_ADDR_MASK) + 1;
    write32(bar, eth.pci_base);
    if (FIELD_GET(PCI_BAR_TYPE, probe) == PCI_BAR_TYPE_64)
        write32(cfg(ETH_BUS, 0, PCI_BAR1), eth.pci_base >> 32);
    set32(cfg(ETH_BUS, 0, PCI_COMMAND), PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

    eth.bar0 = eth.cpu_base;
    printf("eth: BAR0 0x%x bytes at 0x%lx (%s)\n", size, eth.bar0,
           FIELD_GET(PCI_BAR_TYPE, probe) == PCI_BAR_TYPE_64 ? "64-bit" : "32-bit");
    return size > ETH_WINDOW ? -1 : 0;
}

static int eth_setup_dma(void)
{
    eth.dma = memalign(SZ_16K, ETH_DMA_SIZE);
    if (!eth.dma)
        return -1;
    memset(eth.dma, 0, ETH_DMA_SIZE);

    for (int sid = 0; sid < 2; sid++) {
        eth.dart[sid] = dart_init_adt(ETH_DART_PATH, 0, sid, false);
        if (!eth.dart[sid]) {
            printf("eth: DART init for SID %d failed\n", sid);
            return -1;
        }
        if (!eth.iova)
            eth.iova = dart_vm_base(eth.dart[sid]) ?: ETH_DMA_IOVA;
        if (dart_map(eth.dart[sid], eth.iova, eth.dma, ETH_DMA_SIZE) < 0) {
            printf("eth: DART map for SID %d failed\n", sid);
            return -1;
        }
    }

    printf("eth: DMA %p mapped at IOVA 0x%lx (SIDs 0, 1)\n", eth.dma, eth.iova);
    return 0;
}

int eth_init(void)
{
    if (eth.up)
        return 0;

    if (eth_parse_adt() < 0)
        return -1;
    if (pcie_init() < 0) {
        printf("eth: PCIe init failed\n");
        return -1;
    }
    if (eth_setup_pci() < 0 || eth_setup_dma() < 0)
        return -1;

    static const struct bcm57762_pci pci = {
        .cfg_read32 = nic_cfg_read32,
        .cfg_write32 = nic_cfg_write32,
    };
    const struct bcm57762_dma dma = {
        .cpu = eth.dma,
        .iova = eth.iova,
        .size = ETH_DMA_SIZE,
    };
    if (bcm57762_init(&eth.nic, eth.bar0, &pci, &dma, eth.mac) < 0) {
        bcm57762_dump(&eth.nic);
        return -1;
    }

    eth.up = true;
    return 0;
}

void eth_shutdown(void)
{
    if (!eth.up)
        return;

    bcm57762_shutdown(&eth.nic);
    for (int sid = 0; sid < 2; sid++)
        if (eth.dart[sid])
            dart_shutdown(eth.dart[sid]);
    eth.up = false;
}

bool eth_link_poll(void)
{
    return eth.up && bcm57762_link_poll(&eth.nic);
}

const u8 *eth_mac(void)
{
    return eth.mac;
}

void *eth_nic(void)
{
    return eth.up ? &eth.nic : NULL;
}

void eth_dump(void)
{
    if (eth.up)
        bcm57762_dump(&eth.nic);
}
