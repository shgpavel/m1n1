/* SPDX-License-Identifier: MIT */

#ifndef BCM57762_H
#define BCM57762_H

#include "types.h"

#define BCM57762_RX_RING_SIZE 128
#define BCM57762_TX_RING_SIZE 128
#define BCM57762_BUF_SIZE     1536

struct bcm57762_dma {
    void *cpu;
    u64 iova;
    size_t size;
};

struct bcm57762_pci {
    u32 (*cfg_read32)(void *ctx, u16 off);
    void (*cfg_write32)(void *ctx, u16 off, u32 val);
    void *ctx;
};

struct bcm57762 {
    u64 regs;
    struct bcm57762_pci pci;
    u8 *cpu;
    u64 iova;
    volatile u32 *status;
    u32 *rx_std;
    u32 *rx_ret;
    u32 *tx_ring;
    u8 *rx_buf;
    u8 *tx_buf;
    u32 rx_prod;
    u32 rx_cons;
    u32 tx_prod;
    u32 speed;
    bool up;
    bool link;
    bool full_duplex;
    u8 mac[6];
    u32 rx_errors;
    u32 rx_dropped;
    u32 tx_busy;
};

int bcm57762_init(struct bcm57762 *nic, u64 bar0, const struct bcm57762_pci *pci,
                  const struct bcm57762_dma *dma, const u8 mac[6]);
bool bcm57762_link_poll(struct bcm57762 *nic);
int bcm57762_tx(void *opaque, const void *frame, size_t len);
ssize_t bcm57762_rx(void *opaque, void *frame, size_t maxlen);
void bcm57762_shutdown(struct bcm57762 *nic);
void bcm57762_dump(struct bcm57762 *nic);

void bcm57762_set_promisc(struct bcm57762 *nic, bool on);
void bcm57762_mcast_add(struct bcm57762 *nic, const u8 addr[6]);

#endif
