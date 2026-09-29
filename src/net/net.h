/* SPDX-License-Identifier: MIT */

#ifndef NET_H
#define NET_H

#include "types.h"

#define NET_ETH_HLEN  14
#define NET_MTU       1500
#define NET_FRAME_MAX (NET_ETH_HLEN + NET_MTU)
#define NET_UDP_MAX   (NET_MTU - 20 - 8)

#define NET_IP(a, b, c, d) (((u32)(a) << 24) | ((u32)(b) << 16) | ((u32)(c) << 8) | (u32)(d))

struct net_nic_ops {
    int (*tx)(void *opaque, const void *frame, size_t len);
    ssize_t (*rx)(void *opaque, void *frame, size_t maxlen);
};

struct net_config {
    u8 mac[6];
    u32 ip;
    u32 netmask;
    u32 gateway;
};

struct net_stats {
    u64 rx_frames;
    u64 rx_bad;
    u64 rx_frag;
    u64 rx_arp;
    u64 rx_icmp;
    u64 rx_udp;
    u64 rx_udp_unbound;
    u64 tx_frames;
    u64 tx_errors;
    u64 arp_requests;
    u64 arp_resolved;
    u64 arp_failed;
};

typedef void (*udp_rx_fn)(void *ctx, u32 src_ip, u16 src_port, const void *data, size_t len);
typedef void (*net_poll_fn)(void *ctx);

int net_init(const struct net_nic_ops *ops, void *opaque, const struct net_config *cfg);
void net_poll(void);
bool net_busy(void);
int net_add_poll_hook(net_poll_fn fn, void *ctx);
int udp_bind(u16 port, udp_rx_fn fn, void *ctx);
int udp_send(u32 dst_ip, u16 dst_port, u16 src_port, const void *data, size_t len);
void net_get_stats(struct net_stats *stats);

u64 net_now_us(void);

#endif
