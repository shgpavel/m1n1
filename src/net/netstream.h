/* SPDX-License-Identifier: MIT */

#ifndef NETSTREAM_H
#define NETSTREAM_H

#include "types.h"

#define NS_PORT     7101
#define NS_MAGIC    0x736e316d
#define NS_CHANNELS 2
#define NS_CH_PROXY 0
#define NS_CH_VUART 1

#define NS_MSS             1400
#define NS_WINDOW          (32 * NS_MSS)
#define NS_RTO_US          50000
#define NS_RTO_MAX_SHIFT   4
#define NS_PEER_TIMEOUT_US 3000000
#define NS_STALL_US        5000000

#ifndef NS_RX_SIZE
#define NS_RX_SIZE (64 * 1024)
#endif
#ifndef NS_TX_SIZE
#define NS_TX_SIZE (256 * 1024)
#endif

typedef u16 ns_le16;
typedef u32 ns_le32;

enum ns_type {
    NS_SYN = 1,
    NS_SYNACK = 2,
    NS_DATA = 3,
    NS_ACK = 4,
    NS_PING = 5,
    NS_RST = 6,
};

struct ns_hdr {
    ns_le32 magic;
    u8 type;
    u8 channel;
    ns_le16 len;
    ns_le32 session;
    ns_le32 seq;
    ns_le32 ack;
    ns_le32 wnd;
};

_Static_assert(sizeof(struct ns_hdr) == 24, "struct ns_hdr must be 24 bytes");

struct netstream_stats {
    u64 tx_bytes;
    u64 rx_bytes;
    u64 tx_segs;
    u64 rx_segs;
    u64 rto;
    u64 fast_rtx;
    u64 rx_dup;
    u64 stalls;
};

struct iodev_ops;
extern const struct iodev_ops netstream_iodev_ops;

int netstream_init(void);
void *netstream_opaque(int channel);
bool netstream_connected(void);
u32 netstream_session(void);
u32 netstream_sessions(void);
void netstream_get_stats(int channel, struct netstream_stats *stats);

void netstream_peer_alive(void);

#endif
