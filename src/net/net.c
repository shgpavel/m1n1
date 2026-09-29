/* SPDX-License-Identifier: MIT */

#include "net.h"
#include "string.h"
#include "utils.h"

#define ETH_P_IP  0x0800
#define ETH_P_ARP 0x0806
#define ETH_MIN   60

#define IP_ICMP 1
#define IP_UDP  17

#define ARP_ENTRIES  8
#define ARP_TRIES    4
#define ARP_RETRY_US 250000
#define ARP_TTL_US   600000000ULL

#define PEND_SLOTS  2
#define UDP_BINDS   4
#define POLL_HOOKS  4
#define RX_BUDGET   64
#define RX_BUF_SIZE 1536

enum {
    ARP_FREE,
    ARP_PENDING,
    ARP_VALID,
};

struct arp_entry {
    u32 ip;
    u8 mac[6];
    u8 state;
    u8 tries;
    u64 stamp;
};

struct pend_frame {
    u32 hop;
    size_t len;
    u8 frame[NET_FRAME_MAX];
};

struct udp_binding {
    u16 port;
    udp_rx_fn fn;
    void *ctx;
};

struct poll_hook {
    net_poll_fn fn;
    void *ctx;
};

static const struct net_nic_ops *nic;
static void *nic_opaque;
static struct net_config net_cfg;
static struct net_stats net_st;
static bool net_polling;
static bool net_building;
static bool net_in_tx;
static u16 net_ip_id;
static struct arp_entry arp_cache[ARP_ENTRIES];
static struct pend_frame pend[PEND_SLOTS];
static struct udp_binding bindings[UDP_BINDS];
static struct poll_hook hooks[POLL_HOOKS];
static u8 rxbuf[RX_BUF_SIZE] __attribute__((aligned(64)));
static u8 txbuf[NET_FRAME_MAX] __attribute__((aligned(64)));
static u8 arpbuf[ETH_MIN] __attribute__((aligned(64)));

static const u8 bcast_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static const u8 zero_mac[6];

static inline u16 get16(const u8 *p)
{
    return (p[0] << 8) | p[1];
}

static inline u32 get32(const u8 *p)
{
    return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

static inline void put16(u8 *p, u16 v)
{
    p[0] = v >> 8;
    p[1] = v;
}

static inline void put32(u8 *p, u32 v)
{
    p[0] = v >> 24;
    p[1] = v >> 16;
    p[2] = v >> 8;
    p[3] = v;
}

static u32 csum_add(u32 sum, const u8 *p, size_t len)
{
    while (len >= 2) {
        sum += get16(p);
        p += 2;
        len -= 2;
    }
    if (len)
        sum += (u32)p[0] << 8;
    return sum;
}

static u16 csum_fold(u32 sum)
{
    sum = (sum & 0xffff) + (sum >> 16);
    sum = (sum & 0xffff) + (sum >> 16);
    return ~sum & 0xffff;
}

static u32 pseudo_sum(u32 src, u32 dst, u8 proto, u16 len)
{
    return (src >> 16) + (src & 0xffff) + (dst >> 16) + (dst & 0xffff) + proto + len;
}

static bool is_bcast(u32 ip)
{
    if (ip == 0xffffffff)
        return true;
    return net_cfg.netmask != 0xffffffff && ip == (net_cfg.ip | ~net_cfg.netmask);
}

static bool on_link(u32 ip)
{
    return !((ip ^ net_cfg.ip) & net_cfg.netmask);
}

static int frame_tx(u8 *frame, size_t len)
{
    int ret;

    if (net_in_tx) {
        net_st.tx_errors++;
        return -1;
    }
    if (len < ETH_MIN) {
        memset(frame + len, 0, ETH_MIN - len);
        len = ETH_MIN;
    }

    net_in_tx = true;
    ret = nic->tx(nic_opaque, frame, len);
    net_in_tx = false;

    if (ret) {
        net_st.tx_errors++;
        return -1;
    }
    net_st.tx_frames++;
    return 0;
}

static void pend_drop(u32 hop)
{
    for (int i = 0; i < PEND_SLOTS; i++)
        if (pend[i].len && pend[i].hop == hop)
            pend[i].len = 0;
}

static void pend_flush(u32 hop, const u8 *mac)
{
    for (int i = 0; i < PEND_SLOTS; i++) {
        struct pend_frame *p = &pend[i];

        if (!p->len || p->hop != hop)
            continue;
        memcpy(p->frame, mac, 6);
        frame_tx(p->frame, p->len);
        p->len = 0;
    }
}

static struct arp_entry *arp_find(u32 ip)
{
    for (int i = 0; i < ARP_ENTRIES; i++)
        if (arp_cache[i].state != ARP_FREE && arp_cache[i].ip == ip)
            return &arp_cache[i];
    return NULL;
}

static struct arp_entry *arp_alloc(u32 ip)
{
    struct arp_entry *victim = NULL;

    for (int i = 0; i < ARP_ENTRIES; i++) {
        struct arp_entry *e = &arp_cache[i];

        if (e->state == ARP_FREE) {
            victim = e;
            break;
        }
        if (!victim || e->stamp < victim->stamp)
            victim = e;
    }

    if (victim->state == ARP_PENDING)
        pend_drop(victim->ip);
    memset(victim, 0, sizeof(*victim));
    victim->ip = ip;
    return victim;
}

static void arp_send(u16 op, const u8 *dst_mac, const u8 *tha, u32 tpa)
{
    u8 *f = arpbuf;
    u8 *a = f + NET_ETH_HLEN;

    memcpy(f, dst_mac, 6);
    memcpy(f + 6, net_cfg.mac, 6);
    put16(f + 12, ETH_P_ARP);
    put16(a, 1);
    put16(a + 2, ETH_P_IP);
    a[4] = 6;
    a[5] = 4;
    put16(a + 6, op);
    memcpy(a + 8, net_cfg.mac, 6);
    put32(a + 14, net_cfg.ip);
    memcpy(a + 18, tha, 6);
    put32(a + 24, tpa);
    frame_tx(f, NET_ETH_HLEN + 28);
}

static void arp_request(struct arp_entry *e, u64 now)
{
    e->tries++;
    e->stamp = now;
    net_st.arp_requests++;
    arp_send(1, bcast_mac, zero_mac, e->ip);
}

static void arp_learn(u32 ip, const u8 *mac, bool create, u64 now)
{
    struct arp_entry *e;

    if (!ip || (mac[0] & 1) || is_bcast(ip) || !on_link(ip) || ip == net_cfg.ip)
        return;

    e = arp_find(ip);
    if (!e) {
        if (!create)
            return;
        e = arp_alloc(ip);
    }

    if (e->state == ARP_PENDING)
        net_st.arp_resolved++;
    memcpy(e->mac, mac, 6);
    e->state = ARP_VALID;
    e->tries = 0;
    e->stamp = now;
    pend_flush(ip, mac);
}

static void arp_timers(u64 now)
{
    for (int i = 0; i < ARP_ENTRIES; i++) {
        struct arp_entry *e = &arp_cache[i];

        if (e->state != ARP_PENDING || now - e->stamp < ARP_RETRY_US)
            continue;
        if (e->tries >= ARP_TRIES) {
            pend_drop(e->ip);
            e->state = ARP_FREE;
            net_st.arp_failed++;
            continue;
        }
        arp_request(e, now);
    }
}

static int pend_queue(u32 hop, size_t len, u64 now)
{
    struct pend_frame *slot = NULL;
    struct arp_entry *e;

    for (int i = 0; i < PEND_SLOTS; i++) {
        if (!pend[i].len) {
            slot = &pend[i];
            break;
        }
    }
    if (!slot) {
        net_st.tx_errors++;
        return -1;
    }

    e = arp_find(hop);
    if (!e)
        e = arp_alloc(hop);

    slot->hop = hop;
    slot->len = len;
    memcpy(slot->frame, txbuf, len);

    if (e->state != ARP_PENDING) {
        e->state = ARP_PENDING;
        e->tries = 0;
        arp_request(e, now);
    }
    return 0;
}

static int ip_output(u32 dst, u8 proto, size_t plen)
{
    u8 *ip = txbuf + NET_ETH_HLEN;
    size_t len = NET_ETH_HLEN + 20 + plen;
    struct arp_entry *e;
    u64 now;
    u32 hop;

    ip[0] = 0x45;
    ip[1] = 0;
    put16(ip + 2, 20 + plen);
    put16(ip + 4, net_ip_id++);
    put16(ip + 6, 0x4000);
    ip[8] = 64;
    ip[9] = proto;
    put16(ip + 10, 0);
    put32(ip + 12, net_cfg.ip);
    put32(ip + 16, dst);
    put16(ip + 10, csum_fold(csum_add(0, ip, 20)));
    memcpy(txbuf + 6, net_cfg.mac, 6);
    put16(txbuf + 12, ETH_P_IP);

    if (is_bcast(dst)) {
        memcpy(txbuf, bcast_mac, 6);
        return frame_tx(txbuf, len);
    }

    hop = on_link(dst) ? dst : net_cfg.gateway;
    if (!hop) {
        net_st.tx_errors++;
        return -1;
    }

    now = net_now_us();
    e = arp_find(hop);
    if (e && e->state == ARP_VALID && now - e->stamp < ARP_TTL_US) {
        memcpy(txbuf, e->mac, 6);
        return frame_tx(txbuf, len);
    }
    return pend_queue(hop, len, now);
}

static void icmp_rx(const u8 *ip, size_t ihl, size_t tot)
{
    const u8 *ic = ip + ihl;
    size_t len = tot - ihl;
    u8 *o = txbuf + NET_ETH_HLEN + 20;

    if (len < 8 || csum_fold(csum_add(0, ic, len))) {
        net_st.rx_bad++;
        return;
    }
    net_st.rx_icmp++;
    if (ic[0] != 8 || ic[1] != 0 || net_building)
        return;

    net_building = true;
    memcpy(o, ic, len);
    o[0] = 0;
    put16(o + 2, 0);
    put16(o + 2, csum_fold(csum_add(0, o, len)));
    ip_output(get32(ip + 12), IP_ICMP, len);
    net_building = false;
}

static void udp_rx(const u8 *ip, size_t ihl, size_t tot, u32 src, u32 dst)
{
    const u8 *u = ip + ihl;
    size_t ulen;
    u16 port;

    if (tot - ihl < 8) {
        net_st.rx_bad++;
        return;
    }
    ulen = get16(u + 4);
    if (ulen < 8 || ulen > tot - ihl) {
        net_st.rx_bad++;
        return;
    }
    if (get16(u + 6) && csum_fold(csum_add(pseudo_sum(src, dst, IP_UDP, ulen), u, ulen))) {
        net_st.rx_bad++;
        return;
    }

    net_st.rx_udp++;
    port = get16(u + 2);
    for (int i = 0; i < UDP_BINDS; i++) {
        if (bindings[i].fn && bindings[i].port == port) {
            bindings[i].fn(bindings[i].ctx, src, get16(u), u + 8, ulen - 8);
            return;
        }
    }
    net_st.rx_udp_unbound++;
}

static void ip_rx(const u8 *f, size_t len, u64 now)
{
    const u8 *ip = f + NET_ETH_HLEN;
    size_t avail = len - NET_ETH_HLEN;
    size_t ihl, tot;
    u32 src, dst;
    bool bcast;

    if (avail < 20 || (ip[0] >> 4) != 4) {
        net_st.rx_bad++;
        return;
    }
    ihl = (ip[0] & 15) * 4;
    tot = get16(ip + 2);
    if (ihl < 20 || tot < ihl || tot > avail || tot > NET_MTU || csum_fold(csum_add(0, ip, ihl))) {
        net_st.rx_bad++;
        return;
    }
    if (get16(ip + 6) & 0x3fff) {
        net_st.rx_frag++;
        return;
    }

    src = get32(ip + 12);
    dst = get32(ip + 16);
    bcast = is_bcast(dst);
    if (!net_cfg.ip || (dst != net_cfg.ip && !bcast))
        return;
    if (!src || is_bcast(src) || (src >> 28) == 0xe) {
        net_st.rx_bad++;
        return;
    }

    if (!bcast)
        arp_learn(src, f + 6, true, now);

    if (ip[9] == IP_UDP)
        udp_rx(ip, ihl, tot, src, dst);
    else if (ip[9] == IP_ICMP && !bcast)
        icmp_rx(ip, ihl, tot);
}

static void arp_rx(const u8 *f, size_t len, u64 now)
{
    const u8 *a = f + NET_ETH_HLEN;
    bool for_us;
    u32 spa;

    if (len < NET_ETH_HLEN + 28 || get16(a) != 1 || get16(a + 2) != ETH_P_IP || a[4] != 6 ||
        a[5] != 4) {
        net_st.rx_bad++;
        return;
    }

    net_st.rx_arp++;
    spa = get32(a + 14);
    for_us = net_cfg.ip && get32(a + 24) == net_cfg.ip;
    arp_learn(spa, a + 8, for_us, now);
    if (for_us && get16(a + 6) == 1)
        arp_send(2, a + 8, a + 8, spa);
}

static void eth_rx(const u8 *f, size_t len, u64 now)
{
    if (len < NET_ETH_HLEN) {
        net_st.rx_bad++;
        return;
    }
    if (memcmp(f, net_cfg.mac, 6) && memcmp(f, bcast_mac, 6))
        return;

    switch (get16(f + 12)) {
        case ETH_P_ARP:
            arp_rx(f, len, now);
            break;
        case ETH_P_IP:
            ip_rx(f, len, now);
            break;
    }
}

int net_init(const struct net_nic_ops *ops, void *opaque, const struct net_config *cfg)
{
    u32 ip = cfg ? cfg->ip : 0;
    const u8 *m = cfg ? cfg->mac : zero_mac;

    if (!ops || !ops->tx || !ops->rx || !cfg || !ip)
        return -1;

    nic = ops;
    nic_opaque = opaque;
    net_cfg = *cfg;
    net_polling = false;
    net_building = false;
    net_in_tx = false;
    net_ip_id = net_now_us();
    memset(&net_st, 0, sizeof(net_st));
    memset(arp_cache, 0, sizeof(arp_cache));
    for (int i = 0; i < PEND_SLOTS; i++)
        pend[i].len = 0;

    printf("net: %u.%u.%u.%u mask 0x%08x gw 0x%08x mac %02x:%02x:%02x:%02x:%02x:%02x\n", ip >> 24,
           (ip >> 16) & 0xff, (ip >> 8) & 0xff, ip & 0xff, cfg->netmask, cfg->gateway, m[0], m[1],
           m[2], m[3], m[4], m[5]);
    return 0;
}

void net_poll(void)
{
    u64 now;

    if (!nic || net_polling)
        return;
    net_polling = true;

    now = net_now_us();
    for (int i = 0; i < RX_BUDGET; i++) {
        ssize_t n = nic->rx(nic_opaque, rxbuf, sizeof(rxbuf));

        if (n <= 0)
            break;
        if ((size_t)n > sizeof(rxbuf))
            n = sizeof(rxbuf);
        net_st.rx_frames++;
        eth_rx(rxbuf, n, now);
    }
    arp_timers(now);

    for (int i = 0; i < POLL_HOOKS; i++)
        if (hooks[i].fn)
            hooks[i].fn(hooks[i].ctx);

    net_polling = false;
}

bool net_busy(void)
{
    return net_polling || net_building || net_in_tx;
}

int net_add_poll_hook(net_poll_fn fn, void *ctx)
{
    for (int i = 0; i < POLL_HOOKS; i++)
        if (hooks[i].fn == fn && hooks[i].ctx == ctx)
            return 0;
    for (int i = 0; i < POLL_HOOKS; i++) {
        if (!hooks[i].fn) {
            hooks[i].ctx = ctx;
            hooks[i].fn = fn;
            return 0;
        }
    }
    return -1;
}

int udp_bind(u16 port, udp_rx_fn fn, void *ctx)
{
    struct udp_binding *slot = NULL;

    if (!port)
        return -1;
    for (int i = 0; i < UDP_BINDS; i++) {
        if (bindings[i].fn && bindings[i].port == port) {
            slot = &bindings[i];
            break;
        }
        if (!slot && !bindings[i].fn)
            slot = &bindings[i];
    }
    if (!slot)
        return -1;

    slot->port = port;
    slot->ctx = ctx;
    slot->fn = fn;
    return 0;
}

int udp_send(u32 dst_ip, u16 dst_port, u16 src_port, const void *data, size_t len)
{
    u8 *u = txbuf + NET_ETH_HLEN + 20;
    u16 sum;
    int ret;

    if (!nic || len > NET_UDP_MAX || net_building)
        return -1;

    net_building = true;
    put16(u, src_port);
    put16(u + 2, dst_port);
    put16(u + 4, len + 8);
    put16(u + 6, 0);
    memcpy(u + 8, data, len);
    sum = csum_fold(csum_add(pseudo_sum(net_cfg.ip, dst_ip, IP_UDP, len + 8), u, len + 8));
    put16(u + 6, sum ? sum : 0xffff);
    ret = ip_output(dst_ip, IP_UDP, len + 8);
    net_building = false;
    return ret;
}

void net_get_stats(struct net_stats *stats)
{
    *stats = net_st;
}
