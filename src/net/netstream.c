/* SPDX-License-Identifier: MIT */

#include "netstream.h"
#include "iodev.h"
#include "net.h"
#include "string.h"

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "netstream assumes a little-endian CPU"
#endif

_Static_assert(!(NS_RX_SIZE & (NS_RX_SIZE - 1)), "NS_RX_SIZE must be a power of two");
_Static_assert(!(NS_TX_SIZE & (NS_TX_SIZE - 1)), "NS_TX_SIZE must be a power of two");
_Static_assert(NS_MSS + sizeof(struct ns_hdr) <= NET_UDP_MAX, "NS_MSS too large");

#define HDR_LEN   ((u32)sizeof(struct ns_hdr))
#define ACK_EVERY 8

struct ns_chan {
    u32 rcv_nxt;
    u32 rcv_rd;
    u32 adv_edge;
    u32 rx_unacked;
    bool ack_due;
    u32 snd_una;
    u32 snd_nxt;
    u32 snd_max;
    u32 snd_wr;
    u32 snd_lim;
    u32 dupacks;
    bool fr_done;
    bool stalled;
    u32 backoff;
    u64 rto_at;
    u64 progress_us;
    struct netstream_stats st;
    u8 rx[NS_RX_SIZE];
    u8 tx[NS_TX_SIZE];
};

static struct ns_chan chans[NS_CHANNELS];

static struct {
    bool connected;
    u32 session;
    u32 peer_ip;
    u16 peer_port;
    u64 last_rx_us;
    u32 gen;
    u32 sessions;
    u8 seg[sizeof(struct ns_hdr) + NS_MSS];
} ns;

void __attribute__((weak)) netstream_peer_alive(void)
{
}

static inline u32 umin(u32 a, u32 b)
{
    return a < b ? a : b;
}

static inline u32 szmin(size_t a, u32 b)
{
    return a < b ? (u32)a : b;
}

static void ring_put(u8 *ring, u32 size, u32 pos, const u8 *src, u32 n)
{
    u32 off = pos & (size - 1);
    u32 first = umin(n, size - off);

    memcpy(ring + off, src, first);
    memcpy(ring, src + first, n - first);
}

static void ring_get(const u8 *ring, u32 size, u32 pos, u8 *dst, u32 n)
{
    u32 off = pos & (size - 1);
    u32 first = umin(n, size - off);

    memcpy(dst, ring + off, first);
    memcpy(dst + first, ring, n - first);
}

static u32 rx_free(struct ns_chan *c)
{
    return NS_RX_SIZE - (c->rcv_nxt - c->rcv_rd);
}

static u64 rto_us(struct ns_chan *c)
{
    return (u64)NS_RTO_US << c->backoff;
}

static bool ns_alive(u64 now)
{
    return ns.connected && (s64)(now - ns.last_rx_us) < NS_PEER_TIMEOUT_US;
}

static u64 since(u64 now, u64 a, u64 b)
{
    return now - (a > b ? a : b);
}

static int ns_emit(u32 ip, u16 port, u8 type, u8 channel, u32 session, u32 seq, u32 ack, u32 wnd,
                   u32 len)
{
    struct ns_hdr h = {
        .magic = NS_MAGIC,
        .type = type,
        .channel = channel,
        .len = len,
        .session = session,
        .seq = seq,
        .ack = ack,
        .wnd = wnd,
    };

    memcpy(ns.seg, &h, HDR_LEN);
    return udp_send(ip, port, NS_PORT, ns.seg, HDR_LEN + len);
}

static void ns_emit_chan(struct ns_chan *c, u8 type, u32 seq, u32 len)
{
    u32 wnd = rx_free(c);

    c->adv_edge = c->rcv_nxt + wnd;
    c->rx_unacked = 0;
    c->ack_due = false;
    ns_emit(ns.peer_ip, ns.peer_port, type, c - chans, ns.session, seq, c->rcv_nxt, wnd, len);
}

static void ns_send_ack(struct ns_chan *c)
{
    ns_emit_chan(c, NS_ACK, c->snd_nxt, 0);
}

static bool ns_need_timer(struct ns_chan *c)
{
    if (c->snd_max != c->snd_una)
        return true;
    return c->snd_wr != c->snd_nxt && (s32)(c->snd_lim - c->snd_nxt) <= 0;
}

static void ns_output(struct ns_chan *c, bool force, u64 now)
{
    while (ns.connected && c->snd_nxt != c->snd_wr) {
        u32 unsent = c->snd_wr - c->snd_nxt;
        u32 inflight = c->snd_nxt - c->snd_una;
        s32 room = c->snd_lim - c->snd_nxt;
        u32 n;

        if (room <= 0 || inflight >= NS_WINDOW)
            break;
        n = umin(umin(unsent, NS_MSS), umin(room, NS_WINDOW - inflight));
        if (!force && n == unsent && n < NS_MSS && inflight)
            break;

        ring_get(c->tx, NS_TX_SIZE, c->snd_nxt, ns.seg + HDR_LEN, n);
        ns_emit_chan(c, NS_DATA, c->snd_nxt, n);
        c->st.tx_segs++;
        c->snd_nxt += n;
        if ((s32)(c->snd_nxt - c->snd_max) > 0)
            c->snd_max = c->snd_nxt;
        if (!c->rto_at)
            c->rto_at = now + rto_us(c);
    }
}

static void ns_rto_fire(struct ns_chan *c, u64 now)
{
    c->rto_at = 0;
    if (c->snd_una == c->snd_wr)
        return;

    c->st.rto++;
    if (c->backoff < NS_RTO_MAX_SHIFT)
        c->backoff++;
    c->snd_nxt = c->snd_una;
    c->dupacks = 0;
    c->fr_done = true;
    if ((s32)(c->snd_lim - c->snd_una) <= 0)
        ns_emit_chan(c, NS_DATA, c->snd_una, 0);
    else
        ns_output(c, true, now);
    c->rto_at = now + rto_us(c);
}

static void ns_on_ack(struct ns_chan *c, u32 ack, u32 wnd, bool pure, u64 now)
{
    u32 acked = ack - c->snd_una;
    u32 edge = ack + wnd;
    bool opened = false;

    if (acked > c->snd_max - c->snd_una)
        return;

    if ((s32)(edge - c->snd_lim) > 0) {
        c->snd_lim = edge;
        opened = true;
    }

    if (acked) {
        c->snd_una = ack;
        if ((s32)(c->snd_nxt - ack) < 0)
            c->snd_nxt = ack;
        c->st.tx_bytes += acked;
        c->dupacks = 0;
        c->fr_done = false;
        c->backoff = 0;
        c->rto_at = c->snd_una != c->snd_max ? now + NS_RTO_US : 0;
    } else if (pure && !opened && c->snd_una != c->snd_max) {
        if (++c->dupacks >= 3 && !c->fr_done) {
            c->fr_done = true;
            c->st.fast_rtx++;
            c->snd_nxt = c->snd_una;
            c->rto_at = now + rto_us(c);
        }
    }

    if (acked || opened) {
        c->progress_us = now;
        c->stalled = false;
    }
}

static void ns_on_data(struct ns_chan *c, u32 seq, const u8 *data, u32 len)
{
    s32 off = c->rcv_nxt - seq;
    u32 n;

    if (off < 0 || (u32)off >= len) {
        if (len)
            c->st.rx_dup++;
        ns_send_ack(c);
        return;
    }

    n = umin(len - off, rx_free(c));
    ring_put(c->rx, NS_RX_SIZE, c->rcv_nxt, data + off, n);
    c->rcv_nxt += n;
    c->st.rx_bytes += n;
    c->st.rx_segs++;
    if (n < len - off || ++c->rx_unacked >= ACK_EVERY)
        ns_send_ack(c);
    else
        c->ack_due = true;
}

static void ns_start(u32 ip, u16 port, u32 session, u32 wnd, u64 now)
{
    for (int i = 0; i < NS_CHANNELS; i++) {
        struct ns_chan *c = &chans[i];

        c->rcv_nxt = session;
        c->rcv_rd = session;
        c->adv_edge = session + NS_RX_SIZE;
        c->rx_unacked = 0;
        c->ack_due = false;
        c->snd_una = session;
        c->snd_nxt = session;
        c->snd_max = session;
        c->snd_wr = session;
        c->snd_lim = session + wnd;
        c->dupacks = 0;
        c->fr_done = false;
        c->stalled = false;
        c->backoff = 0;
        c->rto_at = 0;
        c->progress_us = now;
    }

    ns.peer_ip = ip;
    ns.peer_port = port;
    ns.session = session;
    ns.connected = true;
    ns.gen++;
    ns.sessions++;
}

static void ns_udp_rx(void *ctx, u32 src_ip, u16 src_port, const void *data, size_t len)
{
    const u8 *payload = (const u8 *)data + HDR_LEN;
    bool same_addr = ns.connected && src_ip == ns.peer_ip && src_port == ns.peer_port;
    struct ns_hdr h;
    u64 now;

    (void)ctx;
    if (len < HDR_LEN)
        return;
    memcpy(&h, data, HDR_LEN);
    if (h.magic != NS_MAGIC || HDR_LEN + h.len > len)
        return;

    now = net_now_us();

    if (h.type == NS_SYN) {
        if (!same_addr || h.session != ns.session)
            ns_start(src_ip, src_port, h.session, h.wnd, now);
        ns.last_rx_us = now;
        ns_emit(ns.peer_ip, ns.peer_port, NS_SYNACK, 0, ns.session, 0, 0, NS_RX_SIZE, 0);
        netstream_peer_alive();
        return;
    }

    if (!same_addr || h.session != ns.session) {
        if (h.type != NS_RST && (!ns.connected || same_addr))
            ns_emit(src_ip, src_port, NS_RST, 0, h.session, 0, 0, 0, 0);
        return;
    }

    ns.last_rx_us = now;
    netstream_peer_alive();

    switch (h.type) {
        case NS_RST:
            ns.connected = false;
            ns.gen++;
            break;
        case NS_PING:
            for (int i = 0; i < NS_CHANNELS; i++)
                chans[i].ack_due = true;
            break;
        case NS_DATA:
        case NS_ACK:
            if (h.channel >= NS_CHANNELS)
                break;
            ns_on_ack(&chans[h.channel], h.ack, h.wnd, h.type == NS_ACK, now);
            if (h.type == NS_DATA)
                ns_on_data(&chans[h.channel], h.seq, payload, h.len);
            break;
    }
}

static void ns_hook(void *ctx)
{
    u64 now = net_now_us();

    (void)ctx;
    if (!ns.connected)
        return;

    for (int i = 0; i < NS_CHANNELS; i++) {
        struct ns_chan *c = &chans[i];

        if (c->rto_at && (s64)(now - c->rto_at) >= 0)
            ns_rto_fire(c, now);
        ns_output(c, false, now);
        if (c->ack_due)
            ns_send_ack(c);
        if (!ns_need_timer(c))
            c->rto_at = 0;
        else if (!c->rto_at)
            c->rto_at = now + rto_us(c);
    }
}

static void ns_window_update(struct ns_chan *c)
{
    if (ns.connected && (c->rcv_rd + NS_RX_SIZE) - c->adv_edge >= NS_RX_SIZE / 4)
        ns_send_ack(c);
}

static ssize_t ns_can_read(void *opaque)
{
    struct ns_chan *c = opaque;

    return c->rcv_nxt - c->rcv_rd;
}

static bool ns_can_write(void *opaque)
{
    struct ns_chan *c = opaque;

    return ns_alive(net_now_us()) && !c->stalled;
}

static ssize_t ns_read(void *opaque, void *buf, size_t len)
{
    struct ns_chan *c = opaque;
    u8 *p = buf;
    size_t done = 0;

    while (done < len) {
        u32 avail = c->rcv_nxt - c->rcv_rd;

        if (avail) {
            u32 n = szmin(len - done, avail);

            ring_get(c->rx, NS_RX_SIZE, c->rcv_rd, p + done, n);
            c->rcv_rd += n;
            done += n;
            continue;
        }
        if (net_busy())
            break;
        ns_window_update(c);
        net_poll();
    }

    if (!net_busy())
        ns_window_update(c);
    return done;
}

static ssize_t ns_put(struct ns_chan *c, const void *buf, size_t len, bool push)
{
    const u8 *p = buf;
    size_t done = 0;
    u32 gen = ns.gen;
    u64 start = 0;

    if (!ns.connected)
        return 0;

    while (done < len) {
        u32 space = NS_TX_SIZE - (c->snd_wr - c->snd_una);
        u64 now;

        if (space) {
            u32 n = szmin(len - done, space);

            ring_put(c->tx, NS_TX_SIZE, c->snd_wr, p + done, n);
            c->snd_wr += n;
            done += n;
            continue;
        }
        if (net_busy() || c->stalled)
            break;

        now = net_now_us();
        if (!start)
            start = now;
        if (!ns_alive(now))
            break;
        if (since(now, start, c->progress_us) > NS_STALL_US) {
            c->stalled = true;
            c->st.stalls++;
            break;
        }

        ns_output(c, false, now);
        net_poll();
        if (gen != ns.gen || !ns.connected)
            break;
    }

    if (push && !net_busy())
        ns_output(c, false, net_now_us());
    return done;
}

static ssize_t ns_write(void *opaque, const void *buf, size_t len)
{
    return ns_put(opaque, buf, len, true);
}

static ssize_t ns_queue(void *opaque, const void *buf, size_t len)
{
    return ns_put(opaque, buf, len, false);
}

static void ns_flush(void *opaque)
{
    struct ns_chan *c = opaque;
    u32 gen = ns.gen;
    u64 start;

    if (!ns.connected || net_busy())
        return;

    start = net_now_us();
    ns_output(c, true, start);
    while (c->snd_una != c->snd_wr) {
        u64 now = net_now_us();

        if (!ns_alive(now) || c->stalled)
            break;
        if (since(now, start, c->progress_us) > NS_STALL_US) {
            c->stalled = true;
            c->st.stalls++;
            break;
        }

        net_poll();
        if (gen != ns.gen || !ns.connected)
            break;
        ns_output(c, true, net_now_us());
    }
}

static void ns_handle_events(void *opaque)
{
    (void)opaque;
    net_poll();
}

const struct iodev_ops netstream_iodev_ops = {
    .can_read = ns_can_read,
    .can_write = ns_can_write,
    .read = ns_read,
    .write = ns_write,
    .queue = ns_queue,
    .flush = ns_flush,
    .handle_events = ns_handle_events,
};

int netstream_init(void)
{
    if (udp_bind(NS_PORT, ns_udp_rx, NULL))
        return -1;
    if (net_add_poll_hook(ns_hook, NULL))
        return -1;
    return 0;
}

void *netstream_opaque(int channel)
{
    if (channel < 0 || channel >= NS_CHANNELS)
        return NULL;
    return &chans[channel];
}

bool netstream_connected(void)
{
    return ns_alive(net_now_us());
}

u32 netstream_session(void)
{
    return ns.connected ? ns.session : 0;
}

u32 netstream_sessions(void)
{
    return ns.sessions;
}

void netstream_get_stats(int channel, struct netstream_stats *stats)
{
    if (channel < 0 || channel >= NS_CHANNELS)
        memset(stats, 0, sizeof(*stats));
    else
        *stats = chans[channel].st;
}
