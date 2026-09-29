/* SPDX-License-Identifier: MIT */

#include "../build/build_tag.h"

#include "netproxy.h"
#include "bcm57762.h"
#include "cpu_regs.h"
#include "disklog.h"
#include "eth.h"
#include "iodev.h"
#include "net.h"
#include "netstream.h"
#include "string.h"
#include "utils.h"
#include "wdt.h"

#define NETPROXY_LINK_TIMEOUT_US 10000000
#define NETPROXY_LINK_POLL_US    1000000
#define NETPROXY_ANNOUNCE_PORT   7100

static const struct net_nic_ops nic_ops = {
    .tx = bcm57762_tx,
    .rx = bcm57762_rx,
};

static struct iodev iodev_net = {
    .ops = &netstream_iodev_ops,
    .usage = USAGE_CONSOLE | USAGE_UARTPROXY,
    .lock = SPINLOCK_INIT,
};

static struct iodev iodev_net_vuart = {
    .ops = &netstream_iodev_ops,
    .lock = SPINLOCK_INIT,
};

static bool netproxy_up;
static u64 next_link_poll;

u64 net_now_us(void)
{
    return ticks_to_usecs(mrs(CNTPCT_EL0));
}

void netstream_peer_alive(void)
{
    wdt_kick();
}

static void link_hook(void *ctx)
{
    UNUSED(ctx);
    u64 now = net_now_us();

    if (now < next_link_poll)
        return;
    next_link_poll = now + NETPROXY_LINK_POLL_US;
    eth_link_poll();
}

static int parse_ip(const char **s, u32 *ip)
{
    u32 v = 0;

    for (int i = 0; i < 4; i++) {
        u32 octet = 0;
        int digits = 0;

        while (**s >= '0' && **s <= '9' && digits < 3) {
            octet = octet * 10 + (*(*s)++ - '0');
            digits++;
        }
        if (!digits || octet > 255)
            return -1;
        v = (v << 8) | octet;
        if (i < 3 && *(*s)++ != '.')
            return -1;
    }

    *ip = v;
    return 0;
}

static int parse_spec(const char *s, struct net_config *cfg)
{
    u32 prefix = 24;

    if (parse_ip(&s, &cfg->ip))
        return -1;

    if (*s == '/') {
        s++;
        prefix = 0;
        while (*s >= '0' && *s <= '9')
            prefix = prefix * 10 + (*s++ - '0');
        if (!prefix || prefix > 32)
            return -1;
    }
    cfg->netmask = prefix == 32 ? 0xffffffff : ~(0xffffffffU >> prefix);

    if (*s == ',') {
        s++;
        if (parse_ip(&s, &cfg->gateway))
            return -1;
    }

    return *s ? -1 : 0;
}

static void announce(const struct net_config *cfg)
{
    char msg[128];
    int len = snprintf(msg, sizeof(msg), "m1n1 %s up at %d.%d.%d.%d, netstream on udp/%d\n",
                       BUILD_TAG, cfg->ip >> 24, (cfg->ip >> 16) & 0xff, (cfg->ip >> 8) & 0xff,
                       cfg->ip & 0xff, NS_PORT);

    for (int i = 0; i < 3; i++) {
        udp_send(0xffffffff, NETPROXY_ANNOUNCE_PORT, NETPROXY_ANNOUNCE_PORT, msg, len);
        net_poll();
        mdelay(10);
    }
}

int netproxy_start(const char *spec)
{
    struct net_config cfg = {0};

    if (netproxy_up)
        return 0;

    if (parse_spec(spec, &cfg)) {
        printf("netproxy: bad address spec '%s'\n", spec);
        return -1;
    }

    printf("netproxy: bringing up Ethernet\n");
    disklog_flush();
    if (eth_init() < 0) {
        printf("netproxy: Ethernet init failed\n");
        disklog_flush();
        return -1;
    }
    disklog_flush();

    u64 timeout = timeout_calculate(NETPROXY_LINK_TIMEOUT_US);
    while (!eth_link_poll()) {
        if (timeout_expired(timeout)) {
            printf("netproxy: no link after %d s\n", NETPROXY_LINK_TIMEOUT_US / 1000000);
            eth_dump();
            disklog_flush();
            return -1;
        }
        mdelay(100);
    }

    memcpy(cfg.mac, eth_mac(), sizeof(cfg.mac));
    if (net_init(&nic_ops, eth_nic(), &cfg) < 0 || netstream_init() < 0) {
        printf("netproxy: network stack init failed\n");
        disklog_flush();
        return -1;
    }
    net_add_poll_hook(link_hook, NULL);

    iodev_net.opaque = netstream_opaque(NS_CH_PROXY);
    iodev_net_vuart.opaque = netstream_opaque(NS_CH_VUART);
    iodev_register_device(IODEV_NET, &iodev_net);

    announce(&cfg);
    printf("netproxy: up, waiting for a host on udp/%d\n", NS_PORT);
    disklog_flush();

    netproxy_up = true;
    return 0;
}

bool netproxy_active(void)
{
    return netproxy_up;
}

void netproxy_vuart_setup(void)
{
    if (netproxy_up)
        iodev_register_device(IODEV_USB_VUART, &iodev_net_vuart);
}

void netproxy_shutdown(void)
{
    if (!netproxy_up)
        return;

    iodev_flush(IODEV_NET);
    iodev_unregister_device(IODEV_NET);
    eth_shutdown();
    netproxy_up = false;
}
