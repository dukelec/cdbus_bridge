/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

/*
 * NCM (usb ethernet) <-> CDBUS bridge.
 *
 * The host sees an ethernet port. Every address inside <prefix>/104 is a bus
 * address: the last 3 bytes are the CDNET address [type, net, mac] and the
 * udp ports are the CDNET ports, so talking to a device is plain udp and
 * needs no daemon on the host. One address is special, <prefix>10:0, which
 * is a node served right here and is how this bridge itself is configured.
 *
 * The host's own port is shifted by port_offset: it sends from, and is sent
 * to, the CDNET port plus the offset, while the ports on the far side are
 * left alone. A level 0 port is only 7 bits wide, so without this a program
 * wanting to talk level 0 would have to bind a port it needs root for. The
 * shift is done where the udp header is parsed and built, the local node
 * included, so one offset covers everything inside the prefix.
 *
 * Only what a point to point link actually needs is implemented. Neighbor
 * solicitations are answered for the whole prefix so the host can resolve
 * any bus address, and everything else it sends (mld, mdns, ipv4, ...) is
 * dropped. There is no ip stack, no timers, and nothing here allocates.
 *
 * Everything runs from the main loop, in the same context as tud_task(), so
 * none of the state below is shared with an interrupt. The frame lists are
 * the exception and they are irq safe already.
 */

#include "app_main.h"
#include "tusb.h"

#define ETH_HDR_LEN         14
#define IP6_HDR_LEN         40
#define UDP_HDR_LEN         8

#define ETHERTYPE_IPV6      0x86dd

#define IP6_NH_UDP          17
#define IP6_NH_ICMPV6       58

#define ICMPV6_NS           135
#define ICMPV6_NA           136

#define NA_BODY_LEN         32      // header + target + link layer option

// how many neighbor advertisements may wait for the endpoint at once; the
// host retransmits a solicitation we had no room for
#define NA_MAX              4

// bounded work per poll, both directions get a turn
#define TX_BURST            8
#define RX_BURST            4

// how long the endpoint may refuse the staged datagram before the host is
// taken to be gone. A host that is reading empties the endpoint within a
// millisecond; one that has not in this long is not reading, whether its
// interface is down or it has never bound the driver at all.
#define STALL_MS            200


net_cnt_t net_cnt = {0};
uint8_t net_dev_mac[6];

// the mac the host uses for its end of the link, announced in the descriptor
uint8_t tud_network_mac_address[6];

static list_head_t pc_tx_bus = {0};  // cdbus frames waiting for the host
static list_head_t pc_tx_loc = {0};  // datagrams from the local node

static struct {
    uint8_t tgt[16];
    uint8_t dst[16];
} na_q[NA_MAX];
static uint8_t na_head, na_tail, na_len;

enum {
    XMIT_UDP = 0,
    XMIT_NA
};

// the single datagram staged for the endpoint; tud_network_xmit_cb() builds
// it straight into the ntb, so it is never copied to a buffer of our own
static struct {
    bool        valid;
    bool        waiting;    // the endpoint refused it at least once
    uint32_t    t_wait;     // ... since when
    uint8_t     kind;
    uint16_t    size;       // resulting ethernet frame size
    cd_frame_t  *frm;       // released once sent, NULL if there is none

    uint8_t     s_addr[3];
    uint8_t     d_addr[3];
    uint16_t    sport;
    uint16_t    dport;
    const uint8_t *dat;
    uint16_t    len;

    uint8_t     na_tgt[16];
    uint8_t     na_dst[16];
} xo;


// the host has stopped taking datagrams, see STALL_MS. Nothing is queued
// for it while this holds, and one datagram stays staged as the probe that
// clears it: the moment the endpoint takes that one, the host is back.
static bool stalled = false;

static uint8_t rsp_buf[NET_LOCAL_MAX];


//--------------------------------------------------------------------
// checksum
//--------------------------------------------------------------------

/*
 * Chunks are summed as big endian 16 bit words, so every chunk but the last
 * one of a packet must have an even length.
 */
static uint32_t cksum_add(uint32_t sum, const uint8_t *p, uint16_t len)
{
    while (len > 1) {
        sum += get_unaligned_be16(p);
        p += 2;
        len -= 2;
    }
    if (len)
        sum += (uint32_t)p[0] << 8;
    return sum;
}

static uint16_t cksum_fold(uint32_t sum)
{
    while (sum >> 16)
        sum = (sum & 0xffff) + (sum >> 16);
    return ~sum & 0xffff;
}

static uint32_t cksum_pseudo(const uint8_t *s6, const uint8_t *d6,
        uint8_t nh, uint32_t len)
{
    uint32_t sum = 0;
    sum = cksum_add(sum, s6, 16);
    sum = cksum_add(sum, d6, 16);
    sum += (len >> 16) + (len & 0xffff);
    sum += nh;
    return sum;
}


//--------------------------------------------------------------------
// addresses
//--------------------------------------------------------------------

static bool pfx_match(const uint8_t *a6)
{
    return memcmp(a6, csa.ip_pfx, IP_PFX_LEN) == 0;
}

static void addr6_make(uint8_t *a6, const uint8_t *cdn_addr)
{
    memcpy(a6, csa.ip_pfx, IP_PFX_LEN);
    memcpy(a6 + IP_PFX_LEN, cdn_addr, 3);
}

// the address the host holds, which is also our identity on the bus: this
// bridge is transparent and does not take a bus address of its own
static void pc_addr(uint8_t *cdn_addr, uint8_t type)
{
    cdn_set_addr(cdn_addr, type, csa.net, csa.bus_cfg.mac);
}

static bool is_pc_addr(const uint8_t *a6)
{
    return (a6[13] == CDN_ADDR_L0 || a6[13] == CDN_ADDR_L1) &&
            a6[14] == csa.net && a6[15] == csa.bus_cfg.mac;
}

static bool addr6_is_zero(const uint8_t *a6)
{
    for (int i = 0; i < 16; i++) {
        if (a6[i])
            return false;
    }
    return true;
}

// an address we are willing to answer a neighbor solicitation for
static bool addr6_is_ours(const uint8_t *a6)
{
    if (!pfx_match(a6))
        return false;
    // the host owns its own address, answering for it fails its duplicate
    // address detection and it would then drop the address entirely
    if (is_pc_addr(a6))
        return false;

    switch (a6[13]) {
    case CDN_ADDR_L0:
    case CDN_ADDR_LOCAL:
    case CDN_ADDR_L1:
    case CDN_ADDR_L1_NET:
    case CDN_ADDR_MULTI:
        return true;
    default:
        return false;
    }
}

// what is left of a bus frame for the payload, once the cdnet header is in
static int max_payload(const cdn_pkt_t *pkt)
{
    int hdr_size = cdn_hdr_size_pkt(pkt);
    if (hdr_size <= 0)
        return -1;
    // frame: src, dst, len, hdr..., dat..., and the len byte must not overflow
    return min(CD_FRAME_SIZE - 5, 255) - hdr_size;
}


//--------------------------------------------------------------------
// host bound: build the ethernet frame straight into the ntb
//--------------------------------------------------------------------

static uint16_t build_udp(uint8_t *ip)
{
    uint8_t src6[16], dst6[16];
    uint16_t udp_len = UDP_HDR_LEN + xo.len;
    uint8_t *udp = ip + IP6_HDR_LEN;
    uint32_t sum;
    uint16_t ck;

    addr6_make(src6, xo.s_addr);
    addr6_make(dst6, xo.d_addr);

    memset(ip, 0, IP6_HDR_LEN);
    ip[0] = 0x60;
    put_unaligned_be16(udp_len, ip + 4);
    ip[6] = IP6_NH_UDP;
    ip[7] = 255;
    memcpy(ip + 8, src6, 16);
    memcpy(ip + 24, dst6, 16);

    put_unaligned_be16(xo.sport, udp);
    // xo.dport is a cdnet port, the host's side of it carries the offset
    put_unaligned_be16(xo.dport + csa.port_offset, udp + 2);
    put_unaligned_be16(udp_len, udp + 4);
    put_unaligned_be16(0, udp + 6);
    memcpy(udp + UDP_HDR_LEN, xo.dat, xo.len);

    sum = cksum_pseudo(src6, dst6, IP6_NH_UDP, udp_len);
    sum = cksum_add(sum, udp, udp_len);
    ck = cksum_fold(sum);
    if (ck == 0)
        ck = 0xffff; // ipv6 has no "checksum not computed" encoding
    put_unaligned_be16(ck, udp + 6);

    return IP6_HDR_LEN + udp_len;
}

static uint16_t build_na(uint8_t *ip)
{
    uint8_t *na = ip + IP6_HDR_LEN;
    uint32_t sum;

    memset(ip, 0, IP6_HDR_LEN);
    ip[0] = 0x60;
    put_unaligned_be16(NA_BODY_LEN, ip + 4);
    ip[6] = IP6_NH_ICMPV6;
    ip[7] = 255; // rfc 4861 requires this, the host drops anything less
    memcpy(ip + 8, xo.na_tgt, 16);
    memcpy(ip + 24, xo.na_dst, 16);

    memset(na, 0, NA_BODY_LEN);
    na[0] = ICMPV6_NA;
    na[4] = 0x60; // solicited | override
    memcpy(na + 8, xo.na_tgt, 16);
    na[24] = 2;   // target link layer address option
    na[25] = 1;   // 8 bytes, including these two
    memcpy(na + 26, net_dev_mac, 6);

    sum = cksum_pseudo(ip + 8, ip + 24, IP6_NH_ICMPV6, NA_BODY_LEN);
    sum = cksum_add(sum, na, NA_BODY_LEN);
    put_unaligned_be16(cksum_fold(sum), na + 2);

    return IP6_HDR_LEN + NA_BODY_LEN;
}

uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg)
{
    (void)ref;
    uint8_t *p = dst;

    memcpy(p, tud_network_mac_address, 6);
    memcpy(p + 6, net_dev_mac, 6);
    put_unaligned_be16(ETHERTYPE_IPV6, p + 12);
    p += ETH_HDR_LEN;

    if (arg == XMIT_NA)
        return ETH_HDR_LEN + build_na(p);
    return ETH_HDR_LEN + build_udp(p);
}

static void xmit_release(void)
{
    if (xo.frm) {
        cd_list_put(&frame_free_head, xo.frm);
        xo.frm = NULL;
    }
    xo.valid = false;
    xo.waiting = false;
}

// stage the next thing waiting for the host, skipping frames we cannot map
static bool pc_tx_pick(void)
{
    cd_frame_t *frm;

    while (true) {
        if (na_len) {
            memcpy(xo.na_tgt, na_q[na_tail].tgt, 16);
            memcpy(xo.na_dst, na_q[na_tail].dst, 16);
            na_tail = (na_tail + 1) % NA_MAX;
            na_len--;
            net_cnt.na_sent++;
            xo.kind = XMIT_NA;
            xo.frm = NULL;
            xo.size = ETH_HDR_LEN + IP6_HDR_LEN + NA_BODY_LEN;
            xo.valid = true;
            return true;
        }

        frm = cd_list_get(&pc_tx_loc);
        if (frm) {
            xo.kind = XMIT_UDP;
            xo.frm = frm;
            cdn_set_addr(xo.s_addr, CDN_ADDR_LOCAL, 0, 0);
            xo.sport = get_unaligned16(frm->dat);
            xo.dport = get_unaligned16(frm->dat + 2);
            memcpy(xo.d_addr, frm->dat + 4, 3);
            xo.len = frm->dat[7];
            xo.dat = frm->dat + 8;
            xo.size = ETH_HDR_LEN + IP6_HDR_LEN + UDP_HDR_LEN + xo.len;
            xo.valid = true;
            return true;
        }

        frm = cd_list_get(&pc_tx_bus);
        if (!frm)
            return false;

        cdn_pkt_t pkt = {0};
        pkt.frm = frm;
        pkt._l_net = csa.net;
        // drop what cannot become a udp datagram for the host: a frame that
        // does not parse, one too big to map, or a dst port with no room
        // left above it for the offset
        if (cdn_frame_r(&pkt) != 0 || pkt.len > CD_FRAME_SIZE - 5 ||
                pkt.dst.port + csa.port_offset > 0xffff) {
            cd_list_put(&frame_free_head, frm);
            net_cnt.drop_fmt++;
            continue;
        }

        xo.kind = XMIT_UDP;
        xo.frm = frm;
        memcpy(xo.s_addr, pkt.src.addr, 3);
        // a level 0 frame reaches the host on its level 0 address
        pc_addr(xo.d_addr, pkt.src.addr[0] == CDN_ADDR_L0 ?
                CDN_ADDR_L0 : CDN_ADDR_L1);
        xo.sport = pkt.src.port;
        xo.dport = pkt.dst.port;
        xo.len = pkt.len;
        xo.dat = pkt.dat;
        xo.size = ETH_HDR_LEN + IP6_HDR_LEN + UDP_HDR_LEN + pkt.len;
        xo.valid = true;
        return true;
    }
}

bool net_local_tx(const uint8_t *dst, uint16_t sport, uint16_t dport,
        const uint8_t *dat, int len)
{
    cd_frame_t *frm;

    if (len < 0 || len > NET_LOCAL_MAX)
        return false;
    frm = cd_list_get(&frame_free_head);
    if (!frm)
        return false;

    put_unaligned16(sport, frm->dat);
    put_unaligned16(dport, frm->dat + 2);
    memcpy(frm->dat + 4, dst, 3);
    frm->dat[7] = len;
    memcpy(frm->dat + 8, dat, len);
    frame_cache_put(&pc_tx_loc, frm);
    return true;
}


/*
 * The bridge's own debug text, from the local node to the host's level 0
 * address on port 9, the way a bus device reports its own. Only while the
 * port is up and being read: the ethernet port has no open to wait for the
 * way the serial port does, so there is nothing to queue the boot log
 * for, and it is printed before the port exists anyway.
 */
bool net_dbg_tx(const uint8_t *dat, int len)
{
    uint8_t dst[3];

    if (!net_bus_active())
        return false;
    pc_addr(dst, CDN_ADDR_L0);
    return net_local_tx(dst, 64, 9, dat, min(len, NET_LOCAL_MAX));
}


//--------------------------------------------------------------------
// bus bound
//--------------------------------------------------------------------

static bool bus_send(const uint8_t *dst6, uint16_t sport, uint16_t dport,
        const uint8_t *dat, uint16_t len)
{
    cdn_pkt_t pkt = {0};
    cd_frame_t *frm;
    int room;

    switch (dst6[13]) {
    case CDN_ADDR_L0:
        pc_addr(pkt.src.addr, CDN_ADDR_L0);
        cdn_set_addr(pkt.dst.addr, CDN_ADDR_L0, dst6[14], dst6[15]);
        pkt._d_mac = dst6[15];
        break;

    case CDN_ADDR_MULTI:
        pc_addr(pkt.src.addr, CDN_ADDR_L1_NET);
        cdn_set_addr(pkt.dst.addr, CDN_ADDR_MULTI, dst6[14], dst6[15]);
        pkt._d_mac = dst6[15];
        break;

    case CDN_ADDR_L1:
    case CDN_ADDR_L1_NET:
        // whether a level 1 packet stays on the link or goes to the router
        // is decided by the net byte, not by which of the two types is used
        if (dst6[14] == csa.net) {
            pc_addr(pkt.src.addr, CDN_ADDR_L1);
            cdn_set_addr(pkt.dst.addr, CDN_ADDR_L1, dst6[14], dst6[15]);
            pkt._d_mac = dst6[15];
        } else {
            if (csa.router_mac == ROUTER_MAC_NONE) {
                net_cnt.drop_fmt++;
                return true;
            }
            pc_addr(pkt.src.addr, CDN_ADDR_L1_NET);
            cdn_set_addr(pkt.dst.addr, CDN_ADDR_L1_NET, dst6[14], dst6[15]);
            pkt._d_mac = csa.router_mac;
        }
        break;

    default:
        net_cnt.drop_fmt++;
        return true;
    }

    pkt._s_mac = csa.bus_cfg.mac;
    pkt.src.port = sport;
    pkt.dst.port = dport;

    room = max_payload(&pkt);
    if (room < 0 || (int)len > room) {
        // there is no fragmentation, the sender has to split the message
        net_cnt.drop_big++;
        return true;
    }

    // back off and let the host offer it again, rather than read it in and
    // then have nowhere to put it
    if (!bus_tx_ready() || !frame_pool_ready())
        return false;
    frm = cd_list_get(&frame_free_head);
    if (!frm)
        return false;

    pkt.frm = frm;
    pkt.dat = frm->dat + 3 + cdn_hdr_size_pkt(&pkt);
    pkt.len = len;
    memcpy(pkt.dat, dat, len);

    if (cdn_frame_w(&pkt) != 0) {
        cd_list_put(&frame_free_head, frm);
        net_cnt.drop_fmt++;
        return true;
    }

    bus_tx(frm);
    net_cnt.to_bus++;
    return true;
}

static bool local_node_handle(const uint8_t *src6, uint16_t sport,
        uint16_t dport, const uint8_t *dat, uint16_t len)
{
    uint8_t reply_to[3];
    int rsp_len;

    // check for room before doing any work: returning false asks the host to
    // offer the same datagram again, and a flash write must not run twice.
    // The pool rather than the direction's share: net_local_tx() takes a
    // frame back from the direction if it has to, while a share that is
    // already spent would never free itself and this would defer for good.
    if (!frame_pool_ready())
        return false;

    // the host holds two addresses for the same node, level 0 and level 1,
    // and picks the level 0 one as the source for this destination; answer
    // the address the request came from, so that a host with only one of
    // them configured is served too
    if (pfx_match(src6))
        memcpy(reply_to, src6 + IP_PFX_LEN, 3);
    else
        pc_addr(reply_to, CDN_ADDR_L1);

    rsp_len = comm_service_handle(dport, dat, len, rsp_buf, sizeof(rsp_buf));
    net_cnt.local++;
    if (rsp_len >= 0)
        net_local_tx(reply_to, dport, sport, rsp_buf, rsp_len);
    return true;
}

static void ns_handle(const uint8_t *ip, uint16_t ip_len)
{
    const uint8_t *icmp = ip + IP6_HDR_LEN;
    const uint8_t *src6 = ip + 8;
    const uint8_t *tgt;

    if (ip_len < IP6_HDR_LEN + 24 || icmp[1] != 0)
        return;
    if (ip[7] != 255)
        return; // not from the local link, rfc 4861
    if (addr6_is_zero(src6))
        return; // duplicate address detection, never answer one

    tgt = icmp + 8;
    if (!addr6_is_ours(tgt))
        return;

    if (na_len >= NA_MAX) {
        net_cnt.na_drop++;
        return; // the host retransmits
    }
    memcpy(na_q[na_head].tgt, tgt, 16);
    memcpy(na_q[na_head].dst, src6, 16);
    na_head = (na_head + 1) % NA_MAX;
    na_len++;
}

// true once the datagram is dealt with, false to be offered it again later
static bool recv_one(const uint8_t *eth, uint16_t size)
{
    const uint8_t *ip = eth + ETH_HDR_LEN;
    const uint8_t *dst6;
    const uint8_t *udp;
    uint16_t ip_len, payload_len, udp_len, sport, dport;

    // in the raw mode the bus belongs to usart1 and the link is reported
    // down; whatever the host sends anyway has nowhere to go
    if (hw_raw)
        return true;
    if (size < ETH_HDR_LEN + IP6_HDR_LEN)
        return true;
    if (get_unaligned_be16(eth + 12) != ETHERTYPE_IPV6)
        return true;

    ip_len = size - ETH_HDR_LEN;
    if ((ip[0] >> 4) != 6)
        return true;
    payload_len = get_unaligned_be16(ip + 4);
    if (payload_len + IP6_HDR_LEN > ip_len)
        return true; // truncated
    ip_len = payload_len + IP6_HDR_LEN;

    if (ip[6] == IP6_NH_ICMPV6) {
        if (ip_len > IP6_HDR_LEN && ip[IP6_HDR_LEN] == ICMPV6_NS)
            ns_handle(ip, ip_len);
        return true;
    }
    if (ip[6] != IP6_NH_UDP)
        return true; // no extension headers, no tcp, nothing else to do

    dst6 = ip + 24;
    if (!pfx_match(dst6))
        return true;

    if (ip_len < IP6_HDR_LEN + UDP_HDR_LEN)
        return true;
    udp = ip + IP6_HDR_LEN;
    udp_len = get_unaligned_be16(udp + 4);
    if (udp_len < UDP_HDR_LEN || udp_len > ip_len - IP6_HDR_LEN)
        return true;

    sport = get_unaligned_be16(udp);
    dport = get_unaligned_be16(udp + 2);

    // the host sends from the cdnet port plus the offset; below the offset
    // there is no cdnet port to map it back to
    if (sport < csa.port_offset) {
        net_cnt.drop_fmt++;
        return true;
    }
    sport -= csa.port_offset;

    if (dst6[13] == CDN_ADDR_LOCAL)
        return local_node_handle(ip + 8, sport, dport, udp + UDP_HDR_LEN,
                udp_len - UDP_HDR_LEN);

    return bus_send(dst6, sport, dport, udp + UDP_HDR_LEN,
            udp_len - UDP_HDR_LEN);
}

/*
 * Returning false leaves the datagram where it is and asks to be offered it
 * again, which is what keeps the host's packet rather than dropping it: the
 * out endpoint is not re-armed while the block is still held, so the host
 * is made to wait instead. Every false below is a resource that frees up on
 * its own, so this cannot sit forever, and nothing else stops meanwhile,
 * the other direction included.
 */
bool tud_network_recv_cb(const uint8_t *src, uint16_t size)
{
    return recv_one(src, size);
}


//--------------------------------------------------------------------
// glue
//--------------------------------------------------------------------

void net_init(void)
{
    const uint8_t *uid = (const uint8_t *)CPU_UID_ADDR;

    // locally administered, and derived from the cpu id so two bridges on
    // one host never collide
    tud_network_mac_address[0] = 0x02;
    tud_network_mac_address[1] = 0xcd;
    for (int i = 0; i < 4; i++)
        tud_network_mac_address[2 + i] = uid[i] ^ uid[i + 4] ^ uid[i + 8];

    memcpy(net_dev_mac, tud_network_mac_address, 6);
    net_dev_mac[5] ^= 0x01; // our end of the link is not the host's end
}

bool net_bus_active(void)
{
    return tud_ready() && !hw_raw && !stalled;
}

// what the ncm driver reports to the host at enumeration, and again after
// every bus reset: in the raw mode there is nothing behind the port, so it
// has no carrier. The hardware choice is made at boot and never changes.
bool tud_network_default_link_state_cb(void)
{
    return !hw_raw;
}

uint32_t net_queued(void)
{
    return pc_tx_bus.len + pc_tx_loc.len + (xo.frm ? 1 : 0);
}

cd_frame_t *net_tx_evict(void)
{
    cd_frame_t *frm = cd_list_get(&pc_tx_bus);
    return frm ? frm : cd_list_get(&pc_tx_loc);
}

void net_bus_rx(cd_frame_t *frame)
{
    frame_cache_put(&pc_tx_bus, frame);
    net_cnt.to_pc++;
}

// what is queued behind a datagram the host would not take is going nowhere
static void pc_tx_drop_queued(void)
{
    cd_frame_t *frm;

    while ((frm = cd_list_get(&pc_tx_bus)) != NULL)
        cd_list_put(&frame_free_head, frm);
    while ((frm = cd_list_get(&pc_tx_loc)) != NULL)
        cd_list_put(&frame_free_head, frm);
}

void net_poll(void)
{
    int i;

    if (tud_ready() && !hw_raw) {
        for (i = 0; i < TX_BURST; i++) {
            if (!xo.valid && !pc_tx_pick())
                break;
            if (!tud_network_can_xmit(xo.size)) {
                // the endpoint is full. Give the host STALL_MS to empty
                // it; if it does not, it is not reading, and the queue is
                // only holding frames for nobody. Let them go, keep this
                // one staged as the probe, and take nothing more from the
                // bus until it has been sent.
                if (!xo.waiting) {
                    xo.waiting = true;
                    xo.t_wait = get_systick();
                } else if (!stalled && get_systick() - xo.t_wait > STALL_MS) {
                    stalled = true;
                    net_cnt.stall++;
                    pc_tx_drop_queued();
                }
                break;
            }
            tud_network_xmit(NULL, xo.kind); // calls tud_network_xmit_cb() now
            xmit_release();
            stalled = false;
        }
    } else {
        // hold nothing back, frame_cache_put() keeps the queues from growing
        // past what the pool can spare; and a host that went away is no
        // evidence about the one that plugs in next
        if (xo.valid)
            xmit_release();
        stalled = false;
    }

    // keep consuming what the host sends even when there is nothing to
    // bridge, so its tx path never sits blocked waiting for us
    if (tud_ready()) {
        for (i = 0; i < RX_BURST; i++)
            tud_network_recv_renew();
    }
}
