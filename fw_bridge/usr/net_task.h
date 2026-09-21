/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __NET_TASK_H__
#define __NET_TASK_H__

/*
 * The last 3 bytes of an ipv6 address inside our prefix are a CDNET address,
 * [type, net, mac]. Everything above those 3 bytes has to match the prefix.
 */
#define CDN_ADDR_L0         0x00    // level 0, local link
#define CDN_ADDR_LOCAL      0x10    // served by this firmware, not on the bus
#define CDN_ADDR_L1         0x80    // level 1, local link
#define CDN_ADDR_L1_NET     0xa0    // level 1, routed
#define CDN_ADDR_MULTI      0xf0    // level 1, multicast

#define IP_PFX_LEN          13      // a /104

#define ROUTER_MAC_NONE     0xff

typedef struct {
    uint32_t to_bus;        // udp datagrams forwarded onto the bus
    uint32_t to_pc;         // bus frames handed to the host
    uint32_t local;         // requests served by the local node
    uint32_t drop_fmt;      // could not be expressed as a cdnet frame
    uint32_t drop_big;      // payload larger than one bus frame can carry
    uint32_t drop_busy;     // bus tx backed up, or the frame pool ran dry
    uint32_t na_sent;
    uint32_t na_drop;
    uint32_t stall;         // times the host stopped taking datagrams
} net_cnt_t;

extern net_cnt_t net_cnt;
extern uint8_t net_dev_mac[6];

void net_init(void);
void net_poll(void);

// true while the network port is an endpoint on the bus: the usb device
// is up, the controller is the bus, and the host is taking what is sent
bool net_bus_active(void);
void net_bus_rx(cd_frame_t *frame);

// frames this side is holding for the host
uint32_t net_queued(void);
// give back the oldest frame waiting for the host, NULL if there is none
cd_frame_t *net_tx_evict(void);

// queue a datagram from the local node to the host, false if there is no room
bool net_local_tx(uint16_t sport, uint16_t dport, const uint8_t *dat, int len);

#endif
