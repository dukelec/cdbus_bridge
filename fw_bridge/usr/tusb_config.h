/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __TUSB_CONFIG_H__
#define __TUSB_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

// AT32F405 has two cores: OTG1 is full speed, OTG2 (OTGHS) has an on-chip
// high speed phy. tinyusb numbers them 0 and 1, we use the high speed one.
#define CFG_TUSB_MCU                OPT_MCU_AT32F402_405
#define CFG_TUSB_OS                 OPT_OS_NONE
#define BOARD_TUD_RHPORT            1
#define BOARD_TUD_MAX_SPEED         OPT_MODE_HIGH_SPEED

#define CFG_TUSB_RHPORT1_MODE       (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)

#define CFG_TUD_ENABLED             1
#define CFG_TUD_MAX_SPEED           OPT_MODE_HIGH_SPEED

// slave (fifo) mode rather than the core's internal dma: the bus never asks
// for more than a few Mbit/s, and slave mode has no alignment or cache
// requirements on the buffers we hand over
#define CFG_TUD_DWC2_SLAVE_ENABLE   1
#define CFG_TUD_DWC2_DMA_ENABLE     0

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE      64

//--------------------------------------------------------------------
// classes
//--------------------------------------------------------------------

#define CFG_TUD_CDC                 1
#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0
#define CFG_TUD_ECM_RNDIS           0
#define CFG_TUD_NCM                 1

// One bus frame is at most 258 bytes with its crc, and the raw mode passes
// through up to 255 at a time, so a buffer of 512 always takes a whole one.
#define CFG_TUD_CDC_RX_BUFSIZE      512
#define CFG_TUD_CDC_TX_BUFSIZE      512

// including the ethernet header; ipv6 needs at least 1280 on the link, so
// this cannot be lowered to the bus frame size, see the packet size limit
// in the Readme
#define CFG_TUD_NET_MTU             1514

// 2048 is the smallest size linux accepts. Our own datagrams never exceed
// ~320 bytes, but the host may hand us a full sized frame and it has to fit.
#define CFG_TUD_NCM_IN_NTB_MAX_SIZE     2048
#define CFG_TUD_NCM_OUT_NTB_MAX_SIZE    2048
// two towards the host, so the next datagram can be staged while one block
// is on the wire, rather than waiting a main loop turn for it to finish
#define CFG_TUD_NCM_IN_NTB_N            2
#define CFG_TUD_NCM_OUT_NTB_N           1

// The bus carries short request/response exchanges, so aggregation buys
// nothing and only adds latency. tinyusb never waits on a timer to fill an
// NTB, it sends whatever it has as soon as the endpoint is free, so the
// device to host direction is already immediate; IN merely caps what may
// share a block when several are waiting.
//
// OUT is what the host is told, and linux does wait on a timer, 400 us and
// more, for a block to fill up to it. 1 makes it send each datagram as it
// comes. The receive path parses whatever a host packs regardless, so one
// that ignores the limit still works.
#define CFG_TUD_NCM_IN_MAX_DATAGRAMS_PER_NTB    4
#define CFG_TUD_NCM_OUT_MAX_DATAGRAMS_PER_NTB   1

#ifdef __cplusplus
}
#endif

#endif
