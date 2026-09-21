/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "cd_utils.h"
#include "cd_list.h"
#include "cdbus.h"
#include "cdnet.h"
#include "cdctl_it.h"
#include "cdbus_uart.h"
#include "modbus_crc.h"
#include "uart_dma_wr.h"
#include "net_task.h"
#include "cdc_task.h"
#include "dbg_uart.h"

#include "wk_system.h"

#define BITS_SET(val, set)          ((val) |= (set))
#define BITS_CLR(val, clr)          ((val) &= ~(clr))
#define BITS_SET_CLR(val, set, clr) ((val) = ((val) | (set)) & ~(clr))

#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0803F800 // last page
#define APP_CONF_VER        0x0205

#define CPU_UID_ADDR        0x1FFFF7E8

#define FRAME_MAX           80
// keep this many frames in the free pool whatever happens. Everything in
// the main loop backs off at this line so that the one allocator that
// cannot, the cdctl receive interrupt, always has a frame: there is no way
// to ask the rs-485 bus to wait, and a frame it cannot receive into is a
// frame off the wire lost.
#define FRAME_RESERVE       10
// what a direction may hold, counted over every queue it uses. Neither may
// go past it, so the other always has the rest to work with; nothing is
// split evenly, whichever direction is busy gets everything the other one
// is not using.
#define FRAME_DIR_MAX       (FRAME_MAX * 4 / 5)


typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: all from flash, 2: partly from flash
    uint8_t         do_reboot;
    bool            _reserved0;
    bool            save_conf;

    uint8_t         dbg_en;         // bit 0: serial port, bit 1: ethernet port
    #define         _end_common _reserved1
    uint8_t         _reserved1[4];

    cdctl_cfg_t     bus_cfg;
    uint32_t        limit_baudrate0;    // baud_l cap in arbitration mode, sw2 off
    uint32_t        limit_baudrate1;    // ... sw2 on
    uint8_t         _reserved2[16];     // room for the bus settings to grow

    // the network settings start at 0x40, so what is above them can be
    // added to without moving them
    uint8_t         ip_pfx[IP_PFX_LEN]; // the /104 the bus is mapped into
    uint8_t         net;                // cdnet net number of the local link
    uint8_t         router_mac;         // 0xff: none, other nets are dropped
    uint8_t         _reserved3;
    uint16_t        port_offset;        // what the host's own udp port carries
                                        // above the cdnet port, 0: none

    // end of flash
    #define         _end_save usb_online

    bool            usb_online;

} csa_t; // config status area

// load_conf() relies on both: an older minor version is loaded up to the
// first, and everything from the second on is versioned by _reserved2
_Static_assert(offsetof(csa_t, _reserved2) == 0x30, "bus config moved");
_Static_assert(offsetof(csa_t, ip_pfx) == 0x40, "network config moved");

extern csa_t csa;
extern const csa_t csa_dft;


int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

extern list_head_t frame_free_head;
extern cdctl_dev_t r_dev;   // CDBUS

// the bus mode selects the hardware at boot: usart1 instead of the cdctl
// controller. raw_mode is that, minus the times the serial port is being
// used for configuration instead.
extern bool hw_raw;
extern bool raw_mode;

void frame_cache_put(list_head_t *head, cd_frame_t *frame);
// the network port asks before taking a frame for the bus, so that what
// cannot be sent yet is left where it came from instead of being read in
// and thrown away. The serial port asks frame_dir_ok() instead: its frames
// are parsed into a queue of its own and handed over from there.
bool bus_tx_ready(void);
void bus_tx(cd_frame_t *frame);
// false once this direction holds its share of the pool; the caller must
// not take another frame for it. to_host is bus -> host, else host -> bus.
bool frame_dir_ok(bool to_host);
uint32_t frame_dir_len(bool to_host);
// a frame taken back from a host-bound queue nobody is draining: the debug
// log in a serial port that is closed, the queue of a network host that
// has stopped taking datagrams. NULL if there is no such frame.
cd_frame_t *frame_dead_evict(void);
// true while the pool is above its reserve, taking dead frames back to get
// there; the to-bus intake asks before it takes a frame
bool frame_pool_ready(void);

extern uint32_t _estack, _Min_Stack_Size, _Noinit_Size; // linker script symbols

void comm_service_init(void);
void comm_service_poll(void);

// serve one request for the local node, return the reply length or -1 for none
int comm_service_handle(uint16_t port, const uint8_t *req, int req_len,
        uint8_t *rsp, int rsp_max);

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

#endif
