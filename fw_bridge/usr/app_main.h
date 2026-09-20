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

#include "wk_system.h"

#define BITS_SET(val, set)          ((val) |= (set))
#define BITS_CLR(val, clr)          ((val) &= ~(clr))
#define BITS_SET_CLR(val, set, clr) ((val) = ((val) | (set)) & ~(clr))

#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0803F800 // last page
#define APP_CONF_VER        0x0204

#define CPU_UID_ADDR        0x1FFFF7E8

#define FRAME_MAX           80
// keep this many frames in the free pool for the rx paths, so caching frames
// for an offline host can never starve them
#define FRAME_RESERVE       10
// and cap each host bound queue, so one host that stopped reading cannot
// take the pool away from the other one
#define CACHE_MAX           20
// bus frames allowed to pile up in the controller's tx queue before we start
// dropping, so a jammed bus cannot eat the pool either
#define BUS_TX_MAX          16


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

    bool            dbg_en;
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
    uint8_t         _reserved3[2];

    // end of flash
    #define         _end_save usb_online

    bool            usb_online;

} csa_t; // config status area

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
// hand a frame to the bus, false if its tx queue is already backed up
bool bus_tx(cd_frame_t *frame);

extern uint32_t _estack, _Min_Stack_Size; // linker script symbols

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
