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
#include "cdbus_uart.h"
#include "cdctl.h"
#include "modbus_crc.h"

#include "usb_conf.h"
#include "usb_core.h"
#include "wk_system.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

#define BITS_SET(val, set)          ((val) |= (set))
#define BITS_CLR(val, clr)          ((val) &= ~(clr))
#define BITS_SET_CLR(val, set, clr) ((val) = ((val) | (set)) & ~(clr))

#define APP_CONF_ADDR       0x0803F800 // last page
#define APP_CONF_VER        0x0200

#define FRAME_MAX           80


typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code;     // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;      // 0: default, 1: all from flash, 2: partly from flash
    bool            do_reboot;
    bool            _reserved0;
    bool            save_conf;

    bool            dbg_en;
    #define         _end_common _reserved1
    uint8_t         _reserved1[4];

    cdctl_cfg_t     bus_cfg;
    uint8_t         _reserved2[4];
    uint32_t        ttl_baudrate;

    // end of flash
    #define         _end_save _reserved3

    uint8_t         _reserved3[1];
    bool            force_115200;

} csa_t; // config status area

extern csa_t csa;
extern const csa_t csa_dft;


int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

extern list_head_t frame_free_head;
extern cduart_dev_t d_dev;  // uart / usb

extern uint32_t end; // end of bss

void common_service_init(void);
void common_service_routine(void);

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

#endif
