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
#include "modbus_crc.h"

#include "usb_conf.h"
#include "usb_core.h"
#include "wk_system.h"
#include "usbd_int.h"
#include "cdc_class.h"
#include "cdc_desc.h"

#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0803F800 // last page
#define APP_CONF_VER        0x0200

#define FRAME_MAX           80


typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code; // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;  // 0: default, 1: load from flash
    uint8_t         do_reboot;
    bool            _reserved;
    bool            save_conf;

    bool            dbg_en;

    uint8_t         _keep[256]; // covers the areas in the app csa that need to be saved

    // end of flash
    bool            _end_save;

} csa_t; // config status area

extern csa_t csa;
extern const csa_t csa_dft;


int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

extern gpio_t led_g;
extern gpio_t sw1;

extern list_head_t frame_free_head;
extern cduart_dev_t d_dev;  // uart / usb

extern uint32_t end; // end of bss
extern uint32_t *bl_args;

void common_service_init(void);
void common_service_routine(void);

void app_main(void);
void load_conf(void);
int save_conf(void);
void try_jump_to_app(void);

#endif
