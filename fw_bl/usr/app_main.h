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

#include "cdnet_core.h"
#include "cd_debug.h"
#include "cdbus_uart.h"

#include "modbus_crc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0801F800 // last page
#define APP_CONF_VER        0x0200

#define FRAME_MAX           80
#define PACKET_MAX          80

typedef enum {
    LED_POWERON = 0,
    LED_WARN,
    LED_ERROR
} led_state_t;

typedef struct {
    list_node_t node;
    uint16_t    len;
    uint8_t     dat[512]; // CDC_DATA_HS_MAX_PACKET_SIZE
} cdc_buf_t;

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
    cdn_sockaddr_t  dbg_dst;

    uint8_t         _keep[256]; // covers the areas in the app csa that need to be saved

    // end of flash
    #define         _end_save usb_online

    bool            usb_online;

} csa_t; // config status area

extern csa_t csa;
extern const csa_t csa_dft;

extern regr_t csa_w_allow[]; // writable list
extern int csa_w_allow_num;

int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uart_t debug_uart;
extern gpio_t led_g;
extern gpio_t sw1;

extern list_head_t cdc_rx_free_head;
extern list_head_t cdc_tx_free_head;
extern list_head_t cdc_rx_head;
extern list_head_t cdc_tx_head;
extern cdc_buf_t *cdc_rx_buf;
extern cdc_buf_t *cdc_tx_buf;

extern list_head_t frame_free_head;

extern cduart_dev_t d_dev;  // uart / usb
extern cdn_ns_t dft_ns;

#define CIRC_BUF_SZ 1024
extern uint8_t circ_buf[];
extern uint32_t rd_pos;

extern uint32_t end; // end of bss
extern uint32_t *bl_args;

void bl_init(void);
void bl_routine(void);

void common_service_init(void);
void common_service_routine(void);

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);
void try_jump_to_app(void);

#endif
