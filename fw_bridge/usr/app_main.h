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
#include "cdctl_fast.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0801F800 // last page
#define APP_CONF_VER        0x0201

#define FRAME_MAX           200
#define PACKET_MAX          80

#define CDC_TX_SIZE         512 // split into 64-byte chunks by hal layer
#define CDC_RX_SIZE         64  // usb full-speed bulk transfers up to 64 bytes

#define CIRC_BUF_SZ         1024

typedef struct {
    list_node_t node;
    uint16_t    len;
    uint8_t     dat[CDC_TX_SIZE];
} cdc_tx_buf_t;

typedef struct {
    list_node_t node;
    uint16_t    len;
    uint8_t     dat[CDC_RX_SIZE];
} cdc_rx_buf_t;

typedef enum {
    SER_USB = 0,
    SER_TTL,
    SER_RS232
} ser_idx_t;


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
    cdn_sockaddr_t  dbg_dst;
    #define         _end_common bus_cfg

    cdctl_cfg_t     bus_cfg;
    uint8_t         _reserved1[4];
    uint32_t        ttl_baudrate;

    // end of flash
    #define         _end_save usb_online

    bool            usb_online;
    bool            force_115200;

} csa_t; // config status area


typedef uint8_t (*hook_func_t)(uint16_t sub_offset, uint8_t len, uint8_t *dat);

typedef struct {
    regr_t          range;
    hook_func_t     before;
    hook_func_t     after;
} csa_hook_t;


extern csa_t csa;
extern const csa_t csa_dft;

extern regr_t csa_w_allow[]; // writable list
extern int csa_w_allow_num;

extern csa_hook_t csa_w_hook[];
extern int csa_w_hook_num;
extern csa_hook_t csa_r_hook[];
extern int csa_r_hook_num;

int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

extern USBD_HandleTypeDef hUsbDeviceFS;

extern list_head_t cdc_rx_free_head;
extern list_head_t cdc_tx_free_head;
extern list_head_t cdc_rx_head;
extern list_head_t cdc_tx_head;
extern cdc_rx_buf_t *cdc_rx_buf;
extern cdc_tx_buf_t *cdc_tx_buf;

extern list_head_t frame_free_head;

extern uart_t ttl_uart;
extern cdctl_dev_t r_dev;   // RS485
extern cduart_dev_t c_dev;  // usb / config mode
extern cduart_dev_t d_dev;  // usb / data mode
extern cdn_ns_t dft_ns;

extern int cdc_rate;
extern int app_mode;

extern uint8_t circ_buf[];
extern uint32_t rd_pos;

extern uint32_t end; // end of bss


void app_bridge(void);

void common_service_init(void);
void common_service_routine(void);

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

#endif
