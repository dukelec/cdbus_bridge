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

#include "cdnet_dispatch.h"
#include "cd_debug.h"
#include "cdbus_uart.h"
#include "cdctl_it.h"

#include "modbus_crc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define APP_CONF_ADDR       0x0801F800 // last page
#define APP_CONF_VER        0x0103

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
    uint8_t         conf_from;      // 0: default, 1: load from flash
    bool            do_reboot;
    bool            _reserved_bl;   // keep_in_bl for bl
    bool            save_conf;

    cdctl_cfg_t     bus_cfg;
    bool            dbg_en;
    cdn_sockaddr_t  dbg_dst;
    #define         _end_common is_rs232

    bool            is_rs232;       // default ttl
    uint32_t        ttl_baudrate;
    uint32_t        rs232_baudrate;

    // end of flash
    #define         _end_save usb_online

    bool            usb_online;

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


extern USBD_HandleTypeDef hUsbDeviceFS;
extern uart_t *hw_uart;

extern list_head_t cdc_rx_free_head;
extern list_head_t cdc_tx_free_head;
extern list_head_t cdc_rx_head;
extern list_head_t cdc_tx_head;
extern cdc_buf_t *cdc_rx_buf;
extern cdc_buf_t *cdc_tx_buf;

extern list_head_t frame_free_head;

extern cdctl_dev_t r_dev;   // RS485
extern cduart_dev_t d_dev;  // uart / usb
extern cdn_ns_t dft_ns;

#define CIRC_BUF_SZ 1024
extern uint8_t circ_buf[];
extern uint32_t rd_pos;


void app_bridge_init(void);
void app_bridge(void);

void common_service_init(void);
void common_service_routine(void);

void app_main(void);
void load_conf(void);
int save_conf(void);
void csa_list_show(void);

#endif
