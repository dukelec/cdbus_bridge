/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "cd_utils.h"
#include "cdnet_dispatch.h"
#include "cdbus_uart.h"
#include "cdctl_it.h"
#include "modbus_crc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "main.h"

typedef enum {
    APP_BRIDGE = 0,
    APP_RAW
} app_mode_t;

typedef enum {
    INTF_RS485 = 0,
    INTF_SER
} intf_idx_t;

typedef enum {
    SER_USB = 0,
    SER_TTL,
    SER_RS232
} ser_idx_t;

typedef struct {
    uint16_t        magic_code; // 0xcdcd
    uint8_t         bl_wait; // run app after timeout (unit 0.1s), 0xff: never

    app_mode_t      mode;
    ser_idx_t       ser_idx;

    uint8_t         rs485_net;
    uint8_t         rs485_mac;
    uint32_t        rs485_baudrate_low;
    uint32_t        rs485_baudrate_high;

    uint32_t        ttl_baudrate;
    uint32_t        rs232_baudrate;

    // raw
    bool            rpt_en;
    cd_sockaddr_t   rpt_sock;

} __attribute__((packed)) app_conf_t;

typedef struct {
    list_node_t node;
    uint16_t    len;
    uint8_t     dat[512]; // CDC_DATA_HS_MAX_PACKET_SIZE
} cdc_buf_t;


#define APP_CONF_ADDR       0x0801F800 // last page
#define RAW_SER_PORT        20


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
extern cdnet_intf_t n_intf; // CDNET

#define CIRC_BUF_SZ 1024
extern uint8_t circ_buf[];
extern uint32_t rd_pos;

extern app_conf_t app_conf;

void app_raw_init(void);
void app_raw(void);
void app_bridge_init(void);
void app_bridge(void);

void common_service_init(void);
void common_service_routine(void);

void app_main(void);
void load_conf_early(void);
void load_conf(void);
void save_conf(void);

#endif
