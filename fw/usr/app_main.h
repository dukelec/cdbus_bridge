/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "common.h"
#include "cdnet.h"
#include "cdbus_uart.h"
#include "cdctl_bx_it.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "modbus_crc.h"
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

    cdnet_addr_t    rs485_addr;
    uint32_t        rs485_baudrate_low;
    uint32_t        rs485_baudrate_high;

    uint32_t        ttl_baudrate;
    uint32_t        rs232_baudrate;

    // raw
    bool            rpt_en;
    cdnet_level_t   rpt_pkt_level;
    bool            rpt_seq;
    cdnet_multi_t   rpt_multi;
    uint8_t         rpt_mac;
    cdnet_addr_t    rpt_addr;

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
extern list_head_t packet_free_head;

extern cdctl_intf_t r_intf;   // RS485
extern cdnet_intf_t n_intf;   // CDNET

#define CIRC_BUF_SZ 1024
extern uint8_t circ_buf[];
extern uint32_t rd_pos;

extern app_conf_t app_conf;

void app_raw_init(void);
void app_raw(void);
void app_bridge_init(void);
void app_bridge(void);

void p1_service(cdnet_packet_t *pkt);
void p3_service_for_raw(cdnet_packet_t *pkt);
void p3_service_for_bridge(cdnet_packet_t *pkt);
void p10_service(cdnet_packet_t *pkt);
void p11_service(cdnet_packet_t *pkt);

void load_conf(void);
void save_conf(void);
void init_info_str(void);

#endif
