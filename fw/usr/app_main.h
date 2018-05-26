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

typedef enum {
    APP_BRIDGE = 0,
    APP_RAW
} app_mode_t;

typedef enum {
    INTF_RS485 = 0,
    INTF_USB,
    INTF_TTL,
    INTF_RS232
} intf_idx_t;

typedef struct {
    uint16_t        magic_code; // 0xcdcd
    uint8_t         bl_wait; // run app after timeout (unit 0.1s), 0xff: never

    app_mode_t      mode;
    intf_idx_t      intf_idx;

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

extern app_conf_t app_conf;
extern cdnet_intf_t n_intf;

extern list_head_t cdc_rx_free_head;
extern list_head_t cdc_rx_head;
extern cdc_buf_t *cdc_rx_buf;

void p1_service(cdnet_packet_t *pkt);
void p2_service(cdnet_packet_t *pkt);
void p3_service(cdnet_packet_t *pkt);
void p11_service(cdnet_packet_t *pkt);

void load_conf(void);
void init_info_str(void);

#endif
