/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#ifndef __UART_RAW_H__
#define __UART_RAW_H__

#include "common.h"
#include "cdnet.h"


#define UR_IDLE_CNT     5
#define UR_DAT_SIZE     200

typedef struct {
    list_node_t node;
    uint8_t     len;
    uint8_t     dat[UR_DAT_SIZE];
} ur_frame_t;

typedef struct {
    list_head_t *free_head;
    list_head_t rx_head;

    int         time_cnt;
    uart_t      *uart;

    list_node_t *rx_node; // init: != NULL
} ur_intf_t;


void ur_intf_init(ur_intf_t *intf, list_head_t *free_head, uart_t *uart);
list_node_t *ur_get_free_node(ur_intf_t *intf);
list_node_t *ur_get_rx_node(ur_intf_t *intf);
void ur_put_free_node(ur_intf_t *intf, list_node_t *node);

void ur_timer_handler(ur_intf_t *intf);
void ur_rx_task(ur_intf_t *intf, uint8_t val);

#endif

