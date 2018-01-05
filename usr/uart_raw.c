/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "common.h"
#include "modbus_crc.h"
#include "uart_raw.h"

// member functions

list_node_t *ur_get_free_node(ur_intf_t *intf)
{
    uint32_t flags;
    list_node_t *node;
    local_irq_save(flags);
    node = list_get(intf->free_head);
    local_irq_restore(flags);
    return node;
}

list_node_t *ur_get_rx_node(ur_intf_t *intf)
{
    uint32_t flags;
    list_node_t *node;
    local_irq_save(flags);
    node = list_get(&intf->rx_head);
    local_irq_restore(flags);
    return node;
}

void ur_put_free_node(ur_intf_t *intf, list_node_t *node)
{
    uint32_t flags;
    local_irq_save(flags);
    list_put(intf->free_head, node);
    local_irq_restore(flags);
}

void ur_intf_init(ur_intf_t *intf, list_head_t *free_head, uart_t *uart)
{
    intf->free_head = free_head;
    intf->rx_head.first = NULL;
    intf->rx_head.last = NULL;

    intf->time_cnt = 0;
    intf->uart = uart;

    intf->rx_node = list_get(intf->free_head);
    ur_frame_t *pkt = container_of(intf->rx_node, ur_frame_t, node);
    pkt->len = 0;
}


static void ur_rx_finish(ur_intf_t *intf)
{
    list_node_t *node = list_get(intf->free_head);

    if (node != NULL) {
        list_put(&intf->rx_head, intf->rx_node);
        intf->rx_node = node;
    } else {
        // set rx_lost flag
    }

    ur_frame_t *pkt = container_of(intf->rx_node, ur_frame_t, node);
    pkt->len = 0;
}

// handlers

void ur_timer_handler(ur_intf_t *intf)
{
    intf->time_cnt++;
    ur_frame_t *pkt = container_of(intf->rx_node, ur_frame_t, node);

    if (pkt->len != 0 && intf->time_cnt > UR_IDLE_CNT)
        ur_rx_finish(intf);
}

void ur_rx_task(ur_intf_t *intf, uint8_t val)
{
    intf->time_cnt = 0;
    ur_frame_t *pkt = container_of(intf->rx_node, ur_frame_t, node);

    pkt->dat[pkt->len] = val;
    if (pkt->len++ == UR_DAT_SIZE)
        ur_rx_finish(intf);
}

