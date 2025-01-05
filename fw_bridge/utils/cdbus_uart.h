/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CDBUS_UART_H__
#define __CDBUS_UART_H__

#include "modbus_crc.h"
#include "cd_frame.h"

#ifndef CDUART_IDLE_TIME
#define CDUART_IDLE_TIME    5 // ms
#endif

#ifdef CDUART_IRQ_SAFE
#define cduart_frame_get(head)  list_get_entry_it(head, cd_frame_t)
#define cduart_list_put         list_put_it
#elif !defined(CDUART_USER_LIST)
#define cduart_frame_get(head)  list_get_entry(head, cd_frame_t)
#define cduart_list_put         list_put
#endif


typedef struct cduart_dev {
    list_head_t         *free_head;
    list_head_t         rx_head;
    list_head_t         tx_head;

    cd_frame_t          *rx_frame;  // init: != NULL
    uint16_t            rx_byte_cnt;
    uint16_t            rx_crc;
    bool                rx_drop;
    uint32_t            t_last;     // last receive time

    uint8_t             local_mac;
} cduart_dev_t;


static inline cd_frame_t *cduart_get_free_frame(cduart_dev_t *dev)
{
    return cduart_frame_get(dev->free_head);
}

static inline cd_frame_t *cduart_get_rx_frame(cduart_dev_t *dev)
{
    return cduart_frame_get(&dev->rx_head);
}

static inline void cduart_put_free_frame(cduart_dev_t *dev, cd_frame_t *frame)
{
    cduart_list_put(dev->free_head, &frame->node);
}

static inline void cduart_put_tx_frame(cduart_dev_t *dev, cd_frame_t *frame)
{
    cduart_list_put(&dev->tx_head, &frame->node);
}


void cduart_dev_init(cduart_dev_t *dev, list_head_t *free_head);
void cduart_rx_handle(cduart_dev_t *dev, const uint8_t *buf, unsigned len);

static inline void cduart_fill_crc(uint8_t *dat)
{
    uint16_t crc_val = crc16(dat, dat[2] + 3);
    dat[dat[2] + 3] = crc_val & 0xff;
    dat[dat[2] + 4] = crc_val >> 8;
}

#endif
