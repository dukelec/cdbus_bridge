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

// 256 bytes are enough for the CDCTL controller (without CRC)
// 258 bytes are enough for the UART controller (with CRC)
// allow smaller sizes to save memory
#ifndef CD_FRAME_SIZE
#define CD_FRAME_SIZE   256
#endif

typedef struct {
    list_node_t node;
    uint8_t     dat[CD_FRAME_SIZE];
} cd_frame_t;


#ifndef CDUART_IDLE_TIME
#define CDUART_IDLE_TIME    5 // ms
#endif
#ifndef CDUART_CRC
#define CDUART_CRC          crc16
#endif
#ifndef CDUART_CRC_SUB
#define CDUART_CRC_SUB      crc16_sub
#endif

#ifdef CD_IRQ_SAFE
#define cd_list_get(head)               list_get_entry_it(head, cd_frame_t)
#define cd_list_get_last(head)          list_get_last_entry_it(head, cd_frame_t)
#define cd_list_put(head, frm)          list_put_it(head, &(frm)->node)
#elif !defined(CD_USER_LIST)
#define cd_list_get(head)               list_get_entry(head, cd_frame_t)
#define cd_list_get_last(head)          list_get_last_entry(head, cd_frame_t)
#define cd_list_put(head, frm)          list_put(head, &(frm)->node)
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


static inline cd_frame_t *cduart_get_rx_frame(cduart_dev_t *dev)
{
    return cd_list_get(&dev->rx_head);
}

static inline void cduart_put_tx_frame(cduart_dev_t *dev, cd_frame_t *frame)
{
    cd_list_put(&dev->tx_head, frame);
}


void cduart_dev_init(cduart_dev_t *dev, list_head_t *free_head);
void cduart_rx_handle(cduart_dev_t *dev, const uint8_t *buf, unsigned len);

static inline void cduart_fill_crc(uint8_t *dat)
{
    uint16_t crc_val = CDUART_CRC(dat, dat[2] + 3);
    put_unaligned16(crc_val, dat + 3 + dat[2]);
}

#endif
