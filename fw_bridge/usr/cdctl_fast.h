/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CDCTL_H__
#define __CDCTL_H__

#include "cdbus.h"
#include "cdctl_regs.h"

#define CD_SPI          SPI1
#define CD_DMA_R        DMA1_Channel1
#define CD_DMA_W        DMA1_Channel2
#define CD_DMA          DMA1
#define CD_DMA_MASK     (2 << 0) // DMA_ISR.TCIF1

#define CD_SS_HIGH()    {CD_SS_GPIO_Port->BSRR = CD_SS_Pin;}
#define CD_SS_LOW()     {CD_SS_GPIO_Port->BRR = CD_SS_Pin;}
#define CD_INT_RD()     (CD_INT_GPIO_Port->IDR & CD_INT_Pin)


#define CDCTL_MASK (BIT_FLAG_RX_PENDING | BIT_FLAG_RX_LOST | BIT_FLAG_RX_ERROR |  \
                    BIT_FLAG_TX_CD | BIT_FLAG_TX_ERROR)

typedef struct {
    cd_dev_t        cd_dev;
    uint8_t         version;

    list_head_t     *free_head;
    list_head_t     rx_head;
    list_head_t     tx_head;

    bool            is_pending;

    uint32_t        rx_cnt;
    uint32_t        tx_cnt;
    uint32_t        rx_lost_cnt;
    uint32_t        rx_error_cnt;
    uint32_t        rx_break_cnt;
    uint32_t        tx_cd_cnt;
    uint32_t        tx_error_cnt;
    uint32_t        rx_no_free_node_cnt;
} cdctl_dev_t;

typedef struct {
    uint8_t         mac;
    uint32_t        baud_l;
    uint32_t        baud_h;
    uint8_t         filter_m[2];

    uint8_t         mode; // 0: Arbitration, 1: Break Sync
    uint16_t        tx_permit_len;
    uint16_t        max_idle_len;
    uint8_t         tx_pre_len;
} cdctl_cfg_t;

#define CDCTL_CFG_DFT(_mac) {   \
    .mac = _mac,                \
    .baud_l = 115200,           \
    .baud_h = 115200,           \
    .filter_m = { 0xff, 0xff }, \
    .mode = 0,                  \
    .tx_permit_len = 0x14,      \
    .max_idle_len = 0xc8,       \
    .tx_pre_len = 0x01          \
}


void cdctl_dev_init(cdctl_dev_t *dev, list_head_t *free_head, cdctl_cfg_t *init, spi_t *spi, gpio_t *rst_n);

cd_frame_t *cdctl_get_free_frame(cd_dev_t *cd_dev);
cd_frame_t *cdctl_get_rx_frame(cd_dev_t *cd_dev);
void cdctl_put_free_frame(cd_dev_t *cd_dev, cd_frame_t *frame);
void cdctl_put_tx_frame(cd_dev_t *cd_dev, cd_frame_t *frame);

void cdctl_routine(void);

#endif
