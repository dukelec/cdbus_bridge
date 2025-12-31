/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CDCTL_IT_H__
#define __CDCTL_IT_H__

#include "cd_utils.h"
#include "cd_list.h"
#include "cdctl_regs.h"
#include "cdctl_spi_wr.h"
#include "cd_frame.h"


typedef enum {
    CDCTL_RST = 0,

    CDCTL_IDLE,
    CDCTL_WAIT_TX_CLEAN,
    CDCTL_RD_FLAG,
    CDCTL_REG_W,

    CDCTL_RX_HEADER,
    CDCTL_RX_BODY,
    CDCTL_TX_FRAME
} cdctl_state_t;


typedef struct {
    uint8_t         mac;
    uint32_t        baud_l;
    uint32_t        baud_h;
    uint8_t         filter_m[2];

    uint8_t         mode; // 0: Traditional, 1: Arbitration, 2: Break Sync, 3: Full-duplex
    uint16_t        tx_permit_len;
    uint16_t        max_idle_len;
    uint8_t         tx_pre_len;
} cdctl_cfg_t;

#define CDCTL_CFG_DFT(_mac) {   \
    .mac = _mac,                \
    .baud_l = 115200,           \
    .baud_h = 115200,           \
    .filter_m = { 0xff, 0xff }, \
    .mode = 1,                  \
    .tx_permit_len = 0x14,      \
    .max_idle_len = 0xc8,       \
    .tx_pre_len = 0x01          \
}

void cdctl_dev_init(cdctl_cfg_t *init);

void cdctl_set_clk(uint32_t target_baud);
void cdctl_set_baud_rate(uint32_t low, uint32_t high);
void cdctl_get_baud_rate(uint32_t *low, uint32_t *high);

void cdctl_put_tx_frame(cd_frame_t *frame);

static inline void cdctl_flush(void)
{
    cdctl_reg_w(REG_RX_CTRL, BIT_RX_RST_ALL);
}

void cdctl_int_isr(void);
void cdctl_spi_isr(void);


extern volatile cdctl_state_t cdctl_state;

extern list_head_t cdctl_rx_head;
extern list_head_t cdctl_tx_head;

extern volatile uint32_t cdctl_rx_cnt;
extern volatile uint32_t cdctl_tx_cnt;
extern volatile uint32_t cdctl_rx_lost_cnt;
extern volatile uint32_t cdctl_rx_error_cnt;
extern volatile uint32_t cdctl_rx_break_cnt;
extern volatile uint32_t cdctl_tx_cd_cnt;
extern volatile uint32_t cdctl_tx_error_cnt;
extern volatile uint32_t cdctl_rx_no_free_node_cnt;
extern volatile uint32_t cdctl_rx_len_err_cnt;

#endif
