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

#include "cd_utils.h"
#include "cd_list.h"
#include "cdctl_regs.h"
#include "cd_frame.h"


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


uint8_t cdctl_reg_r(uint8_t reg);
void cdctl_reg_w(uint8_t reg, uint8_t val);
void cdctl_set_baud_rate(uint32_t low, uint32_t high);
void cdctl_get_baud_rate(uint32_t *low, uint32_t *high);

void cdctl_routine(void);
void cdctl_dev_init(cdctl_cfg_t *init, spi_t *spi);

static inline void cdctl_flush(void)
{
    cdctl_reg_w(REG_RX_CTRL, BIT_RX_RST_ALL);
}

void cdctl_put_tx_frame(cd_frame_t *frame);


#define cdctl_state 0

extern list_head_t cdctl_rx_head;
extern list_head_t cdctl_tx_head;

extern uint32_t cdctl_rx_cnt;
extern uint32_t cdctl_tx_cnt;
extern uint32_t cdctl_rx_lost_cnt;
extern uint32_t cdctl_rx_error_cnt;
extern uint32_t cdctl_rx_break_cnt;
extern uint32_t cdctl_tx_cd_cnt;
extern uint32_t cdctl_tx_error_cnt;
extern uint32_t cdctl_rx_no_free_node_cnt;
extern uint32_t cdctl_rx_len_err_cnt;

#endif
