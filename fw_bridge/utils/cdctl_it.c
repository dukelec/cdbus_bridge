/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cdctl_it.h"
#include "app_main.h"

#define CDCTL_MASK (BIT_FLAG_RX_PENDING | BIT_FLAG_RX_LOST | BIT_FLAG_RX_ERROR |  \
                    BIT_FLAG_TX_CD | BIT_FLAG_TX_ERROR)

cdctl_state_t cdctl_state = CDCTL_RST;

list_head_t cdctl_rx_head = {0};
list_head_t cdctl_tx_head = {0};

static cd_frame_t *rx_frame = NULL;
static cd_frame_t *tx_frame = NULL;
static bool tx_wait_trigger = false;
static bool tx_buf_clean_mask = false;

uint32_t cdctl_rx_cnt = 0;
uint32_t cdctl_tx_cnt = 0;
uint32_t cdctl_rx_lost_cnt = 0;
uint32_t cdctl_rx_error_cnt = 0;
uint32_t cdctl_rx_break_cnt = 0;
uint32_t cdctl_tx_cd_cnt = 0;
uint32_t cdctl_tx_error_cnt = 0;
uint32_t cdctl_rx_no_free_node_cnt = 0;
uint32_t cdctl_rx_len_err_cnt = 0;


void cdctl_put_tx_frame(cd_frame_t *frame)
{
    uint32_t flags;
    local_irq_save(flags);
    cdctl_tx_cnt++;
    list_put(&cdctl_tx_head, &frame->node);
    if (cdctl_state == CDCTL_IDLE)
        cdctl_int_isr();
    local_irq_restore(flags);
}


void cdctl_set_baud_rate(uint32_t low, uint32_t high)
{
    uint16_t l, h;
    l = DIV_ROUND_CLOSEST(CDCTL_SYS_CLK, low) - 1;
    h = DIV_ROUND_CLOSEST(CDCTL_SYS_CLK, high) - 1;
    cdctl_reg_w(REG_DIV_LS_L, l & 0xff);
    cdctl_reg_w(REG_DIV_LS_H, l >> 8);
    cdctl_reg_w(REG_DIV_HS_L, h & 0xff);
    cdctl_reg_w(REG_DIV_HS_H, h >> 8);
    d_debug("cdctl: set baud rate: %lu %lu (%u %u)\n", low, high, l, h);
}

void cdctl_get_baud_rate(uint32_t *low, uint32_t *high)
{
    uint16_t l, h;
    l = cdctl_reg_r(REG_DIV_LS_L) | cdctl_reg_r(REG_DIV_LS_H) << 8;
    h = cdctl_reg_r(REG_DIV_HS_L) | cdctl_reg_r(REG_DIV_HS_H) << 8;
    *low = DIV_ROUND_CLOSEST(CDCTL_SYS_CLK, l + 1);
    *high = DIV_ROUND_CLOSEST(CDCTL_SYS_CLK, h + 1);
}


void cdctl_dev_init(cdctl_cfg_t *init)
{
    rx_frame = list_get_entry_it(&frame_free_head, cd_frame_t);

    d_info("cdctl: init...\n");
    uint8_t ver = cdctl_reg_r(REG_VERSION);
    d_info("cdctl: version: %02x\n", ver);

    cdctl_reg_w(REG_CLK_CTRL, 0x80); // soft reset
    d_info("cdctl: version after soft reset: %02x\n", cdctl_reg_r(REG_VERSION));

    // 12MHz / (0 + 2) * (48 + 2) / 2^1 = 150MHz
    d_info("pll_n: %02x\n", cdctl_reg_r(REG_PLL_N));
    cdctl_reg_w(REG_PLL_ML, 0x30); // 0x30: 48
    d_info("pll_ml: %02x\n", cdctl_reg_r(REG_PLL_ML));
    d_info("pll_ctrl: %02x\n", cdctl_reg_r(REG_PLL_CTRL));
    cdctl_reg_w(REG_PLL_CTRL, 0x10); // enable pll
    d_info("clk_status: %02x\n", cdctl_reg_r(REG_CLK_STATUS));
    cdctl_reg_w(REG_CLK_CTRL, 0x01); // select pll
    d_info("clk_status after select pll: %02x\n", cdctl_reg_r(REG_CLK_STATUS));
    d_info("version after select pll: %02x\n", cdctl_reg_r(REG_VERSION));

    cdctl_reg_w(REG_PIN_RE_CTRL, 0x10); // enable phy rx

    uint8_t setting = (cdctl_reg_r(REG_SETTING) & 0xf) | BIT_SETTING_TX_PUSH_PULL;
    setting |= init->mode == 1 ? BIT_SETTING_BREAK_SYNC : BIT_SETTING_ARBITRATE;
    cdctl_reg_w(REG_SETTING, setting);
    cdctl_reg_w(REG_FILTER, init->mac);
    cdctl_reg_w(REG_FILTER_M0, init->filter_m[0]);
    cdctl_reg_w(REG_FILTER_M1, init->filter_m[1]);
    cdctl_reg_w(REG_TX_PERMIT_LEN_L, init->tx_permit_len & 0xff);
    cdctl_reg_w(REG_TX_PERMIT_LEN_H, init->tx_permit_len >> 8);
    cdctl_reg_w(REG_MAX_IDLE_LEN_L, init->max_idle_len & 0xff);
    cdctl_reg_w(REG_MAX_IDLE_LEN_H, init->max_idle_len >> 8);
    cdctl_reg_w(REG_TX_PRE_LEN, init->tx_pre_len);
    cdctl_set_baud_rate(init->baud_l, init->baud_h);
    cdctl_flush();

    d_debug("cdctl: flags: %02x\n", cdctl_reg_r(REG_INT_FLAG));
    cdctl_reg_w(REG_INT_MASK, CDCTL_MASK);
    cdctl_state = CDCTL_IDLE;
}


// int_n pin interrupt isr
void cdctl_int_isr(void)
{
    if ((cdctl_state == CDCTL_IDLE) || cdctl_state == CDCTL_WAIT_TX_CLEAN) {
        cdctl_state = CDCTL_RD_FLAG;
        cdctl_reg_r_it(REG_INT_FLAG);
    }
}

// dma finish callback
void cdctl_spi_isr(void)
{
    // end of CDCTL_RD_FLAG
    if (cdctl_state == CDCTL_RD_FLAG) {
        uint8_t val = cdctl_buf[1];
        CD_SS_HIGH();

        if (val & (BIT_FLAG_RX_LOST | BIT_FLAG_RX_ERROR | BIT_FLAG_RX_BREAK | \
                BIT_FLAG_TX_CD | BIT_FLAG_TX_ERROR)) {
            if (val & BIT_FLAG_RX_LOST)
                cdctl_rx_lost_cnt++;
            if (val & BIT_FLAG_RX_ERROR)
                cdctl_rx_error_cnt++;
            if (val & BIT_FLAG_RX_BREAK)
                cdctl_rx_break_cnt++;
            if (val & BIT_FLAG_TX_CD)
                cdctl_tx_cd_cnt++;
            if (val & BIT_FLAG_TX_ERROR)
                cdctl_tx_error_cnt++;
        }

        // check for rx
        if (val & BIT_FLAG_RX_PENDING) {
            cdctl_state = CDCTL_RX_HEADER;
            uint8_t *buf = rx_frame->dat - 1;
            *buf = REG_RX; // borrow space from the "node" item
            CD_SS_LOW();
            cdctl_spi_wr_it(buf, buf, 4);
            return;
        }

        // check for tx
        if (tx_wait_trigger) {
            if (val & BIT_FLAG_TX_BUF_CLEAN) {
                tx_wait_trigger = false;
                cdctl_state = CDCTL_REG_W;
                cdctl_reg_w_it(REG_TX_CTRL, BIT_TX_START | BIT_TX_RST_POINTER);
                return;
            } else if (!tx_buf_clean_mask) {
                // enable tx_buf_clean irq
                tx_buf_clean_mask = true;
                cdctl_state = CDCTL_REG_W;
                cdctl_reg_w_it(REG_INT_MASK, CDCTL_MASK | BIT_FLAG_TX_BUF_CLEAN);
                return;
            }
        } else if (cdctl_tx_head.first) {
            tx_frame = list_get_entry(&cdctl_tx_head, cd_frame_t);
            uint8_t *buf = tx_frame->dat - 1;
            *buf = REG_TX | 0x80; // borrow space from the "node" item
            cdctl_state = CDCTL_TX_FRAME;
            CD_SS_LOW();
            cdctl_spi_wr_it(buf, buf, 4 + buf[3]);
            return;
        } else if (tx_buf_clean_mask) {
            tx_buf_clean_mask = false;
            cdctl_state = CDCTL_REG_W;
            cdctl_reg_w_it(REG_INT_MASK, CDCTL_MASK);
            return;
        }

        cdctl_state = tx_wait_trigger ? CDCTL_WAIT_TX_CLEAN : CDCTL_IDLE;
        if (!CD_INT_RD())
            cdctl_int_isr();
        return;
    }

    // end of write RX_CTRL, TX_CTRL, INT_MASK
    if (cdctl_state == CDCTL_REG_W) {
        CD_SS_HIGH();
        cdctl_state = CDCTL_RD_FLAG;
        cdctl_reg_r_it(REG_INT_FLAG);
        return;
    }

    // end of CDCTL_RX_HEADER
    if (cdctl_state == CDCTL_RX_HEADER) {
        cdctl_state = CDCTL_RX_BODY;
        if (rx_frame->dat[2] > min(CD_FRAME_SIZE - 3, 253)) {
            cdctl_rx_len_err_cnt++;
            cdctl_state = CDCTL_REG_W;
            cdctl_reg_w_it(REG_RX_CTRL, BIT_RX_CLR_PENDING | BIT_RX_RST_POINTER);
            return;
        }
        if (rx_frame->dat[2] != 0) {
            cdctl_spi_wr_it(rx_frame->dat + 3, rx_frame->dat + 3, rx_frame->dat[2]);
            return;
        } // no return
    }

    // end of CDCTL_RX_BODY
    if (cdctl_state == CDCTL_RX_BODY) {
        CD_SS_HIGH();
        cd_frame_t *frame = list_get_entry(&frame_free_head, cd_frame_t);
        if (frame) {
            list_put(&cdctl_rx_head, &rx_frame->node);
            rx_frame = frame;
            cdctl_rx_cnt++;
        } else {
            cdctl_rx_no_free_node_cnt++;
        }
        cdctl_state = CDCTL_REG_W;
        cdctl_reg_w_it(REG_RX_CTRL, BIT_RX_CLR_PENDING | BIT_RX_RST_POINTER);
        return;
    }

    // end of CDCTL_TX_FRAME
    if (cdctl_state == CDCTL_TX_FRAME) {
        CD_SS_HIGH();
        list_put(&frame_free_head, &tx_frame->node);
        tx_frame = NULL;
        tx_wait_trigger = true;

        cdctl_state = CDCTL_RD_FLAG;
        cdctl_reg_r_it(REG_INT_FLAG);
        return;
    }

    d_warn("cdctl: unexpected spi dma cb\n");
}
