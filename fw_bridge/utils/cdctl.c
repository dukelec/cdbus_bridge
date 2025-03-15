/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include <math.h>
#include "cdctl.h"
#include "app_main.h"
#include "cdctl_pll_cal.h"

static spi_t *cdctl_spi = NULL;

list_head_t cdctl_rx_head = {0};
list_head_t cdctl_tx_head = {0};

static uint32_t sysclk;
static bool is_pending = false;

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
    cd_list_put(&cdctl_tx_head, frame);
    cdctl_tx_cnt++;
}


uint8_t cdctl_reg_r(uint8_t reg)
{
    uint8_t dat = 0xff;
    spi_mem_read(cdctl_spi, reg, &dat, 1);
    return dat;
}

void cdctl_reg_w(uint8_t reg, uint8_t val)
{
    spi_mem_write(cdctl_spi, reg | 0x80, &val, 1);
}

static int cdctl_read_frame(cd_frame_t *frame)
{
    spi_mem_read(cdctl_spi, REG_RX, frame->dat, 3);
    if (frame->dat[2] > min(CD_FRAME_SIZE - 3, 253))
        return -1;
    spi_mem_read(cdctl_spi, REG_RX, frame->dat + 3, frame->dat[2]);
    return 0;
}

static void cdctl_write_frame(const cd_frame_t *frame)
{
    spi_mem_write(cdctl_spi, REG_TX | 0x80, frame->dat, frame->dat[2] + 3);
}


void cdctl_set_baud_rate(uint32_t low, uint32_t high)
{
    uint16_t l, h;
    l = DIV_ROUND_CLOSEST(sysclk, low) - 1;
    h = DIV_ROUND_CLOSEST(sysclk, high) - 1;
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
    *low = DIV_ROUND_CLOSEST(sysclk, l + 1);
    *high = DIV_ROUND_CLOSEST(sysclk, h + 1);
}

void cdctl_dev_init(cdctl_cfg_t *init, spi_t *spi)
{
    cdctl_spi = spi;

    d_info("cdctl: init...\n");
    uint8_t ver = cdctl_reg_r(REG_VERSION);
    d_info("cdctl: version: %02x\n", ver);

    cdctl_reg_w(REG_CLK_CTRL, 0x80); // soft reset
    d_info("cdctl: version after soft reset: %02x\n", cdctl_reg_r(REG_VERSION));

    sysclk = cdctl_sys_cal(init->baud_h);
    pllcfg_t pll = cdctl_pll_cal(CDCTL_OSC_CLK, sysclk);
    unsigned actual_freq = cdctl_pll_get(CDCTL_OSC_CLK, pll);
    d_info("cdctl: sysclk %ld, actual: %d\n", sysclk, actual_freq);
    sysclk = actual_freq;
    cdctl_reg_w(REG_PLL_N, pll.n);
    cdctl_reg_w(REG_PLL_ML, pll.m & 0xff);
    cdctl_reg_w(REG_PLL_OD_MH, (pll.d << 4) | (pll.m >> 8));
    d_info("pll_n: %02x, ml: %02x, od_mh: %02x\n",
            cdctl_reg_r(REG_PLL_N), cdctl_reg_r(REG_PLL_ML), cdctl_reg_r(REG_PLL_OD_MH));

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

    cdctl_get_baud_rate(&init->baud_l, &init->baud_h);
    d_debug("cdctl: get baud rate: %lu %lu\n", init->baud_l, init->baud_h);
    d_debug("cdctl: get filter(m): %02x (%02x %02x)\n",
            cdctl_reg_r(REG_FILTER), cdctl_reg_r(REG_FILTER_M0), cdctl_reg_r(REG_FILTER_M1));
    d_debug("cdctl: flags: %02x\n", cdctl_reg_r(REG_INT_FLAG));
}

// handlers


void cdctl_routine(void)
{
    uint8_t flags = cdctl_reg_r(REG_INT_FLAG);

    if (flags & BIT_FLAG_RX_LOST)
        cdctl_rx_lost_cnt++;
    if (flags & BIT_FLAG_RX_ERROR)
        cdctl_rx_error_cnt++;
    if (flags & BIT_FLAG_RX_BREAK)
        cdctl_rx_break_cnt++;
    if (flags & BIT_FLAG_TX_CD)
        cdctl_tx_cd_cnt++;
    if (flags & BIT_FLAG_TX_ERROR)
        cdctl_tx_error_cnt++;

    if (flags & BIT_FLAG_RX_PENDING) {
        // if get free list: copy to rx list
        cd_frame_t *frame = cd_list_get(&frame_free_head);
        if (frame) {
            int ret = cdctl_read_frame(frame);
            cdctl_reg_w(REG_RX_CTRL, BIT_RX_CLR_PENDING | BIT_RX_RST_POINTER);
#ifdef VERBOSE
            char pbuf[52];
            hex_dump_small(pbuf, frame->dat, frame->dat[2] + 3, 16);
            d_verbose("cdctl: -> [%s]\n", pbuf);
#endif
            if (ret) {
                d_error("cdctl: rx frame len err\n");
                cd_list_put(&frame_free_head, frame);
                cdctl_rx_len_err_cnt++;
            } else {
                cd_list_put(&cdctl_rx_head, frame);
                cdctl_rx_cnt++;
            }
        } else {
            d_error("cdctl: get rx, no free frame\n");
            cdctl_rx_no_free_node_cnt++;
        }
    }

    if (!is_pending) {
        if (cdctl_tx_head.first) {
            cd_frame_t *frame = cd_list_get(&cdctl_tx_head);
            cdctl_write_frame(frame);

            if (flags & BIT_FLAG_TX_BUF_CLEAN)
                cdctl_reg_w(REG_TX_CTRL, BIT_TX_START | BIT_TX_RST_POINTER);
            else
                is_pending = true;
#ifdef VERBOSE
            char pbuf[52];
            hex_dump_small(pbuf, frame->dat, frame->dat[2] + 3, 16);
            d_verbose("cdctl: <- [%s]%s\n", pbuf, dev->is_pending ? " (p)" : "");
#endif
            cd_list_put(&frame_free_head, frame);
        }
    } else {
        if (flags & BIT_FLAG_TX_BUF_CLEAN) {
            d_verbose("cdctl: trigger pending tx\n");
            cdctl_reg_w(REG_TX_CTRL, BIT_TX_START | BIT_TX_RST_POINTER);
            is_pending = false;
        }
    }
}
