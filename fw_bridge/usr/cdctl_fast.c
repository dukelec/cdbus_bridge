/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cdctl_fast.h"
#include "cd_debug.h"
#include "app_main.h"
#include "cdctl_pll_cal.h"


static inline void cdctl_spi_wr(const uint8_t *w_buf, uint8_t *r_buf, int len)
{
    CD_DMA_R->CCR &= ~DMA_CCR_EN;
    CD_DMA_R->CNDTR = len;
    CD_DMA_R->CMAR = (uint32_t)r_buf;
    CD_DMA_R->CCR |= DMA_CCR_EN;

    CD_DMA_W->CCR &= ~DMA_CCR_EN;
    CD_DMA_W->CNDTR = len;
    CD_DMA_W->CMAR = (uint32_t)w_buf;
    CD_DMA_W->CCR |= DMA_CCR_EN;

    while (!(CD_DMA->ISR & CD_DMA_MASK));
    CD_DMA->IFCR = CD_DMA_MASK;
}


static inline void cdctl_write_frame(cd_frame_t *frame)
{
    uint8_t *buf = frame->dat - 1;
    *buf = CDREG_TX | 0x80; // borrow space from the "node" item
    int len = buf[3] + 4;
    CD_SS_LOW();
    cdctl_spi_wr(buf, buf, len);
    CD_SS_HIGH();
}


static inline void cdctl_read_frame(cd_frame_t *frame)
{
    uint8_t *buf = frame->dat - 1;
    *buf = CDREG_RX; // borrow space from the "node" item

    CD_SS_LOW();
    cdctl_spi_wr(buf, buf, 4);

    int len = min(buf[3], 253);
    if (len)
        cdctl_spi_wr(buf + 4, buf + 4, len);
    CD_SS_HIGH();
}


static inline uint8_t cdctl_reg_r(uint8_t reg)
{
    uint16_t dat = 0xffff;
    uint8_t tbuf[2] = {reg};
    CD_SS_LOW();
    cdctl_spi_wr(tbuf, (uint8_t *)&dat, 2);
    CD_SS_HIGH();
    return dat >> 8;
}

static inline void cdctl_reg_w(uint8_t reg, uint8_t val)
{
    uint8_t tbuf[2] = {reg | 0x80, val};
    CD_SS_LOW();
    cdctl_spi_wr(tbuf, tbuf, 2);
    CD_SS_HIGH();
}

static inline uint8_t cdctl_read_flag(void)
{
    uint8_t flags = cdctl_reg_r(CDREG_INT_FLAG);

    if (flags & 0xdc) {
        if (flags & CDBIT_FLAG_RX_LOST)
            r_dev.rx_lost_cnt++;
        if (flags & CDBIT_FLAG_RX_ERROR)
            r_dev.rx_error_cnt++;
        if (flags & CDBIT_FLAG_TX_CD)
            r_dev.tx_cd_cnt++;
        if (flags & CDBIT_FLAG_TX_ERROR)
            r_dev.tx_error_cnt++;
    }

    return flags;
}

static inline void cdctl_flush(void)
{
    cdctl_reg_w(CDREG_RX_CTRL, CDBIT_RX_RST_ALL);
}


// member functions

cd_frame_t *cdctl_get_rx_frame(cd_dev_t *cd_dev)
{
    cdctl_dev_t *dev = container_of(cd_dev, cdctl_dev_t, cd_dev);
    return list_get_entry_it(&dev->rx_head, cd_frame_t);
}

void cdctl_put_tx_frame(cd_dev_t *cd_dev, cd_frame_t *frame)
{
    cdctl_dev_t *dev = container_of(cd_dev, cdctl_dev_t, cd_dev);
    list_put_it(&dev->tx_head, &frame->node);
}

void cdctl_set_baud_rate(uint32_t low, uint32_t high)
{
    uint16_t l, h;
    l = min(65535, max(2, DIV_ROUND_CLOSEST(r_dev.sysclk, low) - 1));
    h = min(65535, max(2, DIV_ROUND_CLOSEST(r_dev.sysclk, high) - 1));
    cdctl_reg_w(CDREG_DIV_LS_L, l & 0xff);
    cdctl_reg_w(CDREG_DIV_LS_H, l >> 8);
    cdctl_reg_w(CDREG_DIV_HS_L, h & 0xff);
    cdctl_reg_w(CDREG_DIV_HS_H, h >> 8);
    d_debug("cdctl: set baud rate: %u %u (%u %u)\n", low, high, l, h);
}

void cdctl_get_baud_rate(uint32_t *low, uint32_t *high)
{
    uint16_t l, h;
    l = cdctl_reg_r(CDREG_DIV_LS_L) | cdctl_reg_r(CDREG_DIV_LS_H) << 8;
    h = cdctl_reg_r(CDREG_DIV_HS_L) | cdctl_reg_r(CDREG_DIV_HS_H) << 8;
    *low = DIV_ROUND_CLOSEST(r_dev.sysclk, l + 1);
    *high = DIV_ROUND_CLOSEST(r_dev.sysclk, h + 1);
}

void cdctl_set_clk(uint32_t target_baud)
{
    cdctl_reg_w(CDREG_CLK_CTRL, 0x00); // select osc
    d_info("cdctl: version (clk: osc): %02x\n", cdctl_reg_r(CDREG_VERSION));

    r_dev.sysclk = cdctl_sys_cal(target_baud);
    pllcfg_t pll = cdctl_pll_cal(CDCTL_OSC_CLK, r_dev.sysclk);
    unsigned actual_freq = cdctl_pll_get(CDCTL_OSC_CLK, pll);
    d_info("cdctl: sysclk %ld, actual: %d\n", r_dev.sysclk, actual_freq);
    r_dev.sysclk = actual_freq;
    cdctl_reg_w(CDREG_PLL_N, pll.n);
    cdctl_reg_w(CDREG_PLL_ML, pll.m & 0xff);
    cdctl_reg_w(CDREG_PLL_OD_MH, (pll.d << 4) | (pll.m >> 8));
    d_info("pll_n: %02x, ml: %02x, od_mh: %02x\n",
            cdctl_reg_r(CDREG_PLL_N), cdctl_reg_r(CDREG_PLL_ML), cdctl_reg_r(CDREG_PLL_OD_MH));

    d_info("pll_ctrl: %02x\n", cdctl_reg_r(CDREG_PLL_CTRL));
    cdctl_reg_w(CDREG_PLL_CTRL, 0x10); // enable pll
    d_info("clk_status: %02x\n", cdctl_reg_r(CDREG_CLK_STATUS));
    cdctl_reg_w(CDREG_CLK_CTRL, 0x01); // select pll
    d_info("clk_status (clk: pll): %02x\n", cdctl_reg_r(CDREG_CLK_STATUS));
    d_info("version (clk: pll): %02x\n", cdctl_reg_r(CDREG_VERSION));
}


void cdctl_dev_init(cdctl_dev_t *dev, list_head_t *free_head, cdctl_cfg_t *init, spi_t *unused)
{
    SET_BIT(CD_SPI->CR1, SPI_CR1_SPE); // enable spi
    SET_BIT(CD_SPI->CR2, SPI_CR2_RXDMAEN);
    SET_BIT(CD_SPI->CR2, SPI_CR2_TXDMAEN);
    //CD_DMA_R->CCR &= ~DMA_CCR_EN;
    //CD_DMA_W->CCR &= ~DMA_CCR_EN;
    CD_DMA_R->CPAR = (uint32_t)&CD_SPI->DR;
    CD_DMA_W->CPAR = (uint32_t)&CD_SPI->DR;

    dev->free_head = free_head;
    dev->cd_dev.get_rx_frame = cdctl_get_rx_frame;
    dev->cd_dev.put_tx_frame = cdctl_put_tx_frame;

#ifdef USE_DYNAMIC_INIT
    list_head_init(&dev->rx_head);
    list_head_init(&dev->tx_head);
    dev->is_pending = false;
#endif

    d_info("cdctl: init...\n");

    uint8_t ver = cdctl_reg_r(CDREG_VERSION);
    d_info("cdctl: version: %02x\n", ver);

    cdctl_reg_w(CDREG_CLK_CTRL, 0x80); // soft reset
    cdctl_set_clk(init->baud_h);
    cdctl_reg_w(CDREG_PIN_RE_CTRL, 0x10); // enable phy rx

    uint8_t setting = (cdctl_reg_r(CDREG_SETTING) & 0xf) | CDBIT_SETTING_TX_PUSH_PULL;
    setting |= init->mode == 1 ? CDBIT_SETTING_BREAK_SYNC : CDBIT_SETTING_ARBITRATE;
    cdctl_reg_w(CDREG_SETTING, setting);
    cdctl_reg_w(CDREG_FILTER, init->mac);
    cdctl_reg_w(CDREG_FILTER_M0, init->filter_m[0]);
    cdctl_reg_w(CDREG_FILTER_M1, init->filter_m[1]);
    cdctl_reg_w(CDREG_TX_PERMIT_LEN_L, init->tx_permit_len & 0xff);
    cdctl_reg_w(CDREG_TX_PERMIT_LEN_H, init->tx_permit_len >> 8);
    cdctl_reg_w(CDREG_MAX_IDLE_LEN_L, init->max_idle_len & 0xff);
    cdctl_reg_w(CDREG_MAX_IDLE_LEN_H, init->max_idle_len >> 8);
    cdctl_reg_w(CDREG_TX_PRE_LEN, init->tx_pre_len);
    cdctl_set_baud_rate(init->baud_l, init->baud_h);
    cdctl_flush();

    cdctl_get_baud_rate(&init->baud_l, &init->baud_h);
    d_debug("cdctl: get baud rate: %lu %lu\n", init->baud_l, init->baud_h);
    d_debug("cdctl: get filter(m): %02x (%02x %02x)\n",
            cdctl_reg_r(CDREG_FILTER), cdctl_reg_r(CDREG_FILTER_M0), cdctl_reg_r(CDREG_FILTER_M1));
    d_debug("cdctl: flags: %02x\n", cdctl_reg_r(CDREG_INT_FLAG));
    cdctl_reg_w(CDREG_INT_MASK, CDCTL_MASK);
}


void cdctl_routine(void)
{
    uint8_t flags;
    uint32_t cpu_flags;

    if (!r_dev.is_pending && r_dev.tx_head.first) {
        local_irq_save(cpu_flags);
        cd_frame_t *frame = list_get_entry(&r_dev.tx_head, cd_frame_t);
        cdctl_write_frame(frame);

        flags = cdctl_read_flag();
        if (flags & CDBIT_FLAG_TX_BUF_CLEAN) {
            cdctl_reg_w(CDREG_TX_CTRL | 0x80, CDBIT_TX_START | CDBIT_TX_RST_POINTER);
            r_dev.tx_cnt++;
        } else {
            r_dev.is_pending = true;
        }
        list_put(r_dev.free_head, &frame->node); // frame_free_head requires irq safe
        local_irq_restore(cpu_flags);
    }

    if (r_dev.is_pending) {
        local_irq_save(cpu_flags);
        flags = cdctl_read_flag();
        if (flags & CDBIT_FLAG_TX_BUF_CLEAN) {
            cdctl_reg_w(CDREG_TX_CTRL | 0x80, CDBIT_TX_START | CDBIT_TX_RST_POINTER);
            r_dev.tx_cnt++;
            r_dev.is_pending = false;
        }
        local_irq_restore(cpu_flags);
    }
}


void EXTI0_1_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_FALLING_IT(CD_INT_Pin);

    do {
        uint8_t flags = cdctl_read_flag();

        if (flags & CDBIT_FLAG_RX_PENDING) {
            // if get free list: copy to rx list
            cd_frame_t *frame = list_get_entry(&frame_free_head, cd_frame_t);
            if (frame) {
                cdctl_read_frame(frame);
                list_put(&r_dev.rx_head, &frame->node);
                r_dev.rx_cnt++;
            } else {
                r_dev.rx_no_free_node_cnt++;
            }
            cdctl_reg_w(CDREG_RX_CTRL | 0x80, CDBIT_RX_CLR_PENDING | CDBIT_RX_RST_POINTER);
        }
    } while (!CD_INT_RD());
}
