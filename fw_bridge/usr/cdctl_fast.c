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

extern SPI_HandleTypeDef hspi1;

//static uint8_t dummy_rx[256 + 1];


static inline void cdctl_write_frame_fast(cd_frame_t *frame)
{
    uint8_t *buf = frame->dat - 1;
    *buf = REG_TX | 0x80; // borrow space from the "node" item
    int len = buf[3] + 4;
    GPIOA->BRR = 0x0010; // PA4 = 0

    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CNDTR = len;
    //DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel3->CMAR = (uint32_t)buf;
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    // clear spi rx fifo
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CNDTR = len;
    //DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel2->CMAR = (uint32_t)buf; //dummy_rx;
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    while (SPI1->SR & SPI_FLAG_BSY);
    GPIOA->BSRR = 0x0010;  // PA4 = 1
}


static inline void cdctl_read_frame_fast(cd_frame_t *frame)
{
    uint8_t *buf = frame->dat - 1;
    *buf = REG_RX; // borrow space from the "node" item

    GPIOA->BRR = 0x0010; // PA4 = 0

    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CNDTR = 4;
    //DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel3->CMAR = (uint32_t)buf;
    DMA1_Channel3->CCR |= DMA_CCR_EN;

    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CNDTR = 4;
    //DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel2->CMAR = (uint32_t)buf;
    DMA1_Channel2->CCR |= DMA_CCR_EN;

    while (SPI1->SR & SPI_FLAG_BSY);

    __DMB();

    int len = min(buf[3], 253);
    if (len) {
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
        DMA1_Channel3->CNDTR = len;
        //DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
        DMA1_Channel3->CMAR = (uint32_t)(buf + 4);
        DMA1_Channel3->CCR |= DMA_CCR_EN;

        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        DMA1_Channel2->CNDTR = len;
        //DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
        DMA1_Channel2->CMAR = (uint32_t)(buf + 4);
        DMA1_Channel2->CCR |= DMA_CCR_EN;

        while (SPI1->SR & SPI_FLAG_BSY);
    }

    GPIOA->BSRR = 0x0010;  // PA4 = 1
}


static inline uint8_t cdctl_read_reg_fast(uint8_t reg)
{
    uint8_t dat;
    GPIOA->BRR = 0x0010; // PA4 = 0

    *((volatile uint8_t *)&SPI1->DR) = reg;
    *((volatile uint8_t *)&SPI1->DR) = 0x00;

    while (SPI1->SR & SPI_FLAG_BSY);
    GPIOA->BSRR = 0x0010;  // PA4 = 1

    volatile uint8_t tmp = *((volatile uint8_t *)&SPI1->DR); (void)tmp;
    dat = *((volatile uint8_t *)&SPI1->DR);
    return dat;
}

static inline void cdctl_write_reg_fast(uint8_t reg, uint8_t val)
{
    GPIOA->BRR = 0x0010; // PA4 = 0

    *((volatile uint8_t *)&SPI1->DR) = reg; //  | 0x80;
    *((volatile uint8_t *)&SPI1->DR) = val;

    while (SPI1->SR & SPI_FLAG_BSY);

    GPIOA->BSRR = 0x0010;  // PA4 = 1

    // clear rx fifo
    volatile uint8_t tmp = SPI1->DR;
    tmp = SPI1->DR; (void)tmp;
}

static inline uint8_t cdctl_read_flags_fast(void)
{
    uint8_t flags = cdctl_read_reg_fast(REG_INT_FLAG);

    if (flags & 0xdc) {
        if (flags & BIT_FLAG_RX_LOST)
            r_dev.rx_lost_cnt++;
        if (flags & BIT_FLAG_RX_ERROR)
            r_dev.rx_error_cnt++;
        if (flags & BIT_FLAG_TX_CD)
            r_dev.tx_cd_cnt++;
        if (flags & BIT_FLAG_TX_ERROR)
            r_dev.tx_error_cnt++;
    }

    return flags;
}


uint8_t cdctl_read_reg(cdctl_dev_t *dev, uint8_t reg)
{
    uint8_t dat = 0xff;
    spi_mem_read(dev->spi, reg, &dat, 1);
    return dat;
}

void cdctl_write_reg(cdctl_dev_t *dev, uint8_t reg, uint8_t val)
{
    spi_mem_write(dev->spi, reg | 0x80, &val, 1);
}

#if 0
static void cdctl_read_frame(cdctl_dev_t *dev, cd_frame_t *frame)
{
    spi_mem_read(dev->spi, REG_RX, frame->dat, 3);
    spi_mem_read(dev->spi, REG_RX, frame->dat + 3, frame->dat[2]);
}

static void cdctl_write_frame(cdctl_dev_t *dev, const cd_frame_t *frame)
{
    spi_mem_write(dev->spi, REG_TX | 0x80, frame->dat, frame->dat[2] + 3);
}
#endif

// member functions

cd_frame_t *cdctl_get_free_frame(cd_dev_t *cd_dev)
{
    cdctl_dev_t *dev = container_of(cd_dev, cdctl_dev_t, cd_dev);
    return list_get_entry(dev->free_head, cd_frame_t);
}

cd_frame_t *cdctl_get_rx_frame(cd_dev_t *cd_dev)
{
    cdctl_dev_t *dev = container_of(cd_dev, cdctl_dev_t, cd_dev);
    return list_get_entry(&dev->rx_head, cd_frame_t);
}

void cdctl_put_free_frame(cd_dev_t *cd_dev, cd_frame_t *frame)
{
    cdctl_dev_t *dev = container_of(cd_dev, cdctl_dev_t, cd_dev);
    list_put(dev->free_head, &frame->node);
}

void cdctl_put_tx_frame(cd_dev_t *cd_dev, cd_frame_t *frame)
{
    cdctl_dev_t *dev = container_of(cd_dev, cdctl_dev_t, cd_dev);
    list_put(&dev->tx_head, &frame->node);
}

void cdctl_set_baud_rate(cdctl_dev_t *dev, uint32_t low, uint32_t high)
{
    uint16_t l, h;
    l = DIV_ROUND_CLOSEST(CDCTL_SYS_CLK, low) - 1;
    h = DIV_ROUND_CLOSEST(CDCTL_SYS_CLK, high) - 1;
    cdctl_write_reg(dev, REG_DIV_LS_L, l & 0xff);
    cdctl_write_reg(dev, REG_DIV_LS_H, l >> 8);
    cdctl_write_reg(dev, REG_DIV_HS_L, h & 0xff);
    cdctl_write_reg(dev, REG_DIV_HS_H, h >> 8);
    dn_debug(dev->name, "set baud rate: %u %u (%u %u)\n", low, high, l, h);
}

void cdctl_get_baud_rate(cdctl_dev_t *dev, uint32_t *low, uint32_t *high)
{
    uint16_t l, h;
    l = cdctl_read_reg(dev, REG_DIV_LS_L) | cdctl_read_reg(dev, REG_DIV_LS_H) << 8;
    h = cdctl_read_reg(dev, REG_DIV_HS_L) | cdctl_read_reg(dev, REG_DIV_HS_H) << 8;
    *low = DIV_ROUND_CLOSEST(CDCTL_SYS_CLK, l + 1);
    *high = DIV_ROUND_CLOSEST(CDCTL_SYS_CLK, h + 1);
}

void cdctl_dev_init(cdctl_dev_t *dev, list_head_t *free_head, cdctl_cfg_t *init,
        spi_t *spi, gpio_t *rst_n)
{
    if (!dev->name)
        dev->name = "cdctl";
    dev->free_head = free_head;
    dev->cd_dev.get_free_frame = cdctl_get_free_frame;
    dev->cd_dev.get_rx_frame = cdctl_get_rx_frame;
    dev->cd_dev.put_free_frame = cdctl_put_free_frame;
    dev->cd_dev.put_tx_frame = cdctl_put_tx_frame;

#ifdef USE_DYNAMIC_INIT
    list_head_init(&dev->rx_head);
    list_head_init(&dev->tx_head);
    dev->is_pending = false;
#endif

    dev->spi = spi;
    dev->rst_n = rst_n;

    dn_info(dev->name, "init...\n");
    if (rst_n) {
        gpio_set_value(rst_n, 0);
        delay_systick(2000/SYSTICK_US_DIV);
        gpio_set_value(rst_n, 1);
        delay_systick(2000/SYSTICK_US_DIV);
    }

    // the fpga has to be read multiple times, the asic does not
    uint8_t last_ver = 0xff;
    uint8_t same_cnt = 0;
    while (true) {
        uint8_t ver = cdctl_read_reg(dev, REG_VERSION);
        if (ver != 0x00 && ver != 0xff && ver == last_ver) {
            if (same_cnt++ > 10)
                break;
        } else {
            last_ver = ver;
            same_cnt = 0;
        }
        debug_flush(false);
    }
    dn_info(dev->name, "version: %02x\n", last_ver);
    dev->version = last_ver;
    dev->_clr_flag = last_ver >= 0x0e ? false : true;

    if (dev->version >= 0x10) { // asic
        cdctl_write_reg(dev, REG_CLK_CTRL, 0x80); // soft reset
        dn_info(dev->name, "version after soft reset: %02x\n", cdctl_read_reg(dev, REG_VERSION));
#ifndef CDCTL_AVOID_PIN_RE
        cdctl_write_reg(dev, REG_PIN_RE_CTRL, 0x10); // enable phy rx
#endif
    } else {
        d_info("fallback to cdctl-b1 module\n");
        CDCTL_SYS_CLK = 40000000; // 40MHz
    }
    if (dev->version < 0x0e)
        d_error("cdctl-b1 version 0x%02x not supported\n", dev->version);

    uint8_t setting = (cdctl_read_reg(dev, REG_SETTING) & 0xf) | BIT_SETTING_TX_PUSH_PULL;
    setting |= init->mode == 1 ? BIT_SETTING_BREAK_SYNC : BIT_SETTING_ARBITRATE;
    cdctl_write_reg(dev, REG_SETTING, setting);
    cdctl_write_reg(dev, REG_FILTER, init->mac);
    cdctl_write_reg(dev, REG_FILTER_M0, init->filter_m[0]);
    cdctl_write_reg(dev, REG_FILTER_M1, init->filter_m[1]);
    cdctl_write_reg(dev, REG_TX_PERMIT_LEN_L, init->tx_permit_len & 0xff);
    cdctl_write_reg(dev, REG_TX_PERMIT_LEN_H, init->tx_permit_len >> 8);
    cdctl_write_reg(dev, REG_MAX_IDLE_LEN_L, init->max_idle_len & 0xff);
    cdctl_write_reg(dev, REG_MAX_IDLE_LEN_H, init->max_idle_len >> 8);
    cdctl_write_reg(dev, REG_TX_PRE_LEN, init->tx_pre_len);
    cdctl_set_baud_rate(dev, init->baud_l, init->baud_h);
    cdctl_flush(dev);

    dn_debug(dev->name, "flags: %02x\n", cdctl_read_reg(dev, REG_INT_FLAG));
    //cdctl_write_reg(dev, REG_INT_MASK, CDCTL_MASK);

    SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);
    SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
}

// handlers


void cdctl_routine(void)
{
    uint8_t flags;
    uint32_t cpu_flags;

    if (!r_dev.is_pending && r_dev.tx_head.first) {
        local_irq_save(cpu_flags);
        cd_frame_t *frame = list_get_entry(&r_dev.tx_head, cd_frame_t);
        cdctl_write_frame_fast(frame);

        flags = cdctl_read_flags_fast();
        if (flags & BIT_FLAG_TX_BUF_CLEAN) {
            cdctl_write_reg_fast(REG_TX_CTRL | 0x80, BIT_TX_START | BIT_TX_RST_POINTER);
            r_dev.tx_cnt++;
        } else {
            r_dev.is_pending = true;
        }
        local_irq_restore(cpu_flags);
        list_put(r_dev.free_head, &frame->node);
    }

    if (r_dev.is_pending) {
        local_irq_save(cpu_flags);
        flags = cdctl_read_flags_fast();
        if (flags & BIT_FLAG_TX_BUF_CLEAN) {
            cdctl_write_reg_fast(REG_TX_CTRL | 0x80, BIT_TX_START | BIT_TX_RST_POINTER);
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
        uint8_t flags = cdctl_read_flags_fast();

        if (flags & BIT_FLAG_RX_PENDING) {
            // if get free list: copy to rx list
            cd_frame_t *frame = list_get_entry(&frame_free_head, cd_frame_t);
            if (frame) {
                cdctl_read_frame_fast(frame);
                list_put(&r_dev.rx_head, &frame->node);
                r_dev.rx_cnt++;
            } else {
                r_dev.rx_no_free_node_cnt++;
            }
            cdctl_write_reg_fast(REG_RX_CTRL | 0x80, BIT_RX_CLR_PENDING | BIT_RX_RST_POINTER);
        }

    } while (!(GPIOB->IDR & 1)); // PB0
}
