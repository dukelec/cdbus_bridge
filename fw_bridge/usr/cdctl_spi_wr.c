/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cdctl_spi_wr.h"
#include "app_main.h"

#define CCR             ctrl
#define CNDTR           dtcnt
#define CMAR            maddr
#define CPAR            paddr
#define ISR             sts
#define IFCR            clr
#define DMA_CCR_EN      (1 << 0)
#define DMA_CCR_TCIE    (1 << 1)

#define SR              sts
#define DR              dt
#define CR1             ctrl1
#define CR2             ctrl2
#define SPI_FLAG_BSY    (1 << 7)
#define SPI_CR1_SPE     (1 << 6)
#define SPI_CR2_RXDMAEN (1 << 0)
#define SPI_CR2_TXDMAEN (1 << 1)


uint8_t cdctl_buf[2];


static void cdctl_spi_wr(const uint8_t *w_buf, uint8_t *r_buf, int len)
{
    // clear spi rx fifo
    CD_DMA_R->CCR &= ~DMA_CCR_EN;
    CD_DMA_R->CNDTR = len;
    //CD_DMA_R->CPAR = (uint32_t)&CD_SPI->DR;
    CD_DMA_R->CMAR = (uint32_t)r_buf;
    CD_DMA_R->CCR |= DMA_CCR_EN;

    CD_DMA_W->CCR &= ~DMA_CCR_EN;
    CD_DMA_W->CNDTR = len;
    //CD_DMA_W->CPAR = (uint32_t)&CD_SPI->DR;
    CD_DMA_W->CMAR = (uint32_t)w_buf;
    CD_DMA_W->CCR |= DMA_CCR_EN;

    while (!(CD_DMA->ISR & CD_DMA_MASK));
    CD_DMA->IFCR = CD_DMA_MASK;
}

void cdctl_spi_wr_it(const uint8_t *w_buf, uint8_t *r_buf, int len)
{
    // clear spi rx fifo
    CD_DMA_R->CCR &= ~DMA_CCR_EN;
    CD_DMA_R->CNDTR = len;
    //CD_DMA_R->CPAR = (uint32_t)&CD_SPI->DR;
    CD_DMA_R->CMAR = (uint32_t)r_buf;
    CD_DMA_R->CCR |= DMA_CCR_TCIE | DMA_CCR_EN;

    CD_DMA_W->CCR &= ~DMA_CCR_EN;
    CD_DMA_W->CNDTR = len;
    //CD_DMA_W->CPAR = (uint32_t)&CD_SPI->DR;
    CD_DMA_W->CMAR = (uint32_t)w_buf;
    CD_DMA_W->CCR |= DMA_CCR_EN;
}


void cdctl_spi_wr_isr(void)
{
    //uint32_t flag_it = CD_DMA->ISR;
    //if (flag_it & CD_DMA_MASK) {
        CD_DMA->IFCR = CD_DMA_MASK;
        CD_DMA_R->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE);
        cdctl_spi_isr();
    //}
}


void cdctl_spi_wr_init(void)
{
    BITS_SET(CD_SPI->CR1, SPI_CR1_SPE); // enable spi

    BITS_SET(CD_SPI->CR2, SPI_CR2_RXDMAEN);
    BITS_SET(CD_SPI->CR2, SPI_CR2_TXDMAEN);
    CD_DMA_W->CPAR = (uint32_t)&CD_SPI->DR;
    CD_DMA_R->CPAR = (uint32_t)&CD_SPI->DR;
}


// used by init and user configuration, before enable isr
uint8_t cdctl_reg_r(uint8_t reg)
{
    uint16_t dat = 0xffff;
    uint8_t tbuf[2] = {reg};
    CD_SS_LOW();
    cdctl_spi_wr(tbuf, (uint8_t *)&dat, 2);
    CD_SS_HIGH();
    return dat >> 8;
}

void cdctl_reg_w(uint8_t reg, uint8_t val)
{
    uint8_t tbuf[2] = {reg | 0x80, val};
    CD_SS_LOW();
    cdctl_spi_wr(tbuf, tbuf, 2);
    CD_SS_HIGH();
}
