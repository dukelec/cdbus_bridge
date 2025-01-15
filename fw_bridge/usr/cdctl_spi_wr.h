/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CDCTL_SPI_WR_H__
#define __CDCTL_SPI_WR_H__

#include "cd_utils.h"
#include "cd_list.h"

#define CD_SPI          SPI1
#define CD_DMA_R        DMA1_CHANNEL1
#define CD_DMA_W        DMA1_CHANNEL2
#define CD_DMA          DMA1
#define CD_DMA_MASK     (2 << 0) // DMA_ISR.TCIF1

#define CD_SS_HIGH()    {CD_SS_GPIO_PORT->scr = CD_SS_PIN;}
#define CD_SS_LOW()     {CD_SS_GPIO_PORT->clr = CD_SS_PIN;}
#define CD_INT_RD()     (CD_INT_GPIO_PORT->idt & CD_INT_PIN)

extern uint8_t cdctl_buf[];

void cdctl_spi_wr_it(const uint8_t *w_buf, uint8_t *r_buf, int len);
void cdctl_spi_wr_isr(void);
void cdctl_spi_wr_init(void);

uint8_t cdctl_reg_r(uint8_t reg);
void cdctl_reg_w(uint8_t reg, uint8_t val);


static inline void cdctl_reg_r_it(uint8_t reg)
{
    cdctl_buf[0] = reg;
    CD_SS_LOW();
    cdctl_spi_wr_it(cdctl_buf, cdctl_buf, 2);
}

static inline void cdctl_reg_w_it(uint8_t reg, uint8_t val)
{
    cdctl_buf[0] = reg | 0x80;
    cdctl_buf[1] = val;
    CD_SS_LOW();
    cdctl_spi_wr_it(cdctl_buf, cdctl_buf, 2);
}

#endif
