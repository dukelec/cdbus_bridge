/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __UART_DMA_WR_H__
#define __UART_DMA_WR_H__

#include "cd_utils.h"
#include "cd_list.h"

#define UART_DMA_R      DMA2_CHANNEL1
#define UART_DMA_W      DMA2_CHANNEL2
#define UART_DMA        DMA2
#define UART_DMA_MASK   (2 << 4) // DMA_ISR.TCIF2
#define UART_DEV        USART1


void uart_tdc_isr(void);
void uart_dma_isr(void);
void uart_dma_tx(void);
void uart_dma_rx(void);
void uart_dma_wr_init(void);

extern list_head_t raw_rx_head;
extern list_head_t raw_tx_head;

#endif
