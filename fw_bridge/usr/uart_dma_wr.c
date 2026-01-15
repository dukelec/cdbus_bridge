/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "uart_dma_wr.h"
#include "app_main.h"

#define CCR             ctrl
#define CNDTR           dtcnt
#define CMAR            maddr
#define CPAR            paddr
#define ISR             sts
#define IFCR            clr
#define DMA_CCR_EN      (1 << 0)
#define DMA_CCR_TCIE    (1 << 1)


list_head_t raw_rx_head = {0};
list_head_t raw_tx_head = {0};

#define CIRC_BUF_SZ 4096
static uint8_t circ_buf[CIRC_BUF_SZ];

static uint32_t rd_pos = 0;

static cd_frame_t *tx_frame = NULL;


void uart_tdc_isr(void)
{
    UART_DEV->sts = ~USART_TDC_FLAG;
    usart_receiver_enable(UART_DEV, true);
}

static void uart_dma_wr_it(const uint8_t *w_buf, int len)
{
    UART_DMA_W->CCR &= ~DMA_CCR_EN;
    UART_DMA_W->CNDTR = len;
    UART_DMA_W->CMAR = (uint32_t)w_buf;
    UART_DMA_W->CCR |= DMA_CCR_TCIE | DMA_CCR_EN;
}


void uart_dma_isr(void)
{
    //uint32_t flag_it = UART_DMA->ISR;
    //if (flag_it & UART_DMA_MASK) {
        UART_DMA->IFCR = UART_DMA_MASK;
        if (tx_frame) {
            cd_list_put(&frame_free_head, tx_frame);
            tx_frame = NULL;
        }
        uart_dma_tx();
    //}
}


void uart_dma_tx(void)
{
    uint32_t flags;
    local_irq_save(flags);
    if (!tx_frame) {
        tx_frame = cd_list_get(&raw_tx_head);
        if (tx_frame) {
            usart_receiver_enable(UART_DEV, false);
            uart_dma_wr_it(tx_frame->dat, tx_frame->dat[257]);
        }
    }
    local_irq_restore(flags);
}

static void rx_handle(const uint8_t *buf, unsigned len)
{
    const uint8_t *p = buf;
    while (raw_mode && len) {
        cd_frame_t *frm = NULL;
        if (raw_rx_head.len) {
            frm = cd_list_get_last(&raw_rx_head);
            if (frm->dat[257] == 255) {
                cd_list_put(&raw_rx_head, frm);
                frm = NULL;
            }
        }
        if (!frm) {
            frm = cd_list_get(&frame_free_head);
            frm->dat[257] = 0;
        }
        if (!frm)
            break;
        unsigned sub_len = min(255 - frm->dat[257], len);
        memcpy(frm->dat + frm->dat[257], p, sub_len);
        frm->dat[257] += sub_len;
        cd_list_put(&raw_rx_head, frm);
        p += sub_len;
        len -= sub_len;
    }
}

void uart_dma_rx(void)
{
    uint16_t wd_pos = CIRC_BUF_SZ - UART_DMA_R->CNDTR;
    
    const uint8_t *wr = circ_buf + wd_pos;
    const uint8_t *rd = circ_buf + rd_pos;

    if (rd < wr) {
        rx_handle(rd, wr - rd);
    } else if (rd > wr) {
        rx_handle(rd, circ_buf + CIRC_BUF_SZ - rd);
        rx_handle(circ_buf, wr - circ_buf);
    }
    rd_pos = wd_pos;
}


void uart_dma_wr_init(void)
{
    UART_DMA_R->CCR &= ~(DMA_CCR_EN | DMA_CCR_TCIE);
    UART_DMA_R->CNDTR = CIRC_BUF_SZ;
    UART_DMA_R->CMAR = (uint32_t)circ_buf;
    UART_DMA_R->CCR |= DMA_CCR_EN;
}

