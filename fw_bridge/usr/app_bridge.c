/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

static cd_frame_t *d_conv_frame = NULL;
static int wr_cdc_left = 0;

void app_bridge_init(void)
{
    d_conv_frame = list_get_entry(&frame_free_head, cd_frame_t);
}

static void read_from_host(const uint8_t *buf, int size,
        const uint8_t *wr, const uint8_t *rd)
{
    list_node_t *pre, *cur;

    if (rd > wr) {
        cduart_rx_handle(&d_dev, rd, buf + size - rd);
        rd = buf;
    }
    if (rd < wr)
        cduart_rx_handle(&d_dev, rd, wr - rd);

    list_for_each(&d_dev.rx_head, pre, cur) {
        cd_frame_t *fr_src = list_entry(cur, cd_frame_t);
        if (fr_src->dat[1] == 0x56) {
            memcpy(d_conv_frame->dat, fr_src->dat + 3, 2);
            d_conv_frame->dat[2] = fr_src->dat[2] - 2;
            memcpy(d_conv_frame->dat + 3, fr_src->dat + 5, d_conv_frame->dat[2]);

            list_pick(&d_dev.rx_head, pre, cur);
            cdctl_put_tx_frame(&r_dev.cd_dev, d_conv_frame);
            d_conv_frame = fr_src;
            cur = pre;
        }
    }
}

void app_bridge(void)
{
    // handle data exchange
    uint32_t wd_pos = CIRC_BUF_SZ - hw_uart->huart->hdmarx->Instance->CNDTR;

    if (csa.usb_online) {
        if (d_dev.free_head->len > 5) {
            int size;
            uint8_t *wr, *rd;
            cdc_buf_t *bf = list_get_entry_it(&cdc_rx_head, cdc_buf_t);
            if (bf) {
                uint32_t flags;
                size = bf->len + 1; // avoid scroll to begin
                wr = bf->dat + bf->len;
                rd = bf->dat;
                read_from_host(bf->dat, size, wr, rd);

                local_irq_save(flags);
                list_put(&cdc_rx_free_head, &bf->node);
                if (!cdc_rx_buf) {
                    cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_buf_t);
                    d_verbose("continue CDC Rx\n");
                    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf->dat);
                    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                }
                local_irq_restore(flags);
            }
        }
    } else { // hw_uart
        read_from_host(circ_buf, CIRC_BUF_SZ, circ_buf + wd_pos, circ_buf + rd_pos);
    }
    rd_pos = wd_pos;

    cdc_buf_t *bf = NULL;
    if (!cdc_tx_head.last) {
        bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
        if (!bf) {
            df_warn("no cdc_tx_free (tx idle)\n");
            return;
        }
        bf->len = 0;
        list_put(&cdc_tx_head, &bf->node);
    } else {
        bf = list_entry(cdc_tx_head.last, cdc_buf_t);
        if (bf->len == 512) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
            if (!bf) {
                df_warn("no cdc_tx_free (tx pend %d)\n", cdc_tx_head.len);
                return;
            }
            bf->len = 0;
            list_put(&cdc_tx_head, &bf->node);
        }
    }

    // move rs485 rx data to d_dev
    cd_frame_t *frm = list_get_entry_it(&r_dev.rx_head, cd_frame_t);
    if (frm) {
        memmove(frm->dat + 5, frm->dat + 3, frm->dat[2]);
        memcpy(frm->dat + 3, frm->dat, 2);
        frm->dat[0] = 0x56;
        frm->dat[1] = 0xaa;
        frm->dat[2] += 2;
        list_put_it(&d_dev.tx_head, &frm->node);
    }

    // send to host
    if (d_dev.tx_head.first) { // send d_dev.tx_head
        cd_frame_t *frm = list_entry(d_dev.tx_head.first, cd_frame_t);
        int frm_full_len = frm->dat[2] + 5;

        if (wr_cdc_left == 0) {
            cduart_fill_crc(frm->dat);

            if (bf->len + frm_full_len > 512) {
                wr_cdc_left = bf->len + frm_full_len - 512;
                memcpy(bf->dat + bf->len, frm->dat, 512 - bf->len);
                bf->len = 512;
            } else {
                memcpy(bf->dat + bf->len, frm->dat, frm_full_len);
                bf->len += frm_full_len;
                list_get(&d_dev.tx_head);
                list_put_it(d_dev.free_head, &frm->node);
            }

        } else { // bf->len is 0
                memcpy(bf->dat, frm->dat + (frm_full_len - wr_cdc_left), wr_cdc_left);
                bf->len = wr_cdc_left;
                list_get(&d_dev.tx_head);
                list_put_it(d_dev.free_head, &frm->node);
                wr_cdc_left = 0;
        }
    }
}
