/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

static cd_frame_t *d_conv_frame = NULL;

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
    }

    // send to host
    if (d_dev.tx_head.first) { // send d_dev.tx_head
        cd_frame_t *frm = list_entry(d_dev.tx_head.first, cd_frame_t);

        if (bf->len + frm->dat[2] + 5 > 512) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
            if (!bf) {
                df_warn("no cdc_tx_free (d_dev)\n");
                return;
            }
            bf->len = 0;
            list_put(&cdc_tx_head, &bf->node);
        }

        //df_verbose("local ret: 55, dat len %d\n", frm->dat[2]);
        cduart_fill_crc(frm->dat);
        memcpy(bf->dat + bf->len, frm->dat, frm->dat[2] + 5);
        bf->len += frm->dat[2] + 5;

        list_get(&d_dev.tx_head);
        list_put_it(d_dev.free_head, &frm->node);

    } else if (r_dev.rx_head.first) { // send rs485 data (add 56 aa)
        cd_frame_t *frm = list_entry(r_dev.rx_head.first, cd_frame_t);

        if (bf->len + frm->dat[2] + 5 + 2 > 512) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
            if (!bf) {
                d_warn("no cdc_tx_free (bridge)\n");
                return;
            }
            bf->len = 0;
            list_put(&cdc_tx_head, &bf->node);
        }

        uint8_t *buf_dst = bf->dat + bf->len;
        *buf_dst = 0x56;
        *(buf_dst + 1) = 0xaa;
        *(buf_dst + 2) = frm->dat[2] + 2;
        memcpy(buf_dst + 3, frm->dat, 2);
        memcpy(buf_dst + 5, frm->dat + 3, *(buf_dst + 2));
        cduart_fill_crc(buf_dst);
        bf->len += frm->dat[2] + 7;

        list_get_it(&r_dev.rx_head);
        list_put_it(r_dev.free_head, &frm->node);
    }
}
