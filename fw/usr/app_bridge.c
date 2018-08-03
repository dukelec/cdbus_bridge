/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

// dummy interface for UART, for bridge mode only
static cduart_intf_t d_intf = {0};
static cd_frame_t *d_conv_frame = NULL;

void app_bridge_init(void)
{
    d_conv_frame = list_get_entry(&frame_free_head, cd_frame_t);

    cduart_intf_init(&d_intf, &frame_free_head);
    d_intf.remote_filter[0] = 0xaa;
    d_intf.remote_filter_len = 1;
    d_intf.local_filter[0] = 0x55;
    d_intf.local_filter[1] = 0x56;
    d_intf.local_filter_len = 2;

    cdnet_addr_t addr = { .net = 0, .mac = 0x55 };
    cdnet_intf_init(&n_intf, &packet_free_head, &d_intf.cd_intf, &addr);
}


// alloc from r_intf.free_head, send through r_intf.tx_head or d_rx_head
static void app_bridge_from_u(const uint8_t *buf, int size,
        const uint8_t *wr, const uint8_t *rd)
{
    list_node_t *pre, *cur;

    if (rd > wr) {
        cduart_rx_handle(&d_intf, rd, buf + size - rd);
        rd = buf;
    }
    if (rd < wr)
        cduart_rx_handle(&d_intf, rd, wr - rd);

    list_for_each(&d_intf.rx_head, pre, cur) {
        cd_frame_t *fr_src = list_entry(cur, cd_frame_t);
        if (fr_src->dat[1] == 0x56) {
            memcpy(d_conv_frame->dat, fr_src->dat + 3, 2);
            d_conv_frame->dat[2] = fr_src->dat[2] - 2;
            memcpy(d_conv_frame->dat + 3, fr_src->dat + 5, d_conv_frame->dat[2]);

            list_pick(&d_intf.rx_head, pre, cur);
            cdctl_put_tx_frame(&r_intf.cd_intf, d_conv_frame);
            d_conv_frame = fr_src;
            cur = pre;
        }
    }
}


void app_bridge(void)
{
    // handle data exchange
    uint32_t wd_pos = CIRC_BUF_SZ - hw_uart->huart->hdmarx->Instance->CNDTR;

    if (app_conf.ser_idx == SER_USB) {
        int size;
        uint8_t *wr, *rd;
        cdc_buf_t *bf = list_get_entry_it(&cdc_rx_head, cdc_buf_t);
        if (bf) {
            uint32_t flags;
            size = bf->len + 1; // avoid scroll to begin
            wr = bf->dat + bf->len;
            rd = bf->dat;
            app_bridge_from_u(bf->dat, size, wr, rd);

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
        app_bridge_from_u(circ_buf, CIRC_BUF_SZ, circ_buf + wd_pos, circ_buf + rd_pos);
    }
    rd_pos = wd_pos;

    cdc_buf_t *bf = NULL;
    if (!cdc_tx_head.last) {
        bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
        if (!bf) {
            d_warn("no cdc_tx_free, 0\n");
            return;
        }
        bf->len = 0;
        list_put(&cdc_tx_head, &bf->node);
    } else {
        bf = list_entry(cdc_tx_head.last, cdc_buf_t);
    }

    if (d_intf.tx_head.first) { // send to u: d_intf.tx_head
        cd_frame_t *frm = list_entry(d_intf.tx_head.first, cd_frame_t);

        if (bf->len + frm->dat[2] + 5 > 512) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
            if (!bf) {
                d_warn("no cdc_tx_free, 1\n");
                return;
            }
            bf->len = 0;
            list_put(&cdc_tx_head, &bf->node);
        }

        d_verbose("bridge -> u: 55, dat len %d\n", frm->dat[2]);
        cduart_fill_crc(frm->dat);
        memcpy(bf->dat + bf->len, frm->dat, frm->dat[2] + 5);
        bf->len += frm->dat[2] + 5;

        list_get(&d_intf.tx_head);
        list_put_it(r_intf.free_head, &frm->node);

    } else if (r_intf.rx_head.first) { // send to u: r_intf.rx_head (add 56 aa)
        cd_frame_t *frm = list_entry(r_intf.rx_head.first, cd_frame_t);

        if (bf->len + frm->dat[2] + 5 + 2 > 512) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
            if (!bf) {
                d_warn("no cdc_tx_free, 2\n");
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

        list_get_it(&r_intf.rx_head);
        list_put_it(r_intf.free_head, &frm->node);
    }
}
