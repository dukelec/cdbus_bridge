/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

static int wr_cdc_left = 0;


void app_bridge(void)
{
    // handle data exchange

    if (csa.usb_online) {
        if (d_dev.free_head->len > 5) {
            cdc_rx_buf_t *bf = list_get_entry_it(&cdc_rx_head, cdc_rx_buf_t);
            if (bf) {
                uint32_t flags;

                if (app_mode)
                    cduart_rx_handle(&c_dev, bf->dat, bf->len);
                else
                    cduart_rx_handle(&d_dev, bf->dat, bf->len);

                local_irq_save(flags);
                list_put(&cdc_rx_free_head, &bf->node);
                if (!cdc_rx_buf) {
                    cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_rx_buf_t);
                    d_verbose("continue CDC Rx\n");
                    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf->dat);
                    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                }
                local_irq_restore(flags);
            }
        }
    }

    cdc_tx_buf_t *bf = NULL;
    if (!cdc_tx_head.last) {
        bf = list_get_entry(&cdc_tx_free_head, cdc_tx_buf_t);
        if (!bf) {
            df_warn("no cdc_tx_free (tx idle)\n");
            return;
        }
        bf->len = 0;
        list_put(&cdc_tx_head, &bf->node);
    } else {
        bf = list_entry(cdc_tx_head.last, cdc_tx_buf_t);
        if (bf->len == CDC_TX_SIZE) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_tx_buf_t);
            if (!bf) {
                df_warn("no cdc_tx_free (tx pend %d)\n", cdc_tx_head.len);
                return;
            }
            bf->len = 0;
            list_put(&cdc_tx_head, &bf->node);
        }
    }

    // pc data -> rs485
    cd_frame_t *frm = list_get_entry_it(&d_dev.rx_head, cd_frame_t);
    if (frm) {
        cdctl_put_tx_frame(&r_dev.cd_dev, frm);
    }

    // rs485 -> pc
    frm = list_get_entry_it(&r_dev.rx_head, cd_frame_t);
    if (frm) {
        if (app_mode)
            list_put_it(d_dev.free_head, &frm->node); // drop for config mode
        else
            list_put_it(&d_dev.tx_head, &frm->node);
    }

    // config reply -> pc
    frm = list_get_entry_it(&c_dev.tx_head, cd_frame_t);
    if (frm) {
        if (!app_mode)
            list_put_it(d_dev.free_head, &frm->node); // drop for data mode
        else
            list_put_it(&d_dev.tx_head, &frm->node);
    }

    // send to pc
    if (d_dev.tx_head.first) { // send d_dev.tx_head
        cd_frame_t *frm = list_entry(d_dev.tx_head.first, cd_frame_t);
        int frm_full_len = frm->dat[2] + 5;

        if (wr_cdc_left == 0) {
            cduart_fill_crc(frm->dat);

            if (bf->len + frm_full_len > CDC_TX_SIZE) {
                wr_cdc_left = bf->len + frm_full_len - CDC_TX_SIZE;
                memcpy(bf->dat + bf->len, frm->dat, CDC_TX_SIZE - bf->len);
                bf->len = CDC_TX_SIZE;
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
