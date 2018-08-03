/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

// for raw mode only
list_head_t raw2u_head = {0};

void app_raw_init(void)
{
    cdnet_intf_init(&n_intf, &packet_free_head, &r_intf.cd_intf, &app_conf.rs485_addr);
}

// alloc from n_intf.free_head, send through n_intf.tx_head
static void app_raw_from_u(const uint8_t *buf, int size,
        const uint8_t *wr, const uint8_t *rd)
{
    static cdnet_packet_t *pkt = NULL;
    static uint32_t t_last = 0;
    int max_len;
    int cpy_len;

    if (!app_conf.rpt_en) {
        d_warn("raw <- u: rpt_en disabled\n");
        return;
    }

    if (rd == wr && pkt && pkt->len && get_systick() - t_last > (2000 / SYSTICK_US_DIV)) {
        list_put(&n_intf.tx_head, &pkt->node);
        pkt = NULL;
        return;
    }

    while (true) {
        if (rd == wr)
            return;
        else if (rd > wr)
            max_len = buf + size - rd;
        else // rd < wr
            max_len = wr - rd;

        if (!pkt) {
            pkt = cdnet_packet_get(n_intf.free_head);
            if (!pkt) {
                d_error("raw <- u: no free pkt\n");
                return;
            }
            pkt->level = app_conf.rpt_pkt_level;
            pkt->seq = app_conf.rpt_seq;
            pkt->multi = app_conf.rpt_multi;
            cdnet_fill_src_addr(&n_intf, pkt);
            pkt->dst_mac = app_conf.rpt_mac;
            pkt->dst_addr = app_conf.rpt_addr;
            pkt->src_port = CDNET_DEF_PORT;
            pkt->dst_port = RAW_SER_PORT;
            pkt->len = 0;
        }

        t_last = get_systick();
        cpy_len = min(243 - pkt->len, max_len);

        memcpy(pkt->dat + pkt->len, rd, cpy_len);
        pkt->len += cpy_len;
        rd += cpy_len;
        if (rd == buf + size)
            rd = buf;

        if (pkt->len == 243) {
            list_put(&n_intf.tx_head, &pkt->node);
            pkt = NULL;
        }
    }
}


void app_raw(void)
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
            app_raw_from_u(bf->dat, size, wr, rd);

            local_irq_save(flags);
            list_put(&cdc_rx_free_head, &bf->node);
            if (!cdc_rx_buf) {
                cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_buf_t);
                d_verbose("continue CDC Rx\n");
                USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf->dat);
                USBD_CDC_ReceivePacket(&hUsbDeviceFS);
            }
            local_irq_restore(flags);
        } else {
            app_raw_from_u(NULL, 0, NULL, NULL); // check for timeout
        }
    } else { // hw_uart
        app_raw_from_u(circ_buf, CIRC_BUF_SZ, circ_buf + wd_pos, circ_buf + rd_pos);
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

    // send to u: raw2u_head
    if (raw2u_head.first) {
        cdnet_packet_t *pkt = list_entry(raw2u_head.first, cdnet_packet_t);

        if (bf->len + pkt->len > 512) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
            if (!bf) {
                d_warn("no cdc_tx_free_head, 3\n");
                return;
            }
            bf->len = 0;
            list_put(&cdc_tx_head, &bf->node);
        }

        memcpy(bf->dat + bf->len, pkt->dat, pkt->len);
        bf->len += pkt->len;

        list_get(&raw2u_head);
        list_put(n_intf.free_head, &pkt->node);
    }
}
