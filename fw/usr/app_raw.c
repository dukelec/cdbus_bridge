/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

static cdnet_socket_t sock_r = { .port = RAW_SER_PORT };

void app_raw_init(void)
{
    cdnet_intf_init(&n_intf, &r_dev.cd_dev, app_conf.rs485_net, app_conf.rs485_mac);
    cdnet_intf_register(&n_intf);
    cdnet_socket_bind(&sock_r, NULL);
}

static void read_raw_port(const uint8_t *buf, int size,
        const uint8_t *wr, const uint8_t *rd)
{
    static cdnet_packet_t *pkt = NULL;
    static uint32_t t_last = 0;
    int max_len;
    int cpy_len;

    if (!app_conf.rpt_en) {
        df_warn("rpt_en disabled\n");
        return;
    }

    if (rd == wr && pkt && pkt->len && get_systick() - t_last > (2000 / SYSTICK_US_DIV)) {
        cdnet_socket_sendto(&sock_r, pkt);
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
            pkt = cdnet_packet_get(&cdnet_free_pkts);
            if (!pkt) {
                df_error("no free pkt\n");
                return;
            }
            pkt->dst = app_conf.rpt_dst;
            pkt->len = 1;
            pkt->dat[0] = 0; // indicate a report
        }

        t_last = get_systick();
        cpy_len = min(242 - pkt->len, max_len); // 253 - 11 (max 10 byte header, 1 byte command)

        memcpy(pkt->dat + pkt->len, rd, cpy_len);
        pkt->len += cpy_len;
        rd += cpy_len;
        if (rd == buf + size)
            rd = buf;

        if (pkt->len == 242) {
            cdnet_socket_sendto(&sock_r, pkt);
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
            read_raw_port(bf->dat, size, wr, rd);

            local_irq_save(flags);
            list_put(&cdc_rx_free_head, &bf->node);
            if (!cdc_rx_buf) {
                cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_buf_t);
                df_verbose("continue CDC Rx\n");
                USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf->dat);
                USBD_CDC_ReceivePacket(&hUsbDeviceFS);
            }
            local_irq_restore(flags);
        } else {
            read_raw_port(NULL, 0, NULL, NULL); // check for timeout
        }
    } else { // hw_uart
        read_raw_port(circ_buf, CIRC_BUF_SZ, circ_buf + wd_pos, circ_buf + rd_pos);
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

    // write to raw port
    cdnet_packet_t *pkt = list_entry(sock_r.rx_head.first, cdnet_packet_t);
    if (!pkt)
        return;

    if (pkt->len > 1 && pkt->dat[0] == 0) {
        if (bf->len + pkt->len - 1 > 512) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
            if (!bf) {
                df_warn("no cdc_tx_free\n");
                return;
            }
            bf->len = 0;
            list_put(&cdc_tx_head, &bf->node);
        }

        memcpy(bf->dat + bf->len, pkt->dat + 1, pkt->len - 1);
        bf->len += pkt->len - 1;
    }

    cdnet_socket_recvfrom(&sock_r);
    list_put(&cdnet_free_pkts, &pkt->node);
}
