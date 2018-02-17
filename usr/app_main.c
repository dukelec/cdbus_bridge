/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"
#include "cdctl_bx_it.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "modbus_crc.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern USBD_HandleTypeDef hUsbDeviceFS;

gpio_t led1 = { .group = LED1_GPIO_Port, .num = LED1_Pin };
gpio_t led2 = { .group = LED2_GPIO_Port, .num = LED2_Pin };

uart_t debug_uart = { .huart = &huart3 };
static uart_t ttl_uart = { .huart = &huart1 };
static uart_t rs232_uart; // = { .huart = &huart2 };
uart_t *hw_uart = NULL;

static gpio_t r_rst_n = { .group = CDCTL_RST_N_GPIO_Port, .num = CDCTL_RST_N_Pin };
static gpio_t r_int_n = { .group = CDCTL_INT_N_GPIO_Port, .num = CDCTL_INT_N_Pin };
static gpio_t r_ns = { .group = CDCTL_NS_GPIO_Port, .num = CDCTL_NS_Pin };
static spi_t r_spi = { .hspi = &hspi1, .ns_pin = &r_ns };

#define CDC_RX_MAX 3
#define CDC_TX_MAX 3
static cdc_buf_t cdc_rx_alloc[CDC_RX_MAX];
static cdc_buf_t cdc_tx_alloc[CDC_TX_MAX];
list_head_t cdc_rx_free_head = {0};
list_head_t cdc_tx_free_head = {0};
list_head_t cdc_rx_head = {0};
list_head_t cdc_tx_head = {0};
cdc_buf_t *cdc_rx_buf = NULL;
cdc_buf_t *cdc_tx_buf = NULL;

#define FRAME_MAX 10
static cd_frame_t frame_alloc[FRAME_MAX];
static list_head_t frame_free_head = {0};

#define PACKET_MAX 10
static cdnet_packet_t packet_alloc[PACKET_MAX];
static list_head_t packet_free_head = {0};

cdctl_intf_t r_intf = {0};   // RS485
cdnet_intf_t n_intf = {0};   // CDNET

// for raw mode only
static list_head_t raw2u_head = {0};

// dummy interface for UART, for pass-thru mode only
static list_head_t d_rx_head = {0};
static list_head_t d_tx_head = {0};
static cd_intf_t d_intf = {0};

static list_node_t *d_get_free_node(cd_intf_t *_)
{
    return list_get_irq_safe(r_intf.free_head);
}
static void d_put_free_node(cd_intf_t *_, list_node_t *node)
{
    list_put_irq_safe(r_intf.free_head, node);
}
static list_node_t *d_get_rx_node(cd_intf_t *_)
{
    return list_get(&d_rx_head);
}
static void d_put_tx_node(cd_intf_t *_, list_node_t *node)
{
    list_put(&d_tx_head, node);
}
static void d_init(void)
{
    d_intf.get_free_node = d_get_free_node;
    d_intf.put_free_node = d_put_free_node;
    d_intf.get_rx_node = d_get_rx_node;
    d_intf.put_tx_node = d_put_tx_node;
}


static void device_init(void)
{
    int i;
    for (i = 0; i < CDC_RX_MAX; i++)
        list_put(&cdc_rx_free_head, &cdc_rx_alloc[i].node);
    for (i = 0; i < CDC_TX_MAX; i++)
        list_put(&cdc_tx_free_head, &cdc_tx_alloc[i].node);
    for (i = 0; i < FRAME_MAX; i++)
        list_put(&frame_free_head, &frame_alloc[i].node);
    for (i = 0; i < PACKET_MAX; i++)
        list_put(&packet_free_head, &packet_alloc[i].node);
    cdc_rx_buf = container_of(list_get(&cdc_rx_free_head), cdc_buf_t, node);

    cdctl_intf_init(&r_intf, &frame_free_head, app_conf.rs485_mac,
            app_conf.rs485_baudrate_low, app_conf.rs485_baudrate_high,
            &r_spi, &r_rst_n, &r_int_n);

    d_init();
    if (app_conf.mode == APP_PASS_THRU) {
        cdnet_intf_init(&n_intf, &packet_free_head, &d_intf, 0x55);
        n_intf.net = 0;
    } else {
        cdnet_intf_init(&n_intf, &packet_free_head, &r_intf.cd_intf,
                app_conf.rs485_mac);
        n_intf.net = app_conf.rs485_net;
    }

    if (app_conf.intf_idx == INTF_TTL) {
        hw_uart = &ttl_uart;
    } else if (app_conf.intf_idx == INTF_RS232) {
        hw_uart = &rs232_uart;
    }
}

void set_led_state(led_state_t state)
{

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
            list_node_t *node = list_get(n_intf.free_head);
            if (!node) {
                d_error("raw <- u: no free pkt\n");
                return;
            }
            pkt = container_of(node, cdnet_packet_t, node);
            pkt->level = app_conf.rpt_pkt_level;
            pkt->is_seq = true;
            pkt->is_multi_net = app_conf.rpt_multi_net;
            pkt->is_multicast = false;
            cdnet_fill_src_addr(&n_intf, pkt);
            pkt->dst_mac = app_conf.rpt_mac;
            memcpy(pkt->dst_addr, app_conf.rpt_addr, 2);
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


// alloc from r_intf.free_head, send through r_intf.tx_head or d_rx_head
static void app_pass_thru_from_u(const uint8_t *buf, int size,
        const uint8_t *wr, const uint8_t *rd)
{
    static uint8_t tmp[260]; // max size for cdbus through uart
    static int copied_len = 0;
    static uint32_t t_last = 0;
    int max_len;
    int cpy_len;

    while (true) {
        if (rd == wr)
            return;
        else if (rd > wr)
            max_len = buf + size - rd;
        else // rd < wr
            max_len = wr - rd;

        if (get_systick() - t_last > (50000 / SYSTICK_US_DIV)) {
            if (copied_len)
                d_warn("pass_thru <- u: timeout cleanup\n");
            copied_len = 0;
        }
        t_last = get_systick();

        if (copied_len < 3)
            cpy_len = min(3 - copied_len, max_len);
        else
            cpy_len = min(tmp[2] + 5 - copied_len, max_len);

        memcpy(tmp + copied_len, rd, cpy_len);
        copied_len += cpy_len;
        rd += cpy_len;
        if (rd == buf + size)
            rd = buf;

        if ((copied_len >= 1 && tmp[0] != 0xaa) ||
                (copied_len >= 2 && tmp[1] != 0x55 && tmp[1] != 0x56)) {
            d_warn("pass_thru <- u: filtered\n");
            copied_len = 0;
            return;
        }

        if (copied_len == tmp[2] + 5) {
            if (crc16(tmp, tmp[2] + 5) != 0) {
                d_warn("pass_thru <- u: crc error\n");
                copied_len = 0;
                return;
            }

            list_node_t *node = list_get_irq_safe(r_intf.free_head);
            if (!node) {
                d_error("pass_thru <- u: no free pkt\n");
                copied_len = 0;
                return;
            }
            cd_frame_t *frame = container_of(node, cd_frame_t, node);

            if (tmp[1] == 0x55) {
                memcpy(frame->dat, tmp, tmp[2] + 3);
                d_verbose("pass_thru <- u: new 55 pkt\n");
                list_put(&d_rx_head, node);
            } else {
                memcpy(frame->dat, tmp + 3, 2);
                frame->dat[2] = tmp[2] - 2;
                memcpy(frame->dat + 3, tmp + 5, frame->dat[2]);
                d_verbose("pass_thru <- u: new 56 pkt\n");
                cdctl_put_tx_node(&r_intf.cd_intf, node);
            }

            copied_len = 0;
        }
    }
}


#define CIRC_BUF_SZ 1024
static uint8_t circ_buf[CIRC_BUF_SZ];
static uint32_t rd_pos = 0;

extern uint32_t end; // end of bss


void app_main(void)
{
    debug_init();
    device_init();
    local_irq_enable();
    d_debug("start app_main...\n");
    *(uint32_t *)(&end) = 0xababcdcd;

    if (app_conf.intf_idx != INTF_USB)
        HAL_UART_Receive_DMA(hw_uart->huart, circ_buf, CIRC_BUF_SZ);


    while (true) {
        if (*(uint32_t *)(&end) != 0xababcdcd) {
            printf("stack overflow: %08lx\n", *(uint32_t *)(&end));
            while (true);
        }

        if (app_conf.intf_idx != INTF_USB &&
                hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            d_info("usb connected\n");
            app_conf.intf_idx = INTF_USB;
            HAL_UART_DMAStop(hw_uart->huart);
        }


        // handle cdnet

        cdnet_rx(&n_intf);
        list_node_t *nd = list_get(&n_intf.rx_head);
        if (nd) {
            cdnet_packet_t *pkt = container_of(nd, cdnet_packet_t, node);
            if (pkt->level == CDNET_L2 || pkt->src_port < CDNET_DEF_PORT ||
                    pkt->dst_port >= CDNET_DEF_PORT) {
                d_warn("unexpected pkg port\n");
                list_put(n_intf.free_head, nd);
            } else {
                switch (pkt->dst_port) {
                case 1:
                    p1_service(pkt);
                    break;
                case 2:
                    p2_service(pkt);
                    break;
                case 3:
                    p3_service(pkt);
                    break;

                case RAW_SER_PORT:
                    if (app_conf.mode == APP_RAW)
                        list_put(&raw2u_head, nd);
                    else
                        list_put(n_intf.free_head, nd);
                    break;

                case RAW_CONF_PORT:
                    if (pkt->len == 4) {
                        app_conf.rpt_en = !!(pkt->dat[0] & 0x80);
                        app_conf.rpt_multi_net = !!(pkt->dat[0] & 0x20);
                        app_conf.rpt_pkt_level = pkt->dat[0] & 0x0f;
                        app_conf.rpt_mac = pkt->dat[1];
                        memcpy(app_conf.rpt_addr, &pkt->dat[2], 2);

                        pkt->len = 0;
                        cdnet_exchg_src_dst(&n_intf, pkt);
                        list_put(&n_intf.tx_head, nd);
                        d_debug("raw_conf: en: %d, mac: %d, lev: %d\n",
                                app_conf.rpt_en, app_conf.rpt_mac,
                                app_conf.rpt_pkt_level);
                    } else {
                        list_put(n_intf.free_head, nd);
                        d_warn("raw_conf: wrong len: %d\n", pkt->len);
                    }
                    break;

                default:
                    d_warn("unexpected pkg\n");
                    list_put(n_intf.free_head, nd);
                }
            }

        }
        cdnet_tx(&n_intf);


        // handle data exchange

        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        uint32_t wd_pos = CIRC_BUF_SZ - hw_uart->huart->hdmarx->Instance->CNDTR;

        if (app_conf.intf_idx == INTF_USB) {
            int size;
            uint8_t *wr, *rd;
            cdc_buf_t *bf = NULL;
            list_node_t *nd = list_get_irq_safe(&cdc_rx_head);
            if (nd) {
                bf = container_of(nd, cdc_buf_t, node);
                size = bf->len + 1; // avoid scroll to begin
                wr = bf->dat + bf->len;
                rd = bf->dat;
            }

            if (app_conf.mode == APP_PASS_THRU) {
                if (bf)
                    app_pass_thru_from_u(bf->dat, size, wr, rd);
            } else {
                if (bf)
                    app_raw_from_u(bf->dat, size, wr, rd);
                else
                    app_raw_from_u(NULL, 0, NULL, NULL); // check for timeout
            }
            if (bf) {
                uint32_t flags;
                local_irq_save(flags);
                list_put(&cdc_rx_free_head, nd);
                if (!cdc_rx_buf) {
                    nd = list_get(&cdc_rx_free_head);
                    cdc_rx_buf = container_of(nd, cdc_buf_t, node);
                    d_warn("continue CDC Rx\n");
                    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf->dat);
                    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                }
                local_irq_restore(flags);
            }
        } else { // hw_uart
            if (app_conf.mode == APP_PASS_THRU)
                app_pass_thru_from_u(circ_buf, CIRC_BUF_SZ, circ_buf + wd_pos, circ_buf + rd_pos);
            else
                app_raw_from_u(circ_buf, CIRC_BUF_SZ, circ_buf + wd_pos, circ_buf + rd_pos);
        }
        rd_pos = wd_pos;

        cdc_buf_t *bf = NULL;
        if (!cdc_tx_head.last) {
            list_node_t *nd = list_get(&cdc_tx_free_head);
            if (!nd) {
                d_warn("cdc_tx_free_head empty 0\n");
                goto send_list;
            }
            bf = container_of(nd, cdc_buf_t, node);
            bf->len = 0;
            list_put(&cdc_tx_head, nd);
        } else {
            bf = container_of(cdc_tx_head.last, cdc_buf_t, node);
        }

        if (app_conf.mode == APP_PASS_THRU) {
            if (d_tx_head.first) {
                // send to u: d_tx_head
                list_node_t *d_nd = d_tx_head.first;
                cd_frame_t *frm = container_of(d_nd, cd_frame_t, node);

                if (bf->len + frm->dat[2] + 5 > 512) {
                    list_node_t *nd = list_get(&cdc_tx_free_head);
                    if (!nd) {
                        d_warn("cdc_tx_free_head empty 1\n");
                        goto send_list;
                    }
                    bf = container_of(nd, cdc_buf_t, node);
                    bf->len = 0;
                    list_put(&cdc_tx_head, nd);
                }

                d_verbose("pass_thru -> u: 55, dat len %d\n", frm->dat[2]);
                uint16_t crc_val = crc16(frm->dat, frm->dat[2] + 3);
                frm->dat[frm->dat[2] + 3] = crc_val & 0xff;
                frm->dat[frm->dat[2] + 4] = crc_val >> 8;
                memcpy(bf->dat + bf->len, frm->dat, frm->dat[2] + 5);
                bf->len += frm->dat[2] + 5;

                list_get(&d_tx_head);
                list_put_irq_safe(r_intf.free_head, d_nd);

            } else if (r_intf.rx_head.first) {
                // send to u: r_intf.rx_head (add 56 aa)
                list_node_t *d_nd = r_intf.rx_head.first;
                cd_frame_t *frm = container_of(d_nd, cd_frame_t, node);

                if (bf->len + frm->dat[2] + 5 + 2 > 512) {
                    list_node_t *nd = list_get(&cdc_tx_free_head);
                    if (!nd) {
                        d_warn("cdc_tx_free_head empty 2\n");
                        goto send_list;
                    }
                    bf = container_of(nd, cdc_buf_t, node);
                    bf->len = 0;
                    list_put(&cdc_tx_head, nd);
                }

                bf->dat[bf->len + 0] = 0x56;
                bf->dat[bf->len + 1] = 0xaa;
                bf->dat[bf->len + 2] = frm->dat[2] + 2;

                memcpy(bf->dat + bf->len + 3, frm->dat, 2);
                memcpy(bf->dat + bf->len + 5, frm->dat + 3, frm->dat[2] + 2);

                uint16_t crc_val = crc16(bf->dat + bf->len, frm->dat[2] + 5);
                bf->dat[bf->len + frm->dat[2] + 5] = crc_val & 0xff;
                bf->dat[bf->len + frm->dat[2] + 6] = crc_val >> 8;

                bf->len += frm->dat[2] + 7;

                list_get_irq_safe(&r_intf.rx_head);
                list_put_irq_safe(r_intf.free_head, d_nd);
            }
        } else { // app_raw
            // send to u: raw2u_head
            if (raw2u_head.first) {
                list_node_t *p_nd = raw2u_head.first;
                cdnet_packet_t *pkt = container_of(nd, cdnet_packet_t, node);

                if (bf->len + pkt->len > 512) {
                    list_node_t *nd = list_get(&cdc_tx_free_head);
                    if (!nd) {
                        d_warn("cdc_tx_free_head empty 3\n");
                        goto send_list;
                    }
                    bf = container_of(nd, cdc_buf_t, node);
                    bf->len = 0;
                    list_put(&cdc_tx_head, nd);
                }

                memcpy(bf->dat + bf->len, pkt->dat, pkt->len);
                bf->len += pkt->len;

                list_get(&raw2u_head);
                list_put(n_intf.free_head, p_nd);
            }
        }

send_list:
        if (cdc_tx_buf) {
            if (app_conf.intf_idx == INTF_USB) {
                if (hcdc->TxState == 0) {
                    list_put(&cdc_tx_free_head, &cdc_tx_buf->node);
                    cdc_tx_buf = NULL;
                }
            } else { // hw_uart
                if (hw_uart->huart->TxXferCount == 0) {
                    hw_uart->huart->gState = HAL_UART_STATE_READY;
                    list_put(&cdc_tx_free_head, &cdc_tx_buf->node);
                    cdc_tx_buf = NULL;
                    //d_verbose("hw_uart dma done.\n");
                }
            }
        }
        if (!cdc_tx_buf && cdc_tx_head.first) {
            cdc_buf_t *bf = container_of(cdc_tx_head.first, cdc_buf_t, node);
            if (bf->len != 0) {
                if (app_conf.intf_idx == INTF_USB) {
                    CDC_Transmit_FS(bf->dat, bf->len);
                } else { // hw_uart
                    //d_verbose("hw_uart dma tx...\n");
                    HAL_UART_Transmit_DMA(hw_uart->huart, bf->dat, bf->len);
                }
                list_get(&cdc_tx_head);
                cdc_tx_buf = bf;
            }
        }

        debug_flush();
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == r_int_n.num) {
        cdctl_int_isr(&r_intf);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_intf);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_intf);
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    cdctl_spi_isr(&r_intf);
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    d_error("spi error...\n");
}

