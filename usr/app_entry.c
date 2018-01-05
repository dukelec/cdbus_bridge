/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#define DEBUG
#include "common.h"
#include "cdctl_bx.h"
#include "cdbus_uart.h"
#include "uart_raw.h"
#include "port_dispatcher.h"
#include "main.h"

#define FLASH_PORT          1
#define RAW_SER_PORT        10
#define RAW_CONF_PORT       11
#define PASSTHRU_CONF_PORT  15

static void uart_enable_rx_irq(void);

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;

uart_t debug_uart = { .huart = &huart3 };
static uart_t u_uart = { .huart = &huart1 };
static gpio_t r_ns = { .group = CDCTL_NS_GPIO_Port, .num = CDCTL_NS_Pin };
static spi_t r_spi = { .hspi = &hspi1, .ns_pin = &r_ns };

// UART side
#define U_FRAME_MAX 10
static cd_frame_t u_frame_alloc[U_FRAME_MAX];
static list_head_t u_free_head = {0};
static list_head_t ud_head = {0};   // for passthrough mode

// RS485 side
#define R_FRAME_MAX 10
static cd_frame_t r_frame_alloc[R_FRAME_MAX];
static list_head_t r_free_head = {0};

// Bridge
#define N_PACKET_MAX 10
static cdnet_packet_t n_packet_alloc[N_PACKET_MAX];
static list_head_t n_free_head = {0};

static ur_intf_t ur_intf = {0};     // user side raw
static cduart_intf_t uc_intf = {0}; // user side cduart
static cd_intf_t ud_intf = {0};     // user side dummy, for passthrough mode
static cdctl_intf_t r_intf = {0};   // rs485 side
static cdnet_intf_t n_intf = {0};

static port_dispr_t p_dispr = {0};
static udp_ser_t dev_ser = {0};     // only config the intf which cmd comes
static udp_ser_t flash_conf = {0};
static udp_ser_t raw_conf = {0};
static udp_ser_t raw_in = {0};
static udp_req_t raw_out = {0};
static udp_ser_t passthru_conf = {0};

// TODO: save and restore from flash
static int8_t raw_out_type = 0; // -1: disable; 0: local; 1: cross net
static uint8_t raw_out_mac = 0;
static uint8_t raw_out_addr[2];
static uint16_t raw_out_port_begin = CDNET_BASIC_PORT;
static uint16_t raw_out_port_end = CDNET_BASIC_PORT + 1;

static bool raw_mode = false;
static uint8_t local_mac = 254;

static char cpu_id[25];


static void get_uid(char *buf)
{
    const char tlb[] = "0123456789abcdef";
    int i;

    for (i = 0; i < 12; i++) {
        uint8_t val = *((char *)UID_BASE + i);
        buf[i * 2 + 0] = tlb[val & 0xf];
        buf[i * 2 + 1] = tlb[val >> 4];
    }
    buf[24] = '\0';
}

static void cd_dump_frame(cd_frame_t *frame)
{
    int i, len = frame->dat[2] + 3;
    for (i = 0; i < len; i++)
        printf("%02x ", frame->dat[i]);
    printf("\n");
}

// for passthrough mode, end after the r2uc

static list_node_t *ud_get_free_node(cd_intf_t *unused)
{
    return uc_intf.cd_intf.get_free_node(&uc_intf.cd_intf);
}
static list_node_t *ud_get_rx_node(cd_intf_t *unused)
{
    return list_get(&ud_head);
}
static void ud_put_free_node(cd_intf_t *unused, list_node_t *node)
{
    uc_intf.cd_intf.put_free_node(&uc_intf.cd_intf, node);
}
static void ud_put_tx_node(cd_intf_t *unused, list_node_t *node)
{
    uc_intf.cd_intf.put_tx_node(&uc_intf.cd_intf, node);
}
static void ud_set_filter(cd_intf_t *unused, uint8_t filter)
{
    return;
}
static void ud_init(void)
{
    ud_intf.get_free_node = ud_get_free_node;
    ud_intf.get_rx_node = ud_get_rx_node;
    ud_intf.put_free_node = ud_put_free_node;
    ud_intf.put_tx_node = ud_put_tx_node;
    ud_intf.set_filter = ud_set_filter;
}

static void uc2r_ud(void)
{
    list_node_t *node;
    cd_frame_t *frame = NULL;

    while (true) {
        node = uc_intf.cd_intf.get_rx_node(&uc_intf.cd_intf);
        if (!node)
            break;
        frame = container_of(node, cd_frame_t, node);
        if (frame->dat[0] == 0xaa && frame->dat[1] == 0x56) {

            list_node_t *r_node =
                    r_intf.cd_intf.get_free_node(&r_intf.cd_intf);
            if (r_node) {
                cd_frame_t *r_frame = container_of(r_node, cd_frame_t, node);
                memcpy(r_frame->dat, frame->dat + 3, frame->dat[2]);
                d_debug("uc2r: done\n");
                //cd_dump_frame(r_frame);
                r_intf.cd_intf.put_tx_node(&r_intf.cd_intf, r_node);
            } else {
                d_error("uc2r: r_intf can't get free node\n");
            }
            uc_intf.cd_intf.put_free_node(&uc_intf.cd_intf, node);
        } else if (frame->dat[0] == 0xaa && frame->dat[1] == 0x55) {
            d_verbose("get_dummy...\n");
            list_put(&ud_head, node);
        } else {
            d_error("wrong up packet addr...\n");
            uc_intf.cd_intf.put_free_node(&uc_intf.cd_intf, node);
        }
    }
}

static void r2uc(void)
{
    uint32_t flags;
    local_irq_save(flags);

    if (r_intf.rx_head.first == NULL) {
        local_irq_restore(flags);
        return;
    }
    if (uc_intf.free_head->first == NULL) {
        local_irq_restore(flags);
        d_debug("r2uc: get free err...\n");
        return;
    }
    list_node_t *r_node = r_intf.cd_intf.get_rx_node(&r_intf.cd_intf);
    list_node_t *u_node = uc_intf.cd_intf.get_free_node(&uc_intf.cd_intf);
    local_irq_restore(flags);

    cd_frame_t *r_frame = container_of(r_node, cd_frame_t, node);
    cd_frame_t *u_frame = container_of(u_node, cd_frame_t, node);

    u_frame->dat[0] = 0x56;
    u_frame->dat[1] = 0xaa;
    u_frame->dat[2] = r_frame->dat[2] + 3;
    memcpy(u_frame->dat + 3, r_frame->dat, r_frame->dat[2] + 3);

    d_debug("r2uc: done\n");
    //cd_dump_frame(r_frame);
    r_intf.cd_intf.put_free_node(&r_intf.cd_intf, r_node);
    uc_intf.cd_intf.put_tx_node(&uc_intf.cd_intf, u_node);
}


static void dev_ser_cb(void)
{
    char info_str[100];

    list_node_t *node = list_get(&dev_ser.A_head);
    if (node) {
        cdnet_packet_t *pkt = container_of(node, cdnet_packet_t, node);

        // M: model; S: serial string; HW: hardware version; SW: software version
        sprintf(info_str, "M: cdbus_bridge; S: %s; SW: %s", cpu_id, SW_VER);

        // filter string by input data
        if (pkt->dat_len != 0) {
            if (strstr(info_str, (char *)pkt->dat) == NULL) {
                list_put(n_intf.free_head, node);
                return;
            }
        }

        strcpy((char *)pkt->dat, info_str);
        pkt->dat_len = strlen(info_str);
        list_put(&p_dispr.V_ser_head, node);
    }
}

static void raw_conf_cb(void)
{
    list_node_t *node = list_get(&raw_conf.A_head);
    if (node) {
        cdnet_packet_t *pkt = container_of(node, cdnet_packet_t, node);
        if (pkt->dat_len == 4) {
            raw_out_type = pkt->dat[0];
            raw_out_mac = pkt->dat[1];
            memcpy(raw_out_addr, &pkt->dat[2], 2);

            pkt->dat_len = 0;
            list_put(&p_dispr.V_ser_head, node);
            d_debug("raw_conf: set ip: mac: %d, type: %d\n",
                    raw_out_mac, raw_out_type);
        } else {
            list_put(n_intf.free_head, &pkt->node);
            d_debug("raw_conf: set ip wrong len: %d\n", pkt->dat_len);
        }
    }
}

static void raw_in_cb(void)
{
    list_node_t *node = list_get(&raw_in.A_head);
    if (node) {
        cdnet_packet_t *pkt = container_of(node, cdnet_packet_t, node);
        if (pkt->dat_len != 0) {
            while (!uart_transmit_is_ready(ur_intf.uart));
            uart_transmit_it(ur_intf.uart, pkt->dat, pkt->dat_len);

            pkt->dat_len = 0;
            list_put(&p_dispr.V_ser_head, node); // send ack
            d_debug("raw_in: in_data\n");
        } else {
            list_put(n_intf.free_head, &pkt->node);
            d_debug("raw_in: empty data\n");
        }
    }
}

static void raw_out_task(void)
{
    static uint32_t t_send;
    static int retry_cnt = 0;
    static list_node_t *ur_node = NULL;

    if (ur_node) {
        list_node_t *node = list_get(&raw_out.A_head);
        if (node) {
            retry_cnt = 0;
            list_put(ur_intf.free_head, ur_node);
            ur_node = NULL;
            d_debug("raw_out: get ack\n");
        } else {
            if (get_systick() - t_send > 100) {
                if (++retry_cnt > 3) {
                    d_error("raw_out: drop\n");
                    retry_cnt = 0;
                    list_put(ur_intf.free_head, ur_node);
                    ur_node = NULL;
                } else {
                    d_warn("raw_out: retry, cnt: %d\n", retry_cnt);
                }
            } else {
                // wait until timeout
                return;
            }
        }
    }

    if (ur_intf.rx_head.first == NULL)
        return;
    if (raw_out_type < 0) {
        d_debug("raw_out: ip not ready\n");
        list_put(ur_intf.free_head, list_get(&ur_intf.rx_head));
        return;
    }
    list_node_t *n_node = list_get(n_intf.free_head);
    if (!n_node) {
        d_error("raw_out: no free lo_pkt\n");
        return;
    }
    if (!ur_node)
        ur_node = list_get(&ur_intf.rx_head);

    ur_frame_t *ur_frame = container_of(ur_node, ur_frame_t, node);
    cdnet_packet_t *n_pkt = container_of(n_node, cdnet_packet_t, node);

    n_pkt->is_local = (raw_out_type == 0);
    n_pkt->dst_mac = raw_out_mac;
    memcpy(n_pkt->dst_addr, raw_out_addr, 2);
    n_pkt->dst_port = RAW_SER_PORT;

    n_pkt->dat_len = ur_frame->len;
    memcpy(n_pkt->dat, ur_frame->dat, ur_frame->len);

    list_put(&raw_out.V_head, n_node);
    t_send = get_systick();
    d_debug("raw_out: send...\n");
}

static void passthru_conf_cb(void)
{
    list_node_t *node = list_get(&passthru_conf.A_head);
    if (node) {
        cdnet_packet_t *pkt = container_of(node, cdnet_packet_t, node);

        if (pkt->dat_len == 1) {
            r_intf.cd_intf.set_filter(&r_intf.cd_intf, pkt->dat[0]);

            pkt->dat_len = 0;
            list_put(&p_dispr.V_ser_head, node);
            d_debug("passthru set_filter: done: %02x\n", pkt->dat[0]);
        } else {
            list_put(n_intf.free_head, node);
            d_debug("passthru set_filter: wrong len\n");
        }
    }
}


static void net_init(void)
{
    int i;

    for (i = 0; i < U_FRAME_MAX; i++)
        list_put(&u_free_head, &u_frame_alloc[i].node);
    for (i = 0; i < R_FRAME_MAX; i++)
        list_put(&r_free_head, &r_frame_alloc[i].node);
    for (i = 0; i < N_PACKET_MAX; i++)
        list_put(&n_free_head, &n_packet_alloc[i].node);

    cdctl_intf_init(&r_intf, &r_free_head, &r_spi);
    if (raw_mode) {
        ur_intf_init(&ur_intf, &u_free_head, &u_uart);
        cdnet_intf_init(&n_intf, &n_free_head, &r_intf.cd_intf, local_mac);
    } else { // passthrough mode
        cduart_intf_init(&uc_intf, &u_free_head, &u_uart);
        uc_intf.remote_filter[0] = 0xaa;
        uc_intf.remote_filter_len = 1;
        uc_intf.local_filter[0] = 0x55;
        uc_intf.local_filter[1] = 0x56;
        uc_intf.local_filter_len = 2;
        ud_init();
        cdnet_intf_init(&n_intf, &n_free_head, &ud_intf, local_mac);
    }
    port_dispatcher_init(&p_dispr, &n_intf);

    get_uid(cpu_id);

    dev_ser.port = 0; // listen for UDP port 0
    list_put(&p_dispr.udp_ser_head, &dev_ser.node);

    if (raw_mode) {
        raw_in.port = RAW_SER_PORT;
        list_put(&p_dispr.udp_ser_head, &raw_in.node);
        raw_conf.port = RAW_CONF_PORT;
        list_put(&p_dispr.udp_ser_head, &raw_conf.node);

        raw_out.begin = raw_out_port_begin;
        raw_out.end = raw_out_port_end;
        raw_out.cur = raw_out.end;
        raw_out.A_en = true;
        list_put(&p_dispr.udp_req_head, &raw_out.node);
    } else {
        passthru_conf.port = PASSTHRU_CONF_PORT;
        list_put(&p_dispr.udp_ser_head, &passthru_conf.node);
    }
}

void net_task(void)
{
    debug_init();
    net_init();
    local_irq_enable();
    d_debug("start net_task...\n");
    if (raw_mode)
        HAL_TIM_Base_Start_IT(&htim1);
    uart_enable_rx_irq();

    while (true) {
        if (!raw_mode)
            cduart_tx_task(&uc_intf);

        cdctl_task(&r_intf);

        if (!raw_mode) {
            r2uc();
            uc2r_ud();
        }

        port_dispatcher_task(&p_dispr);

        dev_ser_cb();
        if (raw_mode) {
            raw_conf_cb();
            raw_out_task();
            raw_in_cb();
        } else {
            passthru_conf_cb();
        }

        debug_flush();
    }
}


// HAL callback

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (raw_mode)
        ur_timer_handler(&ur_intf);
}

static uint8_t uart_tmp_dat;

static void uart_enable_rx_irq(void)
{
    uart_receive_it(&u_uart, &uart_tmp_dat, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t tmp;

    if (u_uart.huart != huart)
        return;

    tmp = uart_tmp_dat;
    uart_receive_it(&u_uart, &uart_tmp_dat, 1);

    if (raw_mode)
        ur_rx_task(&ur_intf, tmp);
    else
        cduart_rx_task(&uc_intf, tmp);
}

#if !defined(CDUART_TX_IT) || !defined(CDUART_RX_IT)
#error "should define CDUART_RX_IT and CDUART_TX_IT!"
#endif
