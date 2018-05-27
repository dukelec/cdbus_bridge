/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern SPI_HandleTypeDef hspi1;
extern USBD_HandleTypeDef hUsbDeviceFS;

extern list_head_t raw2u_head; // raw mode only

static  gpio_t led_r = { .group = LED_R_GPIO_Port, .num = LED_R_Pin };
static  gpio_t led_g = { .group = LED_G_GPIO_Port, .num = LED_G_Pin };
static  gpio_t led_b = { .group = LED_B_GPIO_Port, .num = LED_B_Pin };
static  gpio_t led_tx = { .group = LED_TX_GPIO_Port, .num = LED_TX_Pin };
static  gpio_t led_rx = { .group = LED_RX_GPIO_Port, .num = LED_RX_Pin };

uart_t debug_uart = { .huart = &huart4 };
static uart_t ttl_uart = { .huart = &huart1 };
static uart_t rs232_uart = { .huart = &huart2 };
uart_t *hw_uart = NULL;

static gpio_t r_rst_n = { .group = CDCTL_RST_N_GPIO_Port, .num = CDCTL_RST_N_Pin };
static gpio_t r_int_n = { .group = CDCTL_INT_N_GPIO_Port, .num = CDCTL_INT_N_Pin };
static gpio_t r_ns = { .group = CDCTL_NS_GPIO_Port, .num = CDCTL_NS_Pin };
static spi_t r_spi = { .hspi = &hspi1, .ns_pin = &r_ns };

#define CDC_RX_MAX 6
#define CDC_TX_MAX 6
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
list_head_t frame_free_head = {0};

#define PACKET_MAX 10
static cdnet_packet_t packet_alloc[PACKET_MAX];
list_head_t packet_free_head = {0};

cdctl_intf_t r_intf = {0};   // RS485
cdnet_intf_t n_intf = {0};   // CDNET

int usb_rx_cnt = 0;
int usb_tx_cnt = 0;

uint8_t circ_buf[CIRC_BUF_SZ];
uint32_t rd_pos = 0;


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

    cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_buf_t);

    cdctl_intf_init(&r_intf, &frame_free_head, app_conf.rs485_addr.mac,
            app_conf.rs485_baudrate_low, app_conf.rs485_baudrate_high,
            &r_spi, &r_rst_n, &r_int_n);

    if (app_conf.ser_idx == SER_TTL) {
        hw_uart = &ttl_uart;
    } else if (app_conf.ser_idx == SER_RS232) {
        hw_uart = &rs232_uart;
    }
}

void set_led_state(led_state_t state)
{
    static bool is_err = false;
    if (is_err)
        return;

    switch (state) {
    case LED_POWERON:
        gpio_set_value(&led_r, 1);
        gpio_set_value(&led_g, 1);
        gpio_set_value(&led_b, 0);
        break;
    case LED_WARN:
        gpio_set_value(&led_r, 0);
        gpio_set_value(&led_g, 0);
        gpio_set_value(&led_b, 1);
        break;
    default:
    case LED_ERROR:
        is_err = true;
        gpio_set_value(&led_r, 0);
        gpio_set_value(&led_g, 1);
        gpio_set_value(&led_b, 1);
        break;
    }
}

static void data_led_task(void)
{
    static uint32_t tx_t_last = 0;
    static uint32_t rx_t_last = 0;
    static uint32_t tx_cnt_last = 0;
    static uint32_t rx_cnt_last = 0;

    if (rx_cnt_last != r_intf.rx_cnt) {
        rx_cnt_last = r_intf.rx_cnt;
        rx_t_last = get_systick();
        gpio_set_value(&led_rx, 0);
    }
    if (tx_cnt_last != r_intf.tx_cnt) {
        tx_cnt_last = r_intf.tx_cnt;
        tx_t_last = get_systick();
        gpio_set_value(&led_tx, 0);
    }

    if (gpio_get_value(&led_rx) == 0 && get_systick() - rx_t_last > 10)
        gpio_set_value(&led_rx, 1);
    if (gpio_get_value(&led_tx) == 0 && get_systick() - tx_t_last > 10)
        gpio_set_value(&led_tx, 1);
}


extern uint32_t end; // end of bss
#define STACK_CHECK_SKIP (5 * 1024)
#define STACK_CHECK_SIZE (64 + STACK_CHECK_SKIP)

static void stack_check_init(void)
{
    int i;
    d_debug("stack_check_init: skip: %p ~ %p, to %p\n",
            &end, &end + STACK_CHECK_SKIP, &end + STACK_CHECK_SIZE);
    for (i = STACK_CHECK_SKIP; i < STACK_CHECK_SIZE; i+=4)
        *(uint32_t *)(&end + i) = 0xababcdcd;
}

static void stack_check(void)
{
    int i;
    for (i = STACK_CHECK_SKIP; i < STACK_CHECK_SIZE; i+=4) {
        if (*(uint32_t *)(&end + i) != 0xababcdcd) {
            printf("stack overflow %p (skip: %p ~ %p): %08lx\n",
                    &end + i, &end, &end + STACK_CHECK_SKIP, *(uint32_t *)(&end + i));
            while (true);
        }
    }
}

static void dump_hw_status(void)
{
    static int t_l = 0;
    if (get_systick() - t_l > 8000) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        t_l = get_systick();
        d_debug("bx: state: %d, tx_l: %d, rx_l: %d, p %d\n",
                r_intf.state, r_intf.tx_head.len, r_intf.rx_head.len,
                gpio_get_value(r_intf.int_n));
        d_debug("bx: cnt: rx: %d %d (%d %d), tx: %d %d (%d)\n",
                r_intf.rx_lost_cnt, r_intf.rx_error_cnt,
                r_intf.rx_cnt, r_intf.rx_no_free_node_cnt,
                r_intf.tx_cd_cnt, r_intf.tx_error_cnt, r_intf.tx_cnt);
        d_debug("usb: rx: %d, tx: %d, tx_buf: %p, tx_l: %d, t_s: %x\n",
                usb_rx_cnt, usb_tx_cnt, cdc_tx_buf, cdc_tx_head.len, hcdc->TxState);
        d_debug("net: tx_l: %d\n", n_intf.tx_head.len);
    }
}


void app_main(void)
{
    debug_init();
    stack_check_init();
    load_conf();
    init_info_str();
    device_init();
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    d_debug("start app_main...\n");
    set_led_state(LED_POWERON);

    if (app_conf.mode == APP_BRIDGE)
        app_bridge_init();
    else
        app_raw_init();

    if (app_conf.ser_idx != SER_USB)
        HAL_UART_Receive_DMA(hw_uart->huart, circ_buf, CIRC_BUF_SZ);

    while (true) {
        data_led_task();
        stack_check();
        dump_hw_status();

        if (app_conf.ser_idx != SER_USB &&
                hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            d_info("usb connected\n");
            app_conf.ser_idx = SER_USB;
            HAL_UART_DMAStop(hw_uart->huart);
        }

        // handle cdnet
        cdnet_rx(&n_intf);
        cdnet_packet_t *pkt = cdnet_packet_get(&n_intf.rx_head);
        if (pkt) {
            if (pkt->level == CDNET_L2 || pkt->src_port < CDNET_DEF_PORT ||
                    pkt->dst_port >= CDNET_DEF_PORT) {
                d_warn("unexpected pkg port\n");
                list_put(n_intf.free_head, &pkt->node);
            } else {
                switch (pkt->dst_port) {
                case 1: // device info
                    p1_service(pkt);
                    break;
                case 3: // address config
                    if (app_conf.mode == APP_BRIDGE)
                        p3_service_for_bridge(pkt);
                    else
                        p3_service_for_raw(pkt);
                    break;
                case 10: // reboot; save config ...
                    p10_service(pkt);
                    break;
                case 11: // flash read, write and erase
                    p11_service(pkt);
                    break;
                case RAW_SER_PORT: // receive raw data
                    if (app_conf.mode == APP_RAW) {
                        list_put(&raw2u_head, &pkt->node);
                        break;
                    }
                    // no break;
                default:
                    d_warn("unknown pkg\n");
                    list_put(n_intf.free_head, &pkt->node);
                }
            }

        }
        cdnet_tx(&n_intf);

        if (app_conf.mode == APP_BRIDGE)
            app_bridge();
        else
            app_raw();

        if (cdc_tx_buf) {
            if (app_conf.ser_idx == SER_USB) {
                USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
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
            cdc_buf_t *bf = list_entry(cdc_tx_head.first, cdc_buf_t);
            if (bf->len != 0) {
                if (app_conf.ser_idx == SER_USB) {
                    local_irq_disable();
                    CDC_Transmit_FS(bf->dat, bf->len);
                    local_irq_enable();
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
