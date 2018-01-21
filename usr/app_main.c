/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"
#include "cdctl_bx.h"
#include "usb_device.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern USBD_HandleTypeDef hUsbDeviceFS;

gpio_t led1 = { .group = LED1_GPIO_Port, .num = LED1_Pin };
gpio_t led2 = { .group = LED2_GPIO_Port, .num = LED2_Pin };

uart_t debug_uart = { .huart = &huart1 };
static uart_t ttl_uart; // = { .huart = &huart3 };
static uart_t rs232_uart; // = { .huart = &huart2 };
uart_t *hw_uart = NULL;

static gpio_t r_rst_n = { .group = CDCTL_RST_N_GPIO_Port, .num = CDCTL_RST_N_Pin };
static gpio_t r_ns = { .group = CDCTL_NS_GPIO_Port, .num = CDCTL_NS_Pin };
static spi_t r_spi = { .hspi = &hspi1, .ns_pin = &r_ns };

// RS485
#define R_FRAME_MAX 10
static cd_frame_t r_frame_alloc[R_FRAME_MAX];
static list_head_t r_free_head = {0};
static cdctl_intf_t r_intf = {0};

// CDNET
#define N_PACKET_MAX 10
static cdnet_packet_t n_packet_alloc[N_PACKET_MAX];
static list_head_t n_free_head = {0};
cdnet_intf_t n_intf = {0};

// dummy interface for UART, for pass-thru mode only
#define D_FRAME_MAX 4
static cd_frame_t d_frame_alloc[D_FRAME_MAX];
static list_head_t d_free_head = {0};
static list_head_t d_rx_head = {0};
list_head_t d_tx_head = {0};
static cd_intf_t d_intf = {0};

static list_node_t *d_get_free_node(cd_intf_t *_)
{
    return list_get(&d_free_head);
}
static void d_put_free_node(cd_intf_t *_, list_node_t *node)
{
    list_put(&d_free_head, node);
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
    for (i = 0; i < R_FRAME_MAX; i++)
        list_put(&r_free_head, &r_frame_alloc[i].node);
    for (i = 0; i < N_PACKET_MAX; i++)
        list_put(&n_free_head, &n_packet_alloc[i].node);
    for (i = 0; i < D_FRAME_MAX; i++)
        list_put(&d_free_head, &d_frame_alloc[i].node);

    cdctl_intf_init(&r_intf, &r_free_head, &r_spi, &r_rst_n);
    r_intf.cd_intf.set_baud_rate(&r_intf.cd_intf,
            app_conf.rs485_baudrate_low, app_conf.rs485_baudrate_high);

    d_init();
    if (app_conf.mode == APP_PASS_THRU) {
        cdnet_intf_init(&n_intf, &n_free_head, &d_intf, 0x55);
        n_intf.net = 0;
    } else {
        cdnet_intf_init(&n_intf, &n_free_head, &r_intf.cd_intf,
                app_conf.rs485_mac);
        n_intf.net = app_conf.rs485_net;
    }

    if (app_conf.intf_idx == INTF_TTL) {
        hw_uart = &ttl_uart;
        // TODO: enable ttl_uart interrupt
    } else if (app_conf.intf_idx == INTF_RS232) {
        hw_uart = &rs232_uart;
        // TODO: enable rs232_uart interrupt
    }
}

void set_led_state(led_state_t state)
{

}

void app_main(void)
{
    debug_init();
    device_init();
    local_irq_enable();
    d_debug("start net_task...\n");

    while (true) {
        if (app_conf.intf_idx != INTF_USB &&
                hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            d_info("usb connected\n");
            app_conf.intf_idx = INTF_USB;
        }

        cdctl_task(&r_intf);

        debug_flush();
    }
}

