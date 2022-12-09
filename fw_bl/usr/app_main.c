/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

extern UART_HandleTypeDef huart5;
extern USBD_HandleTypeDef hUsbDeviceFS;

gpio_t led_r = { .group = RGB_R_GPIO_Port, .num = RGB_R_Pin };
gpio_t led_g = { .group = RGB_G_GPIO_Port, .num = RGB_G_Pin };
gpio_t led_b = { .group = RGB_B_GPIO_Port, .num = RGB_B_Pin };
static  gpio_t sw1 = { .group = SW1_GPIO_Port, .num = SW1_Pin };
//static  gpio_t sw2 = { .group = SW2_GPIO_Port, .num = SW2_Pin };

uart_t debug_uart = { .huart = &huart5 };

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

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

static cdn_pkt_t packet_alloc[PACKET_MAX];
list_head_t packet_free_head = {0};

cduart_dev_t d_dev = {0};   // uart / usb
cdn_ns_t dft_ns = {0};      // CDNET

int usb_rx_cnt = 0;
int usb_tx_cnt = 0;
static bool cdc_need_flush = false;

uint8_t circ_buf[CIRC_BUF_SZ];
uint32_t rd_pos = 0;


static void device_init(void)
{
    int i;
    cdn_init_ns(&dft_ns, &packet_free_head);

    for (i = 0; i < CDC_RX_MAX; i++)
        list_put(&cdc_rx_free_head, &cdc_rx_alloc[i].node);
    for (i = 0; i < CDC_TX_MAX; i++)
        list_put(&cdc_tx_free_head, &cdc_tx_alloc[i].node);
    for (i = 0; i < FRAME_MAX; i++)
        list_put(&frame_free_head, &frame_alloc[i].node);
    for (i = 0; i < PACKET_MAX; i++)
        list_put(&packet_free_head, &packet_alloc[i].node);

    cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_buf_t);

    cduart_dev_init(&d_dev, &frame_free_head);
    d_dev.remote_filter[0] = 0xaa;
    d_dev.remote_filter_len = 1;
    d_dev.local_filter[0] = 0x55;
    d_dev.local_filter_len = 1;
    
    //                    uart / usb
    cdn_add_intf(&dft_ns, &d_dev.cd_dev, 0, 0x55);
    ///hw_uart = &ttl_uart;
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


extern uint32_t end; // end of bss
#define STACK_CHECK_SKIP 0x200
#define STACK_CHECK_SIZE (64 + STACK_CHECK_SKIP)

static void stack_check_init(void)
{
    int i;
    printf("stack_check_init: skip: %p ~ %p, to %p\n",
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


void app_main(void)
{
    printf("\nstart app_main (bl)...\n");
    stack_check_init();
    load_conf();
    debug_init(&dft_ns, &csa.dbg_dst, &csa.dbg_en);
    device_init();
    common_service_init();
    printf("conf: %s\n", csa.conf_from ? "load from flash" : "use default");
    set_led_state(LED_POWERON);
    bl_init();

    while (true) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

        if (!csa.usb_online && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
            printf("usb connected\n");
            csa.usb_online = true;
        }

        if (cdc_tx_buf) {
            if (csa.usb_online) {
                if (hcdc->TxState == 0) {
                    list_put(&cdc_tx_free_head, &cdc_tx_buf->node);
                    cdc_tx_buf = NULL;
                }
            } // else wait online
        }
        if (cdc_tx_head.first) {
            if (!cdc_tx_buf && hcdc->TxState == 0) {
                cdc_buf_t *bf = list_entry(cdc_tx_head.first, cdc_buf_t);
                if (bf->len != 0) {
                    if (csa.usb_online) {
                        local_irq_disable();
                        CDC_Transmit_FS(bf->dat, bf->len);
                        local_irq_enable();
                        cdc_need_flush = true;
                    } // else wait online
                    list_get(&cdc_tx_head);
                    cdc_tx_buf = bf;

                } else if (cdc_need_flush) {
                    local_irq_disable();
                    CDC_Transmit_FS(NULL, 0);
                    local_irq_enable();
                    cdc_need_flush = false;
                }
            }
        }

        stack_check();
        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        bl_routine();
        debug_flush(false);
    }
}
