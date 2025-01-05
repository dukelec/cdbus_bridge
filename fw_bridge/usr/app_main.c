/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

gpio_t led_g = { .group = RGB_G_GPIO_PORT, .num = RGB_G_PIN };
gpio_t sw1 = { .group = SW1_GPIO_PORT, .num = SW1_PIN };

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

cduart_dev_t d_dev = {0};   // uart / usb

extern otg_core_type otg_core_struct_hs;
static uint8_t usb_rx_buf[512];
static bool cdc_need_flush = false;


void app_main(void)
{
    uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);
    cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct_hs.dev.class_handler->pdata;
    cd_frame_t *tx_frame = NULL;
    static uint32_t t_last = 0;

    printf("\nstart app_main ...\n");

    *stack_check = 0xababcdcd12123434;
    for (int i = 0; i < FRAME_MAX; i++)
        list_put(&frame_free_head, &frame_alloc[i].node);

    load_conf();

    cduart_dev_init(&d_dev, &frame_free_head);
    d_dev.local_mac = 0xfe;

    common_service_init();

    printf("conf: %s\n", csa.conf_from ? "load from flash" : "use default");
    gpio_set_value(&led_g, 0);

    while (true) {
        uint16_t len = usb_vcp_get_rxdata(&otg_core_struct_hs.dev, usb_rx_buf);
        cduart_rx_handle(&d_dev, usb_rx_buf, len);

        if (pcdc->g_tx_completed) {
            if (tx_frame) {
                list_put_it(&frame_free_head, &tx_frame->node);
                tx_frame = NULL;
            }

            tx_frame = list_get_entry_it(&d_dev.tx_head, cd_frame_t);
            if (tx_frame) {
                cduart_fill_crc(tx_frame->dat);
                usb_vcp_send_data(&otg_core_struct_hs.dev, tx_frame->dat, tx_frame->dat[2] + 5);
                cdc_need_flush = true;
            } else if (cdc_need_flush) {
                usb_vcp_send_data(&otg_core_struct_hs.dev, NULL, 0);
                cdc_need_flush = false;
            }
        }

        common_service_routine();

        if (get_systick() - t_last > 300000 / SYSTICK_US_DIV) {
            t_last = get_systick();
            gpio_set_value(&led_g, !gpio_get_value(&led_g));
        }

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            while (true);
        }
    }
}
