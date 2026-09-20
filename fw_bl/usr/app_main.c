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

static uint8_t usb_rx_buf[512];
static bool cdc_need_flush = false;
static bool usb_resumed = false;
uint32_t *bl_args = (uint32_t *)BL_ARGS;

#define APP_ADDR 0x08006000 // 24K offset


void try_jump_to_app(void)
{
    static uint32_t func; // not on stack (MSP switches before use)
    uint32_t stack = *(uint32_t *)APP_ADDR;
    func = *(uint32_t *)(APP_ADDR + 4);
    bool sw = !gpio_get_val(&sw1);

    printf("\nbl_args: %08lx, rst flg: %08lx (por %d), sw1: %d\n",
            *bl_args, CRM->ctrlsts, CRM->ctrlsts_bit.porrstf, sw);
    if (CRM->ctrlsts_bit.porrstf)
        *bl_args = 0xcdcd0000;
    CRM->ctrlsts_bit.rstfc = 1;
    if (*bl_args == 0xcdcd0001 || sw) {
        printf("stay in bl...\n");
        return;
    }

    gpio_set_val(&led_g, 1);
    SysTick->CTRL = 0;
    printf("jump to app...\n");
    while (!(UART7->sts & USART_TDC_FLAG));

    __set_MSP(stack); // init stack pointer
    ((void(*)()) func)();
    while (true);
}


static void usb_detection(void)
{
    static uint32_t t_usb = 0;
    static uint8_t cdc_dtr_final = 0;
    uint32_t t_cur = get_systick();

    // cdc_dtr is deliberately not cleared on bus reset: the host only sends
    // SET_CONTROL_LINE_STATE when the application opens or closes the port, so
    // after a reset-resume (pc wake up) it would never be asserted again and
    // the bridge would stay silent until the port is reopened or replugged
    if (!cdc_dtr) {
        t_usb = t_cur;
        if (csa.usb_online)
            printf("usb: 1 -> 0 (!dtr)\n");
        cdc_dtr_final = 0;
        csa.usb_online = false;
    } else if (!cdc_dtr_final && t_cur - t_usb > 5) { // wait for host to turn off echo
        cdc_dtr_final = 1;
        printf("usb: dtr on\n");
    }

    if (otg_core_struct_hs.dev.conn_state != USB_CONN_STATE_CONFIGURED) {
        if (csa.usb_online)
            printf("usb: 1 -> 0 (!state)\n");
        csa.usb_online = false;
    } else if (!csa.usb_online && cdc_dtr_final) {
        usb_resumed = true;
        csa.usb_online = true;
        printf("usb: 0 -> 1\n");
    }
}

void app_main(void)
{
    uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);
    cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct_hs.dev.class_handler->pdata;
    cd_frame_t *tx_frame = NULL;
    static uint32_t t_last = 0;

    printf("\nstart app_main (bl)...\n");

    *stack_check = 0xababcdcd12123434;
    for (int i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);

    load_conf();

    cduart_dev_init(&d_dev, &frame_free_head);
    d_dev.local_mac = 0xff;

    comm_service_init();

    printf("conf: %s\n", csa.conf_from ? "load from flash" : "use default");
    gpio_set_val(&led_g, 0);

    while (true) {
        usb_detection();

        if (csa.usb_online) {
            if (usb_resumed) {
                usb_resumed = false;
                // a transfer armed just before the link went down never
                // completes: usbd_core_in_handler() skips in_handler while the
                // device is not configured, g_tx_completed would stay 0 forever
                pcdc->g_tx_completed = 1;
                if (tx_frame) {
                    cd_list_put(&frame_free_head, tx_frame);
                    tx_frame = NULL;
                }
                cdc_need_flush = false;
            }

            uint16_t len = usb_vcp_get_rxdata(&otg_core_struct_hs.dev, usb_rx_buf);
            cduart_rx_handle(&d_dev, usb_rx_buf, len);

            if (pcdc->g_tx_completed) {
                if (tx_frame) {
                    cd_list_put(&frame_free_head, tx_frame);
                    tx_frame = NULL;
                }

                tx_frame = cd_list_get(&d_dev.tx_head);
                if (tx_frame) {
                    cduart_fill_crc(tx_frame->dat);
                    usb_vcp_send_data(&otg_core_struct_hs.dev, tx_frame->dat, tx_frame->dat[2] + 5);
                    cdc_need_flush = true;
                } else if (cdc_need_flush) {
                    usb_vcp_send_data(&otg_core_struct_hs.dev, NULL, 0);
                    cdc_need_flush = false;
                }
            }
        }

        comm_service_poll();

        if (gpio_get_val(&sw1) && *bl_args != 0xcdcd0001) {
            printf("sw1 switch off, reboot...\n");
            NVIC_SystemReset();
        }

        if (get_systick() - t_last > 300000 / CD_SYSTICK_US_DIV) {
            t_last = get_systick();
            gpio_set_val(&led_g, !gpio_get_val(&led_g));
        }

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            while (true);
        }
    }
}
