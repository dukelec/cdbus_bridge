/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

static gpio_t led_r = { .group = RGB_R_GPIO_PORT, .num = RGB_R_PIN };
static gpio_t led_g = { .group = RGB_G_GPIO_PORT, .num = RGB_G_PIN };
static gpio_t led_b = { .group = RGB_B_GPIO_PORT, .num = RGB_B_PIN };
static gpio_t led_tx = { .group = LED_Y_GPIO_PORT, .num = LED_Y_PIN };
static gpio_t led_rx = { .group = LED_G_GPIO_PORT, .num = LED_G_PIN };
static gpio_t sw1 = { .group = SW1_GPIO_PORT, .num = SW1_PIN };
static gpio_t sw2 = { .group = SW2_GPIO_PORT, .num = SW2_PIN };

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

cduart_dev_t d_dev = {0};   // uart / usb

extern otg_core_type otg_core_struct_hs;
static uint8_t usb_rx_buf[512];
static bool cdc_need_flush = false;


static void data_led_task(void)
{
    static uint32_t tx_t_last = 0;
    static uint32_t rx_t_last = 0;
    static uint32_t tx_cnt_last = 0;
    static uint32_t rx_cnt_last = 0;

    if (rx_cnt_last != cdctl_rx_cnt) {
        rx_cnt_last = cdctl_rx_cnt;
        rx_t_last = get_systick();
        gpio_set_val(&led_rx, 1);
    }
    if (tx_cnt_last != cdctl_tx_cnt) {
        tx_cnt_last = cdctl_tx_cnt;
        tx_t_last = get_systick();
        gpio_set_val(&led_tx, 1);
    }

    if (gpio_get_val(&led_rx) == 1 && get_systick() - rx_t_last > 10)
        gpio_set_val(&led_rx, 0);
    if (gpio_get_val(&led_tx) == 1 && get_systick() - tx_t_last > 10)
        gpio_set_val(&led_tx, 0);
}

static void dump_hw_status(void)
{
    static int t_l = 0;
    if (get_systick() - t_l > 8000) {
        t_l = get_systick();

        d_debug("ctl: %d, pend t %ld r %ld, irq %d\n",
                cdctl_state, cdctl_tx_head.len, cdctl_rx_head.len, !CD_INT_RD());
        d_debug("  r %ld (lost %ld err %ld full %ld), t %ld (cd %ld err %ld)\n",
                cdctl_rx_cnt, cdctl_rx_lost_cnt, cdctl_rx_error_cnt, cdctl_rx_no_free_node_cnt,
                cdctl_tx_cnt, cdctl_tx_cd_cnt, cdctl_tx_error_cnt);
        //d_debug("usb: r_cnt %d, t_cnt %d, t_buf %p, t_len %d, t_state %x\n",
        //        usb_rx_cnt, usb_tx_cnt, cdc_tx_buf, cdc_tx_head.len, hcdc->TxState);
    }
}


void app_main(void)
{
    uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);
    cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct_hs.dev.class_handler->pdata;
    cd_frame_t *tx_frame = NULL;
    static int cdc_rate_bk = 0;

    gpio_set_val(&led_tx, 1);
    gpio_set_val(&led_rx, 1);
    delay_systick(1);
    gpio_set_val(&led_tx, 0);
    gpio_set_val(&led_rx, 0);

    printf("\nstart app_main ...\n");

    *stack_check = 0xababcdcd12123434;
    for (int i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);

    load_conf();
    csa.force_115200 = !gpio_get_val(&sw2);
    if (csa.force_115200) {
        csa.bus_cfg.baud_l = 115200;
        csa.bus_cfg.baud_h = 115200;
        printf("force baudrate to 115200 by sw2!\n");
    }

    cduart_dev_init(&d_dev, &frame_free_head);
    d_dev.local_mac = 0xfe;

    common_service_init();

    printf("conf: %s\n", csa.conf_from ? "load from flash" : "use default");
    csa_list_show();

    delay_systick(100);
    gpio_set_val(&led_r, 0);
    delay_systick(200);
    gpio_set_val(&led_r, 1);
    gpio_set_val(&led_b, 0);
    delay_systick(200);
    gpio_set_val(&led_b, 1);
    gpio_set_val(&led_g, 0);

    cdctl_spi_wr_init();
    cdctl_dev_init(&csa.bus_cfg);

    nvic_irq_enable(EXINT0_IRQn, 2, 0);
    nvic_irq_enable(DMA1_Channel1_IRQn, 2, 0);
    exint_interrupt_enable(EXINT_LINE_0, TRUE);

    while (true) {
        if (frame_free_head.len > 5) {
            uint16_t len = usb_vcp_get_rxdata(&otg_core_struct_hs.dev, usb_rx_buf);
            cduart_rx_handle(&d_dev, usb_rx_buf, len);
        }

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

        if (pcdc->linecoding.bitrate == 0xcdcd) {
            common_service_routine();
        } else {
            cd_frame_t *frame;
            while ((frame = cd_list_get(&d_dev.rx_head)) != NULL)
                cdctl_put_tx_frame(frame);
            while ((frame = cd_list_get(&cdctl_rx_head)) != NULL)
                cd_list_put(&d_dev.tx_head, frame);
        }

        data_led_task();
        dump_hw_status();

        if (csa.force_115200 != !gpio_get_val(&sw2)) {
            printf("sw2 changed, reboot...\n");
            NVIC_SystemReset();
        }
        if (!gpio_get_val(&sw1)) {
            printf("sw1 switch on, reboot...\n");
            NVIC_SystemReset();
        }
        if (pcdc->linecoding.bitrate != cdc_rate_bk) {
            bool app_mode = (pcdc->linecoding.bitrate == 0xcdcd);
            d_info("rate: %ld, mode: %s!\n", pcdc->linecoding.bitrate, app_mode ? "config" : "data");
            cdc_rate_bk = pcdc->linecoding.bitrate;
        }

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            while (true);
        }
    }
}

void EXINT0_IRQHandler(void)
{
    EXINT->intsts = EXINT_LINE_0;
    cdctl_int_isr();
}

void DMA1_Channel1_IRQHandler(void)
{
    cdctl_spi_wr_isr();
}
