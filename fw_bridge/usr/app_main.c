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

cduart_dev_t d_dev = {0};   // usb cdc

static uint8_t usb_rx_buf[512];
static bool cdc_need_flush = false;
static uint32_t cdc_rate;
static uint32_t cdc_rate_final = 115200;

bool raw_mode = false; // raw mode: use usart1 instead of cdctl, no data format restrictions


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


static void usb_detection(void)
{
    static uint32_t t_usb = 0;
    static uint8_t cdc_dtr_final = 0;
    uint32_t t_cur = get_systick();

    if (otg_core_struct_hs.dev.conn_state < USB_CONN_STATE_CONFIGURED)
        cdc_dtr = 0;

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
        csa.usb_online = true;
        printf("usb: 0 -> 1 (baudrate %ld)\n", cdc_rate);
    }
}


void PendSV_Handler(void)
{
    cdc_struct_type *pcdc = (cdc_struct_type *)otg_core_struct_hs.dev.class_handler->pdata;
    cdc_rate = pcdc->linecoding.bitrate;
    static cd_frame_t *tx_frame = NULL;

    if (csa.usb_online) {
        if (cdc_rate != 0xcdcd) {
            cdc_rate_final = cdc_rate;
            if (csa.bus_cfg.mode >= 4)
                raw_mode = true;
        } else {
            raw_mode = false;
        }

        if (frame_free_head.len > 5) {
            uint16_t len = usb_vcp_get_rxdata(&otg_core_struct_hs.dev, usb_rx_buf);
            if (raw_mode) {
                uint8_t *p = usb_rx_buf;
                while (len) {
                    cd_frame_t *frm = NULL;
                    if (raw_tx_head.len) {
                        frm = cd_list_get_last(&raw_tx_head);
                        if (frm->dat[257] == 255) {
                            cd_list_put(&raw_tx_head, frm);
                            frm = NULL;
                        }
                    }
                    if (!frm) {
                        frm = cd_list_get(&frame_free_head);
                        frm->dat[257] = 0;
                    }
                    if (!frm)
                        break;
                    cdctl_tx_cnt++;
                    unsigned sub_len = min(255 - frm->dat[257], len);
                    memcpy(frm->dat + frm->dat[257], p, sub_len);
                    frm->dat[257] += sub_len;
                    cd_list_put(&raw_tx_head, frm);
                    uart_dma_tx();
                    p += sub_len;
                    len -= sub_len;
                }
            } else {
                cduart_rx_handle(&d_dev, usb_rx_buf, len);
            }
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
            } else {
                tx_frame = cd_list_get(&raw_rx_head);
                if (tx_frame) {
                    cdctl_rx_cnt++;
                    usb_vcp_send_data(&otg_core_struct_hs.dev, tx_frame->dat, tx_frame->dat[257]);
                    cdc_need_flush = true;
                }
            }

            if (!tx_frame && cdc_need_flush) {
                usb_vcp_send_data(&otg_core_struct_hs.dev, NULL, 0);
                cdc_need_flush = false;
            }
        }
    }

    uart_dma_rx();

    if (cdc_rate == 0xcdcd) {
        common_service_routine();
    } else if (!raw_mode) {
        cd_frame_t *frame;
        while ((frame = cd_list_get(&d_dev.rx_head)) != NULL)
            cdctl_put_tx_frame(frame);
        while ((frame = cd_list_get(&cdctl_rx_head)) != NULL)
            cd_list_put(&d_dev.tx_head, frame);
    }
}


void app_main(void)
{
    volatile uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);
    uint32_t cdc_rate_bk = 0;
    uint32_t cdctl_baud_l = 115200;
    uint32_t cdctl_baud_h = 115200;
    uint32_t t_update_baud = 0;

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
    csa.bus_cfg.baud_l = csa.bus_cfg.baud_h = 115200;
    cduart_dev_init(&d_dev, &frame_free_head);
    d_dev.local_mac = 0xff;

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

    if (csa.bus_cfg.mode < 4) {
        nvic_irq_enable(EXINT0_IRQn, 2, 0);
        nvic_irq_enable(DMA1_Channel1_IRQn, 2, 0);
        exint_interrupt_enable(EXINT_LINE_0, TRUE);
    } else {
        wk_usart1_init();
        nvic_irq_enable(DMA2_Channel2_IRQn, 2, 0); // uart_tx
        nvic_irq_enable(USART1_IRQn, 2, 0);
        uart_dma_wr_init();
        usart_interrupt_enable(UART_DEV, USART_TDC_INT, true);
    }

    while (true) {
        usb_detection();
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;

        data_led_task();
        dump_hw_status();

        if (!gpio_get_val(&sw1)) {
            printf("sw1 switch on, reboot...\n");
            NVIC_SystemReset();
        }

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            while (true);
        }

        if (cdc_rate != cdc_rate_bk) {
            d_info("rate: %ld, mode: %s!\n", cdc_rate, cdc_rate == 0xcdcd ? "config" : "data");
            cdc_rate_bk = cdc_rate;
        }

        bool sw2_val = !gpio_get_val(&sw2);
        uint32_t baud_limit = sw2_val ? csa.limit_baudrate1 : csa.limit_baudrate0;
        uint32_t baud_h = cdc_rate_final;
        uint32_t baud_l = csa.bus_cfg.mode == 1 ? min(baud_h, baud_limit) : baud_h;

        if (cdctl_baud_l != baud_l || cdctl_baud_h != baud_h) {
            gpio_set_val(&led_g, 1);
            gpio_set_val(&led_b, 0);
            t_update_baud = get_systick();
            cdctl_baud_l = baud_l;
            cdctl_baud_h = baud_h;
            __set_BASEPRI(0xc0); // disable pendsv
            if (!raw_mode) {
                cdctl_set_clk(cdctl_baud_h);
                cdctl_set_baud_rate(cdctl_baud_l, cdctl_baud_h);
                cdctl_flush();
                cdctl_get_baud_rate(&csa.bus_cfg.baud_l, &csa.bus_cfg.baud_h);
            } else {
                crm_clocks_freq_type clocks_freq;
                crm_clocks_freq_get(&clocks_freq);
                usart_enable(UART_DEV, false);
                USART1->baudr = max(DIV_ROUND_CLOSEST(clocks_freq.apb2_freq, baud_h), 16);
                csa.bus_cfg.baud_l = csa.bus_cfg.baud_h = DIV_ROUND_CLOSEST(clocks_freq.apb2_freq, USART1->baudr);
                usart_enable(UART_DEV, true);
            }
            __set_BASEPRI(0); // enable pendsv
            d_debug("get baud rate: %lu %lu\n", csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
        }

        if (gpio_get_val(&led_g) && get_systick() - t_update_baud > 100) {
            gpio_set_val(&led_b, 1);
            gpio_set_val(&led_g, 0);
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

void DMA2_Channel2_IRQHandler(void)
{
    uart_dma_isr();
}

void USART1_IRQHandler(void)
{
    uart_tdc_isr();
}

