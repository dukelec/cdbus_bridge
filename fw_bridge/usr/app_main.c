/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

static gpio_t led_r = { .group = RGB_R_GPIO_Port, .num = RGB_R_Pin };
static gpio_t led_g = { .group = RGB_G_GPIO_Port, .num = RGB_G_Pin };
static gpio_t led_b = { .group = RGB_B_GPIO_Port, .num = RGB_B_Pin };
static gpio_t led_tx = { .group = LED_Y_GPIO_Port, .num = LED_Y_Pin };
static gpio_t led_rx = { .group = LED_B_GPIO_Port, .num = LED_B_Pin };
static gpio_t sw1 = { .group = SW1_GPIO_Port, .num = SW1_Pin };
static gpio_t sw2 = { .group = SW2_GPIO_Port, .num = SW2_Pin };

static gpio_t r_int = { .group = CD_INT_GPIO_Port, .num = CD_INT_Pin };
static gpio_t r_cs = { .group = CD_SS_GPIO_Port, .num = CD_SS_Pin };
static spi_t r_spi = {
        .spi = SPI1,
        .ns_pin = &r_cs,
        .dma_rx = DMA1,
        .dma_ch_rx = DMA1_Channel1,
        .dma_ch_tx = DMA1_Channel2,
        .dma_mask = (2 << 0)
};

#define CDC_RX_MAX  20
static cdc_rx_buf_t cdc_rx_alloc[CDC_RX_MAX];
list_head_t cdc_rx_free_head = {0};
list_head_t cdc_rx_head = {0};
cdc_rx_buf_t * volatile cdc_rx_buf = NULL;

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

cduart_dev_t d_dev = {0};   // uart / usb
cdctl_dev_t r_dev = {0};    // cdbus

volatile int usb_rx_cnt = 0;
volatile int usb_tx_cnt = 0;
static bool cdc_need_flush = false;
volatile uint8_t cdc_dtr;
volatile uint32_t cdc_rate;
static uint32_t cdc_rate_final = 115200;
volatile uint32_t pendsv_val = 0;


static void data_led_task(void)
{
    static uint32_t tx_t_last = 0;
    static uint32_t rx_t_last = 0;
    static uint32_t tx_cnt_last = 0;
    static uint32_t rx_cnt_last = 0;

    if (rx_cnt_last != r_dev.rx_cnt) {
        rx_cnt_last = r_dev.rx_cnt;
        rx_t_last = get_systick();
        gpio_set_val(&led_rx, 1);
    }
    if (tx_cnt_last != r_dev.tx_cnt) {
        tx_cnt_last = r_dev.tx_cnt;
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
        //USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        t_l = get_systick();

        d_debug("ctl: pend t %ld r %ld, irq %d\n",
                r_dev.tx_head.len, r_dev.rx_head.len, !gpio_get_val(&r_int));
        d_debug("  r %ld (lost %ld err %ld full %ld), t %ld (cd %ld err %ld)\n",
                r_dev.rx_cnt, r_dev.rx_lost_cnt, r_dev.rx_error_cnt, r_dev.rx_no_free_node_cnt,
                r_dev.tx_cnt, r_dev.tx_cd_cnt, r_dev.tx_error_cnt);
        //d_debug("usb: r_cnt %d, t_cnt %d, t_buf %p, t_len %d, t_state %x\n",
        //        usb_rx_cnt, usb_tx_cnt, cdc_tx_buf, cdc_tx_head.len, hcdc->TxState);
    }
}


static void usb_detection(void)
{
    static uint32_t t_usb = 0;
    static uint8_t cdc_dtr_final = 0;
    uint32_t t_cur = get_systick();

    if (hUsbDeviceFS.dev_state < USBD_STATE_CONFIGURED)
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

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
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
    static cd_frame_t *tx_frame = NULL;

    if (csa.usb_online) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

        if (cdc_rate != 0xcdcd)
            cdc_rate_final = cdc_rate;

        if (frame_free_head.len > 5) {
            cdc_rx_buf_t *bf = list_get_entry_it(&cdc_rx_head, cdc_rx_buf_t);
            if (bf) {
                cduart_rx_handle(&d_dev, bf->dat, bf->len);
                list_put_it(&cdc_rx_free_head, &bf->node);

                if (!cdc_rx_buf) {
                    cdc_rx_buf = list_get_entry_it(&cdc_rx_free_head, cdc_rx_buf_t);
                    d_verbose("continue CDC Rx\n");
                    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf->dat);
                    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
                }
            }
        }

        if (hcdc->TxState == 0) {
            if (tx_frame) {
                cd_list_put(&frame_free_head, tx_frame);
                tx_frame = NULL;
            }

            tx_frame = cd_list_get(&d_dev.tx_head);
            if (tx_frame) {
                cduart_fill_crc(tx_frame->dat);
                CDC_Transmit_FS(tx_frame->dat, tx_frame->dat[2] + 5);
                cdc_need_flush = true;
            } else if (cdc_need_flush) {
                CDC_Transmit_FS(NULL, 0);
                cdc_need_flush = false;
            }
        }
    }

    if (cdc_rate == 0xcdcd) {
        common_service_routine();
    } else {
        cd_frame_t *frame;
        while ((frame = cd_list_get(&d_dev.rx_head)) != NULL)
            cdctl_put_tx_frame(&r_dev.cd_dev, frame);
        while ((frame = cd_list_get(&r_dev.rx_head)) != NULL)
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

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 3, 0);

    *stack_check = 0xababcdcd12123434;
    for (int i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);
    for (int i = 0; i < CDC_RX_MAX; i++)
        list_put(&cdc_rx_free_head, &cdc_rx_alloc[i].node);
    cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_rx_buf_t);

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

    spi_wr_init(&r_spi);
    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, &r_int, EXTI0_1_IRQn);

    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
    pendsv_val = SCB_ICSR_PENDSVSET_Msk; // enable pendsv

    while (true) {
        usb_detection();
        SCB->ICSR = pendsv_val;

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
        uint32_t baud_l = csa.bus_cfg.mode ? baud_h : min(baud_h, baud_limit);

        if (cdctl_baud_l != baud_l || cdctl_baud_h != baud_h) {
            gpio_set_val(&led_g, 1);
            gpio_set_val(&led_b, 0);
            t_update_baud = get_systick();
            cdctl_baud_l = baud_l;
            cdctl_baud_h = baud_h;
            pendsv_val = 0; // disable pendsv
            cdctl_set_clk(&r_dev, cdctl_baud_h);
            cdctl_set_baud_rate(&r_dev, cdctl_baud_l, cdctl_baud_h);
            cdctl_flush(&r_dev);
            cdctl_get_baud_rate(&r_dev, &csa.bus_cfg.baud_l, &csa.bus_cfg.baud_h);
            pendsv_val = SCB_ICSR_PENDSVSET_Msk; // enable pendsv
            d_debug("cdctl: get baud rate: %lu %lu\n", csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
        }

        if (gpio_get_val(&led_g) && get_systick() - t_update_baud > 100) {
            gpio_set_val(&led_b, 1);
            gpio_set_val(&led_g, 0);
        }
    }
}


void cdctl_rx_cb(cdctl_dev_t *dev, cd_frame_t *frame)
{
    SCB->ICSR = pendsv_val;
}

void EXTI0_1_IRQHandler(void)
{
    __HAL_GPIO_EXTI_CLEAR_FALLING_IT(CD_INT_Pin);
    cdctl_int_isr(&r_dev);
}

void DMA1_Channel1_IRQHandler(void)
{
    r_spi.dma_rx->IFCR = r_spi.dma_mask;
    cdctl_spi_isr(&r_dev);
}
