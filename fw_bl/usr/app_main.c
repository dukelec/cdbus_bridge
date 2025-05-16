/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

gpio_t led_g = { .group = RGB_G_GPIO_Port, .num = RGB_G_Pin };
gpio_t sw1 = { .group = SW1_GPIO_Port, .num = SW1_Pin };

#define CDC_RX_MAX 6
static cdc_rx_buf_t cdc_rx_alloc[CDC_RX_MAX];
list_head_t cdc_rx_free_head = {0};
list_head_t cdc_rx_head = {0};
cdc_rx_buf_t * volatile cdc_rx_buf = NULL;

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

cduart_dev_t d_dev = {0};   // uart / usb

volatile int usb_rx_cnt = 0;
volatile int usb_tx_cnt = 0;
static bool cdc_need_flush = false;
volatile uint8_t cdc_dtr;
uint32_t *bl_args = (uint32_t *)BL_ARGS;

#define APP_ADDR 0x08009000 // 36K offset


void try_jump_to_app(void)
{
    static uint32_t app_func; // not on stack (MSP switches before use)
    uint32_t stack = *(uint32_t *)APP_ADDR;
    app_func = *(uint32_t *)(APP_ADDR + 4);
    bool sw = !gpio_get_val(&sw1);

    uint32_t rcc_csr = RCC->CSR;
    bool por = !!(rcc_csr & RCC_CSR_PWRRSTF);
    printf("\nbl_args: %08lx, rst flg: %08lx (por %d), sw1: %d\n", *bl_args, rcc_csr, por, sw);
    if (por)
        *bl_args = 0xcdcd0000;
    RCC->CSR |= RCC_CSR_RMVF;
    if (*bl_args == 0xcdcd0001 || sw) {
        printf("stay in bl...\n");
        return;
    }

    gpio_set_val(&led_g, 1);
    printf("jump to app...\n");
    while (!__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC));
    HAL_UART_DeInit(&huart5);
    HAL_RCC_DeInit();
    HAL_NVIC_DisableIRQ(SysTick_IRQn);

    __set_MSP(stack); // init stack pointer
    ((void(*)()) app_func)();
    while (true);
}


static void usb_detection(void)
{
    static uint32_t t_usb = 0;
    uint32_t t_cur = get_systick();

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
        cdc_dtr = 0;

    if (!cdc_dtr) {
        t_usb = t_cur;
        if (csa.usb_online)
            printf("usb: 1 -> 0\n");
        csa.usb_online = false;
    } else if (!csa.usb_online && t_cur - t_usb > 5) { // wait for host to turn off echo
        csa.usb_online = true;
        printf("usb: 0 -> 1\n");
    }
}


void app_main(void)
{
    uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);
    cd_frame_t *tx_frame = NULL;
    static uint32_t t_last = 0;

    printf("\nstart app_main (bl)...\n");

    *stack_check = 0xababcdcd12123434;
    for (int i = 0; i < FRAME_MAX; i++)
        cd_list_put(&frame_free_head, &frame_alloc[i]);
    for (int i = 0; i < CDC_RX_MAX; i++)
        list_put(&cdc_rx_free_head, &cdc_rx_alloc[i].node);
    cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_rx_buf_t);

    load_conf();

    cduart_dev_init(&d_dev, &frame_free_head);
    d_dev.local_mac = 0xff;

    common_service_init();

    printf("conf: %s\n", csa.conf_from ? "load from flash" : "use default");
    gpio_set_val(&led_g, 0);

    while (true) {
        usb_detection();

        if (csa.usb_online) {
            USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

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

        common_service_routine();

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
