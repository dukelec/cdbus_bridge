/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

static cd_frame_t *d_conv_frame = NULL;

static uint32_t t_last;
static uint32_t boot_time;
uint32_t *bl_args = (uint32_t *)BL_ARGS;


#define APP_ADDR 0x08009000 // 36K offset

void try_jump_to_app(void)
{
    uint32_t stack = *(uint32_t *)APP_ADDR;
    uint32_t func = *(uint32_t *)(APP_ADDR + 4);
    
    printf("bl_args: %08lx, sw1: %d\n", *bl_args, !gpio_get_val(&sw1));
    if (!gpio_get_val(&sw1) || *bl_args == 0xcdcd0001) {
        printf("stay in bl...\n");
        return;
    }

    gpio_set_val(&led_g, 1);
    printf("jump to app...\n");
    while (!__HAL_UART_GET_FLAG(debug_uart.huart, UART_FLAG_TC));
    HAL_UART_DeInit(debug_uart.huart);
    //USBD_Stop(&hUsbDeviceFS);
    //USBD_DeInit(&hUsbDeviceFS);
    HAL_RCC_DeInit();

    // NOTE: change app's SCB->VTOR in app's system_stm32fxxx.c
    //for(int i = 0; i < 256; i++)
    //    HAL_NVIC_DisableIRQ(i);
    HAL_NVIC_DisableIRQ(SysTick_IRQn);
    __set_MSP(stack); // init stack pointer
    ((void(*)()) func)();
    while (true);
}


void bl_init(void)
{
    t_last = get_systick();
    boot_time = get_systick();
    d_conv_frame = list_get_entry(&frame_free_head, cd_frame_t);
}

static void read_from_host(const uint8_t *buf, int size,
        const uint8_t *wr, const uint8_t *rd)
{
    if (rd > wr) {
        cduart_rx_handle(&d_dev, rd, buf + size - rd);
        rd = buf;
    }
    if (rd < wr)
        cduart_rx_handle(&d_dev, rd, wr - rd);
}

void bl_routine(void)
{
    if (get_systick() - t_last > 300000 / CD_SYSTICK_US_DIV) {
        t_last = get_systick();
        gpio_set_val(&led_g, !gpio_get_val(&led_g));
    }
    //if (!csa.keep_in_bl && get_systick() - boot_time > 3000000 / SYSTICK_US_DIV)
    //    jump_to_app();

    // handle data exchange

    if (csa.usb_online) {
        int size;
        uint8_t *wr, *rd;
        cdc_buf_t *bf = list_get_entry_it(&cdc_rx_head, cdc_buf_t);
        if (bf) {
            uint32_t flags;
            size = bf->len + 1; // avoid scroll to begin
            wr = bf->dat + bf->len;
            rd = bf->dat;
            read_from_host(bf->dat, size, wr, rd);

            local_irq_save(flags);
            list_put(&cdc_rx_free_head, &bf->node);
            if (!cdc_rx_buf) {
                cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_buf_t);
                //printf("continue CDC Rx\n");
                USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf->dat);
                USBD_CDC_ReceivePacket(&hUsbDeviceFS);
            }
            local_irq_restore(flags);
        }
    }

    cdc_buf_t *bf = NULL;
    if (!cdc_tx_head.last) {
        bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
        if (!bf) {
            printf("no cdc_tx_free (tx idle)\n");
            return;
        }
        bf->len = 0;
        list_put(&cdc_tx_head, &bf->node);
    } else {
        bf = list_entry(cdc_tx_head.last, cdc_buf_t);
    }

    // send to host
    if (d_dev.tx_head.first) { // send d_dev.tx_head
        cd_frame_t *frm = list_entry(d_dev.tx_head.first, cd_frame_t);

        if (bf->len + frm->dat[2] + 5 > 512) {
            bf = list_get_entry(&cdc_tx_free_head, cdc_buf_t);
            if (!bf) {
                printf("no cdc_tx_free (d_dev)\n");
                return;
            }
            bf->len = 0;
            list_put(&cdc_tx_head, &bf->node);
        }

        //printf("local ret: 55, dat len %d\n", frm->dat[2]);
        cduart_fill_crc(frm->dat);
        memcpy(bf->dat + bf->len, frm->dat, frm->dat[2] + 5);
        bf->len += frm->dat[2] + 5;

        list_get(&d_dev.tx_head);
        list_put_it(d_dev.free_head, &frm->node);
    }
}
