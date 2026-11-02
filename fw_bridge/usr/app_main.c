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
static gpio_t r_rst = { .group = OLD_CD_RST_GPIO_Port, .num = OLD_CD_RST_Pin }; // hw v4 only, open drain
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
cdc_rx_buf_t * volatile cdc_rx_buf = NULL;  // the one the out endpoint fills
static cdc_rx_buf_t *rx_bf = NULL;          // the packet being parsed
static uint16_t rx_pos = 0;                 // ... and how much of it is

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

cduart_dev_t d_dev = {0};   // usb cdc
cdctl_dev_t r_dev = {0};    // cdbus

volatile int usb_rx_cnt = 0;
volatile int usb_tx_cnt = 0;
static bool cdc_need_flush = false;
volatile uint8_t cdc_dtr;
volatile uint32_t cdc_rate;
static volatile uint32_t cdc_rate_final = 115200;
static uint32_t cdctl_baud_l = 115200;  // what the bus was last set to
static uint32_t cdctl_baud_h = 115200;
static uint32_t cache_drop_cnt = 0;
volatile uint32_t pendsv_val = 0;


// Queue a frame for the host. Keeps FRAME_RESERVE frames in the free pool so
// the rx paths can never starve; once the pool runs low (host offline, or
// not reading) the oldest queued frame is dropped, so the newest data wins.
void frame_cache_put(list_head_t *head, cd_frame_t *frame)
{
    while (frame_free_head.len < FRAME_RESERVE) {
        cd_frame_t *old = cd_list_get(head);
        if (!old)
            break;
        cd_list_put(&frame_free_head, old);
        cache_drop_cnt++;
    }
    cd_list_put(head, frame);
}


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
        t_l = get_systick();

        d_debug("ctl: %d, pend t %ld r %ld, irq %d\n",
                r_dev.state, r_dev.tx_head.len, r_dev.rx_head.len, !gpio_get_val(r_dev.int_n));
        d_debug("  r %ld (lost %ld err %ld full %ld), t %ld (cd %ld err %ld)\n",
                r_dev.rx_cnt, r_dev.rx_lost_cnt, r_dev.rx_error_cnt, r_dev.rx_no_free_node_cnt,
                r_dev.tx_cnt, r_dev.tx_cd_cnt, r_dev.tx_error_cnt);
        d_debug("  free %ld, cache %ld (drop %ld), usb %d\n",
                frame_free_head.len, d_dev.tx_head.len, cache_drop_cnt, csa.usb_online);
    }
}


static void usb_detection(void)
{
    static uint32_t t_usb = 0;
    static uint8_t cdc_dtr_final = 0;
    uint32_t t_cur = get_systick();

    // a bus reset clears the line state like the rest of the device state;
    // the host asserts dtr again when its program reopens the port, which a
    // reset forces it to. A suspend is not a reset: the state is SUSPENDED
    // then, above CONFIGURED, and the port stays open across it
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


// the out endpoint stays nak'ed once it has no buffer to fill; hand it
// one again. At the usb irq priority, like everything on these lists
static void cdc_rx_resume(void)
{
    if (!cdc_rx_buf && hUsbDeviceFS.pClassData) {
        cdc_rx_buf = list_get_entry_it(&cdc_rx_free_head, cdc_rx_buf_t);
        if (cdc_rx_buf) {
            USBD_CDC_SetRxBuffer(&hUsbDeviceFS, cdc_rx_buf->dat);
            USBD_CDC_ReceivePacket(&hUsbDeviceFS);
        }
    }
}

// drop what the host sent and the parser state: the packets waiting, the
// one being parsed and the half frame the parser is in. From PendSV for a
// closed port and from CDC_Init_FS() after a reset, the same priority
void cdc_rx_drop(void)
{
    cdc_rx_buf_t *bf;

    if (rx_bf) {
        list_put_it(&cdc_rx_free_head, &rx_bf->node);
        rx_bf = NULL;
    }
    while ((bf = list_get_entry_it(&cdc_rx_head, cdc_rx_buf_t)) != NULL)
        list_put_it(&cdc_rx_free_head, &bf->node);
    d_dev.rx_byte_cnt = 0;
    d_dev.rx_crc = 0xffff;
    d_dev.rx_drop = false;
}

// parse what the host sent, as much as the pool can take: a frame every 5
// bytes, so a whole packet could empty the pool under the controller. The
// rest waits in its buffer, then in the endpoint, which naks once the
// buffers run out
static void usb_rx_task(void)
{
    while (true) {
        // signed: the receive interrupt may have drawn on the pool since
        int room = (int)frame_free_head.len - FRAME_RESERVE;
        if (room <= 0)
            break;
        if (!rx_bf) {
            rx_bf = list_get_entry_it(&cdc_rx_head, cdc_rx_buf_t);
            rx_pos = 0;
            if (!rx_bf)
                break;
        }
        unsigned n = min(rx_bf->len - rx_pos, room * 5);
        cduart_rx_handle(&d_dev, rx_bf->dat + rx_pos, n);
        rx_pos += n;
        if (rx_pos == rx_bf->len) {
            list_put_it(&cdc_rx_free_head, &rx_bf->node);
            rx_bf = NULL;
            cdc_rx_resume();
        }
    }
}


void PendSV_Handler(void)
{
    static cd_frame_t *tx_frame = NULL;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    // usb_online follows the usb state a main loop pass later, a reset in
    // between has taken the class data away already
    if (csa.usb_online && hcdc) {
        if (cdc_rate != 0xcdcd)
            cdc_rate_final = cdc_rate;

        // read nothing until the bus is at the rate the host asked for: the
        // main loop applies it after this handler, and the host's first
        // request follows the rate in the same instant
        if (cdc_rate_final == cdctl_baud_h)
            usb_rx_task();

        // a transfer a reset swallowed does not hold this up: the set
        // configuration that follows clears TxState with the class data
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
    } else if (!cdc_dtr) {
        // a closed port: what the host wrote after closing, or echoed before
        // its echo was off, and the half frame the parser is in would all go
        // to the next program to open the port. Not during the hold-off
        // after dtr though, the request sent right after opening waits there
        cdc_rx_drop();
        cdc_rx_resume();
    }

    if (cdc_rate == 0xcdcd) {
        comm_service_poll();
    } else {
        cd_frame_t *frame;
        while ((frame = cd_list_get(&d_dev.rx_head)) != NULL)
            cdctl_send_frame(&r_dev.cd_dev, frame);
        while ((frame = cdctl_recv_frame(&r_dev.cd_dev)) != NULL)
            frame_cache_put(&d_dev.tx_head, frame);
    }
}


void app_main(void)
{
    // canary at the bottom of the msp stack area, right above the debug
    // ring that takes the lowest part of the reservation; both are past the
    // heap limit enforced by _sbrk, so malloc can never touch them
    volatile uint64_t *stack_check =
            (uint64_t *)((uint32_t)&_estack - (uint32_t)&_Min_Stack_Size
                    + (uint32_t)&_Noinit_Size);
    uint32_t cdc_rate_bk = 0;
    uint32_t t_update_baud = 0;

    dbg_uart_init(); // before the first print
    for (int i = 0; i < CDC_RX_MAX; i++)
        list_put(&cdc_rx_free_head, &cdc_rx_alloc[i].node);
    cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_rx_buf_t);

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

    load_conf();
    csa.bus_cfg.baud_l = csa.bus_cfg.baud_h = 115200;
    cduart_dev_init(&d_dev, &frame_free_head);
    d_dev.local_mac = 0xff;

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

    // the cdctl-b of the hw v4 is an fpga module that comes up poorly without
    // a reset pulse; the pin floats on the hw v5, where it drives nothing
    gpio_set_val(&r_rst, 0);
    delay_systick(2);
    gpio_set_val(&r_rst, 1);
    delay_systick(2);

    spi_wr_init(&r_spi);
    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, &r_int);
    comm_service_init(); // after cdctl_dev_init: the info string names the hw by the chip

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
            dbg_uart_flush();
            NVIC_SystemReset();
        }

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            dbg_uart_flush();
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
            // nothing pends PendSV from here on, and a pend from before has
            // been served already: on the m0+ it preempts the thread at once
            pendsv_val = 0; // disable pendsv
            __COMPILER_BARRIER();
            // under the mask: PendSV reads the port again once cdctl_baud_h
            // matches, and a bus frame could pend it before the mask otherwise
            cdctl_baud_l = baud_l;
            cdctl_baud_h = baud_h;
            cdctl_set_clk(&r_dev, cdctl_baud_h);
            cdctl_set_baud_rate(&r_dev, cdctl_baud_l, cdctl_baud_h);
            cdctl_flush(&r_dev);
            cdctl_get_baud_rate(&r_dev, &csa.bus_cfg.baud_l, &csa.bus_cfg.baud_h);
            __COMPILER_BARRIER();
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
