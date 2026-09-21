/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"
#include "tusb.h"

static gpio_t led_r = { .group = RGB_R_GPIO_PORT, .num = RGB_R_PIN };
static gpio_t led_g = { .group = RGB_G_GPIO_PORT, .num = RGB_G_PIN };
static gpio_t led_b = { .group = RGB_B_GPIO_PORT, .num = RGB_B_PIN };
static gpio_t led_tx = { .group = LED_Y_GPIO_PORT, .num = LED_Y_PIN };
static gpio_t led_rx = { .group = LED_G_GPIO_PORT, .num = LED_G_PIN };
static gpio_t sw1 = { .group = SW1_GPIO_PORT, .num = SW1_PIN };
static gpio_t sw2 = { .group = SW2_GPIO_PORT, .num = SW2_PIN };

static gpio_t r_int = { .group = CD_INT_GPIO_PORT, .num = CD_INT_PIN };
static gpio_t r_cs = { .group = CD_SS_GPIO_PORT, .num = CD_SS_PIN };
static spi_t r_spi = {
        .spi = SPI1,
        .ns_pin = &r_cs,
        .dma_rx = DMA1,
        .dma_ch_rx = DMA1_CHANNEL1,
        .dma_ch_tx = DMA1_CHANNEL2,
        .dma_mask = (2 << 0)
};


static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

cdctl_dev_t r_dev = {0};    // CDBUS

bool hw_raw = false;        // usart1 instead of the cdctl controller
bool raw_mode = false;      // ... and the serial port is not in config mode

static uint32_t cache_drop_cnt = 0;


/*
 * Frames a direction is holding, counted where they actually sit rather
 * than tracked at every alloc and free. Neither may go past its own limit,
 * so the other always has the rest, and a quiet direction lends everything
 * it is not using to the busy one.
 */
uint32_t frame_dir_len(bool to_host)
{
    if (to_host)
        return r_dev.rx_head.len + cdc_queued_to_host() + net_queued();
    return r_dev.tx_head.len + cdc_queued_to_bus();
}

bool frame_dir_ok(bool to_host)
{
    return frame_dir_len(to_host) < FRAME_DIR_MAX;
}

/*
 * Queue a frame for a host. The oldest queued frame goes once this
 * direction is at its share or the pool is down to its reserve, so the
 * newest data wins, which is what a bus adapter should do.
 */
void frame_cache_put(list_head_t *head, cd_frame_t *frame)
{
    while (!frame_dir_ok(true) || frame_free_head.len < FRAME_RESERVE) {
        cd_frame_t *old = cd_list_get(head);
        // whatever else the direction is sitting on, if this queue has
        // nothing of its own left to give: the debug log with the serial
        // port closed, or a host bound queue nobody is draining
        if (!old)
            old = cdc_tx_evict();
        if (!old)
            old = net_tx_evict();
        if (!old)
            break;
        cd_list_put(&frame_free_head, old);
        cache_drop_cnt++;
    }
    cd_list_put(head, frame);
}

// in the raw mode the controller is not driven at all, anything queued
// there would sit forever
bool bus_tx_ready(void)
{
    return !hw_raw && frame_dir_ok(false);
}

// only after bus_tx_ready() said so
void bus_tx(cd_frame_t *frame)
{
    cdctl_send_frame(&r_dev.cd_dev, frame);
}

/*
 * Hand what came off the bus to the port that has it. One at a time: the
 * serial port while it is open, the ethernet port otherwise. Opening the
 * serial port is something someone does on purpose, while the ethernet
 * interface tends to come up on its own, so the serial port wins. Both are
 * the same node on the bus and a frame cannot be attributed to one of them
 * anyway, so handing it to both would only cost a copy and the frames to
 * hold it.
 */
static void bus_rx_dispatch(void)
{
    cd_frame_t *frm;

    while ((frm = cdctl_recv_frame(&r_dev.cd_dev)) != NULL) {
        if (cdc_bus_active())
            cdc_bus_rx(frm);
        else if (net_bus_active())
            net_bus_rx(frm);
        else
            cd_list_put(&frame_free_head, frm);
    }
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
        d_debug("  free %ld, drop %ld, dir h %ld b %ld, usb %d, raw %d\n",
                frame_free_head.len, cache_drop_cnt,
                frame_dir_len(true), frame_dir_len(false),
                csa.usb_online, raw_mode);
        d_debug("  net: bus %ld, pc %ld, loc %ld, drop f %ld b %ld y %ld\n",
                net_cnt.to_bus, net_cnt.to_pc, net_cnt.local,
                net_cnt.drop_fmt, net_cnt.drop_big, net_cnt.drop_busy);
        d_debug("  cdc: bus %ld, pc %ld, loc %ld, drop y %ld, rate %ld\n",
                cdc_cnt.to_bus, cdc_cnt.to_pc, cdc_cnt.local,
                cdc_cnt.drop_busy, cdc_rate);
    }
}

static void usb_state_task(void)
{
    bool online = tud_ready();

    if (online != csa.usb_online) {
        csa.usb_online = online;
        printf("usb: %s\n", online ? "up" : "down");
    }
}

/*
 * The usb clock source defaults to pllu, but its output is not necessarily
 * running yet. Enabling it here matches what the tinyusb board support does
 * and is harmless if it was already on. Bounded: a stuck pll must not keep
 * the firmware from starting, the usb port simply stays dead.
 */
static void usb_clock_init(void)
{
    uint32_t timeout = 100000;

    crm_pllu_output_set(TRUE);
    while (crm_flag_get(CRM_PLLU_STABLE_FLAG) != SET) {
        if (!--timeout) {
            printf("usb: pllu did not lock\n");
            return;
        }
    }
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_PLLU);
}

/*
 * The bus starts at the rate the config holds and follows the rate the
 * serial port is opened with after that, which is how every existing tool
 * sets it. A host that never opens the serial port therefore keeps the
 * configured rate; changing that one means writing bus_cfg_baud_h, saving
 * and power cycling.
 */
static void bus_baud_task(void)
{
    static uint32_t req_l = 0, req_h = 0;
    static uint32_t t_update = 0;
    uint32_t baud_l, baud_h, limit;

    limit = !gpio_get_val(&sw2) ? csa.limit_baudrate1 : csa.limit_baudrate0;
    baud_h = cdc_rate_final;
    baud_l = csa.bus_cfg.mode == 1 ? min(baud_h, limit) : baud_h;

    if (baud_l != req_l || baud_h != req_h) {
        gpio_set_val(&led_g, 1);
        gpio_set_val(&led_b, 0);
        t_update = get_systick();
        req_l = baud_l;
        req_h = baud_h;

        if (!hw_raw) {
            cdctl_set_clk(&r_dev, baud_h);
            cdctl_set_baud_rate(&r_dev, baud_l, baud_h);
            cdctl_flush(&r_dev);
            // the controller reports what it could actually reach
            cdctl_get_baud_rate(&r_dev, &csa.bus_cfg.baud_l, &csa.bus_cfg.baud_h);
        } else {
            crm_clocks_freq_type clocks_freq;
            crm_clocks_freq_get(&clocks_freq);
            usart_enable(UART_DEV, false);
            USART1->baudr = max(DIV_ROUND_CLOSEST(clocks_freq.apb2_freq, baud_h), 16);
            csa.bus_cfg.baud_l = csa.bus_cfg.baud_h =
                    DIV_ROUND_CLOSEST(clocks_freq.apb2_freq, USART1->baudr);
            usart_enable(UART_DEV, true);
        }
        d_debug("baud rate: %lu %lu\n", csa.bus_cfg.baud_l, csa.bus_cfg.baud_h);
    }

    if (gpio_get_val(&led_g) && get_systick() - t_update > 100) {
        gpio_set_val(&led_b, 1);
        gpio_set_val(&led_g, 0);
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

    // the generated nvic config enables this long before there is a stack to
    // handle it, and the led sequence below takes half a second. tinyusb
    // turns it back on from tusb_rhport_init().
    NVIC_DisableIRQ(OTGHS_IRQn);

    dbg_uart_init(); // before the first print
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
    hw_raw = csa.bus_cfg.mode >= 4;
    cdc_init();
    comm_service_init();

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
    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, &r_int, EXINT0_IRQn);
    cdctl_get_baud_rate(&r_dev, &csa.bus_cfg.baud_l, &csa.bus_cfg.baud_h);

    if (!hw_raw) {
        nvic_irq_enable(EXINT0_IRQn, 2, 0);
        nvic_irq_enable(DMA1_Channel1_IRQn, 2, 0);
        exint_interrupt_enable(EXINT_LINE_0, TRUE);
    } else {
        wk_usart1_init();
        nvic_irq_enable(DMA2_Channel2_IRQn, 2, 0); // uart tx
        nvic_irq_enable(USART1_IRQn, 2, 0);
        uart_dma_wr_init();
        usart_interrupt_enable(UART_DEV, USART_TDC_INT, true);
    }

    net_init();
    usb_clock_init();
    tusb_rhport_init(BOARD_TUD_RHPORT, &(tusb_rhport_init_t){
            .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO });

    while (true) {
        tud_task();
        cdc_poll();
        if (!hw_raw)
            bus_rx_dispatch();
        net_poll();
        comm_service_poll();

        usb_state_task();
        bus_baud_task();
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
    }
}


void cdctl_rx_cb(cdctl_dev_t *dev, cd_frame_t *frame)
{
    // the main loop polls, nothing to wake up
}

void EXINT0_IRQHandler(void)
{
    EXINT->intsts = EXINT_LINE_0;
    cdctl_int_isr(&r_dev);
}

void DMA1_Channel1_IRQHandler(void)
{
    r_spi.dma_rx->clr = r_spi.dma_mask;
    cdctl_spi_isr(&r_dev);
}

void DMA2_Channel2_IRQHandler(void)
{
    uart_dma_isr();
}

void USART1_IRQHandler(void)
{
    uart_tdc_isr();
}
