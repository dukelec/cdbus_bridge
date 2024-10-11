/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi1;
extern USBD_HandleTypeDef hUsbDeviceFS;

int CDCTL_SYS_CLK = 150000000; // 150MHz for cdctl01a

static  gpio_t led_r = { .group = RGB_R_GPIO_Port, .num = RGB_R_Pin };
static  gpio_t led_g = { .group = RGB_G_GPIO_Port, .num = RGB_G_Pin };
static  gpio_t led_b = { .group = RGB_B_GPIO_Port, .num = RGB_B_Pin };
static  gpio_t led_tx = { .group = LED_Y_GPIO_Port, .num = LED_Y_Pin };
static  gpio_t led_rx = { .group = LED_B_GPIO_Port, .num = LED_B_Pin };
static  gpio_t sw1 = { .group = SW1_GPIO_Port, .num = SW1_Pin };
static  gpio_t sw2 = { .group = SW2_GPIO_Port, .num = SW2_Pin };

uart_t debug_uart = { .huart = &huart5 };
uart_t ttl_uart = { .huart = &huart3 };

static gpio_t r_rst = { .group = OLD_CD_RST_GPIO_Port, .num = OLD_CD_RST_Pin };
static gpio_t r_int = { .group = CD_INT_GPIO_Port, .num = CD_INT_Pin };
static gpio_t r_ss = { .group = CD_SS_GPIO_Port, .num = CD_SS_Pin };
static spi_t r_spi = { .hspi = &hspi1, .ns_pin = &r_ss };

#define CDC_RX_MAX  240
#define CDC_TX_MAX   80
static cdc_rx_buf_t cdc_rx_alloc[CDC_RX_MAX];
static cdc_tx_buf_t cdc_tx_alloc[CDC_TX_MAX];
list_head_t cdc_rx_free_head = {0};
list_head_t cdc_tx_free_head = {0};
list_head_t cdc_rx_head = {0};
list_head_t cdc_tx_head = {0};
cdc_rx_buf_t *cdc_rx_buf = NULL;
cdc_tx_buf_t *cdc_tx_buf = NULL;

static cd_frame_t frame_alloc[FRAME_MAX];
list_head_t frame_free_head = {0};

static cdn_pkt_t packet_alloc[PACKET_MAX];
list_head_t packet_free_head = {0};

cdctl_dev_t r_dev = {0};    // 485
cduart_dev_t c_dev = {0};   // usb / config mode
cduart_dev_t d_dev = {0};   // usb / data mode
cdn_ns_t dft_ns = {0};      // CDNET

int usb_rx_cnt = 0;
int usb_tx_cnt = 0;
static bool cdc_need_flush = false;
static uint32_t flush_set_time = 0;

static int cdc_rate_bk = 0;
int cdc_rate = 0;
int app_mode = 0; // 0: data, 1: config

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

    cdc_rx_buf = list_get_entry(&cdc_rx_free_head, cdc_rx_buf_t);

    csa.force_115200 = !gpio_get_value(&sw2);
    if (csa.force_115200) {
        csa.bus_cfg.baud_l = 115200;
        csa.bus_cfg.baud_h = 115200;
        printf("force baudrate to 115200 by sw2!\n");
    }
    cdctl_dev_init(&r_dev, &frame_free_head, &csa.bus_cfg, &r_spi, &r_rst);

    if (r_dev.version >= 0x10) {
        // 24MHz / (2 + 2) * (48 + 2) / 2^1 = 150MHz
        cdctl_write_reg(&r_dev, REG_PLL_N, 0x2);
        d_info("pll_n: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_N));
        cdctl_write_reg(&r_dev, REG_PLL_ML, 0x30); // 0x30: 48
        d_info("pll_ml: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_ML));

        d_info("pll_ctrl: %02x\n", cdctl_read_reg(&r_dev, REG_PLL_CTRL));
        cdctl_write_reg(&r_dev, REG_PLL_CTRL, 0x10); // enable pll
        d_info("clk_status: %02x\n", cdctl_read_reg(&r_dev, REG_CLK_STATUS));
        cdctl_write_reg(&r_dev, REG_CLK_CTRL, 0x01); // select pll

        d_info("clk_status after select pll: %02x\n", cdctl_read_reg(&r_dev, REG_CLK_STATUS));
        d_info("version after select pll: %02x\n", cdctl_read_reg(&r_dev, REG_VERSION));
    }

    cduart_dev_init(&d_dev, &frame_free_head);
    cduart_dev_init(&c_dev, &frame_free_head);
    //                    uart / usb
    cdn_add_intf(&dft_ns, &c_dev.cd_dev, 0, 0xfe);

    // enable interrupt
    cdctl_write_reg(&r_dev, REG_INT_MASK, CDCTL_MASK);
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
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
        gpio_set_value(&led_rx, 1);
    }
    if (tx_cnt_last != r_dev.tx_cnt) {
        tx_cnt_last = r_dev.tx_cnt;
        tx_t_last = get_systick();
        gpio_set_value(&led_tx, 1);
    }

    if (gpio_get_value(&led_rx) == 1 && get_systick() - rx_t_last > 10)
        gpio_set_value(&led_rx, 0);
    if (gpio_get_value(&led_tx) == 1 && get_systick() - tx_t_last > 10)
        gpio_set_value(&led_tx, 0);
}


static void dump_hw_status(void)
{
    static int t_l = 0;
    if (get_systick() - t_l > 8000) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
        t_l = get_systick();

        d_debug("ctl: t_len %d, r_len %d, irq %d\n",
                r_dev.tx_head.len, r_dev.rx_head.len, !gpio_get_value(&r_int));
        d_debug("  r_cnt %d (lost %d, err %d, no-free %d), t_cnt %d (cd %d, err %d)\n",
                r_dev.rx_cnt, r_dev.rx_lost_cnt, r_dev.rx_error_cnt,
                r_dev.rx_no_free_node_cnt,
                r_dev.tx_cnt, r_dev.tx_cd_cnt, r_dev.tx_error_cnt);
        d_debug("usb: r_cnt %d, t_cnt %d, t_buf %p, t_len %d, t_state %x\n",
                usb_rx_cnt, usb_tx_cnt, cdc_tx_buf, cdc_tx_head.len, hcdc->TxState);
    }
}


void app_main(void)
{
    uint64_t *stack_check = (uint64_t *)((uint32_t)&end + 256);
    USBD_CDC_HandleTypeDef *hcdc = NULL;
    
    gpio_set_value(&led_tx, 1);
    gpio_set_value(&led_rx, 1);
    delay_systick(1);
    gpio_set_value(&led_tx, 0);
    gpio_set_value(&led_rx, 0);

    printf("\nstart app_main (bridge)...\n");
    *stack_check = 0xababcdcd12123434;
    load_conf();
    debug_init(&dft_ns, &csa.dbg_dst, &csa.dbg_en);
    device_init();
    common_service_init();
    printf("conf: %s\n", csa.conf_from ? "load from flash" : "use default");
    d_info("conf: %s\n", csa.conf_from ? "load from flash" : "use default");
    ttl_uart.huart->Init.BaudRate = csa.ttl_baudrate;
    UART_SetConfig(ttl_uart.huart);
    HAL_UART_Receive_DMA(ttl_uart.huart, circ_buf, CIRC_BUF_SZ);
    csa_list_show();
    delay_systick(100);
    gpio_set_value(&led_r, 0);
    delay_systick(200);
    gpio_set_value(&led_r, 1);
    gpio_set_value(&led_b, 0);
    delay_systick(200);
    gpio_set_value(&led_b, 1);
    gpio_set_value(&led_g, 0);

    while (true) {
        if (csa.usb_online) {
            if (cdc_tx_buf && hcdc->TxState == 0) {
                list_put(&cdc_tx_free_head, &cdc_tx_buf->node);
                cdc_tx_buf = NULL;
            }

            if (cdc_tx_head.first) {
                if (!cdc_tx_buf && hcdc->TxState == 0) {
                    cdc_tx_buf_t *bf = list_entry(cdc_tx_head.first, cdc_tx_buf_t);
                    if (bf->len != 0) {
                        local_irq_disable();
                        CDC_Transmit_FS(bf->dat, bf->len);
                        local_irq_enable();
                        cdc_need_flush = true;
                        flush_set_time = get_systick();

                        list_get(&cdc_tx_head);
                        cdc_tx_buf = bf;

                    } else if (cdc_need_flush && get_systick() - flush_set_time > 5000) {
                        local_irq_disable();
                        CDC_Transmit_FS(NULL, 0);
                        local_irq_enable();
                        cdc_need_flush = false;
                    }
                }
            }
        } else {
            if (cdc_tx_buf) {
                if (ttl_uart.huart->TxXferCount == 0) {
                    ttl_uart.huart->gState = HAL_UART_STATE_READY;
                    list_put(&cdc_tx_free_head, &cdc_tx_buf->node);
                    cdc_tx_buf = NULL;
                    //d_verbose("ttl_uart dma done.\n");
                }
            } else if (cdc_tx_head.first) {
                cdc_tx_buf_t *bf = list_entry(cdc_tx_head.first, cdc_tx_buf_t);
                if (bf->len != 0) {
                    //d_verbose("ttl_uart dma tx...\n");
                    HAL_UART_Transmit_DMA(ttl_uart.huart, bf->dat, bf->len);
                    list_get(&cdc_tx_head);
                    cdc_tx_buf = bf;
                }
            }

            if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
                printf("usb connected\n");
                csa.usb_online = true;
                hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
            }
        }

        data_led_task();
        dump_hw_status();
        cdctl_routine();
        cdn_routine(&dft_ns); // handle cdnet
        common_service_routine();
        app_bridge();
        debug_flush(false);

        if (csa.force_115200 != !gpio_get_value(&sw2)) {
            printf("sw2 changed, reboot...\n");
            csa.do_reboot = true;
        }
        if (!gpio_get_value(&sw1)) {
            printf("sw1 switch on, reboot...\n");
            csa.do_reboot = true;
        }
        if (cdc_rate != cdc_rate_bk) {
            app_mode = (cdc_rate == 0xcdcd);
            d_info("rate: %ld, mode: %s!\n", cdc_rate, app_mode ? "config" : "data");
            cdc_rate_bk = cdc_rate;
        }

        if (*stack_check != 0xababcdcd12123434) {
            printf("stack overflow\n");
            while (true);
        }
    }
}

