/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "cd_debug.h"
#include "cdbus_uart.h"
#include "modbus_crc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define BL_ARGS             0x20000000 // first word
#define APP_CONF_ADDR       0x0801F800 // last page
#define APP_CONF_VER        0x0200

#define FRAME_MAX           80

typedef enum {
    LED_POWERON = 0,
    LED_WARN,
    LED_ERROR
} led_state_t;

typedef struct {
    list_node_t node;
    uint16_t    len;
    uint8_t     dat[64]; // CDC_DATA_FS_MAX_PACKET_SIZE
} cdc_rx_buf_t;

typedef struct {
    uint16_t        offset;
    uint16_t        size;
} regr_t; // reg range


typedef struct {
    uint16_t        magic_code; // 0xcdcd
    uint16_t        conf_ver;
    uint8_t         conf_from;  // 0: default, 1: load from flash
    uint8_t         do_reboot;
    bool            _reserved;
    bool            save_conf;

    bool            dbg_en;

    uint8_t         _keep[503]; // covers the areas in the app csa that need to be saved

    // end of flash
    #define         _end_save usb_online // offset: 512

    bool            usb_online;

} csa_t; // config status area

extern csa_t csa;
extern const csa_t csa_dft;


int flash_erase(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf);

extern UART_HandleTypeDef huart5;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern gpio_t led_g;
extern gpio_t sw1;

extern list_head_t cdc_rx_free_head;
extern list_head_t cdc_rx_head;
extern cdc_rx_buf_t * volatile cdc_rx_buf;
extern volatile int usb_rx_cnt;
extern volatile int usb_tx_cnt;

extern list_head_t frame_free_head;
extern cduart_dev_t d_dev;  // uart / usb
extern volatile uint8_t cdc_dtr;

extern uint32_t end; // end of bss
extern uint32_t *bl_args;

void common_service_init(void);
void common_service_routine(void);

void app_main(void);
void load_conf(void);
int save_conf(void);
void try_jump_to_app(void);

#endif
