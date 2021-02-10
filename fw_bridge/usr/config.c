/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#include "app_main.h"

regr_t regr_wa[] = {
        { .offset = offsetof(csa_t, magic_code), .size = offsetof(csa_t, _end) - offsetof(csa_t, magic_code) }
};

int regr_wa_num = sizeof(regr_wa) / sizeof(regr_t);


const csa_t csa_dft = {
        .magic_code = 0xcdcd,
        .conf_ver = APP_CONF_VER,
        .bus_cfg = CDCTL_CFG_DFT(0x00),
        .dbg_en = false,
        .dbg_dst = { .addr = {0x80, 0x00, 0xaa}, .port = 9 },

        .is_rs232 = false,
        .ttl_baudrate = 115200,
        .rs232_baudrate = 115200,
};

csa_t csa;


void load_conf(void)
{
    csa_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, offsetof(csa_t, _end));
    memset(&app_tmp.conf_from, 0, 4);

    if (app_tmp.magic_code == 0xcdcd && app_tmp.conf_ver == APP_CONF_VER) {
        memcpy(&csa, &app_tmp, offsetof(csa_t, _end));
        csa.conf_from = 1;
    } else {
        csa = csa_dft;
    }
}

int save_conf(void)
{
    uint8_t ret;
    uint32_t err_page = 0;
    FLASH_EraseInitTypeDef f;

    f.TypeErase = FLASH_TYPEERASE_PAGES;
    f.PageAddress = APP_CONF_ADDR;
    f.NbPages = 1;

    ret = HAL_FLASH_Unlock();
    if (ret == HAL_OK)
        ret = HAL_FLASHEx_Erase(&f, &err_page);

    if (ret != HAL_OK)
        printf("conf: failed to erase flash\n");

    uint32_t *dst_dat = (uint32_t *)APP_CONF_ADDR;
    uint32_t *src_dat = (uint32_t *)&csa;
    int cnt = (offsetof(csa_t, _end) + 3) / 4;

    for (int i = 0; ret == HAL_OK && i < cnt; i++) {
        ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)(dst_dat + i), *(src_dat + i));
        printf("w: csa @ %08lx, addr %08lx <- val: %08lx\n",
                (uint32_t)(src_dat + i), (uint32_t)(dst_dat + i), *(src_dat + i));
    }

    ret |= HAL_FLASH_Lock();

    if (ret == HAL_OK) {
        printf("conf: save to flash successed\n");
        return 0;
    } else {
        printf("conf: save to flash error\n");
        return 1;
    }
}


#define t_name(expr)  \
        (_Generic((expr), \
                int8_t: "b", uint8_t: "B", \
                int16_t: "h", uint16_t: "H", \
                int32_t: "i", uint32_t: "I", \
                int: "i", \
                bool: "b", \
                float: "f", \
                char *: "[c]", \
                uint8_t *: "[B]", \
                regr_t *: "{H,H}", \
                default: "-"))


#define CSA_SHOW(_p, _x, _desc) \
        d_debug("  [ 0x%04x, %d, \"%s\", " #_p ", \"" #_x "\", \"%s\" ],\n", \
                offsetof(csa_t, _x), sizeof(csa._x), t_name(csa._x), _desc);

#define CSA_SHOW_SUB(_p, _x, _y_t, _y, _desc) \
        d_debug("  [ 0x%04x, %d, \"%s\", " #_p ", \"" #_x "_" #_y "\", \"%s\" ],\n", \
                offsetof(csa_t, _x) + offsetof(_y_t, _y), sizeof(csa._x._y), t_name(csa._x._y), _desc);

void csa_list_show(void)
{
    d_debug("csa_list_show:\n");
    d_debug("\n"); debug_flush(true);

    CSA_SHOW(1, magic_code, "Magic code: 0xcdcd");
    CSA_SHOW(1, conf_ver, "Config version");
    CSA_SHOW(0, conf_from, "0: default config, 1: load from flash");
    CSA_SHOW(0, do_reboot, "Write 1 to reboot");
    CSA_SHOW(0, save_conf, "Write 1 to save current config to flash");
    d_debug("\n"); debug_flush(true);

    CSA_SHOW_SUB(1, bus_cfg, cdctl_cfg_t, mac, "RS-485 port id, range: 0~254");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, baud_l, "RS-485 baud rate for first byte");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, baud_h, "RS-485 baud rate for follow bytes");
    CSA_SHOW_SUB(1, bus_cfg, cdctl_cfg_t, filter, "Multicast address");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, mode, "0: Arbitration, 1: Break Sync");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, tx_permit_len, "Allow send wait time");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, max_idle_len, "Max idle wait time for BS mode");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, tx_pre_len, " Active TX_EN before TX");
    d_debug("\n"); debug_flush(true);

    CSA_SHOW(0, dbg_en, "1: Report debug message to host, 0: do not report");
    CSA_SHOW_SUB(2, dbg_dst, cdn_sockaddr_t, addr, "Send debug message to this address");
    CSA_SHOW_SUB(1, dbg_dst, cdn_sockaddr_t, port, "Send debug message to this port");
    d_debug("\n"); debug_flush(true);

    CSA_SHOW(0, is_rs232, "0: TTL, 1: RS-232");
    CSA_SHOW(0, ttl_baudrate, "TTL baudrate");
    CSA_SHOW(0, rs232_baudrate, "RS-232 baudrate");
    d_debug("\n"); debug_flush(true);
}
