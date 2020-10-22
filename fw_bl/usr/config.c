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


csa_t csa = {
        .magic_code = 0xcdcd,
        .conf_ver = APP_CONF_VER,

        .bus_mac = 0,
        .bus_baud_low = 115200,
        .bus_baud_high = 115200,
        .dbg_en = true,
        .dbg_dst = { .addr = {0x80, 0x00, 0x00}, .port = 9 },

        .ser_idx = SER_TTL,
        .ttl_baudrate = 115200,
        .rs232_baudrate = 115200,
};


void load_conf(void)
{
    csa_t app_tmp;
    memcpy(&app_tmp, (void *)APP_CONF_ADDR, offsetof(csa_t, _end));
    memset(&app_tmp.conf_from, 0, 4);

    if (app_tmp.magic_code == 0xcdcd && app_tmp.conf_ver == APP_CONF_VER) {
        memcpy(&csa, &app_tmp, offsetof(csa_t, _end));
        csa.conf_from = 1;
    }
}


#define CSA_SHOW(_x) \
        printf("   R_" #_x " = 0x%04x # len: %d\n", offsetof(csa_t, _x), sizeof(csa._x));

#define CSA_SHOW_SUB(_x, _y_t, _y) \
        printf("   R_" #_x "_" #_y " = 0x%04x # len: %d\n", offsetof(csa_t, _x) + offsetof(_y_t, _y), sizeof(csa._x._y));

void csa_list_show(void)
{
    printf("csa_list_show:\n\n");

    CSA_SHOW(conf_ver);
    CSA_SHOW(conf_from);
    CSA_SHOW(do_reboot);
    CSA_SHOW(keep_in_bl);
    printf("\n");

    CSA_SHOW(bus_mac);
    CSA_SHOW(bus_baud_low);
    CSA_SHOW(bus_baud_high);
    CSA_SHOW(dbg_en);
    CSA_SHOW_SUB(dbg_dst, cdn_sockaddr_t, addr);
    CSA_SHOW_SUB(dbg_dst, cdn_sockaddr_t, port);
    printf("\n");

    CSA_SHOW(ser_idx);
    CSA_SHOW(ttl_baudrate);
    CSA_SHOW(rs232_baudrate);
    printf("\n");

    CSA_SHOW(sw_val);
    printf("\n");
}
