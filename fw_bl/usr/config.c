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

    if (app_tmp.magic_code == 0xcdcd && (app_tmp.conf_ver & 0xff00) == (APP_CONF_VER & 0xff00)) {
        memcpy(&csa, &app_tmp, offsetof(csa_t, _end));
        csa.conf_from = 1;
    } else {
        csa = csa_dft;
    }
}
