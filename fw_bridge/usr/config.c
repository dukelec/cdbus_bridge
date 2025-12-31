/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

#define ONCE_PAGE_SIZE  2048


const csa_t csa_dft = {
        .magic_code = 0xcdcd,
        .conf_ver = APP_CONF_VER,
        .dbg_en = false,
        .bus_cfg = CDCTL_CFG_DFT(0x00),
        .limit_baudrate0 = 1000000,
        .limit_baudrate1 = 2000000
};

csa_t csa;


void load_conf(void)
{
    uint16_t magic_code = *(uint16_t *)APP_CONF_ADDR;
    uint16_t conf_ver = *(uint16_t *)(APP_CONF_ADDR + 2);
    csa = csa_dft;

    if (magic_code == 0xcdcd && conf_ver == APP_CONF_VER) {
        memcpy(&csa, (void *)APP_CONF_ADDR, offsetof(csa_t, _end_save));
        csa.conf_from = 1;
    } else if (magic_code == 0xcdcd && (conf_ver >> 8) == (APP_CONF_VER >> 8)) {
        memcpy(&csa, (void *)APP_CONF_ADDR, offsetof(csa_t, _end_common));
        csa.conf_from = 2;
        csa.conf_ver = APP_CONF_VER;
    }
    if (csa.conf_from)
        memset(&csa.do_reboot, 0, 3);
}

int save_conf(void)
{
    int ret = flash_erase(APP_CONF_ADDR, 2048);
    if (ret)
        d_info("conf: failed to erase flash\n");
    ret = flash_write(APP_CONF_ADDR, offsetof(csa_t, _end_save), (uint8_t *)&csa);

    if (!ret) {
        d_info("conf: save to flash successed, size: %d\n", offsetof(csa_t, _end_save));
        return 0;
    } else {
        d_error("conf: save to flash error\n");
        return 1;
    }
}

int flash_erase(uint32_t addr, uint32_t len)
{
    int ret = 0;
    uint32_t ofs = addr & ~0x08000000;
    if (ofs <= 0x6000 && 0x6000 < ofs + len) {
        d_error("flash erase: avoid erasing self\n");
        return ret;
    }

    uint32_t s_page = ofs / ONCE_PAGE_SIZE;
    int n_page = (ofs + len) / ONCE_PAGE_SIZE - s_page;
    if ((ofs + len) % ONCE_PAGE_SIZE)
        n_page++;

    flash_unlock();
    flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);
    for (int i = 0; i < n_page; i++) {
        flash_status_type st = flash_sector_erase(0x08000000 + ONCE_PAGE_SIZE * (s_page + i));
        if (st != FLASH_OPERATE_DONE) {
            ret = -1;
            break;
        }
    }
    flash_lock();
    d_debug("flash erase: %08lx +%08lx (%ld %d), ret: %d\n", addr, len, s_page, n_page, ret);
    return ret;
}

int flash_write(uint32_t addr, uint32_t len, const uint8_t *buf)
{
    int ret = 0;
    uint32_t write_cnt = (len + 3) / 4;
    uint32_t *src_addr = (uint32_t *) buf;
    uint32_t src_data;

    flash_unlock();
    flash_flag_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);

    for (uint32_t i = 0; i < write_cnt; i++) {
        src_data = get_unaligned32((uint8_t *)(src_addr + i));
        flash_status_type st = flash_word_program(addr + i * 4, src_data);
        if (st != FLASH_OPERATE_DONE) {
            ret = -1;
            break;
        }
    }
    flash_lock();
    d_verbose("flash write: %08lx %ld, ret: %d\n", addr, len, ret);
    return ret;
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
                regr_t: "H,H", \
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
    d_debug("\n");

    CSA_SHOW(1, magic_code, "Magic code: 0xcdcd");
    CSA_SHOW(1, conf_ver, "Config version");
    CSA_SHOW(1, conf_from, "0: default config, 1: all from flash, 2: partly from flash");
    CSA_SHOW(0, do_reboot, "1: reboot to bl, 2: reboot to app");
    CSA_SHOW(0, save_conf, "Write 1 to save current config to flash");
    d_debug("\n");

    CSA_SHOW(0, dbg_en, "1: Report debug message to host, 0: do not report");
    d_debug("\n");

    CSA_SHOW_SUB(1, bus_cfg, cdctl_cfg_t, mac, "RS-485 port id, range: 0~254");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, baud_l, "RS-485 baud rate for first byte");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, baud_h, "RS-485 baud rate for follow bytes");
    CSA_SHOW_SUB(1, bus_cfg, cdctl_cfg_t, filter_m, "Multicast address");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, mode, "0: Traditional, 1: Arbitration, 2: Break Sync");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, tx_permit_len, "Allow send wait time");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, max_idle_len, "Max idle wait time for BS mode");
    CSA_SHOW_SUB(0, bus_cfg, cdctl_cfg_t, tx_pre_len, " Active TX_EN before TX");
    d_debug("\n");

    CSA_SHOW(0, limit_baudrate0, "Low baudrate limit (sw1 off)");
    CSA_SHOW(0, limit_baudrate1, "Low baudrate limit (sw1 on)");
    d_debug("\n");
}
