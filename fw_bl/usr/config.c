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
        .dbg_en = false
};

csa_t csa;


void load_conf(void)
{
    uint16_t magic_code = *(uint16_t *)APP_CONF_ADDR;
    uint16_t conf_ver = *(uint16_t *)(APP_CONF_ADDR + 2);
    csa = csa_dft;

    //d_info("end_save: %d\n", offsetof(csa_t, _end_save)); // 512
    if (magic_code == 0xcdcd && (conf_ver >> 8) == (APP_CONF_VER >> 8)) {
        memcpy(&csa, (void *)APP_CONF_ADDR, offsetof(csa_t, _end_save));
        csa.conf_from = 1;
        memset(&csa.do_reboot, 0, 3);
    }
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
