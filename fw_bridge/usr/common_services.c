/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

static char cpu_id[25];
static char info_str[100];
static cd_spinlock_t p5_lock = {0};


static void send_frame(cd_frame_t *frame, uint8_t p_len)
{
    frame->dat[1] = frame->dat[0];
    frame->dat[0] = 0xff;
    frame->dat[2] = p_len + 2;
    swap(frame->dat[3], frame->dat[4]); // swap src and dst port
    cd_list_put(&d_dev.tx_head, frame);
}

static void get_uid(char *buf)
{
    const char tlb[] = "0123456789abcdef";
    int i;

    for (i = 0; i < 12; i++) {
        uint8_t val = *((char *)0x1FFFF7E8 + i);
        buf[i * 2 + 0] = tlb[val >> 4];
        buf[i * 2 + 1] = tlb[val & 0xf];
    }
    buf[24] = '\0';
}

static void init_info_str(void)
{
    // M: model; S: serial string; HW: hardware version; SW: software version
    get_uid(cpu_id);
    sprintf(info_str, "M: cdbus bridge; S: %s; SW: %s", cpu_id, SW_VER);
    d_info("info: %s, git: %s\n", info_str, SW_VER_FULL);
}


// device info
static void p1_handler(cd_frame_t *frame)
{
    uint8_t *p_dat = frame->dat + 5;
    uint8_t p_len = frame->dat[2] - 2;

    if (p_len == 0) {
        strcpy((char *)p_dat, info_str);
        send_frame(frame, strlen(info_str));
    } else {
        cd_list_put(&frame_free_head, frame);
    }
}

// flash memory manipulation
static void p8_handler(cd_frame_t *frame)
{
    uint8_t *p_dat = frame->dat + 5;
    uint8_t p_len = frame->dat[2] - 2;
    bool reply = !(*p_dat & 0x80);
    *p_dat &= 0x7f;

    if (*p_dat == 0x2f && p_len == 9) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint32_t len = get_unaligned32(p_dat + 5);
        uint8_t ret = flash_erase(addr, len);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, 1);

    } else if (*p_dat == 0x00 && p_len == 6) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint8_t *dst_addr = (uint8_t *) addr;
        uint8_t len = min(p_dat[5], CDN_MAX_DAT - 1);
        memcpy(p_dat + 1, dst_addr, len);
        *p_dat = 0;
        if (reply)
            send_frame(frame, len + 1);

    } else if (*p_dat == 0x20 && p_len > 8) {
        uint32_t addr = get_unaligned32(p_dat + 1);
        uint8_t len = p_len - 5;
        uint8_t ret = flash_write(addr, len, p_dat + 5);
        *p_dat = ret ? 1 : 0;
        if (reply)
            send_frame(frame, 1);

    } else {
        cd_list_put(&frame_free_head, frame);
        return;
    }
    if (!reply)
        cd_list_put(&frame_free_head, frame);
}

// csa manipulation
static void p5_handler(cd_frame_t *frame)
{
    uint32_t flags;
    uint8_t *p_dat = frame->dat + 5;
    uint8_t p_len = frame->dat[2] - 2;
    bool reply = !(*p_dat & 0x80);
    *p_dat &= 0x7f;

    if (*p_dat == 0x00 && p_len == 4) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = min(p_dat[3], CDN_MAX_DAT - 1);
        cd_irq_save(&p5_lock, flags);
        memcpy(p_dat + 1, ((void *) &csa) + offset, len);
        cd_irq_restore(&p5_lock, flags);
        *p_dat = 0;
        if (reply)
            send_frame(frame, len + 1);

    } else if (*p_dat == 0x20 && p_len > 3) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = p_len - 3;
        uint8_t *src_addr = p_dat + 3;
        uint16_t start = clip(offset, 0, sizeof(csa_t));
        uint16_t end = clip(offset + len, 0, sizeof(csa_t));
        cd_irq_save(&p5_lock, flags);
        memcpy(((void *) &csa) + start, src_addr + (start - offset), end - start);
        cd_irq_restore(&p5_lock, flags);
        *p_dat = 0;
        if (reply)
            send_frame(frame, 1);

    } else if (*p_dat == 0x01 && p_len == 4) {
        uint16_t offset = get_unaligned16(p_dat + 1);
        uint8_t len = min(p_dat[3], CDN_MAX_DAT - 1);
        memcpy(p_dat + 1, ((void *) &csa_dft) + offset, len);
        *p_dat = 0;
        if (reply)
            send_frame(frame, len + 1);

    } else {
        cd_list_put(&frame_free_head, frame);
        return;
    }
    if (!reply)
        cd_list_put(&frame_free_head, frame);
}


static inline void serial_cmd_dispatch(void)
{
    cd_frame_t *frame = cd_list_get(&d_dev.rx_head);

    if (frame) {
        uint8_t server_num = frame->dat[4];

        switch (server_num) {
        case 1: p1_handler(frame); break;
        case 5: p5_handler(frame); break;
        case 8: p8_handler(frame); break;
        default:
            printf("cmd err\n");
            cd_list_put(&frame_free_head, frame);
        }
    }
}


void common_service_init(void)
{
    init_info_str();
}

void common_service_routine(void)
{
    if (csa.save_conf) {
        csa.save_conf = false;
        save_conf();
    }
    if (csa.do_reboot) {
        *(uint32_t *)BL_ARGS = 0xcdcd0000 | csa.do_reboot;
        NVIC_SystemReset();
    }
    serial_cmd_dispatch();
}


// for printf
int _write(int file, char *data, int len)
{
    if (csa.dbg_en) {
        cd_frame_t *frm = cd_list_get(&frame_free_head);
        if (frm) {
            len = min(CDN_MAX_DAT - 2, len);
            frm->dat[0] = 0xff;
            frm->dat[1] = 0x0;
            frm->dat[2] = 2 + len;
            frm->dat[3] = 64;
            frm->dat[4] = 9;
            memcpy(frm->dat + 5, data, len);
            cd_list_put(&d_dev.tx_head, frm);
            //return len;
        }
    }

    dbg_transmit((uint8_t *)data, len);
    return len;
}
