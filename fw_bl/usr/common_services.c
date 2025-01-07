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


void send_frame(cd_frame_t *frame)
{
    frame->dat[1] = frame->dat[0];
    frame->dat[0] = 0xfe;
    list_put_it(&d_dev.tx_head, &frame->node);
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
    sprintf(info_str, "M: cdbus bridge (bl); S: %s; SW: %s", cpu_id, SW_VER);
    d_info("info: %s\n", info_str);
}


// device info
static void p1_hander(cd_frame_t *frame)
{
    uint8_t subs = frame->dat[4];
    uint8_t frame_len = frame->dat[2];

    if (subs == 0 && frame_len == 2) {
        frame->dat[3] = 0x40;
        strcpy((char *)frame->dat + 4, info_str);
        frame->dat[2] = strlen(info_str) + 1;
        send_frame(frame);

    } else {
        list_put_it(&frame_free_head, &frame->node);
    }
}

// flash memory manipulation
static void p8_hander(cd_frame_t *frame)
{
    uint8_t subs = frame->dat[4];
    uint8_t frame_len = frame->dat[2];

    if (subs == 0x2f && frame_len == 10) {
        uint32_t addr = get_unaligned32(frame->dat + 5);
        uint32_t len = get_unaligned32(frame->dat + 9);
        uint8_t ret = flash_erase(addr, len);
        frame->dat[3] = ret ? 0x41 : 0x40;
        frame->dat[2] = 1;
        send_frame(frame);

    } else if (subs == 0x00 && frame_len == 7) {
        uint32_t addr = get_unaligned32(frame->dat + 5);
        uint8_t *dst_addr = (uint8_t *) addr;
        uint8_t len = min(frame->dat[9], CDN_MAX_DAT - 1);
        memcpy(frame->dat + 4, dst_addr, len);
        frame->dat[3] = 0x40;
        frame->dat[2] = len + 1;
        send_frame(frame);

    } else if (subs == 0x20 && frame_len > 9) {
        uint32_t addr = get_unaligned32(frame->dat + 5);
        uint8_t len = frame->dat[2] - 6;
        uint8_t ret = flash_write(addr, len, frame->dat + 9);
        frame->dat[3] = ret ? 0x41 : 0x40;
        frame->dat[2] = 1;
        send_frame(frame);

    } else {
        list_put_it(&frame_free_head, &frame->node);
    }
}

// csa manipulation
static void p5_hander(cd_frame_t *frame)
{
    uint32_t flags;
    uint8_t subs = frame->dat[4];
    uint8_t frame_len = frame->dat[2];

    if (subs == 0x00 && frame_len == 5) {
        uint16_t offset = get_unaligned16(frame->dat + 5);
        uint8_t len = min(frame->dat[7], CDN_MAX_DAT - 1);
        local_irq_save(flags);
        memcpy(frame->dat + 4, ((void *) &csa) + offset, len);
        local_irq_restore(flags);
        frame->dat[3] = 0x40;
        frame->dat[2] = len + 1;
        send_frame(frame);

    } else if (subs == 0x20 && frame_len > 4) {
        uint16_t offset = get_unaligned16(frame->dat + 5);
        uint8_t len = frame_len - 4;
        uint8_t *src_addr = frame->dat + 7;
        uint16_t start = clip(offset, 0, sizeof(csa_t));
        uint16_t end = clip(offset + len, 0, sizeof(csa_t));
        local_irq_save(flags);
        memcpy(((void *) &csa) + start, src_addr + (start - offset), end - start);
        local_irq_restore(flags);
        frame->dat[3] = 0x40;
        frame->dat[2] = 1;
        send_frame(frame);

    } else if (subs == 0x01 && frame_len == 5) {
        uint16_t offset = get_unaligned16(frame->dat + 5);
        uint8_t len = min(frame->dat[7], CDN_MAX_DAT - 1);
        memcpy(frame->dat + 4, ((void *) &csa_dft) + offset, len);
        frame->dat[3] = 0x40;
        frame->dat[2] = len + 1;
        send_frame(frame);

    } else {
        list_put_it(&frame_free_head, &frame->node);
    }
}


static inline void serial_cmd_dispatch(void)
{
    cd_frame_t *frame = list_get_entry_it(&d_dev.rx_head, cd_frame_t);

    if (frame) {
        uint8_t server_num = frame->dat[3];

        switch (server_num) {
        case 1: p1_hander(frame); break;
        case 5: p5_hander(frame); break;
        case 8: p8_hander(frame); break;
        default:
            printf("cmd err\n");
            list_put_it(&frame_free_head, &frame->node);
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
    if (csa.do_reboot)
        NVIC_SystemReset();
    serial_cmd_dispatch();
}


// for printf
int _write(int file, char *data, int len)
{
   if (csa.dbg_en) {
       cd_frame_t *frm = list_get_entry_it(&frame_free_head, cd_frame_t);
       if (frm) {
           len = min(CDN_MAX_DAT - 2, len);
           frm->dat[0] = 0xfe;
           frm->dat[1] = 0x0;
           frm->dat[2] = 2 + len;
           frm->dat[3] = 0x9;
           frm->dat[4] = 0x40;
           memcpy(frm->dat + 5, data, len);
           list_put_it(&d_dev.tx_head, &frm->node);
           //return len;
       }
   }

   dbg_transmit((uint8_t *)data, len);
   return len;
}
