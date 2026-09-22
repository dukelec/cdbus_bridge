/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

/*
 * The local node: the services that configure this bridge itself.
 *
 * They are reached over udp at <prefix>10:0, the one address in our prefix
 * that is not on the bus, so the usual tools talk to the bridge exactly the
 * way they talk to a device. Each handler gets the request and fills a reply
 * buffer; there is no frame and no queue, net_task calls them straight from
 * the receive path and sends whatever they return.
 */

#include "app_main.h"

static char cpu_id[25];
static char info_str[100];


static void get_uid(char *buf)
{
    const char tlb[] = "0123456789abcdef";
    int i;

    for (i = 0; i < 12; i++) {
        uint8_t val = *((char *)CPU_UID_ADDR + i);
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

/*
 * Only flash, sram and the info block may be handed to memcpy. Reading an
 * address that is not backed by anything hard faults, and the read service
 * takes its address straight from the network.
 */
static bool mem_readable(uint32_t addr, uint32_t len)
{
    uint32_t end = addr + len;

    if (end < addr)
        return false;
    if (addr >= 0x08000000 && end <= 0x08000000 + 256 * 1024)
        return true;
    if (addr >= 0x20000000 && end <= 0x20000000 + 102 * 1024)
        return true;
    if (addr >= 0x1ffff000 && end <= 0x1ffff800)
        return true;
    return false;
}


// device info
static int p1_handler(const uint8_t *req, int req_len, uint8_t *rsp, int rsp_max)
{
    int len;

    if (req_len != 0)
        return -1;
    len = min((int)strlen(info_str), rsp_max);
    memcpy(rsp, info_str, len);
    return len;
}

// csa manipulation
static int p5_handler(const uint8_t *req, int req_len, uint8_t *rsp, int rsp_max)
{
    uint8_t cmd;
    bool reply;

    if (req_len < 1)
        return -1;
    cmd = req[0] & 0x7f;
    reply = !(req[0] & 0x80);

    if ((cmd == 0x00 || cmd == 0x01) && req_len == 4) {
        // read, from the live copy or from the defaults
        const void *base = (cmd == 0x00) ? (const void *)&csa : (const void *)&csa_dft;
        int offset = get_unaligned16(req + 1);
        int len = req[3];

        // a bad offset must read nothing rather than random memory
        if (offset > (int)sizeof(csa_t))
            len = 0;
        else
            len = min(len, (int)sizeof(csa_t) - offset);
        len = min(len, rsp_max - 1);

        rsp[0] = 0;
        memcpy(rsp + 1, base + offset, len);
        return reply ? len + 1 : -1;

    } else if (cmd == 0x20 && req_len > 3) {
        int offset = get_unaligned16(req + 1);
        int len = req_len - 3;
        int start = clip(offset, 0, (int)sizeof(csa_t));
        int end = clip(offset + len, 0, (int)sizeof(csa_t));

        memcpy(((void *)&csa) + start, req + 3 + (start - offset), end - start);
        rsp[0] = 0;
        return reply ? 1 : -1;
    }

    return -1;
}

// flash memory manipulation
static int p8_handler(const uint8_t *req, int req_len, uint8_t *rsp, int rsp_max)
{
    uint8_t cmd;
    bool reply;

    if (req_len < 1)
        return -1;
    cmd = req[0] & 0x7f;
    reply = !(req[0] & 0x80);

    if (cmd == 0x2f && req_len == 9) {
        uint32_t addr = get_unaligned32(req + 1);
        uint32_t len = get_unaligned32(req + 5);
        rsp[0] = flash_erase(addr, len) ? 1 : 0;
        return reply ? 1 : -1;

    } else if (cmd == 0x00 && req_len == 6) {
        uint32_t addr = get_unaligned32(req + 1);
        int len = min((int)req[5], rsp_max - 1);
        if (!mem_readable(addr, len)) {
            rsp[0] = 1;
            return reply ? 1 : -1;
        }
        rsp[0] = 0;
        memcpy(rsp + 1, (const uint8_t *)addr, len);
        return reply ? len + 1 : -1;

    } else if (cmd == 0x20 && req_len > 5) {
        uint32_t addr = get_unaligned32(req + 1);
        int len = req_len - 5;
        rsp[0] = flash_write(addr, len, req + 5) ? 1 : 0;
        return reply ? 1 : -1;
    }

    return -1;
}


int comm_service_handle(uint16_t port, const uint8_t *req, int req_len,
        uint8_t *rsp, int rsp_max)
{
    if (rsp_max < 1)
        return -1;

    switch (port) {
    case 1:  return p1_handler(req, req_len, rsp, rsp_max);
    case 5:  return p5_handler(req, req_len, rsp, rsp_max);
    case 8:  return p8_handler(req, req_len, rsp, rsp_max);
    default: return -1;
    }
}


void comm_service_init(void)
{
    init_info_str();
}

void comm_service_poll(void)
{
    if (csa.save_conf) {
        csa.save_conf = false;
        save_conf();
    }
    if (csa.do_reboot) {
        *(uint32_t *)BL_ARGS = 0xcdcd0000 | csa.do_reboot;
        dbg_uart_flush();
        NVIC_SystemReset();
    }
}


// for printf
int _write(int file, char *data, int len)
{
    // dbg_en bit 0: the serial port, whose queue holds the text until a
    // host opens the port, the boot log included; bit 1: the ethernet port,
    // which only takes what is printed while it is up and being read. The
    // same text goes out on the debug uart below in any case.
    if (csa.dbg_en & 1)
        cdc_dbg_tx((const uint8_t *)data, min(CDN_MAX_PAYLOAD, len));
    if (csa.dbg_en & 2)
        net_dbg_tx((const uint8_t *)data, len);

    dbg_uart_write((const uint8_t *)data, len);
    return len;
}
