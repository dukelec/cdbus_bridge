/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

/*
 * The serial side of the bridge.
 *
 * It is the original interface and it still behaves the way it always did,
 * so the existing tools keep working unchanged. What the port carries is
 * picked by the rate the host opens it with:
 *
 *   0xcdcd  config: frames are served by the local services, the bus is
 *           not touched at all
 *   other   data: whole CDBUS frames, with crc, in both directions
 *
 * and, when the bus mode is 4 or above, the raw mode replaces all of that
 * with a plain byte stream to usart1. The network port has nothing to
 * bridge then and reports no carrier.
 *
 * Everything here runs from the main loop next to tud_task(). The only
 * exception is the raw mode's frame lists, which the uart dma interrupt
 * also touches and which are irq safe already.
 */

#include "app_main.h"
#include "tusb.h"

static cduart_dev_t d_dev = {0};

static cd_frame_t *tx_frm = NULL;   // staged for the usb fifo
static uint8_t rx_buf[256];
static uint8_t rsp_buf[CDN_MAX_PAYLOAD];

cdc_cnt_t cdc_cnt = {0};

uint32_t cdc_rate = 0;
uint32_t cdc_rate_final = 0;

/*
 * Deliberately not cleared on bus reset. The host only sends
 * SET_CONTROL_LINE_STATE when the application opens or closes the port, so
 * after a reset-resume (a pc waking up) it would never be asserted again
 * and the port would stay silent until it is reopened or replugged.
 */
volatile bool cdc_dtr = false;
static uint32_t dtr_t = 0;


void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    (void)rts;
    if (dtr && !cdc_dtr)
        dtr_t = get_systick();
    cdc_dtr = dtr;
}

void tud_cdc_line_coding_cb(uint8_t itf, const cdc_line_coding_t *coding)
{
    (void)itf;
    cdc_rate = coding->bit_rate;
}

// the host holds the port
static bool cdc_open(void)
{
    return tud_ready() && cdc_dtr;
}

// ... and may be talked to. Nothing is sent for the first few ms after the
// port is opened: a program that has not set the tty raw yet still has
// echo on, and anything of ours it saw then would come back at us, be
// parsed as incoming frames and put on the bus. Nothing of ours reaches it
// in that window, so what it sends meanwhile is its own and is kept.
static bool cdc_up(void)
{
    return cdc_open() && get_systick() - dtr_t > 5;
}

static bool cfg_mode(void)
{
    return cdc_rate == CDC_CONFIG_RATE;
}

bool cdc_bus_active(void)
{
    return cdc_up() && !cfg_mode() && !raw_mode;
}

bool cdc_tx_live(void)
{
    return cdc_up() && !raw_mode;
}

void cdc_bus_rx(cd_frame_t *frame)
{
    frame_cache_put(&d_dev.tx_head, frame);
    cdc_cnt.to_pc++;
}

uint32_t cdc_queued_to_host(void)
{
    return d_dev.tx_head.len + raw_rx_head.len + (tx_frm ? 1 : 0);
}

cd_frame_t *cdc_tx_evict(void)
{
    return cd_list_get(&d_dev.tx_head);
}

uint32_t cdc_queued_to_bus(void)
{
    return d_dev.rx_head.len + raw_tx_head.len;
}


//--------------------------------------------------------------------
// framed mode
//--------------------------------------------------------------------

static void cdc_rx_task(void)
{
    while (tud_cdc_available()) {
        // leave the bytes in the usb fifo rather than read what we cannot
        // turn into frames: that is the back pressure the host needs
        if (!frame_dir_ok(false) || !frame_pool_ready())
            break;
        // a chunk can parse into a frame every 5 bytes, so never read more
        // than what the pool holds above its reserve, or what is left of
        // the share, can take
        uint32_t room = min(frame_free_head.len - FRAME_RESERVE,
                FRAME_DIR_MAX - frame_dir_len(false));
        uint32_t n = min((uint32_t)sizeof(rx_buf), room * 5);
        n = tud_cdc_read(rx_buf, n);
        if (!n)
            break;
        cduart_rx_handle(&d_dev, rx_buf, n);
    }
}

static void cdc_tx_task(void)
{
    while (true) {
        if (!tx_frm)
            tx_frm = cd_list_get(&d_dev.tx_head);
        if (!tx_frm)
            break;
        uint16_t len = tx_frm->dat[2] + 5; // src, dst, len, dat..., crc
        if (tud_cdc_write_available() < len)
            break;
        cduart_fill_crc(tx_frm->dat);
        tud_cdc_write(tx_frm->dat, len);
        cd_list_put(&frame_free_head, tx_frm);
        tx_frm = NULL;
    }
    tud_cdc_write_flush();
}

// hand the controller everything the host has framed. Its queue is drained
// by its own interrupt, and moving a frame there does not change the to-bus
// share, so the share must not gate this: cdc_rx_task() already stops
// reading once the share is spent, and gating here as well left the frames
// in rx_head for good once it alone filled the share.
static void bus_tx_task(void)
{
    cd_frame_t *frm;

    if (hw_raw)
        return;
    while ((frm = cd_list_get(&d_dev.rx_head)) != NULL) {
        bus_tx(frm);
        cdc_cnt.to_bus++;
    }
}

// serve the local services, the same ones the network node offers
static void config_task(void)
{
    cd_frame_t *frm;

    while ((frm = cd_list_get(&d_dev.rx_head)) != NULL) {
        uint8_t sport = frm->dat[3];
        uint8_t dport = frm->dat[4];
        int req_len = (int)frm->dat[2] - 2;
        int rsp_len;

        if (req_len < 0) {
            cd_list_put(&frame_free_head, frm);
            continue;
        }
        rsp_len = comm_service_handle(dport, frm->dat + 5, req_len,
                rsp_buf, sizeof(rsp_buf));
        cdc_cnt.local++;
        if (rsp_len < 0) {
            cd_list_put(&frame_free_head, frm);
            continue;
        }

        frm->dat[1] = frm->dat[0];
        frm->dat[0] = 0xff;
        frm->dat[2] = rsp_len + 2;
        frm->dat[3] = dport;
        frm->dat[4] = sport;
        memcpy(frm->dat + 5, rsp_buf, rsp_len);
        frame_cache_put(&d_dev.tx_head, frm);
    }
}


//--------------------------------------------------------------------
// raw mode
//--------------------------------------------------------------------

// append to the frame already waiting at the end of the uart tx list where
// possible, so a burst of small usb reads becomes one uart transfer
static void raw_feed(const uint8_t *p, unsigned len)
{
    while (len) {
        uint32_t flags;
        cd_irq_save(&raw_tx_head.lock, flags);
        cd_frame_t *frm = list_entry_safe(raw_tx_head.last, cd_frame_t);
        if (frm && frm->dat[257] == 255)
            frm = NULL;
        if (!frm) {
            cd_irq_restore(&raw_tx_head.lock, flags);
            frm = cd_list_get(&frame_free_head);
            if (!frm)
                break;
            frm->dat[257] = 0;
        }
        r_dev.tx_cnt++;
        unsigned sub_len = min(255 - frm->dat[257], len);
        memcpy(frm->dat + frm->dat[257], p, sub_len);
        frm->dat[257] += sub_len;
        if (frm != list_entry_safe(raw_tx_head.last, cd_frame_t))
            cd_list_put(&raw_tx_head, frm);
        else
            cd_irq_restore(&raw_tx_head.lock, flags);
        uart_dma_tx();
        p += sub_len;
        len -= sub_len;
    }
}

static void raw_rx_task(void)
{
    while (tud_cdc_available()) {
        if (!frame_dir_ok(false) || !frame_pool_ready())
            break;
        uint32_t n = tud_cdc_read(rx_buf, sizeof(rx_buf));
        if (!n)
            break;
        raw_feed(rx_buf, n);
    }
}

static void raw_tx_task(void)
{
    while (true) {
        if (!tx_frm)
            tx_frm = cd_list_get(&raw_rx_head);
        if (!tx_frm)
            break;
        if (tud_cdc_write_available() < tx_frm->dat[257])
            break;
        r_dev.rx_cnt++;
        tud_cdc_write(tx_frm->dat, tx_frm->dat[257]);
        cd_list_put(&frame_free_head, tx_frm);
        tx_frm = NULL;
    }
    tud_cdc_write_flush();
}


//--------------------------------------------------------------------
// glue
//--------------------------------------------------------------------

bool cdc_dbg_tx(const uint8_t *dat, int len)
{
    cd_frame_t *frm;

    // the raw mode carries a byte stream, there is nothing to put a frame in.
    // No check on the port being open though: the boot log is printed long
    // before a host can be there, and waiting in the queue until one
    // connects is the only way it is ever seen.
    if (hw_raw)
        return false;
    if (len <= 0 || len > CDN_MAX_PAYLOAD)
        return false;
    if (!frame_dbg_ready())
        return false;
    frm = cd_list_get(&frame_free_head);
    if (!frm)
        return false;

    frm->dat[0] = 0xff;
    frm->dat[1] = 0x00;
    frm->dat[2] = len + 2;
    frm->dat[3] = 64;
    frm->dat[4] = 9;
    memcpy(frm->dat + 5, dat, len);
    frame_cache_put(&d_dev.tx_head, frm);
    return true;
}

void cdc_init(void)
{
    cduart_dev_init(&d_dev, &frame_free_head);
    d_dev.local_mac = 0xff; // the host composes whole frames, take them all
    // until a host opens the port the bus keeps the configured rate
    cdc_rate_final = csa.bus_cfg.baud_h;
}

void cdc_poll(void)
{
    // drain the uart dma ring whatever the port is doing, otherwise it wraps
    // unnoticed and the next read picks up from the wrong place
    if (hw_raw)
        uart_dma_rx();

    // a staged frame carries its length differently in each mode, so it must
    // not survive a switch between them
    bool raw_now = hw_raw && !cfg_mode();
    if (raw_now != raw_mode) {
        raw_mode = raw_now;
        if (tx_frm) {
            cd_list_put(&frame_free_head, tx_frm);
            tx_frm = NULL;
        }
    }

    if (!cdc_open()) {
        cd_frame_t *frm;
        if (tx_frm) {
            cd_list_put(&frame_free_head, tx_frm);
            tx_frm = NULL;
        }
        // half a stream from a port that is closed is stale, and nothing
        // drains rx_head while it stays closed, so it would hold part of
        // the to-bus share against the network port for good. The same
        // goes for bytes still in the usb fifo, which would otherwise be
        // parsed the moment the port is opened again. And for what is
        // still on its way out: the next program to open the port would
        // get it before it has set the tty raw, and echo it back at us.
        while ((frm = cd_list_get(&d_dev.rx_head)) != NULL)
            cd_list_put(&frame_free_head, frm);
        tud_cdc_read_flush();
        tud_cdc_write_clear();
        return;
    }
    // just opened, see cdc_up(): what the host sends waits in the usb fifo
    // until the hold-off is over, a request sent right after opening the
    // port must not be lost to it
    if (!cdc_up())
        return;

    if (cdc_rate != CDC_CONFIG_RATE && cdc_rate)
        cdc_rate_final = cdc_rate;

    if (raw_mode) {
        raw_rx_task();
        raw_tx_task();
        return;
    }

    cdc_rx_task();
    if (cfg_mode())
        config_task();
    else
        bus_tx_task();
    cdc_tx_task();
}
