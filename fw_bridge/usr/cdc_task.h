/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CDC_TASK_H__
#define __CDC_TASK_H__

// the rate the host opens the port with selects what the port carries
#define CDC_CONFIG_RATE     0xcdcd

typedef struct {
    uint32_t to_bus;
    uint32_t to_pc;
    uint32_t local;
} cdc_cnt_t;

extern cdc_cnt_t cdc_cnt;

void cdc_init(void);
void cdc_poll(void);

// true while the serial port is an endpoint on the bus: open, in data mode,
// and not swallowing the bus whole the way the raw mode does
bool cdc_bus_active(void);
// true while something drains the frames queued for the host: the port is
// open in the data or the config mode. Not in the raw mode, and not while
// the port is closed, which is where the boot log waits.
bool cdc_tx_live(void);
void cdc_bus_rx(cd_frame_t *frame);

// frames this side is holding, for the host and for the bus
uint32_t cdc_queued_to_host(void);
uint32_t cdc_queued_to_bus(void);
// give back the oldest frame waiting for the host, NULL if there is none
cd_frame_t *cdc_tx_evict(void);

// queue debug text for the host, false if there is no room for it
bool cdc_dbg_tx(const uint8_t *dat, int len);

// the rate the port was last opened with, which is also the bus baud rate
// while the port is open; see bus_baud_task()
extern uint32_t cdc_rate;
extern uint32_t cdc_rate_final;
extern volatile bool cdc_dtr;

#endif
