/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __DBG_UART_H__
#define __DBG_UART_H__

#include <stdint.h>

// first thing in app_main(): sets up the dma path and prints whatever the
// previous run left unsent, then the reset cause
void dbg_uart_init(void);

// queue text for the debug uart; the dma drains it, nothing here waits on
// the uart. From an interrupt, or with interrupts masked, what does not fit
// is dropped; a thread waits for room, bounded.
void dbg_uart_write(const uint8_t *dat, int len);

// send what is queued by polling, from any context, including a fault
// handler with everything masked. Call before a deliberate reset or before
// hanging on purpose, so the last lines actually get out.
void dbg_uart_flush(void);

extern uint32_t dbg_uart_drop_cnt;

#endif
