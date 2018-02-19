/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <duke@dukelec.com>
 */

#ifndef __CDNET_CONFIG_H__
#define __CDNET_CONFIG_H__

#define ARCH_SPI

#define DEBUG
//#define VERBOSE
#define DBG_LEN             40
//#define DBG_TX_IT

#define CDUART_IRQ_SAFE
#define CDUART_IDLE_TIME    (500000 / SYSTICK_US_DIV) // 500 ms

#define SEQ_TIMEOUT         (500000 / SYSTICK_US_DIV) // 500 ms

#endif
