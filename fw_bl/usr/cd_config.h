/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __CD_CONFIG_H__
#define __CD_CONFIG_H__

//#define ARCH_SPI
#define CD_LIST_IT
#define CD_FRAME_SIZE       258

#define DEBUG
//#define VERBOSE
#define DBG_STR_LEN         160
#define DBG_TX_IT
//#define LIST_DEBUG

#define CDN_IRQ_SAFE
#define CDUART_IRQ_SAFE
#define CDUART_IDLE_TIME    (500000 / SYSTICK_US_DIV) // 500 ms

//#define CDN_RB_TREE
//#define CDN_L0_C

#include "main.h"  // generated by stm32cube
#include "debug_config.h"

#endif
