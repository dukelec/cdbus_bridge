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
#define CDN_MAX_DAT         253

#define DEBUG
//#define VERBOSE
//#define LIST_DEBUG

#define CDUART_IRQ_SAFE
#define CDUART_IDLE_TIME    (500000 / SYSTICK_US_DIV) // 500 ms

#include "wk_system.h"
//#include "at32f402_405.h"
#include "debug_config.h"

#endif
