/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#ifndef __USB_DESC_H__
#define __USB_DESC_H__

// endpoint addresses, here rather than in usb_desc.c so that a task can
// ask the usb core about the state of one of its own endpoints
#define EPNUM_NET_NOTIF     0x81
#define EPNUM_NET_OUT       0x02
#define EPNUM_NET_IN        0x82
#define EPNUM_CDC_NOTIF     0x83
#define EPNUM_CDC_OUT       0x04
#define EPNUM_CDC_IN        0x84

#endif
