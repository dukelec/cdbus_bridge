/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"


uint16_t crc16_sub(const uint8_t *data, uint32_t length, uint16_t crc_val)
{
    uint16_t ret_val;
#ifdef CRC_IRQ_SAFE
    uint32_t flags;
    local_irq_save(flags);
#endif
    CRC->idt = crc_val;
    CRC->ctrl = 0xe9;
    CRC->idt = CRC->dt; // bit-reverse crc_val

    while (((unsigned)data & 3) && length) {
        *(volatile uint8_t *)&CRC->dt = *data++;
        length--;
    }

    unsigned cnt = length >> 2;
    while (cnt--) {
        CRC->dt = *(uint32_t *)data;
        data += 4;
    }

    length &= 3;
    while (length--)
        *(volatile uint8_t *)&CRC->dt = *data++;

    ret_val = CRC->dt;
#ifdef CRC_IRQ_SAFE
    local_irq_restore(flags);
#endif
    return ret_val;
}
