/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "modbus_crc.h"

#define INIT    idt
#define CR      ctrl
#define DR      dt


uint16_t crc16_sub(const uint8_t *data, uint32_t length, uint16_t crc_val)
{
    uint16_t ret_val;
#ifdef CRC_IRQ_SAFE
    uint32_t flags;
    local_irq_save(flags);
#endif
    CRC->INIT = crc_val;
    CRC->CR = 0xe9;
    CRC->INIT = CRC->DR; // bit-reverse crc_val

    while (((unsigned)data & 3) && length) {
        *(volatile uint8_t *)&CRC->DR = *data++;
        length--;
    }

    unsigned cnt = length >> 2;
    while (cnt--) {
        CRC->DR = *(uint32_t *)data;
        data += 4;
    }

    length &= 3;
    while (length--)
        *(volatile uint8_t *)&CRC->DR = *data++;

    ret_val = CRC->DR;
#ifdef CRC_IRQ_SAFE
    local_irq_restore(flags);
#endif
    return ret_val;
}
