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

#ifdef CRC_HW_IRQ_SAFE
static cd_spinlock_t crc_lock = {0};
#endif


uint16_t crc16_hw_sub(const uint8_t *data, uint32_t length, uint16_t crc_val)
{
    uint16_t ret_val;
#ifdef CRC_HW_IRQ_SAFE // not recommended, avoid large critical sections
    uint32_t flags;
    cd_irq_save(&crc_lock, flags);
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
#ifdef CRC_HW_IRQ_SAFE
    cd_irq_restore(&crc_lock, flags);
#endif
    return ret_val;
}
