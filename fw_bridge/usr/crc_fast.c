/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "app_main.h"

extern CRC_HandleTypeDef hcrc;


void crc16_sub(const uint8_t *data, uint32_t length, uint16_t *crc_val)
{
#ifdef CRC_IRQ_SAFE
    uint8_t flags;
    local_irq_save(cpu_flags);
#endif

    hcrc.Instance->INIT = *crc_val;
    __HAL_CRC_DR_RESET(&hcrc);
    hcrc.Instance->INIT = hcrc.Instance->DR; // bit-reversal for init value

    unsigned cnt = length / 4;
    while (cnt--) {
        hcrc.Instance->DR = get_unaligned_be32(data);
        data += 4;
    }

    length = length % 4;
    while (length--)
        *(volatile uint8_t *)&hcrc.Instance->DR = *data++;

    *crc_val = hcrc.Instance->DR;

#ifdef CRC_IRQ_SAFE
    local_irq_restore(cpu_flags);
#endif
}
