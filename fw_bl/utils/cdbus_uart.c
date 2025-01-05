/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

#include "cd_utils.h"
#include "cd_list.h"
#include "cdbus_uart.h"


void cduart_dev_init(cduart_dev_t *dev, list_head_t *free_head)
{
    dev->rx_frame = cduart_frame_get(free_head);
    dev->free_head = free_head;
    dev->t_last = get_systick();
    dev->rx_crc = 0xffff;

#ifdef USE_DYNAMIC_INIT
    list_head_init(&dev->rx_head);
    list_head_init(&dev->tx_head);
    dev->rx_byte_cnt = 0;
    dev->rx_drop = false;
#endif
}

// handler

void cduart_rx_handle(cduart_dev_t *dev, const uint8_t *buf, unsigned len)
{
    unsigned max_len;
    unsigned cpy_len;
    const uint8_t *rd = buf;

    while (true) {
        cd_frame_t *frame = dev->rx_frame;

        if (dev->rx_byte_cnt != 0 && get_systick() - dev->t_last > CDUART_IDLE_TIME) {
            printf("bus: timeout [%02x %02x %02x] %d, %d\n",
                    frame->dat[0], frame->dat[1], frame->dat[2], dev->rx_byte_cnt, dev->rx_drop);
            for (int i = 0; i < dev->rx_byte_cnt; i++)
                printf("%02x ", frame->dat[i]);
            printf("\n");
            dev->rx_byte_cnt = 0;
            dev->rx_crc = 0xffff;
            dev->rx_drop = false;
        }

        if (!len || rd == buf + len)
            return;
        max_len = buf + len - rd;
        dev->t_last = get_systick();

        if (dev->rx_byte_cnt < 3)
            cpy_len = min(3 - dev->rx_byte_cnt, max_len);
        else
            cpy_len = min(frame->dat[2] + 5 - dev->rx_byte_cnt, max_len);

        if (!dev->rx_drop)
            memcpy(frame->dat + dev->rx_byte_cnt, rd, cpy_len);
        dev->rx_byte_cnt += cpy_len;

        if (dev->rx_byte_cnt == 3 &&
                    (frame->dat[2] > CD_FRAME_SIZE - 5 ||
                            (frame->dat[1] != 0xff && frame->dat[1] != dev->local_mac))) {
            printf("bus: drop [%x %x %x]\n", frame->dat[0], frame->dat[1], frame->dat[2]);
            dev->rx_drop = true;
        }

        if (!dev->rx_drop)
            dev->rx_crc = crc16_sub(rd, cpy_len, dev->rx_crc);
        rd += cpy_len;

        if (dev->rx_byte_cnt == frame->dat[2] + 5) {
            if (!dev->rx_drop) {
                if (dev->rx_crc != 0) {
                    printf("bus: !crc [%x %x %x]\n", frame->dat[0], frame->dat[1], frame->dat[2]);

                } else {
                    cd_frame_t *frm = cduart_frame_get(dev->free_head);
                    if (frm) {
                        cduart_list_put(&dev->rx_head, &dev->rx_frame->node);
                        dev->rx_frame = frm;
                    } else {
                        printf("bus: rx lost\n");
                    }
                }
            }
            dev->rx_byte_cnt = 0;
            dev->rx_crc = 0xffff;
            dev->rx_drop = false;
        }
    }
}
