/*
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2017, DUKELEC, Inc.
 * All rights reserved.
 *
 * Author: Duke Fong <d@d-l.io>
 */

/*
 * The debug uart, fed through a ring buffer and drained by dma, so that a
 * printf costs the caller a memcpy rather than 5 us a byte of busy waiting
 * on the main loop.
 *
 * The ring lives in .noinit, the lowest 2 KB of the stack reservation, and
 * survives every reset but a power cycle. What a crash or a watchdog left
 * unsent is therefore printed by the next boot, and what the firmware can
 * see coming (a fault, a deliberate reset) is flushed on the spot by
 * dbg_uart_flush(), which needs no interrupts at all.
 */

#include "app_main.h"

#define DBG_UART        UART7
#define DBG_DMA         DMA1
#define DBG_DMA_CH      DMA1_CHANNEL3
#define DBG_DMA_MUX     DMA1MUX_CHANNEL3
#define DBG_DMA_REQ     DMAMUX_DMAREQ_ID_UART7_TX
#define DBG_DMA_FLAG    DMA1_FDT3_FLAG      // full data transfer, channel 3
#define DBG_DMA_IRQ     DMA1_Channel3_IRQn

#define DMA_CCR_EN      (1 << 0)

#define RING_SIZE       2048                // power of two
#define RING_MASK       (RING_SIZE - 1)
#define RING_MAGIC      0xdb9cd1a5

// how long a thread may wait for room: the ring drains in 10 ms at 2 Mbps,
// so anything longer means the uart is not moving and the text is dropped
#define WAIT_MAX_MS     50

_Static_assert((RING_SIZE & RING_MASK) == 0, "ring size must be a power of two");

// head and tail only ever grow; the count is their difference and the index
// into buf is the low bits, so a wrap costs nothing. Both are validated on
// boot before anything is trusted.
static struct {
    uint32_t            magic;
    volatile uint32_t   head;       // next byte to write
    volatile uint32_t   tail;       // next byte the uart has not sent yet
    uint8_t             buf[RING_SIZE];
} ring __attribute__((section(".noinit")));

static volatile uint32_t dma_len = 0;   // length of the running chunk, 0: idle
static bool inited = false;
static cd_spinlock_t lock = {0};

uint32_t dbg_uart_drop_cnt = 0;


static void poll_tx(const uint8_t *dat, int len)
{
    while (len--) {
        while (!(DBG_UART->sts & USART_TDBE_FLAG));
        DBG_UART->dt = *dat++;
    }
}

static void poll_str(const char *s)
{
    poll_tx((const uint8_t *)s, strlen(s));
}

// with interrupts masked, or from the dma isr
static void start_next(void)
{
    uint32_t n = ring.head - ring.tail;
    uint32_t idx, len;

    if (!n)
        return;
    idx = ring.tail & RING_MASK;
    len = min(n, RING_SIZE - idx);      // up to the end of the buffer
    dma_len = len;

    DBG_DMA_CH->ctrl &= ~DMA_CCR_EN;
    DBG_DMA_CH->dtcnt = len;
    DBG_DMA_CH->maddr = (uint32_t)&ring.buf[idx];
    DBG_DMA_CH->ctrl |= DMA_CCR_EN;
}

void DMA1_Channel3_IRQHandler(void)
{
    DBG_DMA->clr = DBG_DMA_FLAG;
    ring.tail += dma_len;
    dma_len = 0;
    start_next();
}


void dbg_uart_write(const uint8_t *dat, int len)
{
    uint32_t flags, used, idx, first;
    uint32_t t_start = get_systick();
    bool can_wait;

    if (len <= 0)
        return;
    if (!inited) {
        poll_tx(dat, len);
        return;
    }
    if (len > RING_SIZE) {
        // keep the end, that is where a crash report would be
        dat += len - RING_SIZE;
        len = RING_SIZE;
    }

    // waiting is only allowed where the isr that makes room can run
    can_wait = !__get_IPSR() && !__get_PRIMASK();

    while (true) {
        cd_irq_save(&lock, flags);
        used = ring.head - ring.tail;
        if (RING_SIZE - used >= (uint32_t)len) {
            idx = ring.head & RING_MASK;
            first = min((uint32_t)len, RING_SIZE - idx);
            memcpy(&ring.buf[idx], dat, first);
            memcpy(&ring.buf[0], dat + first, len - first);
            ring.head += len;
            if (!dma_len)
                start_next();
            cd_irq_restore(&lock, flags);
            return;
        }
        cd_irq_restore(&lock, flags);

        if (!can_wait || get_systick() - t_start > WAIT_MAX_MS) {
            dbg_uart_drop_cnt++;
            return;
        }
    }
}

void dbg_uart_flush(void)
{
    uint32_t flags, n;

    cd_irq_save(&lock, flags);
    if (!inited) {
        cd_irq_restore(&lock, flags);
        return;
    }

    if (dma_len) {
        // let the running chunk finish, but a dead uart must not hang a
        // fault handler: give it twice what 2 KB takes, then move on
        for (int i = 0; i < 20000 && !(DBG_DMA->sts & DBG_DMA_FLAG); i++)
            delay_us(1);
        DBG_DMA->clr = DBG_DMA_FLAG;
        ring.tail += dma_len;
        dma_len = 0;
    }
    DBG_DMA_CH->ctrl &= ~DMA_CCR_EN;

    // whatever is being reported may have damaged the indexes, so never
    // print more than the ring holds
    n = ring.head - ring.tail;
    if (n > RING_SIZE)
        n = RING_SIZE;
    while (n--) {
        while (!(DBG_UART->sts & USART_TDBE_FLAG));
        DBG_UART->dt = ring.buf[ring.tail & RING_MASK];
        ring.tail++;
    }
    ring.head = ring.tail;
    while (!(DBG_UART->sts & USART_TDC_FLAG)); // the last byte is out
    cd_irq_restore(&lock, flags);
}


static const char *reset_cause(void)
{
    static char buf[40];
    char *p = buf;
    const struct { uint32_t flag; const char *name; } causes[] = {
        { CRM_POR_RESET_FLAG,      "por" },
        { CRM_NRST_RESET_FLAG,     "nrst" },
        { CRM_SW_RESET_FLAG,       "sw" },
        { CRM_WDT_RESET_FLAG,      "wdt" },
        { CRM_WWDT_RESET_FLAG,     "wwdt" },
        { CRM_LOWPOWER_RESET_FLAG, "lowpower" },
    };

    for (unsigned i = 0; i < sizeof(causes) / sizeof(causes[0]); i++) {
        if (crm_flag_get(causes[i].flag) != SET)
            continue;
        if (p != buf)
            *p++ = ' ';
        strcpy(p, causes[i].name);
        p += strlen(causes[i].name);
    }
    if (p == buf)
        strcpy(buf, "none");
    crm_flag_clear(CRM_ALL_RESET_FLAG);
    return buf;
}

void dbg_uart_init(void)
{
    dma_init_type dma_init_struct;
    const char *cause = reset_cause();
    uint32_t unsent = 0;

    // only a ring a previous run left behind intact is trusted
    if (ring.magic == RING_MAGIC && ring.head - ring.tail <= RING_SIZE)
        unsent = ring.head - ring.tail;

    if (unsent) {
        poll_str("\n--- unsent before reset (");
        poll_str(cause);
        poll_str(") ---\n");
        while (unsent--) {
            poll_tx(&ring.buf[ring.tail & RING_MASK], 1);
            ring.tail++;
        }
        poll_str("\n--- end ---\n");
    }

    ring.magic = RING_MAGIC;
    ring.head = ring.tail = 0;
    dma_len = 0;

    dma_reset(DBG_DMA_CH);
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.peripheral_base_addr = (uint32_t)&DBG_UART->dt;
    dma_init_struct.priority = DMA_PRIORITY_LOW;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DBG_DMA_CH, &dma_init_struct);
    dmamux_enable(DBG_DMA, TRUE);
    dmamux_init(DBG_DMA_MUX, DBG_DMA_REQ);
    dma_interrupt_enable(DBG_DMA_CH, DMA_FDT_INT, TRUE);
    usart_dma_transmitter_enable(DBG_UART, TRUE);
    // lowest priority: it only ever starts the next chunk
    nvic_irq_enable(DBG_DMA_IRQ, 3, 0);

    inited = true;
    printf("reset cause: %s\n", cause);
}


//--------------------------------------------------------------------
// hard fault
//--------------------------------------------------------------------

static void out_hex32(uint32_t val)
{
    const char hex[] = "0123456789abcdef";
    uint8_t s[8];

    for (int i = 7; i >= 0; i--) {
        s[i] = hex[val & 0xf];
        val >>= 4;
    }
    dbg_uart_write(s, 8);
}

static void out_str(const char *s)
{
    dbg_uart_write((const uint8_t *)s, strlen(s));
}

/*
 * Entered from HardFault_Handler with r0 = msp, which is the exception
 * frame: r0-r3, r12, lr, pc, psr. No printf here, the heap or the stdio
 * state may be what broke.
 */
void dbg_hard_fault(const uint32_t *frame)
{
    out_str("\nhard fault: cfsr ");
    out_hex32(SCB->CFSR);
    out_str(" hfsr ");
    out_hex32(SCB->HFSR);
    out_str(" mmfar ");
    out_hex32(SCB->MMFAR);
    out_str(" bfar ");
    out_hex32(SCB->BFAR);
    out_str("\n  pc ");
    out_hex32(frame[6]);
    out_str(" lr ");
    out_hex32(frame[5]);
    out_str(" psr ");
    out_hex32(frame[7]);
    out_str(" sp ");
    out_hex32((uint32_t)frame);
    out_str("\n");
    dbg_uart_flush();
    while (true);
}
