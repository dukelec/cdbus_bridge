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

#define DBG_UART        USART5
#define DBG_DMA         DMA2
#define DBG_DMA_CH      DMA2_Channel5       // set up for usart5_tx by stm32cube
#define DBG_DMA_FLAG    DMA_ISR_TCIF5       // transfer complete, channel 5
#define DBG_DMA_IRQ     DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn

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
        while (!(DBG_UART->ISR & USART_ISR_TXE_TXFNF));
        DBG_UART->TDR = *dat++;
    }
}

static void poll_str(const char *s)
{
    poll_tx((const uint8_t *)s, strlen(s));
}

// with interrupts masked
static void start_next(void)
{
    uint32_t n = ring.head - ring.tail;
    uint32_t idx, len;

    if (!n)
        return;
    idx = ring.tail & RING_MASK;
    len = min(n, RING_SIZE - idx);      // up to the end of the buffer
    dma_len = len;

    DBG_DMA_CH->CCR &= ~DMA_CCR_EN;
    DBG_DMA_CH->CNDTR = len;
    DBG_DMA_CH->CMAR = (uint32_t)&ring.buf[idx];
    DBG_DMA_CH->CCR |= DMA_CCR_EN;
}

// shared with dma1 ch4-7 and the other dma2 channels, only ch5 is enabled
void DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQHandler(void)
{
    uint32_t flags;

    if (!(DBG_DMA->ISR & DBG_DMA_FLAG))
        return;
    // a printf from a higher priority irq must not restart the chunk in
    // between the two steps below
    cd_irq_save(&lock, flags);
    DBG_DMA->IFCR = DBG_DMA_FLAG;
    ring.tail += dma_len;
    dma_len = 0;
    start_next();
    cd_irq_restore(&lock, flags);
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
        for (int i = 0; i < 20000 && !(DBG_DMA->ISR & DBG_DMA_FLAG); i++)
            delay_us(1);
        DBG_DMA->IFCR = DBG_DMA_FLAG;
        ring.tail += dma_len;
        dma_len = 0;
    }
    DBG_DMA_CH->CCR &= ~DMA_CCR_EN;

    // whatever is being reported may have damaged the indexes, so never
    // print more than the ring holds
    n = ring.head - ring.tail;
    if (n > RING_SIZE)
        n = RING_SIZE;
    while (n--) {
        while (!(DBG_UART->ISR & USART_ISR_TXE_TXFNF));
        DBG_UART->TDR = ring.buf[ring.tail & RING_MASK];
        ring.tail++;
    }
    ring.head = ring.tail;
    while (!(DBG_UART->ISR & USART_ISR_TC)); // the last byte is out
    cd_irq_restore(&lock, flags);
}


// the bootloader reports the flags itself and clears them before it jumps
// here, so this only finds any when the app was started some other way
static const char *reset_cause(void)
{
    static char buf[48];
    char *p = buf;
    uint32_t csr = RCC->CSR;
    const struct { uint32_t flag; const char *name; } causes[] = {
        { RCC_CSR_PWRRSTF,  "por" },
        { RCC_CSR_PINRSTF,  "nrst" },
        { RCC_CSR_SFTRSTF,  "sw" },
        { RCC_CSR_IWDGRSTF, "iwdg" },
        { RCC_CSR_WWDGRSTF, "wwdg" },
        { RCC_CSR_LPWRRSTF, "lowpower" },
        { RCC_CSR_OBLRSTF,  "obl" },
    };

    for (unsigned i = 0; i < sizeof(causes) / sizeof(causes[0]); i++) {
        if (!(csr & causes[i].flag))
            continue;
        if (p != buf)
            *p++ = ' ';
        strcpy(p, causes[i].name);
        p += strlen(causes[i].name);
    }
    if (p == buf)
        strcpy(buf, "none");
    RCC->CSR |= RCC_CSR_RMVF;
    return buf;
}

void dbg_uart_init(void)
{
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

    DBG_DMA_CH->CCR = 0;
    LL_DMA_SetPeriphRequest(DBG_DMA, LL_DMA_CHANNEL_5, LL_DMAMUX_REQ_USART5_TX);
    DBG_DMA_CH->CPAR = (uint32_t)&DBG_UART->TDR;
    // memory to peripheral, bytes, low priority, interrupt on completion
    DBG_DMA_CH->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_TCIE;
    DBG_DMA->IFCR = DMA_IFCR_CGIF5;
    DBG_UART->CR3 |= USART_CR3_DMAT;
    // lowest priority: it only ever starts the next chunk
    NVIC_SetPriority(DBG_DMA_IRQ, 3);
    NVIC_EnableIRQ(DBG_DMA_IRQ);

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
 * state may be what broke. The m0+ has no fault status registers.
 */
void dbg_hard_fault(const uint32_t *frame)
{
    out_str("\nhard fault: pc ");
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
