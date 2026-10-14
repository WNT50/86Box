/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          80186/80188 integrated peripheral control block (PCB).
 *
 * [Cloud server only, DO NOT USE!!!]
 * EXPERIMENTAL PROTOTYPE - NOT FOR PRODUCTION USE, NOT VALIDATED.
 *
 * This is a bounded attempt at modelling the 80186 on-chip peripherals that
 * the CPU core (src/cpu/808x.c) does NOT provide. It is a register-level
 * model: the Relocation Register is functional, the three timers count on
 * read, and the interrupt-controller / DMA / chip-select blocks are stored
 * but NOT wired to the rest of the emulator (they neither raise interrupts
 * nor perform transfers nor drive chip selects).
 *
 * Register map and reset values follow Intel 270788-001:
 *   Figure 51 (80186/80188 PCB layout), section 5.1 / Figure 53 (Relocation
 *   Register). The relocation encoding is bits 11:0 = address bits A19:A8 of
 *   the new base, bit 12 = M/IO (1 = memory, 0 = I/O), bits 15:13 = ESC-trap
 *   and master/slave controls; the register reads 20FFh after RESET, placing
 *   the PCB at the top of I/O space (0xFF00-0xFFFF). Refresh Control (0xE4)
 *   and Power-Save (0xF0) exist only on the 80C186/80C188 and are therefore
 *   not part of this 80186 map.
 *
 * Authors: (prototype added for the `80186` branch)
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <86box/86box.h>
#include "cpu.h"
#include "x86seg.h"
#include <86box/device.h>
#include <86box/io.h>
#include <86box/log.h>
#include <86box/timer.h>
#include <86box/dma.h>
#include <86box/i80186.h>
#include <86box/mem.h>

#define I80186_PCB_BASE_DEFAULT 0xFF00u
#define I80186_PCB_SIZE         0x100u
#define I80186_RELOC_OFF        0xFEu

/* Reset values per Intel 270788-001 section 5.1. */
#define I80186_RELOC_RESET      0x20FFu /* RELO reads 0x20FF after RESET */
#define I80186_UMCS_RESET       0xFFFBu /* UMCS reads 0xFFFB after RESET */

/* PCB register offsets (word). */
#define I80186_ICU_END          0x3Fu
#define I80186_TMR_COUNT(i)     (0x50u + ((i) * 8u)) /* T0 0x50 / T1 0x58 / T2 0x60 */
#define I80186_TMR_MAXA(i)      (0x52u + ((i) * 8u))
#define I80186_TMR_MAXB(i)      (0x54u + ((i) * 8u))
#define I80186_TMR_CON(i)       (0x56u + ((i) * 8u))
#define I80186_TMR_END          0x67u
#define I80186_CS_BASE          0xA0u /* UMCS/LMCS/MMCS/MPCS/PACS, 0xA0-0xA9 */
#define I80186_CS_END           0xA9u
#define I80186_DMA0_BASE        0xC0u /* channel 0 descriptor, 0xC0-0xCB */
#define I80186_DMA0_END         0xCBu
#define I80186_DMA1_BASE        0xD0u /* channel 1 descriptor, 0xD0-0xDB */
#define I80186_DMA1_END         0xDBu

/* Word indices of the channel 0 descriptor inside regs[]. */
#define I80186_DMA0_SRC_LO      (I80186_DMA0_BASE / 2)
#define I80186_DMA0_SRC_HI      ((I80186_DMA0_BASE + 2) / 2)
#define I80186_DMA0_DST_LO      ((I80186_DMA0_BASE + 4) / 2)
#define I80186_DMA0_DST_HI      ((I80186_DMA0_BASE + 6) / 2)
#define I80186_DMA0_COUNT       ((I80186_DMA0_BASE + 8) / 2)
#define I80186_DMA0_CTRL        ((I80186_DMA0_BASE + 10) / 2)

/* Interrupt controller request/status registers (270788-001 section 9.6.2). */
#define I80186_ICU_REQ          0x2Eu /* Interrupt Request Register */
#define I80186_ICU_STATUS       0x30u /* Interrupt Controller Status Register */
#define I80186_ICU_EOI          0x22u /* End of Interrupt Register (Fig. 87) */
#define I80186_ICU_POLL         0x24u
#define I80186_ICU_POLLSTATUS   0x26u
#define I80186_ICU_MASK         0x28u /* Mask Register (Fig. 83) */
#define I80186_ICU_PRIMASK      0x2Au /* Priority Mask Register (Fig. 84) */
#define I80186_ICU_INSERVICE    0x2Cu /* In-Service Register (Fig. 85) */

/* [EXPERIMENT icu3477] On-chip interrupt-controller latches for the 3477-S disk
   completion; bit 0 is that source.  Declared up here because the register
   handlers below use them. */
/* [EXPERIMENT evseq] One monotonically increasing sequence shared by every
   instrumented event, so the FDC's interrupt raises, the guest's MSR reads and
   its EOIs can be placed on a single timeline (see the wait loop at
   0650:863F which times out unless the ISR sets [53B8] bit 7 in time). */
uint64_t ev_seq = 0;

static uint16_t icu3477_req = 0;     /* request latch */
static uint16_t icu3477_isr = 0;     /* in-service latch */
uint32_t        icu3477_delivered = 0; /* completions delivered */
uint32_t        icu3477_waited = 0;    /* completions that had to wait for EOI */

typedef struct i80186_t {
    uint16_t base;        /* current PCB I/O base */
    uint16_t reloc;       /* Relocation Register value */
    uint8_t  handler_set; /* whether our I/O handler is currently registered */
    uint16_t regs[I80186_PCB_SIZE / 2];
    uint64_t tmr_base[3];   /* tsc when the timer last (re)started its count cycle */
    uint64_t tmr_src[3];    /* clock-source events already counted at that moment */
    uint64_t tmr_done[3];   /* maximum-count cycles completed so far */
    uint16_t tmr_req;       /* sticky timer request latches: bit i = timer i */
    uint8_t  tmr_alt[3];    /* MAX COUNT register in use: 0 = A, 1 = B (ALT) */
} i80186_t;

/* ---------------------------------------------------------------------------
 * [cloud] 3477-S: the on-chip DMA channel 0 is the only data path the IPL's
 * disk service uses -- it programs source = the floppy data port (0x3F5, I/O
 * space), destination = the buffer it then verifies, count = one sector, and
 * starts the channel, while the board's 8237 is never touched.  The transfer
 * therefore has to happen here; the floppy controller hands over each byte it
 * would have driven onto the bus.  The return values mirror
 * dma_channel_write()/dma_channel_read(): the terminal count rides in
 * DMA_OVER.
 *
 * Only channel 0, and only the incrementing memory destination the IPL uses,
 * are modelled; the other channels still just store their registers.
 * ------------------------------------------------------------------------- */

static i80186_t *i80186_dma_dev = NULL;

/* [EXPERIMENT icutrace] Which PCs touch which interrupt-controller register with
   which value, de-duplicated so polling loops do not flood the log.  Needed to
   settle whether the ISR's `out 0x722,0x000E` is a specific EOI for a channel
   whose control register really holds 0x0E, or something else entirely. */
static void
i80186_icu_trace(uint16_t off, uint16_t val, int is_write)
{
    /* NOTE: `pc` and `cs` are macros in cpu.h, so the members must not use
       those names. */
    static struct { uint16_t tpc, tcs, toff, tval; uint8_t twr, tused; } seen[512];
    static unsigned n_seen = 0, n_seen_max = 0;

    if (!getenv("ICUTR"))
        return;
    if ((off < 0x20u) || (off > 0x3Fu))
        return;

    n_seen++;
    for (unsigned i = 0; i < n_seen_max; i++) {
        if ((seen[i].tpc == cpu_state.pc) && (seen[i].tcs == CS) &&
            (seen[i].toff == off) && (seen[i].tval == val) &&
            (seen[i].twr == (uint8_t) !!is_write))
            return;
    }
    if (n_seen_max < (sizeof(seen) / sizeof(seen[0]))) {
        seen[n_seen_max].tpc  = cpu_state.pc;
        seen[n_seen_max].tcs  = CS;
        seen[n_seen_max].toff = off;
        seen[n_seen_max].tval = val;
        seen[n_seen_max].twr  = (uint8_t) !!is_write;
        seen[n_seen_max].tused = 1;
        n_seen_max++;
        pclog("3477: ★ICU %s off=%02X val=%04X (pc=%04X:%04X) [累计 %u]\n",
              is_write ? "写" : "读", off, val, CS, cpu_state.pc, n_seen);
    }
}

int
i80186_dma0_ready(void)
{
    i80186_t *dev = i80186_dma_dev;
    int       rdy = (dev != NULL) && (dev->regs[I80186_DMA0_COUNT] != 0);

    /* [EXPERIMENT DMA0RDY] Log readiness transitions only: does the count stay 0
       after the guest reprograms it (which would starve the FDC delivery path)? */
    if (getenv("DMA0RDY")) {
        static int last = -1;

        if (rdy != last) {
            pclog("3477: ☆dma0ready %d->%d count=%u pc=%04X:%04X\n", last, rdy,
                  (dev != NULL) ? (unsigned) dev->regs[I80186_DMA0_COUNT] : 0,
                  (unsigned) (cs >> 4), cpu_state.pc);
            last = rdy;
        }
    }

    return rdy;
}

static uint32_t
i80186_dma0_dest(i80186_t *dev)
{
    return ((uint32_t) dev->regs[I80186_DMA0_DST_LO]) |
           ((uint32_t) dev->regs[I80186_DMA0_DST_HI] << 16);
}

static void
i80186_dma0_dest_set(i80186_t *dev, uint32_t dest)
{
    dev->regs[I80186_DMA0_DST_LO] = (uint16_t) (dest & 0xFFFFu);
    dev->regs[I80186_DMA0_DST_HI] = (uint16_t) (dest >> 16);
}

/* Shared tail of one byte transfer: step the destination pointer on, drop the
   count and latch the terminal count when it runs out. */
static int
i80186_dma0_step(i80186_t *dev)
{
    i80186_dma0_dest_set(dev, i80186_dma0_dest(dev) + 1);

    if (--dev->regs[I80186_DMA0_COUNT] == 0) {
        /* The channel's TC status bit latches.  The interrupt controller is
           still unwired, so no DMA interrupt is raised here. */
        dev->regs[I80186_DMA0_CTRL] |= 0x0008u;
        return DMA_OVER;
    }

    return 0;
}

int
i80186_dma0_write(uint8_t val)
{
    if (!i80186_dma0_ready())
        return DMA_OVER;

    mem_writeb_phys(i80186_dma0_dest(i80186_dma_dev), val);

    return i80186_dma0_step(i80186_dma_dev);
}

int
i80186_dma0_read(void)
{
    int ret;

    if (!i80186_dma0_ready())
        return DMA_OVER | 0xFF;

    ret = mem_readb_phys(i80186_dma0_dest(i80186_dma_dev));

    return ret | i80186_dma0_step(i80186_dma_dev);
}

static uint16_t i80186_inw(uint16_t port, void *priv);
static void     i80186_outw(uint16_t port, uint16_t val, void *priv);

/* The three timers count at one fourth of the CPU clock (270788-001 section
   6.2). 86Box drives tsc at three times the nominal CPU clock on XT-class
   machines (pit.c: cpuclock = rspeed * 3), so one CPU/4 event is 12 tsc. */
#define I80186_TMR_TSC_PER_EVENT   12u

/* Maximum count of the register currently selected; ALT alternates between
   MAX COUNT register A and B on timers 0 and 1. */
static uint64_t
i80186_tmr_max(i80186_t *dev, int i)
{
    uint16_t reg = (dev->tmr_alt[i] && (i != 2)) ? I80186_TMR_MAXB(i) : I80186_TMR_MAXA(i);

    uint64_t m = (uint64_t) dev->regs[reg / 2];

    /* 270788-001: a maximum count register of 0 means 65536, not 1.  Treating
       it as 1 makes a prescaled timer see a huge jump in its source-event
       count the moment the MAX register is written, latching a spurious
       request (the ROM's timer test treats an early request as a failure). */
    return m ? (m + 1u) : 0x10000u;
}

/* Clock events counted by a timer since it was last started. Timer 2 always
   runs from the internal clock; timers 0 and 1 run from timer 2's
   maximum-count output when the Prescaler bit (bit 3) is set. */
static uint64_t
i80186_tmr_events(i80186_t *dev, int i)
{
    uint64_t src;

    if ((i != 2) && (dev->regs[I80186_TMR_CON(i) / 2] & 0x0008u))
        src = i80186_tmr_events(dev, 2) / i80186_tmr_max(dev, 2);
    else
        src = (tsc - dev->tmr_base[i]) / I80186_TMR_TSC_PER_EVENT;

    return src - dev->tmr_src[i];
}

/* (Re)start a timer's count cycle from the value in its count register. */
static void
i80186_tmr_restart(i80186_t *dev, int i)
{
    dev->tmr_base[i] = tsc;
    dev->tmr_src[i]  = ((i != 2) && (dev->regs[I80186_TMR_CON(i) / 2] & 0x0008u))
                           ? (i80186_tmr_events(dev, 2) / i80186_tmr_max(dev, 2))
                           : 0u;
    dev->tmr_done[i] = 0;
    dev->tmr_alt[i]  = 0;
}

/* Advance the timers to the current tsc. A timer that reaches its maximum
   count latches a request (when its interrupt is enabled) and, unless CONT is
   set, stops counting until the program enables it again. */
static void
i80186_tmr_update(i80186_t *dev)
{
    for (int i = 0; i < 3; i++) {
        uint16_t con = dev->regs[I80186_TMR_CON(i) / 2];

        if (!(con & 0x8000u))
            continue;

        dev->tmr_alt[i] = (uint8_t) (((con & 0x0002u) && (i != 2)) ? (dev->tmr_done[i] & 1u) : 0u);

        uint64_t max = i80186_tmr_max(dev, i);
        uint64_t cyc = i80186_tmr_events(dev, i) / max;

        if (cyc <= dev->tmr_done[i])
            continue;

        dev->tmr_done[i] = cyc;

        if (con & 0x2000u) /* INT: the timer drives its interrupt request */
            dev->tmr_req |= (uint16_t) (1u << i);

        if (!(con & 0x0001u)) /* CONT = 0: the timer halts on maximum count */
            dev->regs[I80186_TMR_CON(i) / 2] &= (uint16_t) ~0x8000u;
    }
}

static uint16_t
i80186_read_word(i80186_t *dev, uint16_t off)
{
    if (off == I80186_RELOC_OFF)
        return dev->reloc;

    /* The count registers free-run while their timer is enabled (270788-001
       section 6.1: the count resets to zero when it equals the maximum). */
    i80186_tmr_update(dev);

    i80186_icu_trace(off, dev->regs[off / 2], 0);

    for (int i = 0; i < 3; i++) {
        if (off == I80186_TMR_COUNT(i)) {
            if (dev->regs[I80186_TMR_CON(i) / 2] & 0x8000u)
                return (uint16_t) (i80186_tmr_events(dev, i) % i80186_tmr_max(dev, i));
            return dev->regs[off / 2];
        }
    }

    /* The request register's timer bit is the OR of the timer requests; the
       status register holds them individually in bits 0-2. */
    if (off == I80186_ICU_REQ)
        return (uint16_t) (dev->regs[off / 2] | (dev->tmr_req ? 0x0001u : 0x0000u));
    if (off == I80186_ICU_STATUS)
        return (uint16_t) (dev->regs[off / 2] | (dev->tmr_req & 0x0007u));

    return dev->regs[off / 2];
}

static void
i80186_write_word(i80186_t *dev, uint16_t off, uint16_t val)
{
    /* [EXPERIMENT icuack] Progress meter: the guest's disk ISR acknowledges a
       completed result phase by writing the vector number to PCB+0x22
       (`0650:8779 mov ax,0xE / out dx,ax` with dx=0x722).  Counting those tells
       us whether the boot keeps completing disk operations or is stuck
       retrying one. */
    if (off == 0x22 && val == 0x000E) {
        static uint32_t n = 0;

        n++;
        if (getenv("EVSEQ"))
            pclog("3477: EV%llu eoi000E n=%u (pc=%04X:%04X)\n", ++ev_seq, n, CS, cpu_state.pc);
    }

    i80186_icu_trace(off, val, 1);

    pclog("3477: i80186 wr %04X <- %04X (pc=%04X:%04X)\n", off, val, CS, cpu_state.pc);

    if (off == I80186_RELOC_OFF) {
        /* Manual 270788-001 section 5.1 / Figure 53: bits 11:0 are address
           bits A19:A8 of the new base, bit 12 selects memory (1) or I/O (0)
           space, bits 15:13 carry the ESC-trap and master/slave controls. */
        uint16_t new_base = (uint16_t) ((val & 0x0FFFu) << 8);

        dev->reloc        = val;
        dev->regs[off / 2] = val;

        /* Only the I/O-space PCB is modelled; a memory-space PCB is stored
           but left where it is. */
        if (!(val & 0x1000u) && (new_base != dev->base)) {
            io_removehandler(dev->base, I80186_PCB_SIZE, NULL, i80186_inw, NULL, NULL, i80186_outw, NULL, dev);
            dev->base = new_base;
            io_sethandler(dev->base, I80186_PCB_SIZE, NULL, i80186_inw, NULL, NULL, i80186_outw, NULL, dev);
        }
        return;
    }

    dev->regs[off / 2] = val;

    /* A control or count-register write (re)starts the count cycle; the request
       latch itself is only touched through the interrupt controller registers. */
    for (int i = 0; i < 3; i++) {
        if ((off == I80186_TMR_CON(i)) || (off == I80186_TMR_COUNT(i)) ||
            (off == I80186_TMR_MAXA(i)) || (off == I80186_TMR_MAXB(i)))
            i80186_tmr_restart(dev, i);
    }

    /* Writing the status register sets or clears the individual timer request
       bits (270788-001 Figure 26) -- the ROM forces a request with 0x8001 and
       clears them again with 0x8000. */
    if (off == I80186_ICU_STATUS) {
        dev->regs[off / 2] = (uint16_t) (val & ~0x0007u);
        dev->tmr_req       = (uint16_t) ((dev->tmr_req & ~0x0007u) | (val & 0x0007u));
    }

    /* Writing a zero to the request register's timer bit clears it; the ROM
       does `in 0x2E / and 0xFFFE / out 0x2E` before its timer test. */
    if ((off == I80186_ICU_REQ) && !(val & 0x0001u))
        dev->tmr_req = 0;

    /* [EXPERIMENT icu3477] End of Interrupt Register (PCB+0x22, Figure 87):
       a *specific* EOI names the interrupt type to take out of service.  The
       guest's disk ISR acknowledges with `out 0x722,0x000E` (measured at
       pc=0650:8780), so type 0x0E is the disk completion on this machine. */
    if (off == I80186_ICU_EOI) {
        if ((val & 0x00FFu) == 0x000Eu) {
            icu3477_isr &= ~0x0001u;
            icu3477_req &= ~0x0001u;
        }
    }
}

static uint16_t
i80186_inw(uint16_t port, void *priv)
{
    i80186_t *dev = (i80186_t *) priv;
    uint16_t  off = (uint16_t) ((port - dev->base) & 0xFEu);

    return i80186_read_word(dev, off);
}

static void
i80186_outw(uint16_t port, uint16_t val, void *priv)
{
    i80186_write_word((i80186_t *) priv, (uint16_t) ((port - ((i80186_t *) priv)->base) & 0xFEu), val);
}

/* [EXPERIMENT icu3477] Native delivery of the 3477-S disk completion through the
   80186's on-chip interrupt controller (270788-001 section 9.5/9.6, Figure 80):

     - PCB+0x2E (Interrupt Request Register): internal-source requests are
       *latched* (section 9.5.3.1) and cleared by the processor;
     - PCB+0x22 (End of Interrupt Register): a specific EOI takes the named
       interrupt type out of service;
     - PCB+0x28 (Mask Register, guest programs 0x0098) / 0x2A (Priority Mask, 7).

   Evidence that the vector is 0x0E: IBMBIO's disk ISR lives at IVT entry 0x0E
   and acknowledges with `out 0x722,0x000E` (measured, pc=0650:8780).  The
   on-chip ICU is what the model has been missing; the three environment-gated
   switches (FDC3477IRQ6 / FDC5550IRQ / PIC3477AEOI) only impersonate it. */
void
i80186_icu3477_complete(void)
{
    if (!getenv("ICU3477"))
        return;
    if (i80186_dma_dev == NULL)
        return;

    icu3477_req |= 0x0001u;

    /* A second completion while the first is still in service waits, as on the
       real chip; the guest's specific EOI releases it. */
    if (icu3477_isr & 0x0001u) {
        icu3477_waited++;
        return;
    }

    /* Mask Register bit 0 covers this source and the guest programs 0x0098
       (bit 0 clear = unmasked); honour it in case a guest masks it later. */
    if (i80186_dma_dev->regs[I80186_ICU_MASK / 2] & 0x0001u)
        return;

    icu3477_isr |= 0x0001u;
    icu3477_delivered++;

    /* NOTE: no vector is injected from here.  Measured + manual evidence says the
       vector arrives through the *external* 8259 (guest initialised it with base
       0x08, so IRQ6 -> vector 0x0E, and the guest's specific EOI names exactly
       that type), with the on-chip ICU cascaded behind it.  Delivery therefore
       stays with picint() in fdc_int(); this function only maintains the ICU's
       request / in-service state, which the guest's `out 0x722,0x000E` clears.
       An earlier attempt that called x86_int(0x0E) directly took the machine
       into the weeds (segment CCCC:) -- do not retry that. */
}

/* Progress meter: how often a completion had to wait for the guest's EOI. */


static void
i80186_reset(void *priv)
{
    i80186_t *dev = (i80186_t *) priv;

    if (dev->handler_set)
        io_removehandler(dev->base, I80186_PCB_SIZE, NULL, i80186_inw, NULL, NULL, i80186_outw, NULL, dev);

    memset(dev->regs, 0x00, sizeof(dev->regs));
    dev->tmr_req = 0;
    for (int i = 0; i < 3; i++) {
        dev->tmr_base[i] = 0;
        dev->tmr_src[i]  = 0;
        dev->tmr_done[i] = 0;
        dev->tmr_alt[i]  = 0;
    }

    /* Reset values from 270788-001 section 5.3. */
    dev->regs[I80186_CS_BASE / 2]   = I80186_UMCS_RESET; /* UMCS */
    dev->reloc                      = I80186_RELOC_RESET;
    dev->regs[I80186_RELOC_OFF / 2] = dev->reloc;
    dev->base                       = I80186_PCB_BASE_DEFAULT;

    io_sethandler(dev->base, I80186_PCB_SIZE, NULL, i80186_inw, NULL, NULL, i80186_outw, NULL, dev);
    dev->handler_set = 1;
}

static void
i80186_close(void *priv)
{
    i80186_t *dev = (i80186_t *) priv;

    if (dev->handler_set)
        io_removehandler(dev->base, I80186_PCB_SIZE, NULL, i80186_inw, NULL, NULL, i80186_outw, NULL, dev);

    if (i80186_dma_dev == dev)
        i80186_dma_dev = NULL;

    free(dev);
}

static void *
i80186_init(const device_t *info)
{
    i80186_t *dev = (i80186_t *) calloc(1, sizeof(i80186_t));

    i80186_reset(dev);

    i80186_dma_dev = dev; /* [cloud] 3477-S disk-service DMA channel 0 */

    return dev;
}

const device_t i80186_device = {
    .name          = "Intel 80186 Integrated Peripherals (EXPERIMENTAL)",
    .internal_name = "i80186",
    .flags         = 0,
    .local         = 0,
    .init          = i80186_init,
    .close         = i80186_close,
    .reset         = i80186_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};