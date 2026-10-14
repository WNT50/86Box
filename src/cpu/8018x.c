/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          8018x integrated peripheral control block: the on-chip peripherals
 *          of the 80186/80188 that the CPU core in 808x.c does not provide.
 *          Relocation and the timers work; the interrupt controller, DMA
 *          and chip selects are stored only.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include "x86seg.h"
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/log.h>
#include <86box/io.h>
#include <86box/dma.h>
#include <86box/mem.h>
#include "8018x.h"

#define I8018X_PCB_BASE_DEFAULT 0xFF00u
#define I8018X_PCB_SIZE         0x100u
#define I8018X_RELOC_OFF        0xFEu

/* Reset values per Intel 270788-001 section 5.1. */
#define I8018X_RELOC_RESET      0x20FFu /* RELO reads 0x20FF after RESET */
#define I8018X_UMCS_RESET       0xFFFBu /* UMCS reads 0xFFFB after RESET */

/* PCB register offsets (word). */
#define I8018X_ICU_END          0x3Fu
#define I8018X_TMR_COUNT(i)     (0x50u + ((i) * 8u)) /* T0 0x50 / T1 0x58 / T2 0x60 */
#define I8018X_TMR_MAXA(i)      (0x52u + ((i) * 8u))
#define I8018X_TMR_MAXB(i)      (0x54u + ((i) * 8u))
#define I8018X_TMR_CON(i)       (0x56u + ((i) * 8u))
#define I8018X_TMR_END          0x67u
#define I8018X_CS_BASE          0xA0u /* UMCS/LMCS/MMCS/MPCS/PACS, 0xA0-0xA9 */
#define I8018X_CS_END           0xA9u
#define I8018X_DMA0_BASE        0xC0u /* channel 0 descriptor, 0xC0-0xCB */
#define I8018X_DMA0_END         0xCBu
#define I8018X_DMA1_BASE        0xD0u /* channel 1 descriptor, 0xD0-0xDB */
#define I8018X_DMA1_END         0xDBu

/* Word indices of the channel 0 descriptor inside regs[]. */
#define I8018X_DMA0_SRC_LO      (I8018X_DMA0_BASE / 2)
#define I8018X_DMA0_SRC_HI      ((I8018X_DMA0_BASE + 2) / 2)
#define I8018X_DMA0_DST_LO      ((I8018X_DMA0_BASE + 4) / 2)
#define I8018X_DMA0_DST_HI      ((I8018X_DMA0_BASE + 6) / 2)
#define I8018X_DMA0_COUNT       ((I8018X_DMA0_BASE + 8) / 2)
#define I8018X_DMA0_CTRL        ((I8018X_DMA0_BASE + 10) / 2)

/* Interrupt controller request/status (270788-001 section 9.6.2). */
#define I8018X_ICU_REQ          0x2Eu /* Interrupt Request Register */
#define I8018X_ICU_STATUS       0x30u /* Interrupt Controller Status Register */

typedef struct i8018x_pcb_t {
    uint16_t base;        /* current PCB I/O base */
    uint16_t reloc;       /* Relocation Register value */
    uint8_t  handler_set; /* whether our I/O handler is currently registered */
    uint16_t regs[I8018X_PCB_SIZE / 2];
    uint64_t tmr_base[3];   /* tsc when the timer last (re)started its count cycle */
    uint64_t tmr_src[3];    /* clock-source events already counted at that moment */
    uint64_t tmr_done[3];   /* maximum-count cycles completed so far */
    uint16_t tmr_req;       /* sticky timer request latches: bit i = timer i */
    uint8_t  tmr_alt[3];    /* MAX COUNT register in use: 0 = A, 1 = B (ALT) */
} i8018x_pcb_t;

/* The 3477-S disk service moves a sector with the on-chip DMA channel 0 --
   source = the floppy data port, destination = the buffer it then verifies --
   and never programs the board's 8237, so the floppy controller hands each
   byte over through these helpers instead.  Only channel 0 and its
   incrementing memory destination are modelled; the terminal count rides in
   DMA_OVER, as in dma_channel_write()/dma_channel_read(). */

static i8018x_pcb_t *i8018x_pcb_dma_dev = NULL;

int
i8018x_pcb_dma0_ready(void)
{
    i8018x_pcb_t *dev = i8018x_pcb_dma_dev;

    /* Armed while the programmed transfer count has not run out. */
    return (dev != NULL) && (dev->regs[I8018X_DMA0_COUNT] != 0);
}

static uint32_t
i8018x_pcb_dma0_dest(i8018x_pcb_t *dev)
{
    return ((uint32_t) dev->regs[I8018X_DMA0_DST_LO]) |
           ((uint32_t) dev->regs[I8018X_DMA0_DST_HI] << 16);
}

static void
i8018x_pcb_dma0_dest_set(i8018x_pcb_t *dev, uint32_t dest)
{
    dev->regs[I8018X_DMA0_DST_LO] = (uint16_t) (dest & 0xFFFFu);
    dev->regs[I8018X_DMA0_DST_HI] = (uint16_t) (dest >> 16);
}

/* Shared tail of one byte transfer: step the destination pointer on, drop the
   count and latch the terminal count when it runs out. */
static int
i8018x_pcb_dma0_step(i8018x_pcb_t *dev)
{
    i8018x_pcb_dma0_dest_set(dev, i8018x_pcb_dma0_dest(dev) + 1);

    if (--dev->regs[I8018X_DMA0_COUNT] == 0) {
        /* The channel's TC status bit latches.  The interrupt controller is
           still unwired, so no DMA interrupt is raised here. */
        dev->regs[I8018X_DMA0_CTRL] |= 0x0008u;
        return DMA_OVER;
    }

    return 0;
}

int
i8018x_pcb_dma0_write(uint8_t val)
{
    if (!i8018x_pcb_dma0_ready())
        return DMA_OVER;

    mem_writeb_phys(i8018x_pcb_dma0_dest(i8018x_pcb_dma_dev), val);

    return i8018x_pcb_dma0_step(i8018x_pcb_dma_dev);
}

int
i8018x_pcb_dma0_read(void)
{
    int ret;

    if (!i8018x_pcb_dma0_ready())
        return DMA_OVER | 0xFF;

    ret = mem_readb_phys(i8018x_pcb_dma0_dest(i8018x_pcb_dma_dev));

    return ret | i8018x_pcb_dma0_step(i8018x_pcb_dma_dev);
}

static uint16_t i8018x_pcb_inw(uint16_t port, void *priv);
static void     i8018x_pcb_outw(uint16_t port, uint16_t val, void *priv);

/* The three timers count at one fourth of the CPU clock (270788-001 section
   6.2). 86Box drives tsc at three times the nominal CPU clock on XT-class
   machines (pit.c: cpuclock = rspeed * 3), so one CPU/4 event is 12 tsc. */
#define I8018X_TMR_TSC_PER_EVENT   12u

/* Maximum count of the register currently selected; ALT alternates between
   MAX COUNT register A and B on timers 0 and 1. */
static uint64_t
i8018x_pcb_tmr_max(i8018x_pcb_t *dev, int i)
{
    uint16_t reg = (dev->tmr_alt[i] && (i != 2)) ? I8018X_TMR_MAXB(i) : I8018X_TMR_MAXA(i);

    uint64_t m = (uint64_t) dev->regs[reg / 2];

    /* A maximum count register of 0 means 65536, not 1. Treating it as 1 makes
       a prescaled timer see a huge jump in its source-event count the moment
       the MAX register is written, latching a spurious request (the ROM's
       timer test treats an early request as a failure). */
    return m ? (m + 1u) : 0x10000u;
}

/* Clock events counted by a timer since it was last started. Timer 2 always
   runs from the internal clock; timers 0 and 1 run from timer 2's
   maximum-count output when the Prescaler bit (bit 3) is set. */
static uint64_t
i8018x_pcb_tmr_events(i8018x_pcb_t *dev, int i)
{
    uint64_t src;

    if ((i != 2) && (dev->regs[I8018X_TMR_CON(i) / 2] & 0x0008u))
        src = i8018x_pcb_tmr_events(dev, 2) / i8018x_pcb_tmr_max(dev, 2);
    else
        src = (tsc - dev->tmr_base[i]) / I8018X_TMR_TSC_PER_EVENT;

    return src - dev->tmr_src[i];
}

/* (Re)start a timer's count cycle from the value in its count register. */
static void
i8018x_pcb_tmr_restart(i8018x_pcb_t *dev, int i)
{
    dev->tmr_base[i] = tsc;
    dev->tmr_src[i]  = ((i != 2) && (dev->regs[I8018X_TMR_CON(i) / 2] & 0x0008u))
                           ? (i8018x_pcb_tmr_events(dev, 2) / i8018x_pcb_tmr_max(dev, 2))
                           : 0u;
    dev->tmr_done[i] = 0;
    dev->tmr_alt[i]  = 0;
}

/* Advance the timers to the current tsc. A timer that reaches its maximum
   count latches a request (when its interrupt is enabled) and, unless
   CONT is set, stops counting until the program enables it again. */
static void
i8018x_pcb_tmr_update(i8018x_pcb_t *dev)
{
    for (int i = 0; i < 3; i++) {
        uint16_t con = dev->regs[I8018X_TMR_CON(i) / 2];

        if (!(con & 0x8000u))
            continue;

        dev->tmr_alt[i] = (uint8_t) (((con & 0x0002u) && (i != 2)) ? (dev->tmr_done[i] & 1u) : 0u);

        uint64_t max = i8018x_pcb_tmr_max(dev, i);
        uint64_t cyc = i8018x_pcb_tmr_events(dev, i) / max;

        if (cyc <= dev->tmr_done[i])
            continue;

        dev->tmr_done[i] = cyc;

        if (con & 0x2000u) /* INT: the timer drives its interrupt request */
            dev->tmr_req |= (uint16_t) (1u << i);

        if (!(con & 0x0001u)) /* CONT = 0: the timer halts on maximum count */
            dev->regs[I8018X_TMR_CON(i) / 2] &= (uint16_t) ~0x8000u;
    }
}

static uint16_t
i8018x_pcb_read_word(i8018x_pcb_t *dev, uint16_t off)
{
    if (off == I8018X_RELOC_OFF)
        return dev->reloc;

    /* The count registers free-run while their timer is enabled (270788-001
       section 6.1: the count resets to zero when it equals the maximum). */
    i8018x_pcb_tmr_update(dev);

    for (int i = 0; i < 3; i++) {
        if (off == I8018X_TMR_COUNT(i)) {
            if (dev->regs[I8018X_TMR_CON(i) / 2] & 0x8000u)
                return (uint16_t) (i8018x_pcb_tmr_events(dev, i) % i8018x_pcb_tmr_max(dev, i));
            return dev->regs[off / 2];
        }
    }

    /* The request register's timer bit is the OR of the timer requests; the
       status register holds them individually in bits 0-2. */
    if (off == I8018X_ICU_REQ)
        return (uint16_t) (dev->regs[off / 2] | (dev->tmr_req ? 0x0001u : 0x0000u));
    if (off == I8018X_ICU_STATUS)
        return (uint16_t) (dev->regs[off / 2] | (dev->tmr_req & 0x0007u));

    return dev->regs[off / 2];
}

static void
i8018x_pcb_write_word(i8018x_pcb_t *dev, uint16_t off, uint16_t val)
{
    pclog("3477: i80186 wr %04X <- %04X (pc=%04X:%04X)\n", off, val, CS, cpu_state.pc);

    if (off == I8018X_RELOC_OFF) {
        /* Manual 270788-001 section 5.1 / Figure 53: bits 11:0 are address
           bits A19:A8 of the new base, bit 12 selects memory (1) or I/O (0)
           space, bits 15:13 carry the ESC-trap and master/slave controls. */
        uint16_t new_base = (uint16_t) ((val & 0x0FFFu) << 8);

        dev->reloc        = val;
        dev->regs[off / 2] = val;

        /* Only the I/O-space PCB is modelled; a memory-space PCB is stored
           but left where it is. */
        if (!(val & 0x1000u) && (new_base != dev->base)) {
            io_removehandler(dev->base, I8018X_PCB_SIZE, NULL, i8018x_pcb_inw, NULL, NULL, i8018x_pcb_outw, NULL, dev);
            dev->base = new_base;
            io_sethandler(dev->base, I8018X_PCB_SIZE, NULL, i8018x_pcb_inw, NULL, NULL, i8018x_pcb_outw, NULL, dev);
        }
        return;
    }

    dev->regs[off / 2] = val;

    /* A control or count-register write (re)starts the count cycle; the request
       latch itself is only touched through the interrupt controller registers. */
    for (int i = 0; i < 3; i++) {
        if ((off == I8018X_TMR_CON(i)) || (off == I8018X_TMR_COUNT(i)) ||
            (off == I8018X_TMR_MAXA(i)) || (off == I8018X_TMR_MAXB(i)))
            i8018x_pcb_tmr_restart(dev, i);
    }

    /* Writing the status register sets or clears the individual timer request
       bits (270788-001 Figure 26) -- the ROM forces a request with 0x8001 and
       clears them again with 0x8000. */
    if (off == I8018X_ICU_STATUS) {
        dev->regs[off / 2] = (uint16_t) (val & ~0x0007u);
        dev->tmr_req       = (uint16_t) ((dev->tmr_req & ~0x0007u) | (val & 0x0007u));
    }

    /* Writing a zero to the request register's timer bit clears it; the ROM
       does `in 0x2E / and 0xFFFE / out 0x2E` before its timer test. */
    if ((off == I8018X_ICU_REQ) && !(val & 0x0001u))
        dev->tmr_req = 0;
}

static uint16_t
i8018x_pcb_inw(uint16_t port, void *priv)
{
    i8018x_pcb_t *dev = (i8018x_pcb_t *) priv;
    uint16_t  off = (uint16_t) ((port - dev->base) & 0xFEu);

    return i8018x_pcb_read_word(dev, off);
}

static void
i8018x_pcb_outw(uint16_t port, uint16_t val, void *priv)
{
    i8018x_pcb_write_word((i8018x_pcb_t *) priv, (uint16_t) ((port - ((i8018x_pcb_t *) priv)->base) & 0xFEu), val);
}

static void
i8018x_pcb_reset(void *priv)
{
    i8018x_pcb_t *dev = (i8018x_pcb_t *) priv;

    if (dev->handler_set)
        io_removehandler(dev->base, I8018X_PCB_SIZE, NULL, i8018x_pcb_inw, NULL, NULL, i8018x_pcb_outw, NULL, dev);

    memset(dev->regs, 0x00, sizeof(dev->regs));
    dev->tmr_req = 0;
    for (int i = 0; i < 3; i++) {
        dev->tmr_base[i] = 0;
        dev->tmr_src[i]  = 0;
        dev->tmr_done[i] = 0;
        dev->tmr_alt[i]  = 0;
    }

    /* Reset values from 270788-001 section 5.3. */
    dev->regs[I8018X_CS_BASE / 2]   = I8018X_UMCS_RESET; /* UMCS */
    dev->reloc                      = I8018X_RELOC_RESET;
    dev->regs[I8018X_RELOC_OFF / 2] = dev->reloc;
    dev->base                       = I8018X_PCB_BASE_DEFAULT;

    io_sethandler(dev->base, I8018X_PCB_SIZE, NULL, i8018x_pcb_inw, NULL, NULL, i8018x_pcb_outw, NULL, dev);
    dev->handler_set = 1;
}

static void
i8018x_pcb_close(void *priv)
{
    i8018x_pcb_t *dev = (i8018x_pcb_t *) priv;

    if (dev->handler_set)
        io_removehandler(dev->base, I8018X_PCB_SIZE, NULL, i8018x_pcb_inw, NULL, NULL, i8018x_pcb_outw, NULL, dev);

    if (i8018x_pcb_dma_dev == dev)
        i8018x_pcb_dma_dev = NULL;

    free(dev);
}

static void *
i8018x_pcb_init(const device_t *info)
{
    i8018x_pcb_t *dev = (i8018x_pcb_t *) calloc(1, sizeof(i8018x_pcb_t));

    i8018x_pcb_reset(dev);

    i8018x_pcb_dma_dev = dev;

    return dev;
}

const device_t i8018x_pcb_device = {
    .name          = "Intel 8018x Integrated Peripheral Control Block",
    .internal_name = "i8018x_pcb",
    .flags         = 0,
    .local         = 0,
    .init          = i8018x_pcb_init,
    .close         = i8018x_pcb_close,
    .reset         = i8018x_pcb_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
