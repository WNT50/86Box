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
#include <86box/i80186.h>

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

/* Interrupt controller request/status registers (270788-001 section 9.6.2). */
#define I80186_ICU_REQ          0x2Eu /* Interrupt Request Register */
#define I80186_ICU_STATUS       0x30u /* Interrupt Controller Status Register */

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

    free(dev);
}

static void *
i80186_init(const device_t *info)
{
    i80186_t *dev = (i80186_t *) calloc(1, sizeof(i80186_t));

    i80186_reset(dev);

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