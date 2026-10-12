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
#define I80186_ICU_BASE         0x20u /* 0x20-0x3E interrupt controller */
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
    uint64_t tmr_start[3];  /* tsc captured when the timer was enabled/loaded */
    uint64_t tmr_period[3]; /* non-zero while the timer is enabled */
    uint64_t tmr_ack[3];    /* overflow count when the request was last acknowledged */
} i80186_t;

static uint16_t i80186_inw(uint16_t port, void *priv);
static void     i80186_outw(uint16_t port, uint16_t val, void *priv);

/* Overflows counted since the timer was enabled; a disabled timer keeps the
   value it had, so acknowledging one is a no-op. */
static uint64_t
i80186_tmr_overflows(i80186_t *dev, int i)
{
    if (!dev->tmr_period[i])
        return dev->tmr_ack[i];

    uint64_t period = (uint64_t) dev->regs[I80186_TMR_MAXA(i) / 2] + 1u;
    return ((tsc - dev->tmr_start[i]) / 4u) / period;
}

/* Timers that have overflowed since their request was acknowledged
   (bit 0/1/2 = timer 0/1/2).  The 3477-S ROM enables a timer and then
   requires the request and status bits to latch, so the hardware side has
   to drive them rather than just store what was written. */
static uint16_t
i80186_tmr_pending(i80186_t *dev)
{
    uint16_t mask = 0;

    for (int i = 0; i < 3; i++) {
        uint16_t con = dev->regs[I80186_TMR_CON(i) / 2];
        if (!(con & 0x8000u) || !(con & 0x2000u))
            continue;
        if (i80186_tmr_overflows(dev, i) > dev->tmr_ack[i])
            mask |= (uint16_t) (1u << i);
    }

    return mask;
}

static uint16_t
i80186_read_word(i80186_t *dev, uint16_t off)
{
    if (off == I80186_RELOC_OFF)
        return dev->reloc;

    /* The count registers free-run while the timer is enabled. 86Box has no
       hook for the 80186 timers, so derive one from tsc, counting up at the
       CPU clock / 4 (270788-001 section 6: "every fourth CPU Clock"). */
    for (int i = 0; i < 3; i++) {
        if (off == I80186_TMR_COUNT(i)) {
            uint16_t max = dev->regs[I80186_TMR_MAXA(i) / 2];
            if (dev->tmr_period[i]) {
                uint64_t ticks  = (tsc - dev->tmr_start[i]) / 4u;
                uint64_t period = (uint64_t) max + 1u; /* max 0 => full range */
                return (uint16_t) (ticks % period);
            }
            return dev->regs[off / 2];
        }
    }

    /* The request and status registers also reflect the pending timers. */
    if (off == I80186_ICU_REQ)
        return (uint16_t) (dev->regs[off / 2] | (i80186_tmr_pending(dev) ? 0x0001u : 0x0000u));
    if (off == I80186_ICU_STATUS)
        return (uint16_t) (dev->regs[off / 2] | i80186_tmr_pending(dev));

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

    /* Latch timer enable on control-register writes; a count-register write
       reloads the free-running value from that instant. */
    for (int i = 0; i < 3; i++) {
        if (off == I80186_TMR_CON(i)) {
            dev->tmr_start[i]  = tsc;
            dev->tmr_period[i] = (val & 0x8000u) ? 1u : 0u; /* bit15 = enable */
            dev->tmr_ack[i]    = 0;
        } else if (off == I80186_TMR_COUNT(i))
            dev->tmr_start[i] = tsc;
    }

    /* The ROM acknowledges a timer by writing the status register, which has
       to make the interrupt request go away until the next overflow. */
    if (off == I80186_ICU_STATUS) {
        dev->regs[off / 2] = val & 0x0007u;
        for (int i = 0; i < 3; i++)
            dev->tmr_ack[i] = i80186_tmr_overflows(dev, i);
    }
}

static uint16_t
i80186_inw(uint16_t port, void *priv)
{
    return i80186_read_word((i80186_t *) priv, (uint16_t) ((port - ((i80186_t *) priv)->base) & 0xFEu));
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
    for (int i = 0; i < 3; i++) {
        dev->tmr_start[i]  = 0;
        dev->tmr_period[i] = 0;
        dev->tmr_ack[i]    = 0;
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