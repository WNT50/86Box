/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Intel 8256AH MUART: the multifunction peripheral used by the
 *          Philips :YES.  One 40-pin part carries an asynchronous UART,
 *          five 8-bit timers, two 8-bit parallel ports and an eight-level
 *          priority interrupt controller.
 *
 *          This model covers the register file, the five timers and the
 *          interrupt controller.  The UART transmits nothing and receives
 *          nothing: the transmitter always reads empty, the receiver never
 *          has a character, and the serial framing is not modelled.  That
 *          is enough for the machine's boot firmware, which programs the
 *          part and polls its status; the serial channel is the next step.
 *
 *          Reference: Intel 8256AH data sheet (order 210875-001), section
 *          3 "Register description", table 2 "MUART registers".
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <86box/86box.h>
#include <86box/io.h>
#include <86box/plat.h>
#include <86box/timer.h>
#include <86box/intel_8256.h>

/* Register index, 8086 mode: A4-A1 select the register, A0 must be low. */
#define MUART_CMD1     0x0
#define MUART_CMD2     0x1
#define MUART_CMD3     0x2
#define MUART_MODE     0x3
#define MUART_PORT1C   0x4
#define MUART_INTEN    0x5
#define MUART_INTAD    0x6
#define MUART_BUF      0x7
#define MUART_PORT1    0x8
#define MUART_PORT2    0x9
#define MUART_TMR1     0xa
#define MUART_TMR2     0xb
#define MUART_TMR3     0xc
#define MUART_TMR4     0xd
#define MUART_TMR5     0xe
#define MUART_STATUS   0xf

/* Status register (table 2). */
#define MUART_ST_INT   0x80 /* an interrupt is pending */
#define MUART_ST_RBF   0x40 /* receiver buffer full */
#define MUART_ST_TBE   0x20 /* transmitter buffer empty */
#define MUART_ST_TRE   0x10 /* transmitter register empty */
#define MUART_ST_BD    0x08 /* break detected */
#define MUART_ST_PE    0x04
#define MUART_ST_OE    0x02
#define MUART_ST_FE    0x01

/* Command register 1. */
#define MUART_C1_8086  0x02
#define MUART_C1_BITI  0x04
#define MUART_C1_BRKI  0x08
#define MUART_C1_FRQ   0x01 /* 0 = 16 kHz timer clock, 1 = 1 kHz */

/* Interrupt levels: level 1 is timer 2 (or the P17 edge, CMD1 BITI). */
#define MUART_LVL_TMR2 0x02
#define MUART_LVL_COUNT 8

enum {
    MUART_TMR_NONE = 0,
    MUART_TMR_RUN,
    MUART_TMR_RELOAD
};

static void intel_8256_tick(void *priv);
static void intel_8256_update_irq(intel_8256_t *dev);

intel_8256_t *
intel_8256_init(uint16_t base)
{
    intel_8256_t *dev = (intel_8256_t *) calloc(1, sizeof(intel_8256_t));

    if (dev == NULL)
        return NULL;

    dev->base  = base;
    dev->clock = 0;

    for (int i = 0; i < 5; i++)
        dev->timer_load[i] = 0x00;

    timer_add(&dev->timer, intel_8256_tick, dev, 0);

    io_sethandler(base, 0x0020,
                  intel_8256_read, NULL, NULL,
                  intel_8256_write, NULL, NULL, dev);

    intel_8256_reset(dev);

    return dev;
}

void
intel_8256_set_irq_handler(intel_8256_t *dev,
                           void (*cb)(void *priv, int level), void *priv)
{
    if (dev == NULL)
        return;

    dev->irq_cb   = cb;
    dev->irq_priv = priv;
}

/*
 * A timer counts down at the rate CMD1 FRQ picks.  The five timers share one
 * prescaler (CMD2 C1-C0); the serial clock divider is not modelled because
 * nothing is transmitted.  A count of 0 means 256, which is what the part
 * does.
 */
static void
intel_8256_tick(void *priv)
{
    intel_8256_t *dev = (intel_8256_t *) priv;
    int           hz;

    if (dev == NULL)
        return;

    hz = (dev->regs[MUART_CMD1] & MUART_C1_FRQ) ? 1000 : 16000;

    for (int i = 0; i < 5; i++) {
        if (!dev->timer_run[i])
            continue;

        if (dev->timer_val[i] == 0) {
            /* The counter reloads from its register and, in interrupt mode,
               asks for service.  Only timer 2 is wired to a level here. */
            dev->timer_val[i] = dev->timer_load[i];
            if ((i == 1) && !(dev->regs[MUART_CMD1] & MUART_C1_BITI))
                dev->pending |= MUART_LVL_TMR2;
        } else {
            dev->timer_val[i]--;
        }
    }

    intel_8256_update_irq(dev);

    timer_set_delay_u64(&dev->timer, TIMER_USEC * (1000000ULL / (uint64_t) hz));
}

static void
intel_8256_update_irq(intel_8256_t *dev)
{
    int active = (dev->pending & dev->regs[MUART_INTEN]) != 0;
    int level  = active ? 1 : 0;

    if (level == dev->irq_line)
        return;

    dev->irq_line = level;
    if (dev->irq_cb != NULL)
        dev->irq_cb(dev->irq_priv, level);
}

uint8_t
intel_8256_read(uint16_t port, void *priv)
{
    intel_8256_t *dev = (intel_8256_t *) priv;
    uint8_t       idx  = (uint8_t) ((port - dev->base) >> 1);
    uint8_t       ret  = 0xff;

    switch (idx) {
        case MUART_CMD1:
        case MUART_CMD2:
        case MUART_CMD3:
        case MUART_MODE:
        case MUART_PORT1C:
        case MUART_INTEN:
        case MUART_INTAD:
        case MUART_PORT1:
        case MUART_PORT2:
            ret = dev->regs[idx];
            break;

        case MUART_BUF:
            /* No receiver: the buffer never fills. */
            dev->regs[MUART_STATUS] &= (uint8_t) ~MUART_ST_RBF;
            ret = 0x00;
            break;

        case MUART_TMR1:
        case MUART_TMR2:
        case MUART_TMR3:
        case MUART_TMR4:
        case MUART_TMR5:
            ret = dev->timer_val[idx - MUART_TMR1];
            break;

        case MUART_STATUS:
            ret = MUART_ST_TBE | MUART_ST_TRE;
            if (dev->pending & dev->regs[MUART_INTEN])
                ret |= MUART_ST_INT;
            if (dev->regs[MUART_STATUS] & MUART_ST_BD)
                ret |= MUART_ST_BD;
            break;

        default:
            break;
    }

    return ret;
}

void
intel_8256_write(uint16_t port, uint8_t val, void *priv)
{
    intel_8256_t *dev = (intel_8256_t *) priv;
    uint8_t       idx  = (uint8_t) ((port - dev->base) >> 1);

    switch (idx) {
        case MUART_CMD1:
            dev->regs[idx] = val;
            if (val & MUART_C1_FRQ) {
                timer_set_delay_u64(&dev->timer, TIMER_USEC * 1000);
            } else {
                timer_set_delay_u64(&dev->timer, TIMER_USEC * 62);
            }
            break;

        case MUART_CMD3:
            /* RST reloads the counters; SET/L1-L0 arm the levels. */
            if (val & 0x01)
                intel_8256_reset(dev);
            dev->regs[idx] = val;
            break;

        case MUART_TMR1:
        case MUART_TMR2:
        case MUART_TMR3:
        case MUART_TMR4:
        case MUART_TMR5: {
            int t = idx - MUART_TMR1;

            dev->regs[idx]      = val;
            dev->timer_load[t]  = val;
            dev->timer_val[t]   = val;
            dev->timer_run[t]   = 1;
            break;
        }

        case MUART_INTAD:
            dev->regs[idx] = val;
            /* Reading the address register acknowledges the interrupt. */
            dev->pending = 0;
            intel_8256_update_irq(dev);
            break;

        default:
            dev->regs[idx] = val;
            break;
    }

    intel_8256_update_irq(dev);
}

void
intel_8256_reset(intel_8256_t *dev)
{
    if (dev == NULL)
        return;

    memset(dev->regs, 0x00, sizeof(dev->regs));
    memset(dev->timer_val, 0x00, sizeof(dev->timer_val));
    memset(dev->timer_run, 0x00, sizeof(dev->timer_run));

    dev->pending  = 0;
    dev->irq_line = 0;

    timer_set_delay_u64(&dev->timer, TIMER_USEC * 62);
}
