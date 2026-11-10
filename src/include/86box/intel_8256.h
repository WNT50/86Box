/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Intel 8256AH MUART: declarations.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_INTEL_8256_H
#define EMU_INTEL_8256_H

#include <86box/timer.h>

typedef struct intel_8256_t {
    uint16_t base;
    uint32_t clock;

    uint8_t regs[16];

    uint8_t timer_load[5];
    uint8_t timer_val[5];
    int     timer_run[5];

    uint8_t pending;
    int     ack_level;
    int     irq_line;

    void (*irq_cb)(void *priv, int level);
    void *irq_priv;

    pc_timer_t timer;
    pc_timer_t ack_timer;
} intel_8256_t;

extern intel_8256_t *intel_8256_init(uint16_t base);
extern void          intel_8256_reset(intel_8256_t *dev);
extern uint8_t       intel_8256_read(uint16_t port, void *priv);
extern void          intel_8256_write(uint16_t port, uint8_t val, void *priv);
extern void          intel_8256_set_irq_handler(intel_8256_t *dev,
                                                void (*cb)(void *priv, int level),
                                                void *priv);
extern uint8_t       intel_8256_acknowledge(intel_8256_t *dev);
extern void          intel_8256_set_level(intel_8256_t *dev, int level);

#endif /* EMU_INTEL_8256_H */
