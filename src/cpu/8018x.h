/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          8018x peripherals - the peripheral part of the 80186/80188;
 *          the CPU core itself is in 808x.c.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_8018X_H
#define EMU_8018X_H

#include <86box/device.h>

extern const device_t i8018x_pcb_device;

/* The 3477-S disk service drives the on-chip DMA channel 0 instead of the
   board's 8237, so the floppy controller hands each byte over here.  The
   return value carries the terminal count in DMA_OVER. */
extern int i8018x_pcb_dma0_ready(void);
extern int i8018x_pcb_dma0_write(uint8_t val);
extern int i8018x_pcb_dma0_read(void);

#endif /* EMU_8018X_H */
