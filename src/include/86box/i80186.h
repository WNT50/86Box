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
 */
#ifndef EMU_I80186_H
#define EMU_I80186_H

#include <86box/device.h>

extern const device_t i80186_device;

/* [cloud] 3477-S: the IPL's disk service moves a sector with the on-chip DMA
   channel 0 (source = the floppy data port 0x3F5, destination = the buffer it
   then verifies) and never programs the board's 8237, so the board's floppy
   controller hands every byte over through these helpers instead.  They
   mirror dma_channel_write()/dma_channel_read(): the return value carries the
   terminal count in DMA_OVER. */
extern int i80186_dma0_ready(void);
extern int i80186_dma0_write(uint8_t val);
extern int i80186_dma0_read(void);

#endif /* EMU_I80186_H */
