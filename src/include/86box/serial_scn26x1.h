/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Signetics SCN2661/SCN2651 EPCI -- Enhanced Programmable
 *          Communications Interface (USART).  Also covers the pin and
 *          software compatible SCN68661 (CMOS) and the plain SCN2651 it
 *          is an enhanced version of.
 *
 *          Register model from the Signetics 1986 Microprocessor Products
 *          data book, "SCN2661/SCN68661 Enhanced Programmable
 *          Communications Interface", tables 4 to 8.
 *
 *
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_SERIAL_SCN26X1_H
#define EMU_SERIAL_SCN26X1_H

#include <86box/device.h>

typedef struct scn26x1_t scn26x1_t;

/* The chip needs four consecutive ports: +0 data, +1 status/SYN, +2 mode,
   +3 command.  INTR is a level line, so the machine passes a callback that
   drives whatever interrupt controller the board wires it to. */
extern scn26x1_t *scn26x1_init(uint16_t base, void (*irq)(void *priv, int level), void *irq_priv);
extern void       scn26x1_close(scn26x1_t *dev);
extern void       scn26x1_reset(scn26x1_t *dev);

/* Serial line side: pull RxD and the two modem inputs. */
extern void scn26x1_set_rx(scn26x1_t *dev, int state);
extern void scn26x1_set_dsr(scn26x1_t *dev, int state);
extern void scn26x1_set_dcd(scn26x1_t *dev, int state);

/* Device entry for the device list; config keys are "base" (hex) and "irq"
   (unused by this model, kept so a machine can declare one). */
extern const device_t scn26x1_device;

#endif /*EMU_SERIAL_SCN26X1_H*/
