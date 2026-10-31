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

/* Byte-level receive side, for peripherals that already speak a framed
   protocol (a mouse, for instance) instead of driving the RxD pin.  The
   caller must wait for scn26x1_rx_free() before pushing the next byte, the
   chip holds a single received character. */
extern void       scn26x1_rx_byte(scn26x1_t *dev, uint8_t val);
extern int        scn26x1_rx_free(scn26x1_t *dev);

/* Called when the CPU writes a character to the transmit register: that is
   the byte the peripheral is meant to receive. */
extern void scn26x1_set_tx_handler(scn26x1_t *dev, void (*cb)(void *priv, uint8_t val), void *priv);
/* Called whenever the CPU takes the received character, so a device with a
   queue behind the UART can hand over the next one without waiting for a
   timer tick. */
void scn26x1_set_rx_pull_handler(scn26x1_t *dev, void (*cb)(void *priv), void *priv);

/* Serial line side: pull RxD and the two modem inputs. */
extern void scn26x1_set_rx(scn26x1_t *dev, int state);
extern void scn26x1_set_dsr(scn26x1_t *dev, int state);
extern void scn26x1_set_dcd(scn26x1_t *dev, int state);

/* Device entry for the device list; config keys are "base" (hex) and "irq"
   (unused by this model, kept so a machine can declare one). */
extern const device_t scn26x1_device;

#endif /*EMU_SERIAL_SCN26X1_H*/
