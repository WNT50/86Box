/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          Emulation of the AMD Am7990 LANCE Ethernet controller in the
 *          16-bit byte-swapped bus mode used by the DEC VAXmate.
 *
 *          The chip itself is board independent: the base I/O address, the
 *          interrupt line and the station address come from a lance_board_t
 *          supplied by whoever adds the device (see device_add_linked()).
 *          Machine specific glue, such as the VAXmate's NICSR interface
 *          logic, lives in the board module next to the chip.
 *
 * Based on @(#)am79c90.cpp Patrick Mackinlay (MAME)
 *
 *          Portions Copyright (C) 2018-2025 Patrick Mackinlay
 *          Portions Copyright (C) 2018 The MAME Project
 *
 *          The MAME device was originally added by mooglyguy in 2018 and has
 *          been developed since by Patrick Mackinlay; it is distributed under
 *          the BSD-3-Clause license, whose notices are retained here.
 */
#ifndef NET_LANCE_H
#define NET_LANCE_H

/* Parameters handed to lance_device by the board that instantiates it. */
typedef struct lance_board_s {
    uint16_t base;      /* RDP base; RAP is base+2, aliases at base+4/+6 */
    int      irq;
    uint8_t  mac[6];    /* station address, DIGITAL OUI on the VAXmate */
} lance_board_t;

/* Generic Am7990. Use device_add_linked(&lance_device, &board). */
extern const device_t lance_device;

/* DEC interface logic (VAXmate NICSR) gates the interrupt line. */
extern void lance_set_irq_gate(void *priv, int on);
extern int  lance_get_irq_gate(void *priv);

/* VAXmate onboard controller: Am7990 at 0C60, IRQ10, NICSR at 0C68. */
extern const device_t vaxmate_lance_device;

#endif /*NET_LANCE_H*/

extern const device_t ne2100_device;
