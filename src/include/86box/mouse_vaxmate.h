/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          DEC VAXmate mouse: a three-button Digital mouse on the SCN2661
 *          USART that sits at I/O port 0x0C40 and interrupts on IRQ12.
 *
 *          The USART itself is modelled in device/serial_scn26x1.c; this
 *          file only holds the mouse protocol: the D/R/P/T/Z command set,
 *          the three-byte position report and the four-byte self-test
 *          report.  Protocol reference: EK-PC500-TD, chapter 2.11.
 *
 *
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_MOUSE_VAXMATE_H
#define EMU_MOUSE_VAXMATE_H

#include <86box/device.h>

extern const device_t vaxmate_mouse_device;

#endif /*EMU_MOUSE_VAXMATE_H*/
