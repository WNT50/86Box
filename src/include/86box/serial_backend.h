/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Serial backend interface: the seam between a UART chip and
 *          whatever is on the other side of the port - a host chardev
 *          (file, pipe, stdio, loopback, ...) or another emulated device.
 *
 * Authors: see the 86Box credits.
 *
 *          This program is free software; you can redistribute it and/or
 *          modify it under the terms of the GNU General Public License
 *          as published by the Free Software Foundation; either version 2
 *          of the License, or (at your option) any later version.
 *
 *          This program is distributed in the hope that it will be useful,
 *          but WITHOUT ANY WARRANTY; without even the implied warranty of
 *          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *          GNU General Public License for more details.
 */
#ifndef EMU_SERIAL_BACKEND_H
#define EMU_SERIAL_BACKEND_H

#include <stdint.h>

#include <86box/device.h>
#include <86box/char.h>

/*
   A backend is what the chip talks to. Every method is optional: a backend
   that has no modem control lines leaves ->status NULL, one that cannot
   receive leaves ->read_byte NULL, and the chip falls back to what it did
   before this interface existed (no polling, no data).
 */
typedef struct serial_backend_s {
    int      (*read_byte)(void *priv);                 /* next byte, or -1 */
    void     (*write_byte)(void *priv, uint8_t val);   /* byte out of the port */
    uint32_t (*status)(void *priv);                    /* CTS/DSR/RI/DCD bits */
    void     (*control)(void *priv, uint32_t flags);   /* DTR/RTS/loopback */
    void     (*set_format)(void *priv, int baud, int data_bits, int parity, int stop_bits);

    /* Hooks an in-machine device asked for through serial_attach_ex*(). */
    void     (*set_lcr)(void *priv, uint8_t lcr);
    void     (*rcr)(void *priv);
    void     (*dtr)(void *priv, int on);
    void     (*transmit_period)(void *priv, double period);

    void      *priv;
} serial_backend_t;

/* Adapter for a host chardev; `index` is the COM port number (0-based). */
extern const serial_backend_t *serial_backend_from_chardev(char_port_t *port, int index);

/* Adapter for another emulated device (modem, serial mouse, tablet, ...). */
struct serial_s;
struct serial_device_s;
extern const serial_backend_t *serial_backend_from_device(struct serial_s *serial,
                                                          struct serial_device_s *sd,
                                                          int index);

#endif /*EMU_SERIAL_BACKEND_H*/
