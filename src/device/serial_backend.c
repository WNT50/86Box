/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Serial backends: adapters that let a UART chip talk to
 *          something that is not a UART.
 *
 *          This file currently holds the host chardev adapter only; the
 *          in-machine device adapter follows in the next step.
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
#include <string.h>
#include <stdint.h>

#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/char.h>
#include <86box/serial.h>
#include <86box/serial_backend.h>

static int
chardev_read_byte(void *priv)
{
    char_port_t *port = (char_port_t *) priv;
    uint8_t      val  = 0;

    if (port->chardev.read(&val, 1, port->chardev.priv) > 0)
        return (int) val;

    return -1;
}

static void
chardev_write_byte(void *priv, uint8_t val)
{
    char_port_t *port = (char_port_t *) priv;

    port->chardev.write(&val, 1, port->chardev.priv);
}

static uint32_t
chardev_status(void *priv)
{
    char_port_t *port = (char_port_t *) priv;

    return port->chardev.status(port->chardev.priv);
}

static void
chardev_control(void *priv, uint32_t flags)
{
    char_port_t *port = (char_port_t *) priv;

    port->chardev.control(flags, port->chardev.priv);
}

static void
chardev_set_format(void *priv, int baud, int data_bits, int parity, int stop_bits)
{
    char_port_t *port = (char_port_t *) priv;

    port->com.baud      = baud;
    port->com.data_bits = data_bits;
    port->com.parity    = parity;
    port->com.stop_bits = stop_bits;

    if (port->chardev.port_config != NULL)
        port->chardev.port_config(port->chardev.priv);
}

static serial_backend_t chardev_backends[SERIAL_MAX];

/*
   The capabilities of a host chardev are per port and per backend: a pipe
   has no modem lines, a socket has no port_config. Copy the ones that exist
   into the adapter and leave the rest NULL, so the chip can keep testing
   the pointer instead of knowing which backend is behind it.
 */
const serial_backend_t *
serial_backend_from_chardev(char_port_t *port, int index)
{
    serial_backend_t *be;

    if ((port == NULL) || (index < 0) || (index >= SERIAL_MAX))
        return NULL;

    be = &chardev_backends[index];

    memset(be, 0, sizeof(*be));

    be->priv       = port;
    be->read_byte  = (port->chardev.read != NULL) ? chardev_read_byte : NULL;
    be->write_byte = (port->chardev.write != NULL) ? chardev_write_byte : NULL;
    be->status     = (port->chardev.status != NULL) ? chardev_status : NULL;
    be->control    = (port->chardev.control != NULL) ? chardev_control : NULL;
    be->set_format = chardev_set_format;

    return be;
}

/* ---- adapter for another emulated device ------------------------------- */

typedef struct {
    serial_t        *serial;
    serial_device_t *sd;
} device_backend_priv_t;

static void
device_write_byte(void *priv, uint8_t val)
{
    device_backend_priv_t *bp = (device_backend_priv_t *) priv;

    bp->sd->dev_write(bp->serial, bp->sd->priv, val);
}

static void
device_set_lcr(void *priv, uint8_t lcr)
{
    device_backend_priv_t *bp = (device_backend_priv_t *) priv;

    bp->sd->lcr_callback(bp->serial, bp->sd->priv, lcr);
}

static void
device_rcr(void *priv)
{
    device_backend_priv_t *bp = (device_backend_priv_t *) priv;

    bp->sd->rcr_callback(bp->serial, bp->sd->priv);
}

static void
device_dtr(void *priv, int on)
{
    device_backend_priv_t *bp = (device_backend_priv_t *) priv;

    bp->sd->dtr_callback(bp->serial, on, bp->sd->priv);
}

static void
device_transmit_period(void *priv, double period)
{
    device_backend_priv_t *bp = (device_backend_priv_t *) priv;

    bp->sd->transmit_period_callback(bp->serial, bp->sd->priv, period);
}

static device_backend_priv_t device_backend_privs[SERIAL_MAX];
static serial_backend_t      device_backends[SERIAL_MAX];

/*
   Unlike a chardev, an in-machine device pushes what it receives straight
   into the FIFO, so there is no read_byte and no status here - only the
   callbacks it asked for in serial_attach_ex*(). Receiving from the device
   keeps going through serial_write_fifo().
 */
const serial_backend_t *
serial_backend_from_device(serial_t *serial, serial_device_t *sd, int index)
{
    serial_backend_t      *be;
    device_backend_priv_t *bp;

    if ((serial == NULL) || (sd == NULL) || (index < 0) || (index >= SERIAL_MAX))
        return NULL;

    be = &device_backends[index];
    bp = &device_backend_privs[index];

    memset(be, 0, sizeof(*be));
    memset(bp, 0, sizeof(*bp));

    bp->serial = serial;
    bp->sd     = sd;

    be->priv            = bp;
    be->write_byte      = (sd->dev_write != NULL) ? device_write_byte : NULL;
    be->set_lcr         = (sd->lcr_callback != NULL) ? device_set_lcr : NULL;
    be->rcr             = (sd->rcr_callback != NULL) ? device_rcr : NULL;
    be->dtr             = (sd->dtr_callback != NULL) ? device_dtr : NULL;
    be->transmit_period = (sd->transmit_period_callback != NULL) ? device_transmit_period : NULL;

    return be;
}
