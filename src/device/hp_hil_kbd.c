/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          HP-HIL keypad device: the Vectra 46030A keyboard on the link.
 *
 *          The keyboard of this machine is mostly wired to the 8041 (ports
 *          0x60/0x64 and the 0x68-0x6A mailbox), so what it does on the HIL
 *          link is only the chain protocol: it takes an address on ACF,
 *          describes itself on IDD and never answers a poll with data.
 *          Key codes reach the System through the 8041, not through a Poll
 *          Record (TRM 45918A appendix B: 00..1Fh are keypads).
 */

#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include <86box/io.h>
#include <86box/device.h>

#include "hp_hil.h"

static void
hp_hil_kbd_idd(hp_hil_t *dev, int i)
{
    hil_dev_t *d = &dev->dev[i];

    /* ID byte, then a single describe byte: no positional information, no
       I/O descriptor (appendix C). */
    hp_hil_fifo_push(hp_hil_reply(d->addr, d->id));
    hp_hil_fifo_push(hp_hil_reply(d->addr, 0x00));
}

static int
hp_hil_kbd_poll(hp_hil_t *dev, int i)
{
    /* A keypad has nothing to report in a poll: the keys go through the 8041,
       and the link only sees the command echo. */
    (void) dev;
    (void) i;

    return 0;
}

const hp_hil_dev_ops_t hp_hil_kbd_ops = {
    .name = "keyboard",
    .idd  = hp_hil_kbd_idd,
    .poll = hp_hil_kbd_poll,
    .tick = NULL,
};
