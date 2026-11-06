/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          HP-HIL pointer device: the HP Mouse 46060A on the link.
 *
 *          The mouse answers IDD with the four frames hle_hp_46060b_device
 *          sends (MAME, hlemouse.cpp), and answers a poll with a Poll Record
 *          carrying the displacement it owes the System.  The host pointer is
 *          sampled on a 10 ms timer by the adapter and latched here; a record
 *          only goes out when a poll asks for it (TRM 45918A appendix E).
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
#include <86box/mem.h>
#include <86box/mouse.h>

#include "hp_hil.h"

static void
hp_hil_mouse_idd(hp_hil_t *dev, int i)
{
    hil_dev_t *d = &dev->dev[i];

    /* ID byte plus the Describe Record: header (8-bit relative X and Y axes),
       then the resolution in counts per meter (appendix C). */
    hp_hil_fifo_push(hp_hil_reply(d->addr, d->id));
    hp_hil_fifo_push(hp_hil_reply(d->addr, HIL_MOUSE_DESCR));
    hp_hil_fifo_push(hp_hil_reply(d->addr, HIL_MOUSE_RES_LO));
    hp_hil_fifo_push(hp_hil_reply(d->addr, HIL_MOUSE_RES_HI));
}

static int
hp_hil_mouse_poll(hp_hil_t *dev, int i)
{
    hil_dev_t *d = &dev->dev[i];

    /* Poll Record (TRM appendix E): header bits 1-0 say which axes follow and
       bit 2 says the device is ready to hand data over.  A device with
       nothing to report answers with no record at all. */
    if (!dev->dx && !dev->dy)
        return 0;

    /* Record layout the ROM parses (F000:8FBA, axes read at F000:9059):
          header   bits 1-0 = which axes are present
          X        one byte when the device's describe record has no 16-bit flag
          Y        one byte, same rule
       This describe record does not set bit 20h, so both axes are 8 bit. */
    hp_hil_fifo_push(hp_hil_reply(d->addr, HIL_POL_AXES_XY));
    hp_hil_fifo_push(hp_hil_reply(d->addr, (uint8_t) dev->dx));
    hp_hil_fifo_push(hp_hil_reply(d->addr, (uint8_t) dev->dy));
    dev->dx = 0;
    dev->dy = 0;

    /* A device that reports data ends Auto Polling: the MLC raises the
       interrupt and clears APE, and the System sets it again in its ISR
       (TRM 45918A 4-19). */
    dev->w3 &= (uint8_t) ~HIL_W3_APE;
    return 1;
}

/* 10 ms housekeeping: take the host delta and, when the link is ready for it,
   strobe a poll so the record has a command to ride on. */
void
hp_hil_mouse_tick(hp_hil_t *dev, int i)
{
    int dx = 0;
    int dy = 0;
    int ox = 0;
    int oy = 0;
    int nx, ny;

    (void) i;

    /* Take the host delta every tick, held back until the link can carry it.
       HP-HIL reports +Y upwards while the host moves +Y downwards, so the
       delta is negated here (MAME's hp_46060b does the same:
       mouse_y_delta += -delta). */
    mouse_subtract_coords(&dx, &dy, &ox, &oy, -128, 127, 0, 0);
    dy = -dy;
    if (dx || dy) {
        nx = (int) dev->dx + dx;
        ny = (int) dev->dy + dy;
        dev->dx = (int8_t) ((nx > 127) ? 127 : ((nx < -128) ? -128 : nx));
        dev->dy = (int8_t) ((ny > 127) ? 127 : ((ny < -128) ? -128 : ny));
        dev->reports++;
    }

    /* The driver configures the pointer, polls it once and then waits for the
       link to raise an interrupt when there is something to report; it leaves
       the device in loop-back between polls (ELB after EPT).  Gating the
       sampler on the link being out of loop-back therefore stopped it dead:
       all that matters is that the pointer has an address. */
    if (dev->dev[HIL_PTR].addr != 0) {
        /* Auto Poll: every AP strobe makes the MLC issue a POL and the answer
           carries whatever the device has.  There is no AP source in the
           model and this ROM never sets W3.APE, so the pointer sampler doubles
           as the strobe.  AP is a 60 Hz line from the 8041 (Vectra TRM Vol 1,
           section 4): three polls in every five 10 ms ticks.

           Two things make a strobe worth issuing: the driver is parked waiting
           for a frame (HPHIL.SYS spins on DS:0x10 bit 0 at F000:92AB), or the
           device has motion to report and nothing has polled it yet (OS/2's
           HIL.SYS stops polling once the link is up and waits for the
           interrupt instead). */
        static int ap_div;
        static const uint8_t ap_pattern[5] = { 1, 0, 1, 1, 0 };

        uint32_t ctx = hp_hil_driver_ctx();

        if (((dev->dx || dev->dy) ||
             ((ctx != 0) && (mem_readw_phys(ctx + 0x10) & 0x0001))) &&
            (dev->fifo_count == 0) && ap_pattern[ap_div++ % 5]) {
            /* Poll this device directly: a broadcast poll is answered by the
               first device in the chain, and OS/2's HIL.SYS keeps its own
               idea of which address the mouse got. */
            dev->w1 = (uint8_t) (0x08 | (dev->dev[HIL_PTR].addr & 0x07));
            hp_hil_transmit(HIL_POL);
        }
        /* Motion is not pushed here: the record is the device's answer to a
           poll (the HIL_POL case in the chain walk), and a record that arrives
           without one makes the driver read a frame it never asked for.  The
           deltas simply stay latched until the next POL asks for them. */
        if (dev->dx || dev->dy) {
            hp_hil_trace("HIL: host pointer %d,%d latched\n", dev->dx, dev->dy);
        }
    }
}

const hp_hil_dev_ops_t hp_hil_mouse_ops = {
    .name = "mouse",
    .idd  = hp_hil_mouse_idd,
    .poll = hp_hil_mouse_poll,
    .tick = hp_hil_mouse_tick,
};
