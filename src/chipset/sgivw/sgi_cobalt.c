/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Cobalt I/O ASIC device emulation.
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 2024 Dmitry Borisov
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/plat_unused.h>
#include <86box/chipset.h>

#include <86box/sgivw/sgi_cobalt.h>

static uint8_t cobalt_io_latch[0x1000];

static uint8_t
cobalt_io_read8(uint16_t port, UNUSED(void *priv))
{
    if (port == 0x0fb2)
        return 0x00; /* Board/clock strap, bits 2-5 */

    return cobalt_io_latch[port & 0xfff];
}

static void
cobalt_io_write8(uint16_t port, uint8_t val, UNUSED(void *priv))
{
    cobalt_io_latch[port & 0xfff] = val;
}

static void
cobalt_close(void *priv)
{
    co_t *dev = (co_t *) priv;

    free(dev);
}

static void *
cobalt_init(UNUSED(const device_t *info))
{
    co_t *dev = (co_t*)calloc(1, sizeof(co_t));

    cobalt_cpu_init(dev);
    cobalt_gfx_init(dev);
    cobalt_mem_init(dev);

    io_sethandler(0x0505, 0x0002, cobalt_io_read8, NULL, NULL, cobalt_io_write8, NULL, NULL, dev);
    io_sethandler(0x0fb0, 0x0008, cobalt_io_read8, NULL, NULL, cobalt_io_write8, NULL, NULL, dev);

    return dev;
}

const device_t cobalt_device = {
    .name          = "Cobalt I/O ASIC",
    .internal_name = "co",
    .flags         = 0,
    .local         = 0,
    .init          = cobalt_init,
    .close         = cobalt_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
