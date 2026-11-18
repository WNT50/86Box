/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Lithium I/O Bus ASIC device emulation.
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

#include <86box/sgivw/lithium.h>

static void
lithium_close(void *priv)
{
    li_t *dev = (li_t *) priv;

    free(dev);
}

static void *
lithium_init(UNUSED(const device_t *info))
{
    li_t *dev = (li_t*)calloc(1, sizeof(li_t));

    device_add(&lithium_bridge_a_device);
    device_add(&lithium_bridge_b_device);
    device_add(&lithium_1394_device);

    lithium_dma_init(dev);

    return dev;
}

const device_t lithium_device = {
    .name          = "Lithium I/O Bus ASIC",
    .internal_name = "li",
    .flags         = 0,
    .local         = 0,
    .init          = lithium_init,
    .close         = lithium_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
