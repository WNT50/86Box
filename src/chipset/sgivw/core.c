/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the Cobalt chipset.
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
#include <86box/nvr.h>

#include <86box/sgivw/cobalt.h>
#include <86box/sgivw/lithium.h>

extern const device_t arsenic_device;

static void
cobalt_core_close(void *priv)
{
    co_t *dev = (co_t *) priv;

    free(dev);
}

static void *
cobalt_core_init(UNUSED(const device_t *info))
{
    nvr_t *nvr;
    void  *ioapic;

    /* Register CMOS memory with additional bank. TODO: move this into the PIIX4 source code */
    nvr = device_add(&nvr_at_device);
    nvr_bank_set(0, 0, nvr);
    /* The PIIX4 extended bank lives in the second 128-byte half of NVRAM. */
    nvr_bank_set(1, 1, nvr);
    nvr_at_sec_handler(1, 0x72, nvr);

    device_add(&cobalt_device);
    device_add(&lithium_device);
    device_add(&arsenic_device);

    ioapic = device_add(&ioapic_device);
    if (ioapic != NULL)
        ioapic_set_mmio_base(ioapic, VW_CO_APIC_IO_BASE);

    return NULL;
}

const device_t cobalt_chipset_device = {
    .name          = "Cobalt Chipset",
    .internal_name = "cobalt",
    .flags         = 0,
    .local         = 0,
    .init          = cobalt_core_init,
    .close         = cobalt_core_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
