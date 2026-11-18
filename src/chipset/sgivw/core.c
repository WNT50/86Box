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

/* CO_APIC_ID (cobalt.h): the Cobalt APIC answers its enable bit at 0FFCh. */
#define VW_CO_APIC_ID_REG 0x00000100

static uint32_t
cobalt_apic_id_readl(UNUSED(uint32_t addr), UNUSED(void *priv))
{
    return VW_CO_APIC_ID_REG;
}

static uint16_t
cobalt_apic_id_readw(uint32_t addr, UNUSED(void *priv))
{
    return (VW_CO_APIC_ID_REG >> ((addr & 2) * 8)) & 0xffff;
}

static uint8_t
cobalt_apic_id_readb(uint32_t addr, UNUSED(void *priv))
{
    return (VW_CO_APIC_ID_REG >> ((addr & 3) * 8)) & 0xff;
}

static mem_mapping_t cobalt_apic_id_mapping;

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

    mem_mapping_add(&cobalt_apic_id_mapping, VW_CO_APIC_IO_BASE + 0x00000ffc, 4,
                    cobalt_apic_id_readb, cobalt_apic_id_readw, cobalt_apic_id_readl,
                    NULL, NULL, NULL, NULL, MEM_MAPPING_EXTERNAL, NULL);

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
