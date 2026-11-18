/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Cobalt I/O APIC device emulation.
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

#include <86box/sgivw/cobalt.h>

#ifdef ENABLE_COBALT_APIC_LOG
int co_apic_do_log = ENABLE_COBALT_APIC_LOG;

static void
cobalt_apic_log(const char *fmt, ...)
{
    va_list ap;

    if (co_apic_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define cobalt_apic_log(fmt, ...)
#endif

static void
cobalt_apic_mmio_write8(uint32_t addr, uint8_t val, void* priv)
{
    cobalt_apic_log("CO: APIC [W08] [%X] <-- %X\n", addr, val);
}

static void
cobalt_apic_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    cobalt_apic_log("CO: APIC [W16] [%X] <-- %X\n", addr, val);
}

static void
cobalt_apic_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    cobalt_apic_log("CO: APIC [W32] [%X] <-- %X\n", addr, val);
}

static uint8_t
cobalt_apic_mmio_read8(uint32_t addr, void* priv)
{
    uint8_t ret;

    ret = 0;

    cobalt_apic_log("CO: APIC [R8] [%X] --> %X\n", addr, ret);
    return ret;
}

static uint16_t
cobalt_apic_mmio_read16(uint32_t addr, void* priv)
{
    uint16_t ret;

    ret = 0;

    cobalt_apic_log("CO: APIC [R16] [%X] --> %X\n", addr, ret);
    return ret;
}

static uint32_t
cobalt_apic_mmio_read32(uint32_t addr, void* priv)
{
    uint32_t ret;

    ret = 0;

    cobalt_apic_log("CO: APIC [R32] [%X] --> %X\n", addr, ret);
    return ret;
}

static void
cobalt_apic_reset_hard(co_t *dev)
{
    uint32_t i;

    for (i = 0; i < VW_CO_APIC_REGS_SIZE / sizeof(uint32_t); i += sizeof(uint32_t) * 2) {
        dev->apic.regs[i] = 0;
        dev->apic.regs[i + 1] = VW_CO_APIC_IRQ_DISABLED;
    }

    dev->apic.regs[1022 / sizeof(uint32_t)] = 0;
    dev->apic.regs[1023 / sizeof(uint32_t)] = 0;
}

void
cobalt_apic_init(co_t *dev)
{
    mem_mapping_add(&dev->apic_mapping,
                    VW_CO_APIC_IO_BASE,
                    VW_CO_APIC_IO_SIZE,
                    cobalt_apic_mmio_read8,
                    cobalt_apic_mmio_read16,
                    cobalt_apic_mmio_read32,
                    cobalt_apic_mmio_write8,
                    cobalt_apic_mmio_write16,
                    cobalt_apic_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    cobalt_apic_reset_hard(dev);
}
