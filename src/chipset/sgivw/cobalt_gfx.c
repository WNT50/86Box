/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Cobalt Graphics Engine device emulation.
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
#include <assert.h>
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

#ifdef ENABLE_COBALT_GFX_LOG
int co_gfx_do_log = ENABLE_COBALT_GFX_LOG;

static void
cobalt_gfx_log(const char *fmt, ...)
{
    va_list ap;

    if (co_gfx_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define cobalt_gfx_log(fmt, ...)
#endif

static void
cobalt_gfx_mmio_write8(uint32_t addr, uint8_t val, void* priv)
{
    cobalt_gfx_log("CO: GFX [W08] [%X] <-- %X\n", addr, val);
}

static void
cobalt_gfx_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    cobalt_gfx_log("CO: GFX [W16] [%X] <-- %X\n", addr, val);
}

static void
cobalt_gfx_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    cobalt_gfx_log("CO: GFX [W32] [%X] <-- %X\n", addr, val);
}

static uint8_t
cobalt_gfx_mmio_read8(uint32_t addr, void* priv)
{
    uint8_t ret;

    ret = 0;

    cobalt_gfx_log("CO: GFX [R8] [%X] --> %X\n", addr, ret);
    return ret;
}

static uint16_t
cobalt_gfx_mmio_read16(uint32_t addr, void* priv)
{
    uint16_t ret;

    ret = 0;

    cobalt_gfx_log("CO: GFX [R16] [%X] --> %X\n", addr, ret);
    return ret;
}

static uint32_t
cobalt_gfx_mmio_read32(uint32_t addr, void* priv)
{
    uint32_t ret;

    ret = 0;

    cobalt_gfx_log("CO: GFX [R32] [%X] --> %X\n", addr, ret);
    return ret;
}

static void
cobalt_gfx_int_status_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    co_t *dev = priv;

    assert((addr & 0x3) == 0);

    cobalt_gfx_log("CO: GFX [W32] [%X] <-- %X\n", addr, val);

    dev->gfx.interrupt_status = val & 0x0000000F;
}

static uint32_t
cobalt_gfx_int_status_mmio_read32(uint32_t addr, void* priv)
{
    co_t *dev = priv;
    uint32_t ret;

    assert((addr & 0x3) == 0);

    ret = dev->gfx.interrupt_status;

    cobalt_gfx_log("CO: GFX [R32] [%X] --> %X\n", addr, ret);
    return ret;
}

void
cobalt_gfx_init(co_t *dev)
{
    mem_mapping_add(&dev->gfx_mapping,
                    0xC8000000,
                    0x02000000, // TODO Verify
                    cobalt_gfx_mmio_read8,
                    cobalt_gfx_mmio_read16,
                    cobalt_gfx_mmio_read32,
                    cobalt_gfx_mmio_write8,
                    cobalt_gfx_mmio_write16,
                    cobalt_gfx_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    mem_mapping_add(&dev->gfx_mapping_interrupt_status,
                    VW_CO_GFX_INT_STATUS_IO_BASE,
                    VW_CO_GFX_INT_STATUS_IO_SIZE,
                    NULL,
                    NULL,
                    cobalt_gfx_int_status_mmio_read32,
                    NULL,
                    NULL,
                    cobalt_gfx_int_status_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);
}
