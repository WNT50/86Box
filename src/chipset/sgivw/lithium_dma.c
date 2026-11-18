/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Lithium I/O DMA engine emulation.
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

#ifdef ENABLE_LITHIUM_DMA_LOG
int li_dma_do_log = ENABLE_LITHIUM_DMA_LOG;

static void
lithium_dma_log(const char *fmt, ...)
{
    va_list ap;

    if (li_dma_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define lithium_dma_log(fmt, ...)
#endif

static void
lithium_dma_mmio_write8(uint32_t addr, uint8_t val, void* priv)
{
    lithium_dma_log("LI: DMA [W08] [%X] <-- %X\n", addr, val);
}

static void
lithium_dma_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    lithium_dma_log("LI: DMA [W16] [%X] <-- %X\n", addr, val);
}

static void
lithium_dma_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    lithium_dma_log("LI: DMA [W32] [%X] <-- %X\n", addr, val);
}

static uint8_t
lithium_dma_mmio_read8(uint32_t addr, void* priv)
{
    uint8_t ret;

    ret = 0;
    lithium_dma_log("LI: DMA [R08] [%X] --> %X\n", addr, ret);

    return ret;
}

static uint16_t
lithium_dma_mmio_read16(uint32_t addr, void* priv)
{
    uint16_t ret;

    ret = 0;
    lithium_dma_log("LI: DMA [W16] [%X] <-- %X\n", addr, ret);

    return ret;
}

static uint32_t
lithium_dma_mmio_read32(uint32_t addr, void* priv)
{
    uint32_t ret;

    ret = 0;
    lithium_dma_log("LI: DMA [R32] [%X] --> %X\n", addr, ret);

    return ret;
}

static void
lithium_dma_reset_hard(li_t *dev)
{
    dev->reset_tsc = tsc;
}

void
lithium_dma_init(li_t *dev)
{
    mem_mapping_add(&dev->dma_mappings[0],
                    VW_LI_DMA_1_IO_BASE,
                    VW_LI_DMA_1_IO_SIZE,
                    lithium_dma_mmio_read8,
                    lithium_dma_mmio_read16,
                    lithium_dma_mmio_read32,
                    lithium_dma_mmio_write8,
                    lithium_dma_mmio_write16,
                    lithium_dma_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    mem_mapping_add(&dev->dma_mappings[1],
                    VW_LI_DMA_2_IO_BASE,
                    VW_LI_DMA_2_IO_SIZE,
                    lithium_dma_mmio_read8,
                    lithium_dma_mmio_read16,
                    lithium_dma_mmio_read32,
                    lithium_dma_mmio_write8,
                    lithium_dma_mmio_write16,
                    lithium_dma_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    mem_mapping_add(&dev->dma_mappings[2],
                    VW_LI_DMA_3_IO_BASE,
                    VW_LI_DMA_3_IO_SIZE,
                    lithium_dma_mmio_read8,
                    lithium_dma_mmio_read16,
                    lithium_dma_mmio_read32,
                    lithium_dma_mmio_write8,
                    lithium_dma_mmio_write16,
                    lithium_dma_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    lithium_dma_reset_hard(dev);
}
