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

#include <86box/sgivw/sgi_cobalt.h>

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

/* Temporary trace for the Cobalt command protocol: the driver's writes and
   reads are the only description of the engine we have.  The counters keep a
   long install run from filling the disk. */
#define CO_TRACE_WRITES 20000
#define CO_TRACE_READS  2000

static int co_trace_writes, co_trace_reads;

/* The engine keeps its command ring in its own memory, which the driver maps
   at 0x60000000.  Without it the ring address means nothing and the kick has
   nothing to read. */
#define CO_LMEM_BASE 0x60000000
#define CO_LMEM_SIZE 0x02000000

static uint8_t *co_lmem;

/* The engine's aperture is not just registers: the driver builds command
   tables in it and reads them back.  Keep what is written so a read returns
   the driver's own data instead of zeroes. */
#define CO_WINDOW_SIZE 0x20000

static uint8_t co_window[CO_WINDOW_SIZE];

static void
co_trace_write(const char *what, uint32_t base, uint32_t addr, uint32_t val)
{
    if (co_trace_writes++ >= CO_TRACE_WRITES)
        return;

    pclog("CO%c+%06X <- %08X  (%s pc=%08X)\n",
          (base == VW_CO_GFX_INT_STATUS_IO_BASE) ? 'I' : ((base == 0x60000000) ? 'L' : 'G'),
          addr - base, val, what, (uint32_t) cpu_state.pc);
}

static void
co_trace_read(const char *what, uint32_t base, uint32_t addr, uint32_t val)
{
    if (co_trace_reads++ >= CO_TRACE_READS)
        return;

    pclog("CO%c+%06X -> %08X  (%s pc=%08X)\n",
          (base == VW_CO_GFX_INT_STATUS_IO_BASE) ? 'I' : ((base == 0x60000000) ? 'L' : 'G'),
          addr - base, val, what, (uint32_t) cpu_state.pc);
}

static void
cobalt_gfx_mmio_write8(uint32_t addr, uint8_t val, void* priv)
{
    cobalt_gfx_log("CO: GFX [W08] [%X] <-- %X\n", addr, val);
    co_trace_write("w8", 0xC8000000, addr, val);

    if ((addr - 0xC8000000) < CO_WINDOW_SIZE)
        co_window[addr - 0xC8000000] = val;
}

static void
cobalt_gfx_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    cobalt_gfx_log("CO: GFX [W16] [%X] <-- %X\n", addr, val);
    co_trace_write("w16", 0xC8000000, addr, val);

    if ((addr - 0xC8000000) < CO_WINDOW_SIZE)
        *(uint16_t *) &co_window[addr - 0xC8000000] = val;
}

static void
cobalt_gfx_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    cobalt_gfx_log("CO: GFX [W32] [%X] <-- %X\n", addr, val);
    co_trace_write("w32", 0xC8000000, addr, val);

    if ((addr - 0xC8000000) < CO_WINDOW_SIZE) {
        uint32_t off = addr - 0xC8000000;

        *(uint32_t *) &co_window[off] = val;

        /* The driver starts the engine by pointing it at a command ring.  Show
           what the ring looks like from the guest's side, and claim the work
           finished so the driver moves on to the next stage. */
        if (off == 0x10020) {
            pclog("CO: KICK ring=%08X\n", val);

            if ((val >= CO_LMEM_BASE) && (val < (CO_LMEM_BASE + CO_LMEM_SIZE)) && co_lmem) {
                for (int i = 0; i < 64; i++)
                    pclog("CO:   RING[%02X] = %08X\n", i,
                          *(uint32_t *) &co_lmem[val - CO_LMEM_BASE + i * 4]);
            } else if (val < (uint32_t) mem_size) {
                for (int i = 0; i < 64; i++)
                    pclog("CO:   RING[%02X] = %08X\n", i, mem_readl_phys(val + i * 4));
            } else {
                pclog("CO: KICK ring outside RAM\n");
            }

            co_window[0x1001C] = 0x01;
        }
    }
}

static uint8_t
cobalt_gfx_mmio_read8(uint32_t addr, void* priv)
{
    uint8_t ret;

    ret = ((addr - 0xC8000000) < CO_WINDOW_SIZE) ? co_window[addr - 0xC8000000] : 0;

    cobalt_gfx_log("CO: GFX [R8] [%X] --> %X\n", addr, ret);
    co_trace_read("r8", 0xC8000000, addr, ret);
    return ret;
}

static uint16_t
cobalt_gfx_mmio_read16(uint32_t addr, void* priv)
{
    uint16_t ret;

    ret = ((addr - 0xC8000000) < CO_WINDOW_SIZE) ? *(uint16_t *) &co_window[addr - 0xC8000000] : 0;

    cobalt_gfx_log("CO: GFX [R16] [%X] --> %X\n", addr, ret);
    co_trace_read("r16", 0xC8000000, addr, ret);
    return ret;
}

static uint32_t
cobalt_gfx_mmio_read32(uint32_t addr, void* priv)
{
    uint32_t ret;

    ret = ((addr - 0xC8000000) < CO_WINDOW_SIZE) ? *(uint32_t *) &co_window[addr - 0xC8000000] : 0;

    if ((addr - 0xC8000000) == 0x1001C)
        ret |= 0x01;

    cobalt_gfx_log("CO: GFX [R32] [%X] --> %X\n", addr, ret);
    co_trace_read("r32", 0xC8000000, addr, ret);
    return ret;
}

static void
cobalt_gfx_int_status_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    co_t *dev = priv;

    assert((addr & 0x3) == 0);

    cobalt_gfx_log("CO: GFX [W32] [%X] <-- %X\n", addr, val);
    co_trace_write("wi32", VW_CO_GFX_INT_STATUS_IO_BASE, addr, val);

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
    co_trace_read("ri32", VW_CO_GFX_INT_STATUS_IO_BASE, addr, ret);
    return ret;
}

static uint8_t
co_lmem_read8(uint32_t addr, void *priv)
{
    return co_lmem[addr - CO_LMEM_BASE];
}

static uint16_t
co_lmem_read16(uint32_t addr, void *priv)
{
    return *(uint16_t *) &co_lmem[addr - CO_LMEM_BASE];
}

static uint32_t
co_lmem_read32(uint32_t addr, void *priv)
{
    return *(uint32_t *) &co_lmem[addr - CO_LMEM_BASE];
}

static void
co_lmem_write8(uint32_t addr, uint8_t val, void *priv)
{
    co_trace_write("lw8", CO_LMEM_BASE, addr, val);
    co_lmem[addr - CO_LMEM_BASE] = val;
}

static void
co_lmem_write16(uint32_t addr, uint16_t val, void *priv)
{
    co_trace_write("lw16", CO_LMEM_BASE, addr, val);
    *(uint16_t *) &co_lmem[addr - CO_LMEM_BASE] = val;
}

static void
co_lmem_write32(uint32_t addr, uint32_t val, void *priv)
{
    co_trace_write("lw32", CO_LMEM_BASE, addr, val);
    *(uint32_t *) &co_lmem[addr - CO_LMEM_BASE] = val;
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

    if (co_lmem == NULL)
        co_lmem = calloc(1, CO_LMEM_SIZE);

    if (co_lmem != NULL) {
        mem_mapping_add(&dev->lmem_mapping,
                        CO_LMEM_BASE,
                        CO_LMEM_SIZE,
                        co_lmem_read8,
                        co_lmem_read16,
                        co_lmem_read32,
                        co_lmem_write8,
                        co_lmem_write16,
                        co_lmem_write32,
                        NULL,
                        MEM_MAPPING_EXTERNAL,
                        dev);
    }

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
