/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Cobalt Memory Controller device emulation.
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
#include <86box/smram.h>
#include <86box/plat_unused.h>
#include <86box/chipset.h>

#include <86box/sgivw/cobalt.h>

#ifdef ENABLE_COBALT_MEM_LOG
int co_mem_do_log = ENABLE_COBALT_MEM_LOG;

static void
cobalt_mem_log(const char *fmt, ...)
{
    va_list ap;

    if (co_mem_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define cobalt_mem_log(fmt, ...)
#endif

static void
cobalt_mem_clear_memory_error(co_t *dev)
{
    dev->mem.regs[VW_CO_MEM_REG_ERROR_STATUS] = 0;

    // TODO: clear interrupt
}

static void
cobalt_mem_dispatch_memory_error(co_t *dev)
{
    dev->mem.regs[VW_CO_MEM_REG_ERROR_STATUS] = 0x00804000;

    // TODO: raise interrupt
}

static void
cobalt_mem_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    co_t *dev = priv;
    uint32_t write_bits_mask;

    assert((addr & 0x3) == 0);

    addr &= VW_CO_MEM_IO_DECODE_MASK;

    cobalt_mem_log("CO: MEM [W32] [%02X] <-- %X\n", addr, val);

    addr /= sizeof(uint32_t);
    switch (addr) {
        case VW_CO_MEM_REG_RAM_BUS_CTRL:
            write_bits_mask = 0x00000003;
            break;
        case VW_CO_MEM_REG_TIMER_AUTO_RELOAD:
        case VW_CO_MEM_REG_DIMM_STATUS_CTRL:
            write_bits_mask = 0x0000FFFF;
            break;
        case VW_CO_MEM_REG_40:
            write_bits_mask = 0x000000FF;
            break;
        case VW_CO_MEM_REG_BANK_A_128_CTRL:
        case VW_CO_MEM_REG_BANK_A_256_CTRL:
        case VW_CO_MEM_REG_BANK_A_384_CTRL:
        case VW_CO_MEM_REG_BANK_A_512_CTRL:
        case VW_CO_MEM_REG_BANK_B_128_CTRL:
        case VW_CO_MEM_REG_BANK_B_256_CTRL:
        case VW_CO_MEM_REG_BANK_B_384_CTRL:
        case VW_CO_MEM_REG_BANK_B_512_CTRL:
        case VW_CO_MEM_REG_BANK_C_128_CTRL:
        case VW_CO_MEM_REG_BANK_C_256_CTRL:
        case VW_CO_MEM_REG_BANK_C_384_CTRL:
        case VW_CO_MEM_REG_BANK_C_512_CTRL:
        case VW_CO_MEM_REG_BANK_D_128_CTRL:
        case VW_CO_MEM_REG_BANK_D_256_CTRL:
        case VW_CO_MEM_REG_BANK_D_384_CTRL:
        case VW_CO_MEM_REG_BANK_D_512_CTRL:
            write_bits_mask = 0x00000007;
            break;

        default:
            write_bits_mask = 0;
            break;
    }

    val &= write_bits_mask;
    dev->mem.regs[addr] = val | (dev->mem.regs[addr] & ~write_bits_mask);

    switch (addr) {
        case VW_CO_MEM_REG_ERROR_STATUS:
            if (val == VW_CO_MEM_STATUS_CLEAR)
                cobalt_mem_clear_memory_error(dev);
            break;

        default:
            break;
    }
}

static uint32_t
cobalt_mem_mmio_read32(uint32_t addr, void* priv)
{
    co_t *dev = priv;
    uint32_t ret;

    assert((addr & 0x3) == 0);

    addr &= VW_CO_MEM_IO_DECODE_MASK;
    addr /= sizeof(uint32_t);

    ret = dev->mem.regs[addr];

    cobalt_mem_log("CO: MEM [R32] [%02X] --> %X\n", addr * sizeof(uint32_t), ret);
    return ret;
}

static void
cobalt_mem_mmio_write8(uint32_t addr, uint8_t val, void* priv)
{
    assert(0);
}

static void
cobalt_mem_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    assert(0);
}

static uint8_t
cobalt_mem_mmio_read8(uint32_t addr, void* priv)
{
    assert(0);
    return 0;
}

static uint16_t
cobalt_mem_mmio_read16(uint32_t addr, void* priv)
{
    assert(0);
    return 0;
}

static void
cobalt_mem_reload_timer(co_t *dev)
{
    // TODO: Not accurate at all, actually the memory timer runs at 100 MHz (VW_COBALT_CLOCK_FREQ)
    timer_on_auto(&dev->mem.countdown_timer, 244.0);
}

static void
cobalt_mem_timer_tick(void *priv)
{
    co_t *dev = priv;
    uint32_t val;

    /* Count down timer */
    val = dev->mem.regs[VW_CO_MEM_REG_TIMER_VALUE];
    if (val == 0)
        val = dev->mem.regs[VW_CO_MEM_REG_TIMER_AUTO_RELOAD];
    else
        val -= 1;
    dev->mem.regs[VW_CO_MEM_REG_TIMER_VALUE] = val;

    cobalt_mem_reload_timer(dev);
}

static void
cobalt_mem_reset_hard(co_t *dev)
{
    memset(dev->mem.regs, 0, sizeof(dev->mem.regs));

    dev->mem.regs[VW_CO_MEM_REG_TIMER_AUTO_RELOAD] = 0x00000A8C;
    dev->mem.regs[VW_CO_MEM_REG_DIMM_STATUS_CTRL] = 0x00016411;
    dev->mem.regs[VW_CO_MEM_REG_38] = 0x30303030;
}

void
cobalt_mem_init(co_t *dev)
{
    mem_mapping_add(&dev->mem_mapping,
                    VW_CO_MEM_IO_BASE,
                    VW_CO_MEM_IO_SIZE,
                    cobalt_mem_mmio_read8,
                    cobalt_mem_mmio_read16,
                    cobalt_mem_mmio_read32,
                    cobalt_mem_mmio_write8,
                    cobalt_mem_mmio_write16,
                    cobalt_mem_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    timer_add(&dev->mem.countdown_timer, cobalt_mem_timer_tick, dev, 0);

    cobalt_mem_reset_hard(dev);
    cobalt_mem_reload_timer(dev);
}
