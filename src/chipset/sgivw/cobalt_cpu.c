/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Cobalt CPU Controller device emulation.
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
#include <stdbool.h>
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
#include <86box/machine.h>

#include <86box/sgivw/cobalt.h>

#ifdef ENABLE_COBALT_CPU_LOG
int co_cpu_do_log = ENABLE_COBALT_CPU_LOG;

static void
cobalt_cpu_log(const char *fmt, ...)
{
    va_list ap;

    if (co_cpu_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define cobalt_cpu_log(fmt, ...)
#endif

static void
cobalt_cpu_reload_timer(co_t *dev)
{
    // TODO: Not accurate at all, actually this timer runs at 100 MHz (VW_COBALT_CLOCK_FREQ)
    timer_on_auto(&dev->cpu.countdown_timer, 244.0);
}

static void
cobalt_cpu_timer_control(co_t *dev, bool do_enable)
{
    if (do_enable)
        cobalt_cpu_reload_timer(dev);
    else
        timer_stop(&dev->cpu.countdown_timer);
}

static void
cobalt_cpu_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    co_t *dev = (co_t *) priv;
    uint32_t write_bits_mask;

    assert((addr & 0x3) == 0);

    addr &= VW_CO_CPU_IO_DECODE_MASK;

    cobalt_cpu_log("CO: CPU [W32] [%02X] <-- %X\n", addr, val);

    addr /= sizeof(uint32_t);
    switch (addr) {
        case VW_CO_CPU_REG_00:
        case VW_CO_CPU_REG_04:
        case VW_CO_CPU_REG_TIMER_AUTO_RELOAD:
            write_bits_mask = 0xFFFFFFFF;
            break;
        case VW_CO_CPU_REG_CTRL:
            write_bits_mask = 0x0000000F;
            break;
        case VW_CO_CPU_REG_18:
            write_bits_mask = 0xD0496BA0;
            break;
        case VW_CO_CPU_REG_28:
            write_bits_mask = 0x00000003;
            break;
        case VW_CO_CPU_REG_40:
            write_bits_mask = 0x0006FFFF;
            break;
        case VW_CO_CPU_REG_48:
            write_bits_mask = 0x7FFFFFE0;
            break;

        default:
            write_bits_mask = 0;
            break;
    }

    val &= write_bits_mask;
    dev->cpu.regs[addr] = val | (dev->cpu.regs[addr] & ~write_bits_mask);

    switch (addr) {
        case VW_CO_CPU_REG_CTRL:
            cobalt_cpu_timer_control(dev, !!(val & VW_CO_CPU_START_TIMER));
            break;

        default:
            break;
    }
}

static uint32_t
cobalt_cpu_mmio_read32(uint32_t addr, void* priv)
{
    co_t *dev = (co_t *) priv;
    uint32_t ret;

    assert((addr & 0x3) == 0);

    addr &= VW_CO_CPU_IO_DECODE_MASK;
    addr /= sizeof(uint32_t);

    switch (addr) {
        case VW_CO_CPU_REG_TIMER_VALUE:
            /* TODO */
            break;

        default:
            break;
    }

    ret = dev->cpu.regs[addr];

    cobalt_cpu_log("CO: CPU [R32] [%02X] --> %X\n", addr * sizeof(uint32_t), ret);
    return ret;
}

static void
cobalt_cpu_mmio_write8(uint32_t addr, uint8_t val, void* priv)
{
    assert(0);
}

static void
cobalt_cpu_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    assert(0);
}

static uint8_t
cobalt_cpu_mmio_read8(uint32_t addr, void* priv)
{
    assert(0);
    return 0;
}

static uint16_t
cobalt_cpu_mmio_read16(uint32_t addr, void* priv)
{
    assert(0);
    return 0;
}

static void
cobalt_cpu_timer_tick(void *priv)
{
    co_t *dev = (co_t *) priv;
    uint32_t val;

    /* Count down timer */
    val = dev->cpu.regs[VW_CO_CPU_REG_TIMER_VALUE];
    if (val == 0)
    {
        val = dev->cpu.regs[VW_CO_CPU_REG_TIMER_AUTO_RELOAD];

        // TODO: raise interrupt
    }
    else
    {
        val -= 1;
    }
    dev->cpu.regs[VW_CO_CPU_REG_TIMER_VALUE] = val;

    cobalt_cpu_reload_timer(dev);
}

static void
cobalt_cpu_reset_hard(co_t *dev)
{
    memset(dev->cpu.regs, 0, sizeof(dev->cpu.regs));

    if (IS_VW_540())
        dev->cpu.regs[VW_CO_CPU_REG_REVISION] = VW_CO_CPU_REV_A5;
    else
        dev->cpu.regs[VW_CO_CPU_REG_REVISION] = VW_CO_CPU_REV_A4;

    dev->cpu.regs[VW_CO_CPU_REG_CTRL] = 0x00000019;
}

void
cobalt_cpu_init(co_t *dev)
{
    mem_mapping_add(&dev->cpu_mapping,
                    VW_CO_CPU_IO_BASE,
                    VW_CO_CPU_IO_SIZE,
                    cobalt_cpu_mmio_read8,
                    cobalt_cpu_mmio_read16,
                    cobalt_cpu_mmio_read32,
                    cobalt_cpu_mmio_write8,
                    cobalt_cpu_mmio_write16,
                    cobalt_cpu_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    timer_add(&dev->cpu.countdown_timer, cobalt_cpu_timer_tick, dev, 0);

    cobalt_cpu_reset_hard(dev);
}
