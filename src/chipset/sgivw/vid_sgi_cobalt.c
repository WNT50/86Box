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
          (base == VW_CO_GFX_INT_STATUS_IO_BASE) ? 'I' : 'G',
          addr - base, val, what, (uint32_t) cpu_state.pc);
}

static void
co_trace_read(const char *what, uint32_t base, uint32_t addr, uint32_t val)
{
    if (co_trace_reads++ >= CO_TRACE_READS)
        return;

    pclog("CO%c+%06X -> %08X  (%s pc=%08X)\n",
          (base == VW_CO_GFX_INT_STATUS_IO_BASE) ? 'I' : 'G',
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
        co_t *dev = priv;

        *(uint32_t *) &co_window[addr - 0xC8000000] = val;

        /* The driver arms its event mask here; that is the point after which
           it expects the engine to speak. */
        if (((addr - 0xC8000000) == 0x10020) && (val != 0))
            dev->engine_armed = 1;
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

    /* The driver reads the register and writes back the complement, so the
       bits are write-one-to-clear, not a plain latch. */
    dev->gfx.interrupt_status &= ~val;
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

/* Temporary: is anything being drawn?  Mapping the two frame buffer regions
   costs too much, so sample a spread of words once a second and report when
   the sum changes.  The span passed in is the window that matters (text and
   the top of a desktop live at the start of the buffer), not the whole
   region. */
#define CO_FB_BASE  0x02000000
#define CO_FB_SIZE  0x02000000
#define CO_POOL_BASE 0x00A00000
#define CO_POOL_SIZE 0x01600000
#define CO_SAMPLES  64

static uint32_t co_fb_sum, co_pool_sum;

static uint32_t
co_sample(uint32_t base, uint32_t span)
{
    uint32_t sum = 0;
    uint32_t i;

    for (i = 0; i < CO_SAMPLES; i++)
        sum ^= mem_readl_phys(base + (i * (span / 4 / CO_SAMPLES)));

    return sum;
}

/* Temporary: where is the CPU?  A dead screen can mean an idle kernel, a
   text-mode setup that has nowhere to draw, or a driver that never gets a
   call; the sampled program counter tells them apart. */
/* The driver enables the engine's event mask and then waits: the queue stays
   empty until the engine reports something.  Pulse the RE input with the
   status bit its handler looks for, so the wait has an answer. */
static void
co_re_drop(void *priv)
{
    ioapic_intin_irq(VW_CO_APIC_RE, 0, 0);
}

static void
co_re_pulse(void *priv)
{
    co_t *dev = priv;

    if (!dev->engine_armed)
        return;

    *(uint32_t *) &co_window[0x10018] = 0x40000000;
    ioapic_intin_irq(VW_CO_APIC_RE, 0, 1);

    timer_set_delay_u64(&dev->re_low_timer, TIMER_USEC * 30);
    timer_set_delay_u64(&dev->re_timer, TIMER_USEC * 100000);
}

static void
co_sample_pc(void *priv)
{
    static int n;
    co_t      *dev = priv;

    pclog("CO: PC n=%d lin=%08X cs=%04X\n", n++,
          (uint32_t) cpu_state.pc, (uint16_t) CS);

    timer_set_delay_u64(&dev->pc_timer, TIMER_USEC * 2000000);
}

/* The display's vertical retrace reaches the driver as an interrupt on the
   APIC's AS input (CO_APIC_AS).  The driver connects it in SGCOMP's start
   routine and uses it to pace its event loop, so without it the engine
   appears dead even when nothing is wrong with the command path. */
static void
co_vblank_drop(void *priv)
{
    ioapic_intin_irq(VW_CO_APIC_AS, 0, 0);
}

static void
co_vblank(void *priv)
{
    co_t     *dev = priv;
    static unsigned frame;

    /* Once a second, look at a spread of words in both frame buffer regions:
       a change means somebody drew something. */
    if ((++frame % 60) == 0) {
        uint32_t f = co_sample(CO_FB_BASE, 0x10000);
        uint32_t p = co_sample(CO_POOL_BASE, 0x10000);

        if (f != co_fb_sum) {
            pclog("CO: FB changed %08X -> %08X\n", co_fb_sum, f);
            co_fb_sum = f;
        }
        if (p != co_pool_sum) {
            pclog("CO: POOL changed %08X -> %08X\n", co_pool_sum, p);
            co_pool_sum = p;
        }

        /* VGA text and graphics apertures: if NT 4 draws its setup screen the
           ordinary way, it writes here and nowhere else. */
        {
            static uint32_t text_sum, gfx_sum;
            uint32_t t = co_sample(0x000B8000, 0x00001000);
            uint32_t g = co_sample(0x000A0000, 0x00004000);

            if (t != text_sum) {
                pclog("CO: TEXT changed %08X -> %08X\n", text_sum, t);
                text_sum = t;
            }
            if (g != gfx_sum) {
                pclog("CO: VGA changed %08X -> %08X\n", gfx_sum, g);
                gfx_sum = g;
            }
        }
    }

    dev->gfx.interrupt_status |= 0x02;
    *(uint32_t *) &co_window[0x10018] = 0x40000000;
    ioapic_intin_irq(VW_CO_APIC_AS, 0, 1);

    timer_set_delay_u64(&dev->vblank_low_timer, TIMER_USEC * 30);
    timer_set_delay_u64(&dev->vblank_timer, TIMER_USEC * 16666);
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

    timer_add(&dev->pc_timer, co_sample_pc, dev, 1);
    timer_set_delay_u64(&dev->pc_timer, TIMER_USEC * 2000000);

    timer_add(&dev->re_timer, co_re_pulse, dev, 1);
    timer_add(&dev->re_low_timer, co_re_drop, dev, 1);
    timer_set_delay_u64(&dev->re_timer, TIMER_USEC * 100000);

    timer_add(&dev->vblank_timer, co_vblank, dev, 1);
    timer_add(&dev->vblank_low_timer, co_vblank_drop, dev, 1);
    timer_set_delay_u64(&dev->vblank_timer, TIMER_USEC * 16666);

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
