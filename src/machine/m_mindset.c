/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Mindset Personal Computer (1984): 80186 at 6 MHz (12 MHz crystal,
 *          divided by two), 256 KB of RAM, 32 KB of display RAM at B8000h
 *          that the GCP blitter and the CPU both write, a 32 KB BIOS made of
 *          two 16 KB EPROMs on a 16-bit bus, two 8042s (system and sound
 *          controller) and an 8749 keyboard scanner, a 8272A floppy
 *          controller, eight blitter blend modes and a stereo DAC.
 *
 *          Step 1 (M0): CPU, RAM, display RAM and the BIOS window, plus an
 *          optional trace (MIND_LOG=1) of every port the ROM touches and of
 *          where the CPU is every 20 ms.  The GCP, the three MCUs, the FDC
 *          and the modules are not modelled yet; this build exists to find
 *          out what the ROM expects before any of that is written.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <86box/86box.h>
#include <86box/machine.h>
#include <86box/mem.h>
#include <86box/io.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/plat.h>
#include <86box/timer.h>
#include <86box/i8018x.h>

#include "../cpu/cpu.h"

#define MIND_BIOS_EVEN    "roms/machines/mindset/mindset_u60_1.7_lo.bin"
#define MIND_BIOS_ODD     "roms/machines/mindset/mindset_u59_1.7_hi.bin"
#define MIND_BIOS_ADDR    0x000f8000
#define MIND_BIOS_SIZE    0x00008000

#define MIND_RAM_SIZE     0x00040000  /* 256 KB, the only RAM the board has */
#define MIND_VRAM_BASE    0x000b8000
#define MIND_VRAM_SIZE    0x00008000

/* Everything from 8000h up is the real I/O space; 0000h-7FFFh is the module
   bus, where the system MCU watches every access the CPU makes. */
#define MIND_TRAP_BASE    0x0000
#define MIND_TRAP_SIZE    0x8000
#define MIND_IO_BASE      0x8040
#define MIND_IO_SIZE      0x0300      /* 8040-833F: FDC, MCUs, GCP, display */

typedef struct {
    uint8_t      vram[MIND_VRAM_SIZE];
    mem_mapping_t vram_map;
    int          log;
    pc_timer_t   sample_timer;
} mindset_t;

static mindset_t mindset;

/* MIND_SAMPLE=<us>: how often the sampler prints CS:IP (default 20 ms). */
static int mind_interval = 20000;

static void
mind_trace(const char *dir, uint16_t port, uint32_t val, int width)
{
    if (!mindset.log)
        return;

    pclog("[MIND] %s%-5s %04X = %0*X  @%04X:%04X\n", dir,
          (width == 1) ? "8-bit" : ((width == 2) ? "16-bit" : "32-bit"),
          port, width * 2, val, cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
}

/* Display RAM: written by the CPU and by the GCP, read back by both. */
static uint8_t
mind_vram_read(uint32_t addr, void *priv)
{
    return mindset.vram[addr & (MIND_VRAM_SIZE - 1)];
}

static uint16_t
mind_vram_readw(uint32_t addr, void *priv)
{
    addr &= (MIND_VRAM_SIZE - 2);
    return *(uint16_t *) &mindset.vram[addr];
}

static uint32_t
mind_vram_readl(uint32_t addr, void *priv)
{
    addr &= (MIND_VRAM_SIZE - 4);
    return *(uint32_t *) &mindset.vram[addr];
}

static void
mind_vram_write(uint32_t addr, uint8_t val, void *priv)
{
    mindset.vram[addr & (MIND_VRAM_SIZE - 1)] = val;
}

static void
mind_vram_writew(uint32_t addr, uint16_t val, void *priv)
{
    addr &= (MIND_VRAM_SIZE - 2);
    *(uint16_t *) &mindset.vram[addr] = val;
}

static void
mind_vram_writel(uint32_t addr, uint32_t val, void *priv)
{
    addr &= (MIND_VRAM_SIZE - 4);
    *(uint32_t *) &mindset.vram[addr] = val;
}

/* The module bus.  On the real board these cycles are serialised to the
   system MCU, which answers for whatever module sits in the slot. */
static uint8_t
mind_trap_read(uint16_t port, void *priv)
{
    mind_trace("r", port, 0xff, 1);
    return 0xff;
}

static uint16_t
mind_trap_readw(uint16_t port, void *priv)
{
    mind_trace("r", port, 0xffff, 2);
    return 0xffff;
}

static void
mind_trap_write(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);
}

static void
mind_trap_writew(uint16_t port, uint16_t val, void *priv)
{
    mind_trace("w", port, val, 2);
}

/* FDC, both 8042s, the GCP, the display registers and the module slots.
   Until the real devices exist this just records who asks for what. */
static uint8_t
mind_io_read(uint16_t port, void *priv)
{
    uint8_t ret = 0xff;

    switch (port) {
        case 0x8282: /* system MCU status: report idle, nothing pending */
            ret = 0x00;
            break;

        case 0x82a2: /* sound MCU status */
            ret = 0x00;
            break;

        default:
            break;
    }

    mind_trace("r", port, ret, 1);
    return ret;
}

static uint16_t
mind_io_readw(uint16_t port, void *priv)
{
    uint16_t ret = (uint16_t) (mind_io_read(port, priv) |
                               (mind_io_read(port + 1, priv) << 8));

    return ret;
}

static void
mind_io_write(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);
}

static void
mind_io_writew(uint16_t port, uint16_t val, void *priv)
{
    mind_trace("w", port, val, 2);
}

/* Sampler: without a display device the only way to tell "the CPU is still
   running the ROM" from "it wedged in a loop" is where it is over time. */
static void
mind_sample(void *priv)
{
    pclog("[MIND] pc=%04X:%04X\n", cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);

    timer_set_delay_u64(&mindset.sample_timer, mind_interval * TIMER_USEC);
}

int
machine_mindset_init(const machine_t *model)
{
    int ret;

    ret = bios_load_interleaved(MIND_BIOS_EVEN, MIND_BIOS_ODD, MIND_BIOS_ADDR,
                                MIND_BIOS_SIZE, 0);
    if (bios_only || !ret)
        return ret;

    memset(&mindset, 0x00, sizeof(mindset));
    mindset.log = (getenv("MIND_LOG") != NULL);
    if (getenv("MIND_SAMPLE") != NULL)
        mind_interval = atoi(getenv("MIND_SAMPLE"));

    pclog("[MIND] init: %d KB RAM, BIOS 32 KB at %05X, VRAM 32 KB at %05X\n",
          mem_size, MIND_BIOS_ADDR, MIND_VRAM_BASE);

    machine_common_init(model);

    /* The board has 256 KB and no upper memory: everything above it reads as
       an open bus.  The display RAM is a window inside the chip's 1 MB. */
    mem_set_mem_state_both(0x00000000, MIND_RAM_SIZE,
                           MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);
    mem_mapping_add(&mindset.vram_map, MIND_VRAM_BASE, MIND_VRAM_SIZE,
                    mind_vram_read, mind_vram_readw, mind_vram_readl,
                    mind_vram_write, mind_vram_writew, mind_vram_writel,
                    NULL, MEM_MAPPING_INTERNAL, NULL);

    /* 80186 on-chip peripherals.  Neither DMA channel has a device yet, and
       the chip selects are stored but not decoded (intel_8018x.c:13). */
    {
        struct i8018x_pcb_t *pcb = device_add(&i8018x_pcb_device);

        i8018x_pcb_set_dma_channels(pcb, -1, -1);
    }

    io_sethandler(MIND_TRAP_BASE, MIND_TRAP_SIZE,
                  mind_trap_read, mind_trap_readw, NULL,
                  mind_trap_write, mind_trap_writew, NULL, NULL);
    io_sethandler(MIND_IO_BASE, MIND_IO_SIZE,
                  mind_io_read, mind_io_readw, NULL,
                  mind_io_write, mind_io_writew, NULL, NULL);

    timer_add(&mindset.sample_timer, mind_sample, NULL, 0);
    timer_set_delay_u64(&mindset.sample_timer, mind_interval * TIMER_USEC);

    return 1;
}

const device_t mindset_device = {
    .name          = "Mindset Personal Computer",
    .internal_name = "mindset",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
