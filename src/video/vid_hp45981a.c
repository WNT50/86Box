/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the HP 45981A Multi-Mode Video Adapter, the
 *          display adapter fitted to the HP Vectra 45945A.  The card is
 *          CGA compatible: it decodes the Motorola MC6845 CRTC at
 *          3D0-3D7 (index/data mirrored four times), plus the CGA mode,
 *          colour and status registers at 3D8-3DC, and adds an extended
 *          control register at 3DE.
 *
 *          Extended control register (3DE, write only, 6 bits):
 *
 *              bit 0   0 = 200 lines, 1 = 400 lines
 *              bit 1   underline enable
 *              bit 2   character set: 0 = standard, 1 = HP Roman-8
 *              bit 3   video memory disabled (card ignores CPU access)
 *              bit 4   0 = 16K address space, 1 = 32K address space
 *              bit 5   page select (graphics modes only)
 *
 *          Video memory is 32K, at B8000-BFFFF.  In the 16K setting the
 *          CPU sees the first 16K, wrapping inside the full 32K; text
 *          pages are selected through the CRTC start address, so that
 *          eight 80x25 pages (2000 words each) fill the whole 32K.
 *
 *          Reading 3DF returns the board ID register (41h).  Line 7 of the
 *          technical reference (table 7.6) gives the full
 *          SPU/display access matrix; the 400 line timing is not modelled
 *          yet, this file only implements the port surface and the 32K
 *          memory map.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free  Software  Foundation; either  version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is  distributed in the hope that it will be useful, but
 * WITHOUT   ANY  WARRANTY;  without  even   the  implied  warranty  of
 * MERCHANTABILITY  or FITNESS  FOR A PARTICULAR  PURPOSE. See  the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the:
 *
 *   Free Software Foundation, Inc.
 *   59 Temple Place - Suite 330
 *   Boston, MA  02111-1307
 *   USA.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/timer.h>
#include "cpu.h"
#include <86box/video.h>
#include <86box/vid_cga.h>
#include <86box/vid_cga_comp.h>
#include <86box/plat_unused.h>
#include <86box/pit.h>

#define HP45981A_VRAM_SIZE 0x8000
#define HP45981A_VRAM_MASK16 0x3fff
#define HP45981A_VRAM_MASK32 0x7fff

/* Character ROM, 256 characters x 16 rows.  The original HP part has never
   been dumped, so the card runs without an alternative character set until
   an image is placed here. */
#define HP45981A_FONT_PATH "roms/machines/hpvectra/45981a_chargen.bin"

#define HP45981A_BOARD_ID 0x41

enum {
    HP45981A_EXT_400LINE  = 0x01,
    HP45981A_EXT_UNDERLINE = 0x02,
    HP45981A_EXT_HPFONT   = 0x04,
    HP45981A_EXT_VRAMOFF  = 0x08,
    HP45981A_EXT_32K      = 0x10,
    HP45981A_EXT_PAGE     = 0x20
};

typedef struct hp45981a_t {
    mem_mapping_t mapping;

    cga_t cga;

    uint8_t vram[HP45981A_VRAM_SIZE];

    uint8_t ext_ctrl;  /* 3DE, write only */
    uint8_t monitor25k; /* W2 jumper: 0 = 15 kHz monitor, 1 = 25 kHz */
    uint8_t font_rom[0x1000];
    int     font_loaded;
} hp45981a_t;

static int hp45981a_trace = -1;

static video_timings_t timing_hp45981a = { .type = VIDEO_ISA, .write_b = 8, .write_w = 16, .write_l = 32, .read_b = 8, .read_w = 16, .read_l = 32 };

/* 25 kHz / 60 Hz 400 line timing: the master oscillator is 22.44 MHz and the
   character clock is eight dots, so a character time is 1/2.805 MHz. */
static uint64_t
hp45981a_clk400(void)
{
    return (uint64_t) ((cpuclock / (22440000.0 / 8.0)) * (double) (1ULL << 32));
}

/* Test aid: program the CRTC the way a 400 line aware application would, so
   the 16 row rendering path can be exercised from the host.  The vertical
   registers of a 200 line mode are not translated by this model (the card's
   translation ROM has never been dumped). */
static void
hp45981a_program_400(UNUSED(hp45981a_t *dev))
{
    static const uint8_t regs[16] = { 0x6f, 0x4f, 0x51, 0x0f, 0x19, 0x00, 0x19, 0x19,
                                      0x00, 0x0f, 0x0e, 0x0f, 0x00, 0x00, 0x00, 0x00 };
    int i;

    for (i = 0; i < 16; i++) {
        cga_out(0x3d4, (uint8_t) i, &dev->cga);
        cga_out(0x3d5, regs[i], &dev->cga);
    }
}

static void
hp45981a_set_ext_ctrl(hp45981a_t *dev, uint8_t val)
{
    const uint8_t old = dev->ext_ctrl;

    dev->ext_ctrl = val & 0x3f;

    dev->cga.vram_mask = (dev->ext_ctrl & HP45981A_EXT_32K) ? HP45981A_VRAM_MASK32 : HP45981A_VRAM_MASK16;

    if (dev->font_loaded && (dev->ext_ctrl & HP45981A_EXT_HPFONT))
        dev->cga.font16 = dev->font_rom;
    else
        dev->cga.font16 = NULL;

    dev->cga.clk = (dev->ext_ctrl & HP45981A_EXT_400LINE) ? hp45981a_clk400() : CGACONST;

    if (old ^ dev->ext_ctrl) {
        dev->cga.fullchange = monitors[dev->cga.monitor_used].mon_changeframecount;
        cga_recalctimings(&dev->cga);
    }
}

static void
hp45981a_out(uint16_t addr, uint8_t val, void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    if (hp45981a_trace > 0)
        pclog("[HP45981A] out %03X = %02X\n", addr, val);

    switch (addr) {
        case 0x03de:
            hp45981a_set_ext_ctrl(dev, val);
            return;

        case 0x03df:
            /* Not used by this card. */
            return;

        default:
            /* 3D0-3D7 (CRTC, mirrored four times), 3D8, 3D9 and 3DB/3DC are
               all handled by the CGA core. */
            cga_out(addr, val, &dev->cga);

            if ((addr == 0x03d8) && (dev->ext_ctrl & HP45981A_EXT_400LINE) &&
                (getenv("HP45981A_CRTC400") != NULL))
                hp45981a_program_400(dev);
            return;
    }
}

static uint8_t
hp45981a_in(uint16_t addr, void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    switch (addr) {
        case 0x03de:
            /* Write only on real hardware; return the latched value. */
            return dev->ext_ctrl;

        case 0x03df:
            /* Board ID register (read only): 41h identifies a 45981A. */
            return HP45981A_BOARD_ID;

        case 0x03da:
            /* Status: bit 4 reports the monitor type selected by the W2
               jumper (0 = 25 kHz, 1 = 15 kHz), bit 5 the colour adapter. */
            return cga_in(addr, &dev->cga) | (dev->monitor25k ? 0x00 : 0x10);

        default:
            return cga_in(addr, &dev->cga);
    }
}

static void
hp45981a_write(uint32_t addr, uint8_t val, void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    if (dev->ext_ctrl & HP45981A_EXT_VRAMOFF)
        return;

    /* The mask inside cga_write applies the 16K/32K wrap. */
    cga_write(addr, val, &dev->cga);
}

static uint8_t
hp45981a_read(uint32_t addr, void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    if (dev->ext_ctrl & HP45981A_EXT_VRAMOFF)
        return 0xff;

    return cga_read(addr, &dev->cga);
}

static void *
hp45981a_init(UNUSED(const device_t *info))
{
    hp45981a_t *dev = (hp45981a_t *) calloc(1, sizeof(hp45981a_t));

    video_inform(VIDEO_FLAG_TYPE_CGA, &timing_hp45981a);

    dev->cga.vram      = dev->vram;
    dev->cga.composite = 0;
    dev->cga.revision  = 1; /* new CGA composite profile, RGB output */

    cga_comp_init(dev->cga.revision);
    cga_init(&dev->cga);

    /* Reset state: 16K address space, standard character set. */
    dev->ext_ctrl      = 0x00;
    dev->cga.vram_mask = HP45981A_VRAM_MASK16;

    mem_mapping_add(&dev->mapping, 0xb8000, 0x08000,
                    hp45981a_read, NULL, NULL,
                    hp45981a_write, NULL, NULL,
                    NULL, MEM_MAPPING_EXTERNAL, dev);

    /* 3D0-3DF: the CRTC index/data pair is mirrored at 3D0-3D7 and is
       decoded by cga_out()/cga_in(). */
    io_sethandler(0x03d0, 0x0010, hp45981a_in, NULL, NULL, hp45981a_out, NULL, NULL, dev);

    overscan_x = overscan_y = 16;

    cga_palette = 0;
    cgapal_rebuild();
    update_cga16_color(dev->cga.cgamode, dev->cga.cgacol);

    dev->cga.double_type = 0;
    cga_interpolate_init();

    if (rom_load_linear(HP45981A_FONT_PATH, 0, 0x1000, 0, dev->font_rom))
        dev->font_loaded = 1;

    /* Test hook: force an initial 3DE value (e.g. HP45981A_EXT=04 selects the
       HP character set) so the extended path can be exercised without guest
       software. */
    {
        const char *ext = getenv("HP45981A_EXT");

        if (ext != NULL)
            hp45981a_set_ext_ctrl(dev, (uint8_t) strtoul(ext, NULL, 16));
    }

    dev->monitor25k = (getenv("HP45981A_25KHZ") != NULL);

    if (hp45981a_trace < 0)
        hp45981a_trace = (getenv("HP45981A_TRACE") != NULL);

    if (hp45981a_trace > 0)
        pclog("[HP45981A] init: 32K VRAM at B8000, character ROM %s\n",
              dev->font_loaded ? "loaded" : "NOT FOUND");

    monitors[monitor_index_global].mon_composite = 0;
    dev->cga.monitor_used                        = monitor_index_global;

    return dev;
}

static void
hp45981a_close(void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    if (dev != NULL)
        free(dev);
}

static void
hp45981a_speed_changed(void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    cga_recalctimings(&dev->cga);
}

const device_t hp45981a_device = {
    .name          = "HP 45981A Multi-Mode Video Adapter",
    .internal_name = "hp45981a",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = hp45981a_init,
    .close         = hp45981a_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = hp45981a_speed_changed,
    .force_redraw  = NULL,
    .config        = NULL
};
