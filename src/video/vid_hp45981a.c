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
 *          control register at 3DD with the board ID at 3DF (table 7.8 of
 *          the 45981A technical reference; 3DE is unused).
 *
 *          Extended control register (3DD, write only, 6 bits):
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
 *          SPU/display access matrix.  The 25 kHz (400 line) timing runs
 *          from the card's 22.44 MHz master oscillator and the documented
 *          CRTC translations; see docs/hp-vectra/VIDEO-45981A.md.
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

    uint8_t gc_index;     /* 3CE: VGA Graphics Controller index */
    uint8_t gc_regs[9];   /* 3CF: 0=SetReset 1=EnableSR 3=DataRotate 5=Mode 8=BitMask */
    uint8_t ext_ctrl;  /* 3DD, write only */
    uint8_t monitor25k; /* W2 jumper: 0 = 15 kHz monitor, 1 = 25 kHz */
    uint8_t font_rom[0x4000];   /* four sets: STD 8x8/16x8, HP 8x8/14x8 */
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

static void
hp45981a_set_ext_ctrl(hp45981a_t *dev, uint8_t val)
{
    const uint8_t old = dev->ext_ctrl;

    dev->ext_ctrl = val & 0x3f;

    dev->cga.vram_mask = (dev->ext_ctrl & HP45981A_EXT_32K) ? HP45981A_VRAM_MASK32 : HP45981A_VRAM_MASK16;

    /* The character ROM holds four sets (manual p.85): A13 picks the standard
       or the HP pair, A12 the 8 row or the 16 row block inside it.  A12 is the
       monitor's cell height and is applied by the CGA core from CRTC R9. */
    if (dev->font_loaded) {
        dev->cga.font16         = dev->font_rom +
                                  ((dev->ext_ctrl & HP45981A_EXT_HPFONT) ? 0x2000 : 0x0000);
        dev->cga.font_row_major = 1;
    } else {
        dev->cga.font16         = NULL;
        dev->cga.font_row_major = 0;
    }

    dev->cga.clk    = (dev->ext_ctrl & HP45981A_EXT_400LINE) ? hp45981a_clk400() : CGACONST;
    /* Two sources select the 400 line mode: the extended control bit and the
       mode control bit 0 ("640x2" in Table 7.8's mode register description).
       HP's MM.DLL writes 3D8=FF and never touches 3DD, so keying only off the
       extended bit leaves the frame buffer banked as a 200 line screen while
       the CRTC is programmed for 25 rows of 16 scanlines. */
    dev->cga.gfx400 = ((dev->ext_ctrl & HP45981A_EXT_400LINE) ||
                       (dev->cga.cgamode & CGA_MODE_FLAG_HIGHRES)) ? 1 : 0;

    /* Page select only applies to the 16K address space (bit 4 clear). */
    dev->cga.gfx_page  = ((dev->ext_ctrl & HP45981A_EXT_PAGE) &&
                          !(dev->ext_ctrl & HP45981A_EXT_32K)) ? 1 : 0;
    dev->cga.underline = (dev->ext_ctrl & HP45981A_EXT_UNDERLINE) ? 1 : 0;

    if (old ^ dev->ext_ctrl) {
        dev->cga.fullchange = monitors[dev->cga.monitor_used].mon_changeframecount;
        cga_recalctimings(&dev->cga);
    }
}

/* CRTC translation.  The card carries a translation ROM that rewrites CRTC
   values written for a 15 kHz monitor into the 25 kHz equivalents, so that
   software written for a 15 kHz display runs unchanged on a 25 kHz one.  The
   ROM has never been dumped, and Table 7.7 of the technical reference shows
   that the mapping is not a plain (register, value) lookup - register 5 maps
   06h to 09h for 80x25 text but to 01h for the 200 line graphics modes - so
   the ROM cannot be reconstructed from the manual.  What is reproduced here
   are the documented mode sets: when the register file matches one of the
   15 kHz sets below, the card substitutes the matching 25 kHz set. */
typedef struct {
    const char *name;
    uint8_t     mode15[12]; /* R0 .. R11 as written for a 15 kHz monitor */
    uint8_t     mode25[12]; /* the same mode as programmed for 25 kHz       */
} hp45981a_mode_t;

static const hp45981a_mode_t hp45981a_modes[] = {
    /* 80x25 text, 60 Hz */
    { "80x25 60", { 0x71, 0x50, 0x5a, 0x0a, 0x1f, 0x06, 0x19, 0x1c, 0x02, 0x07, 0x06, 0x07 },
                  { 0x6d, 0x50, 0x57, 0x0a, 0x19, 0x09, 0x19, 0x19, 0x00, 0x0f, 0x0e, 0x0f } },
    /* 40x25 text, 60 Hz */
    { "40x25 60", { 0x38, 0x28, 0x2d, 0x0a, 0x1f, 0x06, 0x19, 0x1c, 0x02, 0x07, 0x06, 0x07 },
                  { 0x36, 0x28, 0x2c, 0x0a, 0x19, 0x09, 0x19, 0x19, 0x00, 0x0f, 0x0e, 0x0f } },
    /* 320x200 and 640x200 graphics, 60 Hz */
    { "gfx 200 60", { 0x38, 0x28, 0x2d, 0x0a, 0x7f, 0x06, 0x64, 0x70, 0x02, 0x01, 0x06, 0x07 },
                    { 0x36, 0x28, 0x2b, 0x0a, 0x69, 0x01, 0x64, 0x66, 0x00, 0x03, 0x0e, 0x0f } }
};

static void
hp45981a_translate_crtc(hp45981a_t *dev)
{
    int i;
    int r;

    for (i = 0; i < (int) (sizeof(hp45981a_modes) / sizeof(hp45981a_modes[0])); i++) {
        if (memcmp(dev->cga.crtc, hp45981a_modes[i].mode15, 12) != 0)
            continue;

        if (hp45981a_trace > 0)
            pclog("[HP45981A] CRTC translated for a 25 kHz monitor: %s\n", hp45981a_modes[i].name);

        for (r = 0; r < 12; r++)
            dev->cga.crtc[r] = hp45981a_modes[i].mode25[r];

        /* Cursor registers scale with the character cell. */
        dev->cga.crtc[CGA_CRTC_CURSOR_START] = 0x0e;
        dev->cga.crtc[CGA_CRTC_CURSOR_END]   = 0x0f;

        cga_recalctimings(&dev->cga);
        return;
    }
}

static void
hp45981a_out(uint16_t addr, uint8_t val, void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    if (hp45981a_trace > 0)
        pclog("[HP45981A] out %03X = %02X\n", addr, val);

    switch (addr) {
        case 0x03dd:
            /* Table 7.8 of the 45981A manual: 3DD write is the Extended
               Control Register.  HP's MM.DLL writes it on every mode set
               (0x29C8: out 3DD,[0x43D]) right before reprogramming 3D8 and
               the CRTC, so landing anywhere else silently breaks PM. */
            hp45981a_set_ext_ctrl(dev, val);
            return;

        case 0x03de:
            /* Unused on this board (Table 7.8). */
            return;

        case 0x03df:
            /* Board ID is read only. */
            return;

        case 0x03d8:
            /* Mode control register.  Table 7.8 of the 45981A manual uses the
               same bit encoding as the IBM CGA mode register, but this board
               only ever drives an RGB monitor (the 25 kHz Multi-Mode display),
               so the composite colour machinery of the CGA core has nothing to
               do here - and it is the only reason the core's cga16 worker
               thread gets woken from an I/O write.  Handle the register
               locally instead of going through cga_out(). */
            /* Six bit register (Table 7.8); the drivers set bit 7 as the
               "do not translate" marker, which must not reach the renderer. */
            dev->cga.cgamode = val & 0x3f;
            dev->cga.gfx400  = ((dev->ext_ctrl & HP45981A_EXT_400LINE) ||
                                (dev->cga.cgamode & CGA_MODE_FLAG_HIGHRES)) ? 1 : 0;

            cga_recalctimings(&dev->cga);
            return;

        case 0x03d9:
            /* Colour select register, six bits as well. */
            dev->cga.cgacol = val & 0x3f;
            return;

        default:
            /* 3D0-3D7 (CRTC, mirrored four times), 3D8, 3D9 and 3DB/3DC are
               all handled by the CGA core.  With the 25 kHz monitor the card
               sits behind its translation ROM, and HP's drivers mark a value
               as "already in 25 kHz form, do not translate" by setting bit 7
               (e.g. R1 = 0xA8 for a 40 character line).  The ROM passes the
               value on with the marker stripped, so mask it here; storing
               0xA8 as a horizontal display width made the renderer draw a
               2704 pixel line. */
            if (dev->monitor25k && (val & 0x80))
                cga_out(addr, val & 0x7f, &dev->cga);
            else
                cga_out(addr, val, &dev->cga);

            /* Data writes to the CRTC go through the translation logic when a
               25 kHz monitor is connected (W2 jumper); values with bit 7 set
               bypass it, as on the real card. */
            if (dev->monitor25k && (val < 0x80) &&
                ((addr == 0x03d1) || (addr == 0x03d3) || (addr == 0x03d5) || (addr == 0x03d7)))
                hp45981a_translate_crtc(dev);

            return;
    }
}

static uint8_t
hp45981a_in(uint16_t addr, void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    switch (addr) {
        case 0x03dd:
            /* Undefined on read; hand back the latched value so a driver
               that reads back its own mode word sees what it wrote. */
            return dev->ext_ctrl;

        case 0x03de:
            /* Undefined (Table 7.8). */
            return 0xff;

        case 0x03df: {
            /* Board ID register (read only): 41h identifies a 45981A. */
            return HP45981A_BOARD_ID;
        }

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

    /* VGA Graphics Controller bit mask (register 8).  POINTDD draws the mouse
       cursor with Write Mode 0 + bit mask, so honouring it here is what lets
       the pointer update instead of dropping every write on the floor. */
    if (dev->gc_regs[8] != 0xFF) {
        uint32_t off = addr & (HP45981A_VRAM_SIZE - 1);
        uint8_t  o   = dev->vram[off];
        val = (uint8_t) ((o & ~dev->gc_regs[8]) | (val & dev->gc_regs[8]));
    }

    /* The mask inside cga_write applies the 16K/32K wrap. */
    cga_write(addr, val, &dev->cga);
}

static uint8_t
hp45981a_gc_in(uint16_t addr, void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    if (addr == 0x03ce)
        return dev->gc_index;

    return dev->gc_regs[dev->gc_index & 7];
}

static void
hp45981a_gc_out(uint16_t addr, uint8_t val, void *priv)
{
    hp45981a_t *dev = (hp45981a_t *) priv;

    if (addr == 0x03ce) {
        dev->gc_index = val & 0x0f;
        return;
    }

    if (dev->gc_index < 9)
        dev->gc_regs[dev->gc_index] = val;
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

    /* VGA Graphics Controller, decoded here because the manual puts the whole
       3C0-3CF range on the adapter.  Only the bit mask is acted upon for now;
       it is the one POINTDD needs to draw the pointer. */
    dev->gc_regs[8] = 0xff;
    io_sethandler(0x03ce, 0x0002, hp45981a_gc_in, NULL, NULL, hp45981a_gc_out, NULL, NULL, dev);

    overscan_x = overscan_y = 16;

    cga_palette = 0;
    cgapal_rebuild();
    update_cga16_color(dev->cga.cgamode, dev->cga.cgacol);

    dev->cga.double_type = 0;
    cga_interpolate_init();

    if (rom_load_linear(HP45981A_FONT_PATH, 0, 0x4000, 0, dev->font_rom))
        dev->font_loaded = 1;

    /* The Vectra ships with the HP Multi-Mode display, so the W2 jumper sits
       in the 25 kHz position: the card reports a 25 kHz monitor on 3DA bit 4
       and runs the CRTC translation ROM.  HP45981A_15KHZ forces the other
       jumper position for testing. */
    dev->monitor25k = (getenv("HP45981A_15KHZ") == NULL);

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
