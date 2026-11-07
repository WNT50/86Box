/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-D: the SCN2674 video board.
 *
 *          This is the PC-D's board, not the PC-X's: the PC-D drives the
 *          display through an SCN2674 with its own 32 KB display RAM at
 *          0xf0000-0xf7fff and an 8741A carrying the keyboard and the mouse.
 *          The PC-X in turn has an 8031 terminal board (vid_pcx.c) and a
 *          serial keyboard, so the two machines must not share a model.
 *
 *          Modelled here: the display RAM window (characters + attributes in
 *          the lower half, bitmap words when the graphics bit is set), the
 *          host registers at 0xf980-0xf98f and the character generator in
 *          d12-graka.bin.  The SCN2674's own register file, the mouse and the
 *          8741 firmware are not modelled.
 *
 *
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/io.h>
#include <86box/rom.h>
#include <86box/video.h>
#include <86box/plat.h>
#include "../cpu/cpu.h"
#include <86box/pcd_vid.h>

#define PCD_VIDEO_XSIZE 640
#define PCD_VIDEO_YSIZE 400
#define PCD_VIDEO_FREQ  50

#define PCD_TERM_COLS   80
#define PCD_TERM_ROWS   25
#define PCD_CELL_W      8
#define PCD_CELL_H      14

typedef struct pcd_video_t {
    uint8_t  vram[0x8000];
    uint8_t  font[0x2000];
    int      font_ok;

    uint8_t  regs[16];
    int      vram_sw;      /* 0xf9a0 bit 0: 0 = character generator RAM */
    int      gfx_on;       /* command bit 1: the bitmap path */
    int      display_on;   /* command bit 3: display enable */
    int      scr1;         /* screen start 1, in character cells */
    int      inverse;      /* 8741 P2 bit 3: whole screen negative */

    int          vram_mapped;
    mem_mapping_t mapping;
    int          dirty;

    uint32_t fb[PCD_VIDEO_XSIZE * PCD_VIDEO_YSIZE];
    timer_t  timer;
} pcd_video_t;

static video_timings_t timing_pcd = { VIDEO_ISA, 8, 16, 32, 8, 16, 32 };

static void pcd_video_poll(void *priv);

static uint8_t
pcd_vram_read(uint32_t addr, UNUSED(void *priv))
{
    pcd_video_t *dev = (pcd_video_t *) priv;

    return dev->vram[addr & 0x7fff];
}

static void
pcd_vram_write(uint32_t addr, uint8_t val, void *priv)
{
    pcd_video_t *dev = (pcd_video_t *) priv;

    if (!dev->vram_sw) {
        /* One byte per word: the character generator download. */
        if (!(addr & 1)) {
            dev->font[(addr >> 1) & 0x1fff] = val;
            dev->font_ok                     = 1;
        }
        return;
    }

    dev->vram[addr & 0x7fff] = val;
    dev->dirty               = 1;
}

/* Character mode: the lower 4000 bytes hold character/attribute pairs. */
static void
pcd_scan_text(pcd_video_t *dev)
{
    int      row, col, line, bit, x, y;
    uint8_t  ch, attr, glyph;
    uint32_t fg, bg;

    const uint32_t white = 0x00e8e8e8, black = 0x00000000;

    for (row = 0; row < PCD_TERM_ROWS; row++) {
        for (col = 0; col < PCD_TERM_COLS; col++) {
            int cell = ((dev->scr1 + row * PCD_TERM_COLS + col) & 0x1fff) * 2;

            ch   = dev->vram[cell];
            attr = dev->vram[cell + 1];

            fg = dev->inverse ? black : white;
            bg = dev->inverse ? white : black;
            if (attr & 0x10) {
                uint32_t t = fg;
                fg         = bg;
                bg         = t;
            }

            for (line = 0; line < PCD_CELL_H; line++) {
                y = row * PCD_CELL_H + line;
                if (y >= PCD_VIDEO_YSIZE)
                    continue;
                glyph = dev->font_ok
                            ? dev->font[((ch * 16) + line) & 0x1fff]
                            : 0x00;
                for (bit = 0; bit < PCD_CELL_W; bit++) {
                    x                    = col * PCD_CELL_W + bit;
                    dev->fb[y * PCD_VIDEO_XSIZE + x] =
                        (glyph & (0x80 >> bit)) ? fg : bg;
                }
            }
        }
    }

    dev->dirty = 0;
}

/* Graphics: 640 x 350 bits taken straight from the display RAM. */
static void
pcd_scan_bitmap(pcd_video_t *dev)
{
    int      x, y, bit;
    uint8_t  val;
    uint32_t on, off;

    on  = dev->inverse ? 0x00000000 : 0x00e8e8e8;
    off = dev->inverse ? 0x00e8e8e8 : 0x00000000;

    for (y = 0; y < PCD_VIDEO_YSIZE; y++) {
        for (x = 0; x < PCD_VIDEO_XSIZE; x += 8) {
            val = dev->vram[(y * (PCD_VIDEO_XSIZE / 8)) + (x >> 3)];
            for (bit = 0; bit < 8; bit++) {
                int set = (val & (0x80 >> bit)) ? 1 : 0;
                if (dev->inverse)
                    set = !set;
                dev->fb[y * PCD_VIDEO_XSIZE + x + bit] = set ? on : off;
            }
        }
    }

    dev->dirty = 0;
}

static void
pcd_video_poll(void *priv)
{
    pcd_video_t *dev = (pcd_video_t *) priv;
    int          y;

    if (!dev->vram_mapped) {
        mem_mapping_add(&dev->mapping, 0x000f0000, 0x00008000,
                        pcd_vram_read, NULL, NULL,
                        pcd_vram_write, NULL, NULL,
                        NULL, MEM_MAPPING_INTERNAL, dev);
        dev->vram_mapped = 1;
    }

    if (dev->dirty) {
        if (dev->gfx_on)
            pcd_scan_bitmap(dev);
        else
            pcd_scan_text(dev);
    }

    for (y = 0; y < PCD_VIDEO_YSIZE; y++)
        memcpy(buffer32->line[y], &dev->fb[y * PCD_VIDEO_XSIZE],
               PCD_VIDEO_XSIZE * 4);

    video_blit_memtoscreen(0, 0, PCD_VIDEO_XSIZE, PCD_VIDEO_YSIZE);
    timer_advance_u64(&dev->timer, TIMER_USEC * (1000000ULL / PCD_VIDEO_FREQ));
}

static uint8_t
pcd_video_read(uint16_t port, void *priv)
{
    pcd_video_t *dev = (pcd_video_t *) priv;
    uint16_t     reg = (uint16_t) (port & 0x0f);

    /* 0x02 is the port the DOS console driver handshakes on: it sets bit 6 of
       the value and then polls bit 4 until it reads high (0070:4D19-4D20).
       The board only asserts that bit when the character path is idle, and
       this model has no busy state, so report the written value with bit 4
       forced high.  Other ports still return their index: the BIOS reads them
       as part of its init sequence and nothing here proves a read-back yet. */
    if (reg == 0x02)
        return (uint8_t) (dev->regs[reg] | 0x10);

    return (uint8_t) reg;
}

static void
pcd_video_write(uint16_t port, uint8_t val, void *priv)
{
    pcd_video_t *dev = (pcd_video_t *) priv;
    uint16_t     reg = (uint16_t) (port & 0x0f);

    dev->regs[reg] = val;

    if (getenv("PCD_VIDLOG") != NULL)
        pclog("[PCDREG] %02X=%02X @%04X:%04X\n", reg, val, CS, cpu_state.pc);

    switch (reg) {
        /* The chip's port map is not the plain register file: 0x00 feeds the
           initialization registers, 0x01 is the command register, 0x02/0x03
           are the low and high halves of screen start 1 (SCN2674, as MAME
           models it).  Treating 0x02 as the command register made the ROM's
           screen-start write look like "graphics on", and the board then drew
           an empty bitmap where the text should be. */
        case 0x01:
            if (val & 0x02)
                dev->gfx_on = (val & 0x01) ? 1 : 0;
            if (val & 0x08)
                dev->display_on = (val & 0x01) ? 1 : 0;
            dev->dirty = 1;
            break;

        case 0x02:
            dev->scr1  = (dev->scr1 & 0x3f00) | val;
            dev->dirty = 1;
            break;

        case 0x03:
            dev->scr1  = (dev->scr1 & 0x00ff) | ((val & 0x3f) << 8);
            dev->dirty = 1;
            break;

        default:
            break;
    }
}

static void
pcd_vram_sw_write(UNUSED(uint16_t addr), uint8_t val, void *priv)
{
    pcd_video_t *dev = (pcd_video_t *) priv;

    dev->vram_sw = val & 1;
    dev->dirty   = 1;
    if (getenv("PCD_VIDLOG") != NULL)
        pclog("[PCDSW] %02X @%04X:%04X\n", val, CS, cpu_state.pc);
}

static void *
pcd_video_init(UNUSED(device_t *dev))
{
    pcd_video_t *vid = (pcd_video_t *) calloc(1, sizeof(pcd_video_t));
    int          i;

    if (vid == NULL)
        return NULL;

    /* d12-graka: 512 glyphs of 16 bytes, the PC-D's character generator. */
    {
        FILE *f = rom_fopen("roms/machines/pcd/d12-graka.bin", "rb");

        if (f != NULL) {
            i = (int) fread(vid->font, 1, sizeof(vid->font), f);
            fclose(f);
            vid->font_ok = (i == (int) sizeof(vid->font));
        }
    }

    vid->vram_sw   = 1;
    vid->display_on = 1;
    vid->inverse = 0;

    for (i = 0; i < (PCD_VIDEO_XSIZE * PCD_VIDEO_YSIZE); i++)
        vid->fb[i] = 0x00000000;

    overscan_x = 0;
    overscan_y = 0;
    video_inform(VIDEO_FLAG_TYPE_MDA, &timing_pcd);

    timer_add(&vid->timer, pcd_video_poll, vid, 1);
    io_sethandler(0xf980, 0x0010, pcd_video_read, NULL, NULL,
                  pcd_video_write, NULL, NULL, vid);
    io_sethandler(0xf9a0, 0x0001, NULL, NULL, NULL,
                  pcd_vram_sw_write, NULL, NULL, vid);

    return vid;
}

static void
pcd_video_close(void *priv)
{
    free(priv);
}

const device_t pcd_video_device = {
    .name          = "Siemens PC-D video board",
    .internal_name = "pcd_video",
    .flags         = DEVICE_ISA16,
    .local         = 0,
    .init          = pcd_video_init,
    .close         = pcd_video_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
