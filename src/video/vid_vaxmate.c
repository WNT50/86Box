/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the DEC VAXmate I/O-Video module video controller:
 *          a 6845 CRTC with a 64 KB dual-ported video memory, a 4 KB
 *          programmable font RAM, a 16-entry video look-up table and a
 *          translation ROM in front of the industry-standard CRTC ports.
 *
 *          Reference: AA-HD92A-TK VAXmate Technical Reference Manual
 *          Volume 1, Chapter 7 (Video Controller - Hardware Description);
 *          EK-PC500-TD VAXmate Technical Description, Chapter 4.
 *
 *          Register/port map (TRM Table 7-6):
 *            03D0 W     CRTC index   (DIGITAL extended, direct)
 *            03D1 R/W   CRTC data    (DIGITAL extended, direct)
 *            03D4 W     CRTC index   (industry-standard, through translation ROM)
 *            03D5 R/W   CRTC data    (industry-standard, through translation ROM)
 *            03D8 W     Control Register A
 *            03D9 W     Color Select Register
 *            03DA R     Status Register A
 *            03DD R     Status Register B
 *            03DE R     Write Data Register (last value written via 03D4/03D5)
 *            03DF W     Control Register B
 *            0C80 R/W   Special Purpose Register
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/pit.h>

#include "cpu.h"
#include <86box/video.h>
#include <86box/plat_unused.h>

#define VAXMATE_VRAM       0x10000 /* 64 KB video memory, B0000-BFFFF */
#define VAXMATE_FONT_RAM   0x01000 /* 256 characters x 16 bytes         */
#define VAXMATE_VLT        16

/* CRTC register indices (TRM Table 7-7) */
enum {
    CRTC_HTOTAL = 0x00,
    CRTC_HDISP  = 0x01,
    CRTC_HSYNC  = 0x02,
    CRTC_SYNCW  = 0x03,
    CRTC_VTOTAL = 0x04,
    CRTC_VADJ   = 0x05,
    CRTC_VDISP  = 0x06,
    CRTC_VSYNC  = 0x07,
    CRTC_ILACE  = 0x08,
    CRTC_MAXSCAN = 0x09,
    CRTC_CURSTART = 0x0a,
    CRTC_CUREND   = 0x0b,
    CRTC_STARTHI  = 0x0c,
    CRTC_STARTLO  = 0x0d,
    CRTC_CURHI    = 0x0e,
    CRTC_CURLO    = 0x0f,
    CRTC_LPENHI   = 0x10,
    CRTC_LPENLO   = 0x11
};

/* Video modes, TRM Table 7-11 (control register A bits 2-0 + control B bit 7) */
enum {
    VMODE_TEXT40   = 0,
    VMODE_TEXT80   = 1,
    VMODE_G320x200x4  = 2,
    VMODE_G320x200x16 = 3,
    VMODE_G640x400x2  = 4,
    VMODE_G640x200x4  = 5,
    VMODE_G640x200x2  = 6,
    VMODE_G640x400x4  = 7,
    VMODE_G800x252x4  = 8
};

/* Control register B (03DF) */
#define CTLB_MONITOR252   0x80
#define CTLB_SAVER        0x40
#define CTLB_CRA5_ENABLE  0x20
#define CTLB_FONT_ENABLE  0x10
#define CTLB_DISPLAY_EN   0x08
#define CTLB_VLT_ENABLE   0x04

/* Control register A (03D8) */
#define CTLA_BLINK        0x20
#define CTLA_DISPLAY_EN   0x08
#define CTLA_MODE_MASK    0x07

/* Status register A (03DA) */
#define STATA_DISPENSE    0x01
#define STATA_VSYNC       0x08

/* Status register B (03DD) */
#define STATB_FROM_3D4    0x02
#define STATB_FROM_3D0    0x01

/* Special purpose register (0C80), TRM page 7-23 */
#define SPR_WRITE_PROTECT 0x80
#define SPR_TRACK0        0x40
#define SPR_INDEX         0x20
#define SPR_SPEED_IND     0x10
#define SPR_DISABLE_VIDEO 0x08
#define SPR_SPLIT_BAUD    0x04
#define SPR_DISABLE_COMM  0x02
#define SPR_SPEED_SELECT  0x01

typedef struct vaxmate_video_t {
    mem_mapping_t mapping;

    uint8_t  crtc[32];     /* values the 6845 sees (after translation) */
    uint8_t  crtc_raw[32]; /* values the guest wrote to 03D5           */
    uint8_t  crtcreg;
    uint8_t  ctrl_a;
    uint8_t  ctrl_b;
    uint8_t  color_select;
    uint8_t  write_data;
    uint8_t  status_a;
    uint8_t  status_b;

    uint8_t  vram[VAXMATE_VRAM];
    uint8_t  font_ram[VAXMATE_FONT_RAM];
    uint8_t  vlt[VAXMATE_VLT];

    int      monitor_index;
    int      prev_monitor_index;
    int      monitor_mode; /* 0 = 400 line, 1 = 252 line (control B bit 7) */

    int      vc;
    int      sc;
    int      scanline;
    int      memaddr;
    int      memaddr_backup;
    int      text_base;
    int      displine;
    int      active_lines;
    int      hphase;
    int      firstline;
    int      lastline;
    int      linepos;
    int      dispon;
    int      vsynctime;
    int      vadj;
    int      cursorvisible;
    int      cursoron;
    int      blink;

    uint64_t dispontime;
    uint64_t dispofftime;
    uint64_t line_ticks;
    pc_timer_t timer;

    int      mode;
    uint8_t  spr;
    int      video_disabled;
} vaxmate_video_t;

#define VAXMATE_MONITOR_PROLOGUE()                          \
    {                                                       \
        vid->prev_monitor_index = monitor_index_global;     \
        monitor_index_global    = vid->monitor_index;       \
    }
#define VAXMATE_MONITOR_EPILOGUE()                          \
    {                                                       \
        monitor_index_global = vid->prev_monitor_index;     \
    }

static uint64_t vax_fw, vax_vw;
static uint64_t vax_frm_lines, vax_frm_rows, vax_frm_frames, vax_frm_vcresets;
static int      vax_init_count;
static vaxmate_video_t *vax_timer_vid;
static void vaxmate_shot_now(void);
static vaxmate_video_t *vaxmate_vid_global = NULL;  /* 临时诊断：0=3D8 1=3D4/5 2=3D0/1 */

static video_timings_t timing_vaxmate = { .type = VIDEO_ISA, .write_b = 8, .write_w = 16, .write_l = 32,
                                          .read_b = 8, .read_w = 16, .read_l = 32 };

/* Dot clock: the video gate array runs from the 22.367195 MHz crystal for
   every mode except VT240 emulation, which switches to 27.977244 MHz
   (TRM 4.2.7).  The monitor then sees 26.40 kHz horizontal / 60 Hz vertical
   with 848 dots per line at 640 pixels, and 26.49 kHz with 1056 dots at
   800 pixels (TRM chapter 6, "I/O - Video Module"). */
#define VAXMATE_CLK_NORMAL  22367195.0
#define VAXMATE_CLK_VT240   27977244.0

/* Dot counts that make up one scan line, active plus blanking. */
#define VAXMATE_DOTS_640    848
#define VAXMATE_DOTS_800    1056

/* Lines per frame at 60 Hz: 26.40 kHz / 60 Hz = 440, and 441 for the
   800x250 mode (26.49 kHz).  Only the first 400 (or 252) are displayed. */
#define VAXMATE_LINES_640   440
#define VAXMATE_LINES_800   441

/* defined after the mode table; used by recalctimings() */
static int vaxmate_mode_width(int mode);

/*
 * Translation ROM (TRM page 7-26, Figure 7-1).
 *
 * The industry-standard CRTC window (03D4/03D5) feeds a 4K x 8 parameter
 * translation ROM: an 03D4 write latches the low four bits of the register
 * number, and an 03D5 write addresses the ROM with {register 0-15, data 7-0}.
 * Only the ROM output (LAD7-0) reaches the 6845, so software that programs
 * CGA timings still ends up with DIGITAL monitor timings.
 *
 * The ROM contents are not recoverable: it sits inside the gate array and the
 * CPU cannot read it. This is therefore a behavioural model rather than a
 * table dump. It keeps the guest's values in crtc_raw[] and rewrites the
 * vertical registers so that the 6845 describes the frame we actually render:
 * a fixed 400-line monitor (a 200-line field is shown twice) and an 8x16
 * character cell in text modes. R6 is only seven bits wide, so the number of
 * scan lines per character row is the smallest power of two that keeps the
 * row count inside 127. The guest's blanking and sync offsets are kept, in
 * scan lines, so the field proportions do not change.
 *
 * DEC's own ROM BIOS programs the DIGITAL window (03D0/03D1) directly and never
 * touches this path, so the translation only affects software written for an
 * industry-standard CGA.
 */
static double
vaxmate_ticks_per_dot(vaxmate_video_t *vid)
{
    double clk = (vid->mode == VMODE_G800x252x4) ? VAXMATE_CLK_VT240
                                                 : VAXMATE_CLK_NORMAL;

    return (cpuclock * (double) (1ULL << 32)) / clk;
}

static int
vaxmate_line_dots(vaxmate_video_t *vid)
{
    return (vid->mode == VMODE_G800x252x4) ? VAXMATE_DOTS_800
                                           : VAXMATE_DOTS_640;
}

static void
vaxmate_video_recalctimings(vaxmate_video_t *vid)
{
    /* One scan line is a fixed number of dots at the crystal frequency, no
       matter what the guest programmed: the translation ROM keeps the field
       at exactly 26.40 kHz (26.49 in the 800-pixel mode) so the DEC monitor
       stays locked.  The CRTC only decides where inside that line the active
       area starts. */
    int    dots   = vaxmate_line_dots(vid);
    int    active = vaxmate_mode_width(vid->mode);
    double ticks  = vaxmate_ticks_per_dot(vid);

    if (active < 320)
        active = 640;

    vid->dispontime  = (uint64_t) ((double) active * ticks);
    vid->dispofftime = (uint64_t) ((double) (dots - active) * ticks);
    vid->line_ticks  = (uint64_t) ((double) dots * ticks);
}


static int
vaxmate_video_mode(vaxmate_video_t *vid)
{
    int mode = vid->ctrl_a & CTLA_MODE_MASK;

    /* 扩展模式（control A 的 bit4 置位，低 3 位是扩展模式号）：
       0xD0 -> 10h/18h，640x400 2 色；
       0xD1 -> 13h/1Bh，640x400 4 色；0xD2 用同值，靠 control B 的 252 行位区分。 */
    if ((vid->ctrl_a & 0x17) == 0x10)
        return 4;
    if ((vid->ctrl_a & 0x17) == 0x13)
        return (vid->ctrl_b & CTLB_MONITOR252) ? 8 : 7;

    if (mode == 7 && (vid->ctrl_b & CTLB_MONITOR252))
        return VMODE_G800x252x4;

    return mode;
}

/* Text buffer <-> font RAM/VLT shadowing, TRM pages 7-15 and 7-30:
   when control register B bit 4 (font) or bit 2 (VLT) is set, even text
   buffer addresses access the font RAM / VLT instead of video memory. */
static void
vaxmate_video_write(uint32_t addr, uint8_t val, void *priv)
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;

    /* 断开 bit3 后只有 B8000 以上失效，B0000-B7FFF 的 MDA 窗口保留 */
    if (vid->video_disabled && (addr >= 0xb8000))
        return;


    if (addr & 1) {
        /* odd addresses always hit video memory */
        vid->vram[addr & (VAXMATE_VRAM - 1)] = val;
        return;
    }

    if ((vid->ctrl_b & CTLB_FONT_ENABLE) && (vid->mode <= VMODE_TEXT80)) {
        /* B8000-B9FFF even addresses -> 4 KB font RAM */
        vax_fw++;
        uint32_t off = (addr - 0xb8000) >> 1;
        if (off < VAXMATE_FONT_RAM) {
            vid->font_ram[off] = val;
            return;
        }
    }

    if ((vid->ctrl_b & CTLB_VLT_ENABLE) && (vid->mode <= VMODE_TEXT80)) {
        if (addr >= 0xb8000 && addr < 0xb8020) {
            vid->vlt[(addr - 0xb8000) >> 1] = val;
            return;
        }
    }

    vid->vram[addr & (VAXMATE_VRAM - 1)] = val;
}

static uint8_t
vaxmate_video_read(uint32_t addr, void *priv)
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;

    if (vid->video_disabled && (addr >= 0xb8000))
        return 0xff;


    if (!(addr & 1)) {
        if ((vid->ctrl_b & CTLB_FONT_ENABLE) && (vid->mode <= VMODE_TEXT80)) {
            uint32_t off = (addr - 0xb8000) >> 1;
            if (off < VAXMATE_FONT_RAM)
                return vid->font_ram[off];
        }
        if ((vid->ctrl_b & CTLB_VLT_ENABLE) && (vid->mode <= VMODE_TEXT80)) {
            if (addr >= 0xb8000 && addr < 0xb8020)
                return vid->vlt[(addr - 0xb8000) >> 1];
        }
    }

    return vid->vram[addr & (VAXMATE_VRAM - 1)];
}

/*
 * Graphics memory maps (TRM pages 7-11 .. 7-16, Figures 7-3 .. 7-15).
 *
 * A logical scan line L lives in block (L mod P); the block stride and the
 * first block address differ per mode.  200-line modes are displayed twice on
 * the 400-line monitor, 252-line mode is displayed as-is.
 */
typedef struct {
    uint16_t width;
    uint16_t lines;
    uint16_t rowbytes;
    uint8_t  planes;   /* number of memory blocks        */
    uint32_t stride;   /* bytes between blocks           */
    uint32_t origin;   /* first block, absolute address  */
    uint8_t  bpp;      /* bits per pixel                 */
} vaxmate_gfx_t;

static const vaxmate_gfx_t vaxmate_modes[9] = {
    /* 0 40x25 text      */ { 640, 400, 80, 1, 0x2000, 0xb8000, 0 },
    /* 1 80x25 text      */ { 640, 400, 160, 1, 0x2000, 0xb8000, 0 },
    /* 2 320x200 4-color */ { 320, 200, 80, 2, 0x2000, 0xb8000, 2 },
    /* 3 320x200 16-color*/ { 320, 200, 160, 4, 0x2000, 0xb8000, 4 },
    /* 4 640x400 2-color */ { 640, 400, 80, 4, 0x2000, 0xb8000, 1 },
    /* 5 640x200 4-color */ { 640, 200, 160, 2, 0x4000, 0xb8000, 2 },
    /* 6 640x200 2-color */ { 640, 200, 80, 2, 0x2000, 0xb8000, 1 },
    /* 7 640x400 4-color */ { 640, 400, 160, 4, 0x4000, 0xb0000, 2 },
    /* 8 800x250 4-color */ { 800, 250, 200, 4, 0x4000, 0xb0000, 2 },
};

static void
vaxmate_crtc_translate(vaxmate_video_t *vid)
{
    const vaxmate_gfx_t *m = &vaxmate_modes[vid->mode];
    int text = (vid->mode <= VMODE_TEXT80);
    int lines;
    int lpr  = 1;
    int rows;
    int raw4, raw6, raw7, raw9;
    int guest_lpr, guest_lines, guest_total;
    int blank_lines, sync_lines;
    int i, v;

    lines = text ? 400 : m->lines;

    while ((lines / lpr) > 128)
        lpr <<= 1;
    if (text)
        lpr = 16;

    rows = lines / lpr;

    for (i = 0; i < 32; i++)
        vid->crtc[i] = vid->crtc_raw[i];

    raw4 = vid->crtc_raw[CRTC_VTOTAL] & 0x7f;
    raw6 = vid->crtc_raw[CRTC_VDISP] & 0x7f;
    raw7 = vid->crtc_raw[CRTC_VSYNC] & 0x7f;
    raw9 = vid->crtc_raw[CRTC_MAXSCAN] & 0x1f;

    /* Blanking and sync offsets as the guest meant them, counted in scan
       lines so they survive the change of scan lines per row. */
    guest_lpr   = raw9 + 1;
    guest_lines = (raw6 + 1) * guest_lpr;
    guest_total = (raw4 + 1) * guest_lpr;

    blank_lines = guest_total - guest_lines;
    if (blank_lines < 0)
        blank_lines = 0;

    sync_lines = ((raw7 + 1) - (raw6 + 1)) * guest_lpr;
    if (sync_lines < 0)
        sync_lines = 0;

    vid->crtc[CRTC_MAXSCAN] = (uint8_t) (lpr - 1);
    vid->crtc[CRTC_VDISP]   = (uint8_t) (rows - 1);

    v = rows + (blank_lines / lpr);
    vid->crtc[CRTC_VTOTAL] = (uint8_t) ((v > 0x7f) ? 0x7f : v);

    v = rows + (sync_lines / lpr);
    vid->crtc[CRTC_VSYNC] = (uint8_t) ((v > 0x7f) ? 0x7f : v);
}



static int
vaxmate_mode_width(int mode)
{
    if ((mode < 0) || (mode > 8))
        return 0;

    return vaxmate_modes[mode].width;
}

/* CRTC parameter blocks, ROM F000:F13D -- 12 bytes each (R0..R11), R12-R15 = 0.
   Values cross-checked against TRM Table 7-8 "CRTC Register Values".
   Block index per internal mode:
     +00 40x25 text (IBM 0/1)   +10 80x25 text (IBM 2/3)
     +20 320x200 modes and D0   +30 IBM mode 7 (MDA timing)
     +40 D1 640x400x4           +50 D2 800x252x4                        */
static const uint8_t vaxmate_crtc_blocks[6][12] = {
    { 0x34, 0x28, 0x2d, 0x54, 0x1a, 0x08, 0x19, 0x19, 0x40, 0x0f, 0x00, 0x07 },
    { 0x69, 0x50, 0x58, 0x58, 0x1a, 0x08, 0x19, 0x19, 0x40, 0x0f, 0x00, 0x07 },
    { 0x34, 0x28, 0x2c, 0x54, 0x6d, 0x00, 0x64, 0x66, 0x40, 0x03, 0x00, 0x07 },
    { 0x61, 0x50, 0x52, 0x0f, 0x19, 0x06, 0x19, 0x19, 0x02, 0x0d, 0x0b, 0x0c },
    { 0x69, 0x50, 0x58, 0x58, 0x6d, 0x00, 0x64, 0x66, 0x42, 0x03, 0x00, 0x07 },
    { 0x83, 0x64, 0x6d, 0x5a, 0x6d, 0x01, 0x3f, 0x53, 0x40, 0x03, 0x00, 0x07 },
};

/* VLT contents are RGBI ordered: bit 3 = R, 2 = G, 1 = B, 0 = I */
static uint32_t
vaxmate_vlt_color(uint8_t v)
{
    int r = (v >> 3) & 1;
    int g = (v >> 2) & 1;
    int b = (v >> 1) & 1;
    int i = v & 1;
    int lvl = i ? 0xff : 0xa0;
    int y;

    if (!r && !g && !b)
        y = i ? 0x60 : 0x00;
    else
        y = ((r * lvl) * 30 + (g * lvl) * 59 + (b * lvl) * 11) / 100;

    if (y > 255)
        y = 255;

    return 0xff000000 | (uint32_t) (y << 16) | (uint32_t) (y << 8) | (uint32_t) y;
}

/* 4-color palette selection, TRM Table 7-10 (CPS = bit 5, SIC = bit 4) */
static const uint8_t vaxmate_pal4[4][4] = {
    { 0x0, 0x2, 0x4, 0x6 },   /* CPS=0 SIC=0: black, green, red, brown      */
    { 0x0, 0x3, 0x5, 0xb },   /* CPS=1 SIC=0: black, cyan, magenta, l.cyan  */
    { 0x0, 0xa, 0xc, 0xe },   /* CPS=0 SIC=1: black, l.green, l.red, yellow */
    { 0x0, 0x7, 0xd, 0xf },   /* CPS=1 SIC=1: black, white, l.magenta, white*/
};


/* BIOS 从不下载字库（fw=0）：字符发生器在 ROM 里，物理 0xFC0A0 起
   是标准 8x16 字形表（256 字符 x 16 字节，空格全 0、'A' 对称）。
   首次渲染时若 font RAM 全空，就从 ROM 拷一份。 */
static void
vaxmate_font_lazy_load(vaxmate_video_t *vid)
{
    static int loaded = 0;
    int        i;
    int        nz = 0;

    if (loaded)
        return;

    for (i = 0; i < VAXMATE_FONT_RAM; i++) {
        if (vid->font_ram[i]) {
            nz++;
            break;
        }
    }

    if (!nz) {
        for (i = 0; i < VAXMATE_FONT_RAM; i++)
            vid->font_ram[i] = mem_readb_phys(0xfc0a0 + i);
    }

    loaded = 1;
}

static void
vaxmate_video_render_text(vaxmate_video_t *vid, int line)
{
    int      cols = (vid->mode == VMODE_TEXT40) ? 40 : 80;

    vaxmate_font_lazy_load(vid);
    int      cw   = (cols == 40) ? 16 : 8;  /* 40-column modes: 16-dot cells */
    int      pix  = cw / 8;
    int      x;
    uint16_t cursoraddr = (uint16_t) ((vid->crtc[CRTC_CURLO] | (vid->crtc[CRTC_CURHI] << 8)) & 0x3fff);
    int      cursorvis  = vid->cursorvisible && vid->cursoron;
    int      blink      = (vid->blink >> 4) & 1;

    for (x = 0; x < cols; x++) {
        /* text_base 是字节基址（B0000->0, B8000->0x8000），
           memaddr 是字符地址，所以先乘 2 再加基址 */
        uint32_t off  = ((uint32_t) vid->text_base
                       + (((uint32_t) vid->memaddr + (uint32_t) x) * 2)) & 0xffff;
        uint8_t  chr  = vid->vram[off & (VAXMATE_VRAM - 1)];
        uint8_t  attr = vid->vram[(off + 1) & (VAXMATE_VRAM - 1)];
        uint32_t fg = vaxmate_vlt_color(vid->vlt[attr & 0x0f]);
        uint32_t bg = vaxmate_vlt_color(vid->vlt[(attr >> 4) & 0x0f]);
        int      is_cursor = cursorvis && (uint16_t) (vid->memaddr + x) == cursoraddr;
        int      is_blink  = (vid->ctrl_a & CTLA_BLINK) && blink && (attr & 0x80);
        int      c;
        int      p;

        if (is_blink)
            bg = vaxmate_vlt_color(vid->vlt[0]);

        for (c = 0; c < 8; c++) {
            uint8_t  row = vid->font_ram[(chr * 16 + (vid->scanline & 15)) & (VAXMATE_FONT_RAM - 1)];
            int      on  = (row >> (7 - c)) & 1;
            uint32_t col = on ? fg : bg;

            if (is_cursor)
                col = on ? bg : fg;

            for (p = 0; p < pix; p++)
                buffer32->line[line][x * cw + c * pix + p] = col;
        }
    }
}

static void
vaxmate_video_render_graphics(vaxmate_video_t *vid, int line, int displine)
{
    const vaxmate_gfx_t *m = &vaxmate_modes[vid->mode];
    uint32_t off;
    int      l;
    int      x;

    if ((vid->active_lines > 0) && (m->lines != vid->active_lines))
        l = displine * m->lines / vid->active_lines;
    else
        l = displine;

    if (l >= m->lines)
        return;

    off = (m->origin - 0xb0000u) + (uint32_t) (l % m->planes) * m->stride
        + (uint32_t) (l / m->planes) * m->rowbytes;

    for (x = 0; x < m->width; x++) {
        uint8_t v;
        uint8_t idx;

        switch (m->bpp) {
            case 1:
                v   = vid->vram[(off + (uint32_t) (x >> 3)) & (VAXMATE_VRAM - 1)];
                idx = (v >> (7 - (x & 7))) & 1;
                /* 前景色同样走 CPS/SIC 选中的调色板；color_select 的低 4 位
                   是边框色，拿它当前景色会让默认的 0 变成黑底黑字。 */
                buffer32->line[line][x] = vaxmate_vlt_color(
                    vid->vlt[vaxmate_pal4[((vid->color_select >> 4) & 3)][idx] & 0x0f]);
                break;

            case 2:
                v   = vid->vram[(off + (uint32_t) (x >> 2)) & (VAXMATE_VRAM - 1)];
                idx = (v >> (6 - 2 * (x & 3))) & 3;
                buffer32->line[line][x] = vaxmate_vlt_color(
                    vid->vlt[vaxmate_pal4[((vid->color_select >> 4) & 3)][idx] & 0x0f]);
                break;

            default:
                v   = vid->vram[(off + (uint32_t) (x >> 1)) & (VAXMATE_VRAM - 1)];
                idx = (v >> (4 - 4 * (x & 1))) & 0x0f;
                buffer32->line[line][x] = vaxmate_vlt_color(vid->vlt[idx]);
                break;
        }
    }
}


static void
vaxmate_video_poll(void *priv)
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;
    int lpr, vdisp_rows, row, sub, start, active, total;

    VAXMATE_MONITOR_PROLOGUE()

    /* 一条字符行占多少扫描线：文本固定 8x16 格（TRM 7-6）。
       帧高固定：文本 25 行 x 16 点，图形一律 400 行。
       CRTC 的 R6/R9 是 guest 按“行业标准时序”写的值，要经门阵列的
       4Kx8 参数翻译 ROM 才变成真正的 LAD 值；我们没建模这张表，
       直接拿 R6/R9 算帧高会得出上千行（Windows 设 0xD0 时窗口被拉长）。 */
    if (vid->mode <= VMODE_TEXT80) {
        lpr        = 16;
        vdisp_rows = 25;
    } else {
        lpr        = 1;
        vdisp_rows = 400;
    }

    if (vid->mode == VMODE_G800x252x4) {
        /* VT240 raster: pin 7 of the video connector goes high and the monitor
           switches to 250-scan operation (TRM 4-2 "400/250 select"), so 250
           lines are displayed inside the 441-line field. */
        active = 250;
        total  = VAXMATE_LINES_800;
    } else {
        active = vdisp_rows * lpr;
        total  = VAXMATE_LINES_640;
    }
    vid->active_lines = active;

    /* 03DA/03BA 的两位跟着扫描走。一行被拆成两段定时：显示段里 bit0=1，
       行消隐段里 bit0=0，于是 bit0 以行频（26 kHz）翻转；bit3 是垂直同步，
       落在帧末几行。BIOS 的滚动/光标例程靠这两位等回扫。 */
    if (vid->hphase == 0) {
        vid->status_a = 0;
        if (vid->displine < active)
            vid->status_a |= STATA_DISPENSE;
        if ((total > 3) && (vid->displine >= (total - 3)))
            vid->status_a |= STATA_VSYNC;

        vid->hphase = 1;
        timer_advance_u64(&vid->timer, vid->dispontime);
        return;
    }

    /* 行消隐段：bit0 落下，这一行随即结束。 */
    vid->status_a = 0;
    if ((total > 3) && (vid->displine >= (total - 3)))
        vid->status_a |= STATA_VSYNC;
    vid->hphase = 0;

    if (vid->displine >= active) {
        /* 垂直消隐：40 行左右，凑满 440 行 / 60 Hz */
        vid->dispon = 0;
    } else {
        vid->dispon = 1;

        row = vid->displine / lpr;
        sub = vid->displine % lpr;

        if (vid->displine == 0)
            video_wait_for_buffer();

        start = ((vid->crtc[CRTC_STARTHI] & 0x3f) << 8) | vid->crtc[CRTC_STARTLO];

        vid->scanline = sub;
        vid->memaddr  = start + (row * ((vid->mode == VMODE_TEXT40) ? 40 : 80));

        if (vid->mode <= VMODE_TEXT80)
            vaxmate_video_render_text(vid, vid->displine);
        else
            vaxmate_video_render_graphics(vid, vid->displine, vid->displine);
    }

    vid->displine++;

    if (vid->displine >= total) {
        /* 帧结束：把这一帧送屏 */
        int h = active;
        int w = vaxmate_modes[vid->mode].width;

        if (w < 320) w = 640;
        if (h < 200) h = 400;

        if ((xsize != w) || (ysize != h) || video_force_resize_get()) {
            xsize = w;
            ysize = h;
            set_screen_size(xsize, ysize);
            if (video_force_resize_get())
                video_force_resize_set(0);
        }

        video_blit_memtoscreen(0, 0, w, h);
        video_res_x = w;
        video_res_y = h;
        video_bpp   = 1;

        vax_frm_frames++;
        if (getenv("VAX_SHOT") && (vax_frm_frames == 150))
            vaxmate_shot_now();

        vid->displine = 0;
        vid->vc       = 0;
    }

    /* 行剩余的时间：848（或 1056）点里去掉显示段。行频由监视器锁定，
       与 guest 写进 CRTC 的 R0/R1 无关。 */
    timer_advance_u64(&vid->timer, vid->dispofftime);
}

static void
vaxmate_video_out(uint16_t addr, uint8_t val, void *priv)
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;
    int cur_mda = 0;

    /* MDA 窗口别名：DEC BIOS 交接后把设备字设成 0x30（MDA 80x25），
       INT 10h 于是走 3B4/3B5/3B8/3BA。真机上必须能应答，否则设模式死等。
       注意先判地址再改名 —— 否则 3BA 变成 3DA 后会被“断开”门误杀。 */
    {
        int mda = ((addr & 0xfff0) == 0x03b0);

        cur_mda = mda;

        /* 断开时仍放行 MDA 兼容窗口（3B0-3BF），其余丢弃 */
        if (vid->video_disabled && !mda)
            return;

        if (mda) {
            /* MDA 的 CRTC 是 3B4/3B5，对应“工业标准窗口” 3D4/3D5，
               不是 DIGITAL 直通的 3D0/3D1 —— 别搞错 */
            /* MDA 窗口的值已经是 DIGITAL 口径（25 行 × 14/16 线），
               不能再走 3D4/3D5 的转换 ROM（会把 R4/R6/R7 加倍、R9 强制 15），
               否则帧变成 51 行。直接走 3D0/3D1 的直通路径。 */
            if (addr == 0x3b4)
                addr = 0x3d0;
            else if (addr == 0x3b5)
                addr = 0x3d1;
            else
                addr = (uint16_t) (0x3d0 + (addr - 0x3b0));
        }
    }

    switch (addr) {
        case 0x3d0: /* DIGITAL extended CRTC index, direct */
            vid->crtcreg = val & 0x1f;
            return;

        case 0x3d1: /* DIGITAL extended CRTC data, direct */
            if (vid->crtcreg < 32) {
                vid->crtc[vid->crtcreg] = val;
                vaxmate_video_recalctimings(vid);
            }
            return;

        case 0x3d4: /* industry-standard CRTC index, through translation ROM */
            vid->crtcreg    = val & 0x1f;
            vid->status_b  |= STATB_FROM_3D4;
            return;

        case 0x3d5: /* industry-standard CRTC data, through translation ROM */
            if (vid->crtcreg < 32) {
                vid->crtc_raw[vid->crtcreg] = val;
                vaxmate_crtc_translate(vid);
                vid->write_data         = val;
                vid->status_b          |= STATB_FROM_3D4;
                vaxmate_video_recalctimings(vid);
            }
            return;

        case 0x3d8:
            /* MDA 窗口(3B8) -> 文本基址 B0000（本卡 64K 空间偏移 0）；
               DIGITAL/CGA(3D8) -> B8000（偏移 0x8000） */
            vid->text_base = cur_mda ? 0 : 0x8000;
            vid->ctrl_a = val;
            vid->mode   = vaxmate_video_mode(vid);
            vaxmate_crtc_translate(vid);
            vaxmate_video_recalctimings(vid);
            return;

        case 0x3d9:
            vid->color_select = val & 0x3f;
            return;

        case 0x3df:
            vid->ctrl_b     = val;
            vid->monitor_mode = (val & CTLB_MONITOR252) ? 1 : 0;
            vid->mode       = vaxmate_video_mode(vid);
            vaxmate_crtc_translate(vid);
            vaxmate_video_recalctimings(vid);
            return;

        default:
            break;
    }
}

static uint8_t
vaxmate_video_in(uint16_t addr, void *priv)
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;

    {
        int mda = ((addr & 0xfff0) == 0x03b0);

        if (vid->video_disabled && !mda)
            return 0xff;

        if (mda) {
            /* MDA 的 CRTC 是 3B4/3B5，对应“工业标准窗口” 3D4/3D5，
               不是 DIGITAL 直通的 3D0/3D1 —— 别搞错 */
            /* MDA 窗口的值已经是 DIGITAL 口径（25 行 × 14/16 线），
               不能再走 3D4/3D5 的转换 ROM（会把 R4/R6/R7 加倍、R9 强制 15），
               否则帧变成 51 行。直接走 3D0/3D1 的直通路径。 */
            if (addr == 0x3b4)
                addr = 0x3d0;
            else if (addr == 0x3b5)
                addr = 0x3d1;
            else
                addr = (uint16_t) (0x3d0 + (addr - 0x3b0));
        }
    }

    switch (addr) {
        case 0x3d0:
            return vid->crtcreg;

        case 0x3d1:
            return vid->crtc[vid->crtcreg];

        case 0x3d4:
            return vid->crtcreg;

        case 0x3d5:
            return vid->crtc[vid->crtcreg];

        case 0x3da:
            return vid->status_a | 0xf0;

        case 0x3dd:
            return vid->status_b | 0xfc;

        case 0x3de:
            vid->status_b &= ~STATB_FROM_3D4;
            return vid->write_data;

        default:
            break;
    }

    return 0xff;
}

/* Special purpose register at 0C80 (TRM page 7-23). Only the video related
   bit is meaningful for this device; the diskette status bits are driven by
   the machine layer. */
static void
vaxmate_spr_out(uint16_t addr, uint8_t val, void *priv)
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;

    (void) addr;

    int was = vid->video_disabled;

    vid->spr            = val;
    /* bit3 是“断开视频”位（POST 写 00，视频正常；写 08 才关闭）。 */
    vid->video_disabled = (val & SPR_DISABLE_VIDEO) ? 1 : 0;
    if (getenv("VIDEO_FORCE_ON"))
        vid->video_disabled = 0;   /* 实验：忽略断开位 */
    if (getenv("VIDEO_LOG"))
        fprintf(stderr, "[SPR] val=%02X dis=%d CS=%04X PC=%06X DS=%04X ES=%04X\n",
                val, vid->video_disabled, (unsigned) cpu_state.seg_cs.seg,
                (unsigned) cpu_state.pc, (unsigned) cpu_state.seg_ds.seg,
                (unsigned) cpu_state.seg_es.seg);

    (void) was;
    /* 映射常驻：断开时由 read/write 返回 0xFF / 丢弃。
       若真的 mem_mapping_disable，访问会回落到机器层的内部 RAM，
       BIOS 的探测写 0x5A 就能读回 0x5A，被误判成“工业标准卡”。 */
}

static uint8_t
vaxmate_spr_in(uint16_t addr, void *priv)
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;

    (void) addr;

    return vid->spr;
}

static void *
vaxmate_video_init(UNUSED(const device_t *info))
{
    vaxmate_video_t *vid = calloc(1, sizeof(vaxmate_video_t));
    int              c;

    video_inform(VIDEO_FLAG_TYPE_MDA, &timing_vaxmate);

    vid->monitor_index = monitor_index_global;
    vid->ctrl_a        = CTLA_DISPLAY_EN | VMODE_TEXT80;
    vid->ctrl_b        = CTLB_DISPLAY_EN | CTLB_SAVER;
    vid->mode          = VMODE_TEXT80;
    vid->spr           = SPR_DISABLE_VIDEO; /* video enabled at power-on */
    vid->video_disabled = 0;
    vid->dispon        = 1;
    vaxmate_vid_global = vid;
    vax_init_count++;

    /* power-on VLT: identity IRGB ramp (Table 7-4 approximation) */
    for (c = 0; c < VAXMATE_VLT; c++)
        vid->vlt[c] = (uint8_t) (c & 0x0f);

    /* power-on font: MDA 8x16 ROM font */
    for (c = 0; c < 256; c++)
        memcpy(&vid->font_ram[c * 16], fontdatm[c], 16);

    /* 显存必须显式改成 INTERNAL 状态，否则被机型层的 ROM/外部状态压住，
       读写都落到 0xFF（与 0xD0000 私有 RAM 同一个坑）。 */
    mem_set_mem_state_both(0x000b0000, VAXMATE_VRAM,
                           MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);

    mem_mapping_add(&vid->mapping, 0xb0000, VAXMATE_VRAM,
                    vaxmate_video_read, NULL, NULL,
                    vaxmate_video_write, NULL, NULL,
                    NULL, MEM_MAPPING_INTERNAL, vid);

    io_sethandler(0x03b0, 0x0010,
                  vaxmate_video_in, NULL, NULL,
                  vaxmate_video_out, NULL, NULL,
                  vid);

    io_sethandler(0x03d0, 0x0010,
                  vaxmate_video_in, NULL, NULL,
                  vaxmate_video_out, NULL, NULL,
                  vid);



    io_sethandler(0x0c80, 0x0001,
                  vaxmate_spr_in, NULL, NULL,
                  vaxmate_spr_out, NULL, NULL,
                  vid);

    /* Power-on CRTC = the BIOS's own 80x25 text block (ROM F000:F13D + 0x10,
       programmed by INT 10h AH=00 at F000:E4F1), so frames come out with real
       hardware timing before the BIOS touches the CRTC: 25 rows x 16 scan
       lines plus R5=8 vertical adjust = the 400-line field. */
    for (c = 0; c < 12; c++)
        vid->crtc_raw[c] = vaxmate_crtc_blocks[1][c];
    vaxmate_crtc_translate(vid);

    vid->cursorvisible = 0;
    vid->cursoron      = 1;
    vid->firstline     = 1000;
    vid->displine      = 0;
    vid->active_lines  = 400;
    vid->hphase        = 0;
    vid->memaddr       = 0;
    vid->text_base     = 0x8000;

    /* VLT 上电默认值：BIOS 在 MDA 路径下不写 VLT，全 0 会让前景背景
       都是黑色，画面看着就是全黑。给一个最小可用调色板（0 黑、其余亮）。 */
    {
        int i;
        vid->vlt[0] = 0x00;
        for (i = 1; i < VAXMATE_VLT; i++)
            vid->vlt[i] = 0x0e;
    }

    vaxmate_video_recalctimings(vid);
    timer_add(&vid->timer, vaxmate_video_poll, vid, 1);

    return vid;
}

static void
vaxmate_video_close(void *priv)
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;

    if (!vid)
        return;

    free(vid);
}

static void
vaxmate_video_speed_changed(UNUSED(void *priv))
{
    vaxmate_video_t *vid = (vaxmate_video_t *) priv;

    vaxmate_video_recalctimings(vid);
}

const device_t vaxmate_video_device = {
    .name          = "DEC VAXmate video controller",
    .internal_name = "vaxmate_video",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = vaxmate_video_init,
    .close         = vaxmate_video_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = vaxmate_video_speed_changed,
    .force_redraw  = NULL,
    .config        = NULL
};

/* 临时：把当前渲染缓冲导出成 PPM，便于离屏检查画面 */
void
vaxmate_video_dump(int w, int h)
{
    FILE *f;
    int   x, y;

    if ((vaxmate_vid_global == NULL) || (buffer32 == NULL))
        return;

    fprintf(stderr, "[VIDDBG] mode=%d ctrl_a=%02X ctrl_b=%02X memaddr=%04X "
            "R0=%02X R1=%02X R9=%02X disp=%d dis=%d vram0=%02X%02X\n",
            vaxmate_vid_global->mode, vaxmate_vid_global->ctrl_a,
            vaxmate_vid_global->ctrl_b, vaxmate_vid_global->memaddr,
            vaxmate_vid_global->crtc[0], vaxmate_vid_global->crtc[1],
            vaxmate_vid_global->crtc[9], vaxmate_vid_global->dispon,
            vaxmate_vid_global->video_disabled,
            vaxmate_vid_global->vram[0], vaxmate_vid_global->vram[1]);
    if (getenv("VAX_CRTC")) {
        int i;
        {
            int i, nz = 0;
            for (i = 0; i < 4096; i++)
                if (vaxmate_vid_global->font_ram[i]) nz++;
            fprintf(stderr, "[FONT] nonzero=%d fw=%llu vw=%llu ctrl_b=%02X\n", nz,
                    (unsigned long long) vax_fw, (unsigned long long) vax_vw,
                    vaxmate_vid_global->ctrl_b);
            fprintf(stderr, "[FONT] first=%02X%02X%02X%02X\n",
                    vaxmate_vid_global->font_ram[0], vaxmate_vid_global->font_ram[1],
                    vaxmate_vid_global->font_ram[2], vaxmate_vid_global->font_ram[3]);
        }
        fprintf(stderr, "[PTR] timer=%p global=%p\n", (void *) vax_timer_vid, (void *) vaxmate_vid_global);
        fprintf(stderr, "[FRM] lines=%llu rows=%llu frames=%llu vc=%d disp=%d vadj=%d scan=%d dline=%d vt=%02X vd=%02X vs=%02X resets=%llu\n",
                (unsigned long long) vax_frm_lines, (unsigned long long) vax_frm_rows,
                (unsigned long long) vax_frm_frames, vaxmate_vid_global->vc,
                vaxmate_vid_global->dispon, vaxmate_vid_global->vadj,
                vaxmate_vid_global->scanline, vaxmate_vid_global->displine,
                vaxmate_vid_global->crtc[4], vaxmate_vid_global->crtc[6],
                vaxmate_vid_global->crtc[7], (unsigned long long) vax_frm_vcresets);
        fprintf(stderr, "[CRTC]");
        for (i = 0; i < 16; i++)
            fprintf(stderr, " %02X", vaxmate_vid_global->crtc[i]);
        fprintf(stderr, " vc=%d scan=%d\n", vaxmate_vid_global->vc, vaxmate_vid_global->scanline);
    }

    f = fopen("/tmp/vx_shot.ppm", "wb");
    if (f == NULL)
        return;

    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint32_t c = buffer32->line[y][x];
            fputc((c >> 16) & 0xff, f);
            fputc((c >> 8) & 0xff, f);
            fputc(c & 0xff, f);
        }
    }
    fclose(f);
    rename("/tmp/vx_shot_tmp.ppm", "/tmp/vx_shot2.ppm");
}

static void
vaxmate_shot_now(void)
{
    FILE *f = fopen("/tmp/vx_shot_tmp.ppm", "wb");
    int x, y;
    if ((f == NULL) || (buffer32 == NULL)) return;
    fprintf(f, "P6\n640 400\n255\n");
    for (y = 0; y < 400; y++)
        for (x = 0; x < 640; x++) {
            uint32_t c = buffer32->line[y][x];
            fputc((c >> 16) & 0xff, f); fputc((c >> 8) & 0xff, f); fputc(c & 0xff, f);
        }
    fclose(f);
}
