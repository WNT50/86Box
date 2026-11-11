/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Philips :YES video: an HD6845S whose character cells live in the
 *          top of RAM.  The ROM places the frame buffer with the 80186's
 *          MMCS register, so the base moves at boot; the machine tells this
 *          device where it is.  Each cell is two bytes, character then
 *          attribute, and the character generator is the board's own 8 KB
 *          ROM (512 glyphs of 8x16).
 *
 *          Only the text mode is modelled: the pixel graphics mode the
 *          configurator can enable is not.
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
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/plat.h>
#include <86box/rom.h>
#include <86box/timer.h>
#include <86box/video.h>
#include <86box/yes_video.h>

#define YES_XSIZE 640
#define YES_YSIZE 400
#define YES_FREQ  50

/* HD6845S register numbers used here. */
#define CRTC_R1  1  /* horizontal displayed, in characters */
#define CRTC_R6  6  /* vertical displayed, in character rows */
#define CRTC_R4  4  /* offset between rows, in characters */
#define CRTC_R9  9  /* scan lines per row, minus one */
#define YES_PAGE_COLS 80    /* cells in a page row */
#define YES_VISIBLE_ROWS 25 /* rows the panel shows; the CRTC may name one more */
#define CRTC_R12 12 /* start address, high */
#define CRTC_R13 13 /* start address, low */
#define CRTC_R14 14 /* cursor address, high */
#define CRTC_R15 15 /* cursor address, low */

typedef struct yes_video_t {
    int      cols;    /* 40 or 80, from the mode register (port 0x5e0 bit 0) */
    uint8_t  crtc[32];
    uint8_t  crtc_idx;
    uint8_t  pal[16];
    uint8_t  font[8192];
    int      font_ok;
    uint32_t base;      /* frame buffer, from the machine's MMCS write */
    uint32_t fb[YES_XSIZE * YES_YSIZE];
    pc_timer_t timer;
} yes_video_t;

static video_timings_t timing_yes = { VIDEO_ISA, 8, 16, 32, 8, 16, 32 };
static yes_video_t    *yes_video;

/* IRGB: bit 3 is intensity, then red, green and blue. */
static uint32_t
yes_colour(uint8_t v)
{
    uint32_t i = (v & 0x08) ? 0xff : 0xb0;
    uint32_t r = (v & 0x04) ? i : 0x00;
    uint32_t g = (v & 0x02) ? i : 0x00;
    uint32_t b = (v & 0x01) ? i : 0x00;

    return (r << 16) | (g << 8) | b;
}

static void
yes_crtc_addr_write(uint16_t port, uint8_t val, void *priv)
{
    yes_video_t *dev = (yes_video_t *) priv;

    dev->crtc_idx = val & 0x1f;
}

static void
yes_crtc_data_write(uint16_t port, uint8_t val, void *priv)
{
    yes_video_t *dev = (yes_video_t *) priv;

    dev->crtc[dev->crtc_idx] = val;
}

static uint8_t
yes_crtc_data_read(uint16_t port, void *priv)
{
    yes_video_t *dev = (yes_video_t *) priv;

    return dev->crtc[dev->crtc_idx];
}

static void
yes_palette_write(uint16_t port, uint8_t val, void *priv)
{
    yes_video_t *dev = (yes_video_t *) priv;
    int          i   = (port - 0x05c0) >> 1;

    if (i < 16)
        dev->pal[i] = val & 0x0f;
}

static void
yes_render(yes_video_t *dev)
{
    int      cols  = dev->cols;
    int      rows  = dev->crtc[CRTC_R6] + 1;
    uint32_t start = ((uint32_t) dev->crtc[CRTC_R12] << 8) | dev->crtc[CRTC_R13];
    uint32_t cursor = ((uint32_t) dev->crtc[CRTC_R14] << 8) | dev->crtc[CRTC_R15];
    int      row, col, y, x;
    uint32_t page_stride;
    /* R9 says how many scan lines a character row takes, minus one. */
    int      lh = dev->crtc[CRTC_R9] + 1;
    /* The column count is not in the CRTC: R1 stays 0x28 in both text modes
       because the board halves or doubles the character clock instead.  The
       mode register decides -- 40 columns put a character in sixteen pixels,
       80 columns in eight, and the glyph keeps its eight dots either way. */
    int      cw;

    if ((cols != 40) && (cols != 80))
        cols = 80;
    cw = YES_XSIZE / cols;
    if (lh < 8)
        lh = 8;
    if (lh > 16)
        lh = 16;
    if ((rows < 1) || (rows > (YES_YSIZE / lh)))
        rows = YES_YSIZE / lh;

    memset(dev->fb, 0x00, YES_XSIZE * YES_YSIZE * sizeof(uint32_t));

    /* The page is laid out eighty cells to a row whatever the mode displays;
       R4 is the 6845's vertical total, not a row stride (it reads 0x19 = 25 on
       this machine).  Walking the page with the displayed width slid every row
       and split the prompt. */
    page_stride = YES_PAGE_COLS;

    for (row = 0; row < rows; row++) {
        for (col = 0; col < cols; col++) {
            /* The board's address lines start at A1: the start register
               counts characters, each character takes four bytes of the
               window, and the cells themselves are 2 bytes apart. */
            uint32_t cell = (uint32_t) row * page_stride + (uint32_t) col;
            uint32_t addr = dev->base + start * 4u + cell * 2u;
            uint8_t  ch, attr;
            uint32_t fg, bg;
            const uint8_t *glyph;
            uint32_t colour;

            if (addr + 1u >= (1u << 20))
                continue;

            ch   = ram[addr & 0xfffff];
            attr = ram[(addr + 1u) & 0xfffff];

            /* Attribute: low nibble is the foreground, bits 4-6 the
               background.  The configurator calls the palette "IRGB", and
               the ROM's own writes are consistent with the low nibble
               indexing it. */
            fg = yes_colour(dev->pal[attr & 0x0f]);
            bg = yes_colour(dev->pal[(attr >> 4) & 0x07]);
            if (dev->crtc[CRTC_R9] == 0x0d)
                glyph = &dev->font[(ch & 0xff) * 16];
            else
                glyph = &dev->font[(ch & 0xff) * 16];

            for (y = 0; y < lh; y++) {
                uint32_t *line = &dev->fb[((uint32_t) row * (uint32_t) lh + (uint32_t) y) * YES_XSIZE];
                uint8_t bits = dev->font_ok ? glyph[y] : 0x00;

                if (!dev->font_ok)
                    bits = 0x18; /* a marker so a missing font is visible */

                for (x = 0; x < cw; x++) {
                    /* The 6845 shifts eight dots per character, so a sixteen
                       dot cell means the board halves the character clock:
                       every glyph dot is displayed twice.  Padding the cell
                       with background instead left the letters thin with wide
                       gaps, which is not how the narrow mode looks. */
                    int gx = x / (cw / 8);

                    colour = ((gx < 8) && (bits & (0x80 >> gx))) ? fg : bg;
                    if (((uint32_t) col * (uint32_t) cw + (uint32_t) x) < YES_XSIZE)
                        line[(uint32_t) col * (uint32_t) cw + (uint32_t) x] = colour;
                }
            }

            /* A block cursor.  R14/R15 hold the same kind of address as the
               start register, so compare against start + cell, not cell. */
            if ((start + cell) == cursor) {
                for (y = lh - 3; y < lh; y++) {
                    uint32_t *line = &dev->fb[((uint32_t) row * (uint32_t) lh + (uint32_t) y) * YES_XSIZE];
                    for (x = 0; x < 8; x++)
                        line[(uint32_t) col * (uint32_t) cw + (uint32_t) x] = fg;
                }
            }
        }
    }
}

static void
yes_poll(void *priv)
{
    yes_video_t *dev = (yes_video_t *) priv;
    int          y;

    yes_render(dev);

    /* YES_VIDSHOT=<file>: write one PPM of the text plane, so the picture can
       be looked at without an X window. */
    {
        static int frames;
        const char *shot = getenv("YES_VIDSHOT");

        if ((shot != NULL) && (++frames == 500)) {
            FILE *f = fopen(shot, "wb");

            if (f != NULL) {
                fprintf(f, "P6\n%d %d\n255\n", YES_XSIZE, YES_YSIZE);
                for (y = 0; y < YES_YSIZE; y++) {
                    for (int x = 0; x < YES_XSIZE; x++) {
                        uint32_t c = dev->fb[y * YES_XSIZE + x];
                        unsigned char rgb[3] = { (unsigned char) (c >> 16),
                                                 (unsigned char) (c >> 8),
                                                 (unsigned char) c };
                        fwrite(rgb, 1, 3, f);
                    }
                }
                fclose(f);

                /* The raw cells next to the picture, for offline calibration. */
                f = fopen("/tmp/yes_cells.bin", "wb");
                if (f != NULL) {
                    uint32_t st = ((uint32_t) dev->crtc[CRTC_R12] << 8) | dev->crtc[CRTC_R13];
                    fwrite(ram + ((dev->base + st * 4u) & 0xfffff), 1, 8192, f);
                    fclose(f);
                }

                pclog(":YES video: CRTC R0=%02X R1=%02X R2=%02X R3=%02X R4=%02X R5=%02X R6=%02X R9=%02X\n",
                      dev->crtc[0], dev->crtc[1], dev->crtc[2], dev->crtc[3],
                      dev->crtc[4], dev->crtc[5], dev->crtc[6], dev->crtc[9]);
                pclog(":YES video: wrote %s (R1=%02X R6=%02X R9=%02X R12/13=%02X%02X R14/15=%02X%02X)\n",
                      shot, dev->crtc[CRTC_R1], dev->crtc[CRTC_R6], dev->crtc[CRTC_R9],
                      dev->crtc[CRTC_R12], dev->crtc[CRTC_R13],
                      dev->crtc[CRTC_R14], dev->crtc[CRTC_R15]);
            }
        }
    }

    for (y = 0; y < YES_YSIZE; y++)
        memcpy(buffer32->line[y], &dev->fb[y * YES_XSIZE], YES_XSIZE * 4);
    {
        int vh = (dev->crtc[CRTC_R6] + 1) * (dev->crtc[CRTC_R9] + 1);

        if ((vh < 16) || (vh > YES_YSIZE))
            vh = YES_YSIZE;
        /* The panel is 25 rows tall while the CRTC names 26, and the firmware
           only fills the 25 it will show: the last one keeps whatever the RAM
           held, which is the line of coloured blocks that used to sit under
           the prompt.  Truncate the picture here, the way the 3477-S does,
           instead of pretending the CRTC asked for one row less. */
        if (vh > (YES_VISIBLE_ROWS * (dev->crtc[CRTC_R9] + 1)))
            vh = YES_VISIBLE_ROWS * (dev->crtc[CRTC_R9] + 1);
        video_blit_memtoscreen(0, 0, YES_XSIZE, vh);
    }

    timer_advance_u64(&dev->timer, TIMER_USEC * (1000000ULL / YES_FREQ));
}

static void *
yes_video_init(const device_t *info)
{
    yes_video_t *dev;
    FILE        *fp;
    int          i;

    dev = (yes_video_t *) calloc(1, sizeof(yes_video_t));
    if (dev == NULL)
        return NULL;

    yes_video = dev;
    dev->base = 0x00010000;
    dev->cols = 80;

    for (i = 0; i < 16; i++)
        dev->pal[i] = (uint8_t) (0x0f & ~i); /* a visible default until the ROM writes */

    fp = rom_fopen("roms/machines/yes/yes_23502.bin", "rb");
    if (fp != NULL) {
        dev->font_ok = (fread(dev->font, 1, sizeof(dev->font), fp) == sizeof(dev->font));
        fclose(fp);
    }
    if (!dev->font_ok)
        pclog(":YES video: character generator not found, glyphs will be blank\n");

    overscan_x = 0;
    overscan_y = 0;
    video_inform(VIDEO_FLAG_TYPE_MDA, &timing_yes);

    io_sethandler(0x0500, 0x0001, NULL, NULL, NULL, yes_crtc_addr_write, NULL, NULL, dev);
    io_sethandler(0x0502, 0x0001, yes_crtc_data_read, NULL, NULL, yes_crtc_data_write, NULL, NULL, dev);
    io_sethandler(0x05c0, 0x0010, NULL, NULL, NULL, yes_palette_write, NULL, NULL, dev);

    timer_add(&dev->timer, yes_poll, dev, 1);

    return dev;
}

static void
yes_video_close(void *priv)
{
    yes_video_t *dev = (yes_video_t *) priv;

    if (dev != NULL) {
        yes_video = NULL;
        free(dev);
    }
}

/* The machine calls this whenever the ROM rewrites MMCS. */
/* Port 0x5e0 bit 0 picks the text width: set is 40x25, clear is 80x25 (the
   data sheet of the machine, not the CRTC, decides it). */
void
yes_video_set_columns(void *priv, int cols)
{
    yes_video_t *dev = (yes_video_t *) priv;

    if (dev == NULL)
        return;

    dev->cols = (cols == 40) ? 40 : 80;
}

void
yes_video_set_base(void *priv, uint32_t base)
{
    yes_video_t *dev = (yes_video_t *) priv;

    if (dev != NULL)
        dev->base = base & 0x000fffffu;
}

const device_t yes_video_device = {
    .name          = "Philips :YES video",
    .internal_name = "yes_video",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = yes_video_init,
    .close         = yes_video_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
