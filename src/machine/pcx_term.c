/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-X: terminal side of the video board, see the header for
 *          what is modelled and what is not.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <86box/86box.h>
#include <86box/plat_unused.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/pcx_term.h>

/* Control codes the BIOS stream uses.  The message table at v3 0x0470-0x05c0
   terminates every line with CR LF and every message with ETX (0x03). */
#define PCX_TERM_ETX 0x03
#define PCX_TERM_BEL 0x07
#define PCX_TERM_BS  0x08
#define PCX_TERM_TAB 0x09
#define PCX_TERM_LF  0x0a
#define PCX_TERM_CR  0x0d

static void
pcx_term_clear(pcx_term_t *term)
{
    memset(term->cells, ' ', sizeof(term->cells));
    memset(term->attr, 0x0f, sizeof(term->attr));
}

static void
pcx_term_scroll(pcx_term_t *term)
{
    memmove(&term->cells[0][0], &term->cells[1][0],
            (PCX_TERM_ROWS - 1) * PCX_TERM_COLS);
    memmove(&term->attr[0][0], &term->attr[1][0],
            (PCX_TERM_ROWS - 1) * PCX_TERM_COLS);
    memset(&term->cells[PCX_TERM_ROWS - 1][0], ' ', PCX_TERM_COLS);
    memset(&term->attr[PCX_TERM_ROWS - 1][0], 0x0f, PCX_TERM_COLS);
    term->row = PCX_TERM_ROWS - 1;
}

static void
pcx_term_newline(pcx_term_t *term)
{
    term->row++;
    if (term->row >= PCX_TERM_ROWS)
        pcx_term_scroll(term);
}

void
pcx_term_init(pcx_term_t *term)
{
    if (term == NULL)
        return;

    memset(term, 0x00, sizeof(*term));
    pcx_term_clear(term);
    term->col      = 0;
    term->row      = 0;
    term->dirty    = 1;
}

int
pcx_term_load_font(pcx_term_t *term, const char *path)
{
    FILE *fp;
    long  size;
    int   ret = 0;

    if ((term == NULL) || (path == NULL))
        return 0;

    fp = rom_fopen((char *) path, "rb");
    if (fp == NULL)
        return 0;

    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (size == PCX_TERM_FONT_SIZE) {
        if (fread(term->font, 1, PCX_TERM_FONT_SIZE, fp) == PCX_TERM_FONT_SIZE) {
            term->font_loaded = 1;
            term->dirty       = 1;
            ret               = 1;
        }
    }

    fclose(fp);

    return ret;
}

void
pcx_term_write(pcx_term_t *term, uint8_t val)
{
    if ((term == NULL) || (term->row < 0) || (term->row >= PCX_TERM_ROWS))
        return;

    switch (val) {
        case PCX_TERM_ETX: /* end of message, nothing to place */
            break;

        case PCX_TERM_CR:
            term->col = 0;
            break;

        case PCX_TERM_LF:
            pcx_term_newline(term);
            break;

        case PCX_TERM_BS:
            if (term->col > 0)
                term->col--;
            break;

        case PCX_TERM_TAB:
            term->col = (term->col + 8) & ~7;
            if (term->col >= PCX_TERM_COLS) {
                term->col = 0;
                pcx_term_newline(term);
            }
            break;

        case PCX_TERM_BEL:
            break;

        default:
            if (val < 0x20)
                break;

            term->cells[term->row][term->col] = val;
            term->attr[term->row][term->col]  = 0x0f;
            term->dirty                       = 1;

            term->col++;
            if (term->col >= PCX_TERM_COLS) {
                term->col = 0;
                pcx_term_newline(term);
            }
            break;
    }
}

uint8_t
pcx_term_cell(pcx_term_t *term, int row, int col)
{
    if ((term == NULL) || (row < 0) || (row >= PCX_TERM_ROWS) ||
        (col < 0) || (col >= PCX_TERM_COLS))
        return 0x00;

    return term->cells[row][col];
}

void
pcx_term_render(pcx_term_t *term, uint32_t *out, int width, int height,
                uint32_t fg, uint32_t bg)
{
    int      x, y, cx, cy, bit;
    uint8_t  ch;
    const uint8_t *glyph;

    if ((term == NULL) || (out == NULL) || (width <= 0) || (height <= 0))
        return;

    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++)
            out[y * width + x] = bg;

    for (cy = 0; cy < PCX_TERM_ROWS; cy++) {
        for (cx = 0; cx < PCX_TERM_COLS; cx++) {
            ch = term->cells[cy][cx];
            if (ch == ' ')
                continue;

            if (ch >= PCX_TERM_CHARS)
                ch = '?' - 0x20;

            /* 8x14 cell, 16 bytes per character: the first 14 bytes are the
               rows, bit 7 is the leftmost pixel. */
            glyph = &term->font[(ch * 16) & (PCX_TERM_FONT_SIZE - 1)];

            for (y = 0; y < PCX_TERM_CELL_H; y++) {
                int py = cy * PCX_TERM_CELL_H + y;

                if (py >= height)
                    break;

                for (x = 0; x < PCX_TERM_CELL_W; x++) {
                    int px = cx * PCX_TERM_CELL_W + x;

                    if (px >= width)
                        break;

                    bit = term->font_loaded ? ((glyph[y] >> (7 - x)) & 1) : 0;

                    if (bit)
                        out[py * width + px] = fg;
                }
            }
        }
    }

    term->dirty = 0;
}
