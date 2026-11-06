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
#define PCX_TERM_SO  0x0e
#define PCX_TERM_SI  0x0f
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

    /* Only the screen state is reset here: the font is loaded once at reset
       and must survive a later mode set (the board clears the page when the
       BIOS hands it the setup block). */
    pcx_term_clear(term);
    term->col      = 0;
    term->row      = 0;
    term->dirty    = 1;
    term->esc      = 0;
    term->npar     = 0;
    term->priv     = 0;
    term->g1       = 0;
    memset(term->par, 0, sizeof(term->par));
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

/* Blanks a run of cells on one row.  Every erase goes through here so the
   dirty flag and the attribute byte stay in step with the characters. */
static void
pcx_term_erase(pcx_term_t *term, int row, int col0, int col1)
{
    if ((row < 0) || (row >= PCX_TERM_ROWS))
        return;
    if (col0 < 0)
        col0 = 0;
    if (col1 >= PCX_TERM_COLS)
        col1 = PCX_TERM_COLS - 1;

    for (int c = col0; c <= col1; c++) {
        if ((term->cells[row][c] != ' ') || (term->attr[row][c] != 0x0f))
            term->dirty = 1;
        term->cells[row][c] = ' ';
        term->attr[row][c]  = 0x0f;
    }
}

/* Clamps the cursor after a command that moved it. */
static void
pcx_term_clamp(pcx_term_t *term)
{
    if (term->row < 0)
        term->row = 0;
    if (term->row >= PCX_TERM_ROWS)
        term->row = PCX_TERM_ROWS - 1;
    if (term->col < 0)
        term->col = 0;
    if (term->col >= PCX_TERM_COLS)
        term->col = PCX_TERM_COLS - 1;
}

/* Final byte of a CSI sequence.  The 8031 implements the cursor and erase
   subset below; everything else is consumed and dropped, which is what the
   board does with commands its firmware does not know either. */
static void
pcx_term_csi(pcx_term_t *term, uint8_t final)
{
    int p0 = (term->npar > 0) ? term->par[0] : 0;
    int p1 = (term->npar > 1) ? term->par[1] : 0;
    int r;

    term->esc = 0;

    /* '?' prefixes a mode the board has no state for (cursor visible and the
       like).  Swallow it rather than mistaking its final byte for a command. */
    if (term->priv)
        return;

    switch (final) {
        case 'H':
        case 'f':
            term->row = ((p0 > 0) ? p0 : 1) - 1;
            term->col = ((p1 > 0) ? p1 : 1) - 1;
            break;

        case 'A':
            term->row -= (p0 > 0) ? p0 : 1;
            break;
        case 'B':
            term->row += (p0 > 0) ? p0 : 1;
            break;
        case 'C':
            term->col += (p0 > 0) ? p0 : 1;
            break;
        case 'D':
            term->col -= (p0 > 0) ? p0 : 1;
            break;
        case 'G':
            term->col = ((p0 > 0) ? p0 : 1) - 1;
            break;
        case 'd':
            term->row = ((p0 > 0) ? p0 : 1) - 1;
            break;

        case 'J':
            if (p0 == 0) {
                pcx_term_erase(term, term->row, term->col, PCX_TERM_COLS - 1);
                for (r = term->row + 1; r < PCX_TERM_ROWS; r++)
                    pcx_term_erase(term, r, 0, PCX_TERM_COLS - 1);
            } else if (p0 == 1) {
                for (r = 0; r < term->row; r++)
                    pcx_term_erase(term, r, 0, PCX_TERM_COLS - 1);
                pcx_term_erase(term, term->row, 0, term->col);
            } else {
                for (r = 0; r < PCX_TERM_ROWS; r++)
                    pcx_term_erase(term, r, 0, PCX_TERM_COLS - 1);
                term->row = 0;
                term->col = 0;
            }
            break;

        case 'K':
            if (p0 == 0)
                pcx_term_erase(term, term->row, term->col, PCX_TERM_COLS - 1);
            else if (p0 == 1)
                pcx_term_erase(term, term->row, 0, term->col);
            else
                pcx_term_erase(term, term->row, 0, PCX_TERM_COLS - 1);
            break;

        case 'm': /* attributes: the page is monochrome, nothing to place */
        default:
            break;
    }

    pcx_term_clamp(term);
}

/* Raw stream log.  The escape parser below is where the character set gets
   selected, and a designator that is dropped here is invisible in the page
   dump: enable PCX_RAWLOG to see the bytes exactly as the kernel sends them. */
static void
pcx_term_raw(uint8_t val)
{
    static int      on      = -1;
    static uint32_t offset  = 0;
    static char     hex[32 * 3 + 1];
    static char     txt[32 + 1];
    static int      n       = 0;

    if (on < 0)
        on = (getenv("PCX_RAWLOG") != NULL);

    if (!on || (offset >= 0x40000))
        return;

    snprintf(&hex[n * 3], 4, "%02X ", val);
    txt[n] = ((val >= 0x20) && (val < 0x7F)) ? (char) val : '.';
    n++;

    if ((n == 32) || (val == 0x0A)) {
        hex[n * 3] = '\0';
        txt[n]     = '\0';
        pclog("[RAW] %06X: %s| %s\n", offset - n + 1, hex, txt);
        n = 0;
    }
    offset++;
}

void
pcx_term_write(pcx_term_t *term, uint8_t val)
{
    pcx_term_raw(val);

    if ((term == NULL) || (term->row < 0) || (term->row >= PCX_TERM_ROWS))
        return;


    if (term->esc != 0) {
        if (term->esc == 1) {
            switch (val) {
                case '[':
                    term->esc  = 2;
                    term->npar = 0;
                    term->priv = 0;
                    memset(term->par, 0, sizeof(term->par));
                    break;
                case '(':
                case ')':
                case '#':
                case '%':
                case '*':
                case '+':
                    /* Designator for a character set or a keyboard mode: one
                       more byte follows and neither is ours to act on. */
                    term->esc = 3;
                    break;
                case 'c': /* RIS: back to the power-on page */
                    pcx_term_init(term);
                    break;
                case 'D':
                    pcx_term_newline(term);
                    break;
                case 'E':
                    term->col = 0;
                    pcx_term_newline(term);
                    break;
                case 'M': /* reverse index */
                    if (term->row > 0)
                        term->row--;
                    break;
                default:
                    break;
            }
            if (val != '[')
                term->esc = (term->esc == 3) ? 3 : 0;
            return;
        }

        if (term->esc == 3) {
            term->esc = 0;
            return;
        }

        /* Inside CSI. */
        if ((val >= '0') && (val <= '9')) {
            if (term->npar == 0) {
                term->npar    = 1;
                term->par[0]  = 0;
            }
            if (term->npar <= 4)
                term->par[term->npar - 1] =
                    term->par[term->npar - 1] * 10 + (val - '0');
        } else if (val == ';') {
            if (term->npar == 0)
                term->npar = 1;
            if (term->npar < 4) {
                term->npar++;
                term->par[term->npar - 1] = 0;
            }
        } else if (val == '?') {
            term->priv = 1;
        } else if ((val >= 0x20) && (val <= 0x2f)) {
            /* Intermediate byte, part of the sequence. */
        } else if ((val >= 0x40) && (val <= 0x7e)) {
            pcx_term_csi(term, val);
        } else {
            term->esc = 0;
        }
        return;
    }

    if (val == 0x1b) {
        term->esc = 1;
        return;
    }

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

        case PCX_TERM_SO: /* shift out: the graphics set */
            term->g1 = 1;
            break;

        case PCX_TERM_SI: /* shift in: back to ASCII */
            term->g1 = 0;
            break;

        default:
            if (val < 0x20)
                break;

            if (term->g1 && (val >= 0x60) && (val <= 0x7f)) {
                /* In the graphics set 0x60-0x6f are the box drawing glyphs
                   at character generator slots 0x190-0x19f. */
                term->cells[term->row][term->col] = (uint8_t) (0x90 | (val & 0x0f));
                term->attr[term->row][term->col]  = 0x0f | PCX_TERM_ATTR_BANK;
            } else {
                term->cells[term->row][term->col] = val;
                term->attr[term->row][term->col]  = 0x0f;
            }
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
    uint16_t ch;
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

            if (term->attr[cy][cx] & PCX_TERM_ATTR_BANK)
                ch += 0x100;

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
