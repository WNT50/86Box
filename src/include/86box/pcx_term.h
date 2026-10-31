/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-X: terminal side of the video board.
 *
 *          On the real board an 8031 takes the byte stream the host writes to
 *          0xf980, works out the cursor and the attributes, and feeds the
 *          SCN2672 through the character and attribute RAM.  This module models
 *          the terminal behaviour only -- the byte stream, the 80x25 page and
 *          the character generator -- so that the POST messages the BIOS sends
 *          ("*** CRT-CONTR. DEFECT ***", "*** MEMORY-ERROR FOUND *" and the
 *          rest of the v3 table at 0x0470-0x05c0) show up on screen.
 *
 *          The character generator is the board's own d12-graka ROM: 512
 *          characters, 8x14 cells, 16 bytes per character.
 */

#ifndef EMU_PCX_TERM_H
#define EMU_PCX_TERM_H

#include <stdint.h>

#define PCX_TERM_COLS 80
#define PCX_TERM_ROWS 25
#define PCX_TERM_CELL_W 8
#define PCX_TERM_CELL_H 14
#define PCX_TERM_CHARS 512
#define PCX_TERM_FONT_SIZE (PCX_TERM_CHARS * 16)

typedef struct pcx_term_t {
    uint8_t cells[PCX_TERM_ROWS][PCX_TERM_COLS];
    uint8_t attr[PCX_TERM_ROWS][PCX_TERM_COLS];

    uint8_t font[PCX_TERM_FONT_SIZE];

    int col;
    int row;
    int font_loaded;
    int dirty; /* a cell changed since the last render */
} pcx_term_t;

void pcx_term_init(pcx_term_t *term);
/* Loads the character generator; returns 1 on success, 0 if the file is
   missing or the wrong size (the terminal then draws blanks). */
int  pcx_term_load_font(pcx_term_t *term, const char *path);
/* One byte of the host -> board stream (the 0xf980 write). */
void pcx_term_write(pcx_term_t *term, uint8_t val);
/* Renders the page into a 32-bit ARGB buffer of width x height pixels. */
void pcx_term_render(pcx_term_t *term, uint32_t *out, int width, int height,
                     uint32_t fg, uint32_t bg);
/* Reads back a character of the page; used by the tests. */
uint8_t pcx_term_cell(pcx_term_t *term, int row, int col);

#endif /*EMU_PCX_TERM_H*/
