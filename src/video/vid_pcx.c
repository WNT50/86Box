/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-X: the video board.
 *
 *          The board is an SCN2672 driven by an 8031: the host writes bytes to
 *          0xf980 and the 8031 turns them into characters in the board's
 *          character RAM, which the 2672 then scans out.  This model keeps the
 *          host side (0xf980-0xf98f) and the terminal page (pcx_term.c) and
 *          scans the page out at 640x350, which is what the Servicehandbuch
 *          gives for the Bildschirmeinheit: 80 characters of 8x14 dots per
 *          line, 25 lines, and a 640x350 graphics mode on the same tube.
 *
 *          The SCN2672's own register file is not modelled: the BIOS writes its
 *          eleven initialisation registers through 0xf980 on the way to the
 *          8031, and this file only counts them off so they never reach the
 *          screen.  Two of them agree with the geometry above -- IR0 0x69 gives
 *          14 scanlines per character row, IR4 0x18 gives 25 rows -- while IR5
 *          0x27 would say 40 characters per row, which is the one part of the
 *          block that does not fit 80 columns; the page keeps 80 for now.
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

extern void pcx_ring_dump(void);
extern void pcx_dump_code(uint32_t log, int len, const char *why);
#include <86box/pcx_term.h>
#include <86box/pcx_vid.h>
#include <86box/kbc_siemens.h>

#define PCX_VIDEO_XSIZE 640
#define PCX_VIDEO_YSIZE 350
#define PCX_VIDEO_FREQ  50 /* Hz, the board's vertical rate */

typedef struct pcx_video_t {
    pcx_term_t term;
    pc_timer_t timer;

    uint32_t fb[PCX_VIDEO_XSIZE * PCX_VIDEO_YSIZE];

    uint8_t       regs[16]; /* 0xf980-0xf98f as seen by the host */

    /* Host side video RAM: the board's 32 KB at 0xf0000.  It scans it as a
       640x350 bitmap when the host draws there instead of sending characters
       to the 8031 (SCN267x command 0x23).  System RAM stops at 0x80000, so the
       window is the board's alone -- see m_pcx.c. */
    uint8_t        vram[0x00008000];
    mem_mapping_t  vram_mapping;
    int            gfx_on;
    int            vram_dirty;
    int            vram_active; /* the host has drawn into the bitmap */
    int            vram_mapped;
    int            src;         /* where the last host write went: 0 none,
                                   1 the character channel, 2 the bitmap */
    int           parm_left; /* bytes left of the 15-byte setup block */
    int           cmd_prev;  /* previous 0xf982 value, for the double write */
    uint8_t       prev_char; /* last byte of the host -> board stream */
    int           mon;
    int           vram_sw;    /* 0xf9a0 bit 0: 1 = display RAM, 0 = chargen RAM */
    int           chargen_on; /* the host has uploaded a character generator */
    int           text_mode;  /* the display RAM holds character/attribute pairs */
    int           inverse;    /* whole-screen negative display (8741 P2 bit 3) */
    uint8_t       chargen[0x2000];
    int           cmd;        /* last command written to 0xf982 */
    int           cmd_left;   /* operand bytes still owed by that command */
    kbc_siemens_t *kbc;     /* protocol level keyboard/terminal channel */
} pcx_video_t;

static video_timings_t timing_pcx = { VIDEO_ISA, 8, 16, 32, 8, 16, 32 };

/* PCX_VID_LOG in the environment dumps the host -> 8031 stream; the POST's
   message table is ASCII, so the log reads as text with a byte index. */
static int pcx_vid_log;
static int pcx_vid_log_n;

static uint8_t pcx_vram_read(uint32_t addr, void *priv);
static void    pcx_vram_write(uint32_t addr, uint8_t val, void *priv);

/* 640 pixels of 8 bits per line, 350 lines, most significant bit leftmost. */
static void
pcx_video_scan_bitmap(pcx_video_t *dev)
{
    int           x, y, bit;
    uint8_t       val;
    const uint8_t *line;

    for (y = 0; y < PCX_VIDEO_YSIZE; y++) {
        line = &dev->vram[y * (PCX_VIDEO_XSIZE / 8)];

        for (x = 0; x < PCX_VIDEO_XSIZE; x += 8) {
            val = line[x >> 3];

            for (bit = 0; bit < 8; bit++) {
                int on = (val & (0x80 >> bit)) ? 1 : 0;

                /* The 8741 inverts the bitmap as well when P2 bit 3 is set
                   (pcd_v.cpp display_pixels, the "lg" branch). */
                if (dev->inverse)
                    on = !on;
                dev->fb[y * PCX_VIDEO_XSIZE + x + bit] =
                    on ? 0x00e8e8e8 : 0x00000000;
            }
        }
    }

    dev->vram_dirty = 0;
}


/* Character mode: the display RAM holds one character and one attribute byte
   per cell (MAME pcd_v.cpp display_pixels reads m_vram[address] as the code and
   m_vram[address+1] as the attribute).  The glyph comes from the character
   generator the host downloaded with 0xf9a0 = 0 -- 32 bytes per character, the
   loader writes 14 of them and then skips 4 (0x5494-0x5498). */
static void
pcx_video_scan_text(pcx_video_t *dev)
{
    int           row, col, line, bit, x, y;
    uint8_t       ch, glyph, attr;
    const uint8_t *font;
    uint32_t      fg, bg, pix;
    const uint32_t white = 0x00e8e8e8, black = 0x00000000, grey = 0x00808080;

    for (row = 0; row < PCX_TERM_ROWS; row++) {
        for (col = 0; col < PCX_TERM_COLS; col++) {
            ch   = dev->vram[(row * PCX_TERM_COLS + col) * 2];
            attr = dev->vram[(row * PCX_TERM_COLS + col) * 2 + 1];

            if (dev->chargen_on) {

                /* The SCN2672 character generator holds 16 bytes per glyph and
                   the attribute's bit 5 picks the second 4 KB bank; indexing
                   with a 32 byte stride fetched the neighbouring glyph's
                   second half and every character came out as noise. */
                font = &dev->chargen[((ch * 16) + ((attr & 0x20) ? 4096 : 0)) & 0x1fff];
            }
            else
                font = &dev->term.font[(ch * 16) % PCX_TERM_FONT_SIZE];

            /* Colour semantics are the ones MAME's pcd_v.cpp display_pixels
               implements, since the 8741 firmware that drives the real board
               is not dumped and cannot be emulated here: P2 bit 3 makes the
               whole screen negative (foreground black, background white),
               attribute bit 4 inverts a single cell, bit 3 gives that cell a
               grey background. */
            fg = dev->inverse ? black : white;
            bg = dev->inverse ? white : black;
            if (attr & 0x10) {
                uint32_t t = fg;

                fg = bg;
                bg = t;
            }
            if (attr & 0x08)
                bg = grey;

            for (line = 0; line < PCX_TERM_CELL_H; line++) {
                glyph = font[line];
                y     = row * PCX_TERM_CELL_H + line;

                /* One glyph dot per pixel: the PC-D prints 80 characters of 8
                   dots per line (Servicehandbuch: "80 Zeichen/Zeile,
                   Zeichenfeld 14 x 8 Punkte"), so 80 columns fill the 640 dot
                   line exactly.  MAME's pcd_v.cpp draws each dot twice for a
                   40 column layout, which does not fit the manual. */
                for (bit = 0; bit < PCX_TERM_CELL_W; bit++) {
                    x   = col * PCX_TERM_CELL_W + bit;
                    pix = (glyph & (0x80 >> bit)) ? fg : bg;
                    dev->fb[y * PCX_VIDEO_XSIZE + x] = pix;
                }
            }
        }
    }

    dev->vram_dirty = 0;
}

static void
pcx_video_poll(void *priv)
{
    pcx_video_t *dev = (pcx_video_t *) priv;
    int          y;

    if (!dev->vram_mapped) {
        /* The 32 KB bitmap window at 0xf0000 belongs to the PC-D's SCN2674
           board; the PC-X's SCN2672 is character-only and has no such window
           (MAME pcd_map leaves 0x80000-0xfbfff unmapped, and only
           pcd_video_device::device_start installs the 0xf0000 handler).
           PCX_PCDVRAM=1 brings it back for experiments. */
        /* The window stays: the BIOS downloads the character generator into
           it (0xf9a0 bit 0 = 0), even though the board itself is character
           only.  What the PC-X does not have is the PC-D's bitmap page, which
           is why the display below never switches to it. */
        if (getenv("PCX_NOVRAM") == NULL)
        mem_mapping_add(&dev->vram_mapping, 0x000f0000, 0x00008000,
                        pcx_vram_read, NULL, NULL,
                        pcx_vram_write, NULL, NULL,
                        NULL, MEM_MAPPING_INTERNAL, dev);
        dev->vram_mapped = 1;
    }

    /* The 2672 scans the character page out.  The "gfx" bit of command group
       0x20 exists on the 2674 in the PC-D and does not turn this board into a
       bitmap, so the page stays the display; PCX_PCDVRAM=1 restores the old
       PC-D style switch for comparison. */
    {
        static int pcdv2 = -1;
        if (pcdv2 < 0)
            pcdv2 = getenv("PCX_PCDVRAM") ? 1 : 0;
        dev->text_mode = pcdv2 ? (dev->vram_active ? !dev->gfx_on : 0) : 1;
    }

    {
        static int last = -1;
        static int logon = -1;

        if (logon < 0)
            logon = (getenv("PCX_VIDLOG") != NULL);
        if (logon && last != dev->text_mode) {
            last = dev->text_mode;
            pclog("[VID] mode=%s gfx_on=%d vram_active=%d\n",
                  dev->text_mode ? "text" : "bitmap", dev->gfx_on, dev->vram_active);
        }
    }

    if (dev->text_mode) {
        /* The characters the host sends end up in the terminal page, which is
           what the 2672 scans out; dev->vram only holds the character
           generator the BIOS downloads. */
        if (dev->term.dirty) {
            uint32_t fg = dev->inverse ? 0x00000000 : 0x00e8e8e8;
            uint32_t bg = dev->inverse ? 0x00e8e8e8 : 0x00000000;

            pcx_term_render(&dev->term, dev->fb, PCX_VIDEO_XSIZE,
                            PCX_VIDEO_YSIZE, fg, bg);
            dev->term.dirty = 0;
        }

        for (y = 0; y < PCX_VIDEO_YSIZE; y++)
            memcpy(buffer32->line[y], &dev->fb[y * PCX_VIDEO_XSIZE],
                   PCX_VIDEO_XSIZE * 4);
        video_blit_memtoscreen(0, 0, PCX_VIDEO_XSIZE, PCX_VIDEO_YSIZE);

        timer_advance_u64(&dev->timer, TIMER_USEC * (1000000ULL / PCX_VIDEO_FREQ));
        return;
    }

    if (dev->vram_active) {
        if (dev->vram_dirty)
            pcx_video_scan_bitmap(dev);

        for (y = 0; y < PCX_VIDEO_YSIZE; y++)
            memcpy(buffer32->line[y], &dev->fb[y * PCX_VIDEO_XSIZE],
                   PCX_VIDEO_XSIZE * 4);
        video_blit_memtoscreen(0, 0, PCX_VIDEO_XSIZE, PCX_VIDEO_YSIZE);

        timer_advance_u64(&dev->timer, TIMER_USEC * (1000000ULL / PCX_VIDEO_FREQ));
        return;
    }

    if (!dev->term.dirty) {
        timer_advance_u64(&dev->timer, TIMER_USEC * (1000000ULL / PCX_VIDEO_FREQ));
        return;
    }

    pcx_term_render(&dev->term, dev->fb, PCX_VIDEO_XSIZE, PCX_VIDEO_YSIZE,
                    0x00e8e8e8, 0x00000000);

    for (y = 0; y < PCX_VIDEO_YSIZE; y++)
        memcpy(buffer32->line[y], &dev->fb[y * PCX_VIDEO_XSIZE],
               PCX_VIDEO_XSIZE * 4);
    video_blit_memtoscreen(0, 0, PCX_VIDEO_XSIZE, PCX_VIDEO_YSIZE);

    timer_advance_u64(&dev->timer, TIMER_USEC * (1000000ULL / PCX_VIDEO_FREQ));
}

static uint8_t
pcx_video_read(uint16_t addr, void *priv)
{
    pcx_video_t *dev = (pcx_video_t *) priv;
    int          reg = addr & 0x0f;

    switch (reg) {
        case 0x00:
            /* 0xf980: what the keyboard/terminal side has to say.  The POST's
               identity check (v3 0xfd8d3) reads its answer here. */
            return kbc_siemens_read_data(dev->kbc);

        case 0x01:
            /* 0xf981: bit 0 "a byte waits", bit 1 "not taken yet". */
            return kbc_siemens_read_status(dev->kbc);

        case 0x02:
            /* 0xf982: command register; bit 4 reads back as "ready" -- the
               loader's handshake (0x547d) polls it after every command. */
            return 0x10 | (dev->regs[reg] & 0x0f);

        default:
            return dev->regs[reg];
    }
}


/* PCX_TERMDUMP=1: print the terminal page as plain text.  The console text
   only exists here -- the board draws it itself and keeps no copy in the host's
   memory space. */
static void
pcx_term_dump(pcx_video_t *dev)
{
    static int      n;
    static uint32_t last;
    int             row, col;
    char            line[PCX_TERM_COLS + 1];
    uint32_t        now = plat_get_ticks();

    if ((n >= 6) || (getenv("PCX_TERMDUMP") == NULL) || (now < 4600u) ||
        ((now - last) < 250u))
        return;
    last = now;

    n++;
    pclog("[TERM] --- page %d, t=%u ---\n", n, now);
    for (row = 0; row < PCX_TERM_ROWS; row++) {
        for (col = 0; col < PCX_TERM_COLS; col++) {
            char c = (char) (dev->term.cells[row][col] & 0x7f);
            line[col] = ((c < 0x20) || (c == 0x7f)) ? ' ' : c;
        }
        line[PCX_TERM_COLS] = 0;
        pclog("[TERM] %s\n", line);
    }
}

static void
pcx_video_write(uint16_t addr, uint8_t val, void *priv)
{
    pcx_video_t *dev = (pcx_video_t *) priv;
    int          reg = addr & 0x0f;

    dev->regs[reg] = val;


    if ((reg == 0x00) && (pcx_vid_log > 0) && (pcx_vid_log_n < 8192))
        pclog("[PCXV] %05i %02X %c\n", pcx_vid_log_n++, val,
              ((val >= 0x20) && (val < 0x7f)) ? val : '.');

    if (reg == 0x02) {
        /* 0xf982 is the board's command register.  The BIOS writes 0 here
           (twice) before the 15-byte setup block, and 0x28/0x23 once the block
           is in.  Command 0 puts the 8031 back into setup mode: the next 15
           bytes on 0xf980 are the display parameters, not characters, and the
           page starts blank (v3 0xfc99b). */
        if (val != 0x00)
            dev->parm_left = 0; /* 0x28/0x23 end the setup phase */
        if ((val == 0x00) && (dev->cmd_prev == 0x00)) {
            /* Two zeros in a row open the setup block; the register clear loop
               also ends on 0xf982, but that write follows 0xf983, so it does not
               arm the counter (v3 0xfc99b vs 0xfc9c7). */
            dev->parm_left = 15;
            pcx_term_init(&dev->term);
        }
        dev->cmd_prev = val;
        dev->cmd      = val;
        /* 0x16 takes a cursor address, 0x17 an attribute byte.  Their
           operands arrive on 0xf980 and are not characters. */
        if (val == 0x16)
            dev->cmd_left = 2;
        else if (val == 0x17)
            dev->cmd_left = 1;
        else
            dev->cmd_left = 0;
        /* SCN267x command group 0x20: bit1 selects the graphics path, bit0 is
           on/off.  0x23 is what the BIOS sends (v3 0xfd6fc). */
        if (((val & 0xe0) == 0x20) && (val & 0x02)) {
            int on = (val & 0x01) ? 1 : 0;

            if ((getenv("PCX_VIDLOG") != NULL) && (on != dev->gfx_on))
                pclog("[LG] cmd=%02X gfx_on %d -> %d\n", val, dev->gfx_on, on);
            dev->gfx_on = on;
        }

    } else if (reg == 0x00) {
        if (dev->parm_left > 0) {
            /* Setup block, the SCN2672 initialisation registers: IR0 69
               (14 scanlines, transparent buffer), IR4 18 (25 rows), IR5 27,
               IR9 f0, the rest zero.
               Nothing of it reaches the screen; the 8031 turns it into SCN2672
               timing.  Kept as a byte count for now, see the file header. */
            dev->parm_left--;
        } else if (dev->cmd_left > 0) {
            dev->cmd_left--;
        } else {
            if ((val == 'G') && (dev->prev_char == '\n') && (getenv("PCX_RING") != NULL)) {
                static int gn;
                if (gn < 3) {
                    int k;

                    gn++;
                    pclog("[RING] request %d at t=%u ss=%04X sp=%04X\n", gn,
                          plat_get_ticks(), cpu_state.seg_ss.seg,
                          cpu_state.regs[4].w);
                    /* Walk the stack and report any word that points at the
                       text being printed; that is the format string argument. */
                    for (k = 0; k < 64; k++) {
                        uint32_t slot = ((uint32_t) cpu_state.seg_ss.seg << 4) +
                                        cpu_state.regs[4].w + 2 * k;
                        uint16_t w;
                        uint32_t a;

                        if (mem_machine_translate != NULL) {
                            w = (uint16_t) (readmembl(mem_machine_translate(slot, 0)) |
                                            (readmembl(mem_machine_translate(slot + 1, 0)) << 8));
                        } else {
                            w = (uint16_t) (readmembl(slot) | (readmembl(slot + 1) << 8));
                        }
                        a = w;
                        if (mem_machine_translate != NULL)
                            a = mem_machine_translate(w, 0);
                        if (readmembl(a) == '\n') {
                            char t[9];
                            int  j;

                            for (j = 0; j < 8; j++) {
                                uint32_t b = w + 1 + j;
                                if (mem_machine_translate != NULL)
                                    b = mem_machine_translate(b, 0);
                                t[j] = (char) readmembl(b);
                            }
                            t[8] = 0;
                            pclog("[RING] sp+%02X = %04X -> \"%s\"\n", 2 * k, w, t);
                        }
                    }
                    for (k = 0x2c; k < 0x60; k += 2) {
                        uint32_t slot = ((uint32_t) cpu_state.seg_ss.seg << 4) +
                                        cpu_state.regs[4].w + k;
                        uint16_t w;

                        if (mem_machine_translate != NULL)
                            w = (uint16_t) (readmembl(mem_machine_translate(slot, 0)) |
                                            (readmembl(mem_machine_translate(slot + 1, 0)) << 8));
                        else
                            w = (uint16_t) (readmembl(slot) | (readmembl(slot + 1) << 8));
                        pclog("[RING] sp+%02X = %04X\n", k, w);
                    }
                    /* The frame right above the format pointer is the call
                       that produced this message: dump the code around it. */
                    {
                        uint32_t slot = ((uint32_t) cpu_state.seg_ss.seg << 4) +
                                        cpu_state.regs[4].w + 0x30;
                        uint16_t ipw, csw;

                        if (mem_machine_translate != NULL) {
                            ipw = (uint16_t) (readmembl(mem_machine_translate(slot, 0)) |
                                              (readmembl(mem_machine_translate(slot + 1, 0)) << 8));
                            csw = (uint16_t) (readmembl(mem_machine_translate(slot + 2, 0)) |
                                              (readmembl(mem_machine_translate(slot + 3, 0)) << 8));
                        } else {
                            ipw = (uint16_t) (readmembl(slot) | (readmembl(slot + 1) << 8));
                            csw = (uint16_t) (readmembl(slot + 2) | (readmembl(slot + 3) << 8));
                        }
                        pclog("[RING] caller %04X:%04X\n", csw, ipw);
                        {
                            uint32_t v = 0x6424;
                            uint16_t pg;
                            if (mem_machine_translate != NULL)
                                v = mem_machine_translate(v, 0);
                            pg = (uint16_t) (readmembl(v) | (readmembl(v + 1) << 8));
                            pclog("[RING] free pages [0x6424] = %u -> %u KB\n",
                                  pg, (unsigned) pg * 2u);
                        }
                        pcx_dump_code((((uint32_t) csw) << 4) + ipw - 64, 160, "fmt-caller");
                    }
                    pcx_ring_dump();
                }
            }
            dev->prev_char = val;
            pcx_term_write(&dev->term, val);
            pcx_term_dump(dev);
            dev->vram_dirty = 1;   /* the page changed, rescan it */
            dev->src = 1;
            /* The 8031 sees the same stream and answers queries that need one. */
            kbc_siemens_rx(dev->kbc, val);
        }
    }
}

static uint8_t pcx_video_read(uint16_t addr, void *priv);
static void    pcx_video_write(uint16_t addr, uint8_t val, void *priv);
static void    pcx_vram_sw_write(uint16_t addr, uint8_t val, void *priv);

/* Whole 64-byte window of the video board: only 0x00-0x0f and 0x20 decode,
   everything else answers an undriven bus, as the device's own map does in
   MAME (offsets outside its map fall through to unmap_value_high). */
static uint8_t
pcx_port_read(uint16_t addr, void *priv)
{
    if ((uint16_t) (addr - 0xf980) < 0x10)
        return pcx_video_read(addr, priv);

    return 0xff;
}

static void
pcx_port_write(uint16_t addr, uint8_t val, void *priv)
{
    uint16_t off = (uint16_t) (addr - 0xf980);

    if (off < 0x10)
        pcx_video_write(addr, val, priv);
    else if (off == 0x20)
        pcx_vram_sw_write(addr, val, priv);
}

static void
pcx_vram_sw_write(UNUSED(uint16_t addr), uint8_t val, void *priv)
{
    pcx_video_t *dev = (pcx_video_t *) priv;

    /* 0xf9a0 bit 0 picks what the host window at 0xf0000 is: 0 the character
       generator RAM (one byte per word, the loader's font download at 0x5485),
       1 the display RAM the CRTC scans out. */
    dev->vram_sw = val & 1;
}

static uint8_t
pcx_vram_read(uint32_t addr, UNUSED(void *priv))
{
    pcx_video_t *dev = (pcx_video_t *) priv;

    return dev->vram[addr & 0x7fff];
}

static void
pcx_vram_write(uint32_t addr, uint8_t val, void *priv)
{
    pcx_video_t *dev = (pcx_video_t *) priv;

    if (!dev->vram_sw) {
        if (!(addr & 1)) {
            dev->chargen[(addr >> 1) & 0x1fff] = val;
            dev->chargen_on                    = 1;
            dev->vram_dirty                    = 1;
        }
        return;
    }

    dev->vram[addr & 0x7fff] = val;
    dev->vram_dirty          = 1;
    dev->vram_active         = 1;
    dev->src                 = 2;
}

/* The machine creates the controller and hands it to the board, which owns the
   two ports it appears on. */
void
pcx_video_set_kbc(void *priv, kbc_siemens_t *kbc)
{
    pcx_video_t *dev = (pcx_video_t *) priv;

    if (dev != NULL)
        dev->kbc = kbc;
}

/* Called by the machine once the keyboard is modelled; a byte from the 8031 to
   the host, read at 0xf980. */
void
pcx_video_set_key(void *priv, uint8_t val)
{
    pcx_video_t *dev = (pcx_video_t *) priv;

    if (dev != NULL)
        dev->regs[0x00] = val;
}

static void *
pcx_video_init(const device_t *info)
{
    pcx_video_t *dev;
    char         path[512];

    dev = (pcx_video_t *) calloc(1, sizeof(pcx_video_t));
    if (dev == NULL)
        return NULL;

    pcx_vid_log = (getenv("PCX_VID_LOG") != NULL);

    pcx_term_init(&dev->term);

    /* The board's own character generator: 512 glyphs of 8x14, 16 bytes each. */
    snprintf(path, sizeof(path), "%s", "roms/machines/pcx/d12-graka.bin");
    if (!pcx_term_load_font(&dev->term, path))
        pclog("PC-X video: character generator not found, text will be blank\n");

    /* The 8741 on the video board holds P2 bit 3 high out of reset, which is
       the "negative" picture the real machine shows: black text on a white
       background.  MAME's pcd_v.cpp does the same through m_p2 & 8. */
    dev->inverse   = 0;   /* PC-X inverts through P1 bit 5, and only if set */

    memset(dev->regs, 0x00, sizeof(dev->regs));
    dev->parm_left = 0;
    dev->cmd_prev  = 0xff;
    dev->vram_sw   = 1;
    dev->cmd       = 0xff;

    overscan_x = 0;
    overscan_y = 0;
    video_inform(VIDEO_FLAG_TYPE_MDA, &timing_pcx);

    /* Nothing else decodes 0xf0000-0xf7fff, but the mapping still has to go
       in after the machine has built its own: a mapping that is already in the
       list when RAM is mapped gets overwritten by it and never sees an
       access.  So it is added from the first scan. */

    timer_add(&dev->timer, pcx_video_poll, dev, 1);
    /* MAME's pcx_io hands the whole 0xf980-0xf9bf span to the video board
       (pcd.cpp:511) and lets the device's own map decide what the gaps answer;
       the POST walks that window when it builds its device table.  Claim the
       same 64 bytes here, with the undecoded ones reading as open bus.
       PCX_PORTS16=1 restores the old 16-byte claim for comparison. */
    if (getenv("PCX_PORTS16") != NULL) {
        io_sethandler(0xf980, 0x0010,
                      pcx_video_read, NULL, NULL, pcx_video_write, NULL, NULL, dev);
        io_sethandler(0xf9a0, 0x0001,
                      NULL, NULL, NULL, pcx_vram_sw_write, NULL, NULL, dev);
    } else {
        io_sethandler(0xf980, 0x0040,
                      pcx_port_read, NULL, NULL, pcx_port_write, NULL, NULL, dev);
    }

    return dev;
}

static void
pcx_video_close(void *priv)
{
    pcx_video_t *dev = (pcx_video_t *) priv;

    if (dev != NULL)
        free(dev);
}

static void
pcx_video_speed_changed(void *priv)
{
    /* The scan-out is on its own timer, nothing to re-derive. */
    (void) priv;
}

const device_t pcx_video_device = {
    .name          = "Siemens PC-X video board",
    .internal_name = "pcx_video",
    .flags         = DEVICE_ISA16,
    .local         = 0,
    .init          = pcx_video_init,
    .close         = pcx_video_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = pcx_video_speed_changed,
    .force_redraw  = NULL,
    .config        = NULL
};
