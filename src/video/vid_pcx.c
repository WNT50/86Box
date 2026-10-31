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
    int           mon;
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

            for (bit = 0; bit < 8; bit++)
                dev->fb[y * PCX_VIDEO_XSIZE + x + bit] =
                    (val & (0x80 >> bit)) ? 0x00e8e8e8 : 0x00000000;
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
        mem_mapping_add(&dev->vram_mapping, 0x000f0000, 0x00008000,
                        pcx_vram_read, NULL, NULL,
                        pcx_vram_write, NULL, NULL,
                        NULL, MEM_MAPPING_INTERNAL, dev);
        dev->vram_mapped = 1;
    }

    /* Graphics: the bitmap is shown while it is the last thing the host wrote.
       The BIOS turns the graphics mode on unconditionally -- and fills the
       window with 0xaa while it tests it -- so the command alone cannot
       decide; whichever channel was used last is what the host is talking to. */
    if (dev->gfx_on && dev->vram_active && (dev->src == 2)) {
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

        default:
            return dev->regs[reg];
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
        /* SCN267x command group 0x20: bit1 selects the graphics path, bit0 is
           on/off.  0x23 is what the BIOS sends (v3 0xfd6fc). */
        if (((val & 0xe0) == 0x20) && (val & 0x02)) {
            dev->gfx_on = (val & 0x01) ? 1 : 0;
        }

    } else if (reg == 0x00) {
        if (dev->parm_left > 0) {
            /* Setup block, the SCN2672 initialisation registers: IR0 69
               (14 scanlines, transparent buffer), IR4 18 (25 rows), IR5 27,
               IR9 f0, the rest zero.
               Nothing of it reaches the screen; the 8031 turns it into SCN2672
               timing.  Kept as a byte count for now, see the file header. */
            dev->parm_left--;
        } else {
            pcx_term_write(&dev->term, val);
            dev->src = 1;
            /* The 8031 sees the same stream and answers queries that need one. */
            kbc_siemens_rx(dev->kbc, val);
        }
    }
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

    memset(dev->regs, 0x00, sizeof(dev->regs));
    dev->parm_left = 0;
    dev->cmd_prev  = 0xff;

    overscan_x = 0;
    overscan_y = 0;
    video_inform(VIDEO_FLAG_TYPE_MDA, &timing_pcx);

    /* Nothing else decodes 0xf0000-0xf7fff, but the mapping still has to go
       in after the machine has built its own: a mapping that is already in the
       list when RAM is mapped gets overwritten by it and never sees an
       access.  So it is added from the first scan. */

    timer_add(&dev->timer, pcx_video_poll, dev, 1);
    io_sethandler(0xf980, 0x0010,
                  pcx_video_read, NULL, NULL, pcx_video_write, NULL, NULL, dev);

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
