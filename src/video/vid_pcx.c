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
 *          scans the page out at 960x350, which is what the board's timing
 *          gives: 24 MHz / 8 pixels per character = 120 columns of 8 pixels and
 *          25 rows of 14 lines.
 *
 *          The SCN2672's own register file is not modelled yet: the BIOS writes
 *          a parameter block through 0xf980 on the way to the 8031 (v3 0xfc9a3)
 *          and those bytes are swallowed by the terminal, which is harmless as
 *          long as the geometry above is the one the BIOS programs.
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

#define PCX_VIDEO_XSIZE 960
#define PCX_VIDEO_YSIZE 350
#define PCX_VIDEO_FREQ  50 /* Hz, the board's vertical rate */

typedef struct pcx_video_t {
    pcx_term_t term;
    pc_timer_t timer;

    uint32_t fb[PCX_VIDEO_XSIZE * PCX_VIDEO_YSIZE];

    uint8_t regs[16]; /* 0xf980-0xf98f as seen by the host */
    int     mon;
} pcx_video_t;

static video_timings_t timing_pcx = { VIDEO_ISA, 8, 16, 32, 8, 16, 32 };

static void
pcx_video_poll(void *priv)
{
    pcx_video_t *dev = (pcx_video_t *) priv;
    int          y;

    if (!dev->term.dirty) {
        timer_advance_u64(&dev->timer, 1000000ULL / PCX_VIDEO_FREQ);
        return;
    }

    pcx_term_render(&dev->term, dev->fb, PCX_VIDEO_XSIZE, PCX_VIDEO_YSIZE,
                    0x00e8e8e8, 0x00000000);

    for (y = 0; y < PCX_VIDEO_YSIZE; y++)
        memcpy(buffer32->line[y], &dev->fb[y * PCX_VIDEO_XSIZE],
               PCX_VIDEO_XSIZE * 4);
    video_blit_memtoscreen(0, 0, PCX_VIDEO_XSIZE, PCX_VIDEO_YSIZE);

    timer_advance_u64(&dev->timer, 1000000ULL / PCX_VIDEO_FREQ);
}

static uint8_t
pcx_video_read(uint16_t addr, void *priv)
{
    pcx_video_t *dev = (pcx_video_t *) priv;
    int          reg = addr & 0x0f;

    switch (reg) {
        case 0x00:
            /* 0xf980: keyboard byte from the 8031; the POST reads it back while
               probing the board, so return something harmless. */
            return 0x00;

        case 0x01:
            /* 0xf981: status.  Bit 1 is "the 8031 has not taken the character
               yet"; the model consumes the stream immediately. */
            return 0x00;

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

    if (reg == 0x00)
        pcx_term_write(&dev->term, val);
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

    pcx_term_init(&dev->term);

    /* The board's own character generator: 512 glyphs of 8x14, 16 bytes each. */
    snprintf(path, sizeof(path), "%s", "roms/machines/pcx/d12-graka.bin");
    if (!pcx_term_load_font(&dev->term, path))
        pclog("PC-X video: character generator not found, text will be blank\n");

    memset(dev->regs, 0x00, sizeof(dev->regs));

    overscan_x = 0;
    overscan_y = 0;
    video_inform(VIDEO_FLAG_TYPE_MDA, &timing_pcx);

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
