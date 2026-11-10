/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Mindset Personal Computer display: 32 KB of display RAM, a
 *          16-entry palette written through an index port, and the three
 *          pixel formats the GCP can put on screen.  There is no CRTC: the
 *          display control word picks the format and the bank, the display
 *          registers set the border colour and the screen position, and a
 *          separate "interrupt position" register names the display address
 *          the frame interrupt fires at.
 *
 *          Only the IBM-compatible formats are drawn for now; the native
 *          packed formats still log and show the border.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define HAVE_STDARG_H
#include <stdarg.h>

#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/plat.h>
#include <86box/timer.h>
#include <86box/video.h>

#define MIND_VRAM_SIZE   0x00008000
#define MIND_VRAM_BASE   0x000b8000
#define MIND_XSIZE       751      /* the raster the board actually produces */
#define MIND_YSIZE       480
#define MIND_FIELD_X     640      /* the picture inside it */
#define MIND_FIELD_Y     400

typedef struct {
    uint8_t     vram[MIND_VRAM_SIZE];

    uint16_t    dispctrl;
    uint16_t    screenpos;
    uint16_t    intpos;
    uint32_t    intaddr;
    uint8_t     borderidx;
    uint32_t    palette[16];
    uint8_t     genlock[16];

    uint64_t      frames;
    mem_mapping_t vram_map;
    pc_timer_t    timer;
    pc_timer_t    irq_timer;
    void        (*irq)(void *priv, int level);
    void         *irq_priv;
} mindset_vid_t;

static mindset_vid_t *mindset_vid;
static int            mindset_log;

static video_timings_t timing_mindset = { .type = VIDEO_BUS };

static uint8_t
mind_vram_read(uint32_t addr, void *priv)
{
    static int shown = 0;

    if (mindset_log && (shown < 4)) {
        pclog("[MIND] vram r %05X (priv=%p)\n", addr, priv);
        shown++;
    }

    return ((uint8_t *) priv)[addr & (MIND_VRAM_SIZE - 1)];
}

static uint16_t
mind_vram_readw(uint32_t addr, void *priv)
{
    uint8_t *vram = (uint8_t *) priv;
    uint32_t off  = addr & (MIND_VRAM_SIZE - 2);

    return (uint16_t) (vram[off] | (vram[off + 1] << 8));
}

static uint32_t
mind_vram_readl(uint32_t addr, void *priv)
{
    uint8_t *vram = (uint8_t *) priv;
    uint32_t off  = addr & (MIND_VRAM_SIZE - 4);

    return (uint32_t) (vram[off] | (vram[off + 1] << 8) |
                       (vram[off + 2] << 16) | ((uint32_t) vram[off + 3] << 24));
}

static void
mind_vram_write(uint32_t addr, uint8_t val, void *priv)
{
    ((uint8_t *) priv)[addr & (MIND_VRAM_SIZE - 1)] = val;
}

static void
mind_vram_writew(uint32_t addr, uint16_t val, void *priv)
{
    uint8_t *vram = (uint8_t *) priv;
    uint32_t off  = addr & (MIND_VRAM_SIZE - 2);

    vram[off]     = (uint8_t) (val & 0xff);
    vram[off + 1] = (uint8_t) (val >> 8);
}

static void
mind_vram_writel(uint32_t addr, uint32_t val, void *priv)
{
    uint8_t *vram = (uint8_t *) priv;
    uint32_t off  = addr & (MIND_VRAM_SIZE - 4);

    vram[off]     = (uint8_t) (val & 0xff);
    vram[off + 1] = (uint8_t) ((val >> 8) & 0xff);
    vram[off + 2] = (uint8_t) ((val >> 16) & 0xff);
    vram[off + 3] = (uint8_t) (val >> 24);
}

static uint16_t
sw16(uint16_t v)
{
    return (uint16_t) ((v << 8) | (v >> 8));
}

uint8_t *
mindset_video_vram(void)
{
    return (mindset_vid != NULL) ? mindset_vid->vram : NULL;
}

/* Reading a display register picks one of eight views, selected by the low
   three bits of the control word for writes and by bits 4-6 for reads. */
static uint16_t
mind_dispreg_read(mindset_vid_t *vid)
{
    switch ((vid->dispctrl >> 4) & 7) {
        case 1: /* display word at the interrupt position */
            {
                uint16_t v = (uint16_t) (vid->vram[(vid->intaddr >> 1) & (MIND_VRAM_SIZE - 1)] |
                                         (vid->vram[((vid->intaddr >> 1) + 1) & (MIND_VRAM_SIZE - 1)] << 8));

                return (uint16_t) (sw16(v) << 4);
            }

        case 5: /* the palette upload path wants this set */
            return 0x0080;

        default:
            break;
    }

    return 0;
}

static void
mind_dispreg_write(mindset_vid_t *vid, uint16_t val)
{
    if (mindset_log)
        pclog("[MIND] dispreg[%d] = %04X\n", vid->dispctrl & 7, val);

    switch (vid->dispctrl & 7) {
        case 0:
            vid->screenpos = val;
            break;

        case 1:
            vid->borderidx = (val >> 8) & 0xf;
            break;

        case 2: /* interrupt position: remember the display address it maps to */
            {
                vid->intpos = val;
                int      intx = (159 - ((val >> 8) & 255)) * ((vid->dispctrl & 0x100) ? 2 : 4);
                int      inty = 199 - (val & 255);
                bool     bank = (vid->dispctrl & 0x4000) != 0;
                bool     ibm  = (vid->dispctrl & 0x2000) != 0;
                int      ppb  = (vid->dispctrl & 0x0600) >> 9;
                bool     big  = (vid->dispctrl & 0x100) != 0;

                if (ibm)
                    vid->intaddr = (uint32_t) ((inty & 1) * 0x2000 + (inty >> 1) * 80 +
                                               (intx >> (3 - big)));
                else {
                    int stepy = (ppb == 2) ? 40 : ((ppb == 1) ? 80 : 160);

                    vid->intaddr = (uint32_t) (inty * stepy + (intx >> (ppb - big + 2)));
                }

                if (bank)
                    vid->intaddr += 16000;
            }
            break;

        case 4: /* palette entry: 3 bits per gun, written to the border index */
            {
                uint16_t d = sw16(val);
                uint8_t  r = (uint8_t) ((0x49 * (d & 7)) >> 1);
                uint8_t  g = (uint8_t) ((0x49 * ((d & 0x38) >> 3)) >> 1);
                uint8_t  b = (uint8_t) ((0x49 * ((d & 0x1c0) >> 6)) >> 1);

                vid->palette[vid->borderidx]   = ((uint32_t) r << 16) | ((uint32_t) g << 8) | b;
                vid->genlock[vid->borderidx]   = (val & 0x0200) ? 1 : 0;
                vid->borderidx                 = (vid->borderidx + 1) & 0xf;
            }
            break;

        case 5:
            break;

        default:
            break;
    }
}

static uint8_t
mind_dispreg_read8(uint16_t port, void *priv)
{
    uint16_t v = mind_dispreg_read((mindset_vid_t *) priv);

    return (port & 1) ? (uint8_t) (v >> 8) : (uint8_t) (v & 0xff);
}

static uint16_t
mind_dispreg_read16(uint16_t port, void *priv)
{
    return mind_dispreg_read((mindset_vid_t *) priv);
}

static void
mind_dispreg_write8(uint16_t port, uint8_t val, void *priv)
{
    mindset_vid_t *vid = (mindset_vid_t *) priv;
    uint16_t       cur = mind_dispreg_read(vid);

    if (port & 1)
        mind_dispreg_write(vid, (uint16_t) ((cur & 0x00ff) | (val << 8)));
    else
        mind_dispreg_write(vid, (uint16_t) ((cur & 0xff00) | val));
}

static void
mind_dispreg_write16(uint16_t port, uint16_t val, void *priv)
{
    mind_dispreg_write((mindset_vid_t *) priv, val);
}

static uint16_t
mind_dispctrl_read16(uint16_t port, void *priv)
{
    return ((mindset_vid_t *) priv)->dispctrl;
}

static uint8_t
mind_dispctrl_read8(uint16_t port, void *priv)
{
    uint16_t v = ((mindset_vid_t *) priv)->dispctrl;

    return (port & 1) ? (uint8_t) (v >> 8) : (uint8_t) (v & 0xff);
}

static void
mind_dispctrl_write16(uint16_t port, uint16_t val, void *priv)
{
    mindset_vid_t *vid = (mindset_vid_t *) priv;

    if (mindset_log)
        pclog("[MIND] dispctrl = %04X (ibm=%d bank=%d big=%d ppb=%d 400=%d)\n",
              val, (val & 0x2000) ? 1 : 0, (val & 0x4000) ? 1 : 0,
              (val & 0x0100) ? 1 : 0, (val >> 9) & 3, (val & 0x0800) ? 1 : 0);

    vid->dispctrl = val;
}

static void
mind_dispctrl_write8(uint16_t port, uint8_t val, void *priv)
{
    mindset_vid_t *vid = (mindset_vid_t *) priv;

    if (port & 1)
        vid->dispctrl = (uint16_t) ((vid->dispctrl & 0x00ff) | (val << 8));
    else
        vid->dispctrl = (uint16_t) ((vid->dispctrl & 0xff00) | val);
}

/* The board pulses the frame interrupt when the display reaches the address
   in the interrupt-position register; it is a short pulse, not a level. */
static void
mind_irq_lower(void *priv)
{
    mindset_vid_t *vid = (mindset_vid_t *) priv;

    if (vid->irq != NULL)
        vid->irq(vid->irq_priv, 0);
}

void
mindset_video_set_irq(void (*cb)(void *priv, int level), void *priv)
{
    if (mindset_vid == NULL)
        return;

    mindset_vid->irq      = cb;
    mindset_vid->irq_priv = priv;
}

static void
mind_render(mindset_vid_t *vid)
{
    uint32_t border = vid->palette[vid->borderidx];

    if (mindset_log && ((vid->frames % 60) == 0))
        pclog("[MIND] render: dispctrl=%04X border=%d pal=%08X,%08X,%08X,%08X pos=%04X\n",
              vid->dispctrl, vid->borderidx, vid->palette[0], vid->palette[1],
              vid->palette[2], vid->palette[3], vid->screenpos);
    int      dx     = ((vid->screenpos >> 8) & 15) * (MIND_XSIZE - MIND_FIELD_X) / 15;
    int      dy     = ((vid->screenpos >> 12) & 15) * (MIND_YSIZE - MIND_FIELD_Y) / 15;

    for (int y = 0; y < MIND_YSIZE; y++) {
        uint32_t *p = buffer32->line[y];

        for (int x = 0; x < MIND_XSIZE; x++)
            p[x] = border;
    }

    if (!(vid->dispctrl & 0x2000)) {
        /* Native formats: the pixels are packed most-significant-first, one
           to four bits per pixel, straight out of display RAM. */
        const uint8_t *base   = vid->vram + ((vid->dispctrl & 0x4000) ? 16000 : 0);
        int            order  = (vid->dispctrl & 0x0600) >> 9;
        int            bpp    = (order == 2) ? 1 : ((order == 1) ? 2 : 4);
        int            width  = (vid->dispctrl & 0x0100) ? 320 : 640;
        int            lines  = (vid->dispctrl & 0x0800) ? 400 : 200;
        uint32_t       stride = (uint32_t) (width * bpp) / 8;
        int            mask   = (1 << bpp) - 1;
        int            repeat = (lines == 200) ? 2 : 1;

        for (int row = 0; row < lines; row++) {
            const uint8_t *src = base + ((uint32_t) row * stride);

            for (int line = 0; line < repeat; line++) {
                int       y = (row * repeat) + line + dy;
                uint32_t *p = buffer32->line[y];

                for (int x = 0; x < width; x++) {
                    int bit  = x * bpp;
                    int byte = src[bit >> 3];
                    int val  = (byte >> (8 - bpp - (bit & 7))) & mask;

                    p[x + dx] = vid->palette[val];
                }
            }
        }

        return;
    }

        /* IBM-compatible formats: the display word is fetched as a 16-bit
           value whose halves are swapped, and each source row is drawn twice
           into the two fields so that 200 source lines fill 400. */
    const uint8_t *base = vid->vram + ((vid->dispctrl & 0x4000) ? 16000 : 0);

    if (vid->dispctrl & 0x100) {
        static const int palind[4] = { 0, 1, 4, 5 };

        for (int field = 0; field < 2; field++) {
            for (uint32_t yy = 0; yy < 2; yy++) {
                const uint8_t *src = base + (yy * 8192);

                for (uint32_t y = yy; y < 200; y += 2) {
                    uint32_t *dest = &buffer32->line[2 * y + field + dy][dx];

                    for (uint32_t x = 0; x < 320; x += 8) {
                        uint16_t sv = sw16((uint16_t) (src[0] | (src[1] << 8)));

                        src += 2;

                        for (uint32_t xx = 0; xx < 8; xx++) {
                            uint32_t colour = vid->palette[palind[(sv >> (14 - 2 * xx)) & 3]];

                            *dest++ = colour;
                            *dest++ = colour;
                        }
                    }
                }
            }
        }

        return;
    }

    {
        static const int palind[2] = { 0, 4 };

        for (int field = 0; field < 2; field++) {
            for (uint32_t yy = 0; yy < 2; yy++) {
                const uint8_t *src = base + (yy * 8192);

                for (uint32_t y = yy; y < 200; y += 2) {
                    uint32_t *dest = &buffer32->line[2 * y + field + dy][dx];

                    for (uint32_t x = 0; x < 640; x += 16) {
                        uint16_t sv = sw16((uint16_t) (src[0] | (src[1] << 8)));

                        src += 2;

                        for (uint32_t xx = 0; xx < 16; xx++) {
                            uint32_t colour = vid->palette[palind[(sv >> (15 - xx)) & 1]];

                            *dest++ = colour;
                        }
                    }
                }
            }
        }
    }
}

static void
mind_poll(void *priv)
{
    mindset_vid_t *vid = (mindset_vid_t *) priv;

    timer_advance_u64(&vid->timer, (uint64_t) (TIMER_USEC * (1000000.0 / 60.0)));

    if ((xsize != MIND_XSIZE) || (ysize != MIND_YSIZE) || video_force_resize_get()) {
        xsize = MIND_XSIZE;
        ysize = MIND_YSIZE;
        set_screen_size(xsize, ysize);

        if (video_force_resize_get())
            video_force_resize_set(0);
    }

    video_wait_for_buffer();

    mind_render(vid);

    video_blit_memtoscreen(0, 0, xsize, ysize);

    if (vid->irq != NULL) {
        vid->irq(vid->irq_priv, 1);
        timer_set_delay_u64(&vid->irq_timer, 10 * TIMER_USEC);
    }

    video_res_x = xsize;
    video_res_y = ysize;
    video_bpp   = 8;

    vid->frames++;
    frames++;
}

static void *
mind_vid_init(UNUSED(const device_t *info))
{
    mindset_vid_t *vid = (mindset_vid_t *) calloc(1, sizeof(mindset_vid_t));

    mindset_vid = vid;
    mindset_log = (getenv("MIND_LOG") != NULL);

    /* Until the guest uploads a palette the display shows the mono ramp the
       board powers up with; the BIOS only sends colours once it believes the
       display processor passed its self-test. */
    for (int i = 0; i < 16; i++)
        vid->palette[i] = (i == 0) ? 0x000000 : 0xffffff;

    video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_mindset);

    overscan_x = overscan_y = 0;

    /* The display RAM window: the CPU and the blitter both come through
       here, so the device that owns the memory owns the mapping too. */
    if (mindset_log)
        pclog("[MIND] video init: vram=%p mapping at %05X\n", (void *) vid->vram, MIND_VRAM_BASE);

    mem_mapping_add(&vid->vram_map, MIND_VRAM_BASE, MIND_VRAM_SIZE,
                    mind_vram_read, mind_vram_readw, mind_vram_readl,
                    mind_vram_write, mind_vram_writew, mind_vram_writel,
                    NULL, MEM_MAPPING_EXTERNAL, vid->vram);

    io_sethandler(0x8320, 0x0002,
                  mind_dispreg_read8, mind_dispreg_read16, NULL,
                  mind_dispreg_write8, mind_dispreg_write16, NULL, vid);
    io_sethandler(0x8322, 0x0002,
                  mind_dispctrl_read8, mind_dispctrl_read16, NULL,
                  mind_dispctrl_write8, mind_dispctrl_write16, NULL, vid);

    timer_add(&vid->irq_timer, mind_irq_lower, vid, 0);
    timer_add(&vid->timer, mind_poll, vid, 1);
    timer_set_delay_u64(&vid->timer, (uint64_t) (TIMER_USEC * (1000000.0 / 60.0)));

    return vid;
}

static void
mind_vid_close(void *priv)
{
    mindset_vid_t *vid = (mindset_vid_t *) priv;

    free(vid);
    mindset_vid = NULL;
}

static void
mind_vid_speed_changed(UNUSED(void *priv))
{
    /* The display runs from its own crystal; the CPU speed is irrelevant. */
}

const device_t mindset_video_device = {
    .name          = "Mindset Personal Computer Display",
    .internal_name = "mindset_video",
    .flags         = 0,
    .local         = 0,
    .init          = mind_vid_init,
    .close         = mind_vid_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = mind_vid_speed_changed,
    .force_redraw  = NULL,
    .config        = NULL
};
