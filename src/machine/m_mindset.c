/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Mindset Personal Computer (1984): 80186 at 6 MHz (12 MHz crystal,
 *          divided by two), 256 KB of RAM, 32 KB of display RAM at B8000h
 *          that the GCP blitter and the CPU both write, a 32 KB BIOS made of
 *          two 16 KB EPROMs on a 16-bit bus, two 8042s (system and sound
 *          controller) and an 8749 keyboard scanner, a 8272A floppy
 *          controller, eight blitter blend modes and a stereo DAC.
 *
 *          Step 2 (M1): the display device and the GCP blitter.  The blitter
 *          takes its orders from a command block at the top of display RAM
 *          (status word, global mode, packet address and segment), reads a
 *          14-word packet out of physical memory, applies one of eight blend
 *          modes to 16-bit slices of display RAM and marks the command done.
 *          The two 8042s, the 8749, the FDC and the modules are not
 *          modelled yet.
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

#include <86box/86box.h>
#include <86box/machine.h>
#include <86box/mem.h>
#include <86box/io.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/plat.h>
#include <86box/timer.h>
#include <86box/video.h>
#include <86box/i8018x.h>
#include <86box/keyboard.h>
#include <86box/dma.h>
#include <86box/fdd.h>
#include <86box/fdc.h>

#include "../cpu/cpu.h"

#define MIND_BIOS_EVEN    "roms/machines/mindset/mindset_u60_1.7_lo.bin"
#define MIND_BIOS_ODD     "roms/machines/mindset/mindset_u59_1.7_hi.bin"
#define MIND_BIOS_ADDR    0x000f8000
#define MIND_BIOS_SIZE    0x00008000

#define MIND_RAM_SIZE     0x00040000  /* 256 KB, the only RAM the board has */
#define MIND_VRAM_BASE    0x000b8000
#define MIND_VRAM_SIZE    0x00008000

/* The GCP's command block lives at the top of display RAM. */
#define MIND_CMD_STATUS   0x000bfd74
#define MIND_CMD_MODE     0x000bfd76
#define MIND_CMD_PACKET   0x000bfd78
#define MIND_CMD_SEGMENT  0x000bfd7a

/* Everything from 8000h up is the real I/O space; 0000h-7FFFh is the module
   bus, where the system MCU watches every access the CPU makes. */
#define MIND_TRAP_BASE    0x0000
#define MIND_TRAP_SIZE    0x8000

extern const device_t mindset_video_device;
extern uint8_t       *mindset_video_vram(void);
extern void           mindset_video_set_irq(void (*cb)(void *priv, int level), void *priv);

typedef struct {
    uint8_t idx;         /* register index latched from the index port */
    uint8_t regs[256];
} mind_mailbox_t;

/* The keyboard scanner hands one byte at a time to the system controller,
   which raises its interrupt line until the BIOS has picked the byte up. */
typedef struct {
    uint8_t key;
    int     valid;      /* a byte is waiting to be read out of register 8Ch */
    int     ready;      /* the BIOS sees this as status bit 5 */
    uint8_t queue[16];  /* bytes typed while the previous one was still there */
    int     qhead;
    int     qtail;
    int     qcount;
} mind_keyboard_t;

/* The controller is fed byte by byte by the CPU through 8054h, so the host
   side keeps a ring buffer and a terminal count the BIOS programs at 8058h. */
#define MIND_FIFO_SIZE 0x8000

/* The controller is the slave end of the 80186's second DMA channel. */
#define MIND_FDC_DMA_CH 1

typedef struct {
    uint8_t  buf[MIND_FIFO_SIZE];
    int      head;
    int      tail;
    int      count;
    uint32_t left;
} mind_fdc_fifo_t;

typedef struct {
    int            keytest;
    int            dump_at;
    int            badwrites;
    int            boot_at;
    int            fdc_irqs;
    int            dma_logged;
    int            log;
    int            vramdump;
    int            kicks;
    mind_mailbox_t sys;  /* system controller register file */
    mind_mailbox_t snd;  /* sound controller register file */
    uint8_t         sys_sts; /* status byte the controller reports */
    mind_keyboard_t kbd;
    void          *fdc_dev;      /* hosted floppy controller */
    fdc_t         *fdc;
    mind_fdc_fifo_t fifo;
    int            fdc_irq;      /* asserted, waiting for the BIOS to ack */
    int            fdc_sched;    /* the controller is still finishing up */
    pc_timer_t     fdc_timer;
    pc_timer_t     mcu_timer;
    pc_timer_t     dump_timer;
    pc_timer_t     sample_timer;
} mindset_t;

/* The controller takes a moment to answer, and the BIOS polls for it. */
#define MIND_MCU_BUSY     0x0a
#define MIND_MCU_DELAY    20

static mindset_t mindset;

static struct i8018x_pcb_t *mind_pcb;

/* The display's frame interrupt lands on line 2 of the 80186's own ICU. */
static void
mind_display_irq(void *priv, int level)
{
    if (mind_pcb != NULL)
        i8018x_pcb_icu_int(mind_pcb, 2, level);
}

/* The keyboard scanner sits behind the system controller: it leaves a byte for
   the controller, which raises line 3 until the BIOS reads it out of register
   8Ch inside its interrupt handler. */
/* One byte at a time leaves the controller: the next one only goes out once
   the BIOS has acknowledged the previous one, the way the real part behaves. */
static void
mind_keyboard_deliver(uint8_t code)
{
    mindset.kbd.key   = code;
    mindset.kbd.valid = 1;
    mindset.kbd.ready = 1;

    if (mind_pcb != NULL)
        i8018x_pcb_icu_int(mind_pcb, 3, 1);
}

static void
mind_keyboard_send(uint16_t val)
{
    if ((val & 0xff00) != 0x0000)
        return;

    if (mindset.log)
        pclog("[MIND] kbd send %04X ready=%d\n", val, mindset.kbd.ready);

    if (mindset.kbd.valid) {
        if (mindset.kbd.qcount < (int) sizeof(mindset.kbd.queue)) {
            mindset.kbd.queue[mindset.kbd.qtail] = (uint8_t) (val & 0xff);
            mindset.kbd.qtail = (mindset.kbd.qtail + 1) % (int) sizeof(mindset.kbd.queue);
            mindset.kbd.qcount++;
        }

        /* The request is retriggered so a byte that was acknowledged but
           never read out of 8Ch does not silence the controller: without a
           fresh edge the interrupt controller has nothing left to deliver. */
        if ((mind_pcb != NULL) && !mindset.kbd.ready) {
            mindset.kbd.ready = 1;
            i8018x_pcb_icu_int(mind_pcb, 3, 0);
            i8018x_pcb_icu_int(mind_pcb, 3, 1);
        }

        return;
    }

    mind_keyboard_deliver((uint8_t) (val & 0xff));
}

/* MIND_SAMPLE=<us>: how often the sampler prints CS:IP (default 20 ms). */
static int mind_interval = 20000;

static uint16_t
sw16(uint16_t v)
{
    return (uint16_t) ((v << 8) | (v >> 8));
}

static void
mind_trace(const char *dir, uint16_t port, uint32_t val, int width)
{
    if (!mindset.log)
        return;

    pclog("[MIND] io %s%d %04X %0*X @%04X:%04X\n", dir, width, port,
          width * 2, val, cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
}

/* The GCP is a bus master, so it sees physical memory, not the CPU's view. */
static uint16_t
mind_phys_rdw(uint32_t addr)
{
    return (uint16_t) (mem_readb_phys(addr & 0x0fffff) |
                       (mem_readb_phys((addr + 1) & 0x0fffff) << 8));
}

static void
mind_phys_wrw(uint32_t addr, uint16_t val)
{
    mem_writeb_phys(addr & 0x0fffff, (uint8_t) (val & 0xff));
    mem_writeb_phys((addr + 1) & 0x0fffff, (uint8_t) (val >> 8));
}

static uint16_t
mind_mask(int bits)
{
    return (uint16_t) ((bits >= 16) ? 0xffff : ((1 << bits) - 1));
}

/* Blend modes, in the order the command word selects them. */
static uint16_t mind_blend_0(uint16_t src, uint16_t dst) { return src; }
static uint16_t mind_blend_1(uint16_t src, uint16_t dst) { return (uint16_t) (src & dst); }
static uint16_t mind_blend_2(uint16_t src, uint16_t dst) { return (uint16_t) (src | dst); }
static uint16_t mind_blend_3(uint16_t src, uint16_t dst) { return (uint16_t) (src ^ dst); }
static uint16_t mind_blend_4(uint16_t src, uint16_t dst) { return (uint16_t) ~src; }
static uint16_t mind_blend_5(uint16_t src, uint16_t dst) { return (uint16_t) ((~src) & dst); }
static uint16_t mind_blend_6(uint16_t src, uint16_t dst) { return (uint16_t) ((~src) | dst); }
static uint16_t mind_blend_7(uint16_t src, uint16_t dst) { return (uint16_t) ((~src) ^ dst); }

static uint16_t (*const mind_blend[8])(uint16_t, uint16_t) = {
    mind_blend_0, mind_blend_1, mind_blend_2, mind_blend_3,
    mind_blend_4, mind_blend_5, mind_blend_6, mind_blend_7
};

/* One blit command: a 14-word packet whose fields are all byte-swapped in
   memory, with 16-bit segment bases for the source and the destination. */
static void
mind_blit(uint32_t packet)
{
    uint16_t mode    = sw16(mind_phys_rdw(packet + 0));
    uint16_t src_adr = sw16(mind_phys_rdw(packet + 2));
    uint16_t src_sft = sw16(mind_phys_rdw(packet + 4));
    uint16_t dst_adr = sw16(mind_phys_rdw(packet + 6));
    uint16_t dst_sft = sw16(mind_phys_rdw(packet + 8));
    uint16_t width   = sw16(mind_phys_rdw(packet + 10));
    uint16_t height  = sw16(mind_phys_rdw(packet + 12));
    uint16_t sy      = sw16(mind_phys_rdw(packet + 14));
    uint16_t dy      = sw16(mind_phys_rdw(packet + 16));
    uint16_t rmask   = sw16(mind_phys_rdw(packet + 18));
    uint16_t src_seg = sw16(mind_phys_rdw(packet + 20));
    uint16_t dst_seg = sw16(mind_phys_rdw(packet + 22));
    uint16_t wmask   = sw16(mind_phys_rdw(packet + 24));
    uint16_t kmask   = sw16(mind_phys_rdw(packet + 26));

    uint32_t src_base = (uint32_t) src_seg << 4;
    uint32_t dst_base = (uint32_t) dst_seg << 4;

    if (mindset.log)
        pclog("[MIND] gcp mode=%04X %04X:%04X.%X -> %04X:%04X.%X %dx%d step %d/%d mask %04X/%04X k=%04X\n",
              mode, src_seg, src_adr, src_sft, dst_seg, dst_adr, dst_sft,
              width, height, sy, dy, rmask, wmask, kmask);

    if (mode & 0x0200) {
        /* Pattern fill: one source word repeated across the row. */
        uint16_t pattern = sw16(mind_phys_rdw(src_base + src_adr));

        for (uint16_t w = 0; w != width / 2; w++) {
            mind_phys_wrw(dst_base + dst_adr, pattern);
            dst_adr += 2;
        }

        return;
    }

    if (mode & 0x4000) {
        /* Fast mode: a straight word copy. */
        uint32_t nw = (uint32_t) ((width + 15) >> 4);

        for (uint32_t y = 0; y < height; y++) {
            uint16_t src_cadr = src_adr;
            uint16_t dst_cadr = dst_adr;

            for (uint32_t w = 0; w != nw; w++) {
                mind_phys_wrw(dst_base + dst_cadr, mind_phys_rdw(src_base + src_cadr));
                src_cadr += 2;
                dst_cadr += 2;
            }

            src_adr += sy;
            dst_adr += dy;
        }

        return;
    }

    {
        uint16_t (*blend)(uint16_t, uint16_t) = mind_blend[(mode >> 2) & 7];
        uint16_t awmask  = (uint16_t) (((uint32_t) ((wmask << 16) | wmask)) >> (15 - dst_sft));
        uint16_t swmask;
        uint16_t mwmask;
        uint16_t ewmask;
        bool     preload;
        int      src_do_sft;
        int      dst_do_sft;
        uint16_t nw;

        if (dst_sft >= width) {
            swmask = (uint16_t) (mind_mask(dst_sft + 1) & ~mind_mask(dst_sft - width + 1));
            mwmask = 0xffff;
            ewmask = swmask;
        } else {
            swmask = mind_mask(dst_sft + 1);
            mwmask = 0xffff;
            ewmask = (uint16_t) ~mind_mask((dst_sft - width + 1) & 15);
        }

        swmask &= awmask;
        mwmask &= awmask;
        ewmask &= awmask;

        preload    = dst_sft > src_sft;
        src_do_sft = 15 - src_sft;
        dst_do_sft = (preload ? 31 : 15) - dst_sft;
        nw         = (uint16_t) ((width + (15 - dst_sft) + 15) >> 4);

        for (uint32_t y = 0; y < height; y++) {
            uint16_t src_cadr = src_adr;
            uint16_t dst_cadr = dst_adr;
            uint16_t cmask    = swmask;
            uint16_t nw1      = nw;
            uint32_t srcs     = 0;

            if (preload) {
                srcs = (uint32_t) sw16(mind_phys_rdw(src_base + src_cadr)) << src_do_sft;

                if (mode & 0x0100)
                    src_cadr += 2;
            }

            do {
                uint16_t src;
                uint16_t dst;
                uint16_t res;

                srcs = (srcs << 16) | ((uint32_t) sw16(mind_phys_rdw(src_base + src_cadr)) << src_do_sft);

                src = (uint16_t) ((srcs >> dst_do_sft) & rmask);
                dst = sw16(mind_phys_rdw(dst_base + dst_cadr));
                res = blend(src, dst);

                if (mode & 0x0040) {
                    uint16_t tmask;

                    switch ((mode >> 11) & 3) {
                        case 0:
                        default:
                            tmask = src;
                            break;

                        case 1:
                            tmask = (uint16_t) ((src >> 1) | src);
                            tmask = (uint16_t) ((tmask & 0x5555) * 0x3);
                            break;

                        case 2:
                            tmask = (uint16_t) ((src >> 2) | src);
                            tmask = (uint16_t) ((tmask >> 1) | tmask);
                            tmask = (uint16_t) ((tmask & 0x1111) * 0xf);
                            break;

                        case 3:
                            tmask = (uint16_t) ((src >> 4) | src);
                            tmask = (uint16_t) ((tmask >> 2) | tmask);
                            tmask = (uint16_t) ((tmask >> 1) | tmask);
                            tmask = (uint16_t) ((tmask & 0x0101) * 0xff);
                            break;
                    }

                    cmask &= tmask;
                }

                res = (uint16_t) ((dst & ~cmask) | (res & cmask));

                mind_phys_wrw(dst_base + dst_cadr, sw16(res));

                /* A blit whose destination does not read back is writing into
                   something other than display RAM. */
                if (mindset.log && (mindset.badwrites < 8)) {
                    uint16_t back = sw16(mind_phys_rdw(dst_base + dst_cadr));

                    if (back != res) {
                        pclog("[MIND] blit write lost at %05X: wrote %04X read %04X\n",
                              dst_base + dst_cadr, res, back);
                        mindset.badwrites++;
                    }
                }

                if (mode & 0x0100)
                    src_cadr += 2;
                dst_cadr += 2;

                nw1--;

                cmask = (nw1 == 1) ? ewmask : mwmask;
            } while (nw1);

            if (mode & 0x0100)
                src_adr += sy;
            dst_adr += dy;
        }
    }
}

/* Writing the kick port runs whatever the command block describes. */
/* The board's ROM signature check compares eight words the blitter left at
   1040h against a table in the BIOS, so both sides have to agree there. */
static void
mind_dump_sig(const char *tag)
{
    char line[128];
    int  n = 0;

    n += snprintf(line + n, sizeof(line) - n, "[MIND] sig %s ram:", tag);
    for (int i = 0; i < 16; i++)
        n += snprintf(line + n, sizeof(line) - n, " %02X", mem_readb_phys(0x1040 + i));
    n += snprintf(line + n, sizeof(line) - n, " flags1E=%04X f15=%04X",
                  (uint16_t) (mem_readb_phys(0x1e) | (mem_readb_phys(0x1f) << 8)),
                  (uint16_t) (mem_readb_phys(0x15) | (mem_readb_phys(0x16) << 8)));
    n += snprintf(line + n, sizeof(line) - n, " rom:");
    for (int i = 0; i < 16; i++)
        n += snprintf(line + n, sizeof(line) - n, " %02X", mem_readb_phys(0xfe106 + i));
    pclog("%s\n", line);
}

static void
mind_dump_vram(const char *tag)
{
    uint8_t *vram = mindset_video_vram();
    char     path[64];
    FILE    *f;

    if (vram == NULL)
        return;

    snprintf(path, sizeof(path), "/tmp/mind-%s.bin", tag);
    f = fopen(path, "wb");
    if (f == NULL)
        return;

    /* Read it back through the window the guest writes, so what the dump
       holds is what the display device actually answers. */
    for (int i = 0; i < MIND_VRAM_SIZE; i++)
        fputc(mem_readb_phys(0xB8000 + i), f);

    fclose(f);
    pclog("[MIND] vram dumped to %s (%p)\n", path, vram);
}

static void
mind_gcp_kick(uint16_t port, uint16_t val, void *priv)
{
    mind_trace("w", 0x8300, val, 2);

    mindset.kicks++;

    if ((mindset.dump_at != 0) && (mindset.kicks == mindset.dump_at))
        mind_dump_vram("at");

    if (mindset.vramdump) {
        switch (mindset.kicks) {
            case 1:
                mind_dump_vram("kick1");
                mind_dump_sig("kick1");
                break;

            case 2:
                mind_dump_sig("kick2");
                break;

            case 20:
                mind_dump_vram("kick20");
                break;

            case 40:
                mind_dump_vram("kick40");
                break;

            case 56:
                mind_dump_vram("kick56");
                break;

            case 100:
                mind_dump_vram("kick100");
                break;

            case 400:
                mind_dump_vram("kick400");
                break;

            case 800:
                mind_dump_vram("kick800");
                break;

            case 1200:
                mind_dump_vram("kick1200");
                break;

            default:
                break;
        }
    }

    uint16_t gmode = sw16(mind_phys_rdw(MIND_CMD_MODE));
    uint16_t padr  = sw16(mind_phys_rdw(MIND_CMD_PACKET));
    uint16_t pseg  = sw16(mind_phys_rdw(MIND_CMD_SEGMENT));

    if (mindset.log) {
        uint8_t *vram = mindset_video_vram();
        int      shown = 0;

        pclog("[MIND] gcp raw: status=%04X mode=%04X pkt=%04X seg=%04X  phys[BFD74]=%02X\n",
              mind_phys_rdw(MIND_CMD_STATUS), mind_phys_rdw(MIND_CMD_MODE),
              mind_phys_rdw(MIND_CMD_PACKET), mind_phys_rdw(MIND_CMD_SEGMENT),
              mem_readb_phys(MIND_CMD_STATUS));

        if (vram != NULL) {
            for (int i = 0; (i < MIND_VRAM_SIZE) && (shown < 8); i++) {
                if (vram[i] != 0x00) {
                    pclog("[MIND] vram[%04X] = %02X\n", i, vram[i]);
                    shown++;
                }
            }
        }
    }

    switch (gmode) {
        case 0x0005:
        case 0x0101:
            mind_blit(((uint32_t) pseg << 4) + padr);
            break;

        default:
            if (mindset.log)
                pclog("[MIND] gcp kick with global mode %04X (not implemented)\n", gmode);
            break;
    }

    /* The blitter is instantaneous here, so the done bits go up at once. */
    mind_phys_wrw(MIND_CMD_STATUS,
                  (uint16_t) (mind_phys_rdw(MIND_CMD_STATUS) | 0x0700));
}

/* The module bus.  On the real board these cycles are serialised to the
   system MCU, which answers for whatever module sits in the slot. */
static uint8_t
mind_trap_read(uint16_t port, void *priv)
{
    mind_trace("r", port, 0xff, 1);
    return 0xff;
}

static uint16_t
mind_trap_readw(uint16_t port, void *priv)
{
    mind_trace("r", port, 0xffff, 2);
    return 0xffff;
}

static void
mind_trap_write(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);
}

static void
mind_trap_writew(uint16_t port, uint16_t val, void *priv)
{
    mind_trace("w", port, val, 2);
}

/* The system controller's mailbox.  The BIOS talks to it with an index port, a
   status port and a data port; the firmware behind that keeps a small register
   file, and the values below are what it answers on real hardware (harvested
   from MAME's emulation of the same firmware).  The status byte reports "ready"
   in every direction, which is what the synchronous BIOS handshake expects. */
static void
mind_mcu_reset(mindset_t *m)
{
    int i;

    memset(&m->sys, 0x00, sizeof(m->sys));
    memset(&m->snd, 0x00, sizeof(m->snd));

    m->sys.regs[0x00] = 0xbf;  /* POST control byte */
    m->sys.regs[0x01] = 0x7b;  /* configuration byte */
    m->sys.regs[0x81] = 0x01;
    m->sys.regs[0x82] = 0x07;  /* machine state block */

    for (i = 0x83; i <= 0x8b; i++)
        m->sys.regs[i] = 0x01;

    m->sys.regs[0x9d] = 0x01;  /* keyboard status */
    m->sys.regs[0x9e] = 0x01;  /* monitor type; MIND_MON overrides it */
}

static void
mind_mcu_ready(void *priv)
{
    mindset.sys_sts = 0x00;
}

/* Both status bits go up while the controller works on the byte it was just
   handed, and drop when it is done. */
static void
mind_mcu_busy(void)
{
    mindset.sys_sts = MIND_MCU_BUSY;
    timer_set_delay_u64(&mindset.mcu_timer, MIND_MCU_DELAY * TIMER_USEC);
}

static uint8_t
mind_sys_status_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    uint8_t ret = mindset.sys_sts;

    /* Bit 5 tells the interrupt handler that a keyboard byte is waiting; it
       acknowledges with a 40h write and then reads register 8Ch. */
    if (mindset.kbd.ready)
        ret |= 0x20;

    mind_trace("r", 0x8282, ret, 1);

    return ret;
}

static void
mind_sys_index_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    mind_trace("w", 0x8282, val, 1);

    /* 40h and 41h are the two acknowledgement codes the handler sends; the
       first one releases the keyboard interrupt line. */
    switch (val) {
        case 0x40:
            /* The acknowledge only releases the interrupt; the byte itself
               stays in register 8Ch until the keyboard handler reads it. */
            if (mindset.kbd.ready) {
                mindset.kbd.ready = 0;

                if (mind_pcb != NULL)
                    i8018x_pcb_icu_int(mind_pcb, 3, 0);
            }
            return;

        case 0x41:
            return;

        default:
            break;
    }

    mindset.sys.idx = val;
    mind_mcu_busy();
}

static uint8_t
mind_sys_data_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    uint8_t ret;

    if ((mindset.sys.idx == 0x8c) && mindset.kbd.valid) {
        ret = mindset.kbd.key;

        mindset.kbd.valid = 0;
        mindset.kbd.ready = 0;

        if (mind_pcb != NULL)
            i8018x_pcb_icu_int(mind_pcb, 3, 0);

        /* Anything typed while this byte was waiting goes out now. */
        if (mindset.kbd.qcount > 0) {
            uint8_t next = mindset.kbd.queue[mindset.kbd.qhead];

            mindset.kbd.qhead = (mindset.kbd.qhead + 1) % (int) sizeof(mindset.kbd.queue);
            mindset.kbd.qcount--;
            mind_keyboard_deliver(next);
        }
    } else {
        ret = mindset.sys.regs[mindset.sys.idx];
    }

    mind_trace("r", 0x8280, ret, 1);

    return ret;
}

static void
mind_sys_data_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    mind_trace("w", 0x8280, val, 1);
    mind_mcu_busy();

    /* The BIOS writes a value twice; keeping the last one is what the
       firmware does too.  Bit 6 of the configuration byte is an input the
       firmware feeds from the board, so writing it changes nothing. */
    if (mindset.sys.idx == 0x01)
        mindset.sys.regs[0x01] = (uint8_t) ((val & ~0x40) | 0x40);
    else
        mindset.sys.regs[mindset.sys.idx] = val;
}

static uint16_t
mind_sys_status_readw(uint16_t port, void *priv)
{
    return (uint16_t) (mind_sys_status_read(port, priv) |
                       (mind_sys_data_read(port + 1, priv) << 8));
}

static void
mind_sys_index_writew(uint16_t port, uint16_t val, void *priv)
{
    mind_sys_index_write(port, (uint8_t) (val & 0xff), priv);
    mind_sys_data_write(port + 1, (uint8_t) (val >> 8), priv);
}

/* The sound controller takes a two-byte command and never has to answer: the
   BIOS polls its status and never reads a data byte back. */
static uint8_t
mind_snd_status_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    mind_trace("r", 0x82a2, 0x00, 1);

    return 0x00;
}

static uint8_t
mind_snd_data_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    mind_trace("r", 0x82a0, 0xff, 1);

    return 0xff;
}

static void
mind_snd_index_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    mind_trace("w", 0x82a2, val, 1);

    mindset.snd.idx = val;
}

static void
mind_snd_data_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    mind_trace("w", 0x82a0, val, 1);

    mindset.snd.regs[mindset.snd.idx] = val;
}

static uint16_t
mind_snd_status_readw(uint16_t port, void *priv)
{
    return (uint16_t) (mind_snd_status_read(port, priv) |
                       (mind_snd_data_read(port + 1, priv) << 8));
}

static void
mind_snd_index_writew(uint16_t port, uint16_t val, void *priv)
{
    mind_snd_index_write(port, (uint8_t) (val & 0xff), priv);
    mind_snd_data_write(port + 1, (uint8_t) (val >> 8), priv);
}

/* Everything else in the board's I/O block: the FDC, the module slots.  The
   display registers belong to the display device and the kick port to the
   blitter above. */
static uint8_t
mind_io_read(uint16_t port, void *priv)
{
    uint8_t ret = 0xff;

    /* An empty module slot leaves the bus pulled high, which is what makes the
       BIOS skip it; the floppy controller's neighbours in the board block read
       low instead, the way the unpopulated trap hardware does. */
    if ((port >= 0x8040) && (port <= 0x804f))
        ret = 0x00;

    mind_trace("r", port, ret, 1);
    return ret;
}

static uint16_t
mind_io_readw(uint16_t port, void *priv)
{
    return (uint16_t) (mind_io_read(port, priv) |
                       (mind_io_read(port + 1, priv) << 8));
}

static void
mind_io_write(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);
}

static void
mind_io_writew(uint16_t port, uint16_t val, void *priv)
{
    mind_trace("w", port, val, 2);
}

/* MIND_KEYTEST=1 pushes a space bar press once the BIOS is waiting for a key,
   so the boot path can be exercised before the real keyboard is wired. */
/* MIND_KEYTEST plays a short key script so the boot path can be exercised
   without a keyboard: space gets past the diagnostics, then Alt and the Reset
   key (57h) ask the BIOS to leave the configuration screen. */
static const struct {
    uint64_t delay_us;
    uint8_t  code;
} mind_key_script[] = {
    { 3000000, 0x39 },  /* space: leave the diagnostics screen   */
    {  300000, 0xb9 },
    { 6000000, 0x38 },  /* the configuration screen is up by now:
                           Alt down ...                            */
    {   80000, 0x57 },  /* ... Reset, which saves and leaves it   */
    {  300000, 0xd7 },
    {  200000, 0xb8 },
    { 5000000, 0x39 },  /* answer the POST that follows           */
    {  300000, 0xb9 },
    { 4000000, 0x39 },
    {  300000, 0xb9 },
};

static int        mind_key_pos;
static pc_timer_t mind_key_timer;

static void
mind_key_next(void *priv)
{
    if (mind_key_pos >= (int) (sizeof(mind_key_script) / sizeof(mind_key_script[0])))
        return;

    uint8_t code = mind_key_script[mind_key_pos].code;

    pclog("[MIND] key test: sending %02X\n", code);
    mind_keyboard_send(code);
    mind_key_pos++;

    if (mind_key_pos < (int) (sizeof(mind_key_script) / sizeof(mind_key_script[0]))) {
        uint64_t delay = mind_key_script[mind_key_pos].delay_us;

        timer_set_delay_u64(&mind_key_timer, delay * TIMER_USEC);
    }
}

static void
mind_key_test(void *priv)
{
    timer_set_delay_u64(&mind_key_timer, 0);
    mind_key_next(NULL);
}

/* ---- floppy controller -------------------------------------------------
   The 8272A sits at 8060h/8062h and is fed through 8054h; the BIOS programs a
   byte count at 8058h and acknowledges the interrupt by reading 805Ch. */

static void
mind_fifo_reset(void)
{
    mindset.fifo.head  = 0;
    mindset.fifo.tail  = 0;
    mindset.fifo.count = 0;
}

static void
mind_fifo_push(uint8_t val)
{
    if (mindset.fifo.count >= MIND_FIFO_SIZE)
        return;

    mindset.fifo.buf[mindset.fifo.tail] = val;
    mindset.fifo.tail = (mindset.fifo.tail + 1) % MIND_FIFO_SIZE;
    mindset.fifo.count++;
}

static int
mind_fifo_pop(void)
{
    int val;

    if (mindset.fifo.count == 0)
        return -1;

    val = mindset.fifo.buf[mindset.fifo.head];
    mindset.fifo.head = (mindset.fifo.head + 1) % MIND_FIFO_SIZE;
    mindset.fifo.count--;

    return val;
}

/* One byte of the channel's traffic: the controller asks for a transfer and the
   80186 moves the byte between memory and the board's data port, where the
   handlers below hand it over. */
static int
mind_fdc_transfer(void)
{
    dma_set_drq(MIND_FDC_DMA_CH, 1);
    dma_set_drq(MIND_FDC_DMA_CH, 0);

    if (mindset.fifo.left != 0)
        mindset.fifo.left--;

    return (mindset.fifo.left == 0) ? DMA_OVER : 0;
}

static int
mind_fdc_dma_read(void *priv)
{
    int val;

    mind_fdc_transfer();

    val = mind_fifo_pop();

    return (val < 0) ? DMA_NODATA : val;
}

static int
mind_fdc_dma_write(void *priv, uint8_t val)
{
    mind_fifo_push(val);

    return mind_fdc_transfer();
}

static void
mind_fdc_drq(void *priv, int set)
{
    /* The transfer itself is driven from the data callbacks above, which run
       after the request goes up; the line is only interesting in the log. */
    if (mindset.log)
        pclog("[MIND] fdc drq %d\n", set);
}

static void
mind_fdc_irq(void *priv, int set)
{
    if (mindset.log)
        pclog("[MIND] fdc irq %d\n", set);

    /* The board latches the request: the line stays up until the BIOS
       acknowledges it by reading 805Ch, even if the controller has already
       dropped its own INTRQ.  The request also arrives a little later than the
       command that caused it - the reset takes a moment - which is what lets
       the BIOS clear the controller and still see the interrupt afterwards. */
    if (set) {
        mindset.fdc_sched = 1;
        timer_set_delay_u64(&mindset.fdc_timer, 200 * TIMER_USEC);
    }
}

static const fdc_host_t mind_fdc_host = {
    .dma_read  = mind_fdc_dma_read,
    .dma_write = mind_fdc_dma_write,
    .drq       = mind_fdc_drq,
    .irq       = mind_fdc_irq
};

/* MIND_DUMP_BOOT=<n>: write the 512 bytes at 7C00h after the nth controller
   interrupt, which is where a boot sector ends up. */
static void
mind_dump_boot(const char *tag)
{
    char  path[64];
    FILE *f;

    snprintf(path, sizeof(path), "/tmp/mind-%s.bin", tag);

    f = fopen(path, "wb");
    if (f == NULL)
        return;

    for (int i = 0; i < 0x100000; i++)
        fputc(mem_readb_phys(i), f);

    fclose(f);
    pclog("[MIND] boot sector dumped to %s\n", path);
}

/* MIND_DUMP_SEC=<n>: write display RAM once, n seconds in. */
static void
mind_dump_timed(void *priv)
{
    mind_dump_vram("sec");
}

static void
mind_fdc_raise(void *priv)
{
    if (!mindset.fdc_sched)
        return;

    mindset.fdc_sched = 0;
    mindset.fdc_irq   = 1;

    if (mindset.boot_at != 0) {
        char tag[16];

        mindset.fdc_irqs++;

        if (mindset.fdc_irqs <= mindset.boot_at) {
            snprintf(tag, sizeof(tag), "boot%d", mindset.fdc_irqs);
            mind_dump_boot(tag);
        }
    }

    if (mindset.log)
        pclog("[MIND] fdc irq raised\n");

    if (mind_pcb != NULL)
        i8018x_pcb_icu_int(mind_pcb, 0, 1);
}

/* 8050h: drive select, motor and the reset pulse, in the board's own layout. */
static void
mind_fdc_ctrl_write(uint16_t port, uint8_t val, void *priv)
{
    uint8_t dor = 0x0c; /* enabled, out of reset */

    mind_trace("w", port, val, 1);

    if (val & 0x01)
        dor |= 0x01; /* selected drive */

    if (val & 0x02)
        dor |= (uint8_t) (0x10 << (val & 1)); /* motor for that drive */

    /* The board's bit 2 is a reset pulse, the opposite sense of the PC's DOR
       bit, so the controller is pulsed here and stays enabled for the PC side
       of the model. */
    if (val & 0x04) {
        /* A reset starts the controller over: whatever the previous command
           left pending is gone and this reset raises its own interrupt.  The
           enable bit is dropped and raised again because that edge, not a
           bare reset call, is what the controller answers with an interrupt. */
        mindset.fdc_sched = 0;
        mindset.fdc_irq   = 0;

        if (mind_pcb != NULL)
            i8018x_pcb_icu_int(mind_pcb, 0, 0);

        fdc_write(0x03f2, (uint8_t) (dor & ~0x04), mindset.fdc);
    }

    fdc_write(0x03f2, dor, mindset.fdc);
}

static uint8_t
mind_fdc_msr_read(uint16_t port, void *priv)
{
    uint8_t ret = fdc_read(0x03f4, mindset.fdc);

    mind_trace("r", port, ret, 1);

    return ret;
}

static uint8_t
mind_fdc_data_read(uint16_t port, void *priv)
{
    uint8_t ret = fdc_read(0x03f5, mindset.fdc);

    mind_trace("r", port, ret, 1);

    return ret;
}

static void
mind_fdc_data_write(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);

    fdc_write(0x03f5, val, mindset.fdc);
}

static uint8_t
mind_fdc_dma_read_port(uint16_t port, void *priv)
{
    int val = mind_fifo_pop();

    if ((val < 0) && (mindset.fifo.left != 0))
        val = 0xff;

    mind_trace("r", port, (val < 0) ? 0xff : val, 1);

    return (uint8_t) ((val < 0) ? 0xff : val);
}

static void
mind_fdc_dma_write_port(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);

    mind_fifo_push(val);
}

static void
mind_fdc_count_write(uint16_t port, uint16_t val, void *priv)
{
    mind_trace("w", port, val, 2);

    mindset.fifo.left = val;
}

static uint8_t
mind_fdc_ack_read(uint16_t port, void *priv)
{
    mind_trace("r", port, 0x00, 1);

    if (mindset.fdc_irq) {
        mindset.fdc_irq = 0;

        if (mind_pcb != NULL)
            i8018x_pcb_icu_int(mind_pcb, 0, 0);
    }

    return 0x00;
}

/* Sampler: without a serial console the only way to tell "the CPU is still
   running the ROM" from "it wedged in a loop" is where it is over time. */
static void
mind_sample(void *priv)
{
    pclog("[MIND] pc=%04X:%04X frames=%d\n", cpu_state.seg_cs.seg,
          (uint16_t) cpu_state.pc, frames);

    /* MIND_VRAMDUMP=1: drop the display RAM to /tmp at fixed frame numbers,
       which separates "the guest drew nothing" from "the renderer maps the
       pixels wrongly" and lets the same frames be compared with another
       emulator's dump. */
    if (mindset.vramdump && (frames >= 100) && ((frames % 100) == 0)) {
        static uint64_t dumped = 0;
        uint8_t        *vram   = mindset_video_vram();

        if ((vram != NULL) && (dumped != frames)) {
            char path[64];
            FILE *f;

            snprintf(path, sizeof(path), "/tmp/mind-vram-%llu.bin",
                     (unsigned long long) frames);
            f = fopen(path, "wb");
            if (f != NULL) {
                fwrite(vram, 1, MIND_VRAM_SIZE, f);
                fclose(f);
                dumped = frames;
                pclog("[MIND] vram dumped at frame %llu\n", (unsigned long long) frames);
            }
        }
    }

    timer_set_delay_u64(&mindset.sample_timer, mind_interval * TIMER_USEC);
}

int
machine_mindset_init(const machine_t *model)
{
    int      ret;

    ret = bios_load_interleaved(MIND_BIOS_EVEN, MIND_BIOS_ODD, MIND_BIOS_ADDR,
                                MIND_BIOS_SIZE, 0);
    if (bios_only || !ret)
        return ret;

    memset(&mindset, 0x00, sizeof(mindset));
    mind_mcu_reset(&mindset);
    mindset.log      = (getenv("MIND_LOG") != NULL);
    mindset.vramdump = (getenv("MIND_VRAMDUMP") != NULL);
    mindset.dump_at  = 0;

    if (getenv("MIND_DUMP_AT") != NULL)
        mindset.dump_at = atoi(getenv("MIND_DUMP_AT"));

    mindset.boot_at = 0;

    if (getenv("MIND_DUMP_BOOT") != NULL)
        mindset.boot_at = atoi(getenv("MIND_DUMP_BOOT"));
    mindset.keytest  = (getenv("MIND_KEYTEST") != NULL);

    if (getenv("MIND_MON") != NULL)
        mindset.sys.regs[0x9e] = (uint8_t) strtoul(getenv("MIND_MON"), NULL, 16);
    if (getenv("MIND_SAMPLE") != NULL)
        mind_interval = atoi(getenv("MIND_SAMPLE"));

    pclog("[MIND] init: %d KB RAM, BIOS 32 KB at %05X, VRAM 32 KB at %05X\n",
          mem_size, MIND_BIOS_ADDR, MIND_VRAM_BASE);

    machine_common_init(model);

    /* The board has 256 KB and no upper memory: everything above it reads as
       an open bus.  The display RAM is a window inside the chip's 1 MB. */
    mem_set_mem_state_both(0x00000000, MIND_RAM_SIZE,
                           MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);

    /* The display device owns the 32 KB of display RAM and installs its own
       window at B8000h; the blitter reaches the same memory through the
       physical bus. */
    device_add(&mindset_video_device);

    /* 80186 on-chip peripherals.  Neither DMA channel has a device yet, and
       the chip selects are stored but not decoded (intel_8018x.c:13). */
    mind_pcb = device_add(&i8018x_pcb_device);
    if (mind_pcb != NULL) {
        i8018x_pcb_set_dma_channels(mind_pcb, -1, MIND_FDC_DMA_CH);
        mindset_video_set_irq(mind_display_irq, NULL);
    }

    /* The BIOS expects XT-style scan codes in register 8Ch. */
    keyboard_set_table(scancode_xt);
    keyboard_send = mind_keyboard_send;

    /* The 8272A hangs off the board's own ports, not the PC ones, so it runs as
       a hosted controller: the CPU moves every byte through 8054h. */
    mind_fifo_reset();
    mindset.fdc_irq   = 0;
    mindset.fdc_sched = 0;
    timer_add(&mindset.fdc_timer, mind_fdc_raise, NULL, 0);

    if (getenv("MIND_DUMP_SEC") != NULL) {
        uint64_t when = (uint64_t) atoi(getenv("MIND_DUMP_SEC")) * 1000000u;

        timer_add(&mindset.dump_timer, mind_dump_timed, NULL, 0);
        timer_set_delay_u64(&mindset.dump_timer, when);
    }
    mindset.fdc_dev = device_add(&fdc_hosted_device);
    mindset.fdc     = (fdc_t *) mindset.fdc_dev;

    if (mindset.fdc != NULL) {
        /* The board runs its drives at 250 kbit/s, double density; the hosted
           controller defaults to high density and then cannot find a sector. */
        mindset.fdc->rate = 2;
        fdc_set_host(mindset.fdc, &mind_fdc_host, &mindset);

        if (mindset.log)
            pclog("[MIND] fdc: drives %d/%d, types %d/%d, image '%s'\n",
                  (mindset.fdc->fdd[0] != NULL), (mindset.fdc->fdd[1] != NULL),
                  fdd_get_type(&drives[0]), fdd_get_type(&drives[1]),
                  drives[0].image_path);
    }

    io_sethandler(0x8050, 0x0001,
                  NULL, NULL, NULL,
                  mind_fdc_ctrl_write, NULL, NULL, NULL);
    io_sethandler(0x8054, 0x0001,
                  mind_fdc_dma_read_port, NULL, NULL,
                  mind_fdc_dma_write_port, NULL, NULL, NULL);
    io_sethandler(0x8058, 0x0001,
                  NULL, NULL, NULL,
                  NULL, mind_fdc_count_write, NULL, NULL);
    io_sethandler(0x805c, 0x0001,
                  mind_fdc_ack_read, NULL, NULL,
                  NULL, NULL, NULL, NULL);
    io_sethandler(0x8060, 0x0001,
                  mind_fdc_msr_read, NULL, NULL,
                  NULL, NULL, NULL, NULL);
    io_sethandler(0x8062, 0x0001,
                  mind_fdc_data_read, NULL, NULL,
                  mind_fdc_data_write, NULL, NULL, NULL);

    io_sethandler(MIND_TRAP_BASE, MIND_TRAP_SIZE,
                  mind_trap_read, mind_trap_readw, NULL,
                  mind_trap_write, mind_trap_writew, NULL, NULL);

    /* 8040-804F, 8064-827F, 8284-829F and 82A4-82FF: the module slots and the
       leftovers of the board's I/O block (the controller owns 8050-8063). */
    io_sethandler(0x8040, 0x0010,
                  mind_io_read, mind_io_readw, NULL,
                  mind_io_write, mind_io_writew, NULL, NULL);
    io_sethandler(0x8064, 0x021c,
                  mind_io_read, mind_io_readw, NULL,
                  mind_io_write, mind_io_writew, NULL, NULL);
    io_sethandler(0x8284, 0x001c,
                  mind_io_read, mind_io_readw, NULL,
                  mind_io_write, mind_io_writew, NULL, NULL);
    io_sethandler(0x82a4, 0x005c,
                  mind_io_read, mind_io_readw, NULL,
                  mind_io_write, mind_io_writew, NULL, NULL);
    /* 8280-8283: the system controller's data and index/status ports. */
    io_sethandler(0x8280, 0x0001,
                  mind_sys_data_read, NULL, NULL,
                  mind_sys_data_write, NULL, NULL, NULL);
    io_sethandler(0x8282, 0x0001,
                  mind_sys_status_read, mind_sys_status_readw, NULL,
                  mind_sys_index_write, mind_sys_index_writew, NULL, NULL);
    /* 82A0-82A3: the sound controller's data and command ports. */
    io_sethandler(0x82a0, 0x0001,
                  mind_snd_data_read, NULL, NULL,
                  mind_snd_data_write, NULL, NULL, NULL);
    io_sethandler(0x82a2, 0x0001,
                  mind_snd_status_read, mind_snd_status_readw, NULL,
                  mind_snd_index_write, mind_snd_index_writew, NULL, NULL);
    /* The GCP is commanded through one write-only port. */
    io_sethandler(0x8300, 0x0002,
                  NULL, NULL, NULL,
                  NULL, mind_gcp_kick, NULL, NULL);
    /* 8302-831F and 8324-833F: the rest of the board's I/O block. */
    io_sethandler(0x8302, 0x001e,
                  mind_io_read, mind_io_readw, NULL,
                  mind_io_write, mind_io_writew, NULL, NULL);
    io_sethandler(0x8324, 0x001c,
                  mind_io_read, mind_io_readw, NULL,
                  mind_io_write, mind_io_writew, NULL, NULL);

    timer_add(&mindset.mcu_timer, mind_mcu_ready, NULL, 0);
    timer_add(&mind_key_timer, mind_key_next, NULL, 0);

    if (mindset.keytest) {
        static pc_timer_t key_timer;

        timer_add(&key_timer, mind_key_test, NULL, 0);
        timer_set_delay_u64(&key_timer, 6ULL * 1000000ULL * TIMER_USEC);
    }
    timer_add(&mindset.sample_timer, mind_sample, NULL, 0);
    timer_set_delay_u64(&mindset.sample_timer, mind_interval * TIMER_USEC);

    return 1;
}

const device_t mindset_device = {
    .name          = "Mindset Personal Computer",
    .internal_name = "mindset",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
