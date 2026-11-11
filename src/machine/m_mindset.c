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
#include <86box/mcs48.h>
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
extern void           mindset_video_set_irq(void (*cb)(void *priv, int line, int level), void *priv);

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
    int            probes;
    int            log;
    int            vramdump;
    int            kicks;
    mind_mailbox_t sys;  /* system controller register file */
    mind_mailbox_t snd;  /* sound controller register file */
    uint8_t         sys_sts; /* status byte the controller reports */
    mind_keyboard_t kbd;
    void          *fdc_dev;      /* hosted floppy controller */
    fdc_t         *fdc;
    mind_fdc_fifo_t cmdfifo;   /* command and result bytes, 8062h */
    mind_fdc_fifo_t datafifo;  /* sector bytes for the DMA, 8054h */
    int            fdc_irq;      /* asserted, waiting for the BIOS to ack */
    int            fdc_sched;    /* the controller is still finishing up */
    pc_timer_t     fdc_timer;
    pc_timer_t     mcu_timer;
    pc_timer_t     dump_timer;
    pc_timer_t     sample_timer;
    pc_timer_t     fast_timer;
    pc_timer_t     rtc_timer;
    pc_timer_t     tick_timer;
    int            tick_pending;
    int            fdc_portdma;
    mcs48         *mcu;
    int            mcu_fw;
    pc_timer_t     mcu_fw_timer;
    int            jump_reported;
} mindset_t;

/* The controller takes a moment to answer, and the BIOS polls for it. */
#define MIND_MCU_BUSY     0x0a
#define MIND_MCU_DELAY    20

static mindset_t mindset;

static struct i8018x_pcb_t *mind_pcb;

/* The display's frame interrupt lands on line 2 of the 80186's own ICU. */
/* Every interrupt the board raises goes through here so its delivery can be
   watched: a line that fires inside a critical section corrupts the return
   frame on the BIOS's stack and the machine comes back to a wild address. */
static int mind_int_log_left = 20000;

static void
mind_icu_int(int line, int level)
{
    if (mind_pcb == NULL)
        return;

    if (level && (mind_int_log_left > 0)) {
        mind_int_log_left--;
        pclog("[MIND] int line=%d type=%02X at %04X:%04X sp=%04X\n",
              line, 0x0c + line, cpu_state.seg_cs.seg,
              (uint16_t) cpu_state.pc, (uint16_t) cpu_state.regs[4].w);
    }

    i8018x_pcb_icu_int(mind_pcb, line, level);
}

static void
mind_display_irq(void *priv, int line, int level)
{
    if (mind_pcb == NULL)
        return;

    /* Two lines leave the display: INT1 at vertical blank (a level) and INT2
       at the scan line compare.  The BIOS's display bookkeeping - the pending
       mode-change flags - runs on the vblank one. */
    if (line == 1) {
        if (getenv("MIND_NOINT1") == NULL)
            mind_icu_int(1, level);
    } else {
        mind_icu_int(2, level);
    }
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
        mind_icu_int(3, 1);
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
            mind_icu_int(3, 0);
            mind_icu_int(3, 1);
        }

        return;
    }

    mind_keyboard_deliver((uint8_t) (val & 0xff));
}

/* MIND_SAMPLE=<us>: how often the sampler prints CS:IP (default 20 ms). */
static int mind_interval = 20000;
static uint16_t mind_prev_cs, mind_prev_ip;

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

    /* The event slots the BIOS polls are filled by blits, not by the CPU, so
       a blit that lands there is worth a line of its own. */
    if (getenv("MIND_SLOTLOG") != NULL) {
        uint32_t slot_lo = 0xb8000u + 0x7d50u;
        uint32_t slot_hi = 0xb8000u + 0x7d5fu;
        uint32_t end     = dst_base + dst_adr + (uint32_t) (width ? width : 1) * 2u;

        if ((dst_base + dst_adr) <= slot_hi && end >= slot_lo)
            pclog("[MIND] slot blit mode=%04X dst=%05X w=%d h=%d src=%05X\n",
                  mode, dst_base + dst_adr, width, height, src_base + src_adr);
    }

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

                if (mindset.log && (mindset.probes < 4) && (dst_base + dst_cadr >= 0xB8000) &&
                    (dst_base + dst_cadr < 0xC0000)) {
                    uint8_t *v = mindset_video_vram();

                    pclog("[MIND] blit probe %05X wrote %04X vram[%04X]=%02X\n",
                          dst_base + dst_cadr, res,
                          dst_base + dst_cadr - 0xB8000,
                          (v != NULL) ? v[dst_base + dst_cadr - 0xB8000] : 0xff);
                    mindset.probes++;
                }

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

    fwrite(vram, 1, MIND_VRAM_SIZE, f);

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
   system MCU, which answers for whatever module sits in the slot: each access
   is queued as a pair of words, the queue is drained by the board's DMA into
   memory, and the end of the transfer raises the module interrupt. */
#define MIND_TRAP_SIZE 0x100

static uint16_t mind_trap_queue[MIND_TRAP_SIZE];
static int      mind_trap_head, mind_trap_tail, mind_trap_count;
static int      mind_trap_int;

static void
mind_trap_push(uint16_t word)
{
    if (mind_trap_count >= MIND_TRAP_SIZE)
        return;

    mind_trap_queue[mind_trap_tail] = word;
    mind_trap_tail = (mind_trap_tail + 1) & (MIND_TRAP_SIZE - 1);
    mind_trap_count++;
}

static uint16_t
mind_trap_pop(void)
{
    uint16_t word;

    if (mind_trap_count == 0)
        return 0x0000;

    word = mind_trap_queue[mind_trap_head];
    mind_trap_head = (mind_trap_head + 1) & (MIND_TRAP_SIZE - 1);
    mind_trap_count--;

    return word;
}

static void
mind_trap_queue_access(uint16_t port, uint16_t data)
{
    mind_trap_push((uint16_t) ((port << 1) | 0x8000));
    mind_trap_push(data);

    /* The board asks the DMA channel for the pair; the channel reads it back
       through 8040h and writes it into memory, and the last read raises the
       module interrupt. */
    if (mind_pcb != NULL)
        i8018x_pcb_dma_request(mind_pcb, 1);

    /* The board's DMA reads the pair back out of 8040h and writes it into
       memory, so the guest never sees the bus cycle itself. */
}

static uint8_t
mind_trap_read(uint16_t port, void *priv)
{
    mind_trace("r", port, 0x00, 1);
    mind_trap_queue_access(port, 0x0000);

    return 0x00;
}

static uint16_t
mind_trap_readw(uint16_t port, void *priv)
{
    mind_trace("r", port, 0x0000, 2);
    mind_trap_queue_access(port, 0x0000);

    return 0x0000;
}

static void
mind_trap_write(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);
    mind_trap_queue_access(port, val);
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

    /* Register 9Dh is the stored configuration: bit 5 says one exists, bit 6
       marks it as saved by the setup screen.  The BIOS reads it while it sets
       the display up, and without bit 5 it takes the "no configuration" path,
       which never programs the text-mode control word. */
    m->sys.regs[0x9d] = 0x34;
    m->sys.regs[0x9e] = 0x01;  /* monitor type; MIND_MON overrides it */
}

static void
mind_mcu_ready(void *priv)
{
    mindset.sys_sts = 0x00;
}

/* MIND_IVT=1 watches the interrupt vector table: the guest's own writes are
   passed through, but the first few hundred are logged with the code that
   issued them, which is the only way to find what scribbles over the vectors
   when control later jumps into unpopulated memory. */
static int      mind_ivt_left;
static mem_mapping_t mind_ivt_map;
static void     mind_dump_ram(void);

static uint8_t
mind_ivt_read(uint32_t addr, void *priv)
{
    return ram[addr & 0xfffff];
}

static uint16_t
mind_ivt_readw(uint32_t addr, void *priv)
{
    return (uint16_t) (ram[addr & 0xfffff] | (ram[(addr + 1) & 0xfffff] << 8));
}

static void
mind_ivt_write(uint32_t addr, uint8_t val, void *priv)
{
    if ((val == 0) && ((addr & 0xfffff) < 0x0400)) {
        pclog("[MIND] ivt w1 %04X = %02X @%04X:%04X\n", addr & 0xfffff, val,
              cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
    }

    ram[addr & 0xfffff] = val;
}

static void
mind_ivt_writew(uint32_t addr, uint16_t val, void *priv)
{
    if ((val == 0) && ((addr & 0xfffff) < 0x0400)) {
        uint32_t cs_base = (uint32_t) cpu_state.seg_cs.seg << 4;
        uint16_t at      = (uint16_t) cpu_state.pc;

        /* The pc has usually moved past the store by the time the handler
           runs, so print the bytes just before it: decoding those tells which
           instruction wrote the vector. */
        pclog("[MIND] ivt w2 %04X = %04X @%04X:%04X code:",
              addr & 0xfffff, val, cpu_state.seg_cs.seg, at);
        for (int i = 8; i >= 1; i--)
            pclog(" %02X", mem_readb_phys(cs_base + ((at - i) & 0xffff)));
        pclog("\n");
    }

    ram[addr & 0xfffff] = (uint8_t) val;
    ram[(addr + 1) & 0xfffff] = (uint8_t) (val >> 8);

    /* The BIOS puts the interrupt table back the moment the memory test lets
       go of it, which is the point worth comparing with another emulator. */
    if (((addr & 0xfffff) == 0x00034) && (val == 0x5bff) &&
        (getenv("MIND_RAMDUMP_IVT") != NULL))
        mind_dump_ram();
}

/* The BIOS reads the time of day as three BCD registers - 82h hours, 83h
   minutes, 84h seconds - and the date as 86h month, 87h day, 88h year. */
static uint8_t
mind_bcd(int v)
{
    return (uint8_t) (((v / 10) << 4) | (v % 10));
}

static int
mind_unbcd(uint8_t v)
{
    return ((v >> 4) * 10) + (v & 0x0f);
}

/* --- the system controller running its own firmware --------------------- */

static void
mind_mcu_p1_out(mcs48 *c, uint8_t v)
{
    if (mindset.log && ((v & 0x41) != 0x41))
        pclog("[MIND] mcu P1 = %02X\n", v);
}

static void
mind_mcu_p2_out(mcs48 *c, uint8_t v)
{
    static int last_bit7 = 1;
    int bit7 = (v & 0x80) ? 1 : 0;

    /* P2 bit 7 is the controller's line to the main CPU's INT3. */
    if (bit7 != last_bit7) {
        last_bit7 = bit7;

        if (mind_pcb != NULL)
            mind_icu_int(3, !bit7);
    }
}

static uint8_t
mind_mcu_bus_in(mcs48 *c)
{
    return 0xff;
}

static void
mind_mcu_bus_out(mcs48 *c, uint8_t v)
{
    if (mindset.log)
        pclog("[MIND] mcu BUS out = %02X\n", v);
}

static int
mind_mcu_t0_in(mcs48 *c)
{
    return 0;
}

static int
mind_mcu_t1_in(mcs48 *c)
{
    /* The controller watches the floppy interrupt through T1. */
    return mindset.fdc_irq ? 0 : 1;
}

static void
mind_mcu_fw_tick(void *priv)
{
    static int in_tick = 0;

    if (!in_tick) {
        in_tick = 1;
        mcs48_run(mindset.mcu, 480);   /* ~7.16 MHz / 15 cycles per instruction */
        in_tick = 0;
    }

    timer_set_delay_u64(&mindset.mcu_fw_timer, 1000ULL * TIMER_USEC);
}

/* 18.2 Hz: the board's time base, one pulse per 54945 us like the PC's.  The
   BIOS's interrupt handler turns it into the tick counter behind INT 1Ah. */
static void
mind_tick_pulse(void *priv)
{
    mindset.tick_pending = 1;

    if (mind_pcb != NULL)
        mind_icu_int(3, 1);

    timer_set_delay_u64(&mindset.tick_timer, 54945ULL * TIMER_USEC);
}

static void
mind_rtc_tick(void *priv)
{
    uint8_t *r = mindset.sys.regs;
    int sec = mind_unbcd(r[0x84]);
    int min = mind_unbcd(r[0x83]);
    int hr  = mind_unbcd(r[0x82]);

    if (++sec >= 60) {
        sec = 0;

        if (++min >= 60) {
            min = 0;

            if (++hr >= 24)
                hr = 0;
        }
    }

    r[0x82] = mind_bcd(hr);
    r[0x83] = mind_bcd(min);
    r[0x84] = mind_bcd(sec);

    timer_set_delay_u64(&mindset.rtc_timer, (uint64_t) TIMER_USEC * 1000000u);
}

/* Both status bits go up while the controller works on the byte it was just
   handed, and drop when it is done. */
static void
mind_mcu_busy(void)
{
    /* The controller answers at once: holding the busy bits up makes the
       BIOS's timed waits fall through to their failure path and retry, which
       walks the stack down through the vector table. */
    mindset.sys_sts = 0x00;
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

    /* Bit 6 is the board's other event source: the 18.2 Hz tick. */
    if (mindset.tick_pending)
        ret |= 0x40;

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
                    mind_icu_int(3, 0);
            }
            return;

        case 0x41:
            if (mindset.tick_pending) {
                mindset.tick_pending = 0;

                if (mind_pcb != NULL)
                    mind_icu_int(3, 0);
            }
            return;

        default:
            break;
    }

    /* The BIOS asks for the clock in registers 82h-88h; the return address on
       the guest stack is the caller, which is what says whether DOS reached
       this through its own service or through the BIOS's. */
    if (mindset.log && (val >= 0x82) && (val <= 0x88)) {
        uint32_t stk = ((uint32_t) cpu_state.seg_ss.seg << 4) + (uint16_t) cpu_state.regs[4].w;
        uint16_t ret_cs = 0, ret_ip = 0;
        int k;

        /* The service routine pushes its own working registers first, so the
           real return address is a few frames down: take the first pair on the
           stack whose segment is the ROM's. */
        for (k = 0; k < 12; k++) {
            if (mem_readw_phys(stk + (k * 2) + 2) == 0xf800) {
                ret_ip = mem_readw_phys(stk + (k * 2));
                ret_cs = 0xf800;
                break;
            }
        }

        pclog("[MIND] mcu reg %02X asked by %04X:%04X (caller %04X:%04X)\n",
              val, cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc, ret_cs, ret_ip);
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
            mind_icu_int(3, 0);

        /* Anything typed while this byte was waiting goes out now. */
        if (mindset.kbd.qcount > 0) {
            uint8_t next = mindset.kbd.queue[mindset.kbd.qhead];

            mindset.kbd.qhead = (mindset.kbd.qhead + 1) % (int) sizeof(mindset.kbd.queue);
            mindset.kbd.qcount--;
            mind_keyboard_deliver(next);
        }
    } else {
        ret = mindset.sys.regs[mindset.sys.idx];

        /* The completion bit is a latch the BIOS expects to find set once per
           controller interrupt, so reading it clears it. */
        if (mindset.sys.idx == 0x02)
            mindset.sys.regs[0x02] &= (uint8_t) ~0x01;
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
/* 8040h hands the queued module bus pairs back one word at a time; when the
   queue runs dry the transfer is over and the module interrupt goes up. */
static uint16_t
mind_trap_dma_read(UNUSED(uint16_t port), void *priv)
{
    uint16_t word = mind_trap_pop();

    if (mind_trap_count == 0) {
        mind_trap_int = 1;

        if (mind_pcb != NULL)
            mind_icu_int(0, 1);

        mindset.sys.regs[0x02] |= 0x01;
    }

    return word;
}

static uint16_t
mind_trap_clear_int(UNUSED(uint16_t port), void *priv)
{
    mind_trap_int = 0;
    mindset.sys.regs[0x02] &= (uint8_t) ~0x01;

    if (mind_pcb != NULL)
        mind_icu_int(0, 0);

    return 0x0000;
}

/* 8340 is written from the MCU event handler with whatever register 0 holds;
   nothing reads it back, so latching the value is all the board does. */
static uint8_t mind_latch;

static void
mind_latch_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    mind_latch = val;

    if (mindset.log)
        pclog("[MIND] 8340 <- %02X\n", val);
}

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
    /* Control keys keep their release codes: the BIOS tracks Alt and Reset by
       them. */
    { 3000000, 0x39 },  /* space: leave the diagnostics screen */
    {  300000, 0xb9 },
    { 6000000, 0x38 },  /* Alt down ... */
    {   80000, 0x57 },  /* ... Reset, which saves and leaves the config screen */
    {  300000, 0xd7 },
    {  200000, 0xb8 },
    { 5000000, 0x39 },  /* answer the POST that follows */
    {  300000, 0xb9 },
    /* Typed characters carry NO release code: the keyboard MCU is what strips
       those, and here the register goes straight to the BIOS, so a release
       code would be handed to DOS as a character of its own. */
    {16000000, 0x0b },  /* 0 */
    { 250000, 0x02 },   /* 1 */
    { 250000, 0x0c },   /* - */
    { 250000, 0x0b },   /* 0 */
    { 250000, 0x02 },   /* 1 */
    { 250000, 0x0c },   /* - */
    { 250000, 0x09 },   /* 8 */
    { 250000, 0x06 },   /* 5 */
    { 800000, 0x1c },   /* Return */
    { 8000000, 0x1c },  /* the time prompt takes the default */
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
mind_fifo_reset(mind_fdc_fifo_t *q)
{
    q->head  = 0;
    q->tail  = 0;
    q->count = 0;
}

static void
mind_fifo_push(mind_fdc_fifo_t *q, uint8_t val)
{
    if (q->count >= MIND_FIFO_SIZE)
        return;

    q->buf[q->tail] = val;
    q->tail = (q->tail + 1) % MIND_FIFO_SIZE;
    q->count++;
}

static int
mind_fifo_pop(mind_fdc_fifo_t *q)
{
    int val;

    if (q->count == 0)
        return -1;

    val = q->buf[q->head];
    q->head = (q->head + 1) % MIND_FIFO_SIZE;
    q->count--;

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

    /* The board counts the bytes it still has to move and only raises terminal
       count on the access after the counter has run out, so the controller
       sees TC exactly one byte past the last byte of the transfer. */
    if (mindset.datafifo.left == 0)
        return DMA_OVER;

    mindset.datafifo.left--;

    return 0;
}

static int
mind_fdc_dma_read(void *priv)
{
    int val;

    mind_fdc_transfer();

    /* Only sector bytes travel through the queue: the BIOS hands the command
       bytes over the command register, which reaches the controller directly. */
    val = mind_fifo_pop(&mindset.datafifo);

    return (val < 0) ? DMA_NODATA : val;
}

static int
mind_fdc_dma_write(void *priv, uint8_t val)
{
    mind_fifo_push(&mindset.datafifo, val);

    return mind_fdc_transfer();
}

static void
mind_fdc_drq(void *priv, int set)
{
    /* In port mode the controller parks a byte and asks for a transfer; the
       board's channel 1 then reads it from 8054h, which is what makes the
       transfer happen without any pushing from us. */
    if (mindset.fdc_portdma && set && (mind_pcb != NULL)) {
        if (i8018x_pcb_dma_request(mind_pcb, 1) && (mindset.fdc != NULL))
            fdc_set_tc(mindset.fdc);
    }

    if (mindset.log)
        pclog("[MIND] fdc drq %d\n", set);
}

/* The BIOS's floppy interrupt handler asks the system MCU about register 2 and
   reads bit 0 as "the controller finished"; without it every interrupt is
   taken for a failure and the boot gives up. */
static void
mind_fdc_completion(void)
{
    mindset.sys.regs[0x02] |= 0x01;
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
        mind_fdc_completion();
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
mind_dump_ram(void)
{
    FILE *f = fopen("/tmp/mind-ram.bin", "wb");

    if (f == NULL)
        return;

    for (uint32_t a = 0; a < 0x10000; a++)
        fputc(mem_readb_phys(a), f);

    fclose(f);
    pclog("[MIND] low 64 KB of memory dumped\n");
}

static void
mind_dump_timed(void *priv)
{
    mind_dump_vram("sec");
    mind_dump_ram();
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
        mind_icu_int(0, 1);
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
            mind_icu_int(0, 0);

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

    if (mindset.log)
        pclog("[MIND] F R %02X\n", ret);

    mind_trace("r", port, ret, 1);

    return ret;
}

static void
mind_fdc_data_write(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);

    if (mindset.log)
        pclog("[MIND] F W %02X\n", val);

    fdc_write(0x03f5, val, mindset.fdc);
}

static uint8_t
mind_fdc_dma_read_port(uint16_t port, void *priv)
{
    int val;

    if (mindset.fdc_portdma)
        val = fdc_port_data_read(mindset.fdc);
    else
        val = mind_fifo_pop(&mindset.datafifo);

    if ((val < 0) && (mindset.datafifo.left != 0))
        val = 0xff;

    mind_trace("r", port, (val < 0) ? 0xff : val, 1);

    return (uint8_t) ((val < 0) ? 0xff : val);
}

static void
mind_fdc_dma_write_port(uint16_t port, uint8_t val, void *priv)
{
    mind_trace("w", port, val, 1);

    if (mindset.fdc_portdma)
        fdc_port_data_write(mindset.fdc, val);
    else
        mind_fifo_push(&mindset.datafifo, val);
}

static void
mind_fdc_count_write(uint16_t port, uint16_t val, void *priv)
{
    mind_trace("w", port, val, 2);

    mindset.datafifo.left = val;
}

static uint8_t
mind_fdc_ack_read(uint16_t port, void *priv)
{
    mind_trace("r", port, 0x00, 1);

    if (mindset.fdc_irq) {
        mindset.fdc_irq = 0;

        if (mind_pcb != NULL)
            mind_icu_int(0, 0);
    }

    return 0x00;
}

/* MIND_FASTTRACE keeps the last few dozen program counters at a hundred
   microseconds a sample, so the instruction that leaves the ROM can be seen
   even though the ordinary sampler only looks every twenty milliseconds. */
#define MIND_RING 96

static struct {
    uint16_t rseg;
    uint16_t roff;
    uint16_t rsp;
} mind_ring[MIND_RING];

static int mind_stack_low_reported;

static int mind_ring_pos;
static int mind_ring_on;
static int mind_boot_reported;
static uint16_t mind_last_cs;

static void
mind_fast_sample(void *priv)
{
    mind_ring[mind_ring_pos].rseg = cpu_state.seg_cs.seg;
    mind_ring[mind_ring_pos].roff = (uint16_t) cpu_state.pc;
    mind_ring[mind_ring_pos].rsp  = (uint16_t) cpu_state.regs[4].w;
    mind_ring_pos = (mind_ring_pos + 1) % MIND_RING;

    /* The stack is only a few kilobytes; once it drops into the interrupt
       vector table something is pushing in a loop. */
    if (mind_ring_on && !mind_stack_low_reported &&
        ((uint16_t) cpu_state.regs[4].w < 0x0400)) {
        mind_stack_low_reported = 1;

        pclog("[MIND] stack collapsed: SP=%04X at %04X:%04X\n",
              (uint16_t) cpu_state.regs[4].w, cpu_state.seg_cs.seg,
              (uint16_t) cpu_state.pc);

        for (int i = 0; i < MIND_RING; i++) {
            int k = (mind_ring_pos + i) % MIND_RING;

            pclog("[MIND]   stk[%02d] = %04X:%04X sp=%04X\n", i,
                  mind_ring[k].rseg, mind_ring[k].roff, mind_ring[k].rsp);
        }
    }

    /* The ROM keeps the time and date services around F800:92C0; the ring
       holds the instructions that led there, which is the only way to see the
       caller without a debugger. */
    if (mind_ring_on && (cpu_state.seg_cs.seg == 0xf800) &&
        (cpu_state.pc >= 0x92bc) && (cpu_state.pc <= 0x92e8)) {
        static int clock_dumped = 0;

        if (!clock_dumped) {
            int i;

            clock_dumped = 1;
            pclog("[MIND] clock service reached, ring before it:\n");

            for (i = 0; i < MIND_RING; i++) {
                int k = (mind_ring_pos + i) % MIND_RING;

                pclog("[MIND]   bring[%02d] = %04X:%04X\n", i, mind_ring[k].rseg, mind_ring[k].roff);
            }
        }
    }

    /* A divide error vectors to F800:03DE; the ring then shows the division
       that faulted, which is the only way to see it without a debugger. */
    if ((cpu_state.seg_cs.seg != 0xf800) || (cpu_state.pc < 0x03d0) || (cpu_state.pc > 0x0410))
        ;
    else if (mindset.log && (mind_prev_cs != 0)) {
        pclog("[MIND] #DE entered from %04X:%04X\n", mind_prev_cs, mind_prev_ip);
        mind_prev_cs = 0;
    }

    mind_prev_cs = cpu_state.seg_cs.seg;
    mind_prev_ip = (uint16_t) cpu_state.pc;

    if (mind_ring_on && (cpu_state.seg_cs.seg == 0xf800) &&
        (cpu_state.pc >= 0x03de) && (cpu_state.pc <= 0x0400)) {
        static int de_dumped = 0;
        int outside = 0, i;

        /* Only the fault that comes from DOS code is interesting; the POST
           takes this vector too, early on, with nothing but BIOS frames on the
           ring. */
        for (i = 0; i < MIND_RING; i++) {
            int k = (mind_ring_pos + i) % MIND_RING;
            uint16_t seg = mind_ring[k].rseg;

            if ((seg != 0xf800) && (seg != 0x0000))
                outside = 1;
        }

        if (!de_dumped && outside) {
            de_dumped = 1;
            pclog("[MIND] divide error handler entered, ring before it:\n");

            for (i = MIND_RING - 10; i < MIND_RING; i++) {
                int k = (mind_ring_pos + i) % MIND_RING;

                pclog("[MIND]   dring[%02d] = %04X:%04X\n", i, mind_ring[k].rseg, mind_ring[k].roff);
            }

            /* The newest frame is the code that divided; keep its bytes. */
            {
                int      k = (mind_ring_pos + MIND_RING - 1) % MIND_RING;
                uint32_t at = ((uint32_t) mind_ring[k].rseg << 4) + mind_ring[k].roff;
                FILE    *ff = fopen("/tmp/mind-fault.bin", "wb");

                if (ff != NULL) {
                    uint32_t a;

                    for (a = at - 16; a < at + 48; a++)
                        fputc(mem_readb_phys(a), ff);

                    fclose(ff);
                    pclog("[MIND] fault bytes at %04X:%04X saved\n",
                          mind_ring[k].rseg, mind_ring[k].roff);
                }
            }
        }
    }

    /* The boot sector hands over to the second stage at 0060:0000, which the
       board's DMA read into 0600h; keep that image for comparison. */
    if (mind_ring_on && (cpu_state.seg_cs.seg == 0x0060)) {
        static int stage2_dumped = 0;

        if (!stage2_dumped) {
            FILE *f = fopen("/tmp/mind-stage2.bin", "wb");

            stage2_dumped = 1;

            if (f != NULL) {
                for (uint32_t a = 0x600; a < 0x1200; a++)
                    fputc(mem_readb_phys(a), f);

                fclose(f);
                pclog("[MIND] second stage dumped at 0060:%04X\n", (uint16_t) cpu_state.pc);
            }
        }
    }

    /* Once the boot sector is running, the interesting moment is control
       leaving it again: that is where the boot comes apart. */
    if (mind_ring_on && mindset.jump_reported && !mind_boot_reported &&
        (cpu_state.seg_cs.seg != 0xf800) && (cpu_state.seg_cs.seg != 0x0000)) {
        mind_boot_reported = 1;

        pclog("[MIND] left the boot sector: CS:IP=%04X:%04X AX=%04X BX=%04X CX=%04X DX=%04X\n",
              cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc,
              (uint16_t) cpu_state.regs[0].w, (uint16_t) cpu_state.regs[3].w,
              (uint16_t) cpu_state.regs[1].w, (uint16_t) cpu_state.regs[2].w);
        pclog("[MIND]   DS=%04X ES=%04X SS=%04X SP=%04X SI=%04X DI=%04X BP=%04X\n",
              cpu_state.seg_ds.seg, cpu_state.seg_es.seg, cpu_state.seg_ss.seg,
              (uint16_t) cpu_state.regs[4].w, (uint16_t) cpu_state.regs[6].w,
              (uint16_t) cpu_state.regs[7].w, (uint16_t) cpu_state.regs[5].w);

        for (int i = 0; i < MIND_RING; i++) {
            int k = (mind_ring_pos + i) % MIND_RING;

            pclog("[MIND]   bring[%02d] = %04X:%04X\n", i, mind_ring[k].rseg, mind_ring[k].roff);
        }

        /* Keep the interrupt table too: which vectors the BIOS installs is
           what tells whether a DOS service exists at all. */
        {
            FILE *iv = fopen("/tmp/mind-ivt.bin", "wb");

            if (iv != NULL) {
                for (uint32_t a = 0; a < 0x400; a++)
                    fputc(mem_readb_phys(a), iv);

                fclose(iv);
            }
        }

        /* The BIOS keeps the time of day counter at 40h:6Ch; dump the block. */
        {
            FILE *bd = fopen("/tmp/mind-bda.bin", "wb");

            if (bd != NULL) {
                for (uint32_t a = 0x400; a < 0x500; a++)
                    fputc(mem_readb_phys(a), bd);

                fclose(bd);
            }
        }

        /* The first FAT sector is read into 8000h; keep it for comparison. */
        {
            FILE *f = fopen("/tmp/mind-fat.bin", "wb");

            if (f != NULL) {
                for (uint32_t a = 0x8000; a < 0x8200; a++)
                    fputc(mem_readb_phys(a), f);

                fclose(f);
            }
        }
    }

    mind_last_cs = cpu_state.seg_cs.seg;

    /* Catch the moment control leaves the ROM here, while the ring still
       holds the samples from just before it. */
    if (!mindset.jump_reported && (cpu_state.seg_cs.seg != 0xf800)) {
        mindset.jump_reported = 1;

        for (int i = 0; i < MIND_RING; i++) {
            int k = (mind_ring_pos + i) % MIND_RING;

            pclog("[MIND]   ring[%02d] = %04X:%04X\n", i, mind_ring[k].rseg, mind_ring[k].roff);
        }

        /* Control has just left the ROM: whatever sits at 7C00h is the sector
           the guest is about to run. */
        {
            FILE *f = fopen("/tmp/mind-boot.bin", "wb");

            if (f != NULL) {
                for (uint32_t a = 0x7c00; a < 0x7e00; a++)
                    fputc(mem_readb_phys(a), f);

                fclose(f);
            }
        }
    }

    timer_set_delay_u64(&mindset.fast_timer, 100 * TIMER_USEC);
}

/* Sampler: without a serial console the only way to tell "the CPU is still
   running the ROM" from "it wedged in a loop" is where it is over time. */
static void
mind_sample(void *priv)
{
    if (mindset.log)
        pclog("[MIND] flags [0x12]=%02X [0x15]=%04X [0x1e]=%04X eq=%04X int0D=%04X:%04X\n",
              mem_readb_phys(0x412), mem_readw_phys(0x415), mem_readw_phys(0x41e),
              mem_readw_phys(0x410), mem_readw_phys(0x36), mem_readw_phys(0x34));
    if (mindset.log)
        pclog("[MIND] mode=%02X cols=%04X disp=%04X cur=%04X\n",
              mem_readb_phys(0x449), mem_readw_phys(0x44a),
              mem_readw_phys(0x462), mem_readw_phys(0x450));

    pclog("[MIND] pc=%04X:%04X frames=%d\n", cpu_state.seg_cs.seg,
          (uint16_t) cpu_state.pc, frames);

    /* MIND_RAMDUMP=<frame>: drop the low 64 KB of memory so the same frame can
       be compared byte for byte with another emulator's view of it. */
    {
        static const char *ramdump_at   = NULL;
        static int         ramdump_done = 0;
        static int         ram_samples  = 0;

        if (ramdump_at == NULL)
            ramdump_at = getenv("MIND_RAMDUMP");

        ram_samples++;

        if ((ramdump_at != NULL) && !ramdump_done && (ram_samples == atoi(ramdump_at))) {
            FILE *f = fopen("/tmp/mind-ram.bin", "wb");

            ramdump_done = 1;

            if (f != NULL) {
                for (uint32_t a = 0; a < 0x10000; a++)
                    fputc(mem_readb_phys(a), f);

                fclose(f);
                pclog("[MIND] low 64 KB of memory dumped at frame %d\n", frames);
            }
        }
    }

    /* If control ever leaves the ROM, the stack frame that got it there says
       who jumped: dump it once, then stay quiet. */
    if ((cpu_state.seg_cs.seg != 0xf800) && !mindset.jump_reported) {
        uint32_t stack_top_base = ((uint32_t) cpu_state.seg_ss.seg << 4) + cpu_state.regs[4].w;

        mindset.jump_reported = 1;

        if (mind_ring_on)
            for (int i = 0; i < MIND_RING; i++) {
                int k = (mind_ring_pos + i) % MIND_RING;

                pclog("[MIND]   ring[%02d] = %04X:%04X\n", i, mind_ring[k].rseg, mind_ring[k].roff);
            }

        pclog("[MIND] left the ROM: CS:IP=%04X:%04X SS:SP=%04X:%04X\n",
              cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc,
              cpu_state.seg_ss.seg, cpu_state.regs[4].w);

        for (int i = 0; i < 8; i++)
            pclog("[MIND]   stack[%04X] = %04X\n", (uint16_t) (cpu_state.regs[4].w + i * 2),
                  mem_readw_phys(stack_top_base + i * 2));
    }

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

    if (mindset.log) {
        static int tick_secs = 0;

        if (++tick_secs >= 10) {
            uint16_t lo = mem_readw_phys(0x46c);
            uint16_t hi = mem_readw_phys(0x46e);

            tick_secs = 0;
            pclog("[MIND] tick = %u (hi %04X) @ %04X:%04X\n", lo, hi,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
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

    if (getenv("MIND_IVT") != NULL) {
        mind_ivt_left = atoi(getenv("MIND_IVT"));
        /* Hand the first page to the probe; the handlers put the bytes back
           into RAM themselves.  Mappings work on 4 KB pages, so the probe sees
           the whole first page even though only the vectors matter here. */
        mem_set_mem_state_both(0x00000, 0x01000,
                               MEM_READ_EXTANY | MEM_WRITE_EXTANY);
        mem_mapping_add(&mind_ivt_map, 0x00000, 0x01000,
                        mind_ivt_read, mind_ivt_readw, NULL,
                        mind_ivt_write, mind_ivt_writew, NULL,
                        NULL, MEM_MAPPING_EXTERNAL, NULL);
    }

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
    mind_fifo_reset(&mindset.cmdfifo);
    mind_fifo_reset(&mindset.datafifo);
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

    /* MIND_FDC=port: let the controller own the data register and the board's
       DMA read the byte from 8054h, the way the hardware does it. */
    if ((getenv("MIND_FDC") != NULL) && (strcmp(getenv("MIND_FDC"), "port") == 0)) {
        mindset.fdc->flags |= FDC_FLAG_PORTDMA;
        mindset.fdc_portdma = 1;
        pclog("[MIND] floppy: data register in port mode\n");
    }
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
    io_sethandler(0x8040, 0x0002,
                  NULL, mind_trap_dma_read, NULL,
                  NULL, NULL, NULL, NULL);
    io_sethandler(0x8048, 0x0002,
                  NULL, mind_trap_clear_int, NULL,
                  NULL, NULL, NULL, NULL);
    io_sethandler(0x804a, 0x0006,
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
    /* 8340: one write-only latch, fed with the value of MCU register 0. */
    io_sethandler(0x8340, 0x0001,
                  NULL, NULL, NULL,
                  mind_latch_write, NULL, NULL, NULL);

    timer_add(&mindset.mcu_timer, mind_mcu_ready, NULL, 0);
    timer_add(&mindset.rtc_timer, mind_rtc_tick, NULL, 0);
    /* Run the system controller's own firmware, the way the board does. */
    mindset.mcu_fw = (getenv("MIND_MCU") != NULL) && (strcmp(getenv("MIND_MCU"), "fw") == 0);

    if (mindset.mcu_fw) {
        FILE *f = rom_fopen("roms/machines/mindset/mindset_u17_253002.bin", "rb");

        if ((f != NULL) && (mindset.mcu = mcs48_create()) != NULL) {
            mcs48_hooks *hk = mcs48_hooks_of(mindset.mcu);
            uint8_t     *rom = mcs48_rom(mindset.mcu);
            size_t       n;

            n = fread(rom, 1, 4096, f);
            fclose(f);

            mcs48_set_rom_mask(mindset.mcu, (n == 2048) ? 0x7ff : 0xfff);

            hk->p1_out  = mind_mcu_p1_out;
            hk->p2_out  = mind_mcu_p2_out;
            hk->bus_in  = mind_mcu_bus_in;
            hk->bus_out = mind_mcu_bus_out;
            hk->t0_in   = mind_mcu_t0_in;
            hk->t1_in   = mind_mcu_t1_in;

            mcs48_reset(mindset.mcu);

            /* Seed the shared register file with the values the HLE would
               return, then let the firmware keep them up to date. */
            memcpy(mcs48_ext(mindset.mcu), mindset.sys.regs, 256);

            timer_add(&mindset.mcu_fw_timer, mind_mcu_fw_tick, NULL, 0);
            timer_set_delay_u64(&mindset.mcu_fw_timer, 1000ULL * TIMER_USEC);

            pclog("[MIND] system controller firmware loaded (%zu bytes)\n", n);
        } else {
            pclog("[MIND] system controller firmware missing, staying on the HLE\n");
            mindset.mcu_fw = 0;
        }
    }

    /* The board's 18.2 Hz tick, still being worked out: the BIOS's handler for
       that event goes somewhere we have not identified, so it stays off. */
    if (getenv("MIND_TICK") != NULL) {
        timer_add(&mindset.tick_timer, mind_tick_pulse, NULL, 0);
        timer_set_delay_u64(&mindset.tick_timer, 54945ULL * TIMER_USEC);
    }
    timer_set_delay_u64(&mindset.rtc_timer, (uint64_t) TIMER_USEC * 1000000u);
    timer_add(&mind_key_timer, mind_key_next, NULL, 0);

    if (mindset.keytest) {
        static pc_timer_t key_timer;

        timer_add(&key_timer, mind_key_test, NULL, 0);
        timer_set_delay_u64(&key_timer, 6ULL * 1000000ULL * TIMER_USEC);
    }
    timer_add(&mindset.sample_timer, mind_sample, NULL, 0);
    timer_add(&mindset.fast_timer, mind_fast_sample, NULL, 0);

    if (getenv("MIND_FASTTRACE") != NULL) {
        mind_ring_on = 1;
        timer_set_delay_u64(&mindset.fast_timer, 100 * TIMER_USEC);
    }
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
