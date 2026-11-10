/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Philips :YES (1985): 80186 at 16 MHz, 128/256/640 KB of RAM,
 *          a 64 KB BIOS at 0xf0000, an HD6845S text display whose frame
 *          buffer the ROM places at the top of RAM through the 80186's
 *          MMCS register (0xffa6), a WD2797 floppy controller, an i8256
 *          MUART for serial/parallel and a SASI port for the hard disk.
 *
 *          Step 1: CPU, RAM, ROM and the I/O decode, with an optional
 *          trace (YES_LOG=1) of every port the ROM touches.  The display,
 *          keyboard and floppy are not modelled yet; this build exists to
 *          find out what the ROM expects before any of that is written.
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
#include <86box/machine.h>
#include <86box/mem.h>
#include <86box/io.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/plat.h>
#include <86box/timer.h>
#include <86box/i8018x.h>
#include <86box/wd2791.h>
#include <86box/intel_8256.h>
#include <86box/yes_video.h>

#include "../cpu/cpu.h"

extern int fdcinited;

/* The two BIOS halves are interleaved: 0x23494 carries the even bytes and
   0x23484 the odd ones (MAME ROMX_LOAD with ROM_SKIP(1)). */
#define YES_BIOS_EVEN "roms/machines/yes/yes_23494.bin"
#define YES_BIOS_ODD  "roms/machines/yes/yes_23484.bin"
#define YES_BIOS_ADDR 0x000f0000
#define YES_BIOS_SIZE 0x00010000

#define YES_MUART_BASE 0x0400
#define YES_MUART_SIZE 0x0020
#define YES_FDC_BASE   0x0480
#define YES_FDC_SIZE   0x0008
#define YES_CRTC_ADDR  0x0500
#define YES_CRTC_DATA  0x0502
#define YES_DSKSEL     0x0580
#define YES_KBD        0x05a0
#define YES_PALETTE    0x05c0
#define YES_SYSCtl     0x05e0
#define YES_RTC        0x066a
#define YES_BOARD      0x0422
#define YES_SASI_DATA  0x067a
#define YES_SASI_STAT  0x067c
#define YES_MMCS       0xffa6

typedef struct yes_t {
    uint8_t  crtc_idx;
    uint8_t  crtc_reg[32];
    uint8_t  palette[16];
    uint8_t  sysctl;
    uint8_t  dsksel;
    uint16_t mmcs;
    uint32_t vid_base; /* frame buffer base, derived from MMCS */
    wd279x_t *fdc;     /* WD2797 formatter at 0x480 */
    struct i8018x_pcb_t *pcb;
    intel_8256_t *muart; /* serial / parallel / timers / interrupt levels */
    void         *video;
    uint8_t  kbd_code; /* 0x5a0: last scan code from the keyboard */
} yes_t;

static yes_t yes;
static int   yes_log = -1;
static pc_timer_t yes_sample_timer;
static pc_timer_t yes_vram_timer;

/* YES_LOG=1: sample where the CPU is every 20 ms.  A wait loop that touches
   no I/O is invisible to the port trace, and that is exactly what the POST
   sits in after the tick handler has come up. */
/* YES_DUMP=1: the frame buffer is plain RAM (the ROM puts it at the top of
   memory with MMCS), so a snapshot of it shows both the character cells and
   their attributes before the display is modelled. */
static void
yes_dump_vram(void *priv)
{
    const char *path = getenv("YES_DUMP");
    FILE       *f;

    if ((path == NULL) || (ram == NULL))
        return;

    f = fopen(path, "wb");
    if (f != NULL) {
        fwrite(ram + yes.vid_base, 1, 0x00010000, f);
        fclose(f);
        pclog("[YES] vram dumped: %s (base %05X, mmcs %04X)\n", path,
              yes.vid_base, yes.mmcs);
    }

    /* The interrupt vectors say which handlers the ROM installed, and those
       tell which subsystem is supposed to end the wait. */
    f = fopen("/tmp/yes_ram.bin", "wb");
    if (f != NULL) {
        fwrite(ram, 1, 0x00100000, f);
        fclose(f);
    }

    f = fopen("/tmp/yes_low.bin", "wb");
    if (f != NULL) {
        fwrite(ram, 1, 0x00001000, f);
        fclose(f);
    }
}

static void
yes_sample(void *priv)
{
    if (getenv("YES_PC") != NULL) {
        /* Skip the idle wait loop at FCB0:013C-0143 so the samples show the
           code around it. */
        if (!((cpu_state.seg_cs.seg == 0xfcb0) &&
              (cpu_state.pc >= 0x0130) && (cpu_state.pc <= 0x0150)))
            pclog("[YES] pc=%04X:%04X if=%d\n", cpu_state.seg_cs.seg,
                  (uint16_t) cpu_state.pc, (cpu_state.flags & 0x200) ? 1 : 0);
    }

    timer_set_delay_u64(&yes_sample_timer, 2000 * TIMER_USEC);
}

static void
yes_log_write(const char *what, uint16_t port, uint8_t val)
{
    if (yes_log > 0)
        pclog("[YES] w %-9s %03X = %02X\n", what, port, val);
}

static uint8_t
yes_log_read(const char *what, uint16_t port, uint8_t val)
{
    if (yes_log > 0)
        pclog("[YES] r %-9s %03X -> %02X\n", what, port, val);
    return val;
}

static void
yes_muart_irq(void *priv, int level)
{
    static int line = -1;

    if (line < 0) {
        const char *l = getenv("YES_MUARTIRQ");
        line = (l != NULL) ? atoi(l) : 1;
    }

    if (yes_log > 0)
        pclog("[YES] muart IRQ %d -> INT%d\n", level, line);

    i8018x_pcb_icu_int(yes.pcb, line, level);
}

/* The MUART supplies its own vector (40h + priority level) during the
   acknowledge cycle, so the 80186's fixed vector for the pin is not used. */
static int
yes_muart_ack(void *priv)
{
    uint8_t vec = intel_8256_acknowledge(yes.muart);

    if (yes_log > 0)
        pclog("[YES] muart ack -> vector %02X\n", vec);

    return vec;
}

static void
yes_dsksel_write(uint16_t port, uint8_t val, void *priv)
{
    yes.dsksel = val;
    yes_log_write("dsksel", port, val);

    /* YES_STK=1: the floppy reset routine selects a drive a few instructions
       before it resets the formatter, so the stack at this point carries the
       return address of whoever asked for the reset. */
    if (getenv("YES_STK") != NULL) {
        static int n;
        if (n < 8) {
            uint32_t lin = ((uint32_t) cpu_state.seg_ss.seg << 4) +
                           cpu_state.regs[4].w;
            n++;
            pclog("[YES] sel=%02X @%04X:%04X stk:", val,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
            for (int i = 0; i < 6; i++) {
                uint16_t w = 0;
                memcpy(&w, ram + ((lin + (uint32_t) i * 2) & 0xfffff), 2);
                pclog(" %04X", w);
            }
            pclog("\n");
        }
    }

    wd279x_set_dskctl(yes.fdc, val);
}

/* INTRQ from the formatter.  The board feeds it to the 80186's INT2 input:
   vector 0x0e holds a handler that acknowledges through the ICU's EOI
   register (0xff22 = 0x0e) and then reads the formatter's status at 0x480
   (ROM 0xfd052-0xfd05d), which is exactly the line the boot wait is waiting
   for. */
static void
yes_fdc_irq(void *priv, int level)
{
    if (yes_log > 0)
        pclog("[YES] fdc IRQ %d -> INT2\n", level);

    i8018x_pcb_icu_int(yes.pcb, 2, level);
}

static uint8_t
yes_board_read(uint16_t port, void *priv)
{
    /* Read once during the keyboard setup; zero makes the ROM raise MODE bit 5. */
    const char *v = getenv("YES_BOARD");
    uint8_t r = 0x00;

    if (v != NULL)
        r = (uint8_t) strtoul(v, NULL, 16);

    if (yes_log > 0)
        pclog("[YES] rd board -> %02X\n", r);
    return r;
}

static uint8_t
yes_kbd_read(uint16_t port, void *priv)
{
    {
        static int n;
        if (n < 12)
            pclog("[KBD] read 5a0 -> %02X @%04X:%04X\n", yes.kbd_code,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc), n++;
    }
    return yes_log_read("kbd", port, yes.kbd_code);
}

/*
 * YES_KEY=1c (hex) & YES_KEYIRQ=0|1: press one key a moment after reset, so
 * the boot menu (if that is what the ROM is waiting in) can be answered
 * before the display exists.  IRQ1 is the keyboard's own line; which of the
 * 80186's INT0-INT3 inputs carries it on this board is what the experiment
 * is for, hence the selectable line.
 */
static pc_timer_t yes_key_timer;
static int        yes_key_phase;

static void
yes_key_tick(void *priv)
{
    /* The keyboard is one of the MUART's interrupt sources: level 1, which the
       part answers with vector 0x41 -- the handler the ROM installs for the
       keyboard.  The scancode itself is read back from 0x5a0. */
    /* The ISR latches the make code and only queues it when the matching
       break code arrives as a second interrupt, so send both. */
    if (yes_key_phase == 0) {
        yes_key_phase = 1;
        pclog("[YES] key %02X -> MUART level 1\n", yes.kbd_code);
        intel_8256_set_level(yes.muart, 1);
        timer_set_delay_u64(&yes_key_timer, 20000 * TIMER_USEC);
    } else if (yes_key_phase == 1) {
        yes_key_phase = 2;
        yes.kbd_code |= 0x80;
        pclog("[YES] break %02X -> MUART level 1\n", yes.kbd_code);
        intel_8256_set_level(yes.muart, 1);
        timer_set_delay_u64(&yes_key_timer, 20000 * TIMER_USEC);
    } else if (yes_key_phase == 2) {
        yes_key_phase = 3;
        yes.kbd_code &= 0x7f;
        pclog("[YES] key again %02X -> MUART level 1\n", yes.kbd_code);
        intel_8256_set_level(yes.muart, 1);
    }
}

static void
yes_sysctl_write(uint16_t port, uint8_t val, void *priv)
{
    yes.sysctl = val;
    yes_log_write("sysctl", port, val);
}

static uint8_t
yes_sysctl_read(uint16_t port, void *priv)
{
    return yes_log_read("sysctl", port, yes.sysctl);
}

static void
yes_rtc_write(uint16_t port, uint8_t val, void *priv)
{
    yes_log_write("rtc", port, val);
}

static uint8_t
yes_rtc_read(uint16_t port, void *priv)
{
    return yes_log_read("rtc", port, 0x00);
}

static void
yes_sasi_write(uint16_t port, uint8_t val, void *priv)
{
    yes_log_write("sasi", port, val);
}

static uint8_t
yes_sasi_read(uint16_t port, void *priv)
{
    /* No SASI target is fitted: the status port reads idle instead of an open
       bus, so the ROM's poll sees "nothing to do" rather than "busy for ever"
       (0xff has bit 3 set, which is the bit the boot code waits to clear). */
    {
        const char *v = getenv("YES_SASI");
        uint8_t     idle = (v != NULL) ? (uint8_t) strtoul(v, NULL, 16) : 0x00;

        return yes_log_read("sasi", port, idle);
    }
}

/*
 * 80186 MMCS (mid-range memory chip select).  The BIOS writes the frame
 * buffer's base here: 0x01f8 -> 0x10000, 0x21f8 -> 0x30000, 0x81f8 ->
 * 0x90000 for 128/256/640 KB machines.
 */
static void
yes_mmcs_write16(uint16_t port, uint16_t val, void *priv)
{
    yes.mmcs = val;
    yes_video_set_base(yes.video, (uint32_t) (((val >> 11) & 0x1f) << 15) + 0x10000u);
    pclog("[YES] MMCS = %04X -> vid_base %05X\n", val,
              (uint32_t) (((val >> 11) & 0x1f) << 15) + 0x10000);
}

static void
yes_mmcs_write8(uint16_t port, uint8_t val, void *priv)
{
    if (port == YES_MMCS)
        yes.mmcs = (yes.mmcs & 0xff00) | val;
    else
        yes.mmcs = (yes.mmcs & 0x00ff) | (val << 8);

    yes_video_set_base(yes.video, (uint32_t) (((yes.mmcs >> 11) & 0x1f) << 15) + 0x10000u);

    if (yes_log > 0)
        pclog("[YES] w MMCS8     %03X = %02X  -> %04X\n", port, val, yes.mmcs);
}

static uint16_t
yes_mmcs_read16(uint16_t port, void *priv)
{
    return yes.mmcs;
}

static uint8_t
yes_mmcs_read8(uint16_t port, void *priv)
{
    return (port == YES_MMCS) ? (yes.mmcs & 0xff) : (yes.mmcs >> 8);
}

static void
yes_io_init(void)
{
    io_sethandler(YES_DSKSEL, 0x0001, NULL, NULL, NULL, yes_dsksel_write, NULL,
                  NULL, NULL);
    io_sethandler(YES_BOARD, 0x0001, yes_board_read, NULL, NULL, NULL, NULL, NULL,
                  NULL);
    io_sethandler(YES_KBD, 0x0001, yes_kbd_read, NULL, NULL, NULL, NULL, NULL,
                  NULL);
    io_sethandler(YES_SYSCtl, 0x0001, yes_sysctl_read, NULL, NULL,
                  yes_sysctl_write, NULL, NULL, NULL);
    io_sethandler(YES_RTC, 0x0001, yes_rtc_read, NULL, NULL, yes_rtc_write, NULL,
                  NULL, NULL);
    io_sethandler(YES_SASI_DATA, 0x0001, yes_sasi_read, NULL, NULL,
                  yes_sasi_write, NULL, NULL, NULL);
    io_sethandler(YES_SASI_STAT, 0x0001, yes_sasi_read, NULL, NULL,
                  yes_sasi_write, NULL, NULL, NULL);
    io_sethandler(YES_MMCS, 0x0002, yes_mmcs_read8, yes_mmcs_read16, NULL,
                  yes_mmcs_write8, yes_mmcs_write16, NULL, NULL);
}

int
machine_yes_init(const machine_t *model)
{
    int ret;

    if (yes_log < 0)
        yes_log = (getenv("YES_LOG") != NULL);

    pclog("[YES] machine_yes_init entered\n");

    ret = bios_load_interleaved(YES_BIOS_EVEN, YES_BIOS_ODD, YES_BIOS_ADDR,
                                YES_BIOS_SIZE, 0);

    pclog("[YES] bios_load -> %d\n", ret);

    if (bios_only || !ret)
        return ret;

    if (yes_log < 0)
        yes_log = (getenv("YES_LOG") != NULL);

    memset(&yes, 0x00, sizeof(yes));
    yes.mmcs     = 0x0000;
    yes.vid_base = 0x10000;

    pclog("[YES] init: %d KB RAM, BIOS at %05X\n", mem_size, YES_BIOS_ADDR);

    /* 80186 on-chip peripherals.  DRQ0 is the SASI port, DRQ1 the formatter
       (the same assignment the MAME driver makes).  It has to exist before
       the WD279x is created: that device asks the PCB for its DMA port. */
    {
        struct i8018x_pcb_t *pcb = device_add(&i8018x_pcb_device);

        yes.pcb = pcb;
        i8018x_pcb_set_dma_channels(pcb, 0, 1);
        /* TIMERIN0 is the board's 1 MHz-ish tick; without a divider the
           timer fires on every CPU clock and the ROM's tick handler turns
           into an interrupt storm.  16 is the divider the PC-X board uses
           for the same 80186 part, a first cut until the :YES schematic
           value is known. */
        i8018x_pcb_set_timer_input(pcb, 0, 16);
    }

    yes.fdc = wd279x_init_at(YES_FDC_BASE);
    {
        const char *sb = getenv("YES_SBASE");

        if (sb != NULL)
            wd279x_set_sector_base(yes.fdc, atoi(sb));
        wd279x_set_reg_step(yes.fdc, 2); /* the ROM uses 0x480/2/4/6 */
    }
    if (yes.fdc != NULL) {
        wd279x_set_irq_handler(yes.fdc, yes_fdc_irq, NULL);
        fdcinited = 1;
    }

    yes.muart = intel_8256_init(YES_MUART_BASE);
    if (getenv("YES_NONOMVIDEO") == NULL)
        yes.video = device_add(&yes_video_device);

    if (yes.muart != NULL) {
        intel_8256_set_irq_handler(yes.muart, yes_muart_irq, NULL);
        /* The ICU leaves INT0 and INT3 masked (mask register reads 0x9c), so the
           MUART's interrupt output is on INT1; the acknowledge cycle asks the
           part for its vector. */
        i8018x_pcb_set_ext_ack(yes.pcb, 1, yes_muart_ack, NULL);
    }
    yes_io_init();

    timer_add(&yes_sample_timer, yes_sample, NULL, 0);
    timer_set_delay_u64(&yes_sample_timer, 20000 * TIMER_USEC);

    {
        const char *k = getenv("YES_KEY");

        if (k != NULL) {
            yes.kbd_code = (uint8_t) strtoul(k, NULL, 16);
            timer_add(&yes_key_timer, yes_key_tick, NULL, 0);
            timer_set_delay_u64(&yes_key_timer, 3000000 * TIMER_USEC);
        }
    }

    if (getenv("YES_DUMP") != NULL) {
        timer_add(&yes_vram_timer, yes_dump_vram, NULL, 0);
        timer_set_delay_u64(&yes_vram_timer, 6000000 * TIMER_USEC);
    }

    /* The frame buffer sits inside RAM at 0x10000 (128 KB machine) up to
       0x90000 (640 KB machine); it is decoded by the display once that is
       modelled, so nothing is mapped over the RAM here. */
    return 1;
}
