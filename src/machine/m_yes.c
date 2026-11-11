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
#include <86box/keyboard.h>
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
#define YES_STRAP_SMALL 0x01  /* bit 0 of the system control port: 128 KB base memory */
#define YES_RTC        0x066a
#define YES_RTCD       0x066e
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
    int      kbd_pending; /* a scan code is latched and not read yet */
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

/* The keyboard reports standard PC set-1 scan codes: the guest's own IO.SYS
   decodes them with the two tables at file offset 0x0AE4 (no shift) and 0x0B1F
   (shift), both named "UK 8-Bit", and the ROM hands the raw code on to the BIOS
   buffer without translating it.

   It sends make codes only -- there is no break code on this board.  The ROM
   merges every break code it sees into the BIOS shift-flag byte at 0x40:0x17
   (ISR at 0xFFA07), and the conversion path then reads that byte as the
   modifier state, so a break of 0x13 makes the next 'r' come out as ^R
   (0x12 = 'r' & 0x1f).  With no breaks the flag stays 0 and the letters decode
   normally.  Extended (E0) keys are left empty too: this keyboard has none. */
static const scancode yes_scancode_table[512] = {
  // clang-format off
    [0x01] = { .mk = { 0x01 } },
    [0x02] = { .mk = { 0x02 } },
    [0x03] = { .mk = { 0x03 } },
    [0x04] = { .mk = { 0x04 } },
    [0x05] = { .mk = { 0x05 } },
    [0x06] = { .mk = { 0x06 } },
    [0x07] = { .mk = { 0x07 } },
    [0x08] = { .mk = { 0x08 } },
    [0x09] = { .mk = { 0x09 } },
    [0x0A] = { .mk = { 0x0A } },
    [0x0B] = { .mk = { 0x0B } },
    [0x0C] = { .mk = { 0x0C } },
    [0x0D] = { .mk = { 0x0D } },
    [0x0E] = { .mk = { 0x0E } },
    [0x0F] = { .mk = { 0x0F } },
    [0x10] = { .mk = { 0x10 } },
    [0x11] = { .mk = { 0x11 } },
    [0x12] = { .mk = { 0x12 } },
    [0x13] = { .mk = { 0x13 } },
    [0x14] = { .mk = { 0x14 } },
    [0x15] = { .mk = { 0x15 } },
    [0x16] = { .mk = { 0x16 } },
    [0x17] = { .mk = { 0x17 } },
    [0x18] = { .mk = { 0x18 } },
    [0x19] = { .mk = { 0x19 } },
    [0x1A] = { .mk = { 0x1A } },
    [0x1B] = { .mk = { 0x1B } },
    [0x1C] = { .mk = { 0x1C } },
    [0x1D] = { .mk = { 0x1D } },
    [0x1E] = { .mk = { 0x1E } },
    [0x1F] = { .mk = { 0x1F } },
    [0x20] = { .mk = { 0x20 } },
    [0x21] = { .mk = { 0x21 } },
    [0x22] = { .mk = { 0x22 } },
    [0x23] = { .mk = { 0x23 } },
    [0x24] = { .mk = { 0x24 } },
    [0x25] = { .mk = { 0x25 } },
    [0x26] = { .mk = { 0x26 } },
    [0x27] = { .mk = { 0x27 } },
    [0x28] = { .mk = { 0x28 } },
    [0x29] = { .mk = { 0x29 } },
    [0x2A] = { .mk = { 0x2A } },
    [0x2B] = { .mk = { 0x2B } },
    [0x2C] = { .mk = { 0x2C } },
    [0x2D] = { .mk = { 0x2D } },
    [0x2E] = { .mk = { 0x2E } },
    [0x2F] = { .mk = { 0x2F } },
    [0x30] = { .mk = { 0x30 } },
    [0x31] = { .mk = { 0x31 } },
    [0x32] = { .mk = { 0x32 } },
    [0x33] = { .mk = { 0x33 } },
    [0x34] = { .mk = { 0x34 } },
    [0x35] = { .mk = { 0x35 } },
    [0x36] = { .mk = { 0x36 } },
    [0x37] = { .mk = { 0x37 } },
    [0x38] = { .mk = { 0x38 } },
    [0x39] = { .mk = { 0x39 } },
    [0x3A] = { .mk = { 0x3A } },
    [0x3B] = { .mk = { 0x3B } },
    [0x3C] = { .mk = { 0x3C } },
    [0x3D] = { .mk = { 0x3D } },
    [0x3E] = { .mk = { 0x3E } },
    [0x3F] = { .mk = { 0x3F } },
    [0x40] = { .mk = { 0x40 } },
    [0x41] = { .mk = { 0x41 } },
    [0x42] = { .mk = { 0x42 } },
    [0x43] = { .mk = { 0x43 } },
    [0x44] = { .mk = { 0x44 } },
    [0x45] = { .mk = { 0x45 } },
    [0x46] = { .mk = { 0x46 } },
    [0x47] = { .mk = { 0x47 } },
    [0x48] = { .mk = { 0x48 } },
    [0x49] = { .mk = { 0x49 } },
    [0x4A] = { .mk = { 0x4A } },
    [0x4B] = { .mk = { 0x4B } },
    [0x4C] = { .mk = { 0x4C } },
    [0x4D] = { .mk = { 0x4D } },
    [0x4E] = { .mk = { 0x4E } },
    [0x4F] = { .mk = { 0x4F } },
    [0x50] = { .mk = { 0x50 } },
    [0x51] = { .mk = { 0x51 } },
    [0x52] = { .mk = { 0x52 } },
    [0x53] = { .mk = { 0x53 } },
    [0x54] = { .mk = { 0x54 } },
    [0x55] = { .mk = { 0x55 } },
    [0x56] = { .mk = { 0x56 } },
    [0x57] = { .mk = { 0x57 } },
    [0x58] = { .mk = { 0x58 } },
    [0x59] = { .mk = { 0x59 } },
    [0x5A] = { .mk = { 0x5A } },
    [0x5B] = { .mk = { 0x5B } },
    [0x5C] = { .mk = { 0x5C } },
    [0x5D] = { .mk = { 0x5D } },
    [0x5E] = { .mk = { 0x5E } },
    [0x5F] = { .mk = { 0x5F } },
    [0x60] = { .mk = { 0x60 } },
    [0x61] = { .mk = { 0x61 } },
    [0x62] = { .mk = { 0x62 } },
    [0x63] = { .mk = { 0x63 } },
    [0x64] = { .mk = { 0x64 } },
    [0x65] = { .mk = { 0x65 } },
    [0x66] = { .mk = { 0x66 } },
    [0x67] = { .mk = { 0x67 } },
    [0x68] = { .mk = { 0x68 } },
    [0x69] = { .mk = { 0x69 } },
    [0x6A] = { .mk = { 0x6A } },
    [0x6B] = { .mk = { 0x6B } },
    [0x6C] = { .mk = { 0x6C } },
    [0x6D] = { .mk = { 0x6D } },
    [0x6E] = { .mk = { 0x6E } },
    [0x6F] = { .mk = { 0x6F } },
    [0x70] = { .mk = { 0x70 } },
    [0x71] = { .mk = { 0x71 } },
    [0x72] = { .mk = { 0x72 } },
    [0x73] = { .mk = { 0x73 } },
    [0x74] = { .mk = { 0x74 } },
    [0x75] = { .mk = { 0x75 } },
    [0x76] = { .mk = { 0x76 } },
    [0x77] = { .mk = { 0x77 } },
    [0x78] = { .mk = { 0x78 } },
    [0x79] = { .mk = { 0x79 } },
    [0x7A] = { .mk = { 0x7A } },
    [0x7B] = { .mk = { 0x7B } },
    [0x7C] = { .mk = { 0x7C } },
    [0x7D] = { .mk = { 0x7D } },
    [0x7E] = { .mk = { 0x7E } },
    [0x7F] = { .mk = { 0x7F } },
  // clang-format on
};

/* Called by the keyboard core for every byte it emits.  The board is just a
   latch on 0x5a0 plus one MUART interrupt level: the code is held here and the
   part answers the acknowledge with vector 0x41, which is the ROM's keyboard
   handler.  The request stays raised until the guest reads the code back --
   that read is the acknowledge. */
static void
yes_kbd_send(uint16_t val)
{
    if ((val == 0x100) || (val == 0x101))
        return;    /* fake-shift markers the core may emit, not scan codes */

    yes.kbd_code    = (uint8_t) (val & 0xff);
    yes.kbd_pending = 1;
    yes_log_write("kbd", YES_KBD, yes.kbd_code);
    intel_8256_set_level(yes.muart, 1);
}

/* The keyboard holds its request line until the guest reads the code back; the
   MUART only carries a pending bit, so the line is re-asserted while the latch
   is unread.  Without this the power-on code raised during init is lost: the
   ICU is still masked then, and a read of the MUART's INTAD drops the request. */
static pc_timer_t yes_kbd_hold_timer;

static void
yes_kbd_hold(void *priv)
{
    if (yes.kbd_pending)
        intel_8256_set_level(yes.muart, 1);
    timer_set_delay_u64(&yes_kbd_hold_timer, 20000 * TIMER_USEC);
}

static uint8_t
yes_kbd_read(uint16_t port, void *priv)
{
    if (yes.kbd_pending) {
        yes.kbd_pending = 0;
        intel_8256_clear_level(yes.muart, 1);
    }
    return yes_log_read("kbd", port, yes.kbd_code);
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
    /* Bit 0 reads back as a board strap, not as the latched value.  The POST
       writes 0x43 here, reads the port back and branches on that bit: set, it
       takes the 128 KB base-memory path and stores 0x78 (120 KB) at 0x413,
       which is what makes DOS Plus stop with "Configuration too large for
       memory"; clear, it goes on to probe the 512 KB expansion.  A 640 KB
       machine has the strap clear. */
    uint8_t     r = (uint8_t) (yes.sysctl & ~YES_STRAP_SMALL);
    const char *e = getenv("YES_STRAP");

    if (e != NULL)
        r = (uint8_t) ((r & ~YES_STRAP_SMALL) | (strtoul(e, NULL, 16) & YES_STRAP_SMALL));

    return yes_log_read("sysctl", port, r);
}

static void
yes_rtc_write(uint16_t port, uint8_t val, void *priv)
{
    yes_log_write("rtc", port, val);
}

static uint8_t
yes_rtcd_read(uint16_t port, void *priv)
{
    /* The RTC answers bit-serially on bit 0; the value is a probe knob until
       the protocol itself is modelled. */
    const char *v = getenv("YES_RTCD");
    uint8_t     b = (v != NULL) ? (uint8_t) strtoul(v, NULL, 16) : 0x00;

    return b;
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



        {
            static int n;

            if (n < 16)
                pclog("[SASIR] port=%04X -> %02X @%04X:%04X\n", port, idle,
                      cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc), n++;
        }
        return yes_log_read("sasi", port, idle);
    }
}

static uint16_t
yes_sasi_read16(uint16_t port, void *priv)
{
    const char *v = getenv("YES_SASI");
    uint8_t     idle = (v != NULL) ? (uint8_t) strtoul(v, NULL, 16) : 0x00;

    return (uint16_t) (idle | (idle << 8));
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
    io_sethandler(YES_RTCD, 0x0001, yes_rtcd_read, NULL, NULL, NULL, NULL, NULL,
                  NULL);
    io_sethandler(YES_RTC, 0x0001, yes_rtc_read, NULL, NULL, yes_rtc_write, NULL,
                  NULL, NULL);
    io_sethandler(YES_SASI_DATA, 0x0001, yes_sasi_read, yes_sasi_read16, NULL,
                  yes_sasi_write, NULL, NULL, NULL);
    io_sethandler(YES_SASI_STAT, 0x0001, yes_sasi_read, yes_sasi_read16, NULL,
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

    /* The keyboard needs nothing but a code table and a send callback: 0x5a0 is
       the latch and one MUART level is the interrupt (see yes_kbd_send). */
    keyboard_set_table(yes_scancode_table);
    keyboard_send = yes_kbd_send;

    /* The board sends its power-on code once, and the ROM's keyboard ISR spends
       its first interrupt on latching only ([0x92] == 0 at 0xFF6E1): without
       that code the first real key would never reach the BIOS buffer.  The
       request stays pending until the guest enables the MUART level. */
    yes.kbd_code    = 0xaa;
    yes.kbd_pending = 1;
    timer_add(&yes_kbd_hold_timer, yes_kbd_hold, NULL, 0);
    timer_set_delay_u64(&yes_kbd_hold_timer, 20000 * TIMER_USEC);


    timer_add(&yes_sample_timer, yes_sample, NULL, 0);
    timer_set_delay_u64(&yes_sample_timer, 20000 * TIMER_USEC);

    if (getenv("YES_DUMP") != NULL) {
        timer_add(&yes_vram_timer, yes_dump_vram, NULL, 0);
        timer_set_delay_u64(&yes_vram_timer, 6000000 * TIMER_USEC);
    }

    /* The frame buffer sits inside RAM at 0x10000 (128 KB machine) up to
       0x90000 (640 KB machine); it is decoded by the display once that is
       modelled, so nothing is mapped over the RAM here. */
    return 1;
}
