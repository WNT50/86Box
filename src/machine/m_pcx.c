/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-X emulation: 80186 board, SINIX paging MMU, WD2791
 *          floppy controller, on-board SM912 SCSI, three SCN2661B USARTs and
 *          the SCN2672/8031 graphics board.
 *
 *          SKELETON.  What works here: the address decode (RAM / graphics
 *          window / ROM / undecoded space that raises NMI), the status and
 *          configuration ports, the MMU hookup and the interrupt wiring.
 *          The floppy controller, SCSI, USARTs and video board are stubs that
 *          read 0xff, so the machine does not boot yet.
 *
 *          Sources (docs/86box/pcx-machine-plan/README.md has the details):
 *            - Siemens PC-D Servicehandbuch Teil 1, sections 5.10 and 5.5:
 *              memory map, RAM sizes, S2/S3/S4 board switches, bus timeout.
 *            - MAME src/mame/siemens/pcd.cpp + pcd_v.cpp for the I/O map and
 *              the MMU register layout.
 *            - PC-D BIOS disassembly: POST writes the usable RAM ceiling as a
 *              segment to 0040:0014 (0xf000 / 0xd800 / 0xc000, depending on the
 *              display board it detects).
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <86box/plat.h>
#include <86box/keyboard.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/machine.h>
#include <86box/mem.h>

extern volatile int fdcinited;
#include <86box/mem.h>
#include <86box/nmi.h>
#include <86box/pcx_mmu.h>
#include <86box/pic.h>
#include <86box/rom.h>
#include <86box/i8018x.h>
#include <86box/wd2791.h>
#include <86box/scsi_sm912.h>
#include <86box/serial.h>
#include <86box/kbc_siemens.h>
#include <86box/keyboard.h>
#include <86box/pcd_vid.h>
#include <86box/pcx_vid.h>
#include <86box/plat_unused.h>
#include "cpu.h"

/* BIOS: two 8 KB halves of one 16-bit ROM.  The low byte half (.d43) is at
   even addresses, the high byte half (.d42) at odd addresses -- exactly what
   bios_load_interleaved() wants, first argument is the even one.

   Both machines run the same board ROM; what differs is the video device the
   machine installs.  V3 GS4 is the revision to use: its disk check accepts the
   SINIX boot record (track 0 sector 7 -> 0x63e, byte 4 must be 'S'), which the
   earlier V2 GS build never tries because it only takes that fallback when the
   sector-1 read itself fails.  The V2 pair is kept next to it as s26361-d359.*
   for A/B runs; switching this macro back to it gives a PC-X that stops in the
   ROM retry loop instead of booting the SINIX disk. */
#define PCD_BIOS_LOW      "roms/machines/pcd/361d0359.d43"
#define PCD_BIOS_HIGH     "roms/machines/pcd/361d0359.d42"
#define PCX_BIOS_LOW      "roms/machines/pcx/361d0359.d43"
#define PCX_BIOS_HIGH     "roms/machines/pcx/361d0359.d42"
#define PCX_BIOS_ADDR     0x000fc000
#define PCX_BIOS_SIZE     0x00004000

/* TMRIN0 pulse period, in CPU clocks.  The service manual (Teil II, adapter
   type 5, p. 4-11) puts timer 0's count input on the serial interface's
   Schrittakt, which the board derives as 8 MHz / 16 = 500 kHz.  The POST then
   times its calibration loop against one 25001-count period and accepts
   8192..24576 passes; 16 clocks puts us at ~14,300, inside that window and
   above the 12,288 the V2 GS BIOS demands (12 clocks gave 10,714, which the
   V3 GS4 BIOS accepts but V2 rejects with "BASIC-FUNCTION ERROR"). */
#define PCX_TMRIN_CLOCKS  16

/* System RAM runs from 0 up to the graphics board: the service manual's 1 MB
   configuration note (Teil I, p. 4-11) says the refresh memory, the EPROM and
   any expansion board also sit in the 1 MB address range and that the RAM is
   blanked out there by an inhibit signal -- i.e. the memory fills everything
   the video board and the ROM do not claim, so the ceiling is the window
   itself.  The board owns the 32 KB at 0xf0000 and the 16 KB at 0xf8000 are
   left undecoded.  (MAME's pcd_map() stops at 0x7ffff, but pcd is flagged
   MACHINE_NOT_WORKING and only models the 512 KB machine.) */
#define PCX_VRAM_BASE     0x000f0000
#define PCX_VRAM_SIZE     0x00008000
#define PCX_RAM_CEIL      PCX_GRAPH_TOP
#define PCX_GRAPH_TOP     0x000fc000

/* I/O */
#define PCX_PIC1_PORT     0x00f800
#define PCX_PIC2_PORT     0x00f820
#define PCX_STAT_PORT     0x00f840
#define PCX_DIP_PORT      0x00f841
#define PCX_FAULTSEG_PORT 0x00f842
#define PCX_RTC_PORT      0x00f880
#define PCX_RTC_SIZE      0x0040 /* 64 registers, 0x0e-0x3f are battery RAM */
#define PCX_RTC_DATA      0x00f880
#define PCX_RTC_STATUS    0x00f88a
#define PCX_RTC_CMD       0x00f88b
#define PCX_NVRAM_PORT    0x00f000   /* 2 KB battery-backed config RAM */
#define PCX_NVRAM_SIZE    0x000800
#define PCX_VIDEO_DATA    0x00f980
#define PCX_VIDEO_STATUS  0x00f981
#define PCX_VIDEO_CTRL    0x00f982
#define PCX_VIDEO_EXEC    0x00f98f
#define PCX_DSKCTL2_PORT  0x00f905
#define PCX_FDC_PORT      0x00f900
#define PCX_DSKCTL_PORT   0x00f904
#define PCX_SCSI_PORT     0x00f940
#define PCX_VIDEO_PORT    0x00f980
#define PCX_USART1_PORT   0x00f9c0
#define PCX_USART2_PORT   0x00f9d0
#define PCX_USART3_PORT   0x00f9e0
#define PCX_MMU_PORT      0x008000

/* Status byte, 0xf840.  The BIOS latches the same 0xfd MAME uses for an
   undecoded access; writing a 1 clears the corresponding bit. */
#define PCX_STAT_NMI      0xfd

typedef struct pcx_t {
    mem_mapping_t ram_map;   /* RAM from 640 KB up, when that much is fitted */
    mem_mapping_t hole_map;  /* undecoded space above the fitted RAM */
    rom_t         bios;

    uint8_t stat;    /* 0xf840: error/latch byte, write-1-to-clear */
    uint8_t dip;     /* 0xf841: read = S9 switches, write = LED latch */
    uint8_t led;     /* what the BIOS last wrote to 0xf841 */
    uint8_t dskctl;  /* 0xf904 */
    uint8_t dskctl2; /* 0xf905: second latch, read-modify-written by the POST */

    uint8_t nvram[PCX_NVRAM_SIZE];
    uint8_t       rtc[PCX_RTC_SIZE]; /* 0xf880-0xf8bf, MC146818 register file */
    pc_timer_t    rtc_timer;         /* one-second time base for registers 0-9 */
    uint8_t       kbd_q[8];          /* keyboard answers waiting for the receiver */
    int           kbd_qh, kbd_qt, kbd_out;

    uint8_t vid_data, vid_ctrl, vid_exec;
    uint8_t err_seg; /* A19..A12 of the last bus timeout / parity error */

    wd2791_t *fdc;   /* WD2791 formatter, 0xf900-0xf903 */
    void          *usart[3]; /* SCN2661B EPCI at 0xf9c0 / 0xf9d0 / 0xf9e0 */
    scsi_sm912_t  *scsi;     /* SM912 SCSI adapter at 0xf940, IRQ 5 */
    kbc_siemens_t *kbc;      /* keyboard/terminal channel behind 0xf980 */
} pcx_t;

static pcx_t pcx;

/* Which machine is being initialised (set before the common init runs). */
static int pcx_is_pcd = 0;

static uint8_t pcx_usart2_read(uint16_t addr, void *priv);

static uint8_t
pcx_usart2_read(uint16_t addr, void *priv)
{
    uint8_t ret = scn26x1_read(addr, priv);


    if ((getenv("PCX_RXLOG") != NULL) && (CS == 0xfc00) &&
        (cpu_state.pc == 0x0c86))
        pclog("[CMP] al=%02X cl=%02X port=%03X\n", ret,
              cpu_state.regs[1].w & 0xff, addr);

    return ret;
}



/* ------------------------------------------------------------------------- */
/* Undecoded space                                                           */
/* ------------------------------------------------------------------------- */

/* The board has no bus error signal; a cycle to an address nothing answers is
   caught by a time-out after roughly 128 us (Servicehandbuch 5.5) and turned
   into an NMI.  MAME returns 0 from its fallback handlers; a real open bus
   reads 0xff, which is what we give the guest here -- if the POST turns out to
   look at the value, this is the place to change. */
/* The NMI of this board comes from the parity logic and from the bus time-out;
   there is no 0xa0-style mask register in front of it, so the CPU's NMI input is
   always armed.  (nmi_mask is a global that nmi_init() clears, and no port of
   this machine ever sets it, which would swallow every NMI.) */
static void
pcx_nmi_enable(void)
{
    nmi_mask = 1;
}

static void
pcx_bus_timeout(uint32_t addr)
{
    if (getenv("PCX_NMILOG") != NULL) {
        static int n;
        if (n < 200) {
            n++;
            pclog("[NMILOG] t=%u addr=%05X @%04X:%04X ds=%04X\n", plat_get_ticks(),
                  addr & 0xfffffu, cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc,
                  cpu_state.seg_ds.seg);
        }
    }
    pcx.stat |= PCX_STAT_NMI;
    pcx.err_seg = (uint8_t) (addr >> 12);
    nmi_raise();
}

static uint8_t
pcx_nmi_readb(uint32_t addr, UNUSED(void *priv))
{
    pcx_bus_timeout(addr);
    return 0xff;
}

static uint16_t
pcx_nmi_readw(uint32_t addr, UNUSED(void *priv))
{
    pcx_bus_timeout(addr);
    return 0xffff;
}

static uint32_t
pcx_nmi_readl(uint32_t addr, UNUSED(void *priv))
{
    pcx_bus_timeout(addr);
    return 0xffffffff;
}

static void
pcx_nmi_writeb(uint32_t addr, UNUSED(uint8_t val), UNUSED(void *priv))
{
    pcx_bus_timeout(addr);
}

/* The same time-out applies to I/O cycles: the board watches the bus, not just
   the memory decode.  The POST relies on it -- it reads 0xfbbf's neighbour
   0xfbbe in a loop and only leaves that loop when the NMI handler has run
   (v3 BIOS 0xfc911). */
static uint8_t
pcx_io_hole_read8(uint16_t port, UNUSED(void *priv))
{
    pcx_bus_timeout(port);
    return 0xff;
}

static uint16_t
pcx_io_hole_read16(uint16_t port, UNUSED(void *priv))
{
    pcx_bus_timeout(port);
    return 0xffff;
}

static void
pcx_io_hole_write8(uint16_t port, UNUSED(uint8_t val), UNUSED(void *priv))
{
    pcx_bus_timeout(port);
}

static void
pcx_io_hole_write16(uint16_t port, UNUSED(uint16_t val), UNUSED(void *priv))
{
    pcx_bus_timeout(port);
}

static void
pcx_nmi_writew(uint32_t addr, UNUSED(uint16_t val), UNUSED(void *priv))
{
    pcx_bus_timeout(addr);
}

static void
pcx_nmi_writel(uint32_t addr, UNUSED(uint32_t val), UNUSED(void *priv))
{
    pcx_bus_timeout(addr);
}

/* ------------------------------------------------------------------------- */
/* Ports                                                                     */
/* ------------------------------------------------------------------------- */

/* Called by the mmu when its I/O trap raises NMI; the status port is the only
   way the handler can tell the sources apart. */
static void
pcx_nmi_status_set(uint8_t bits)
{
    pcx.stat |= bits;
}

static uint8_t
pcx_stat_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return pcx.stat;
}

static void
pcx_stat_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    pcx.stat &= ~val;
}

static uint8_t
pcx_dip_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return pcx.dip;
}

static void
pcx_dip_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    /* Reading 0xf841 returns the S9 switches, writing it drives the LED latch:
       the POST writes 0x3e there (v3 BIOS 0x5e2) and 0xc0 to 0xf840 before it
       touches anything else.  Keep the value for a display model later. */

    /* PCX_LEDLOG=1: the POST reports progress by code on this latch. */
    if (getenv("PCX_LEDLOG")) {
        static long led_n;

        led_n++;
        if (led_n <= 120)
            pclog("[LED] #%ld val=%02X\n", led_n, val);
    }

    pcx.led = val;
}


/* Undecoded ports on the PC-X read back as zero, not open bus 0xff: the POST's
   device scan (0xfb00-0xfb0f) decides presence from the high byte, and the
   DOS boot loader checks the resulting bitmap at 0x402. */


/* USART2 carries the keyboard.  The DOS loader asks "are you there?" with a
   lone '&' on the transmit side and then polls RxRDY on 0xf9d1 (0x6ace); it
   only accepts '<'.  Play the keyboard's part: hand the answer straight back
   to the receiver. */
static void
pcx_kbd_usart_write(uint16_t addr, uint8_t val, void *priv)
{
    scn26x1_write(addr, val, priv);

    /* The loader asks whether a keyboard is there with a lone '&' on this
       line and only accepts '<'.  The POST's own self test is answered by the
       chip, so only that one question is answered here. */
    if (((addr & 0x0003) == 0x0000) && (val == 0x26) && (CS != 0xfc00))
        serial_endpoint_receive(scn26x1_endpoint(priv), '<');

}

/* The PC-X console has no 8042: the keys go to the 8031 terminal board, which
   hands them to the host over USART2.  With no key source the shell prints its
   prompt and sits in HLT forever, so take the scan codes 86Box produces and
   hand them to the second USART the way the board would. */
static const char pcx_kbd_plain[0x40] = {
    [0x02] = '1', [0x03] = '2', [0x04] = '3', [0x05] = '4', [0x06] = '5',
    [0x07] = '6', [0x08] = '7', [0x09] = '8', [0x0a] = '9', [0x0b] = '0',
    [0x0c] = '-', [0x0d] = '=', [0x0e] = '\b', [0x0f] = '\t',
    [0x10] = 'q', [0x11] = 'w', [0x12] = 'e', [0x13] = 'r', [0x14] = 't',
    [0x15] = 'y', [0x16] = 'u', [0x17] = 'i', [0x18] = 'o', [0x19] = 'p',
    [0x1a] = '[', [0x1b] = ']', [0x1c] = '\r',
    [0x1e] = 'a', [0x1f] = 's', [0x20] = 'd', [0x21] = 'f', [0x22] = 'g',
    [0x23] = 'h', [0x24] = 'j', [0x25] = 'k', [0x26] = 'l', [0x27] = ';',
    [0x28] = '\'', [0x29] = '`', [0x2b] = '\\',
    [0x2c] = 'z', [0x2d] = 'x', [0x2e] = 'c', [0x2f] = 'v', [0x30] = 'b',
    [0x31] = 'n', [0x32] = 'm', [0x33] = ',', [0x34] = '.', [0x35] = '/',
    [0x39] = ' '
};

static const char pcx_kbd_shifted[0x40] = {
    [0x02] = '!', [0x03] = '@', [0x04] = '#', [0x05] = '$', [0x06] = '%',
    [0x07] = '^', [0x08] = '&', [0x09] = '*', [0x0a] = '(', [0x0b] = ')',
    [0x0c] = '_', [0x0d] = '+',
    [0x1a] = '{', [0x1b] = '}', [0x27] = ':', [0x28] = '"', [0x29] = '~',
    [0x2b] = '|', [0x33] = '<', [0x34] = '>', [0x35] = '?'
};

static int pcx_kbd_shift;

static void
pcx_kbd_host(uint16_t scan, int down, UNUSED(void *priv))
{
    uint8_t sc = (uint8_t) (scan & 0xff);
    char    ch;

    if (!down) {
        if ((sc == 0x2a) || (sc == 0x36))
            pcx_kbd_shift = 0;
        return;
    }

    if ((sc == 0x2a) || (sc == 0x36)) {
        pcx_kbd_shift = 1;
        return;
    }

    ch = pcx_kbd_shift ? pcx_kbd_shifted[sc & 0x3f] : pcx_kbd_plain[sc & 0x3f];
    if (getenv("PCX_KBDLOG") != NULL)
        pclog("[KBD] sc=%02X ch=%02X\n", sc, (uint8_t) ch);
    if (ch == 0)
        return;

    /* The PC-X console is the terminal byte stream on 0xf980/0xf981, which the
       kbc_siemens model serves; the PC-D's Graphik-BIOS instead puts its
       keyboard on the second USART.  Sending PC-X keys to USART2 left SINIX
       halted at 1000:03a4 with no way to type anything. */
    if (!pcx_is_pcd) {
        if (pcx.kbc != NULL)
            kbc_siemens_key(pcx.kbc, (uint8_t) ch);
        return;
    }

    serial_endpoint_receive(scn26x1_endpoint(pcx.usart[1]), (uint8_t) ch);
}


static uint8_t
pcx_scan_read8(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return 0x00;
}

static uint8_t
pcx_faultseg_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    /* 0xf842: what the NMI handler reads to name the segment the fault was
       taken in.  The forum only knows the name the manual gives it
       ("SPEICHER-SEGMENT") and asks whether it is the high eight bits of the
       faulting address; nothing in the ROM or the kernel reads it back in a way
       that settles the polarity, so report bits 19..16 of the address the card
       refused and let the firmware print what it likes. */
    return (uint8_t) ((pcx_mmu_fault_addr() >> 16) & 0xff);
}

static uint8_t
pcx_dskctl_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return pcx.dskctl;
}

static void
pcx_dskctl_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    /* bit 0/1 = drive 0/1 select, bit 2 = motor on, bit 3 = side select,
       bit 4 = DDEN (1 = FM single density, 0 = MFM).  Same mapping MAME uses
       in pcd.cpp:190-214.  The controller latches what it needs for drive,
       side and density; the byte is also kept for a read back. */
    pcx.dskctl = val;
    wd2791_set_dskctl(pcx.fdc, val);
}

static uint8_t
pcx_dskctl2_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return pcx.dskctl2;
}

static void
pcx_dskctl2_write(UNUSED(uint16_t port), uint8_t val, UNUSED(void *priv))
{
    /* The POST's helper at v3 0xfc26b does a read-modify-write of 0xf904 and
       0xf905 as a pair (density/side select); keep the second latch so the
       read back is what the board would hold, not open bus. */
    pcx.dskctl2 = val;
}

/* 2 KB NVRAM the BIOS uses for the machine configuration.  It is mapped as
   I/O on this board, not as memory. */
static uint8_t
pcx_nvram_read(uint16_t port, UNUSED(void *priv))
{
    return pcx.nvram[(port - PCX_NVRAM_PORT) & (PCX_NVRAM_SIZE - 1)];
}

static void
pcx_nvram_write(uint16_t port, uint8_t val, UNUSED(void *priv))
{
    pcx.nvram[(port - PCX_NVRAM_PORT) & (PCX_NVRAM_SIZE - 1)] = val;
}

/* RTC / NVRAM control block.  The POST (v3 0xfc95c) reads 0xf88a, expects
   bits 4-6 to read 0b010, then checks bit 7 of 0xf88b as "calibrated"; after
   writing 0x20/0x02 it polls bit 7 of 0xf88a until it clears, so the status
   register reports idle at once. */
/* The register file is a real clock, not a scratch array: the POST compares
   the seconds register with a RAM byte (v3 0xfcd8b) and wants them to differ,
   and later the guest sets and reads the time through the same registers.
   Registers 0-9 are the time in BCD (or binary when register 0x0b bit 2 says
   so); everything from 0x0e up is battery-backed RAM the guest owns. */
static uint8_t
pcx_rtc_bcd(int val)
{
    return (uint8_t) (((val / 10) << 4) | (val % 10));
}

static void
pcx_rtc_tick(void *priv)
{
    time_t     now = time(NULL);
    struct tm *tm  = localtime(&now);
    int        bin = (pcx.rtc[0x0b] & 0x04) ? 1 : 0;

    if (tm != NULL) {
        pcx.rtc[0x00] = bin ? (uint8_t) tm->tm_sec : pcx_rtc_bcd(tm->tm_sec);
        pcx.rtc[0x02] = bin ? (uint8_t) tm->tm_min : pcx_rtc_bcd(tm->tm_min);
        pcx.rtc[0x04] = bin ? (uint8_t) tm->tm_hour : pcx_rtc_bcd(tm->tm_hour);
        pcx.rtc[0x06] = bin ? (uint8_t) tm->tm_wday : pcx_rtc_bcd(tm->tm_wday);
        pcx.rtc[0x07] = bin ? (uint8_t) tm->tm_mday : pcx_rtc_bcd(tm->tm_mday);
        pcx.rtc[0x08] = bin ? (uint8_t) (tm->tm_mon + 1) : pcx_rtc_bcd(tm->tm_mon + 1);
        pcx.rtc[0x09] = bin ? (uint8_t) (tm->tm_year % 100) : pcx_rtc_bcd(tm->tm_year % 100);
    }

    /* MC146818: a one-second update-end sets UF in register C and, when the
       guest has enabled UIE (register B bit 4), pulls IRQ7.  MAME wires the
       part's IRQ to PIC1 IR7 (pcd.cpp:575); reading C acknowledges it. */
    if ((pcx.rtc[0x0b] & 0x10) && (getenv("PCX_NORTCTICK") == NULL)) {
        pcx.rtc[0x0c] |= 0x10;
        picint(1 << 7);
    }

    /* Temporary: type an answer into the installer, one byte per second step. */
    if (getenv("PCX_TTYPOLL") != NULL) {
        static uint32_t last = 0;
        uint32_t t = plat_get_ticks();
        if (t > 50000 && (t - last) > 4000) {
            last = t;
            pclog("[TP] t=%u cnt=%02X%02X rd25=%02X\n", t,
                  readmembl(0x6ffb), readmembl(0x6ffa), readmembl(0x6ffa + 0x25));
        }
    }

    if ((getenv("PCX_KDUMP") != NULL) && (plat_get_ticks() > 60000)) {
        static int done = 0;
        if (!done) {
            FILE *f = fopen("/tmp/kern.bin", "wb");
            if (f != NULL) {
                fwrite(&ram[0x10000], 1, 0x20000, f);
                fclose(f);
            }
            done = 1;
            pclog("[KDUMP] written\n");
        }
    }

    if (getenv("PCX_AUTOKEY") != NULL) {
        static int      stage = 0;
        static uint32_t next  = 0;
        uint32_t        t     = plat_get_ticks();

        /* Drive the installer off the screen instead of the wall clock: the
           first key is eaten when formint switches the tty to raw mode, so a
           throwaway key goes first, then the answer. */
        if ((stage == 0) && pcx_video_screen_contains("Antworten Sie mit")) {
            kbc_siemens_key(pcx.kbc, ' ');
            stage = 1;
            next  = t + 600;
            pclog("[AUTO] prompt at %u, priming key\n", t);
        } else if ((stage == 1) && (t >= next)) {
            kbc_siemens_key(pcx.kbc, 'j');
            stage = 2;
            next  = t + 400;
            pclog("[AUTO] j at %u\n", t);
        } else if ((stage >= 2) && (t >= next)) {
            /* From here on the installer waits for a key at whatever page it
               likes ("Weiter mit").  Press Return only once the screen has
               stopped changing: a key that lands mid-paint gets dropped. */
            static uint32_t last_hash   = 0;
            static uint32_t last_change = 0;
            uint32_t        h = pcx_video_screen_hash();

            if (h != last_hash) {
                last_hash   = h;
                last_change = t;
            } else if ((h != 0) && ((t - last_change) > 4000)) {
                kbc_siemens_key(pcx.kbc, 0x0d);
                last_change = t;
                pclog("[AUTO] idle screen, CR at %u\n", t);
            }

            next = t + 500;
        }
    }

    timer_on_auto(&pcx.rtc_timer, 1000000.0);
}

static uint8_t
pcx_rtc_read(uint16_t port, UNUSED(void *priv))
{
    uint8_t reg = (uint8_t) (port - PCX_RTC_PORT);

    if (reg >= PCX_RTC_SIZE)
        return 0xff;

    /* Register 0x0a carries the divider/update-in-progress bits; the POST
       checks bits 4-6 == 0b010 there (v3 0xfcd80) and polls bit 7 while it
       reads the 50 battery-backed bytes above it. */
    if (reg == 0x0a)
        return (uint8_t) ((pcx.rtc[0x0a] & 0x8f) | 0x20);

    if (reg == 0x0c) {
        uint8_t v = pcx.rtc[0x0c];

        pcx.rtc[0x0c] = 0x00;
        picintc(1 << 7);
        return v;
    }

    return pcx.rtc[reg];
}

static void
pcx_rtc_write(uint16_t port, uint8_t val, UNUSED(void *priv))
{
    uint8_t reg = (uint8_t) (port - PCX_RTC_PORT);

    if (reg >= PCX_RTC_SIZE)
        return;

    if (((reg == 0x0a) || (reg == 0x0b)) && (getenv("PCX_RTCW") != NULL))
        pclog("[RTCW] reg=%02X val=%02X @%04X:%04X\n", reg, val, CS, cpu_state.pc);

    pcx.rtc[reg] = val;
}

/* Placeholders: return open bus so the POST at least does not hang on them.
   Each of these is a separate item in docs/86box/pcx-machine-plan. */
static uint8_t
pcx_stub_readb(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return 0xff;
}

/* The POST walks 0xf88e-0xf8bf, writing each port's own address byte and
   reading it back (v3 0xfcdcc); the board answers with the port number, so a
   port that falls through to an open bus fails the walk and sets
   CRT-CONTR. DEFECT (bit 5 of 0x411). */
static uint8_t
pcx_vid_id_read(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return (uint8_t) port;
}

static uint16_t
pcx_stub_readw(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return 0xffff;
}


/* Qt hands the machine set 1 scan codes; the PC-X keyboard is not a PC
   keyboard at all.  It sits on the terminal board's serial line and the 8031
   forwards what it receives to the host as bytes, which SINIX's console
   reads at 0xf980 -- printable keys arrive as their ASCII code, Return as
   CR.  So the scan code is turned back into the character the terminal would
   have sent; break codes are dropped because the board sends a single byte
   per key. */
static void
pcx_keyboard_send(uint16_t val)
{
    static const char plain[0x40] = {
        /* 00 */ 0, 0, '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', '\b', '\t',
        /* 10 */ 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', '\r', 0,
        /* 20 */ 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', '`', 0, '\\',
        /* 30 */ 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 0, 0, 0, ' '
    };
    static const char shifted[0x40] = {
        /* 00 */ 0, 0, '!', '@', '#', '$', '%', '^', '&', '*', '(', ')', '_', '+', '\b', '\t',
        /* 10 */ 'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P', '{', '}', '\r', 0,
        /* 20 */ 'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', ':', '"', '~', 0, '|',
        /* 30 */ 'Z', 'X', 'C', 'V', 'B', 'N', 'M', '<', '>', '?', 0, 0, 0, ' '
    };
    uint8_t ch;

    if ((pcx.kbc == NULL) || (val >= 0x100) || (val >= 0x40))
        return;

    /* Bit 7 marks a break code. */
    if (val & 0x80)
        return;

    ch = (keyboard_get_shift() ? shifted : plain)[val];
    if (ch != 0)
        kbc_siemens_key(pcx.kbc, ch);
}

static void
pcx_stub_writeb(UNUSED(uint16_t port), UNUSED(uint8_t val), UNUSED(void *priv))
{
}

/* ------------------------------------------------------------------------- */
/* Interrupts                                                                */
/* ------------------------------------------------------------------------- */

/* The chip's cascade ack puts the vector of whichever 8259 owns the request on
   the bus; both are wired to separate 80186 interrupt inputs, not cascaded. */
static int
pcx_8259_ack(void *priv)
{
    return pic_irq_ack_instance((pic_t *) priv);
}

/* The three EPCI chips drive the first 8259.  INTR is a level line, so it has
   to be withdrawn explicitly when the chip drops it; the IRQ number travels as
   the callback's private pointer. */
static void
pcx_usart_irq(void *priv, int level)
{
    int irq = (int) (intptr_t) priv;

    if (getenv("PCX_IRQLOG") != NULL) {
        static int il = 0;
        if (il < 200) {
            pclog("[IRQ] t=%u irq=%d level=%d\n", plat_get_ticks(), irq, level);
            il++;
        }
    }

    if (level)
        picint(1 << irq);
    else
        picintc(1 << irq);
}

static void
pcx_icu_int0(void *priv, int level)
{
    i8018x_pcb_icu_int((struct i8018x_pcb_t *) priv, 0, level);
}

static void
pcx_icu_int1(void *priv, int level)
{
    i8018x_pcb_icu_int((struct i8018x_pcb_t *) priv, 1, level);
}

/* ------------------------------------------------------------------------- */
/* Machine                                                                   */
/* ------------------------------------------------------------------------- */

/* The PC-D and the PC-X are two machines built on the same mainboard: the
   same BIOS, the same 80186, the same WD2791.  What differs is the equipment
   around it -- the video/keyboard board and whether the MMU card is fitted --
   so the two get their own init and must never share a video or keyboard
   model.  See vid_pcx.c (8031 terminal board) vs vid_pcd.c (SCN2674). */
/* Temporary probe: does the guest programme the 80186 internal peripherals
   (timer / interrupt controller) which our CPU core does not model? */
static int pcx_i186_log_on = -1;

static void
pcx_i186_logw(uint16_t addr, uint8_t val, void *priv)
{
    if (pcx_i186_log_on < 0)
        pcx_i186_log_on = (getenv("PCX_I186") != NULL);
    if (pcx_i186_log_on)
        pclog("[I186] w %04X=%02X @%04X:%04X\n", addr, val, CS, cpu_state.pc);
}

static uint8_t
pcx_i186_logr(uint16_t addr, void *priv)
{
    if (pcx_i186_log_on < 0)
        pcx_i186_log_on = (getenv("PCX_I186") != NULL);
    if (pcx_i186_log_on)
        pclog("[I186] r %04X @%04X:%04X\n", addr, CS, cpu_state.pc);
    return 0xff;
}


static uint8_t
pcx_i186_logr16(uint16_t addr, void *priv)
{
    if (pcx_i186_log_on < 0)
        pcx_i186_log_on = (getenv("PCX_I186") != NULL);
    if (pcx_i186_log_on)
        pclog("[I186] r16 %04X @%04X:%04X\n", addr, CS, cpu_state.pc);
    return 0xffff;
}

static void
pcx_i186_logw16(uint16_t addr, uint16_t val, void *priv)
{
    if (pcx_i186_log_on < 0)
        pcx_i186_log_on = (getenv("PCX_I186") != NULL);
    if (pcx_i186_log_on)
        pclog("[I186] w16 %04X=%04X @%04X:%04X\n", addr, val, CS, cpu_state.pc);
}

static int machine_pcx_init_ex(const machine_t *model);

int
machine_pcx_init(const machine_t *model)
{
    pcx_is_pcd = 0;

    return machine_pcx_init_ex(model);
}

int
machine_pcd_init(const machine_t *model)
{
    pcx_is_pcd = 1;

    return machine_pcx_init_ex(model);
}

static int
machine_pcx_init_ex(const machine_t *model)
{
    int ret;

    /* 16 KB BIOS, assembled from the two 8 KB halves of the MAME dump.  The
       two machines keep their own set: the ROM is the same dump today, but
       keeping the paths apart means a machine-specific build can be dropped
       into its own directory without touching the other one. */
    ret = bios_load_interleaved(pcx_is_pcd ? PCD_BIOS_LOW : PCX_BIOS_LOW,
                                pcx_is_pcd ? PCD_BIOS_HIGH : PCX_BIOS_HIGH,
                                PCX_BIOS_ADDR, PCX_BIOS_SIZE, 0);

    if (bios_only || !ret)
        return ret;

    pcx.stat   = 0x00;
    pcx.dskctl = 0x00;
    pcx.fdc    = NULL;
    /* 0xf841 read = S9.  Bit 0 is "no MMU" (set on the PC-D, clear on the
       PC-X); bits 4/5 ask the POST to drop into the monitor after/before POST,
       bit 6 skips the early RAM clear.  MAME hardcodes 0x01 because it models
       the PC-D.  Verified against the v3 BIOS: 0x3c reads it, tests 0x60. */
    /* 0xf841 reads the S9 block.  Bit 0 is the one the POST looks at right after
       the memory test: with it clear the ROM installs a keyboard-failure vector
       and never reaches the rest of the self test (v3 BIOS 0xfcab4-0xfcac6).
       MAME's pcd_v sets 0x01 here as well; 0x10/0x20 would drop into a monitor. */
    /* 0xf841 bit 0 is the strap that tells the POST no MMU card is fitted.  It
       stays set even when the "MMU Type" option fits one: with the strap clear
       the POST runs its own MMU self test, which our model only satisfies up to
       test four -- the I/O trap raises NMI but the handler the test installs
       never runs -- so the machine would not come up at all. */
    /* PCX_DIP (hex) overrides the strap byte for bench tests; the default is
       the value the POST needs to come up. */
    {
        const char *dip_env = getenv("PCX_DIP");

        pcx.dip = (dip_env != NULL) ? (uint8_t) strtoul(dip_env, NULL, 16) : 0x01;
    }
    pcx.led  = 0x00;
    pcx.dskctl2 = 0x00;

    /* MAME's pcd/pcx driver uses nvram_device::DEFAULT_ALL_1, i.e. an erased
       battery reads 0xFF.  All-zero makes the boot loader's config scan take a
       different (and fatal) path. */
    memset(pcx.nvram, 0xff, sizeof(pcx.nvram));
    memset(pcx.rtc, 0x00, sizeof(pcx.rtc));
    pcx.rtc[0x0a] = 0x20; /* divider running, no update in progress */

    timer_add(&pcx.rtc_timer, pcx_rtc_tick, NULL, 0);
    pcx_rtc_tick(NULL); /* load the host clock, and arm the one-second tick */ /* divider/status: bits 4-6 = 0b010, no update in progress */
    pcx.vid_data   = 0x00;
    pcx.vid_ctrl   = 0x00;
    pcx.vid_exec   = 0x00;
    pcx.err_seg = 0x00;

    /* Low memory ends at 0x80000.  The 32 KB at 0xf0000 belong to the graphics
       board, so they are left out of both the RAM and the hole windows and the
       video device maps them itself (vid_pcx.c).  MEM_MAPPING_CPU_ONLY on the
       holes keeps the on-chip DMA from raising NMIs through mem_readb_phys()
       when it walks into one. */
    {
        uint32_t ram_top   = (uint32_t) mem_size << 10;
        uint32_t ram_limit = (ram_top < PCX_RAM_CEIL) ? ram_top : PCX_RAM_CEIL;

        /* MAME's pcd_map decodes 0x00000-0x7FFFF, but its offsets are 16-bit
           words (the 80186 bus), so that range covers the whole 1 MB and the
           guest sees all of it: SINIX reports 810 KB free here, 830 KB under
           MAME, and only 352 KB when the map stopped at 512 KB. */

        /* The 640K..1M window is decoded by this machine, not by the generic
           RAM mapping: its state has to allow internal mappings or nothing
           placed here (the holes, the video window) is ever consulted. */
        mem_set_mem_state_both(0x000a0000, PCX_GRAPH_TOP - 0x000a0000,
                               MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);

        /* Above 640 KB the RAM has to be mapped here as well: the generic
           mapping stops at A0000h, and without this the BIOS sizes anything
           fitted past 640 KB as absent (same as m_tandy2k.c). */
        if (ram_limit > 0x000a0000) {
            uint32_t size = ram_limit - 0x000a0000;

            mem_mapping_add(&pcx.ram_map, 0x000a0000, size,
                            mem_read_ram, mem_read_ramw, mem_read_raml,
                            mem_write_ram, mem_write_ramw, mem_write_raml,
                            NULL, MEM_MAPPING_INTERNAL, NULL);
        }

        /* RAM above the fitted size, up to the BIOS.  Its state has to
           be opened here too: the POST sizes the memory by writing to each
           64 KB block and waiting for the bus timeout, so a block that is not
           marked internal swallows the write instead of raising the NMI, the
           walk runs on to A0000h and every board reports 640 KB. */
        if (ram_limit < PCX_GRAPH_TOP) {
            mem_set_mem_state_both(ram_limit, PCX_GRAPH_TOP - ram_limit,
                                   MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);
            mem_mapping_add(&pcx.hole_map, ram_limit, PCX_GRAPH_TOP - ram_limit,
                            pcx_nmi_readb, pcx_nmi_readw, pcx_nmi_readl,
                            pcx_nmi_writeb, pcx_nmi_writew, pcx_nmi_writel,
                            NULL, MEM_MAPPING_INTERNAL | MEM_MAPPING_CPU_ONLY, NULL);
        }

    }

    /* Two 8259s on separate 80186 interrupt inputs (pic.c helper). */
    pic_init_pcx();

    {
        struct i8018x_pcb_t *pcb = device_add(&i8018x_pcb_device);

        i8018x_pcb_set_dma_channels(pcb, 0, 1); /* DRQ0: SCSI, DRQ1: WD2791 */
        /* TIMERIN0 is a board signal, not the internal clock; see
           PCX_TMRIN_CLOCKS for how the divider was derived. */
        i8018x_pcb_set_timer_input(pcb, 0, PCX_TMRIN_CLOCKS);
        i8018x_pcb_set_ext_ack(pcb, 0, pcx_8259_ack, &pic);
        i8018x_pcb_set_ext_ack(pcb, 1, pcx_8259_ack, &pic2);
        pic_set_output_callback(&pic, pcx_icu_int0, pcb);
        pic_set_output_callback(&pic2, pcx_icu_int1, pcb);
    }

    /* Undecoded I/O: a cycle nothing answers times out and raises the NMI.  Only
       the port the POST's own time-out test reads is claimed here (v3 BIOS
       0xfc911: `in 0xfbbe` in a loop, leaving it only when the NMI handler has
       run).  A blanket handler over the whole hole cannot be used: the POST
       scans 0xfb00-0xffff for devices before it has put a vector in IVT 8
       (0xfc405-0xfc433, vector written at 0xfc458), so every one of those reads
       would raise an NMI nobody is listening for and kill the boot. */
    /* The POST's device scan (v2 0xfc38c, v3 0xfc405) reads 0xfb00-0xfb0f and
       treats a port whose high byte reads zero as absent, setting one bit per
       absent port in the equipment word at 0x402.  Undecoded 86Box I/O answers
       0xff, which makes every port look present and leaves 0x402 at zero; the
       PC-D DOS loader refuses to boot unless bit 1 of that word is set, so the
       scan window answers zero here, the way an undriven bus does on the card. */
    io_sethandler(0xfb01, 0x0001, pcx_scan_read8, NULL, NULL, NULL, NULL, NULL,
                  NULL);
    io_sethandler(0xfb00, 0x0001, pcx_io_hole_read8, NULL, NULL, NULL, NULL, NULL,
                  NULL);
    io_sethandler(0xfb02, 0x000e, pcx_io_hole_read8, NULL, NULL, NULL, NULL, NULL,
                  NULL);
    io_sethandler(0xfbbe, 0x0001, pcx_io_hole_read8, pcx_io_hole_read16, NULL,
                  pcx_io_hole_write8, pcx_io_hole_write16, NULL, NULL);

    /* Probe: 80186 on-chip peripherals.  Only install it when asked for -- the
       handlers above shadow the PCB's own DMA block (0xFF00-0xFF1F) and would
       swallow every DMA register write the driver makes. */
    if (getenv("PCX_I186") != NULL)
        io_sethandler(0xff00, 0x0020, pcx_i186_logr, pcx_i186_logr16, NULL,
                      pcx_i186_logw, pcx_i186_logw16, NULL, NULL);

    io_sethandler(PCX_STAT_PORT, 0x0001, pcx_stat_read, NULL, NULL,
                  pcx_stat_write, NULL, NULL, NULL);
    io_sethandler(PCX_DIP_PORT, 0x0001, pcx_dip_read, NULL, NULL,
                  pcx_dip_write, NULL, NULL, NULL);
    io_sethandler(PCX_FAULTSEG_PORT, 0x0001, pcx_faultseg_read, NULL, NULL,
                  NULL, NULL, NULL, NULL);
    io_sethandler(PCX_DSKCTL_PORT, 0x0001, pcx_dskctl_read, NULL, NULL,
                  pcx_dskctl_write, NULL, NULL, NULL);
    io_sethandler(PCX_DSKCTL2_PORT, 0x0001, pcx_dskctl2_read, NULL, NULL,
                  pcx_dskctl2_write, NULL, NULL, NULL);
    io_sethandler(PCX_NVRAM_PORT, PCX_NVRAM_SIZE, pcx_nvram_read, NULL, NULL,
                  pcx_nvram_write, NULL, NULL, NULL);
    io_sethandler(PCX_RTC_PORT, PCX_RTC_SIZE, pcx_rtc_read, NULL, NULL,
                  pcx_rtc_write, NULL, NULL, NULL);
    /* PC-X has no 8237: the 80186's on-chip DMA is programmed through the PCB
       registers at 0xff50+.  dma_init() still installs the PC's 8237 handlers
       on 0x0000-0x000f and the page registers at 0x0080-0x0087 for every
       machine, and the POST's I/O walk (v3 0xfcd3a) writes a pattern to every
       port -- on a real PC-X those two windows are empty, here they would arm
       DMA channels and scribble over memory, so take them back.  Same idea as
       m_tandy2k.c:1129, which shadows 0x0000-0x000f with its own latch. */
    io_sethandler(0x0000, 0x0010, pcx_stub_readb, NULL, NULL,
                  pcx_stub_writeb, NULL, NULL, NULL);
    io_sethandler(0x0080, 0x0008, pcx_stub_readb, NULL, NULL,
                  pcx_stub_writeb, NULL, NULL, NULL);
    /* 0xff50-0xff5f is empty on the PC-X as well.  The POST spins on bit 5 of
       0xff56 (v3 0xfcc7b) and an open bus answers 0xff, which ends the wait at
       once; whatever answered 0x04 here left it looping for ever. */
    io_sethandler(0xf88e, 0x0032, pcx_vid_id_read, NULL, NULL,
                  pcx_stub_writeb, NULL, NULL, NULL);
    io_sethandler(0xff50, 0x0010, pcx_stub_readb, pcx_stub_readw, NULL,
                  pcx_stub_writeb, NULL, NULL, NULL);

    /* The video board owns 0xf980-0xf98f itself; registering it here as well
       would shadow the device (io.c: later registration wins). */
    void *video = device_add(pcx_is_pcd ? &pcd_video_device : &pcx_video_device);

    /* The keyboard and the terminal share one channel on the video board; the
       controller answers what the POST asks it and will carry key strokes.
       The PC-D's board has no terminal, so its 8741 is the only keyboard
       interface and there is nobody to hand the controller to. */
    pcx.kbc = kbc_siemens_init();
    if (!pcx_is_pcd) {
        pcx_video_set_kbc(video, pcx.kbc);
        keyboard_send = pcx_keyboard_send;
    }
    /* The 8741A keyboard interface sits next to the terminal channel
       (0xf9b0-0xf9b3); the controller registers those ports itself. */
    kbc_siemens_ports(pcx.kbc);

    /* WD2791 formatter: registers at 0xf900-0xf903, INTRQ on IR6, DRQ on the
       80186 DMA channel 1.  The 0xf904 latch above feeds it drive/side/DDEN. */
    pcx.fdc = wd2791_init();
    /* The Qt front end only builds the floppy indicators (and their media
       menus) when the standard controller has announced itself.  This machine
       brings its own WD2791 instead, so say so here. */
    fdcinited = 1;

    /* Not implemented yet -- see the TODO list below. */

    /* PCX_NOSCSI=1 takes the adapter out of the machine entirely, so the boot
       chain can be observed with no SCSI target to answer on 0xf940. */
    if (getenv("PCX_NOSCSI") == NULL)
        pcx.scsi = scsi_sm912_init(PCX_SCSI_PORT, pcx_usart_irq, (void *) (intptr_t) 5);

    /* The three USARTs are SCN2661B parts, all fed from one 4.9152 MHz crystal,
       which is the B rate table.  The shared BIOS names them after their PC-D
       wiring -- PRINTER (0xf9c0), KEYBORD (0xf9d0), RESERVE (0xf9e0) -- but the
       PC-X only brings two of them out: the printer line and the reserve line,
       the one the kernel polls as its console.  The second part is fitted and
       answers the POST's loopback, yet it reaches no connector: it is the line
       kept for a modem and, on the PC-D, the line the keyboard hangs off.  So
       only the first and third parts claim a COM port here; the middle one is
       handed SERIAL_PORT_NONE and stays invisible to the host.  The chip
       decodes no address of its own: the four consecutive ports are registered
       below and A1:A0 picks the register.  INTR is a level line on the system
       PIC, IR3 / IR2 / IR4 for the first / second / third USART. */
    serial_set_next_inst(SERIAL_MAX - 1);

    pcx.usart[0] = device_add_inst_params(&scn2661b_device, 1, (void *) (intptr_t) 0);              /* printer -> COM1 */
    scn26x1_set_irq(pcx.usart[0], 1 << 3);
    io_sethandler(PCX_USART1_PORT, 0x0004, scn26x1_read, NULL, NULL,
                  scn26x1_write, NULL, NULL, pcx.usart[0]);

    pcx.usart[1] = device_add_inst_params(&scn2661b_device, 2, (void *) (intptr_t) SERIAL_PORT_NONE); /* no connector */
    scn26x1_set_irq(pcx.usart[1], 1 << 2);
    keyboard_set_input_handler(pcx_kbd_host, NULL);
    /* Both machines reach the keyboard through the second USART: the PC-D's
       Graphik-BIOS (loaded into RAM at 0x7000) sends its presence query from
       segment 0x0070, the PC-X terminal does it from its own board.  The 8741
       on the PC-D video board is not on this path -- its mailbox at 0xf9b0 is
       never touched by that BIOS. */
    io_sethandler(PCX_USART2_PORT, 0x0004, pcx_usart2_read, NULL, NULL,
                  pcx_kbd_usart_write, NULL, NULL, pcx.usart[1]);

    pcx.usart[2] = device_add_inst_params(&scn2661b_device, 3, (void *) (intptr_t) 1);              /* reserve -> COM2 */
    scn26x1_set_irq(pcx.usart[2], 1 << 4);
    io_sethandler(PCX_USART3_PORT, 0x0004, scn26x1_read, NULL, NULL,
                  scn26x1_write, NULL, NULL, pcx.usart[2]);

    /* The MMU sits on port 0x8000-0x8fff and translates every CPU access to the
       RAM window in 2 KB pages; it is also the place that catches the hole when
       the board layout later maps more RAM than the configuration has.  Which
       width the task / bank field of the control register has is a property of
       the board rather than of the software: the "MMU Type" option stands in for
       the X8 jumper (1-2 = SINIX before 1.2A, 2-3 = 1.2A and later), and "None"
       fits no card at all, which is what the PC-D runs with. */
    pcx_nmi_enable();
    pcx_mmu_nmi_status = pcx_nmi_status_set;

    pcx_mmu_init(machine_get_config_int((char *) "mmu"));
    pcx_mmu_set_hole(0, 0); /* the mapping above already reports the hole */

    return ret;
}

/* A machine reset clears the translation table: the part comes out of reset
   with translation off and an empty table, and a driver that turns it on and
   then resets the machine expects the next POST to run untranslated. */
static void
pcx_reset(void *priv)
{
    (void) priv;

    pcx_mmu_reset();
}

/* Machine level options, the way the IBM 5550 exposes its font and model: the
   MMU Type stands in for the X8 jumper of the board (and for MAME's input port
   of the same name), because SINIX 1.0 and 1.2 split the page table entries
   differently and nothing in software tells the card which one it is. */
static const device_config_t pcx_config[] = {
    // clang-format off
    {
        .name        = "mmu",
        .description = "MMU Type",
        .type        = CONFIG_SELECTION,
        .default_int = PCX_MMU_NONE,
        .selection   = {
            {
                .description = "None",
                .value = PCX_MMU_NONE
            },
            {
                .description = "SINIX 1.0",
                .value = PCX_MMU_SINIX10
            },
            {
                .description = "SINIX 1.2",
                .value = PCX_MMU_SINIX12
            },
            { .description = "" }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t pcx_device = {
    .name          = "Siemens PC-X",
    .internal_name = "pcx",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = pcx_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = pcx_config
};


/* The PC-D has no MMU card by default: the strap on S9 bit 0 tells the POST
   so, and DOS does not need one.  The option stays available for SINIX. */
static const device_config_t pcd_config[] = {
    // clang-format off
    {
        .name        = "mmu",
        .description = "MMU Type",
        .type        = CONFIG_SELECTION,
        .default_int = PCX_MMU_NONE,
        .selection   = {
            { .description = "None",      .value = PCX_MMU_NONE    },
            { .description = "SINIX 1.0", .value = PCX_MMU_SINIX10 },
            { .description = "SINIX 1.2", .value = PCX_MMU_SINIX12 },
            { .description = "" }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t pcd_device = {
    .name          = "Siemens PC-D",
    .internal_name = "pcd",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = pcx_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = pcd_config
};

/*
 * TODO, in dependency order (each item is its own step in the plan):
 *   1. src/pic.c: pic_init_pcx() -- two 8259s at 0xf800 / 0xf820, no cascade
 *      (copy pic_init_tandy2k()).
 *   2. src/floppy/wd2793.c + the fdd_drive_t controller callbacks: WD2791 at
 *      0xf900, INTRQ on the master's IR6, DRQ on the 80186's DRQ1.
 *   3. SCSI port at 0xf940 (SM912 + OMTI 5100 for ST506 drives), IRQ5.
 *   4. Three SCN2661B at 0xf9c0 / 0xf9d0 / 0xf9e0, IRQ3 / IRQ2 / IRQ4.
 *   5. SCN2672 + on-board 8031 video board: needs an MCS-51 core, or start with
 *      the reduced "static picture" model described in plan section 4.
 *   6. MC146818 window at 0xf880, 2 KB NVRAM at 0xf000-0xf7ff.
 *   7. Keyboard: the board talks to an 8035 over a serial line from the video
 *      board, so it lands after item 5.
 */
