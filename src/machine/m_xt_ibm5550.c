/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Emulation of the IBM Multistation 5550 and 5535-M.
 * 
 *          The IBM 5550 was launched with three models:
 *            5551-Axx: 12" monochrome CRT with 16x16 font
 *                       (replaced by model D)
 *            5551-Bxx: 15" monochrome CRT with 24x24 font
 *                       (replaced by model G, J, M)
 *            5551-Cxx: 14" color CRT with 16x16 font
 *                       (replaced by model E, H, K, P)
 *          These first-gen models have 1-3 DSQD diskette drives.
 *          You need select "Type: 5.25" 720k" in the Settings dialog - Floppy & CD-ROM drives.
 * 
 *          Currently, this module supports model A and B configurations without hard disk.
 * 
 * Authors: Akamaki
 *          WNT50
 *
 *          Copyright 2026 Akamaki.
 *          Copyright 2026 WNT50.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdarg.h>
#include <wchar.h>
#define HAVE_STDARG_H
extern uint8_t *rom; /* [FIX] the ROM buffer bios_load_linear filled */
#include <86box/config.h>
#include <86box/ini.h>
#include <86box/86box.h>
#include "cpu.h"
#include <86box/timer.h>
#include <86box/io.h>
#include <86box/dma.h>
#include <86box/pic.h>
#include <86box/ppi.h>
#include <86box/nmi.h>
#include <86box/mem.h>
#include <86box/log.h>
#include <86box/i80186.h>
#include <86box/device.h>
#include <86box/lpt.h>
#include <86box/nvr.h>
#include <86box/keyboard.h>
#include <86box/rom.h>
#include <86box/fdd.h>
#include <86box/fdc.h>
#include <86box/fdc_ext.h>
#include <86box/serial.h>
#include <86box/snd_speaker.h>
#include <86box/video.h>
#include <86box/machine.h>
#include <86box/plat_unused.h>
#define EMU_DEVICE_H
#include <86box/pit.h>
#include <86box/mouse.h>

#define EPOCH_VRAM_SBCS        0x38000
#define EPOCH_VRAM_SBEX        0x30000
#define EPOCH_INVALIDACCESS8       0xffu
#define EPOCH_INVALIDACCESS16      0xffffu
#define EPOCH_INVALIDACCESS32      0xffffffffu
#define EPOCH_SIZE_VRAM            (256 * 1024) /* 0x40000 */
#define EPOCH_SIZE_CRAM            (4 * 1024)    /* 0x1000 */
#define EPOCH_MASK_A000            0x1ffff       /* 0x1FFFF */
#define EPOCH_MASK_CRAM            0xfff         /* 0xFFF */
/* The 3477-S picks one of four 4 KB planes of the display RAM into the 0xE0000
   window through write-only port 0x400 (its burn-in writes 3,2,1,0 there and
   runs the same pattern test on each). [Cloud server only, DO NOT USE!!!] */
#define EPOCH_CRAM_PLANES          4
#define EPOCH_SIZE_CRAM_ALL        (EPOCH_SIZE_CRAM * EPOCH_CRAM_PLANES)
#define EPOCH_MASK_VRAM            0x3ffff       /* 0xFFFFF */
// #define EPOCH_MASK_VRAMPLANE       0x1ffff       /* 0x1FFFF */
#define EPOCH_PIXELCLOCK24         40000000.0    /* 40 MHz interlaced */
#define EPOCH_PIXELCLOCK16         20000000.0    /* 20 MHz interlaced (not confirmed) */
#define EPOCH_CONFIG_MONO16 0 /* Model 5551-Axx (Font 16, monochrome) */
#define EPOCH_CONFIG_MONO24 1 /* Model 5551-Bxx (Font 24, monochrome) */

/* Optional kanji font card: a page ROM behind a 32KB or 48KB RAM window
   at F0000h. Only installed when its ROM image is present, so a machine
   without the card behaves exactly as before. */
#define EPOCH_FONTCARD_WIN_ADDR 0xf0000
#define EPOCH_FONTCARD_WIN_SIZE 0x8000
#define EPOCH_FONTCARD_ROM_OLD  0 /* Old font card ROM revision (94X1372) */
#define EPOCH_FONTCARD_ROM_NEW  1 /* New font card ROM revision (65X1460) */
#define EPOCH_FONTCARD_ROM_SIZE (512 * 1024) /* Font card ROM size (512K) */
#define EPOCH_FONTCARD_ROM_SIZE_3477 (1024 * 1024) /* 3477-S ROS FONT ROM size (1M, 32 x 32K pages) */
#define EPOCH_FONTCARD_ROM_PATH_OLD "roms/machines/ibm5535/94X1372.BIN"
#define EPOCH_FONTCARD_ROM_PATH_NEW "roms/machines/ibm5535/65X1460.BIN"
#define EPOCH_FONTCARD_ROM_PATH_K16 "roms/machines/ibm5550/FONTK16.BIN"
#define EPOCH_FONTCARD_ROM_PATH_K24 "roms/machines/ibm5550/FONTK24.BIN"
#define EPOCH_FONTCARD_ROM_PATH_K52 "roms/machines/ibm5550/FONTK52.BIN"
#define EPOCH_FONTCARD_ROM_PATH_P24 "roms/machines/ibm5550/FONTP24.BIN"
#define EPOCH_FONTCARD_ROM_PATH_3477 "roms/machines/ibm3477/font.bin" /* 3477-S built-in ROS FONT ROM */

/* Korean (Hangul) and Traditional Chinese version of Multistation DOS are not dumped yet,
   so font image cannot be generated. They would have ID 4 and 8 for codepages 934 and 938. */
#define EPOCH_FONTCARD_CHARSET_NONE 0 /* Software font only, no font card will be available */
#define EPOCH_FONTCARD_CHARSET_KJ16 1 /* Code page 932 Japanese, Font 16 ($SYS1Z16.FNT) */
#define EPOCH_FONTCARD_CHARSET_KJ24 2 /* Code page 932 Japanese, Font 24 ($SYS1Z24.FNT) */
#define EPOCH_FONTCARD_CHARSET_KJ52 3 /* Code page 932 Japanese, 5250 Kanji emulation */
#define EPOCH_FONTCARD_CHARSET_HANS 6 /* Code page 936 Simplified Chinese, Font 24 */

#define LC_INDEX                0x3D0
#define LC_DATA                 0x3D1
#define LC_HORIZONTAL_TOTAL     0x00 /* -1 */
#define LC_HORIZONTAL_DISPLAYED 0x01
#define LC_H_SYNC_POSITION      0x02
#define LC_SYNC_WIDTH           0x03
#define LC_VERTICAL_TOTAL       0x04 /* -1 */
#define LC_V_TOTAL_ADJUST       0x05
#define LC_VERTICAL_DISPLAYED   0x06
#define LC_V_SYNC_POSITION      0x07
#define LC_INTERLACE_AND_SKEW   0x08
#define LC_MAXIMUM_SCAN_LINE    0x09 /* -1 */
#define LC_CURSOR_ROW_START     0x0A
#define LC_CURSOR_ROW_END       0x0B
#define LC_START_ADDRESS_HIGH   0x0C
#define LC_START_ADDRESS_LOW    0x0D
#define LC_CURSOR_LOC_HIGH      0x0E
#define LC_CURSOR_LOC_LOWJ      0x0F
#define LC_LIGHT_PEN_HIGH       0x10
#define LC_LIGHT_PEN_LOW        0x11
#define LS_ENABLE               0x3D2
#define LS_DISABLE              0x3D3
#define LS_MODE                 0x3D8
#define LS_MONSENSE             0x3DA
// #define LV_PORT                 0x3E8
// #define LV_PALETTE_0            0x00
// #define LV_MODE_CONTROL         0x10
// #define LV_OVERSCAN_COLOR       0x11
// #define LV_COLOR_PLANE_ENAB     0x12
// #define LV_PANNING              0x13
// #define LV_VIEWPORT1_BG         0x14
// #define LV_VIEWPORT2_BG         0x15
// #define LV_VIEWPORT3_BG         0x16
// #define LV_BLINK_COLOR          0x17
// #define LV_BLINK_CODE           0x18
// #define LV_GR_CURSOR_ROTATION   0x19
// #define LV_GR_CURSOR_COLOR      0x1A
// #define LV_GR_CURSOR_CONTROL    0x1B
// #define LV_COMMAND              0x1C
// #define LV_VP_BORDER_LINE       0x1D
// #define LV_SYNC_POLARITY        0x1F
// #define LV_CURSOR_CODE_0        0x20
// #define LV_GRID_COLOR_0         0x34
// #define LV_GRID_COLOR_1         0x35
// #define LV_GRID_COLOR_2         0x36
// #define LV_GRID_COLOR_3         0x37
// #define LV_ATTRIBUTE_CNTL       0x38
// #define LV_CURSOR_COLOR         0x3A
// #define LV_CURSOR_CONTROL       0x3B
// #define LV_RAS_STATUS_VIDEO     0x3C
// #define LV_PAS_STATUS_CNTRL     0x3D
// #define LV_IDENTIFICATION       0x3E
// #define LV_OUTPUT               0x3E
// #define LV_COMPATIBILITY        0x3F

#define TIMER_CTR_0 0 /* DMA */
#define TIMER_CTR_1 1 /* PIT */
#define TIMER_CTR_2 2 /* Speaker */

#define EPOCH_IRQ3_BIT (1 << 3) /* Keyboard */
#define EPOCH_IRQ6_BIT (1 << 6) /* PIT */
#define EPOCH_IRQ7_BIT (1 << 7) /* Software timer */

/* The 5535-M's IPL restarts POST at a given stage by writing the stage number
   into the IIP diagnostic register file (index 0x10 of 0x368/0x36A) and then
   writing 0x4F to port 0xA1; the board pulses the processor's reset and the
   reset entry reads the stage back. The pulse is delayed by this much so
   the instructions the IPL puts between the two writes still run. */
#define EPOCH_RESET_PULSE 25 /* microseconds */

/* Software timer (port 0xA6) timings, in microseconds.
   Writing 1 arms a free-running timer: first raises IRQ7 after a short delay
   (long enough for the ROM to have written the IMR), then keeps raising IRQ7
   periodically. Writing 0 from the IRQ handler acknowledges (keeps the timer
   running); writing 0 from normal code (FE67:0191 DMA-test cleanup) disarms it.
   The ROM BAT checks five IRQ7 interrupts arrive while its handler is running. */
#define EPOCH_SWTIMER_FIRST  10    /* first IRQ7 delay after arming */
#define EPOCH_SWTIMER_PERIOD 20000 /* 20 ms free-running period */

/* Set by machine_xt_ibm5535_init() before the system unit is added: the IBM
   5535-M is the same platform with a 286 and a different IPL, and it moves
   the software timer's arm/ack from port 0xA6 to bit 0 of port 0xA0. */
static int epoch_is_5535 = 0;
int epoch_is_3477 = 0; /* [AB] non-static: the FDC model needs it */

enum epoch_nvr_ADDR {
    epoch_nvr_SECOND1,
    epoch_nvr_SECOND10,
    epoch_nvr_MINUTE1,
    epoch_nvr_MINUTE10,
    epoch_nvr_HOUR1,
    epoch_nvr_HOUR10,
    epoch_nvr_WEEKDAY,
    epoch_nvr_DAY1,
    epoch_nvr_DAY10,
    epoch_nvr_MONTH1,
    epoch_nvr_MONTH10,
    epoch_nvr_YEAR1,
    epoch_nvr_YEAR10,
    epoch_nvr_UNKOWN_D,
    epoch_nvr_UNKOWN_E,
    epoch_nvr_UNKOWN_F,
    epoch_nvr_CONTROL /* Internal data for configuration port (I/O 360h) */
};

#ifndef RELEASE_BUILD
// #define ENABLE_EPOCH_LOG 1
#endif

#ifdef ENABLE_EPOCH_LOG
// #define ENABLE_EPOCH_DEBUGIO 1
#define ENABLE_EPOCH_DEBUGKBD 1
int epoch_do_log = ENABLE_EPOCH_LOG;

static void
epoch_log(const char *fmt, ...)
{
    va_list ap;

    if (epoch_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define epoch_log(fmt, ...)
#endif
#ifdef ENABLE_EPOCH_DEBUGIO
#    define epoch_iolog epoch_log
#else
#    define epoch_iolog(fmt, ...)
#endif
#ifdef ENABLE_EPOCH_DEBUGKBD
#    define epoch_kbdlog epoch_log
#else
#    define epoch_kbdlog(fmt, ...)
#endif


typedef struct epoch_t {
    mem_mapping_t cmapping;

    /* The controller exposes a full 256-entry register file: the 3477-S ROM
       write/read-back tests indices all the way up to 0xE5. */
    uint16_t crtc[256];
    // uint16_t crtc_vpreg[128];
    // uint8_t  crtc_vpsel;
    uint8_t  crtmode;
    // uint8_t  attrc[0x40];
    // int      attraddr, attrff;
    // int      attr_palette_enable;
    int      outflipflop;
    int      inflipflop;
    int      iolatch;

    int crtcaddr;

    uint8_t cgastat;

    // int      writemode, readplane;
    // uint8_t  planemask;

    uint8_t  egapal[16];
    uint32_t pallook[64];
    PALETTE  vgapal;

    int      vtotal, dispend, vsyncstart, vblankstart;
    int      hdisp, htotal, hdisp_time, rowoffset;
    int      lowres;
    int      rowcount;
    double   clock;
    uint32_t memaddr_latch, ca_adj;

    uint64_t   dispontime, dispofftime;
    pc_timer_t timer;
    uint64_t   epochconst;

    int dispon;
    int hdisp_on;

    uint32_t memaddr, memaddr_backup, cursoraddr;
    int      vc;
    int      scanline;
    int      linepos, vslines, linecountff;
    int      cursorvisible, cursoron, blink;
    int      scrollcache;
    int      char_width;
    int      font24;
    double   pixelclock;

    int firstline, lastline;
    int firstline_draw, lastline_draw;
    int displine;
    int oddeven;

    /* Attribute Buffer E0000-E0FFFh (4 KB) */
    uint8_t *cram;
    /* SBCS Font Buffer D8000-DBFFFh (16 KB) */
    /* APA Buffer A0000-DFFFFh (256 KB) */
    uint8_t *vram;
    mem_mapping_t cmap, vmap, vmap2, paritymap;
    /* Kanji font card (optional): the card's page ROM and the 32 or 48 KB
       window RAM the guest sees at F0000h. */
    uint8_t      *fontcard_rom;
    uint8_t      *fontcard_win;
    uint32_t      fontcard_page;
    uint32_t      fontcard_pages;
    int           fontcard_isnew;
    int           fontcard_enabled;
    int           fontcard_charset; /* 5550 selection, EPOCH_FONTCARD_CHARSET */
    /* Holds the last port written (0x160/0x168/0x16A), not a data byte: it must
       be wide enough to keep the port number, otherwise the 0x168/0x16A cases
       in epoch_fontcard_in() can never match. */
    uint16_t      fontcard_data;
    mem_mapping_t fontcard_map;
    mem_mapping_t hiram_map; /* 3477-S 60 KB RAM at 0xE1000-0xEFFFF */
    uint8_t      *hiram;
    // uint8_t *changedvram;
    uint32_t vram_display_mask;

    int parityerror;
    int parityenabled;
    uint8_t parityerroraddr;
    int lowmemorydisabled;
    int crtioenabled;
    uint8_t dispbank; /* port 0x400: bits 1-0 pick the 0xE0000 plane (3477-S) */
    /* [3477-S] Write-only memory-window selectors.  0x390 picks the
       view of 0x40000-0x9FFFF that the IPL's memory walk uses (it
       writes 2 to open and 0 to close around the 0x4000+ pass);
       0x1B2/0x2C2 hold the A2h-derived window parameters.  All three
       are write-only on the real board - the firmware never reads them
       back.  On a single-board model there is no second view, so these
       latch the value without re-mapping (the range stays visible). */
    uint8_t map390;   /* port 0x390: memory-window select */
    uint8_t reg1b2;   /* port 0x1B2: window parameter (A2h bit 7 = 0) */
    uint8_t reg2c2;   /* port 0x2C2: window parameter (A2h bit 7 = 1) */

    int fullchange;

    void (*render)(struct epoch_t *epoch);

    nvr_t nvr;
    int nvrctrl;
    int nvrdata;

    /* Software timer (IRQ7) control, port 0xA6 */
    pc_timer_t swtimer_timer;
    uint8_t swtimer;

    int testmode;   /* [FIX] cfg [Machine] testmode: which 32 KB ROM bank is
                       mapped at F8000-FFFFF (1 = burn-in/upper). */
    int testsw;     /* cfg [Machine] testsw: the rear test-switch input the firmware
                       reads back as port 0xA0 bit 1.  Kept separate from
                       testmode so bank and switch can be A/B-ed. */

    /* 5535-M status/diagnostic register file: index port 0x368, data port
       0x36A. It latches the POST error code (indices 0x10-0x14) and is read
       back by the IPL to report the previous failure, so it is deliberately
       left untouched by epoch_reset(). */
    uint8_t diag_index;
    uint8_t diag_reg[0x20];

    /* Set once the IPL has asked for a POST restart, and cleared by a hard
       reset: port 0xA0 reads it back on bit 3 and the IPL's reset entry uses
       it to tell a restart request from a power-on. resetarmed holds the
       stage request until the 0x4F restart command on port 0xA1. */
    int warmstart;
    int resetarmed;

    /* One-shot pulse that restarts the processor. */
    pc_timer_t pulse_timer;

    /* 5535-M memory configuration block (ports 0x280-0x28A, 5535-M only -
       5550's IPL never touches these). 0x284 reads a block-present bitmap
       (bit set = block absent), 0x288 takes a 16-bit configuration value,
       0x282/0x28A take block control bytes, and 0x286 reads back the high
       half of that value plus a nibble mirroring the two control bits of
       the last 0x282/0x28A write. A write to 0x280 clears the block back
       to that power-on state (the IPL does it before its 0x286 checks). */
    uint16_t mem_present;
    uint16_t mem_cfg;
    uint8_t mem_ctrl;

    /* Parity state of RAM, one bit per byte (the planar stores a parity bit
       with every byte). A set bit means the byte was last written while parity
       update was disabled, so its parity is stale; reading it back with updating
       enabled raises the error. Both IPLs use the "disable -> write -> enable ->
       read" sequence on port 0xA0h, but they doit per address - the 5535-M's walk
       at F87CA re-checks the latch against 0xA1 for every 64/128/256/512K boundary
       it reads, and the 5550's walk (a routine copied to 0200:0 by FC5CC) writes
       whole blocks and requires the latch to stay clear - so the stale state has
       to be per byte and not a single flag. */
    uint8_t *paritybad;
    /* Port 0xA0 bit 3: parity error test mode. While set, every checked read
       reports an error and every write leaves a stale parity bit; it is a level,
       cleared only by the next write to port 0xA0 (the 5535-M's F87CA re-arms it
       once and then reads each boundary in turn, and clears it with out A0h,40h
       before the 0xCCCC write that must then fail the read). */
    int parityforce;

} epoch_t;

static void     epoch_recalctimings(epoch_t *epoch);
static void     epoch_reset(void *priv);

/*
[IRQ]
The IBM 5550 has different IRQ assignments like the 6580 Displaywriter System.

| IRQ | 5550       | Displaywriter                      | PC/XT            |
| --- | ---------- | ---------------------------------- | ---------------- |
| 0   | ?          | Incoming data for printer sharing  | Timer            |
| 1   | Async Comm | Transfer data to commo data link   | Keyboard         |
| 2   | Fixed Disk | Printer and Mag Card data transfer | Reserved         |
| 3   | Keyboard   | Keyboard incoming data             | Async Comm (Sec) |
| 4   | Diskette   | Diskette interrupt                 | Async Comm (Pri) |
| 5   | ?          | Not used                           | Fixed Disk       |
| 6   | Timer      | Software timer                     | Diskette         |
| 7   | Soft Timer | Error on commo data link           | Printer          |

[Memory map]
| Start Address | Function                                              | 
| ------------- | ----------------------------------------------------- | 
| 0             | 256 KB RAM on System Board                            | 
| 40000h        | 128 KB Expansion RAM Card                             | 
| 60000h        | 128 KB Expansion RAM Card                             | 
| 80000h        | 128 KB Expansion RAM Card                             | 
| A0000h        | Video RAM (Font 16: 144 KB, Font 24: 256 KB)          | 
| E0000h        | 4 KB Code/Attribute Buffer                            | 
| E8000h        | ? KB Hard Disk Control Local Memory (not implemented) | 
| F0000h        | Kanji Font Card                                       | 
| FC000h        | ROM                                                   | 
*/

static void epoch_fontcard_set_page(epoch_t *epoch, uint32_t page);

static void
epoch_out(uint16_t addr, uint16_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    epoch_iolog("%04X:%04X epoch Out addr %03X val %02X\n", cs >> 4, cpu_state.pc, addr, val);
    switch (addr) {
        case LC_INDEX:
            epoch->crtcaddr = val & 0xFF;
            break;
        case LC_DATA:
            if (epoch->crtioenabled == 0)
                return;
            if (!(epoch->crtcaddr == LC_CURSOR_LOC_HIGH || epoch->crtcaddr == LC_CURSOR_LOC_LOWJ))
                epoch_iolog("%04X:%04X epoch Out addr %03X idx %02X val %02X (%d)\n", cs >> 4, cpu_state.pc, addr, epoch->crtcaddr, val, val);
            if (!(epoch->crtc[epoch->crtcaddr] ^ val))
                return;
            // switch (epoch->crtcaddr) {
            //     // case LC_CRTC_OVERFLOW:
            //     //     // return;
            //     //     break;
            //     case LC_MAXIMUM_SCAN_LINE:
            //         // if (!(epoch->ioctl[LS_MODE] & 0x01)) /* 16 or 256 color graphics mode */
            //         //     val = 0;
            //         break;
            //     // case LC_VERTICAL_TOTALJ:      /* Vertical Total */
            //     //     break;
            // }
            switch (epoch->crtcaddr) {
                case LC_START_ADDRESS_HIGH:
                case LC_CURSOR_LOC_HIGH:
                case LC_LIGHT_PEN_HIGH:
                    val &= 0x3F; /* this is required for the IPL BAT test */
                    break;
            }
            epoch->crtc[epoch->crtcaddr] = val;
            switch (epoch->crtcaddr) {
                case LC_HORIZONTAL_DISPLAYED:
                case LC_VERTICAL_DISPLAYED:
                case LC_MAXIMUM_SCAN_LINE:
                case LC_START_ADDRESS_HIGH:
                case LC_START_ADDRESS_LOW:
                    epoch->fullchange = 3;
                    epoch_recalctimings(epoch);
                    break;
                default:
                    break;
            }
            break;
        case LS_ENABLE:
            // mem_mapping_disable(&epoch->paritymap);
            epoch->crtioenabled = 1;
            break;
        case LS_DISABLE:
            epoch->crtioenabled = 0;
            // mem_mapping_enable(&epoch->paritymap);
            break;
        case LS_MODE:
            /* Bit 3: Video output enable, Bit 1: Graphic mode (switch 16 / 9 bit word in Font 16 system) */
            epoch->crtmode = val;
            epoch_recalctimings(epoch);
            // epoch->attrff ^= 1;
            break;
        case 0x3e3:
            /* 3477-S: its built-in ROS FONT ROM sits in the F000 window and this port
               selects the 32 KB page (the burn-in always writes 0, i.e. page 0). */
            if (epoch_is_3477)
                epoch_fontcard_set_page(epoch, val);
            break;
        // case LV_PORT:
        //     // epoch_iolog("epoch Out addr %03X val %02X ff %d %04X:%04X\n", addr, val, epoch->attrff,cs >> 4, cpu_state.pc);
        //     if (!epoch->attrff) {
        //         epoch->attraddr = val & 0x3f;
        //         if ((val & 0x20) != (epoch->attr_palette_enable & 0x20)) {
        //             epoch->fullchange          = 3;
        //             epoch->attr_palette_enable = val & 0x20;
        //             epoch_recalctimings(epoch);
        //         }
        //         // epoch_iolog("set attraddr: %X\n", epoch->attraddr);
        //     } else {
        //         if ((epoch->attraddr == LV_PANNING) && (epoch->attrc[LV_PANNING] != val))
        //             epoch->fullchange = changeframecount;
        //         if (epoch->attrc[epoch->attraddr & 0x3f] != val)
        //             epoch_iolog("attr changed %x: %x -> %x\n", epoch->attraddr & 0x3f, epoch->attrc[epoch->attraddr & 0x3f], val);
        //         epoch->attrc[epoch->attraddr & 0x3f] = val;
        //         // epoch_iolog("set attrc %x: %x\n", epoch->attraddr & 31, val);
        //         if (epoch->attraddr < 16)
        //             epoch->fullchange = changeframecount;
        //         if (epoch->attraddr == LV_MODE_CONTROL || epoch->attraddr < 0x10) {
        //             for (uint8_t c = 0; c < 16; c++) {
        //                 // if (epoch->attrc[LV_MODE_CONTROL] & 0x80)
        //                 //     epoch->egapal[c] = epoch->attrc[c] & 0xf;
        //                 // else
        //                 //     epoch->egapal[c] = epoch->attrc[c] & 0x3f;
        //             }
        //         }
        //         switch (epoch->attraddr) {
        //             case LV_COLOR_PLANE_ENAB:
        //                 if ((val & 0xff) != epoch->plane_mask)
        //                     epoch->fullchange = changeframecount;
        //                 epoch->plane_mask = val & 0xff;
        //                 break;
        //             case LV_CURSOR_CONTROL:
        //                 switch (val & 0x18) {
        //                     case 0x08: /* fast blink */
        //                         epoch->blinkconf = 0x10;
        //                         break;
        //                     case 0x18: /* slow blink */
        //                         epoch->blinkconf = 0x20;
        //                         break;
        //                     default: /* no blink */
        //                         epoch->blinkconf = 0xff;
        //                         break;
        //                 }
        //                 break;
        //             case LV_MODE_CONTROL:
        //             case LV_ATTRIBUTE_CNTL:
        //             case LV_COMPATIBILITY:
        //                 epoch_recalctimings(epoch);
        //                 break;
        //             default:
        //                 break;
        //         }
        //     }
        //     epoch->attrff ^= 1;
        //     break;
        // case LV_PORT+1:
        //     /* VZ Editor's CURSOR.COM writes data via this port */
        //     epoch->attrc[epoch->attraddr & 0x3f] = val;
        //     break;
        default:
            // epoch_iolog("epoch? Out addr %03X val %02X\n", addr, val);
            break;
    }
}

static uint16_t
epoch_in(uint16_t addr, void *priv)
{
    epoch_t   *epoch  = (epoch_t *) priv;
    uint16_t temp = 0xff;

    switch (addr) {
        case LC_INDEX:
            temp = epoch->crtcaddr;
            break;
        case LC_DATA:
            if (epoch->crtioenabled == 0)
                return EPOCH_INVALIDACCESS16;
            temp = epoch->crtc[epoch->crtcaddr];
            break;
        // case LV_PORT:
        //     temp = epoch->attraddr | epoch->attr_palette_enable;
        //     break;
        // case LV_PORT + 1:
        //     switch (epoch->attraddr) {
        //         case LV_RAS_STATUS_VIDEO: /* this maybe equivalent to 3ba / 3da ISR1 */
        //             if (epoch->cgastat & 0x01)
        //                 epoch->cgastat &= ~0x30;
        //             else
        //                 epoch->cgastat ^= 0x30; /* toggle */
        //             if (epoch->cgastat & 0x08)
        //                 epoch->cgastat &= ~0x08;
        //             else
        //                 epoch->cgastat ^= 0x08; /* toggle */
        //             temp = epoch->cgastat;
        //             break;
        //         case LV_IDENTIFICATION:
        //             temp = 0x28;
        //             break;
        //         default:
        //             temp = epoch->attrc[epoch->attraddr];
        //             break;
        //     }
        //     // epoch_iolog("epoch In %04X(%02X) %04X %04X:%04X\n", addr, epoch->attraddr, temp, cs >> 4, cpu_state.pc);
        //     epoch->attrff = 0; /* reset flipflop (VGA does not reset flipflop) */
        //     break;
        case LS_MONSENSE:
            /* 3DAh bit 7 reads clear only while the video output is enabled
               (mode register bit 3 clear) and the display is in its active
               period; bit 6 is the vertical blanking flag (clear while
               blanking). The 5535-M's IPL relies on both: it polls bit 7
               while the output is off, writes mode 0 (F8436), polls again
               and requires it clear with AH non-zero, then runs two bit 6
               handshakes (F8456/F8464). */
            temp = 0xff;
            /* The 3477-S burn-in watches bits 4-6 and needs them to invert
               between two consecutive reads, but only while mode register 2 is
               selected: it reads 0x94 twice, then 0x04 (F8655/F86C2). */
            if (epoch_is_3477 && ((epoch->crtmode == 0x04) || (epoch->crtmode == 0x10) || (epoch->crtmode == 0x94))) {
                epoch->cgastat ^= 0x30;
                epoch->cgastat ^= 0x08;
            }
            if (epoch->cgastat & 0x10)
                temp &= ~0x10;
            if (epoch->cgastat & 0x20)
                temp &= ~0x20;
            if (!(epoch->crtmode & 0x08) && (epoch->cgastat & 8))
                temp &= 0x7f;      /* display active */
            if (!(epoch->cgastat & 8))
                temp &= ~0x40;     /* vertical blanking */
            temp |= 0x01; /* monitor mono or !color */
            // temp &= 0xfe; /* color */
            break;
        }
    if (addr != LS_MONSENSE)
        epoch_iolog("%04X:%04X epoch In %04X %04X\n", cs >> 4, cpu_state.pc, addr, temp);
    return temp;
}
/*
 * Write I/O
 * out b(idx), out b(data), out b(data)
 * out b(idx), out w(data)
 * out b(idx), out w(data), out b(data)
 *             out w(idx)
 * Read I/O
 * out b(idx), in b(data)
 * out b(idx), in b, in b(data)
 * out b(idx), in w(data)
 */
/* [Cloud server only, DO NOT USE!!!] 3477-S bring-up: latest epoch instance, so
   the sampler can report what the display controller would draw. */
static epoch_t *epoch_dbg_cur = NULL;

static void
epoch_outb(uint16_t addr, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    epoch_dbg_cur = epoch;
    pclog("3477: outb %03X <- %02X\n", addr, val);
    // epoch_iolog("%04X:%04X epoch Outb addr %03X val %02X es:di=%x:%x ds:si=%x:%x\n", cs >> 4, cpu_state.pc, addr, val, ES, DI, DS, SI);
    epoch->inflipflop = 0;
    switch (addr) {
        // case LS_DATA:
        // case LF_DATA:
        case LC_DATA:
            if (epoch->outflipflop) {
                /* out b(idx), out b(data), out b(data) */
                epoch->iolatch |= (uint16_t) val << 8;
                epoch->outflipflop = 0;
            } else { //
                epoch->iolatch     = val;
                epoch->outflipflop = 1;
            }
            break;
        // case LS_INDEX:
        // case LF_INDEX:
        case LC_INDEX:
        default:
            epoch->iolatch     = val;
            epoch->outflipflop = 0;
            break;
    }
    epoch_out(addr, epoch->iolatch, epoch);
}
static void
epoch_outw(uint16_t addr, uint16_t val, void *priv)
{
    epoch_iolog("epoch Outw addr %03X val %04X\n", addr, val);
    epoch_t *epoch      = (epoch_t *) priv;
    epoch->inflipflop = 0;
    switch (addr) {
        // case LS_INDEX:
        // case LF_INDEX:
        case LC_INDEX:
        // case LG_INDEX:
            epoch_out(addr, val & 0xff, epoch);
            epoch->iolatch = val >> 8;
            epoch_out(addr + 1, epoch->iolatch, epoch);
            epoch->outflipflop = 1;
            break;
        // case LV_PORT:
        //     // epoch->attrff = 0;
        //     epoch_out(addr, val & 0xff, epoch);
        //     epoch_out(addr, val >> 8, epoch);
        //     epoch->outflipflop = 0;
        //     break;
        // case LS_DATA:
        // case LF_DATA:
        case LC_DATA:
        // case LG_DATA:
        default:
            epoch_out(addr, val, epoch);
            epoch->outflipflop = 0;
            break;
    }
}
static uint8_t
epoch_inb(uint16_t addr, void *priv)
{
    uint8_t temp;
    epoch_t  *epoch      = (epoch_t *) priv;
    epoch->outflipflop = 0;
    switch (addr) {
        case LC_DATA:
            if (epoch->inflipflop) {
                /* out b(idx), in b(low data), in b(high data) */
                temp            = epoch->iolatch >> 8;
                epoch->inflipflop = 0;
            } else { //
                epoch->iolatch    = epoch_in(addr, epoch);
                temp            = epoch->iolatch & 0xff;
                epoch->inflipflop = 1;
            }
            break;
        // case LS_INDEX:
        // case LF_INDEX:
        case LC_INDEX:
        // case LS_DATA:
        // case LF_DATA:
        default:
            temp            = epoch_in(addr, epoch) & 0xff;
            epoch->inflipflop = 0;
            break;
    }
    // epoch_iolog("epoch Inb %04X %02X %04X:%04X\n", addr, temp, cs >> 4, cpu_state.pc);
    return temp;
}
static uint16_t
epoch_inw(uint16_t addr, void *priv)
{
    uint16_t temp;
    epoch_t *epoch       = (epoch_t *) priv;
    epoch->inflipflop  = 0;
    epoch->outflipflop = 0;
    temp = epoch_in(addr, epoch);
    epoch_iolog("epoch Inw addr %03X val %04X\n", addr, temp);
    return temp;
}

/* The 3477-S also decodes its CRTC block at 0x2D0-0x2EF; the burn-in reads
   0x2DA for the display status and writes 0x2D7/0x2DF.  Treat it as the same
   registers as 0x3D0-0x3EF.  [Cloud server only, DO NOT USE!!!] */
static uint8_t
epoch_mirror_inb(uint16_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    uint8_t  save;
    uint8_t  val;

    if (addr != 0x2da)
        return epoch_inb((uint16_t) (addr + 0x100), priv);
    /* The burn-in reads 0x2DA once and needs bit 6 clear, while it polls 0x3DA
       for bits that invert between two reads, so this is a status that can
       report ready and must not carry the read-side inversion. */
    save           = epoch->cgastat;
    val            = epoch_inb((uint16_t) (addr + 0x100), priv);
    epoch->cgastat = save;
    return (uint8_t) (val & ~0x40);
}
static uint16_t
epoch_mirror_inw(uint16_t addr, void *priv)
{
    return epoch_inw((uint16_t) (addr + 0x100), priv);
}
static void
epoch_mirror_outb(uint16_t addr, uint8_t val, void *priv)
{
    epoch_outb((uint16_t) (addr + 0x100), val, priv);
}
static void
epoch_mirror_outw(uint16_t addr, uint16_t val, void *priv)
{
    epoch_outw((uint16_t) (addr + 0x100), val, priv);
}

/* Return a memory address for 9-bit word access */
static uint32_t
getaddr_9bitword(int32_t addr)
{
            int32_t bit9addr = addr;
            if (bit9addr & 2)
                bit9addr--;
            if (bit9addr & 0x20000)
                bit9addr += 2;
            bit9addr &= 0x1ffff;
            return bit9addr;
}

/* One scanline of a DBCS glyph out of video memory: two 9-dot halves, the 9th
   dot of each taken from the plane at 0x20000.  This is the layout the kanji
   card fills in (18 dots wide, the glyph itself sitting on columns 1-16). */
static uint32_t
getfont_ps55dbcs_vram(epoch_t *epoch, int32_t code, int32_t line)
{
    uint32_t font = 0;

    int32_t  chr  = code;
    code *= 0x80;
    code += line * 4;
    /* [AB] 3477-S: the built-in ANK font (font.bin, 256 codes x 128 bytes) is
       loaded into the SBCS area at EPOCH_VRAM_SBCS.  The 3477 render branch
       indexed from 0, so every lookup read zeros, every cell drew the
       background colour and the screen stayed black even though the mode was
       programmed correctly. */
    if (epoch_is_3477 && chr < 0x100)
        code += EPOCH_VRAM_SBCS;
    if (epoch->font24) { /* Font 24 (2 x 13 x 29) */
        font = epoch->vram[code];
        font <<= 8;
        code++;
        font |= epoch->vram[code];
        font <<= 8;
        code++;
        font |= epoch->vram[code];
        font <<= 8;
        code++;
        font |= epoch->vram[code];
    } else { /* Font 16 (2 x 9 x 21) */
        int32_t bit9addr = getaddr_9bitword(code);
        int bitnum = bit9addr & 7;
        bit9addr >>= 3;
        bit9addr += 0x20000; /* real: C0000h */
        /* The card is filled through 9-bit word addressing, so the data bytes
           must be translated the same way the guest's writes were. */
        font = epoch->vram[getaddr_9bitword(code)];
        font <<= 8;
        font |= (epoch->vram[bit9addr] << (7 - bitnum)) & 0x80; /* get 9th bit */
        font <<= 8;
        code++;
        font |= epoch->vram[getaddr_9bitword(code)];
        font <<= 8;
        bitnum = getaddr_9bitword(code) & 0x7;
        font |= (epoch->vram[bit9addr] << (7 - bitnum)) & 0x80; /* get 9th bit */
    }
    return font;
}

/* Get font pattern in a line */
static uint32_t
getfont_ps55dbcs(int32_t code, int32_t line, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    if (code < 1536)
        return getfont_ps55dbcs_vram(epoch, code, line);
    return EPOCH_INVALIDACCESS32;
}

/* Get the foreground color from the attribute byte */
static uint8_t
getPS55ForeColor(uint8_t attr, epoch_t *epoch)
{
    uint8_t foreground = ~attr & 0x08; /* 0000 1000 */
    foreground <<= 2;                  /* 0010 0000 */
    foreground |= ~attr & 0xc0;        /*  1110 0000 */
    foreground >>= 4;                  /* 0000 1110 */
    // if (epoch->attrc[LV_PAS_STATUS_CNTRL] & 0x40)
    //     foreground |= 0x01; /* bright color palette */
    return foreground;
}

static void
epoch_render_blank(epoch_t *epoch)
{
    int x, xx;

    if (epoch->firstline_draw == 2000)
        epoch->firstline_draw = epoch->displine;
    epoch->lastline_draw = epoch->displine;

    for (x = 0; x < epoch->hdisp + epoch->scrollcache; x++) {
        for (xx = 0; xx < epoch->char_width; xx++)
            ((uint32_t *) buffer32->line[epoch->displine])[(x * epoch->char_width) + xx] = 0;
    }
}
/* Display Adapter Mode 8, E Drawing */
static void
epoch_render_text(epoch_t *epoch)
{
    /* Refresh a "what is on screen right now" pair every few frames, so the
       state behind a problem screen can be picked up at any time. */
    if (epoch->firstline_draw == 2000)
        epoch->firstline_draw = epoch->displine;
    epoch->lastline_draw = epoch->displine;

    if (epoch->fullchange) {
        int       offset = (8 - epoch->scrollcache) + 24;
        uint32_t *p      = &((uint32_t *) buffer32->line[epoch->displine])[offset];
        int       x;
        int       drawcursor;
        uint8_t   chr, attr;
        int       fg, bg;
        uint32_t  chr_dbcs;
        int       underscore_y = (epoch->font24) ? 28 : 20;
        int       chr_wide = 0;
        // int       colormode = ((epoch->attrc[LV_PAS_STATUS_CNTRL] & 0x80) == 0x80);
        int       colormode = 0;
        // epoch_log("displ: %x, first: %x, epochma: %x, epochsc: %x\n", 
        //     epoch->displine , epoch->firstline_draw, epoch->memaddr, epoch->scanline);
        for (x = 0; x < epoch->hdisp; x += epoch->char_width) {
            chr  = epoch->cram[(epoch->memaddr) & EPOCH_MASK_CRAM];
            attr = epoch->cram[(epoch->memaddr + 1) & EPOCH_MASK_CRAM];
            // if(chr!=0x20) epoch_log("chr: %x, attr: %x    ", chr, attr);
            if (colormode) /* IO 3E8h, Index 1Dh */
            {                                           /* --Parse attribute byte in color mode-- */
                bg = 0;                                 /* bg color is always black (the only way to change background color is programming PAL) */
                fg = getPS55ForeColor(attr, epoch);
                if (attr & 0x04) { /* reverse 0000 0100 */
                    bg = fg;
                    fg = 0;
                }
            } else { /* --Parse attribute byte in monochrome mode-- */
                if (attr & 0x08)
                    fg = 3; /* Highlight 0000 1000 */
                else
                    fg = 2;
                bg = 0;              /* Background is always color #0 (default is black) */
                if (!(~attr & 0xCC)) /* Invisible 11xx 11xx -> 00xx 00xx  */
                {
                    fg = bg;
                    attr &= 0x33; /* disable blinkking, underscore, highlight and reverse */
                }
                if (attr & 0x04) { /* reverse 0000 0100 */
                    bg = fg;
                    fg = 0;
                }
                /* Blinking 1000 0000 */
                fg = ((epoch->blink & 0x20) || (!(attr & 0x80))) ? fg : bg;
                // if(chr!=0x20) epoch_log("chr: %x, %x, %x, %x, %x    ", chr, attr, fg, epoch->egapal[fg], epoch->pallook[epoch->egapal[fg]]);
            }
            /* Draw character */
            for (uint32_t n = 0; n < epoch->char_width; n++)
                p[n] = epoch->pallook[epoch->egapal[bg]]; /* draw blank */
            /* [Cloud server only, DO NOT USE!!!] 3477-S: one 26-dot glyph per
               13-dot cell (40 glyphs x 26 = 1040).  The ROM stores one word per
               character in CRAM, so consume one pair and draw the whole glyph;
               the code indexes the card's glyph area directly. */
            if (epoch_is_3477) {
                uint32_t font = getfont_ps55dbcs(chr, epoch->scanline, epoch);
                for (uint32_t n = 0; n < 26; n++) {
                    p[n] = epoch->pallook[epoch->egapal[(font & 0x80000000) ? fg : bg]];
                    font <<= 1;
                }
                x += epoch->char_width; /* the loop adds another 13 => 26 total */
                p += epoch->char_width * 2;
                epoch->memaddr += 2;
                continue;
            }
            /*  SBCS or DBCS left half */
            if (chr_wide == 0) {
                if (attr & 0x01)
                    chr_wide = 1;
                // chr_wide = 0;
                /* Stay drawing If the char code is DBCS and not at last column. */
                if (chr_wide) {
                    /* Get high DBCS code from the next video address */
                    chr_dbcs = epoch->cram[(epoch->memaddr + 2) & EPOCH_MASK_CRAM];
                    chr_dbcs <<= 8;
                    chr_dbcs |= chr;
                    /* Get the font pattern */
                    uint32_t font = getfont_ps55dbcs(chr_dbcs, epoch->scanline, epoch);
                    /* Draw 13 dots */
                    for (uint32_t n = 0; n < epoch->char_width; n++) {
                        p[n] = epoch->pallook[epoch->egapal[(font & 0x80000000) ? fg : bg]];
                        font <<= 1;
                    }
                } else {
                    /* the char code is SBCS (ANK) */
                    uint32_t fontbase;
                    uint16_t font;
                    if (attr & 0x02) /* second map of SBCS font */
                        fontbase = EPOCH_VRAM_SBEX;
                    else
                        fontbase = EPOCH_VRAM_SBCS;
                    if (epoch->font24) {
                        font = epoch->vram[fontbase + chr * 0x80 + epoch->scanline * 4 + 2]; /* w13xh29 font */
                        font <<= 8;
                        font |= epoch->vram[fontbase + chr * 0x80 + epoch->scanline * 4 + 3];
                    } else {
                        uint32_t bitnum, bit9addr;
                        fontbase += chr * 0x80 + epoch->scanline * 4;
                        bit9addr = getaddr_9bitword(fontbase);
                        bitnum   = bit9addr & 7;
                        bit9addr >>= 3;
                        bit9addr += 0x20000; /* real: C0000h */

                        fontbase &= 0x1ffff;
                        font = epoch->vram[fontbase + 2]; /* w9xh21 font */
                        font <<= 8;
                        font |= (epoch->vram[bit9addr] << (7 - bitnum)) & 0x80;
                        // if(chr!=0x20) epoch_log("faddr: %x, scline: %x, chr: %x, font: %x    ", fontbase + chr * 0x80 + epoch->scanline * 4, epoch->scanline, chr, font);
                    }
                    // if(chr!=0x20) epoch_log("memaddr: %x, scanline: %x, chr: %x, font: %x    ", epoch->memaddr, epoch->scanline, chr, font);
                    /* Draw 13 dots */
                    for (uint32_t n = 0; n < epoch->char_width; n++) {
                        p[n] = epoch->pallook[epoch->egapal[(font & 0x8000) ? fg : bg]];
                        font <<= 1;
                    }
                }
            }
            /* right half of DBCS */
            else {
                uint32_t font = getfont_ps55dbcs(chr_dbcs, epoch->scanline, epoch);
                /* Draw 13 dots */
                for (uint32_t n = 0; n < epoch->char_width; n++) {
                    p[n] = epoch->pallook[epoch->egapal[(font & 0x8000) ? fg : bg]];
                    font <<= 1;
                }
                chr_wide = 0;
            }
            /* Line 28 (Underscore) Note: Draw this first to display blink + vertical + underline correctly. */
            if (epoch->scanline == underscore_y && attr & 0x40 && !colormode) { /* Underscore only in monochrome mode */
                for (uint32_t n = 0; n < epoch->char_width; n++)
                    p[n] = epoch->pallook[epoch->egapal[fg]]; /* under line (white) */
            }
            /* Column 1 (Vertical Line) */
            if (attr & 0x10) {
                p[0] = epoch->pallook[epoch->egapal[2]]; /* vertical line (white) */
            }
            if (epoch->scanline == 0 && attr & 0x20) { /* HGrid */
                for (uint32_t n = 0; n < epoch->char_width; n++)
                    p[n] = epoch->pallook[epoch->egapal[2]]; /* horizontal line (white) */
            }
            /* Drawing text cursor */
            drawcursor = ((epoch->memaddr == epoch->cursoraddr) && epoch->cursorvisible && epoch->cursoron);
            if (drawcursor) {
                // int cursorwidth = (epoch->crtc[LC_COMPATIBILITY] & 0x20 ? 26 : 13);
                int cursorwidth = epoch->char_width;
                int cursorcolor = 2; /* Choose color 2 if mode 8 */
                fg              = ((attr & 0x08) ? 3 : 2);
                bg              = 0;
                if (attr & 0x04) { /* Color 0 if reverse */
                    bg = fg;
                    fg = 0;
                }
                for (uint32_t n = 0; n < cursorwidth; n++)
                    if (p[n] == epoch->pallook[epoch->egapal[cursorcolor]] || epoch->egapal[bg] == epoch->egapal[cursorcolor])
                        p[n] = (p[n] == epoch->pallook[epoch->egapal[bg]]) ? epoch->pallook[epoch->egapal[fg]] : epoch->pallook[epoch->egapal[bg]];
                    else
                        p[n] = (p[n] == epoch->pallook[epoch->egapal[bg]]) ? epoch->pallook[epoch->egapal[cursorcolor]] : p[n];
            }
            epoch->memaddr += 2;
            p += epoch->char_width;
        }
    }
}

static void
epoch_render_color_4bpp(epoch_t *epoch)
{
    // int changed_offset = epoch->memaddr >> 9;
    // epoch_log("memaddr %x cf %x\n", epoch->memaddr, changed_offset);
    // epoch->plane_mask &= 1; /*safety */

    // if (epoch->changedvram[changed_offset] || epoch->changedvram[changed_offset + 1] || epoch->fullchange) {
        int       x;
        int       offset = (8 - epoch->scrollcache) + 24;
        uint32_t *p      = &((uint32_t *) buffer32->line[epoch->displine])[offset];

        if (epoch->firstline_draw == 2000)
            epoch->firstline_draw = epoch->displine;
        epoch->lastline_draw = epoch->displine;
        // epoch_log("d %X\n", epoch->memaddr);
        int readvaddr = (epoch->memaddr * 8) + (epoch->scanline * 128);
        for (x = 0; x <= epoch->hdisp; x += 8) /* hdisp = 1024 */
        {
            uint8_t edat[8];
            uint8_t dat;

            /* get 8 pixels from vram */
            readvaddr &= epoch->vram_display_mask;
            *(uint8_t *) (&edat[0]) = *(uint8_t *) (&epoch->vram[readvaddr]);
            readvaddr += 1;

            dat  = ((edat[0] >> 7) & 1);
            p[0] = epoch->pallook[epoch->egapal[dat]];
            dat  = ((edat[0] >> 6) & 1);
            p[1] = epoch->pallook[epoch->egapal[dat]];
            dat  = ((edat[0] >> 5) & 1);
            p[2] = epoch->pallook[epoch->egapal[dat]];
            dat  = ((edat[0] >> 4) & 1);
            p[3] = epoch->pallook[epoch->egapal[dat]];
            dat  = ((edat[0] >> 3) & 1);
            p[4] = epoch->pallook[epoch->egapal[dat]];
            dat  = ((edat[0] >> 2) & 1);
            p[5] = epoch->pallook[epoch->egapal[dat]];
            dat  = ((edat[0] >> 1) & 1);
            p[6] = epoch->pallook[epoch->egapal[dat]];
            dat  = ((edat[0] >> 0) & 1);
            p[7] = epoch->pallook[epoch->egapal[dat]];
            p += 8;
        }
}

/*
    INT 10h video modes supported in DOS K3.44.
    Mode        Type    Colors  Text    Base Address    PELs        Render
    2           A/N     3       80 x 25 E0000h          1040 x 725* text
    8           A/N/K   3       80 x 25 E0000h          1040 x 725* text
    9           APA     2       80 x 25 A0000h          720 x 512   color_4bpp
    Ah          APA     2       78 x 25 A0000h          1024 x 768  color_4bpp
    Bh          APA     16      40 x 25 A0000h          360 x 512   n/a
    Ch          APA     16      80 x 25 A0000h          720 x 512   n/a
    Dh          APA     16      78 x 25 A0000h          1024 x 768  n/a
    Eh          A/N/K   16      80 x 25 E0000h          1040 x 725* n/a
                                        (* 720 x 525 in the Font 16 system.)
*/
static void
epoch_recalctimings(epoch_t *epoch)
{
    double crtcconst;
    double _dispontime, _dispofftime, disptime;
    
    epoch->vblankstart = epoch->crtc[LC_VERTICAL_TOTAL] & 0x7f;
    epoch->dispend     = epoch->crtc[LC_VERTICAL_DISPLAYED] & 0x7f;
    epoch->vsyncstart  = epoch->crtc[LC_V_SYNC_POSITION] & 0x7f;
    epoch->vblankstart += 1;
    epoch->vtotal      =  epoch->vblankstart + (epoch->crtc[LC_V_TOTAL_ADJUST] & 0x1f);
    epoch->hdisp       = epoch->crtc[LC_HORIZONTAL_DISPLAYED] & 0xff;

    // epoch->hdisp -= epoch->crtc[LC_START_H_DISPLAY_ENAB];
    // epoch->dispend -= epoch->crtc[LC_START_V_DISPLAY_ENAB];
    //epoch_log("Dispend %d\n", epoch->dispend);
    // epoch->vsyncstart += 1;
    //epoch->vtotal += 1;

    epoch->htotal = epoch->crtc[LC_HORIZONTAL_TOTAL] & 0xff;
    epoch->htotal += 1;

    // epoch->rowoffset = epoch->crtc[LC_OFFSET]; /* number of bytes in a scanline */
    epoch->rowoffset = epoch->crtc[LC_HORIZONTAL_DISPLAYED] & 0xff;

    epoch->clock = epoch->epochconst;

    if (epoch->vtotal == 0)
        epoch->vtotal = epoch->vsyncstart = epoch->vblankstart = 32;
    if (epoch->htotal == 0)
        epoch->htotal = epoch->dispend = epoch->hdisp = 64;
    if (epoch->rowoffset == 0)
        epoch->rowoffset = 64 * 2; /* To avoid causing a DBZ error */
        
    epoch->memaddr_latch = ((epoch->crtc[LC_START_ADDRESS_HIGH] & 0x3f) << 8) | epoch->crtc[LC_START_ADDRESS_LOW];

    epoch->ca_adj = 0;
    epoch->rowcount = epoch->crtc[LC_MAXIMUM_SCAN_LINE] & 0x1f;
    epoch->rowcount += 1;

    epoch->vtotal *= (epoch->rowcount + 1);
    epoch->dispend *= (epoch->rowcount + 1);
    epoch->vsyncstart *= (epoch->rowcount + 1);
    epoch->vblankstart *= (epoch->rowcount + 1);

    epoch->hdisp_time = epoch->hdisp;

    /* determine display mode */
    if (epoch->crtmode & 0x02) {
        if (epoch->font24) {
            epoch->hdisp *= 16;
            epoch->char_width = 16;
        } else {
            epoch->hdisp *= 12;
            epoch->char_width = 12;
        }
        /* PS/55 8-color */
        epoch_log("Set videomode to PS/55 4 bpp graphics.\n");
        epoch->render            = epoch_render_color_4bpp;
        epoch->vram_display_mask = EPOCH_MASK_A000;
    } else {
        /* PS/55 text(color/mono) */
        epoch_log("Set videomode to PS/55 Mode 8/E text.\n");
        epoch->render            = epoch_render_text;
        epoch->vram_display_mask = EPOCH_MASK_CRAM;
        if (epoch->font24) {
            epoch->hdisp *= 13;
            epoch->char_width = 13;
        } else {
            epoch->hdisp *= 9;
            epoch->char_width = 9;
        }
    }
    if (epoch->crtmode & 0x08)
        epoch->render = epoch_render_blank;

    if (epoch->vblankstart < epoch->vsyncstart)
        epoch->vsyncstart = epoch->vblankstart;
    if (epoch->vsyncstart < epoch->dispend)
        epoch->dispend = epoch->vsyncstart;

    crtcconst = epoch->clock * epoch->char_width;

    disptime    = epoch->htotal;
    _dispontime = epoch->hdisp_time;

    epoch_log("Disptime %f dispontime %f hdisp %i\n", disptime, _dispontime, epoch->hdisp);

    _dispofftime = disptime - _dispontime;
    _dispontime *= crtcconst;
    _dispofftime *= crtcconst;

    epoch->dispontime  = (uint64_t) _dispontime;
    epoch->dispofftime = (uint64_t) _dispofftime;
    if (epoch->dispontime < TIMER_USEC)
        epoch->dispontime = TIMER_USEC;
    if (epoch->dispofftime < TIMER_USEC)
        epoch->dispofftime = TIMER_USEC;
    epoch_log("epoch horiz display end %i vidclock %f htotal %i\n", epoch->hdisp, epoch->clock, epoch->htotal);
    epoch_log("epoch vert display end %i max row %i vsyncstart %i vtotal %i\n",epoch->dispend,epoch->rowcount,epoch->vsyncstart,epoch->vtotal);
    epoch_log("epoch dispon %lu dispoff %lu on(us) %f off(us) %f\n",epoch->dispontime, epoch->dispofftime, 
        (double)epoch->dispontime / (double)cpuclock /  (double) (1ULL << 32) * 1000000.0, 
        (double)epoch->dispofftime / (double)cpuclock /  (double) (1ULL << 32) * 1000000.0);
}

static void
epoch_doblit(int wx, int wy, epoch_t *epoch)
{
    if (wx != xsize || wy != ysize) {
        xsize = wx;
        ysize = wy;
        set_screen_size(xsize, ysize);
        if (video_force_resize_get())
            video_force_resize_set(0);
    }
    video_blit_memtoscreen(32, 0, xsize, ysize);
    frames++;

    video_res_x = wx;
    video_res_y = wy;
    video_bpp   = 8;
}

static void
epoch_poll(void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    int    x;

    if (!epoch->linepos) {
        timer_advance_u64(&epoch->timer, epoch->dispofftime);
        
        epoch->cgastat |= 1;
        epoch->linepos = 1;

        if (epoch->dispon) {
            epoch->hdisp_on = 1;

            epoch->memaddr &= epoch->vram_display_mask;
            if (epoch->firstline == 2000) {
                epoch->firstline = epoch->displine;
                video_wait_for_buffer();
            }

            /* [AB] The 3477-S display is not interlaced: the IPL programs a 29-line
               character cell and the built-in font supplies 29 glyph lines 1:1.  The
               odd/even gate below (5535/5550 timing) rendered only every other
               display line, so each glyph lost half of its scanlines and the screen
               came out as garbage. */
            if (epoch_is_3477 || ((epoch->displine ^ !epoch->oddeven) & 1))
                epoch->render(epoch);

            if (epoch->lastline < epoch->displine)
                epoch->lastline = epoch->displine;
        }

        // epoch_log("%03i %06X %06X\n", epoch->displine, epoch->memaddr,epoch->vram_display_mask);
        epoch->displine++;
        /* cgastat bit 3 backs 3DAh (bit 7 = display status, bit 6 = the
           vertical blanking flag).  The IPLs poll that port before the CRTC
           is programmed, so the blanking is taken from the line counter rather
           than from the (still invalid) CRTC geometry: half of each 512-line
           window. */
        if (epoch->displine & 0x100)
            epoch->cgastat &= ~8;
        else
            epoch->cgastat |= 8;
        epoch->vslines++;
        if (epoch->displine > 2000)
            epoch->displine = 0;
    } else {
        // epoch_log("VC %i memaddr %05X\n", epoch->vc, epoch->memaddr);
        timer_advance_u64(&epoch->timer, epoch->dispontime);

        if (epoch->dispon)
            epoch->cgastat &= ~1;
        epoch->hdisp_on = 0;

        epoch->linepos = 0;
        if (epoch->scanline == (epoch->crtc[LC_CURSOR_ROW_END] & 0x1f))
            epoch->cursorvisible = 0;
        if (epoch->dispon) {
            if (epoch->scanline == epoch->rowcount) {
                epoch->linecountff = 0;
                epoch->scanline          = 0;

                epoch->memaddr_backup += (epoch->rowoffset << 1); /* interlace */
                epoch->memaddr_backup &= epoch->vram_display_mask;
                epoch->memaddr = epoch->memaddr_backup;
            } else {
                epoch->linecountff = 0;
                epoch->scanline++;
                epoch->scanline &= 0x1f;
                epoch->memaddr = epoch->memaddr_backup;
            }
        }

        epoch->vc++;
        epoch->vc &= 0x7ff;

        if (epoch->vc == epoch->dispend) {
            epoch->dispon = 0;
            if (!(epoch->crtmode & 0x02)) {                /* in text mode */
                switch (epoch->crtc[LC_CURSOR_ROW_START] & 0x60) {
                    case 0x20:
                        epoch->cursoron = 0;
                        break;
                    case 0x60:
                        epoch->cursoron = epoch->blink & 0x10;
                        break;
                    case 0x40:
                        epoch->cursoron = epoch->blink & 0x08;
                        break;
                    default:
                        epoch->cursoron = 1;
                        break;
                }
                if (!(epoch->blink & (0x08 - 1))) /* force redrawing for cursor and blink attribute */
                    epoch->fullchange = 3;
            }
            epoch->blink++;

            // for (x = 0; x <= (EPOCH_MASK_VRAMPLANE >> 9); x++) {
            //     if (epoch->changedvram[x])
            //         epoch->changedvram[x]--;
            // }
            // memset(changedvram,0,2048);  del
            if (epoch->fullchange) {
                epoch->fullchange--;
            }
        }
        if (epoch->vc == epoch->vsyncstart) {
            int wx, wy;
            // epoch_log("VC vsync  %i %i\n", epoch->firstline_draw, epoch->lastline_draw);
            epoch->dispon = 0;
            epoch->cgastat |= 8;
            x = epoch->hdisp;

            if (!epoch->oddeven)
                epoch->lastline++;
            if (epoch->oddeven)
                epoch->firstline--;

            wx = x;
            wy = epoch->lastline - epoch->firstline;
            
            epoch_doblit(wx, wy, epoch);

            epoch->firstline = 2000;
            epoch->lastline  = 0;

            epoch->firstline_draw = 2000;
            epoch->lastline_draw  = 0;

            epoch->oddeven ^= 1;

            epoch->vslines     = 0;

            epoch->memaddr
                = epoch->memaddr_backup = epoch->memaddr_latch << 1;
            epoch->cursoraddr = ((epoch->crtc[LC_CURSOR_LOC_HIGH] << 8) | epoch->crtc[LC_CURSOR_LOC_LOWJ]) + epoch->ca_adj;
            epoch->cursoraddr <<= 1;

            // epoch_log("Addr %08X vson %03X vsoff %01X\n",epoch->memaddr,epoch->vsyncstart,epoch->crtc[0x11]&0xF);
        }
        if (epoch->vc == epoch->vtotal) {
            // epoch_log("VC vtotal\n");
            // printf("Frame over at line %i %i  %i %i\n",displine,vc,epoch_vsyncstart,epoch_dispend);
            epoch->vc          = 0;
            epoch->scanline    = 0;
            epoch->dispon      = 1;
            epoch->displine    = 0;
            // epoch->scrollcache = epoch->attrc[LV_PANNING] & 0x07;
            epoch->scrollcache = 0;
        }
        if (epoch->scanline == (epoch->crtc[LC_CURSOR_ROW_START] & 31))
            epoch->cursorvisible = 1;
    }
}

static void
epoch_vram_write(uint32_t addr, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    // epoch_log("epoch_vw: %x %x\n", addr, val);
    addr &= EPOCH_MASK_VRAM;
    /* [Cloud server only, DO NOT USE!!!] 3477-S bring-up: the ANK (SBCS) font
       area lives in VRAM at 0x38000; log the first writes so we can tell whether
       anything ever loads it. */
    if (addr >= EPOCH_VRAM_SBCS) {
        static int n = 0;
        if (n < 24) {
            pclog("3477: vram ank %05X <- %02X\n", addr, val);
            n++;
        }
    }
    epoch->vram[addr] = val;
    epoch->fullchange = 3;
}
static uint8_t
epoch_vram_read(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    addr &= EPOCH_MASK_VRAM;
    return epoch->vram[addr];
}

/* The adapter's RAM has no parity bits, so a write into it with parity
   updating enabled latches a parity error.  The 5535-M's 0x47 test (F8246)
   depends on it: it walks 0xAC000-0xFB000 and then requires port 0xA0 bit 0
   to be set, to test the error latch and its clear through port 0xA2. */
static void
epoch_parity_device_write(epoch_t *epoch)
{
    /* A write into the adapter area with parity updating enabled latches an
       error, but only while the adapter's I/O is disabled. The 5535-M's 0x47
       test walks that area with its I/O still off (F8246 out A0h,40h; F8252
       the walk) and then requires the latch; the display routines only touch
       it after enabling the I/O (FC70:00AC), and both IPLs' memory walks
       require the latch to be clear, so it must not latch there. */
    /* [3477-S] The 3477-S powers up with the CRTC I/O open (crtioenabled is
       forced to 1 so display works - see epoch_reset), but its IPL still runs
       the 0x47-style memory walk that waits for this latch to fire at
       F800:0796 (in A0h, test bit 0, loop back while clear).  Gating the latch
       on !crtioenabled would make that walk unsatisfiable on the 3477-S, so
       allow it there as well. */
    if (epoch->parityenabled && (!epoch->crtioenabled || epoch_is_3477))
        epoch->parityerror = 1;
}

/* The video RAM is only decoded for word accesses (the ROM's video code uses
   word stores), but the parity latch still reacts to byte stores: the 5535-M's
   0x47 test stores bytes across 0xAC000-0xFB000 (F8251/F8259) and then requires
   port 0xA0 bit 0 to be set so it can test the latch and its clear. */
static void
epoch_vram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    epoch_parity_device_write(epoch);
}

static void
epoch_vram_writew(uint32_t addr, uint16_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    epoch_parity_device_write(epoch);
    // epoch_log("%04X:%04X epoch_vww: %x, val %x DS %x SI %x ES %x DI %x %x\n", cs >> 4, cpu_state.pc, addr, val,DS,SI,ES,DI, epoch->crtc[LC_INTERLACE_AND_SKEW]);
    // epoch_log("%04X:%04X epoch_vww: %x, val %x cm %x\n", cs >> 4, cpu_state.pc, addr, val, epoch->crtmode);
    cycles -= video_timing_write_w;
    addr -= 0xA0000;
    if (!(epoch->crtmode & 0x02) && !(epoch->font24)) {
        uint32_t toaddr, bitnum;

        /* rw one word with 9 bits */
        /* virtual: 20000h (0010b, 0011b) -> real: 00001h (0000b, 0001b) */
        toaddr = getaddr_9bitword(addr);
        bitnum   = toaddr & 7;
        epoch_vram_write(toaddr, val & 0xff, epoch);
        
        /* get 9th bit */
        toaddr >>= 3;
        toaddr += 0x20000; /* real: C0000h */
        val >>= 15;
        val <<= bitnum;

        /* mask to update one bit */
        val |= (epoch_vram_read(toaddr, epoch) & (~(1 << bitnum)));
        epoch_vram_write(toaddr, val, epoch);
        // epoch_log("%x %x\n", toaddr, val);
        // epoch_log("%x %x\n", addr, val);
    } else {/* is graphic mode */
        epoch_vram_write(addr, val & 0xff, epoch);
        epoch_vram_write(addr + 1, val >> 8, epoch);
    }
    // epoch_log("%x %x\n", addr, val);
}

static uint16_t
epoch_vram_readw(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    cycles -= video_timing_read_w;
    addr -= 0xA0000;
    // epoch_log("%04X:%04X epoch_vrw: %x cm %x\n", cs >> 4, cpu_state.pc, addr, epoch->crtmode);
    if (!(epoch->crtmode & 0x02) && !(epoch->font24)) {
        uint16_t ret;
        uint32_t bitnum;
        uint32_t toaddr;
        /* rw one word with 9 bits */
        /* virtual: 20000h (0010b, 0011b) -> real: 00001h (0000b, 0001b) */
        toaddr = getaddr_9bitword(addr);
        bitnum   = toaddr & 7;
        ret = epoch_vram_read(toaddr, epoch);
        /* get 9th bit */
        toaddr >>= 3;
        toaddr += 0x20000; /* real: C0000h */
        ret |= (epoch_vram_read(toaddr, epoch) << (8 + 7 - bitnum)) & 0x8000;
        return ret;
        // return epoch_vram_read(addr, epoch) | (epoch_vram_read(addr + 1, epoch) << 8);
    } else {/* is graphic mode */
        return epoch_vram_read(addr, epoch) | (epoch_vram_read(addr + 1, epoch) << 8);
    }
}


/* Index into the display RAM: the 3477-S window shows the plane port 0x400
   points at, everyone else gets the single flat 4 KB block. */
static uint32_t
epoch_cram_index(epoch_t *epoch, uint32_t addr)
{
    addr &= EPOCH_MASK_CRAM;
    if (epoch_is_3477)
        addr += (uint32_t) (epoch->dispbank & 3) << 12;
    return addr;
}

static void
epoch_cram_write(uint32_t addr, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    epoch->cram[epoch_cram_index(epoch, addr)] = val;
    epoch->fullchange = 3;
    // epoch_log("cw %04X:%04X %04X %02X\n", cs >> 4, cpu_state.pc, addr, val);
}
static void
epoch_cram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    // epoch_log("epoch_cram_writeb: Write to %x, val %x\n", addr, val);
    cycles -= video_timing_write_b;
    epoch_cram_write(addr, val, epoch);
}
static void
epoch_cram_writew(uint32_t addr, uint16_t val, void *priv)
{
    static int dbg_n = 0;

    /* [Cloud server only, DO NOT USE!!!] 3477-S bring-up: show the first
       text/attribute writes at 0xE0000. */
    if (dbg_n < 8) {
        pclog("3477: cram wr %05X <- %04X\n", addr, val);
        dbg_n++;
    }
    // epoch_log("epoch_cram_writ   ew: Write to %x, val %x\n", addr, val);
    epoch_t *epoch = (epoch_t *) priv;
    cycles -= video_timing_write_w;
    epoch_cram_write(addr, val & 0xff, epoch);
    epoch_cram_write(addr + 1, val >> 8, epoch);
}

static uint8_t
epoch_cram_read(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    return epoch->cram[epoch_cram_index(epoch, addr)];
}
static uint8_t
epoch_cram_readb(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    cycles -= video_timing_read_b;
    return epoch_cram_read(addr, epoch);
}

static uint16_t
epoch_cram_readw(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    cycles -= video_timing_read_w;
    return epoch_cram_read(addr, epoch) | (epoch_cram_read(addr + 1, epoch) << 8);
}

/* The board decodes 640 KB, but only the installed RAM is backed by the
   memory buffer: a larger address is open bus, so it must neither reach
   mem_read_ram()/mem_write_ram() nor be given a parity bit. */
static int
epoch_ram_present(uint32_t addr)
{
    return addr < (mem_size << 10);
}

/* Parity state of one byte; addresses outside RAM are not tracked. */
static void
epoch_parity_mark(epoch_t *epoch, uint32_t addr, int stale)
{
    if (epoch_ram_present(addr) && (addr < 0xA0000)) {
        const uint8_t mask = (uint8_t) (1 << (addr & 7));
        if (stale)
            epoch->paritybad[addr >> 3] |= mask;
        else
            epoch->paritybad[addr >> 3] &= (uint8_t) ~mask;
    }
}

static int
epoch_parity_check(epoch_t *epoch, uint32_t addr)
{
    if (epoch_ram_present(addr) && (addr < 0xA0000))
        return (epoch->paritybad[addr >> 3] >> (addr & 7)) & 1;
    return 0;
}

static uint8_t
epoch_parity_readb(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    int newerr = 0;
    if (epoch->parityenabled != 0) {
        if (epoch_parity_check(epoch, addr) || epoch->parityforce) {
            newerr = (epoch->parityerror == 0);
            epoch->parityerror = 1;
            if (epoch_is_3477)
                epoch->parityforce = 0; /* [AB] the arm is one-shot */
        }
    }
    if ((epoch->parityerror != 0) && (epoch->parityenabled != 0)) {
        epoch->parityerroraddr = (addr >> 16) & 0xFF; /* X000:0 */
        epoch_log("%04X:%04X perror at %0X\n", cs >> 4, cpu_state.pc, addr);
        if (nmi_mask && (!epoch_is_3477 || newerr)) {
            nmi_raise();
        }
    }
    if (epoch->lowmemorydisabled) {
        if ((addr >= 0x40000) && (addr < 0xA0000)) {
            epoch_log("%04X:%04X mrerror at %0X\n", cs >> 4, cpu_state.pc, addr);
            return EPOCH_INVALIDACCESS8;
        }
    }
    if (!epoch_ram_present(addr))
        return 0xff;
    return mem_read_ram(addr, priv);
}

static uint16_t
epoch_parity_readw(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    int newerr = 0;
    if (epoch->parityenabled != 0) {
        if (epoch_parity_check(epoch, addr) || epoch_parity_check(epoch, addr + 1) || epoch->parityforce) {
            newerr = (epoch->parityerror == 0);
            epoch->parityerror = 1;
            if (epoch_is_3477)
                epoch->parityforce = 0; /* [AB] the arm is one-shot */
        }
    }
    if ((epoch->parityerror != 0) && (epoch->parityenabled != 0)) {
        epoch->parityerroraddr = (addr >> 16) & 0xFF;
        epoch_log("%04X:%04X perror at %X\n", cs >> 4, cpu_state.pc, addr);
        if (nmi_mask && (!epoch_is_3477 || newerr)) {
            nmi_raise();
        }
    }
    if (epoch->lowmemorydisabled) {
        if ((addr >= 0x40000) && (addr < 0xA0000)) {
            epoch_log("%04X:%04X mrerror at %X\n", cs >> 4, cpu_state.pc, addr);
            return EPOCH_INVALIDACCESS16;
        }
    }
    if (!epoch_ram_present(addr) || !epoch_ram_present(addr + 1))
        return 0xffff;
    return mem_read_ramw(addr, priv);
}

/* A write stores a fresh parity bit unless parity updating is off or the error
   test mode is set, in which case the byte keeps a stale one. */
static void
epoch_parity_writeb(uint32_t addr, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    // epoch_log("%04X:%04X mw %0X\n", cs >> 4, cpu_state.pc, addr);
    epoch_parity_mark(epoch, addr, (epoch->parityenabled == 0) || epoch->parityforce);
    if (epoch->lowmemorydisabled) {
        if ((addr >= 0x40000) && (addr < 0xA0000)) {
            epoch_log("%04X:%04X mwerror at %X\n", cs >> 4, cpu_state.pc, addr);
            return;
        }
    }
    if (!epoch_ram_present(addr))
        return;
    mem_write_ram(addr, val, priv);
}
static void
epoch_parity_writew(uint32_t addr, uint16_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    epoch_parity_mark(epoch, addr, (epoch->parityenabled == 0) || epoch->parityforce);
    epoch_parity_mark(epoch, addr + 1, (epoch->parityenabled == 0) || epoch->parityforce);
    if (epoch->lowmemorydisabled) {
        if ((addr >= 0x40000) && (addr < 0xA0000)) {
            epoch_log("%04X:%04X mwerror at %X\n", cs >> 4, cpu_state.pc, addr);
            return;
        }
    }
    if (!epoch_ram_present(addr) || !epoch_ram_present(addr + 1))
        return;
    mem_write_ramw(addr, val, priv);
}

/* Software timer (IRQ7): a free-running timer controlled via port 0xA6.
   Writing 1 arms it (first fire after EPOCH_SWTIMER_FIRST), writing 0 from
   IRQ handler acknowledges. This callback raises IRQ7 and re-arms itself. */
static void
epoch_swtimer_callback(void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    picint(EPOCH_IRQ7_BIT);
    timer_advance_u64(&epoch->swtimer_timer, EPOCH_SWTIMER_PERIOD * TIMER_USEC);
}

/* POST stage restart: a soft reset, so RAM (and the stage register the IPL
   left behind) survives while the processor comes out of protected mode. */
static void
epoch_pulse_timer(UNUSED(void *priv))
{
    softresetx86();
}

/* True while the CPU is inside the IRQ7 (software timer) handler, whose
   acknowledge must not disarm the free-running timer; only a write that
   clears the arm bit from normal code (e.g. the FE67:0191 DMA-test cleanup)
   stops it. The 5550 acknowledges at FC00:0619; the 5535-M's handler counts
   the interrupts at F800:0EDC and acknowledges at F800:0EF5. */
static int
epoch_in_swtimer_isr(void)
{
    if (epoch_is_5535)
        return ((cs >> 4) == 0xf800) && (cpu_state.pc >= 0x0edc) && (cpu_state.pc <= 0x0efb);

    return ((cs >> 4) == 0xfc00) && (cpu_state.pc >= 0x600) && (cpu_state.pc <= 0x61f);
}

/* val bit 0 set = arm, clear = acknowledge (from the IRQ7 handler) or disarm. */
static void
epoch_swtimer_write(epoch_t *epoch, uint8_t val)
{
    if (val & 1) {
        epoch->swtimer = 1;
        /* Armed: start the free-running timer with a short first delay. */
        timer_set_delay_u64(&epoch->swtimer_timer, EPOCH_SWTIMER_FIRST * TIMER_USEC);
    } else if (!epoch_in_swtimer_isr()) {
        epoch->swtimer = 0;
        timer_disable(&epoch->swtimer_timer);
    }
}

static uint8_t
epoch_misc_in(uint16_t port, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    uint8_t           ret = 0xff;

    switch (port) {
        case 0x400:
            /* Display-RAM plane select (3477-S).  The burn-in reads it back
               and keeps bit 5, which therefore reads set. */
            ret = epoch->dispbank | 0x20;
            break;
/*
I/O A0h R:
xxxx xxx1: Memory or parity error?
xxxx xx1x: Test switch (physical switch on the rear panel)
x1xx xxxx: Sense DREQ? (only used in diag test)
0xxx xxxx: Coprocessor installed?
*/
        case 0xA0:
            ret = 0;
            if (epoch->parityenabled)
                ret |= epoch->parityerror & 1;
            /* Bit 3: the IPL asked for a restart at a POST stage, so this
               reset came from its own HLT (F812D tests it to reach the stage
               dispatcher at F8003 instead of the power-on path). */
            if (epoch_is_5535 && epoch->warmstart)
                ret |= 0x08;
            if (fpu_type == FPU_NONE)
                ret |= 0x80;
            for (int i = 0; i < 2; i++)
                if (dma_get_drq(i))
                    ret |= 0x40;
            if (epoch->testsw)
                ret |= 0x02;
            /* [AB] Test-harness switch: the 3477-S test suite contains a step
               that waits for the operator to flip the rear-panel test switch.
               NOTE (measured, do not retry): toggling this bit on a *read
               counter* or on a *timer* cannot satisfy that wait loop.  86Box
               detects the tight I/O polling loop and fast-forwards it: the
               port handler is not called once per repeat (the log shows a
               single call with a huge '*N repeats*' count) and the refresh
               timers do not advance either.  So the loop is unsatisfiable in
               the model and the ROS must be run with the switch reading off
               (testsw=0, where the wait exits immediately).  This hook is kept
               for the 5535-M path only and is off by default.
               that waits for the operator to flip the rear-panel test switch
               (A0h bit1 transition: F8DE:02F5 waits for 0, F8DE:02FB waits for
               1).  With TESTSW_AUTO=1 the bit toggles every 20000 reads of A0h
               so that wait loop gets its transition while ordinary branch
               logic (a handful of reads) still sees a stable value.  Off by
               default - it simulates an operator action, it is not a hardware
               behaviour. */
            {
                static int auto_sw = -1;
                if (auto_sw < 0)
                    auto_sw = (getenv("TESTSW_AUTO") != NULL);
                if (auto_sw) {
                    /* [3477-S] Rear-panel test switch.  The ROM waits for a
                       0 -> 1 transition *observed inside its own poll loop* at
                       F8DE:02F5 (loop while bit1 == 1) then F8DE:02FB (loop
                       while bit1 == 0).  A global read counter is no use - the
                       ROM has already read A0h thousands of times before it
                       gets there, so bit1 must be delivered per-visit: hold 0
                       for the first few polls of that loop, then release to 1,
                       and reset when the ROM moves on. */
                    static int  sw_phase = 0;
                    static long sw_polls = 0;
                    if ((CS == 0xF8DE) && (cpu_state.pc >= 0x02F5) && (cpu_state.pc <= 0x02FF)) {
                        if (++sw_polls > 8)
                            sw_phase = 1;
                    } else {
                        sw_polls = 0;
                        sw_phase = 0;
                    }
                    if (sw_phase)
                        ret |= 0x02;
                }
            }
            /* Bit 5 is an undocumented status that reads set. The 5535-M's
               IPL requires it after the timer-2 test and retries that test
               while it is set, so a clear bit turns the test into a hard
               failure. */
            if (epoch_is_5535)
                ret |= 0x20;
            /* [AB] Bit 4 has to read set on the 3477-S: its IPL's memory walk
               polls it at F807A8 and, when clear, jumps through the restart
               vector at FFF3:000C -- which is why the ROS restarted once per
               pass.  The value is undocumented; the burn-in never reads it. */
            if (epoch_is_3477)
                ret |= 0x10;
            pclog("3477: in 0xA0 = %02X (pc=%04X:%04X)\n", ret, CS, cpu_state.pc);
            break;
        case 0xA1: /* High address where memory error occured */
            ret = epoch->parityerroraddr;
            break;
/*
I/O A2h R: 
xxxx 0x0x: Color 16 CRT
xxxx 1x0x: Mono 24 CRT ?
xxxx xx1x: Mono 16 CRT
xx1x xxxx: No hard drive
x1xx xxxx: No floppy drive
1xxx xxxx: No (bootable?) hard drive
*/
        case 0xA2:
            if (epoch_is_3477) {
                /* The 3477-S IPL identifies its display as (A2 & 0xA0) == 0x80
                   for the 24-dot panel; the 5550 values (bit 5 set) never match. */
                ret = epoch->font24 ? 0x88 : 0x22;
                /* A2h 的 bit7 同时是"无错误"标志：RAM 测试校验失败时 ROM 会
                   out 0xA2 锁存错误，随后 in 0xA2 / not al / test al,0x80 判断
                   是否出错（0x6E5）；而面板 ID 测试用 and al,0xA0 / cmp al,0x80
                  （0x6D44）——两处用法推出：错误锁存时 bit7 应为 0。 */
                if (epoch->parityerror & 1)
                    ret &= ~0x80;
            } else if (epoch->font24)
                ret = 0xA8; /* Mono 24 */
            else
                ret = 0xA2; /* Mono 16 */
            break;
/*
I/O A3h R: 
xxxx x111: Main RAM 256 KB
xxxx x110: Main RAM 384 KB
xxxx x100: Main RAM 512 KB
xxxx x000: Main RAM 640 KB
xxxx 1xxx: Serial port 3f8h
xxx1 xxxx: Serial port 2f8h
*/
        case 0xA3:
            ret = 0x08;
            if (mem_size < 384)
                ret |= 0x07;
            else if (mem_size < 512)
                ret |= 0x06;
            else if (mem_size < 640)
                ret |= 0x04;
            break;
        case 0xA4:
            ret = 0;
            break;
        case 0xA5:
            ret = 0x08; /* Bit 3: Keyboard connected? */
            break;
        case 0xA6: /* Software timer status (bit 0 = armed) */
            ret = epoch->swtimer;
            break;
        case 0xA7:
            ret = 0;
            break;
        /* 5535-M memory configuration block. */
        case 0x284: /* block-present bitmap, low half */
        case 0x285: /* block-present bitmap, high half */
            ret = (port & 1) ? (epoch->mem_present >> 8) : (epoch->mem_present & 0xff);
            break;
        case 0x286: /* status low: the two control bits mirrored into the nibble */
            ret = (epoch->mem_ctrl & 0x80 ? 0x0a : 0x00) |
                  (epoch->mem_ctrl & 0x40 ? 0x05 : 0x00);
            break;
        case 0x287: /* status high: the last 0x288 value's high half */
            ret = (epoch->mem_cfg >> 8) & 0xff;
            break;
        // case 0x164:
        //     switch (epoch->fontcard.portdata) {
        //         case 0x16A:
        //             ret = 0xFD;
        //             break;
        //         case 0x168:
        //             ret = 0xFE;
        //             break;
        //     }
        //     break;
        /* 5535-M status/diagnostic register file. The data port returns
           the selected register; the IPL only uses the low nibble. */
        case 0x368: /* register index */
            ret = epoch->diag_index;
            break;
        case 0x36A: /* register data */
            ret = epoch->diag_reg[epoch->diag_index & 0x1f];
            break;
        default:
            break;
    }
    epoch_log("%04X:%04X I/O In %02X: %02X\n", cs >> 4, cpu_state.pc, port, ret);
    return ret;
}

static void
epoch_misc_out(uint16_t port, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    pit_intf_t *pit_intf = &pit_devs[0];

    // dev->regs[port & 0x0007] = val;
    // epoch_log("%04X:%04X I/O Out %02X: %02X\n  AX=%04X BX=%04X CX=%04X DX=%04X ES=%04X DI=%04X DS=%04X SI=%04X\n",
    //      cs >> 4, cpu_state.pc, port, val, AX, BX, CX, DX, ES, DI, DS, SI);
    if(port != 0x44)
        epoch_log("%04X:%04X I/O Out %02X: %02X\n", cs >> 4, cpu_state.pc, port, val);

    switch (port) {
        case 0x400:
            epoch->dispbank = val; /* 3477-S display-RAM plane select */
            break;
        case 0x44:
            for (uint8_t i = 0; i < 3; i++) {
                pit_intf->set_gate(pit_intf->data, i, val & 1);
            }
            break;
/*
I/O A0h W:
x1xx xxxx: Enable parity update (Disable this -> Write data -> Enable this -> Read data, causes NMI)
1xxx xxxx: Enable NMI check
*/
        case 0xA0:
            nmi_mask = val & 0x80;
            /* [AB-TEMP-PROBE] A0h 写入总表 + 停摆循环签名(00/01) */
            if (getenv("A0ALL")) {
                static int cnt[256];
                static int n00 = 0;
                if (cnt[val]++ < 3)
                    pclog("A0ALL val=%02X pc=%04X:%04X\n", val, cs >> 4, cpu_state.pc);
                if (((val == 0x00) || (val == 0x01)) && (n00 < 30)) {
                    n00++;
                    pclog("A0STALL #%d val=%02X pc=%04X:%04X\n", n00, val, cs >> 4, cpu_state.pc);
                }
            }
            if (epoch_is_3477) {
                /* The 3477-S burn-in never writes bit 6: it uses bit 3 as
                   "parity updating enabled" - it writes a byte with the bit
                   clear (parity stays stale), sets the bit, reads the byte back
                   and requires A0h bit 0 to latch.  Same disable/write/enable/
                   read recipe the 5535-M's 0x47 test uses with bit 6. */
                epoch->parityenabled = (val & 0x08) ? 1 : 0;
                /* Bit 0 of the write side stays ignored.  The manual only
                   documents bits 6/7 here, and an undriven bit 0 reads back
                   clear - exactly what the BIOS's "no parity error" path wants
                   (a set bit 0 sends it to the error exit).  The System ROS's
                   00h/01h writes to A0h are its blink-and-park loop, not a
                   disable/write/enable/read parity recipe. */
                /* [AB] Bit 5 arms one parity error for the next memory read: the
                   System ROS stores with 88h/A8h (bit 7 set) and
                   loads the word back, requiring the A0h bit 0 latch and the NMI. */
                epoch->parityforce   = (val & 0x80) ? 1 : 0;
            } else {
                epoch->parityenabled = val & 0x40; /* 1 = Enable read/write with parity */
                /* Bit 3: parity error test mode (see epoch_t). */
                epoch->parityforce = (val & 0x08) != 0;
            }
            /* The 5535-M arms and acknowledges the software timer here
               (out A0h, 41h / 40h) instead of on port 0xA6. */
            if (epoch_is_5535)
                epoch_swtimer_write(epoch, val);
            break;
        case 0xA1:
            /* Diagnostics LED / POST checkpoint (used by debug card module).
               Value 0x4F is also the POST restart command: it resets the
               processor, but only after the IPL has named the stage it wants
               to resume at (F9128/other checkpoints write other values). */
            if (epoch_is_5535 && epoch->resetarmed && (val == 0x4f)) {
                epoch->resetarmed = 0;
                epoch->warmstart  = 1;
                timer_set_delay_u64(&epoch->pulse_timer, EPOCH_RESET_PULSE * TIMER_USEC);
            }
            break;
        case 0xA2:
            /* Reset memory error bit. Only the reported error is cleared, not
               the test mode, so an IPL can acknowledge and re-check. */
            epoch->parityerror = 0;
            break;
        case 0xA6:
            /* Software timer (IRQ7) arm/ack. Bit 0 set arms it; a write
               with bit 0 clear and no other bit is either the IRQ handler's
               acknowledge (which must keep the timer running so the ROM BAT
               can count five IRQ7s) or a disarm from normal code. Only the
               latter stops it. */
            if (val & 1)
                epoch_swtimer_write(epoch, 1);
            else if ((val & 0xfe) == 0)
                epoch_swtimer_write(epoch, 0);
            break;
        // case 0x160 ... 0x16A:
        //     mem_mapping_enable(&epoch->fontcard.map);
        //     epoch->fontcard.portdata = port;
        //     break;
        /* 5535-M memory configuration block. The IPL writes 0x288 with a word,
           and 0x280/0x286 are write-only there. */
        case 0x280:
            /* Clearing the block back to its power-on state: the IPL writes
               zero here before checking that 0x286 reads the default, so
               the control bits a previous POST left in 0x282/0x28A must
               not survive this write - the check halts the machine on a
               warm start otherwise. */
            epoch->mem_ctrl = 0xc0;
            epoch->mem_cfg  = 0;
            break;
        case 0x282:
        case 0x28A:
            epoch->mem_ctrl = val;
            break;
        case 0x288:
            epoch->mem_cfg = (epoch->mem_cfg & 0xff00) | val;
            break;
        case 0x289:
            epoch->mem_cfg = (epoch->mem_cfg & 0x00ff) | (val << 8);
            break;
        /* Only the 5550's IPL drives this window; the 5535-M always has
           0x40000-0x9FFFF and its memory-size walk requires that. */
        case 0x310 ... 0x312:
            if (!epoch_is_5535) {
                epoch->lowmemorydisabled = 0;
                epoch_log("Low memory enabled\n");
            }
            break;
        case 0x314 ... 0x316:
            if (!epoch_is_5535) {
                epoch->lowmemorydisabled = 1;
                epoch_log("Low memory disabled\n");
            }
            break;
        /* [3477-S] Write-only memory-window selectors (see epoch_t). */
        case 0x390:
            epoch->map390 = val;
            break;
        case 0x1B2:
            epoch->reg1b2 = val;
            break;
        case 0x2C2:
            epoch->reg2c2 = val;
            break;
        case 0x368: /* register index */
            epoch->diag_index = val;
            break;
        case 0x36A: /* register data */
            epoch->diag_reg[epoch->diag_index & 0x1f] = val & 0x0f;
            /* Register 0x10 holds the POST stage to resume at. A non-zero
               write arms the restart command (the IPL then writes 0x4F to
               0xA1); the IPL's own register-file test ends by writing
               zero here, so it leaves the command disarmed. */
            if (epoch_is_5535 && (epoch->diag_index & 0x1f) == 0x10)
                epoch->resetarmed = (val & 0x0f) != 0;
            break;
        default:
            break;
    }
}
typedef struct epochkbd_t {
    int want_irq;
    int blocked;
    uint8_t irq_state;

    uint8_t pa;
    uint8_t pb;
    uint8_t clk_hold;
    uint8_t key_waiting;
    int8_t kbd_readdata_step;
    int mouse_enabled;
    int mouse_queue_num;
    int kbc_resetstep;
    uint8_t mouse_queue[4];

    pc_timer_t send_delay_timer;
} epochkbd_t;

static uint8_t key_queue[16]; /* buffer in the keyboard */
static int     key_queue_start = 0;
static int     key_queue_end   = 0;

static void
kbd_epoch_poll(void *priv)
{
    epochkbd_t *kbd = (epochkbd_t *) priv;

    timer_advance_u64(&kbd->send_delay_timer, 250 * TIMER_USEC);

    if (kbd->pb & 0x04) /* controller is sending something to keyboard */
        return;

    if (kbd->kbd_readdata_step < 16)
        kbd->kbd_readdata_step++;
    
    if (!(kbd->pb & 0x08)) /* keyboard interrupt is disabled */
        return;

    if (kbd->want_irq && kbd->kbd_readdata_step >= 16) {
        if (kbd->blocked) {
            kbd->want_irq  = 0;
            kbd->irq_state = 1;
            picint(EPOCH_IRQ3_BIT);
            epoch_kbdlog("epochkbd: kbd_poll(): keyboard_xt : take IRQ\n");
        } else {
            kbd->pa                = kbd->key_waiting;
            kbd->kbd_readdata_step = -4;
            kbd->blocked           = 1;
        }
    }

    if (!kbd->blocked) {
        if (kbd->mouse_queue_num > 0) {
            kbd->mouse_queue_num--;
            kbd->key_waiting = kbd->mouse_queue[kbd->mouse_queue_num];
            epoch_kbdlog("epochkbd: reading %02X from the mouse queue at %i\n", kbd->key_waiting, kbd->mouse_queue_num);
            kbd->want_irq = 1;
        } else if (key_queue_start != key_queue_end) {
            kbd->key_waiting = key_queue[key_queue_start];
            epoch_kbdlog("epochkbd: reading %02X from the key queue at %i\n",
                         kbd->key_waiting, key_queue_start);
            key_queue_start = (key_queue_start + 1) & 0x0f;
            kbd->want_irq   = 1;
        }
    }
}

static void
kbd_epoch_adddata_process(uint16_t val, void (*adddata)(uint16_t val))
{
    // uint8_t num_lock = 0;
    // uint8_t shift_states = 0;

    if (!adddata)
        return;

    // keyboard_get_states(NULL, &num_lock, NULL, NULL);
    // shift_states = keyboard_get_shift() & STATE_LSHIFT;

    // /* If NumLock is on, invert the left shift state so we can always check for
    //    the the same way flag being set (and with NumLock on that then means it
    //    is actually *NOT* set). */
    // if (num_lock)
    //     shift_states ^= STATE_LSHIFT;

    adddata(val);
}

static void
kbd_epoch_adddata(uint16_t val)
{
    key_queue[key_queue_end] = val;
    epoch_log("epochkbd: %02X added to key queue at %i\n",
            val, key_queue_end);
    key_queue_end = (key_queue_end + 1) & 0x0f;
}

static void
kbd_adddata_ex(uint16_t val)
{
    if (val < 0x100)
        kbd_epoch_adddata_process(val, kbd_epoch_adddata);
}

static void
kbd_epoch_softreset(void *priv)
{
    epochkbd_t *kbd = (epochkbd_t *) priv;
    key_queue_start = key_queue_end = 0;
    kbd->kbd_readdata_step          = 16;
    kbd->want_irq                   = 0;
    kbd->blocked                    = 0;
    kbd->clk_hold                   = 0;
    kbd->mouse_queue_num            = 0;
    kbd->blocked                    = 0;
    kbd->irq_state                  = 0;
    kbd->pa                         = 0;
    picintc(EPOCH_IRQ3_BIT);
}
/*
I/O 61h W:
xxxx xxx1: ? (used by kbd interrupt in DOS)
xxxx xx1x: Beep
xxxx x1xx: Hold clock line low (used to reset kbd)
xxxx 1xxx: Enable kbd?
xxx1 xxxx: Send data from system to kbd
1xxx xxxx: Clear buffer?
*/
static void
kbd_write(uint16_t port, uint8_t val, void *priv)
{
    epochkbd_t *kbd = (epochkbd_t *) priv;
    uint8_t     new_clock;
    epoch_kbdlog("%04X:%04X epochkbd: Port %02X out: %02X BX: %04x", cs >> 4, cpu_state.pc, port, val, BX);
    epoch_kbdlog(" Clk: %x %x\n", kbd->blocked, kbd->clk_hold);

    switch (port) {
        case 0x60: /* Diagnostic Output */
            if ((kbd->pb & 0x10) && ((val & 0xf0) == 0xd0)) {
                kbd->mouse_enabled = 1;
                epoch_kbdlog("epochkbd: Mouse is enabled.\n");
            }
            break;
        case 0x61: /* Keyboard Control Register (aka Port B) */
            kbd->pb = val;

            if (val == 0x00) {
                /* The IPL writes 0x00 to port B at the start of the keyboard
                   test to disable the keyboard. Discard any leftover keydata
                   (e.g. the tail of a Ctrl+Alt+Del scancode that the ROM did 
                   not consume before reset) so the clock-poll that follows 
                   sees an idle line. */
                key_queue_start        = key_queue_end = 0;
                kbd->kbd_readdata_step = 16;
                kbd->want_irq          = 0;
            }

            if ((val & 0x18) == 0x08) {
                new_clock = !(val & 0x04);
                /* Trigger kbd reset after the clk line is reset to a high level */
                if (kbd->clk_hold && new_clock) {
                    kbd->mouse_enabled = 0;
                    kbd_epoch_softreset(kbd);
                    epoch_kbdlog("epochkbd: Starting keyboard reset sequence.\n");
                    /* IBM 5556 keyboards send three bytes of identification code in the reset sequence.
                       DOS K3.44 supports following IDs: 
                        A5xx1x: ? (standard layout)
                        A6xx1x: AIUEO RPQ Keyboard
                        B1xx1x, B2xx1x: Type 3, 4 Keyboard (1972 JIS layout)
                        B5xx1x: ? (standard layout)
                        AA----: XT keyboard? (The IPL supports it, but the Japanese DOS doesn't) 
                    */
                    kbd_epoch_adddata(0xA5);
                    kbd_epoch_adddata(0x00);
                    kbd_epoch_adddata(0x10);
                    keyboard_set_in_reset(1);
                }
            } else if ((val & 0x18) == 0x18) {
                new_clock = !(val & 0x04);
                if (kbd->clk_hold && new_clock) {
                    kbd_epoch_softreset(kbd);
                    epoch_kbdlog("epochkbd: Starting mouse reset sequence.\n");
                    kbd_epoch_adddata(0x00);
                }
            }

            if (kbd->pb & 0x08)
                kbd->clk_hold = !!(kbd->pb & 0x04);

            timer_process();
            speaker_update();
            // if(!speaker_enable && (val & 2)) epoch_log("Buz!\n");
            speaker_gated  = val & 2;
            speaker_enable = val & 2;

            if (speaker_enable)
                was_speaker_enable = 1;
            pit_devs[0].set_gate(pit_devs[0].data, TIMER_CTR_2, val & 2);

            if (val & 0x80) {
                /* clear buffer */
                kbd->pa      = 0;
                kbd->blocked = 0;
                kbd->irq_state = 0;
                picintc(EPOCH_IRQ3_BIT);
            }

            break;
        default:
            break;
    }
}

/*
I/O 61h R
xxxx xx1x: Beep input?
xxxx x1xx: KB -CLK?
xxxx 1xxx: KB -DATA?
*/
static uint8_t
kbd_read(uint16_t port, void *priv)
{
    epochkbd_t *kbd = (epochkbd_t *) priv;
    uint8_t     ret = 0;

    switch (port) {
        case 0x60: /* Keyboard Data Register  (aka Port A) */
            ret = kbd->pa;
            if (keyboard_get_in_reset() && (ret == 0x10))
                keyboard_set_in_reset(0);
            break;

        case 0x61: /* Keyboard Control Register (aka Port B) */
            ret = kbd->pb & 0xf0; /* reset sense bit */
            ret |= 0x0c;          /* reset sense bit */
            if (kbd->kbd_readdata_step < 16) {
                ret &= 0xf3;
                switch (kbd->kbd_readdata_step) {
                    case -4:
                        ret |= 0x0c; /* start bit */
                        break;
                    case -3:
                        ret |= 0x0c; /* start bit */
                        break;
                    case -2:
                        ret |= 0x08; /* start bit */
                        break;
                    case -1:
                        ret |= 0x0c; /* start bit */
                        break;
                    default:
                        if (!!(kbd->kbd_readdata_step & 1))
                            ret |= 0x04;
                        if ((kbd->key_waiting >> (kbd->kbd_readdata_step >> 1)) & 1)
                            ret |= 0x08;
                        break;
                }
                epoch_kbdlog("  rdata step: %d %x %x\n", kbd->kbd_readdata_step, ret & 0x08, ret & 0x04);
            }

            /* Bit 1: Timer 2 (Speaker) out state */
            if (pit_devs[0].get_outlevel(pit_devs[0].data, TIMER_CTR_2) && speaker_enable)
                ret &= 0xfd; /* 1111 1101 */
            else
                ret |= 0x02;

            break;
        default:
            break;
    }
    // if (port != 0x61)
        epoch_kbdlog("%04X:%04X epochkbd: Port %02X in : %02X pb: %02x CX: %04x\n", cs >> 4, cpu_state.pc, port, ret, kbd->pb, CX);
    return ret;
}

static void
kbd_reset(void *priv)
{
    epochkbd_t *kbd = (epochkbd_t *) priv;

    kbd_epoch_softreset(kbd);
    kbd->pb = 0x00;

    keyboard_scan      = 1;
    kbd->mouse_enabled = 0;
}

static void *
kbd_init(const device_t *info)
{
    epochkbd_t *kbd;

    kbd = (epochkbd_t *) calloc(1, sizeof(epochkbd_t));

    io_sethandler(0x0060, 4,
                  kbd_read, NULL, NULL, kbd_write, NULL, NULL, kbd);
    keyboard_send = kbd_adddata_ex;
    kbd_reset(kbd);

    timer_add(&kbd->send_delay_timer, kbd_epoch_poll, kbd, 1);

    keyboard_set_table(scancode_set8a);
    keyboard_mode = 0x8a;

    return kbd;
}

static void
kbd_close(void *priv)
{
    epochkbd_t *kbd = (epochkbd_t *) priv;

    /* Stop the timer. */
    timer_disable(&kbd->send_delay_timer);
    mouse_close();

    /* Disable scanning. */
    keyboard_scan = 0;
    keyboard_send = NULL;

    io_removehandler(0x0060, 2,
                     kbd_read, NULL, NULL, kbd_write, NULL, NULL, kbd);

    free(kbd);
}

static const device_t kbc_epoch_device = {
    .name          = "IBM 5556 Keyboard and Mouse",
    .internal_name = "kbc_epoch",
    .flags         = 0,
    .local         = 0,
    .init          = kbd_init,
    .close         = kbd_close,
    .reset         = kbd_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

/*
The IBM 5550 DOS K3.44 comes with a mouse driver
for the M1 and P1 keyboards that have a built-in mouse adapter.
The mouse adapter for the expansion slot was also available,
but it may require the bundled driver. (not confirmed)

IBM 5550 DOS K3.4 Mouse Driver (Int 0Bh and 33h)
61h <- 1st kbd indata
[si+4Ch] status <- 2nd kbd indata
1xxx xxxx negative dX
x1xx xxxx bit 9 of dX
xx1x xxxx negative dY
xxx1 xxxx bit 9 of dY
xxxx x1xx cursor moved
xxxx xx1x right button
xxxx xxx1 left button
[si+4Dh] previous button status <- [si+4Ch]
[si+4Eh] dX <- 3rd kbd indata
[si+50h] dY <- 4th kbd indata
*/
static int
epoch_mouse_poll(void *priv)
{
    epochkbd_t *kbd = (epochkbd_t *) priv;
    int         dat = mouse_get_buttons_ex();
    int         dx, dy;

    if (!kbd->mouse_enabled)
        return 0;
    
    if (kbd->mouse_queue_num > 0)
        return 0;

    if (kbd->pb & 0x10)
        return 0;

    dat &= 0x03;
    if (!mouse_moved()) {
        kbd->mouse_queue[1] = 0x61;
        kbd->mouse_queue[0] = dat;
        kbd->mouse_queue_num = 2;
    } else {
        dat |= 0x04;
        mouse_subtract_x(&dx, NULL, -512, 511, 0);
        mouse_clear_x();
        if (dx < 0)
            dat |= 0x20;
        if (dx & 0x100)
            dat |= 0x10;
        mouse_subtract_y(&dy, NULL, -512, 511, 0, 0);
        mouse_clear_y();
        if (dy < 0)
            dat |= 0x80;
        if (dy & 0x100)
            dat |= 0x40;
        epoch_log("Mouse moved %x %x %x\n", dat, dx, dy);
        kbd->mouse_queue[3] = 0x61;
        kbd->mouse_queue[2] = dat;
        kbd->mouse_queue[1] = dx & 0xff;
        kbd->mouse_queue[0] = dy & 0xff;
        kbd->mouse_queue_num = 4;
    }
    return 0;
}

static void
epoch_nvr_time_set(uint8_t *regs, struct tm *tm)
{
    regs[epoch_nvr_SECOND1]  = tm->tm_sec % 10;
    regs[epoch_nvr_SECOND10] = (tm->tm_sec / 10);
    regs[epoch_nvr_MINUTE1]  = (tm->tm_min % 10);
    regs[epoch_nvr_MINUTE10] = (tm->tm_min / 10);
    regs[epoch_nvr_HOUR1]  = (tm->tm_hour % 10);
    regs[epoch_nvr_HOUR10] = (tm->tm_hour / 10);
    regs[epoch_nvr_WEEKDAY] = tm->tm_wday;
    regs[epoch_nvr_DAY1]    = (tm->tm_mday % 10);
    regs[epoch_nvr_DAY10]   = (tm->tm_mday / 10);
    regs[epoch_nvr_MONTH1]  = ((tm->tm_mon + 1) % 10);
    regs[epoch_nvr_MONTH10] = ((tm->tm_mon + 1) / 10);
    regs[epoch_nvr_YEAR1]   = (tm->tm_year % 10);
    regs[epoch_nvr_YEAR10]  = ((tm->tm_year % 100) / 10);
}

/* Get the chip time. */
#define nibbles(a) (regs[(a##1)] + 10 * regs[(a##10)])
static void
epoch_nvr_time_get(uint8_t *regs, struct tm *tm)
{
    tm->tm_sec = nibbles(epoch_nvr_SECOND);
    tm->tm_min = nibbles(epoch_nvr_MINUTE);
    tm->tm_hour = nibbles(epoch_nvr_HOUR);
    tm->tm_wday = regs[epoch_nvr_WEEKDAY];
    tm->tm_mday = nibbles(epoch_nvr_DAY);
    tm->tm_mon  = (nibbles(epoch_nvr_MONTH) - 1);
    tm->tm_year = (nibbles(epoch_nvr_YEAR));
}

/* This is called every second through the NVR/RTC hook. */
static void
epoch_nvr_tick(nvr_t *nvr)
{
    struct tm tm;
    if (!(nvr->regs[epoch_nvr_CONTROL] & 0x40)) {
        /* Get the current time from the internal clock. */
        nvr_time_get(&tm);
        /* Update registers with current time. */
        epoch_nvr_time_set(nvr->regs, &tm);
    }
}

static void
epoch_nvr_start(nvr_t *nvr)
{
    struct tm tm;
    /* Initialize the internal and chip times. */
    if (time_sync & TIME_SYNC_ENABLED) {
        /* Use the internal clock's time. */
        nvr_time_get(&tm);
        epoch_nvr_time_set(nvr->regs, &tm);
    } else {
        /* Set the internal clock from the chip time. */
        epoch_nvr_time_get(nvr->regs, &tm);
        nvr_time_set(&tm);
    }
}

/* Write to one of the chip registers. */
static void
epoch_nvr_write(uint16_t port, uint8_t val, void *priv)
{
    epoch_t *epoch      = (epoch_t *) priv;
    int addr = 0;

    switch (port) {
        case 0x360:
            epoch->nvrctrl = val;
            addr = val & 0xf;
            if (val & 0x20) { /* Write */
                if (addr >= 0x8 && (epoch->nvr.regs[addr] != val))
                    nvr_dosave = 1;
                epoch->nvr.regs[addr] = epoch->nvrdata;
            }
            break;
        case 0x361:
            if(epoch->nvrctrl & 0x40) /* Is the access enabled? */
                epoch->nvrdata = val;
            break;
    }
    // epoch_log("%04X:%04X I/O Out %02X: %02X\n", cs >> 4, cpu_state.pc, port, val);
}

/* Read from one of the chip registers. */
static uint8_t
epoch_nvr_read(uint16_t port, void *priv)
{
    const epoch_t *epoch = (epoch_t *) priv;
    
    switch (port) {
        case 0x360:
            return epoch->nvrctrl;
            break;
        case 0x361:
            if(epoch->nvrctrl & 0x40) /* Is the access enabled? */
                return (epoch->nvr.regs[(epoch->nvrctrl & 0xf)]);
            break;
    }
    return EPOCH_INVALIDACCESS8;
}

/* Reset the RTC registers to a default state. */
static void
epoch_nvr_reset(nvr_t *nvr)
{
    /* Clear the NVRAM. */
    memset(nvr->regs, 0xff, nvr->size);
    /* Reset the RTC registers. */
    memset(nvr->regs, 0x00, 0xc);
    nvr->regs[epoch_nvr_CONTROL] = 0;
}

static void
epoch_nvr_init(epoch_t *epoch)
{
    nvr_t* nvr = &epoch->nvr;
    /* This is machine specific. */
    nvr->size = 17;
    nvr->irq  = -1;
    /* Set up any local handlers here. */
    nvr->reset = epoch_nvr_reset;
    nvr->start = epoch_nvr_start;
    nvr->tick  = epoch_nvr_tick;
    /* Initialize the actual NVR. */
    nvr_init(nvr);
    io_sethandler(0x0360, 2,
                  epoch_nvr_read, NULL, NULL, epoch_nvr_write, NULL, NULL, epoch);
}

/* Attribute to DAC index for monochrome text: index 0 = background, 2 =
   normal, 3 = highlighted.  The 5550's monochrome CRT draws light text (the
   adapter's amber pair 6/62) on black; the 5535-M's LCD is the light one, so
   its indices are the nearest DAC entries to a warm off-white background,
   that colour inverted, and the mid-tone of those two. */
static uint8_t ibm5550_attr_mono[16] = 
{
	0,6,6,62,0,0,0,0,0,0,0,0,0,0,0,0
};

static uint8_t ibm5535_attr_mono[16] = 
{
	55,8,8,7,0,0,0,0,0,0,0,0,0,0,0,0
};

// static uint8_t ps55_attr_color[16] = /* for video mode 0eh color character */
// {
// 	0x00,0x38,0x24,0x3c,0x12,0x3a,0x36,0x3e,0x09,0x39,0x2d,0x3d,0x1b,0x3b,0x3f,0x3f
// };

/* 12-bit DAC color palette for IBMJ Display Adapter with color monitor */
static uint8_t ps55_palette_color[64][3] = {
    { 0x00, 0x00, 0x00 },    { 0x00, 0x00, 0x2A },    { 0x00, 0x2A, 0x00 },    { 0x00, 0x2A, 0x2A },
    { 0x2A, 0x00, 0x00 },    { 0x2A, 0x00, 0x2A },    { 0x2A, 0x2A, 0x00 },    { 0x2A, 0x2A, 0x2A },
    { 0x00, 0x00, 0x15 },    { 0x00, 0x00, 0x3F },    { 0x00, 0x2A, 0x15 },    { 0x00, 0x2A, 0x3F },
    { 0x2A, 0x00, 0x15 },    { 0x2A, 0x00, 0x3F },    { 0x2A, 0x2A, 0x15 },    { 0x2A, 0x2A, 0x3F },
    { 0x00, 0x15, 0x00 },    { 0x00, 0x15, 0x2A },    { 0x00, 0x3F, 0x00 },    { 0x00, 0x3F, 0x2A },
    { 0x2A, 0x15, 0x00 },    { 0x2A, 0x15, 0x2A },    { 0x2A, 0x3F, 0x00 },    { 0x2A, 0x3F, 0x2A },
    { 0x00, 0x15, 0x15 },    { 0x00, 0x15, 0x3F },    { 0x00, 0x3F, 0x15 },    { 0x00, 0x3F, 0x3F },
    { 0x2A, 0x15, 0x15 },    { 0x2A, 0x15, 0x3F },    { 0x2A, 0x3F, 0x15 },    { 0x2A, 0x3F, 0x3F },
    { 0x15, 0x00, 0x00 },    { 0x15, 0x00, 0x2A },    { 0x15, 0x2A, 0x00 },    { 0x15, 0x2A, 0x2A },
    { 0x3F, 0x00, 0x00 },    { 0x3F, 0x00, 0x2A },    { 0x3F, 0x2A, 0x00 },    { 0x3F, 0x2A, 0x2A },
    { 0x15, 0x00, 0x15 },    { 0x15, 0x00, 0x3F },    { 0x15, 0x2A, 0x15 },    { 0x15, 0x2A, 0x3F },
    { 0x3F, 0x00, 0x15 },    { 0x3F, 0x00, 0x3F },    { 0x3F, 0x2A, 0x15 },    { 0x3F, 0x2A, 0x3F },
    { 0x15, 0x15, 0x00 },    { 0x15, 0x15, 0x2A },    { 0x15, 0x3F, 0x00 },    { 0x15, 0x3F, 0x2A },
    { 0x3F, 0x15, 0x00 },    { 0x3F, 0x15, 0x2A },    { 0x3F, 0x3F, 0x00 },    { 0x3F, 0x3F, 0x2A },
    { 0x15, 0x15, 0x15 },    { 0x15, 0x15, 0x3F },    { 0x15, 0x3F, 0x15 },    { 0x15, 0x3F, 0x3F },
    { 0x3F, 0x15, 0x15 },    { 0x3F, 0x15, 0x3F },    { 0x3F, 0x3F, 0x15 },    { 0x3F, 0x3F, 0x3F }
};

static video_timings_t timing_epoch_vid = 
{ .type = VIDEO_ISA, .write_b = 8, .write_w = 8, .write_l =  16, .read_b = 8, .read_w = 8, .read_l = 16 };

static void
epoch_reset(void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    epoch->parityerror = 0;
    epoch->parityforce = 0;
    memset(epoch->paritybad, 0, 0xA0000 >> 3);
    epoch->parityenabled = 1;
    epoch->dispbank      = 0;
    epoch->map390        = 0;
    epoch->reg1b2        = 0;
    epoch->reg2c2        = 0;
    /* A hard reset is a power-on: only the soft reset a POST restart command
       issues (see epoch_misc_out's 0x36A/0xA1) leaves this set. */
    epoch->warmstart  = 0;
    epoch->resetarmed = 0;
    timer_disable(&epoch->pulse_timer);
    /* The 0x310-0x316 window un-hides the 0x40000-0x9FFFF range, and only the
       5550's IPL uses it.  The 5535-M always has that memory, and its IPL
       requires the first unbacked block to be at 0xA0000 (memory-size walk at
       F871C, error 0x6111), so it must not be hidden there.  The 3477-S also
       has 640 KB and never touches the window, and its RAM pattern test
       (F05CC, run out of the copy at 0200:0) walks 0x10000-0x7FFFF expecting
       the range to be backed. */
    epoch->lowmemorydisabled = (epoch_is_5535 || epoch_is_3477) ? 0 : 1;
    /* The 5550's IPL opens the CRTC I/O and video output by writing 3D2h; the
       3477-S never does, so it must power up open or every CRTC write is
       dropped and nothing is drawn.  [Cloud server only, DO NOT USE!!!] */
    epoch->crtioenabled = epoch_is_3477 ? 1 : 0;
    /* The IPL reads 0x286 before programming the block and requires the two
       control bits to read as set then (FCE5B), so those power up set. */
    epoch->mem_ctrl = 0xc0;
    epoch->mem_cfg  = 0;
    epoch->swtimer = 0;
    timer_disable(&epoch->swtimer_timer);
    /* The video and attribute RAMs stay mapped (only the video output is gated
       by crtioenabled); unlike a disable, this also lets the parity latch see
       accesses to the device area, which the 5535-M's IPL tests. */
    // mem_mapping_disable(&epoch->cmap);
    // mem_mapping_disable(&epoch->vmap);
    // epoch->attrc[LV_CURSOR_COLOR]    = 0x0f;                   /* cursor color */
    epoch->crtc[LC_HORIZONTAL_TOTAL] = 103;    /* Horizontal Total */
    epoch->crtc[LC_VERTICAL_TOTAL]  = 26;    /* Vertical Total (These two must be set before the timer starts.) */
    epoch->crtmode = 0;
    epoch->vram_display_mask = EPOCH_MASK_CRAM;
    // epoch->plane_mask = 1;
    epoch->oddeven = 0;
    // epoch->memaddr_latch                  = 0;
    // epoch->attrc[LV_CURSOR_CONTROL]  = 0x13; /* cursor options */
    // epoch->attr_palette_enable       = 0;    /* disable attribute generator */

    // epoch->attrc[LV_PAS_STATUS_CNTRL] = 0;
    // epoch->attrc[LV_PANNING] = 0;
    
    /* Set internal color palette registers */
    for (uint16_t i = 0; i < 16; i++) {
        epoch->egapal[i] = epoch_is_5535 ? ibm5535_attr_mono[i] : ibm5550_attr_mono[i];
    }
    /* Set color palette for video output */
    for (uint16_t i = 0; i < 64; i++) {
        epoch->vgapal[i].r = ps55_palette_color[i & 0x3F][0];
        epoch->vgapal[i].g = ps55_palette_color[i & 0x3F][1];
        epoch->vgapal[i].b = ps55_palette_color[i & 0x3F][2];
        epoch->pallook[i]  = makecol32((epoch->vgapal[i].r & 0x3f) * 4, (epoch->vgapal[i].g & 0x3f) * 4, (epoch->vgapal[i].b & 0x3f) * 4);
    }

    epoch_log("epoch_reset done.\n");
}

/* ---- Optional kanji font card ------------------------------------------
   The card holds a page ROM behind a 32 KB RAM window at F0000h; the guest
   selects a page and reads glyphs from this window. Page 0 is the page the
   IPL sums: after clearing F000:0000 it wants the 32 KB 16-bit word sum to
   be 1. Note what is *not* done here: IBMBIO also compares F000:0000..000F
   with a constant in its own segment (0540:8083), but a real card's page 0
   holds the pointer table's entries for cells 0..7 there, so that compare
   cannot pass on real 5535-M and is intended to protect glyphs[0..7]. */
static void
epoch_fontcard_fix_page(uint8_t *page, uint32_t index)
{
    uint32_t i;
    uint16_t sum;
    uint8_t  first = page[0];

    if (index != 0)
        return;

    page[0] = 0x00;
    sum     = 0;
    for (i = 0; i < EPOCH_FONTCARD_WIN_SIZE; i += 2)
        sum += (uint16_t) (page[i] | (page[i + 1] << 8));
    page[0] = first;
    if (sum == 1)
        return;

    epoch_log("font card: page 0 sum %04X, repaired\n", sum);
    page[0] = 0x00;
    sum     = 0;
    for (i = 0; i < EPOCH_FONTCARD_WIN_SIZE; i += 2)
        sum += (uint16_t) (page[i] | (page[i + 1] << 8));
    /* Replace the last word so the whole page sums to exactly 1. */
    sum -= (uint16_t) (page[EPOCH_FONTCARD_WIN_SIZE - 2]
                     | (page[EPOCH_FONTCARD_WIN_SIZE - 1] << 8));
    sum  = (uint16_t) (1 - sum);
    page[EPOCH_FONTCARD_WIN_SIZE - 2] = sum & 0xff;
    page[EPOCH_FONTCARD_WIN_SIZE - 1] = sum >> 8;
    page[0] = first;
}

static void
epoch_fontcard_set_page(epoch_t *epoch, uint32_t page)
{
    uint32_t offset;

    if (epoch->fontcard_win == NULL)
        return;
    if (epoch->fontcard_pages != 0)
        page %= epoch->fontcard_pages;
    epoch->fontcard_page = page;
    offset               = page * EPOCH_FONTCARD_WIN_SIZE;
    if ((epoch->fontcard_rom != NULL) && (page < epoch->fontcard_pages))
        memcpy(epoch->fontcard_win, &epoch->fontcard_rom[offset],
               EPOCH_FONTCARD_WIN_SIZE);
    else
        memset(epoch->fontcard_win, 0, EPOCH_FONTCARD_WIN_SIZE);
    /* Only the 5535's IBMBIO wants page 0's word sum repaired; on the 3477 the
       page is a real ROM image and patching it would corrupt the last word. */
    if (!epoch_is_3477)
        epoch_fontcard_fix_page(epoch->fontcard_win, page);
}

static uint8_t
epoch_fontcard_readb(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    return epoch->fontcard_win[addr & (EPOCH_FONTCARD_WIN_SIZE - 1)];
}

static uint16_t
epoch_fontcard_readw(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    addr &= EPOCH_FONTCARD_WIN_SIZE - 1;
    return (uint16_t) (epoch->fontcard_win[addr]
        | (epoch->fontcard_win[(addr + 1) & (EPOCH_FONTCARD_WIN_SIZE - 1)] << 8));
}

static void
epoch_fontcard_writeb(uint32_t addr, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    addr &= EPOCH_FONTCARD_WIN_SIZE - 1;
    /* Writing 2 to the first byte is IBMBIO's "serve" command; re-serving
       page 0 keeps the signature comparison it does right after reading a
       stable place. */
    if ((addr == 0) && (val == 0x02)) {
        epoch_fontcard_set_page(epoch, 0);
        return;
    }
    /* The low 16 bytes are the page latch: the guest selects a page by writing
       the page number at the matching offset (seg0540:7F0D, "mov [bx],bl" with
       bx == bl). Requiring val == addr keeps unrelated memory writes out (the
       IPL's walk only touches F000:0000 and F000:7FFF). */
    if ((addr <= 15) && (val == addr)) {
        epoch_fontcard_set_page(epoch, val);
        return;
    }
    epoch->fontcard_win[addr] = val;
}

static void
epoch_fontcard_writew(uint32_t addr, uint16_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    addr &= EPOCH_FONTCARD_WIN_SIZE - 1;
    epoch->fontcard_win[addr] = val & 0xff;
    epoch->fontcard_win[(addr + 1) & (EPOCH_FONTCARD_WIN_SIZE - 1)] = val >> 8;
}

/* 0x164 reports whether the card accepted the last command, and which one:
   the IPL reads it right after writing 0x160 and needs 0xFF, while IBMBIO
   looks for 0xFD after a 0x16A write and 0xFE after a 0x168 write. */
static uint8_t
epoch_fontcard_in(uint16_t port, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    uint8_t  ret   = 0xff;

    if (port == 0x164) {
        switch (epoch->fontcard_data) {
            case 0x168:
                ret = 0xfe;
                break;
            case 0x16a:
                ret = 0xfd;
                break;
            default:
                ret = 0xff;
                break;
        }
    }

    return ret;
}

static void
epoch_fontcard_out(uint16_t port, uint8_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    /* The last port touched decides what a 0x164 read reports. */
    epoch->fontcard_data = port;
    switch (port) {
        case 0x160:
            /* Command/index; its bank semantics are still unknown,
               so the current page simply stays served. */
            break;
        default:
            break;
    }
}

static void
epoch_fontcard_init(epoch_t *epoch)
{
    FILE       *fp;
    size_t      fsize;
    uint32_t    rom_size;
    const char *path;

    rom_size = epoch_is_3477 ? EPOCH_FONTCARD_ROM_SIZE_3477 : EPOCH_FONTCARD_ROM_SIZE;

    if (epoch_is_3477) {
        path = EPOCH_FONTCARD_ROM_PATH_3477;
    } else if (epoch_is_5535) {
        path = epoch->fontcard_isnew ? EPOCH_FONTCARD_ROM_PATH_NEW
                                     : EPOCH_FONTCARD_ROM_PATH_OLD;
    } else {
        switch (epoch->fontcard_charset) {
            case EPOCH_FONTCARD_CHARSET_KJ16:
                path = EPOCH_FONTCARD_ROM_PATH_K16;
                break;
            case EPOCH_FONTCARD_CHARSET_KJ24:
                path = EPOCH_FONTCARD_ROM_PATH_K24;
                break;
            case EPOCH_FONTCARD_CHARSET_KJ52:
                path = EPOCH_FONTCARD_ROM_PATH_K52;
                break;
            case EPOCH_FONTCARD_CHARSET_HANS:
                path = EPOCH_FONTCARD_ROM_PATH_P24;
                break;
            default:  /* No font card installed. */
                return;
        }
    }

    fp = rom_fopen(path, "rb");
    if (fp == NULL)
        return; /* No font card installed. */
    epoch->fontcard_rom = calloc(1, rom_size);
    fsize = fread(epoch->fontcard_rom, 1, rom_size, fp);
    fclose(fp);
    if (fsize < EPOCH_FONTCARD_WIN_SIZE) {
        free(epoch->fontcard_rom);
        epoch->fontcard_rom = NULL;
        return;
    }
    epoch->fontcard_pages = (uint32_t) (fsize / EPOCH_FONTCARD_WIN_SIZE);
    epoch->fontcard_win   = calloc(1, EPOCH_FONTCARD_WIN_SIZE);

    mem_mapping_add(&epoch->fontcard_map, EPOCH_FONTCARD_WIN_ADDR,
                    EPOCH_FONTCARD_WIN_SIZE, epoch_fontcard_readb,
                    epoch_fontcard_readw, NULL,
                    epoch_is_3477 ? NULL : epoch_fontcard_writeb,
                    epoch_is_3477 ? NULL : epoch_fontcard_writew, NULL, NULL,
                    MEM_MAPPING_EXTERNAL, epoch);
    /* The 3477's ROM is on the system board behind port 0x3E3, not on the
       optional card, so it has no 0x160 command interface. */
    if (!epoch_is_3477)
        io_sethandler(0x160, 0x000b, epoch_fontcard_in, NULL, NULL,
                      epoch_fontcard_out, NULL, NULL, epoch);
    epoch_fontcard_set_page(epoch, 0);
    epoch_log("font card: %s, %u pages installed\n", path, epoch->fontcard_pages);
}

/* The 3477-S has 60 KB of RAM visible from 0xE1000 to 0xEFFFF.  The burn-in
   fills and verifies it (0xE4000-0xE7FFF with the AB54/54AB pattern, 0xE1000
   and 0xE8000 with bit patterns), and with the range left unmapped the reads
   return open bus so those tests record a failure.  The 4 KB shown at 0xE0000
   is the windowed view of the first display plane.  [Cloud server only,
   DO NOT USE!!!] */
static uint8_t
epoch_hiram_readb(uint32_t addr, void *priv)
{
    return ((epoch_t *) priv)->hiram[(addr - 0xE1000) & 0xFFFF];
}
static uint16_t
epoch_hiram_readw(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    return (uint16_t) (epoch->hiram[(addr - 0xE1000) & 0xFFFF]
        | (epoch->hiram[(addr - 0xE1000 + 1) & 0xFFFF] << 8));
}
static void
epoch_hiram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    ((epoch_t *) priv)->hiram[(addr - 0xE1000) & 0xFFFF] = val;
}
static void
epoch_hiram_writew(uint32_t addr, uint16_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    epoch->hiram[(addr - 0xE1000) & 0xFFFF]         = val & 0xff;
    epoch->hiram[(addr - 0xE1000 + 1) & 0xFFFF]     = val >> 8;
}

static void *
epoch_init(UNUSED(const device_t *info))
{
    epoch_t *epoch  = calloc(1, sizeof(epoch_t));
    epoch->testmode = device_get_config_int("testmode");

    /* The 5535-M has only the 16-dot monochrome LCD model, so it has
       no font selection; the 5550's CRT is either 16- or 24-dot. */
    if (epoch_is_5535) {
        epoch->fontcard_isnew = device_get_config_int("font");
    } else {
        epoch->font24 = device_get_config_int("model");
        if (epoch_is_3477) {
            /* The 3477-S is a 24-dot panel, but device_get_config_int()
               returns only the default (0) in this legacy machine init, and
               font24=0 makes A2h report 0x22 so the IPL takes the wrong
               branch. */
            epoch->font24 = 1;
        }
        epoch->fontcard_charset = device_get_config_int("font");
        epoch->fontcard_enabled = (epoch->fontcard_charset != EPOCH_FONTCARD_CHARSET_NONE);
    }

    video_inform(VIDEO_FLAG_TYPE_NONE, &timing_epoch_vid);
    video_update_timing();

    epoch->dispontime        = 1000ull << 32;
    epoch->dispofftime       = 1000ull << 32;
    // epoch->changedvram       = calloc(1,  (EPOCH_MASK_VRAMPLANE + 1) >> 9); /* XX000h */

    if (epoch->font24)
        epoch->pixelclock = EPOCH_PIXELCLOCK24;
    else
        epoch->pixelclock = EPOCH_PIXELCLOCK16;

    epoch->vram              = calloc(1, 256* 1024);
    // for(int i=0;i<256*1024;i++) /* for debug */
    //     epoch->vram[i] = 0xff;
    epoch->cram              = calloc(1, EPOCH_SIZE_CRAM_ALL);
    epoch->hiram             = calloc(1, 0x10000);
    epoch->paritybad         = calloc(1, 0xA0000 >> 3);
    /* Both the 5550 and the 5535-M can carry the kanji font card; the 5550 has
       a switch for it, the 5535-M always has one. The 3477-S has its ROS FONT
       ROM on the system board. It is installed only when its image is present. */
    if (epoch_is_5535 || epoch_is_3477 || epoch->fontcard_enabled)
        epoch_fontcard_init(epoch);

    epoch->epochconst = (uint64_t) ((cpuclock / epoch->pixelclock) * (double) (1ull << 32));

    mem_mapping_add(&epoch->cmap, 0xE0000, 0x1000, epoch_cram_readb, epoch_cram_readw, NULL,
        epoch_cram_writeb, epoch_cram_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, epoch);
    if (epoch_is_3477) {
        /* The 3477-S shares 0xC0000 with its display controller, so leave that
           page out of the VRAM window. [Cloud server only, DO NOT USE!!!] */
        mem_mapping_add(&epoch->vmap, 0xA0000, 0x20000, NULL, epoch_vram_readw, NULL,
            epoch_vram_writeb, epoch_vram_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, epoch);
        mem_mapping_add(&epoch->vmap2, 0xC1000, 0x1F000, NULL, epoch_vram_readw, NULL,
            epoch_vram_writeb, epoch_vram_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, epoch);
        mem_mapping_add(&epoch->hiram_map, 0xE1000, 0xF000, epoch_hiram_readb, epoch_hiram_readw, NULL,
            epoch_hiram_writeb, epoch_hiram_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, epoch);
    } else {
        mem_mapping_add(&epoch->vmap, 0xA0000, 0x40000, NULL, epoch_vram_readw, NULL,
            epoch_vram_writeb, epoch_vram_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, epoch);
    }
    mem_mapping_add(&epoch->paritymap, 0, 0xA0000, epoch_parity_readb, epoch_parity_readw, NULL,
        epoch_parity_writeb, epoch_parity_writew, NULL, NULL, MEM_MAPPING_CACHE, epoch);

    io_sethandler(0x03d0, 0x0020, epoch_inb, epoch_inw, NULL, epoch_outb, epoch_outw, NULL, epoch);

    io_sethandler(0x44, 0x0001,
                  epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
    io_sethandler(0xA0, 0x0008,
                  epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
    /* The 3477-S selects the display-RAM plane shown at 0xE0000 here. */
    if (epoch_is_3477)
        io_sethandler(0x400, 0x0001,
                      epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
    /* The 3477-S also decodes its CRTC block at 0x2D0-0x2EF. */
    if (epoch_is_3477)
        io_sethandler(0x02d0, 0x0020,
                      epoch_mirror_inb, epoch_mirror_inw, NULL,
                      epoch_mirror_outb, epoch_mirror_outw, NULL, epoch);
    // io_sethandler(0x160, 0x0010,
    //               epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);

    /* Only the 5535-M has the memory configuration block at 0x280-0x28A. */
    if (epoch_is_5535) {
        io_sethandler(0x280, 0x000B,
                      epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
        /* One 128 KB block per bitmap bit; the IPL walks the bits that are
           clear, so a present block is a zero bit. */
        epoch->mem_present = (uint16_t) ~((1u << (mem_size >> 7)) - 1u);
    }

    io_sethandler(0x310, 0x0008,
                  epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);

    /* [3477-S] Write-only memory-window selectors. */
    if (epoch_is_3477) {
        io_sethandler(0x390, 0x0001,
                      epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
        io_sethandler(0x1B2, 0x0001,
                      epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
        io_sethandler(0x2C2, 0x0001,
                      epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
    }

    /* Only the 5535-M's IPL uses the status register file at 0x368/0x36A. */
    if (epoch_is_5535)
        io_sethandler(0x368, 0x0003,
                      epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
    epoch_reset(epoch);
    
    timer_add(&epoch->timer, epoch_poll, epoch, 1);
    timer_add(&epoch->pulse_timer, epoch_pulse_timer, epoch, 0);
    timer_add(&epoch->swtimer_timer, epoch_swtimer_callback, epoch, 0);

    epoch_nvr_init(epoch);

    return epoch;
}

static void
epoch_close(void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;

    /* dump mem for debug */
#ifdef ENABLE_EPOCH_LOG
    FILE *fp;
    fp = fopen("epoch_cram.dmp", "wb");
    if (fp != NULL) {
        fwrite(epoch->cram, EPOCH_SIZE_CRAM, 1, fp);
        fclose(fp);
    }
    fp = fopen("epoch_vram.dmp", "wb");
    if (fp != NULL) {
        fwrite(epoch->vram, EPOCH_SIZE_VRAM, 1, fp);
        fclose(fp);
    }
    // fp = fopen("epoch_attrpal.dmp", "wb");
    // if (fp != NULL) {
    //     fwrite(epoch->attrc, 32, 1, fp);
    //     fclose(fp);
    // }
    fp = fopen("epoch_daregs.txt", "w");
    if (fp != NULL) {
            fprintf(fp, "3d8(crtmode) %02X\n", epoch->crtmode);
        // for (uint8_t i = 0; i < 0x10; i++)
        //     fprintf(fp, "3e1(ioctl) %02X: %4X %d\n", i, epoch->ioctl[i], epoch->ioctl[i]);
        // for (uint8_t i = 0; i < 0x20; i++)
        //     fprintf(fp, "3e3(fctl)  %02X: %4X %d\n", i, epoch->fctl[i], epoch->fctl[i]);
        for (uint8_t i = 0; i < 0x20; i++)
            fprintf(fp, "3e5(crtc)  %02X: %4X %d\n", i, epoch->crtc[i], epoch->crtc[i]);
        // for (uint8_t i = 0; i < 0x40; i++)
        //     fprintf(fp, "3e8(attr)  %02X: %4X %d\n", i, epoch->attrc[i], epoch->attrc[i]);
        // for (uint8_t i = 0; i < 0x10; i++)
        //     fprintf(fp, "3eb(gcr)   %02X: %4X\n", i, epoch->gdcreg[i]);
        // for (uint8_t i = 0; i < 0x20; i++) {
        //     fprintf(fp, "vp         %02X: %4X %4X %4X %4X\n", i, 
        //         epoch->crtc_vpreg[0 + i], epoch->crtc_vpreg[0x20 + i], epoch->crtc_vpreg[0x40 + i], epoch->crtc_vpreg[0x60 + i]);
        // }
        fclose(fp);
    }
    fp = fopen("ram_low.dmp", "wb");
    if (fp != NULL) {
        fwrite(ram, 0x40000, 1, fp);
        fclose(fp);
    }
    epoch_log("closed %04X:%04X AX=%04X BX=%04X CX=%04X DX=%04X ES=%04X DI=%04X DS=%04X SI=%04X\n",
          cs >> 4, cpu_state.pc, AX, BX, CX, DX, ES, DI, DS, SI);
    epoch_log("PIC IRR=%02X ISR=%02X IMR=%02X ICW1=%02X ICW2=%02X ICW3=%02X ICW4=%02X OCW2=%02X OCW3=%02X\n",
          pic.irr, pic.isr, pic.imr, pic.icw1, pic.icw2, pic.icw3, pic.icw4, pic.ocw2, pic.ocw3);
#endif
    /* The 3477 sampler caches this pointer; a hard reset frees the epoch here
       and a fresh one is made later, so drop the stale reference. */
    if (epoch_dbg_cur == epoch)
        epoch_dbg_cur = NULL;
    free(epoch->cram);
    free(epoch->hiram);
    free(epoch->vram);
    free(epoch->paritybad);
    free(epoch->fontcard_win);
    free(epoch->fontcard_rom);
    // free(epoch->changedvram);
    free(epoch);
}

static void
epoch_speed_changed(void *priv)
{
    epoch_t *epoch    = (epoch_t *) priv;
    epoch->epochconst = (uint64_t) ((cpuclock / epoch->pixelclock) * (double) (1ull << 32));
    epoch_recalctimings(epoch);
}

static void
epoch_force_redraw(void *priv)
{
    epoch_t *epoch      = (epoch_t *) priv;
    epoch->fullchange = changeframecount;
}

static void
pit_irq6_timer(int new_out, int old_out, UNUSED(void *priv))
{
    // epoch_log("%04X:%04X IRQ6 Timer triggered.\n", cs >> 4, cpu_state.pc);
    if (new_out && !old_out)
        picint(EPOCH_IRQ6_BIT);

    if (!new_out)
        picintc(EPOCH_IRQ6_BIT);
}

/* Counter 0 output: keep the DRAM refresh on the rising edge, and cascade a clock
   pulse to counter 1 on the falling edge (counter 0 -> counter 1 -> IRQ6). This
   gives the 50 ms software timer from the 1 ms counter 0 rate. */
static void
pit_ibm5550_ctr0(int new_out, int old_out, void *priv)
{
    pit_refresh_timer_xt(new_out, old_out, priv);

    if (!new_out && old_out)
        ctr_clock(priv, TIMER_CTR_1);
}

static pit_t *
pit_ibm5550_init(void)
{
    void *pit;

    pit_intf_t *pit_intf = &pit_devs[0];

            pit       = device_add(&i8253_device);
            *pit_intf = pit_classic_intf;

    pit_intf->data = pit;

    for (uint8_t i = 0; i < 3; i++) {
        pit_intf->set_gate(pit_intf->data, i, 1);
        pit_intf->set_using_timer(pit_intf->data, i, 1);
    }
    /* Counter 1 is clocked by counter 0's output (cascade), not by
       the PIT timer, so it must not use the internal timer clock. */
    pit_intf->set_using_timer(pit_intf->data, TIMER_CTR_1, 0);

    pit_intf->set_out_func(pit_intf->data, TIMER_CTR_1, pit_irq6_timer);
    pit_intf->set_out_func(pit_intf->data, TIMER_CTR_0, pit_ibm5550_ctr0);
    pit_intf->set_out_func(pit_intf->data, TIMER_CTR_2, pit_speaker_timer);
    pit_intf->set_load_func(pit_intf->data, TIMER_CTR_2, speaker_set_count);

    pit_intf->set_gate(pit_intf->data, TIMER_CTR_2, 0);

    return pit;
}

static const device_config_t epoch_5550_config[] = {
    // clang-format off
    {
        .name        = "font",
        .description = "Font",
        .type        = CONFIG_SELECTION,
        .default_int = EPOCH_FONTCARD_CHARSET_NONE,
        .selection   = {
            {
                .description = "None",
                .value = EPOCH_FONTCARD_CHARSET_NONE
            },
            {
                .description = "Japanese (Font 16)",
                .value = EPOCH_FONTCARD_CHARSET_KJ16
            },
            {
                .description = "Japanese (Font 24)",
                .value = EPOCH_FONTCARD_CHARSET_KJ24
            },
            {
                .description = "Japanese (5250 Emulation)",
                .value = EPOCH_FONTCARD_CHARSET_KJ52
            },
            {
                .description = "Simplified Chinese (Font 24)",
                .value = EPOCH_FONTCARD_CHARSET_HANS
            },
            { .description = "" }
        }
    },
    {
        .name        = "model",
        .description = "Model",
        .type        = CONFIG_SELECTION,
        .default_int = EPOCH_CONFIG_MONO24,
        .selection   = {
            {
                .description = "A (Font 16)",
                .value = EPOCH_CONFIG_MONO16
            },
            {
                .description = "B (Font 24)",
                .value = EPOCH_CONFIG_MONO24
            },
            { .description = "" }
        }
    },
    {
        .name        = "testmode",
        .description = "Test mode",
        .type        = CONFIG_BINARY,
        .default_int    = 0,
        .selection      = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t ibm5550_device = {
    .name          = "IBM 5551 System Unit",
    .internal_name = "ibm5550",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = epoch_init,
    .close         = epoch_close,
    .reset         = epoch_reset,
    .available     = NULL,
    .speed_changed = epoch_speed_changed,
    .force_redraw  = epoch_force_redraw,
    .config        = epoch_5550_config
};

/* -------------------------------------------------------------------------
   IBM 3477-S: the CPU talks to a separate keyboard/display controller through
   a shared RAM at segment C000 (mailboxes + POST log) plus 0x362/0x363
   doorbells.  This is a minimal model: a 4 KB RAM window with the "other
   side" mailbox already published (0x00AA at C000:001D / C000:0148).
   [Cloud server only, DO NOT USE!!!] Experimental, not validated.
   ------------------------------------------------------------------------- */
typedef struct i3477_shared_t {
    mem_mapping_t map;
    uint8_t       ram[0x1000];
    pc_timer_t    sample_timer;
} i3477_shared_t;

static uint8_t
i3477_ram_readb(uint32_t addr, void *priv)
{
    uint8_t v = ((i3477_shared_t *) priv)->ram[addr & 0xfff];

    pclog("3477: rdb C000:%04X = %02X\n", addr & 0xfff, v);
    return v;
}

static uint16_t
i3477_ram_readw(uint32_t addr, void *priv)
{
    i3477_shared_t *dev = (i3477_shared_t *) priv;
    uint16_t        v   = (uint16_t) (dev->ram[addr & 0xfff] | (dev->ram[(addr + 1) & 0xfff] << 8));

    pclog("3477: rdw C000:%04X = %04X\n", addr & 0xfff, v);
    return v;
}

static void
i3477_ram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    uint16_t off = addr & 0xfff;

    if ((off >= 0x1D) && (off <= 0x45)) /* the burn-in's failure-code log */
        pclog("3477: wrb C000:%04X = %02X bp=%04X (pc=%04X:%04X)\n", off, val, BP, CS, cpu_state.pc);
    else
        pclog("3477: wrb C000:%04X = %02X\n", off, val);
    ((i3477_shared_t *) priv)->ram[addr & 0xfff] = val;
}



static void
i3477_ram_writew(uint32_t addr, uint16_t val, void *priv)
{
    i3477_shared_t *dev = (i3477_shared_t *) priv;

    pclog("3477: wrw C000:%04X = %04X\n", addr & 0xfff, val);
    dev->ram[addr & 0xfff]         = val & 0xff;
    dev->ram[(addr + 1) & 0xfff]   = val >> 8;
}

static void
i3477_doorbell_out(uint16_t port, uint8_t val, void *priv)
{
    pclog("3477: doorbell %03X <- %02X (pc=%04X:%04X)\n", port, val, CS, cpu_state.pc);
}

/* A log's trailing byte is A5h minus the 8-bit sum of its whole range; the ROM
   recomputes it the same way after every entry it records. */
static uint8_t
i3477_log_checksum(const uint8_t *ram, uint32_t base, uint32_t len)
{
    uint8_t sum = 0;

    for (uint32_t i = 0; i < len; i++)
        sum += ram[base + i];

    return (uint8_t) (0xa5 - sum);
}

static void
i3477_shared_reset(void *priv)
{
    i3477_shared_t *dev = (i3477_shared_t *) priv;

    memset(dev->ram, 0, sizeof(dev->ram));
    /* The "other side" state.  Both words are read by the IPL with byte 0 in
       AH (big-endian compare), so a matched signature becomes 55 then AA and
       the ready mailbox word becomes 00 then AA. */
    dev->ram[0x0000] = 0x55;
    dev->ram[0x0001] = 0xaa;
    dev->ram[0x001d] = 0x00;
    dev->ram[0x001e] = 0xaa;
    dev->ram[0x0148] = 0x00;
    dev->ram[0x0149] = 0xaa;

    /* Unused log entries read 0xFF - the state a previously booted machine
       leaves, and what the burn-in tests an entry for emptiness with. */
    memset(&dev->ram[0x001f], 0xff, 0x0771 - 0x001f);
    dev->ram[0x0771] = i3477_log_checksum(dev->ram, 0x001d, 0x0754);
    memset(&dev->ram[0x014a], 0xff, 0x06ee - 0x014a);
    dev->ram[0x06ee] = i3477_log_checksum(dev->ram, 0x0148, 0x05a6);
}

static void
i3477_shared_close(void *priv)
{
    i3477_shared_t *dev = (i3477_shared_t *) priv;

    io_removehandler(0x0362, 2, NULL, NULL, NULL, i3477_doorbell_out, NULL, NULL, dev);
    free(dev);
}

static void
i3477_sample(void *priv)
{
    i3477_shared_t *dev = (i3477_shared_t *) priv;
    static int      n   = 0;
    static int      hb  = 0;

    /* Fast sampling (20us) but only log the first 60 PCs that are NOT the known
       permanent spin in the ROS delay loop, so the code between delays shows up. */
    if ((n < 60) && !((CS == 0xF9F6) && (cpu_state.pc >= 0x0443) && (cpu_state.pc <= 0x044B))) {
        pclog("3477: PC %04X:%04X ret %04X:%04X\n", CS, cpu_state.pc,
              mem_readw_phys(ss + SP + 6), mem_readw_phys(ss + SP + 4));
        n++;
    }

    /* Heartbeat every 2 s, unconditionally: shows where the ROM rests once the
       early sample budget is exhausted and whether it is still moving. */
    if (++hb >= 100000) {
        hb = 0;
        pclog("3477: HB PC %04X:%04X\n", CS, cpu_state.pc);

        /* What the display controller would draw right now. */
        if (epoch_dbg_cur != NULL) {
            epoch_t *e = epoch_dbg_cur;
            char     buf[17];
            for (int i = 0; i < 16; i++) {
                uint8_t c = e->cram[i * 2];
                buf[i]    = ((c >= 0x20) && (c < 0x7f)) ? (char) c : '.';
            }
            buf[16] = '\0';
            pclog("3477: DISP crtmode=%02X render=%s memaddr=%04X hdisp=%d cw=%d f24=%d "
                  "crtc00=%02X crtc01=%02X crtc06=%02X htotal=%d dispend=%d vtotal=%d "
                  "latch=%04X c0C=%02X c0D=%02X row=%d ankA=%02X%02X%02X%02X cram0='%s'\n",
                  e->crtmode,
                  (e->render == epoch_render_text) ? "text" :
                  (e->render == epoch_render_blank) ? "blank" :
                  (e->render == epoch_render_color_4bpp) ? "gfx4" : "other",
                  e->memaddr, e->hdisp, e->char_width, e->font24,
                  e->crtc[0x00], e->crtc[0x01], e->crtc[0x06], e->htotal, e->dispend, e->vtotal,
                  e->memaddr_latch, e->crtc[0x0C], e->crtc[0x0D], e->rowcount,
                  e->vram[EPOCH_VRAM_SBCS + 0x41 * 0x80 + 8 * 4 + 2], e->vram[EPOCH_VRAM_SBCS + 0x41 * 0x80 + 8 * 4 + 3],
                  e->vram[EPOCH_VRAM_SBCS + 0x41 * 0x80 + 16 * 4 + 2], e->vram[EPOCH_VRAM_SBCS + 0x41 * 0x80 + 16 * 4 + 3],
                  buf);
        }
    }

    timer_advance_u64(&dev->sample_timer, 20ULL * TIMER_USEC);
}

static void *
i3477_shared_init(const device_t *info)
{
    i3477_shared_t *dev = (i3477_shared_t *) calloc(1, sizeof(i3477_shared_t));

    i3477_shared_reset(dev);

    mem_mapping_add(&dev->map, 0xC0000, 0x1000, i3477_ram_readb, i3477_ram_readw, NULL,
                    i3477_ram_writeb, i3477_ram_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, dev);
    io_sethandler(0x0362, 2, NULL, NULL, NULL, i3477_doorbell_out, NULL, NULL, dev);

    timer_add(&dev->sample_timer, i3477_sample, dev, 0);
    timer_set_delay_u64(&dev->sample_timer, 50000ULL * TIMER_USEC);

    return dev;
}

static const device_t i3477_shared_device = {
    .name          = "IBM 3477-S shared RAM controller (EXPERIMENTAL)",
    .internal_name = "i3477_shared",
    .flags         = 0,
    .local         = 0,
    .init          = i3477_shared_init,
    .close         = i3477_shared_close,
    .reset         = i3477_shared_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

int
machine_xt_ibm5550_init(const machine_t *model)
{
    int ret;

    ret = bios_load_linear("roms/machines/ibm5550/ipl5550.rom",
                           0x000fc000, 16384, 0);

    if (bios_only || !ret)
        return ret;

    epoch_is_5535 = 0;

    device_add(&fdc_xt_5550_device);

    epochkbd_t *kbc = device_add(&kbc_epoch_device);
    
    pic_init();
    dma_init();
    pit_ibm5550_init();
    nmi_mask = 0;

    device_add(&ibm5550_device);

    device_add(&lpt_port_device);
    serial_t *uart = device_add(&ns8250_device);
    serial_setup(uart, 0x3f8, 1); /* Use IRQ 1 */

    if (mouse_type == MOUSE_TYPE_INTERNAL) {
        /* Tell mouse driver about our internal mouse. */
        mouse_reset();
        mouse_set_buttons(2);
        /* I don't know the actual polling speed, but
           a higher value may cause a conflict with the mouse driver  */
        mouse_set_sample_rate(30.0);
        mouse_set_poll(epoch_mouse_poll, kbc);
    }

    return ret;
}

/* [EXPERIMENT rombank] The 3477-S ROM is two COMPLETE 32 KB banks; each has its
   own reset vector (its offset 0x7FF0 = "ljmp 0xFFF5:0") and its own reset stub
   (its offset 0x7F50 = UMCS write + ljmp to its own entry), so whichever bank is
   mapped at F8000-FFFFF boots itself.  The machine's test-mode switch selects
   the bank: 1 = in-situ burn-in (upper), 0 = System ROS (lower).  F0000-F7FFF
   belongs to the kanji font card window, never to the ROM.
   [Cloud server only, DO NOT USE!!!] */
static uint8_t rombank[0x8000]; /* the 32 KB bank patched into rom[] */

/* IBM 3477-S: an 80186-12 based, cut-down colour Multistation with no hard disk
   and a 1.44 MB floppy, reusing the 5550 epoch display/keyboard/font stack.
   [Cloud server only, DO NOT USE!!!] First wiring attempt, not validated. */
int
machine_xt_ibm3477_init(const machine_t *model)
{
    int ret;

    /* [EXPERIMENT ipl1] The 3477-S ROM pair is a 64 KB 16-bit image: the upper
       32 KB (F8000-FFFFF) is the in-situ burn-in (test) program ("AZALEA-DBCS
       ... Insitu Burn-in Program Ver 1.2"), the lower half (F0000-F7FFF) is the
       IPL/BIOS ("...System ROS (c) Copyright IBM Corp. 1989"), and the image's
       reset vector at 0xFFFF0 is 0xEA 00 00 F5 FF = jmp F5FF:0000, i.e. into
       the BIOS.  Load the whole 64 KB so the BIOS runs first, as it does on the
       real machine (the burn-in itself documents that it assumes a normal BIOS
       has already programmed the 8259).  [Cloud server only, DO NOT USE!!!] */
    /* The lower half of the ROM (F0000-F7FFF) is a second, independent
       firmware program - the normal-boot IPL - and the burn-in never reads it:
       a read probe on that window logs zero reads over a whole run, and the
       run is byte-for-byte identical whether the lower half is present or not.
       (The one difference seen earlier, a second recording pass, came from
       this window shadowing the font card while the character generator was
       still blank.)  The burn-in is self-contained, so no burn-in failure code
       can be about the lower half.  [Cloud server only, DO NOT USE!!!] */
    ret = bios_load_linear("roms/machines/ibm3477/3477ipl.bin",
                           0x000f0000, 65536, 0);

    if (bios_only || !ret)
        return ret;

    epoch_is_5535 = 0;
    epoch_is_3477 = 1;

    device_add(&fdc_xt_5550_device);

    /* The 3477-S IPL drives the standard AT 8042 controller on 0x60/0x64
       (self-test 0xAA->0x55, interface tests 0xAB/0xA9, enable 0xAE, keyboard
       reset 0xFF, set-scancode 0xF0), not the 5550's epoch 0x60-0x63 map.
       [Cloud server only, DO NOT USE!!!] */
    device_add_params(machine_get_kbc_device(machine), (void *) model->kbc_params);

    /* The ROM is an in-situ burn-in program: it assumes a normal BIOS has
       already programmed the 8259 to the XT convention (8086 mode, base 8, its
       IVT entries 8..0x13) and never programs it itself. Without this the
       controller stays in 8080 mode, an IRQ resolves to (irq << 3), and IRQ 4
       lands on the unfilled vector 0x20 - the CPU then runs away into video
       memory.  [Cloud server only, DO NOT USE!!!] */
    pic_init();
    pic_xt_bios_init(0x08);
    dma_init();
    pit_ibm5550_init();
    nmi_mask = 0;

    /* Shared RAM + doorbells towards the keyboard/display controller.  Register
       before the epoch device so this window wins over its 0xA0000-0xDFFFF map. */
    device_add(&i3477_shared_device);

    /* The 80186's integrated peripherals / chip-select block (the ROM programs
       UMCS via out 0xFFA0 at reset). [Cloud server only, DO NOT USE!!!] */
    device_add(&i80186_device);

    /* The 5250 adapter's self-test (FD43:0A0E via the module entry at
       FD43:0000) drives its registers through a port table built into the
       parameter block at DS:0x7C0: phase A uses 0x1C0-0x1D7, phase B the
       0x100-0x10F bank.  A probe over both banks shows the ROM only ever
       *writes* - 0x1B0/0x1B2/0x1B6 as control, 0x1C0/0x1CA/0x1D0 as commands,
       and the table port 0x1C6 strobed through bit 6 (0x10 -> 0x50 -> 0x10)
       in an endless timeout loop entered at FD43:06D2 - and never reads
       anything back, so the sub-test result bytes (DS:[2]/[3], folded into
       DS:[0]/[1] with or 0x30 by FD43:0AFA) never clear.  Code 21 is a missing
       *interrupt*: the adapter has to answer the strobe.  [Cloud server only,
       DO NOT USE!!!] */
    epoch_t *epoch = device_add(&ibm5550_device);
    /* [FIX] Read the machine's own section.  A machine_t-style init has no
       "current device", so device_get_config_int() only ever returned the
       option default and both the bank choice and 0xA0h bit 1 were stuck.
       testmode = which 32 KB bank, testsw = the 0xA0h bit 1 switch input. */
    epoch->testmode = config_get_int("Machine", "testmode", 0);
    epoch->testsw   = config_get_int("Machine", "testsw", 0);
    pclog("3477: cfg testmode = %d, testsw = %d\n", epoch->testmode, epoch->testsw);
    /* The 3477-S powers up running the burn-in (test) program, which the ROM's
       early 0xA0h bit 1 check selects, on a 24-dot display (0xA2h reports 0xA8,
       so the ROM polls the C000:0148 mailbox).  [Cloud server only, DO NOT USE!!!] */
    /* [EXPERIMENT rombank] Let the config's testmode drive BOTH the port 0xA0
       bit 1 and which 32 KB bank is mapped at F8000-FFFFF (see rombank above).
       [Cloud server only, DO NOT USE!!!] */
    {
        FILE *fp = rom_fopen("roms/machines/ibm3477/3477ipl.bin", "rb");

        /* [FIX] Swap the bank inside the ROM buffer the machine already loaded,
           instead of adding a second mapping over F8000: a mapping with its own
           read handlers is not the same service path as the BIOS mapping's
           bios_read(), and it changed the burn-in's recorded code sequence
           (14/10 appeared, exactly as when the font card is absent).  Patching
           rom[] keeps F0000-FFFFF served by bios_read() exactly as before. */
        if ((fp != NULL) && (rom != NULL)) {
            fseek(fp, epoch->testmode ? 0x8000 : 0x0000, SEEK_SET);
            if (fread(rombank, 1, 0x8000, fp) == 0x8000) {
                memcpy(&rom[0x8000], rombank, 0x8000);
                pclog("3477: ROM bank = %s (testmode=%d), patched into rom[]\n",
                      epoch->testmode ? "burn-in (upper 32K)" : "System ROS (lower 32K)",
                      epoch->testmode);
            }
            fclose(fp);
        }
    }
    epoch->font24   = 1;

    /* The burn-in ROM is an in-situ program: it assumes the machine has already
       been booted, so the half-width (ANK) character generator in VRAM
       (0x38000, 128 bytes per code) is normally loaded by the IPL / OS and the
       ROM never loads one itself.  The 3477 IPL does not read a disk either, so
       the generator has to come from ROM: use the ANK font from the 3477 boot
       volume ($SYSHN16.FNT: 80-byte header then 64 bytes per code, 32 rows of
       2 bytes, the glyph in the first byte of each row) and lay it out the way
       the font-24 text renderer reads it (one 16-bit word per row at +2).
       [Cloud server only, DO NOT USE!!!] */
    {
        FILE *fp = rom_fopen("roms/machines/ibm3477/card0.bin", "rb");

        if (fp != NULL) {
            /* The font-24 text renderer fetches the half-width generator from
               EPOCH_VRAM_SBCS (0x38000), 128 bytes per code - loading the page
               at vram[0] leaves the renderer reading zeros (the ankA=00000000
               seen in every run).  Put it where it is read.  [Cloud server
               only, DO NOT USE!!!] */
            size_t rd = fread(epoch->vram + EPOCH_VRAM_SBCS, 1, 0x8000, fp);
            fclose(fp);
            pclog("3477: card page0 -> vram[%05X], %u bytes; 'A' row3 = %02X%02X%02X%02X\n",
                  EPOCH_VRAM_SBCS, (unsigned) rd,
                  epoch->vram[EPOCH_VRAM_SBCS + 0x41 * 0x80 + 3 * 4], epoch->vram[EPOCH_VRAM_SBCS + 0x41 * 0x80 + 3 * 4 + 1],
                  epoch->vram[EPOCH_VRAM_SBCS + 0x41 * 0x80 + 3 * 4 + 2], epoch->vram[EPOCH_VRAM_SBCS + 0x41 * 0x80 + 3 * 4 + 3]);
        } else
            pclog("3477: card0.bin missing (roms/machines/ibm3477/card0.bin)\n");
    }

    device_add(&lpt_port_device);
    serial_t *uart = device_add(&ns8250_device);
    serial_setup(uart, 0x3f8, 1); /* Use IRQ 1 */

    return ret;
}

static const device_config_t epoch_5535_config[] = {
    // clang-format off
    {
        .name        = "font",
        .description = "Font",
        .type        = CONFIG_SELECTION,
        .default_int = EPOCH_FONTCARD_ROM_NEW,
        .selection   = {
            {
                .description = "Old",
                .value = EPOCH_FONTCARD_ROM_OLD
            },
            {
                .description = "New",
                .value = EPOCH_FONTCARD_ROM_NEW
            },
            { .description = "" }
        }
    },
    {
        .name        = "testmode",
        .description = "Test mode",
        .type        = CONFIG_BINARY,
        .default_int    = 0,
        .selection      = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t ibm5535_device = {
    .name          = "IBM 5535 System Unit",
    .internal_name = "ibm5535",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = epoch_init,
    .close         = epoch_close,
    .reset         = epoch_reset,
    .available     = NULL,
    .speed_changed = epoch_speed_changed,
    .force_redraw  = epoch_force_redraw,
    .config        = epoch_5535_config
};

/* IBM Multistation 5535-M: a 286 on the 5550 planar, with a 32 KB IPL at
   F8000. Currently we reports no hard disk through port 0xA2 bit 7, which
   makes it skip the 0xE800 hard disk controller probe and its RAM test. */
int
machine_xt_ibm5535_init(const machine_t *model)
{
    int ret;

    ret = bios_load_interleaved("roms/machines/ibm5535/94X1301.BIN",
                                "roms/machines/ibm5535/94X1302.BIN",
                                0x000f8000, 32768, 0);

    if (bios_only || !ret)
        return ret;

    epoch_is_5535 = 1;

    device_add(&fdc_xt_5550_device);

    epochkbd_t *kbc = device_add(&kbc_epoch_device);

    pic_init();
    dma_init();
    pit_ibm5550_init();
    nmi_mask = 0;

    device_add(&ibm5535_device);

    device_add(&lpt_port_device);
    serial_t *uart = device_add(&ns8250_device);
    serial_setup(uart, 0x3f8, 1); /* Use IRQ 1 */

    if (mouse_type == MOUSE_TYPE_INTERNAL) {
        /* Tell mouse driver about our internal mouse. */
        mouse_reset();
        mouse_set_buttons(2);
        /* I don't know the actual polling speed, but
           a higher value may cause a conflict with the mouse driver  */
        mouse_set_sample_rate(30.0);
        mouse_set_poll(epoch_mouse_poll, kbc);
    }

    return ret;
}
