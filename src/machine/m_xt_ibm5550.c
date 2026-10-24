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
#include <86box/ibm5250.h>

/* [PROBE] A0 写访问环形缓冲：判断进故障程序前 0x28/0xA9 是否真被写过 */
static uint16_t kr_ring[16];
static unsigned kr_n;
static void
kr_dump(void)
{
    unsigned i, k = (kr_n < 16) ? kr_n : 16;
    char b[220]; int o = 0;

    o += snprintf(b + o, sizeof(b) - o, "3477: KRRING n=%u:", kr_n);
    for (i = k; i > 0; i--)
        o += snprintf(b + o, sizeof(b) - o, " %02X", (uint8_t) (kr_ring[(kr_n - i) & 15] & 0xFF));
    pclog("%s\n", b);
}

static void
kr_ivt(void)
{
    static int kn;
    unsigned v;

    if (kn++ >= 3)
        return;

    pclog("3477: KRIVT cs=%04X ip=%04X\n", CS, cpu_state.pc);
    /* 自证：崩溃处理程序刚写过 0:0x14A-0x14F，应读到 00 00 00 F8 03 00 */
    /* 找调用帧：处理程序自身已压 7 字(14B)，若由 call far 进入，其后就是 调用者 IP:CS */
    {
        char b[220]; int o = 0;
        uint32_t ssb = (uint32_t) SS << 4;
        unsigned sp  = SP;

        o += snprintf(b + o, sizeof(b) - o, "3477: KRSTK ss=%04X sp=%04X:", SS, sp);
        for (unsigned i = 0; i < 24; i++)
            o += snprintf(b + o, sizeof(b) - o, " %02X",
                          mem_readb_phys(ssb + ((sp + 14 + i) & 0xFFFF)));
        pclog("%s\n", b);
    }
    for (unsigned base = 0; base < 2; base++) {
        uint32_t a0 = base ? 0x140u : 0x000u;
        char b[120]; int o = 0;

        o += snprintf(b + o, sizeof(b) - o, "3477: KRRAW %03X phys:", a0);
        for (unsigned i = 0; i < 16; i++)
            o += snprintf(b + o, sizeof(b) - o, " %02X", mem_readb_phys(a0 + i));
        pclog("%s\n", b);
    }
    {
        char b[120]; int o = 0;

        /* 用了非零的已知内容做自证：ROM 窗口首字节（应从 ROM 文件读得到） */
        o += snprintf(b + o, sizeof(b) - o, "3477: KRRAW ROM@F8000:");
        for (unsigned i = 0; i < 8; i++)
            o += snprintf(b + o, sizeof(b) - o, " %02X", mem_readb_phys(0xF8000u + i));
        pclog("%s\n", b);
    }
    for (v = 0; v < 32; v++) {
        uint16_t off = (uint16_t) (mem_readb_phys((v << 2) + 0) | (mem_readb_phys((v << 2) + 1) << 8));
        uint16_t seg = (uint16_t) (mem_readb_phys((v << 2) + 2) | (mem_readb_phys((v << 2) + 3) << 8));

        if (off || seg)
            pclog("3477: KRIVT v=%02X -> %04X:%04X\n", v, seg, off);
    }
}
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

/* ---- 探针开关（一次性解析，之后按位判断）------------------------------------
   getenv() 每次调用都要线性扫描 environ：本机显示/IO 热路径（每个文本单元、每个
   像素、每次 I/O 写）原先每次都调用它，实测把仿真拖慢一倍以上。这里只在首次用
   到时解析一次并缓存。加新探针时在这里登记一个 PB_*。 */
enum {
    PB_OUTB = 0, PB_LCDATA, PB_MODE, PB_PIXEL, PB_WLOG, PB_EBTRC,
    PB_EBANK, PB_HIRAM, PB_FCWR, PB_PCSAMPLE, PB_C000LOG, PB_A0LOG, PB_ROMW, PB_ROMPG, PB_ADR, PB_DEAD, PB_CWLOG, PB_N
};
/* [PROBE] 例程出口取指跟踪：命中触发线性地址后记录其后 N 条取指 pc（拿调用者返回地址） */
static int      epoch_pctrace_left;
static uint32_t epoch_pctrace_trig = 0xFF03B;   /* 位脉冲原语 ret（文件 0xE03B → 线性 0xFF03B） */
static int      epoch_pctrace_n = -1;            /* -1 = 尚未从环境读取 */
static const char *const pb_name[PB_N] = {
    "EPOCH3477_OUTB", "EPOCH3477_LCDATA", "EPOCH3477_MODE", "EPOCH3477_PIXEL",
    "EPOCH3477_WLOG", "EPOCH3477_EBTRC", "EPOCH3477_EBANK", "EPOCH3477_HIRAM",
    "EPOCH3477_FCWR", "EPOCH3477_PCSAMPLE", "EPOCH3477_C000LOG", "EPOCH3477_A0LOG",
    "EPOCH3477_ROMW", "EPOCH3477_ROMPG", "EPOCH3477_ADAPTER", "EPOCH3477_DEAD",
    "EPOCH3477_CWLOG"
};
static int pb_flag[PB_N];

/* 5250 适配器中断应答（EPOCH3477_ADAPTER=1）：命令端口写之后延迟投递一次中断。
   自检在 FD43:06D2 循环选通 0x1C6，然后 sti 等中断服务程序置位 bh/bl
   （判据原文 FDC11/FDC1C：未置位则报 0x95/0x96）。IRQ 号线由
   EPOCH3477_ADAPTER_IRQ 选择（5550 IRQ 表里 IRQ5 空位最可能是它）。 */
static int        epoch_adr_irq = -1;
static pc_timer_t epoch_adr_timer;
static ibm5250_t *epoch_adr_dev_handle;
static void
epoch_adr_irq_cb(void *priv)
{
    (void) priv;
    if (epoch_adr_irq >= 0) {
        if (getenv("EPOCH5250_AB0NOIRQ") && epoch_adr_dev_handle &&
            ibm5250_in_ab0(epoch_adr_dev_handle))
            return;
        pclog("3477: ADAPTER IRQ fired (irq=%d)\n", epoch_adr_irq);
        { static int ivt_done;
          if (!ivt_done && getenv("EPOCH3477_IVTDBG")) {
              int v; ivt_done = 1;
              for (v = 0x08; v <= 0x12; v++)
                  pclog("3477: IVT %02X -> %04X:%04X\n", v,
                        (ram[v*4+3] << 8) | ram[v*4+2], (ram[v*4+1] << 8) | ram[v*4]);
          } }
        picint((uint16_t) (1 << epoch_adr_irq));
    }
}
/* [ADRDEV] IRQ line callback handed to the standalone device model. */
static void
epoch_adr_dev_irq(void *priv)
{
    (void) priv;
    if (epoch_adr_irq >= 0)
        picint((uint16_t) (1 << epoch_adr_irq));
}

/* 电平型中断：客人写 0x722 收尾时才消线 */
static void
epoch_adr_dev_irq_clear(void *priv)
{
    (void) priv;
    if (epoch_adr_irq >= 0)
        picintc((uint16_t) (1 << epoch_adr_irq));
}

static ibm5250_t *epoch_adr_dev_handle = NULL;

static void epoch_adr_shared_clear(void *priv);

static void
epoch_adr_ack_out(uint16_t port, uint8_t val, void *priv)
{
    (void) port; (void) priv;
    if (getenv("EPOCH5250_PCLOG"))
        pclog("5250: ACK 0x722 = %04X\n", val);
    if (epoch_adr_dev_handle != NULL)
        ibm5250_irq_ack(epoch_adr_dev_handle);
}

static inline int
pb(int id)
{
    if (pb_flag[id] == 0)
        pb_flag[id] = (getenv(pb_name[id]) != NULL) ? 1 : -1;
    return pb_flag[id] > 0;
}

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
/* [3477-S ROM paging — probe EPOCH3477_ROMPG]
   System ROS = a 32 KB window at F8000-FFFFF fed by the 0x390 page register.
   Every 32 KB page is a self-contained image: reset vector at page offset
   0x7FF0 (`EA 00 00 F5 FF` = jmp far 0xFFF5:0 -> linear 0xFFF50 = the page's
   0x7F50 stub, which programs the 80186 UMCS via 0xFFA0 and then far-jumps
   into the F800 segment), a 4-entry far-pointer table at 0x7FE0 using F8xx/
   F9xx/FAxx segments, and its own signature ending the page.
   Port 0x390 values seen statically: 0x00, 0x02 and 0x80 (0x80 is written by
   the IPL half's 0x7F40 stub just before that hand-over). */
static mem_mapping_t epoch_rom_map;
static uint8_t      *epoch_rom_pagebuf[2];
static uint8_t      *epoch_rom_sel = NULL;
static int           epoch_rom_pages = 0;
static int           epoch_rom_trace = 1 << 30; /* [ROMX] dump the first N page offsets when EPOCH3477_ROMX is set */
static int           epoch_rom_cur   = -1;

static uint8_t
epoch_rom_readb(uint32_t addr, void *priv)
{
    if (epoch_rom_trace < 64) {
        pclog("3477: ROMX off=%04X pc=%04X:%04X\n", addr & 0x7fff, cs >> 4, cpu_state.pc);
        epoch_rom_trace++;
    }
    {
        if (epoch_pctrace_n < 0) {
            const char *e = getenv("EPOCH3477_PCTRACE");
            const char *t = getenv("EPOCH3477_PCTRACE_TRIG");
            epoch_pctrace_n = e ? atoi(e) : 0;
            if (t)
                epoch_pctrace_trig = (uint32_t) strtoul(t, NULL, 0);
        }
        if (epoch_pctrace_n > 0) {
            uint32_t lin = ((uint32_t) cs) + cpu_state.pc;   /* cs 为段基（本文件既有用法：cs >> 4 = 段值） */
            if (epoch_pctrace_left <= 0) {
                if (lin == epoch_pctrace_trig)
                    epoch_pctrace_left = epoch_pctrace_n;
            } else {
                pclog("3477: PCTRACE %05X\n", lin);
                epoch_pctrace_left--;
            }
        }
    }
    return epoch_rom_sel ? epoch_rom_sel[addr & 0x7fff] : 0xff;
}

static uint16_t
epoch_rom_readw(uint32_t addr, void *priv)
{
    if (!epoch_rom_sel)
        return 0xffff;
    return *(uint16_t *) &epoch_rom_sel[addr & 0x7ffe];
}

static void
epoch_rompg_switch(uint8_t val)
{
    int idx;

    if (epoch_rom_pages == 0)
        return;
    /* Bit 7 selects the page (set -> page 1/ROS half, clear -> page 0/IPL
       half); the 0x00/0x02 writes seen all over both halves are "back to page
       0".  Bit 1 is an independent control bit (0x82 write site at F888B),
       not part of the page select. */
    idx = (val & 0x80) ? 1 : 0;
    if (idx >= epoch_rom_pages)
        idx = 0;
    epoch_rom_cur = idx;
    epoch_rom_sel = epoch_rom_pagebuf[idx];
    /* [T1000 方案] 取指走映射的 exec 指针，必须随换页一起重设 */
    mem_mapping_set_exec(&epoch_rom_map, epoch_rom_sel);

    pclog("3477: ROMPG out 0x390 = %02X -> page %d pc=%04X:%04X\n",
          val, idx, cs >> 4, cpu_state.pc);
}



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

    /* [ADRDEV] standalone 5250 adapter instance (EPOCH3477_ADRDEV=1). */
    ibm5250_t *adapter;

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
            /* [PROBE] EPOCH3477_LCDATA=1: log every (index,value) pair the guest
               writes through 0x3D0/0x3D1 - that is where the card's mode setup
               lives. */
            if (!(epoch->crtcaddr == LC_CURSOR_LOC_HIGH || epoch->crtcaddr == LC_CURSOR_LOC_LOWJ)) {
                epoch_iolog("%04X:%04X epoch Out addr %03X idx %02X val %02X (%d)\n", cs >> 4, cpu_state.pc, addr, epoch->crtcaddr, val, val);
            }
              if (pb(PB_LCDATA)) {
                  static uint8_t lc_seen[256];
                  static int lc_init = 0;
                  if (!lc_init) { memset(lc_seen, 0xff, sizeof(lc_seen)); lc_init = 1; }
                  if (lc_seen[epoch->crtcaddr] != (uint8_t) val) {
                      lc_seen[epoch->crtcaddr] = (uint8_t) val;
                      pclog("3477: LCDATA idx=%02X val=%02X pc=%04X:%04X\n",
                            epoch->crtcaddr, val, cs >> 4, cpu_state.pc);
                  }
              }
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
    /* [PROBE] EPOCH3477_MODE=1: log every CRTC mode write (text 04 vs APA 09/0A). */
    if (pb(PB_MODE))
        pclog("3477: CRTMODE <- %02X (was %02X) pc=%04X:%04X\n",
              (unsigned) (val & 0xff), (unsigned) (epoch->crtmode & 0xff), cs >> 4, cpu_state.pc);
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
               selected: it reads 0x94 twice, then 0x04 (F8655/F86C2).

               The 0x94 pass must NOT invert.  The IPL enters the poll block at
               F800:0547 with mode 0x94, and its exit at F800:0587 is only the
               success path once the timeout branch has run -- that branch is
               what sets AH (xor ah, 0FFh at F800:05C0), while AH = 0 at
               F800:05C8 sends it to the error exit F800:05C5 (jmp 0x53B ->
               mov ax, 0Bh).  The timeout branch itself writes mode 0x04
               (F800:05B0), so inverting only for 0x04/0x10 makes sweep #1 time
               out, the timeout set AH and switch to 0x04, and sweep #2 invert
               and satisfy the poll.  Inverting for 0x94 as well let sweep #1
               satisfy the poll immediately, leaving AH = 0 and the IPL
               reporting failure code 0x0B at the handoff doorbell. */
            if (epoch_is_3477 && ((epoch->crtmode == 0x04) || (epoch->crtmode == 0x10))) {
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
    if (pb(PB_OUTB))
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
    if (pb(PB_DEAD)) {
        static uint32_t dn;
        if ((dn++ % 64u) == 0u)
            pclog("3477: DEAD IN(3D0) addr=%03X n=%u cs=%04X ip=%04X\n",
                  addr, dn, (unsigned) (cs >> 4), cpu_state.pc);
    }
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
    if (pb(PB_DEAD)) {
        static uint32_t dm;
        if ((dm++ % 64u) == 0u)
            pclog("3477: DEAD IN(2D0) addr=%03X n=%u cs=%04X ip=%04X\n",
                  addr, dm, (unsigned) (cs >> 4), cpu_state.pc);
    }
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

    code *= 0x80;
    code += line * 4;
    /* DBCS glyphs live in the video memory's kanji area, indexed by the 16-bit
       code, exactly as on the 5550.  (A 3477-only redirect of small codes into
       the SBCS area used to sit here; it existed solely for the 26-dot per-cell
       experiment's lookup of ASCII codes as DBCS codes, and is gone.) */
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
            /* 3477-S: a text cell is 13 dots wide (80 columns x 13 = 1040, as
               measured on the machine), one CRAM word pair per cell.  The
               half-width generator comes from the card's GB2312 row 3 (see the
               ANK loader in the machine init); the card stores those ANK forms
               in the left 16 columns of the 32x32 cell with the right half
               empty, so the cell shows the first 13 of those 16 dots.  DBCS
               characters (attr bit 0) are drawn as two 13-dot halves by the
               branches below. */
            uint32_t cell_dots = epoch->char_width;
            /* Draw character */
            for (uint32_t n = 0; n < cell_dots; n++)
                p[n] = epoch->pallook[epoch->egapal[bg]]; /* draw blank */
            /* 3477-S: the half-width generator comes from the card's GB2312
               row 3 (see the ANK loader in the machine init), and a text cell
               is 26 dots: the 32x32 bitmap of the character is cut down the
               middle and its two halves are drawn side by side.  DBCS
               characters (attr bit 0) are drawn as two 13-dot halves by the
               branches below. */
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
                    if (epoch_is_3477 && epoch->font24 && !(attr & 0x01)) {
                        /* The generator was already narrowed to the 13-dot text
                           cell at load time (see epoch_ank_narrow): its thirteen
                           columns are the top 13 bits of the first word. */
                        const uint32_t fbase = fontbase + chr * 0x80 + epoch->scanline * 4;
                        const uint16_t lo = (uint16_t) ((epoch->vram[fbase + 0] << 8) | epoch->vram[fbase + 1]);
                                                uint32_t       mid = (uint32_t) (lo >> 3);

                        for (uint32_t n = 0; n < 13; n++) {
                            p[n] = epoch->pallook[epoch->egapal[(mid & 0x1000) ? fg : bg]];
                            mid <<= 1;
                        }
                    } else if (epoch->font24) {
                        font = epoch->vram[fontbase + chr * 0x80 + epoch->scanline * 4 + 2]; /* w13xh29 font */
                        font <<= 8;
                        font |= epoch->vram[fontbase + chr * 0x80 + epoch->scanline * 4 + 3];
                        /* Draw 13 dots */
                        for (uint32_t n = 0; n < epoch->char_width; n++) {
                            p[n] = epoch->pallook[epoch->egapal[(font & 0x8000) ? fg : bg]];
                            font <<= 1;
                        }
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
                        /* Draw 13 dots */
                        for (uint32_t n = 0; n < epoch->char_width; n++) {
                            p[n] = epoch->pallook[epoch->egapal[(font & 0x8000) ? fg : bg]];
                            font <<= 1;
                        }
                    }
                    // if(chr!=0x20) epoch_log("memaddr: %x, scanline: %x, chr: %x, font: %x    ", epoch->memaddr, epoch->scanline, chr, font);
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
                for (uint32_t n = 0; n < cell_dots; n++)
                    p[n] = epoch->pallook[epoch->egapal[fg]]; /* under line (white) */
            }
            /* Column 1 (Vertical Line) */
            if (attr & 0x10) {
                p[0] = epoch->pallook[epoch->egapal[2]]; /* vertical line (white) */
            }
            if (epoch->scanline == 0 && attr & 0x20) { /* HGrid */
                for (uint32_t n = 0; n < cell_dots; n++)
                    p[n] = epoch->pallook[epoch->egapal[2]]; /* horizontal line (white) */
            }
            /* Drawing text cursor */
            drawcursor = ((epoch->memaddr == epoch->cursoraddr) && epoch->cursorvisible && epoch->cursoron);
            if (drawcursor) {
                // int cursorwidth = (epoch->crtc[LC_COMPATIBILITY] & 0x20 ? 26 : 13);
                int cursorwidth = cell_dots;
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
            p += cell_dots;
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
    /* [PROBE] EPOCH3477_PIXEL=1: count non-zero pixels in the frame about to be
       blitted (every 200 frames + the first 3), so "black screen" is measurable. */
    if (pb(PB_PIXEL)) {
        static long fr = 0;
        fr++;
        if ((fr <= 3) || ((fr % 200) == 0)) {
            unsigned long nz = 0, tot = 0;
            for (int yy = 0; yy < ysize && yy < buffer32->h; yy++) {
                uint32_t *row = (uint32_t *) buffer32->line[yy];
                for (int xx = 0; xx < xsize && xx < buffer32->w; xx++) {
                    tot++;
                    if (row[xx] & 0x00FFFFFF) nz++;
                }
            }
            pclog("3477: PIXEL frame=%ld size=%dx%d nz=%lu/%lu\n", fr, xsize, ysize, nz, tot);
        }
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
        /* [PROBE] EPOCH3477_VDUMP=1: where do the guest plane writes actually land
       in epoch->vram, and does the text renderer's glyph slot (0x38000+code*0x80)
       already contain them?  Dumps 3 windows every 8192 byte-writes. */
                epoch_t *epoch = (epoch_t *) priv;
    // epoch_log("epoch_vw: %x %x\n", addr, val);
    addr &= EPOCH_MASK_VRAM;
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
        /* [PROBE 2026-10-15] EPOCH3477_WLOG=1: 记下开头 300 次落在 0xE0000-0xE1FFF
       的写入（含 val==0 与奇地址），带 PC 与当前 plane —— 用来对账「渲染器读到
       的文本字符码是否经由本函数写进 CRAM」（写侧普查 distinct=12 只有数字+图案，
       读侧却读到 "(C)opy right IBM." 的字符码）。 */
    if (pb(PB_WLOG) && ((epoch->dispbank & 3) == 0)
        && ((addr & 0xfff) >= 0x800) && ((addr & 0xfff) < 0xc00)
        && (val >= 0x21) && (val <= 0x7e)
        && (val != 0x54) && ((val < 0x30) || (val > 0x39))) {
        static int wl = 0;
        if (wl < 400) {
            if (wl == 0)
                pclog("3477: WLOG start cram=%p vram=%p\n",
                      (void *) epoch->cram, (void *) epoch->vram);
            wl++;
            pclog("3477: WLOG #%d addr=%05X val=%02X ('%c') plane=%u pc=%04X:%04X\n",
                  wl, addr, val, val, epoch->dispbank & 3, cs >> 4, cpu_state.pc);
        }
    }
    if (pb(PB_CWLOG) && ((cs >> 4) < 0xF000) && (val >= 0x21) && (val <= 0x7e)) {
        static int cwn = 0;
        if (cwn < 2000) {
            cwn++;
            pclog("3477: CW #%d addr=%05X val=%02X pc=%04X:%04X plane=%u\n",
                  cwn, addr, val, cs >> 4, cpu_state.pc, epoch->dispbank & 3);
        }
    }
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


    // epoch_log("epoch_cram_writ   ew: Write to %x, val %x\n", addr, val);
    epoch_t *epoch = (epoch_t *) priv;
    cycles -= video_timing_write_w;
    epoch_cram_write(addr, val & 0xff, epoch);
    epoch_cram_write(addr + 1, val >> 8, epoch);
}

/* Build the narrow (13 dot) form of a card glyph.  The card draws every ANK
   character TWICE inside its 32 column cell - two complete narrow glyphs side
   by side, one per 16 column half, with the gap always over columns 13-15
   (verified: 'I' shows two stems with two sets of serifs, 'M' two complete M's,
   'V' two complete V's, '0' two complete ovals).  So the narrow form is not a
   crop of a wide glyph at all: it is one of those two copies.  Take the left
   half and centre its ink bounding box in the 13 dot cell, squeezing by a dot
   or two when the copy is slightly wider than 13 ('M' spans 14 columns).  That
   way nothing is cut off (the old left-six-plus-right-six rule threw away the
   middle of V/M/I) and no ink is drawn twice (it duplicated '.' and 'i').
   The thirteen bits land in the top of each row (columns 0-12), which the text
   renderer reads as bits 15..3 of the first word. */
static void
epoch_ank_narrow(const uint8_t *src, uint8_t *dst)
{
    uint32_t rowbits[32];
    int      c0 = 32, c1 = -1;

    /* [FIX 2026-10-15] Measure the ink over the whole 32-column cell.  The card
       stores the ASCII set full width with the ink centred (measured in
       font.bin: g192 '!' cols 11-16, g207 '0' cols 5-23, g224 'A' cols 3-25,
       g226 'C' cols 4-23, g251 '/' cols 1-27), i.e. up to 27 columns wide, and
       the text cell is only 13 dots - so the >13 branch below must re-sample
       the ink proportionally.  Scanning only columns 0-15 made c1 stop at 15
       for every glyph wider than the cell, so w came out <= 13 and the
       proportional branch was never taken: the left half of each character was
       drawn centred and the right half (which lives in columns 16-31) was
       dropped.  Verified by rendering the stored words as ASCII art. */
    for (uint32_t row = 0; row < 32; row++) {
        const uint32_t v = ((uint32_t) src[row * 4 + 0] << 24) |
                           ((uint32_t) src[row * 4 + 1] << 16) |
                           ((uint32_t) src[row * 4 + 2] << 8) |
                           (uint32_t) src[row * 4 + 3];

        rowbits[row] = v;
        for (int col = 0; col < 32; col++) {
            if (v & (0x80000000u >> col)) {
                if (col < c0)
                    c0 = col;
                if (col > c1)
                    c1 = col;
            }
        }
    }
    for (uint32_t row = 0; row < 32; row++) {
        uint32_t out = 0;

        if (c1 >= c0) {
            const int w = c1 - c0 + 1;

            if (w <= 13) {
                const int off = (13 - w) / 2;

                for (int i = 0; i < w; i++)
                    out |= ((rowbits[row] >> (31 - (c0 + i))) & 1u) << (12 - off - i);
            } else {
                for (int i = 0; i < 13; i++) {
                    int a = c0 + (i * w) / 13;
                    int b = c0 + ((i + 1) * w) / 13;

                    if (b <= a)
                        b = a + 1;
                    for (int col = a; (col < b) && (col < 32); col++) {
                        if (rowbits[row] & (0x80000000u >> col)) {
                            out |= 1u << (12 - i);
                            break;
                        }
                    }
                }
            }
        }
        const uint32_t w32 = out << 19;

        dst[row * 4 + 0] = (uint8_t) (w32 >> 24);
        dst[row * 4 + 1] = (uint8_t) (w32 >> 16);
        dst[row * 4 + 2] = (uint8_t) (w32 >> 8);
        dst[row * 4 + 3] = (uint8_t) w32;
    }
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
        /* [EXP] The 3477-S IPL arms a deliberate error with A0h bit 7
           (`mov al,0xA9 / out 0xA0,al` at F800:0836) and then consumes it with
           the very next read (F914:000C, which is also how it fetches the SS
           base at 0x131).  That read is a test, not a fault: the IPL only
           ever looks at A0h bit 0 for the walk at F800:0796, and it
           acknowledges the interrupt condition at F914:0040 (0x730) - which
           it can only reach if the armed read does not raise an NMI.  A real
           fault (the paritybad bitmap) still propagates. */
        int armed = 0;
        if (epoch_parity_check(epoch, addr) || epoch->parityforce) {
            newerr = (epoch->parityerror == 0);
            armed  = (epoch_is_3477 && (epoch->parityforce != 0));
            epoch->parityerror = 1;
            if (epoch_is_3477)
                epoch->parityforce = 0; /* [AB] the arm is one-shot */
        }
        if ((epoch->parityerror != 0) && (epoch->parityenabled != 0)) {
            epoch->parityerroraddr = (addr >> 16) & 0xFF; /* X000:0 */
            epoch_log("%04X:%04X perror at %0X\n", cs >> 4, cpu_state.pc, addr);
            if (nmi_mask && (!epoch_is_3477 || newerr) && !armed) {
                nmi_raise();
            }
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
    /* [PROBE] ROM 自检完成的证据：F9216-F921D 会把设备 ID 两字节写到绝对 0x14C/0x14D */
if ((addr >= 0x148) && (addr <= 0x14f)) {
pclog("3477: RAMW %04X = %02X pc=%04X:%04X\n", addr, val, CS, cpu_state.pc);
if (pb(PB_DEAD)) kr_dump();
}
/* [PROBE] 磁盘服务状态块 0x53B8-0x53BF（中性：只读值，不注入）。
   0650:87D1 写 0x80 = 等 RQM 超时；0650:87E0 写 0x20 = 无 DIO；0x53B8 bit7 = 完成标志。 */

if ((pb(PB_ROMW)) && (((addr >= 0x7cd0) && (addr <= 0x7cd7)) || ((addr >= 0xddec) && (addr <= 0xddef)))) {
    static int mf;
    if (mf < 200)
        pclog("3477: MODFLAG %04X = %02X pc=%04X:%04X\n", addr, val, CS, cpu_state.pc);
    mf++;
}
if ((pb(PB_ROMW)) && (addr >= 0x5ab0) && (addr <= 0x5aef)) {   /* ISR 信箱+结果缓冲(0x5ABA-0x5AC0)+状态位 [0x53E4](0x5AE4)：ss=0x0070 */
static int fst;
if (fst < 400)
pclog("3477: MAILBOX %04X = %02X pc=%04X:%04X\n", addr, val, CS, cpu_state.pc);
fst++;
}
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
if ((addr >= 0x148) && (addr <= 0x14f)) {
pclog("3477: RAMW16 %04X = %04X pc=%04X:%04X\n", addr, val, CS, cpu_state.pc);
if (getenv("EPOCH3477_W2A") && addr >= 0x1020 && addr <= 0x1040) {
    static int w2a;
    if (w2a < 40) pclog("3477: W2A %04X = %04X CS=%04X ip=%04X (DS=%04X)\n", addr, val, CS, cpu_state.pc, DS);
    w2a++;
}
if (getenv("EPOCH3477_IVTDBG") && addr >= 0x30 && addr <= 0x33) {
    static int ivtw;
    if (ivtw < 24) pclog("3477: IVTW %04X = %02X pc=%04X:%04X\n", addr, val, CS >> 4, cpu_state.pc);
    ivtw++;
}
if (pb(PB_DEAD)) {
    kr_dump();
    if (addr == 0x14a)
        kr_ivt();
}
}
/* [PROBE] 同一区（0x53B8-0x53BF）的字写：ISR 把栈切到 0x0070:0x53B8 后，推栈/信箱都是字写。 */
if ((pb(PB_ROMW)) && (addr >= 0x5ab0) && (addr <= 0x5aee)) {   /* 同上（字写版） */
    static int fsw;

    if (fsw < 400)
        pclog("3477: MAILBOXW %04X = %04X pc=%04X:%04X\n", addr, val, CS, cpu_state.pc);
    fsw++;
}
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

/* [MODEL] 5250 适配器状态（端口映射由 gdbstub 运行期实测：0x1C0 控制 / 0x1C1 寄存器 /
   0x1C2 选通 / 0x1C6 数据+两相握手） */
static uint8_t    epoch_adr_reg;
static int epoch_adr_reg42 = -1;

static uint8_t    epoch_adr_latch;
static uint8_t    epoch_adr_phase = 1;
static int        epoch_adr_ack;   /* 0x1C6 写后 bit7=1 */
static int        epoch_adr_syn_idx; /* [SYNTH] 0x1C6 回读波形索引 */
static int        epoch_adr_replied; /* 本轮 8 次轮询里已经回应过一次 */
static int        epoch_adr_polls;   /* 主机轮询次数 */
static int        epoch_adr_cooldown; /* 应答一次后静默的轮询数 */
static int        epoch_adr_cmd;      /* 最近一次 0x1C0 命令 */
static int        epoch_adr_b1;       /* 0x1C6.bit1 相位（0x1C2 写 4 翻转） */
static int        epoch_adr_zrun;     /* 连续 0 数据脉冲 */
static uint32_t   epoch_adr_hist;     /* 最近 10 个脉冲（帧头图样） */
static int        epoch_adr_serve;    /* 本轮是否应答 */
static int        epoch_adr_fresh;    /* 刚收完一帧：下一次读从序列 0 开始 */
static uint8_t    epoch_adr_tx[128];  /* [ECHO] 本帧发出去的位（bit4） */
static int        epoch_adr_txc;      /* 已发位数 */
static int        epoch_adr_txr;      /* 已读位数 */
static int        epoch_adr_pls[64];  /* [PPM] 时钟脉冲的数据位序列 */
static int        epoch_adr_plsn;     /* 已收脉冲数 */
static uint16_t   epoch_adr_word;     /* [PPM] 解码出的发送字 */
static int rw_ph = 1, rw_rd = 0, rw_cmdlat = 0, rw_serve = 0, rw_inframe = 0, rw_pcnt = 0, rw_hist = 0;
static int rw_done40 = 0;
static int rw_dummy = 0;
static int rw_fno = 0;
static uint16_t rw_word = 0;
static int        epoch_adr_ph = 1;   /* 1=握手 2=帧 */
static int        epoch_adr_rd;
static int        epoch_adr_rdcnt;    /* 收帧阶段的读计数（替代会漂移的 syn_idx） */
static int        epoch_adr_cmdlat;   /* 锁存的命令字：后续 out 0x00 不得覆盖 */
static int        epoch_adr_armed;    /* 事件驱动：命令到达后挂起的应答序列 */
static int        epoch_adr_pprev = -1;
static int        epoch_adr_bitn;
static int        epoch_adr_pcnt;
static int        epoch_adr_needsync;
static int        epoch_adr_burst;
static int        epoch_adr_run;   /* 连续 1 对计数（用于识别复位前缀） */
static int        epoch_adr_run0;  /* 连续 0 对计数（帧间长空闲） */
static int        epoch_adr_inframe;
static int        epoch_adr_ack_cnt; /* [ACK] 前 3 次读为 1，之后为 0（两相握手） */


static int epoch_adr_bitcnt;
static int epoch_adr_rst;
static int epoch_adr_wait;
static int epoch_adr_resp_bit;
static uint16_t epoch_adr_shift;
static uint16_t epoch_adr_payload;

/* [P3C] 3477-S 入口/重启收尾口 0x3C0-0x3DF 观测（打印 AX）。 */
static void
epoch_p3c_write(uint16_t port, uint8_t val, void *priv)
{
    if (getenv("EPOCH3477_P3C"))
        pclog("3477: P3C out %03X = %02X ax=%04X pc=%04X:%04X\n",
              port, val, AX, CS, cpu_state.pc);
}

static uint8_t
epoch_misc_in(uint16_t port, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    uint8_t           ret = 0xff;

    switch (port) {
        case 0x1C1:
            if (pb(PB_ADR))
                ret = epoch_adr_reg;
            { static int rdbg; if (rdbg < 60) pclog("3477: ADRREAD 1C1 -> %02X pc=%04X:%04X\n", ret, CS >> 4, cpu_state.pc); rdbg++; }
            if (epoch_adr_reg42 < 0) { const char *v = getenv("EPOCH3477_ADR_REG42"); epoch_adr_reg42 = v ? atoi(v) : 0; }
            if (epoch_adr_reg42) ret |= 0x42;
            { static int fx = -1; if (fx < 0) fx = getenv("EPOCH3477_ADRFIX") ? 1 : 0; if (fx) ret = 0x48; }
            break;
        case 0x1C2:
        case 0x1C3:
            if (pb(PB_ADR))
                ret = 0xff;
            break;
        case 0x1C6:
            if (getenv("EPOCH3477_ADR_RW")
                && ((((CS > 0xFFFF) ? (int) (CS >> 4) : (int) CS) >= 0x3500))
                && ((((CS > 0xFFFF) ? (int) (CS >> 4) : (int) CS) <= 0x3700))) {
                static const uint8_t rws[19] = {
                    0x00, 0x00, 0x00, 0x80, 0x00,
                    0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
                    0x80, 0x80, 0x80,
                    0x00, 0x00, 0x00
                };
                uint8_t base = (uint8_t) ((epoch_adr_latch & 0x7d)
                                | ((rw_cmdlat == 0x40) ? (epoch_adr_b1 ? 0x02 : 0) : 0x02));
                ret = (uint8_t) (base & 0x3f);
                if (!rw_serve) {
                    ret = (uint8_t) (base & 0x3f);
                } else if (rw_ph != 2) {
                    ret = (uint8_t) ((base & 0x3f) | rws[rw_rd]);
                    /* 实验：客人把握手最后一读当作帧首读 ⇒ 该拍就给起始位 1 */
                    if (getenv("EPOCH3477_ADR_RW18") && (rw_rd == 18))
                        ret = (uint8_t) ((base & 0x3f) | 0x80);
                    if (++rw_rd >= 19) { rw_ph = 2; rw_rd = 0; rw_dummy = 1; }
                } else if (getenv("EPOCH3477_ADR_RW0") && rw_dummy) {
                    rw_dummy = 0;      /* 帧相首读：给 0，让客人的 15 次移位与我们 16 位对齐 */
                    ret = (uint8_t) (base & 0x3f);
                } else {
                    int k = rw_rd / 2;
                    int cmd40 = (rw_cmdlat == 0x40);
                    int d = cmd40 ? 0x55 : 0xFF;
                    int ones = 0, fb = 0, t;
                    for (t = 0; t < 8; t++) ones += (d >> t) & 1;
                    ones += 3;
                    if (k == 0)       fb = 1;
                    else if (k <= 8)  fb = (d >> (k - 1)) & 1;
                    else if (k <= 11) fb = 1;
                    else if (k == 12) { int o12 = ones + 1; fb = cmd40 ? ((o12 & 1) ? 0 : 1) : ((o12 & 1) ? 1 : 0); }
                    else              fb = 0;
                    ret = (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0));
                    if (++rw_rd >= 32) { rw_rd = 0; if (rw_cmdlat == 0x40) rw_done40 = 1; }
                }
                { static int q; if (q < 1500 && rw_ph == 2 && rw_serve) { pclog("3477: RWREAD f=%d rd=%d ret=%02X pc=%04X:%04X\n", rw_fno, rw_rd, ret, (CS > 0xFFFF) ? (int) (CS >> 4) : (int) CS, cpu_state.pc); q++; } }
                return ret;
            }
            if (getenv("EPOCH3477_ADR_SYNTH") && ((CS >> 4) >= 0x3500) && ((CS >> 4) <= 0x3700)) {
                static const uint8_t seq[19] = {
                    0x00, 0x00, 0x00, 0x80, 0x00,
                    0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
                    0x80, 0x80, 0x80,
                    0x00, 0x00, 0x00
                };
                int i;
                uint8_t base = (uint8_t) ((epoch_adr_latch & 0x7d)
                                | ((epoch_adr_cmd == 0x40) ? (epoch_adr_b1 ? 0x02 : 0) : 0x02));
                ret = (uint8_t) (base & 0x3f);
                if (!epoch_adr_serve) {
                    ret = (uint8_t) (base & 0x3f);
                } else if (epoch_adr_ph != 2) {
                    i = epoch_adr_rd;
                    ret = (uint8_t) ((base & 0x3f) | seq[i]);
                    if (++epoch_adr_rd >= 19) { epoch_adr_ph = 2; epoch_adr_rd = 0; }
                } else {
                    int k = epoch_adr_rd / 2;
                    int cmd40 = (epoch_adr_cmdlat == 0x40);
                    int d = cmd40 ? 0x55 : 0xFF;
                    int ones = 0, fb = 0, t;
                    for (t = 0; t < 8; t++) ones += (d >> t) & 1;
                    ones += 3;
                    if (k == 0)       fb = 1;
                    else if (k <= 8)  fb = (d >> (k - 1)) & 1;
                    else if (k <= 11) fb = 1;
                    else if (k == 12) { int o12 = ones + 1; fb = cmd40 ? ((o12 & 1) ? 0 : 1) : ((o12 & 1) ? 1 : 0); }
                    else              fb = 0;
                    ret = (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
                    if (++epoch_adr_rd >= 32) epoch_adr_rd = 0;
                }
                return ret;
            }
            { static int rdbg6; if (rdbg6 < 40) {
    pclog("3477: ADRREAD 1C6 #%d CSbase=%05X ip=%04X lin_cs_plus_pc=%05X pc_as_lin=%05X DSbase=%05X SSbase=%05X\n",
          rdbg6, CS, cpu_state.pc, (CS + cpu_state.pc) & 0xFFFFF, cpu_state.pc & 0xFFFFF, DS, SS);
} rdbg6++; }
            { /* [PROBE] 把客人 RAM 里 0x3659 段的握手环代码 dump 出来（每 256B 一段） */
                static int adr_dump_done;
                if (!adr_dump_done && getenv("EPOCH3477_ADRDUMP")) {
                    int k;
                    const char *ev_seg = getenv("EPOCH3477_ADRDUMP_SEG");
                    const char *ev_base = getenv("EPOCH3477_ADRDUMP_BASE");
                    const char *ev_n = getenv("EPOCH3477_ADRDUMP_N");
                    uint32_t dseg = ev_seg ? (strtoul(ev_seg, NULL, 0) << 4) : ((uint32_t) 0x3659 << 4);
                    uint32_t dbase = ev_base ? strtoul(ev_base, NULL, 0) : 0x0600;
                    int dcount = ev_n ? atoi(ev_n) : 20;
                    adr_dump_done = 1;
                    for (k = 0; k < dcount; k++) {
                        int i;
                        pclog("3477: DUMPSEG base=%04X ", dbase + k * 0x40);
                        for (i = 0; i < 0x40; i++)
                            pclog("%02X", ram[dseg + dbase + k * 0x40 + i]);
                        pclog("\n");
                    }
                }
            }
            { /* [PROBE] 5250 驱动状态字节随读取次数变化（动态看它卡在哪个状态） */
                static int adr_st_dbg;
                if (adr_st_dbg < 24 && getenv("EPOCH3477_ADRST")) {
                    /* [FIX] 段址必须用运行时 CS（pc 日志里的 3659 是 cs>>4 的截断值）*/
                    const uint32_t b = (CS & 0xFFF0000u) + ((CS & 0xF0000u) ? 0 : 0);
                    /* [FIX2] 驱动用 DS 相对寻址（CS 段是代码、数据区为 0xCC 未初始化）*/
                    const uint32_t bb = ((uint32_t) DS) << 4;   /* 本核 seg base 存原始段值 */
                    pclog("3477: ADRST #%d 2a=%02X 2b=%02X 2d=%02X 30=%02X 31=%02X 32=%02X 33=%02X 34=%02X\n",
                          adr_st_dbg, ram[bb + 0x2a], ram[bb + 0x2b], ram[bb + 0x2d], ram[bb + 0x30],
                          ram[bb + 0x31], ram[bb + 0x32], ram[bb + 0x33], ram[bb + 0x34]);
                    pclog("3477: ADRPORT ds=%04X [6]=%04X [8]=%04X [C]=%04X [24]=%04X E400blk=%02X%02X%02X%02X\n", DS, ram[bb+6] | (ram[bb+7]<<8), ram[bb+8] | (ram[bb+9]<<8), ram[bb+0xC] | (ram[bb+0xD]<<8), ram[bb+0x24] | (ram[bb+0x25]<<8), ram[0xE4000+0], ram[0xE4000+1], ram[0xE4000+2], ram[0xE4000+3]);
                }
                adr_st_dbg++;
            }
            { static int adr_in_dbg; if (adr_in_dbg < 400) { pclog("3477: IN1C6 #%d latch=%02X phase=%d reg=%02X bitcnt=%d wait=%d\n", adr_in_dbg, epoch_adr_latch, epoch_adr_phase, epoch_adr_reg, epoch_adr_bitcnt, epoch_adr_wait); } adr_in_dbg++; } 
            if (pb(PB_ADR)) {
                /* bit1 = 握手相位；bit4/bit6 = 已写值回显；bit7 = 从设备数据入（暂 0） */
                static int adrrb = -1;
                if (adrrb < 0) { const char *v = getenv("EPOCH3477_ADRRB"); adrrb = v ? atoi(v) : 0; }
                /* [AB] 读侧原本用 0x7d 掩掉了 bit7，而写侧辛苦算出的 resp_bit 正落在 bit7
                   （"从设备数据入"）⇒ 客人轮询 0x1C6 等应答时永远等不到。adrrb=1 放行 bit7。 */
                ret = (uint8_t) ((epoch_adr_latch & 0x7d) | (epoch_adr_phase ? 0x02 : 0x00)
                                 | (adrrb ? (epoch_adr_latch & 0x80) : 0)
                                 | (getenv("EPOCH3477_ADRB0") ? 0x01 : 0));
            }
            break;

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
            /* [3477-S] Rear-panel test switch (0xA0 bit 1) is a STATIC level,
               not a transition to be awaited: the operator flips it after a
               system crash and resets, which makes the ROS take the test path
               at its entry (F8012: `test al,2; jnz`).  The 0->1 "transition"
               model that used to live here was wrong: the only code that
               polls bit 1 (F989B/F98A1) sits inside the test program itself
               (its own switch-verification step) and is never reached on the
               normal path.  Removed 2026-10-15. */

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
            /* [FIX] 设备侧状态：bit0 = 自检通过/继续（步驱动器 F800:085F 的 test $1 闸） */
                /* [A0] 实验门控：EPOCH3477_A0B1=1 只置 bit1，=2 两位都置（默认 0 保持原行为） */
    if (getenv("EPOCH3477_A0B1")) {
        int a0b1 = atoi(getenv("EPOCH3477_A0B1"));
        ret &= (uint8_t) ~0x01;
        if (a0b1 >= 1) ret |= 0x02;
        if (a0b1 >= 2) ret |= 0x01;
    }
            if (pb(PB_A0LOG))
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
                {
                    static int f24ovr = -1;
                    if (f24ovr < 0) { const char *v = getenv("EPOCH3477_FONT24"); f24ovr = v ? atoi(v) : 2; }
                    ret = ((f24ovr == 2) ? epoch->font24 : f24ovr) ? 0x88 : 0x22;
                }
                /* A2h bit7 is the panel/model-configuration bit and stays set
                   even while a memory error is latched.  The floppy-resident
                   IBMBIO picks the 3472/3477 banner with
                   `in al,0xA2 / test al,0x80` (IBMBIO.COM+0x7A8), and the
                   models differ in their RAM hole layout, so this bit is how a
                   3477 says so; clearing it on a stale error latch (the IPL's
                   own armed test at F914:0010 is never acknowledged) printed
                   "3472 Microcode" on a 3477.  The error itself is still
                   reported on A0h bit 0 and cleared by a write to A2h. */
            } else if (epoch->font24)
                ret = 0xA8; /* Mono 24 */
            else
                ret = 0xA2; /* Mono 16 */
            if (pb(PB_A0LOG))
                pclog("3477: in 0xA2 = %02X (pc=%04X:%04X) font24=%d\n", ret, CS, cpu_state.pc, ((ret == 0x88) ? 1 : 0));
                /* [PROBE] 同一次 A2 读：打印栈上的返回地址（pushf 之后 [sp+2]），认出是谁在轮询这条环 */
                pclog("3477: A2CALLER ss=%04X sp=%04X [sp]=%04X [sp+2]=%04X [sp+4]=%04X cs=%04X pc=%04X\n",
                      SS, SP, mem_readw_phys((((uint32_t) SS) << 4) + (SP & 0xFFFF)),
                      mem_readw_phys((((uint32_t) SS) << 4) + ((SP + 2) & 0xFFFF)),
                      mem_readw_phys((((uint32_t) SS) << 4) + ((SP + 4) & 0xFFFF)),
                      CS, cpu_state.pc);
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
    if (pb(PB_ADR) && port >= 0x1B0 && port <= 0x1D6)
        pclog("3477: ADR in %03X = %02X pc=%04X:%04X\n", port, ret, cs >> 4, cpu_state.pc);
    return ret;
}

static void
epoch_misc_out(uint16_t port, uint8_t val, void *priv)
{
    if (port == 0xCF)
        pclog("3477: TESTID %02X pc=%04X:%04X\n", val,
              (CS > 0xFFFF) ? (int) (CS >> 4) : (int) CS, cpu_state.pc);
    if (port == 0x1B0)
        /* 通过记录计数：实测每次跑批只出现 2 次 OUT1B0 00/ax=0000，
           不是「每项一次」，故仅作现场计数，不当进度用 */
        { static int out1b0_pass = 0;
        if ((val == 0x00) && ((AX & 0xFFFF) == 0x0000))
            out1b0_pass++;
        pclog("3477: OUT1B0 %02X ax=%04X pc=%04X:%04X [2F]=%02X [28]=%02X pass=%d\n", val, AX,
              (CS > 0xFFFF) ? (int) (CS >> 4) : (int) CS, cpu_state.pc, epoch_adr_latch,
              mem_readb_phys(((uint32_t) DS << 4) + 0x28), out1b0_pass); }
    if (getenv("EPOCH3477_ADR_RW")) {
        if (port == 0x1C0) {
            if ((val == 0x40) || (val == 0x08)) {
                /* 0x440 进行中（上次命令是 0x40 且还没收完一帧）时，忽略后来的 0x08，
                   否则帧内容会被换成 d=0xFF 那一版（实测导致 BH=0x0E、校验 0） */
                int hold = (val == 0x08) && (rw_cmdlat == 0x40) && !rw_done40;
                if (!hold) {
                    rw_cmdlat = val; rw_ph = 1; rw_rd = 0; rw_serve = 1; rw_fno++;
                    if (val == 0x40) rw_done40 = 0;
                    rw_word = 0; rw_inframe = 0; rw_hist = 0;
                    pclog("3477: RWM ARM %02X\n", val);
                } else {
                    pclog("3477: RWM ARM %02X ignored (0x40 pending)\n", val);
                }
            }
        }
        if ((port == 0x1C6) && (val & 0x40)) {
            int d = (val >> 4) & 1;
            rw_hist = ((rw_hist << 1) | (uint32_t) d) & 0x3FF;
            if (!rw_inframe) {
                if (rw_hist == 0x2AA) { rw_inframe = 1; rw_pcnt = 0; rw_word = 0; }
            } else {
                if ((rw_pcnt >= 6) && (((rw_pcnt - 6) & 1) == 0)) {
                    int n = (rw_pcnt - 6) >> 1;
                    if ((n < 16) && d) rw_word |= (uint16_t) (1u << n);
                }
                rw_pcnt++;
                if (rw_pcnt >= 38) {
                    rw_inframe = 0;
                    rw_serve = (mem_readb_phys(((uint32_t) DS << 4) + 0x2F) == 1)
                             ? (rw_word == 0x1221) : 1;
                    pclog("3477: RWM FRAME %04X serve=%d\n", rw_word, rw_serve);
                }
            }
        }
    }
    if (port == 0x1C0) {
        epoch_adr_cmd = val;
        if ((val == 0x40) || (val == 0x08)) {
            epoch_adr_cmdlat = val; epoch_adr_ph = 1; epoch_adr_rd = 0;
            epoch_adr_serve = 1; epoch_adr_word = 0; epoch_adr_inframe = 0; epoch_adr_hist = 0;
            pclog("3477: ARM cmd=%02X\n", val);
        }
    }
    if ((port == 0x1C6) && (val & 0x40) && getenv("EPOCH3477_ADR_SYNTH")) {
        int d = (val >> 4) & 1;
        epoch_adr_hist = ((epoch_adr_hist << 1) | (uint32_t) d) & 0x3FF;
        if (!epoch_adr_inframe) {
            if (epoch_adr_hist == 0x2AA) { epoch_adr_inframe = 1; epoch_adr_pcnt = 0; epoch_adr_word = 0; }
        } else {
            if ((epoch_adr_pcnt >= 6) && (((epoch_adr_pcnt - 6) & 1) == 0)) {
                int n = (epoch_adr_pcnt - 6) >> 1;
                if ((n < 16) && d) epoch_adr_word |= (uint16_t) (1u << n);
            }
            epoch_adr_pcnt++;
            if (epoch_adr_pcnt >= 38) {
                epoch_adr_inframe = 0; epoch_adr_fresh = 1; epoch_adr_polls++;
                epoch_adr_serve = (mem_readb_phys(((uint32_t) DS << 4) + 0x2F) == 1)
                                ? (epoch_adr_word == 0x1221) : 1;
                { static int fc; if (fc < 40) { pclog("3477: FRAME #%d word=%04X polls=%d serve=%d\n", fc, epoch_adr_word, epoch_adr_polls, epoch_adr_serve); fc++; } }
            }
        }
        epoch_adr_pprev = d;
    }
    if (pb(PB_ADR)) {
        /* [PROBE] 16 位字发送中的字中握手脉冲（out 1C0=08）：顺手读调用栈拿返回地址 */
        {
            static int xfer_dbg;
            if (port == 0x1C0 && val == 0x08 && xfer_dbg < 6) {
                uint32_t ssp = (((uint32_t) SS) << 4) + (SP & 0xFFFF);
                pclog("3477: XFER pc=%04X:%04X ss=%04X sp=%04X w0=%04X w1=%04X w2=%04X\n",
                      cs >> 4, cpu_state.pc, SS, SP,
                      (ram[ssp] | (ram[ssp + 1] << 8)),
                      (ram[ssp + 2] | (ram[ssp + 3] << 8)),
                      (ram[ssp + 4] | (ram[ssp + 5] << 8)));
                xfer_dbg++;
            }
        }
        {
            static int rst_init = 0;
            if (!rst_init) {
                const char *r = getenv("EPOCH3477_ADR_RST");
                epoch_adr_rst = r ? (int) strtoul(r, NULL, 0) : 0;
                rst_init = 1;
            }
            if (epoch_adr_rst && port == epoch_adr_rst) {
                epoch_adr_bitcnt = 0;
                epoch_adr_shift   = epoch_adr_payload;
            }
        }
        if (port == 0x1C1) {
            epoch_adr_reg = (uint8_t) (val | 0x42);   /* [FIX] 设备侧特征位：bit1|bit6 恒置（ROM 握手判据要 0x42） */
            if ((val & 0x42) == 0x42)
                epoch_adr_phase = 0;
        } else if (port == 0x1C2 && (val & 0x04)) {
            /* [AB] 客人从不写 0x1C1 ⇒ epoch_adr_reg 恒 0 ⇒ 原式每次选通都把相位复位成 1，
           而 0x7FF 助手的协议是「两次读之间相位翻转」(1→选通→0)。ADRSTB=1 改为选通即翻转。 */
        { static int fx2 = -1; if (fx2 < 0) fx2 = getenv("EPOCH3477_ADRFIX") ? 1 : 0; if (fx2) epoch_adr_phase = 0; }
        if (getenv("EPOCH3477_ADRSTB")) epoch_adr_phase ^= 1;
        else epoch_adr_phase = ((epoch_adr_reg & 0x42) == 0x42) ? 0 : 1;
        } else if (port == 0x1C6) {
            /* [MODEL] 位串行从设备：bit6 上升沿推进响应移位寄存器，最高位回读为 bit7 */
            static int     adr_clk_last;
            static int     adr_resp_init;
            static uint8_t adr_resp_shift;
            static int     adr_rdir;
                                                                        if (!adr_resp_init) {
                const char *rv = getenv("EPOCH3477_ADR_RESP");
                const char *dv = getenv("EPOCH3477_ADR_RDIR");
                adr_resp_shift = rv ? (uint8_t) strtoul(rv, NULL, 0) : 0;
                adr_rdir       = dv ? atoi(dv) : 0;
                {
                    const char *wv = getenv("EPOCH3477_ADR_WAIT");
                    const char *pv = getenv("EPOCH3477_ADR_PAYLOAD");
                    epoch_adr_wait    = wv ? atoi(wv) : 7;
                    epoch_adr_payload = pv ? (uint16_t) strtoul(pv, NULL, 0) : 0x5400;
                    epoch_adr_shift   = epoch_adr_payload;
                }
                adr_resp_init  = 1;
            }
            {
                int clk = (val & 0x40) ? 1 : 0;
{ const char *v = getenv("EPOCH3477_ADRPH"); if (v && atoi(v)) epoch_adr_phase ^= 1; }
                (void) clk;
                {
                    epoch_adr_bitcnt++;   /* [FIX] 每次 out 0x1C6 都计一拍，不再依赖 bit6 上升沿 */
                    if (epoch_adr_bitcnt > 600) { epoch_adr_bitcnt = 0; epoch_adr_shift = epoch_adr_payload; }   /* 新帧自动复位 */
                    if (epoch_adr_bitcnt == epoch_adr_wait)          epoch_adr_resp_bit = 1;
                    else if (epoch_adr_bitcnt > epoch_adr_wait)    { epoch_adr_resp_bit = (epoch_adr_shift & 0x8000) ? 1 : 0; epoch_adr_shift <<= 1; }
                    else                                 epoch_adr_resp_bit = 0;
                }
                adr_clk_last = clk;
            }
            epoch_adr_latch = (uint8_t) ((val & 0x7d) | (epoch_adr_resp_bit ? 0x80 : 0) | (epoch_adr_phase ? 0x02 : 0));
        }
    }

    epoch_t *epoch = (epoch_t *) priv;
    pit_intf_t *pit_intf = &pit_devs[0];

    // dev->regs[port & 0x0007] = val;
    // epoch_log("%04X:%04X I/O Out %02X: %02X\n  AX=%04X BX=%04X CX=%04X DX=%04X ES=%04X DI=%04X DS=%04X SI=%04X\n",
    //      cs >> 4, cpu_state.pc, port, val, AX, BX, CX, DX, ES, DI, DS, SI);
    if(port != 0x44)
        epoch_log("%04X:%04X I/O Out %02X: %02X\n", cs >> 4, cpu_state.pc, port, val);
    /* [PROBE EPOCH3477_ROMW] 3477-S system-ROM window selectors: 0x390 is the
       one the IPL writes right before jumping from the IPL image into the ROS
       image (mov al,0x80; mov dx,0x390; out dx,al; ... jmp F800:0092), so its
       value is what picks which 32 KB page the ROM window shows. */
    if (pb(PB_ROMW) && (port == 0x390 || port == 0x1B2 || port == 0x2C2 ||
                       (port >= 0x1B0 && port <= 0x1D6) || (port >= 0x720 && port <= 0x72F)))
        pclog("3477: ROMW out %03X = %02X pc=%04X:%04X\n",
              port, val, cs >> 4, cpu_state.pc);
    if (pb(PB_ADR) && epoch_adr_irq >= 0 &&
        (port == 0x1C0 || port == 0x1C6 || port == 0x1CA || port == 0x1D0))
        timer_set_delay_u64(&epoch_adr_timer, TIMER_USEC * 80);

    switch (port) {
        case 0x400:
            /* Display-RAM plane select (3477-S).  The burn-in reads it back
               and keeps bit 5, which therefore reads set.  The IPL also
               walks bits 7-5 as a 32 KB font-ROM bank at 0xE8000 - probe
               gate EPOCH3477_EBTRC=1 (off by default). */
            if (pb(PB_EBTRC))
                pclog("3477: EBTRC out 0x400 = %02X pc=%04X:%04X\n",
                    val, cs >> 4, cpu_state.pc);
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
            if (pb(PB_DEAD)) kr_ring[kr_n++ & 15] = (uint16_t) val;
            if (pb(PB_ROMW))
                pclog("3477: A0W out A0 = %02X pc=%04X:%04X\n", val, cs >> 4, cpu_state.pc);
            if (pb(PB_DEAD)) {
                static long dn;
                uint32_t dl = (uint32_t) (cs + cpu_state.pc);

                if (dn++ < 300)
                    pclog("3477: DEAD A0W v=%02X page=%d lin=%06X off=%05X\n",
                          val, epoch_rom_cur, dl, (dl - 0xF8000u) & 0xFFFFFu);
            }
            nmi_mask = val & 0x80;
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
                /* [AB] Bit 7 arms one parity error for the next memory read,
                   NOT bit 5: the IPL's own recipe is 88h (bit 5 clear, no arm)
                   then A8h (bit 5 set), and driving the arm from bit 5 instead
                   makes the parity loop at F800:01F1 fail with its own code 5
                   at F800:021B - so the arm is load-bearing and bit 7 is what
                   raises it.  The System ROS stores with 88h/A8h (both with
                   bit 7 set) and loads the word back, requiring the A0h bit 0
                   latch.  The deliberate case is F800:0833
                   (`mov al,0xA9 / out A0h,al`): the very next read
                   (F914:000C) consumes the arm and is also how the IPL fetches
                   the SS base at 0x131.  That read is a test, not a fault, so
                   an armed error latches A0h bit 0 WITHOUT raising an NMI -
                   raising one there drops the IPL straight into its
                   interrupt-cause dispatch (F800:0D4B) and it hands the board
                   back with AL=0xFA before reaching the acknowledge at
                   F914:0040.  Real faults (the paritybad bitmap) still
                   propagate as an NMI. */
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
            if (epoch_is_3477)
                epoch_rompg_switch(val); /* [ROMPG] no-op unless paging is loaded */
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
    uint8_t  ret   = epoch->fontcard_win[addr & (EPOCH_FONTCARD_WIN_SIZE - 1)];


    return ret;
}

static uint16_t
epoch_fontcard_readw(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    addr &= EPOCH_FONTCARD_WIN_SIZE - 1;

    /* [Cloud server only, DO NOT USE!!!] same read counter as readb. */
    {
        static int      on = -1;
        static uint32_t n  = 0;

        if (on < 0)
            on = (getenv("CARDRD") != NULL) ? 1 : 0;
        if (on) {
            n++;
            if (n <= 60)
                pclog("3477: card rdw %04X (pc=%04X:%04X)\n", addr,
                      (unsigned) (cs >> 4), cpu_state.pc);
            if ((n % 4096) == 0)
                pclog("3477: card rdw 计数 %u\n", n);
        }
    }

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
                    /* 3477-S disassembly: IBMBIO latches the card page by WRITING
                       F000:0..15 (mov [bx],bl, bx==bl).  The handlers exist and
                       match that protocol; they were disabled for this epoch.
                       EPOCH3477_FCWR=1 re-enables them for A/B probing only
                       (never in the clean 80186 baseline). */
                    (!epoch_is_3477 || (pb(PB_FCWR))) ? epoch_fontcard_writeb : NULL,
                    (!epoch_is_3477 || (pb(PB_FCWR))) ? epoch_fontcard_writew : NULL, NULL, NULL,
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
/* 3477-S: the E800 window (32 KB, 0xE8000-0xEFFFF) is a banked view of the ROS
   FONT ROM - port 0x400's bits 7-5 pick the bank, and the IPL walks all 8 of
   them (0xE0,0xC0,..,0x00) checksumming 32 KB per bank, while its read-back
   keeps only bit 5 and bits 1-0 select the display plane.  Gated off by
   default: EPOCH3477_EBANK=1 enables it for A/B probing only.  [Cloud server
   only, DO NOT USE!!!] */
static int
epoch_ebank_enabled(void)
{
    static int ebank = -1;
    if (ebank < 0)
        ebank = (pb(PB_EBANK));
    return ebank;
}
static uint8_t
epoch_hiram_readb(uint32_t addr, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    if (epoch_ebank_enabled() && epoch_is_3477 && (epoch->fontcard_rom != NULL)
        && (addr >= 0xE8000) && (addr < 0xF0000)) {
        uint32_t off = (addr - 0xE8000)
            + ((((uint32_t) epoch->dispbank >> 5) & 7) * 0x8000);
        return epoch->fontcard_rom[off & (EPOCH_FONTCARD_ROM_SIZE_3477 - 1)];
    }
    if (getenv("EPOCH5250_POLLOG") && (addr == 0xE4000)) {
        static long e40 = 0; e40++;
        if (e40 <= 40)
            pclog("3477: E4000RD n=%ld pc=%04X val=%02X\n", e40, cpu_state.pc,
                  epoch->hiram[(addr - 0xE1000) & 0xFFFF]);
    }
    if (getenv("EPOCH5250_POLLOG") && (addr >= 0xE4000) && (addr < 0xE4100)
        && (cpu_state.pc >= 0xA40) && (cpu_state.pc <= 0xA70)) {
        static long e4all = 0; e4all++;
        if (e4all <= 300)
            pclog("3477: E4A n=%ld addr=%05X val=%02X pc=%04X\n", e4all, addr,
                  epoch->hiram[(addr - 0xE1000) & 0xFFFF], cpu_state.pc);
    }
    if (getenv("EPOCH5250_POLLOG") && (addr >= 0xE4000) && (addr < 0xE4010)) {
        static long e4r = 0; e4r++;
        if (e4r <= 6) pclog("3477: E4R n=%ld addr=%05X=%02X\n", e4r, addr,
                            epoch->hiram[(addr - 0xE1000) & 0xFFFF]);
    }
    return epoch->hiram[(addr - 0xE1000) & 0xFFFF];
}
static uint16_t
epoch_hiram_readw(uint32_t addr, void *priv)
{
    return (uint16_t) (epoch_hiram_readb(addr, priv)
        | (epoch_hiram_readb(addr + 1, priv) << 8));
}
static void
epoch_hiram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    /* [PROBE] EPOCH3477_HIRAM=1: count the guest's writes into the E800-EC00
       block, which is where IBMBIO is supposed to land the delivered glyphs. */
    if (pb(PB_HIRAM)) {
        static long hn = 0;
        hn++;
        if ((hn <= 12) || ((hn % 4096) == 0))
            pclog("3477: HIRAMW n=%ld addr=%05X=%02X\n", hn, addr, val);
    }
    if (getenv("EPOCH5250_POLLOG") && (addr == 0xE4000)) {
        static long e4n = 0; e4n++;
        pclog("3477: E4W n=%ld pc=%04X val=%02X\n", e4n, cpu_state.pc, val);
    }
    ((epoch_t *) priv)->hiram[(addr - 0xE1000) & 0xFFFF] = val;
}
static void
epoch_hiram_writew(uint32_t addr, uint16_t val, void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    if (getenv("EPOCH5250_POLLOG") && (addr == 0xE4000)) {
        static long e4w = 0; e4w++;
        pclog("3477: E4WW n=%ld pc=%04X val=%04X\n", e4w, cpu_state.pc, val);
    }
    epoch->hiram[(addr - 0xE1000) & 0xFFFF]         = val & 0xff;
    epoch->hiram[(addr - 0xE1000 + 1) & 0xFFFF]     = val >> 8;
}

/* 适配器把 0xE400:0000 起的 256 字节作为自己的共享 RAM 窗口：
   块 0x9D0 选中它之后立刻要求全 0（0x9D0 自己不清）。 */
static void
epoch_adr_shared_clear(void *priv)
{
    epoch_t *epoch = (epoch_t *) priv;
    int      i;
    /* 判据是 cmp al,[bx] + inc al：窗口里第 i 个字节必须等于 i（0,1,2,…）。 */
    if (epoch->hiram != NULL) {
        for (i = 0; i < 0x100; i++)
            epoch->hiram[0x3000 + i] = (uint8_t) i;
    }
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
        { const char *fv = getenv("EPOCH3477_FONT24"); if (fv) epoch->font24 = atoi(fv); }   /* [TEST] 覆盖 A2 显示类型位（0x390 换页门） */
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
    io_sethandler(0x03C0, 0x0010, NULL, NULL, NULL, epoch_p3c_write, NULL, NULL, epoch);

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
        if (pb(PB_ADR)) {
            {
                const char *e_irq = getenv("EPOCH3477_ADAPTER_IRQ");
                timer_add(&epoch_adr_timer, epoch_adr_irq_cb, NULL, 0);   /* 0 = 不立即启动，稍后 delay 武装 */
                epoch_adr_irq = e_irq ? atoi(e_irq) : 5;
                /* [AB] 驱动 ISR 以 ICU type 0x0C(INT0)/0x0D(INT1) 收尾 ⇒ 8259 的 icw2 基址
                   给出的向量不是它；ADR_OVR=1 时把该 IRQ 的向量覆盖到 0x0C（仿 m_ps2_isa.c:778）。 */
                if (epoch_adr_irq >= 0 && getenv("EPOCH3477_ADR_OVR"))
                    { const char *vv = getenv("EPOCH3477_ADR_OVRV");
                      pic_set_vector_override((uint8_t) epoch_adr_irq, vv ? (int) strtoul(vv, NULL, 0) : 0x0D); }
                pclog("3477: ADAPTER IRQ = %d\n", epoch_adr_irq);
            }
            if (getenv("EPOCH3477_ADRDEV")) {
                /* 独立设备模型：端口 / 状态 / 中断线全在 src/device/ibm5250.c */
                epoch->adapter = ibm5250_init(epoch_adr_irq, epoch_adr_dev_irq, epoch,
                                                epoch_adr_dev_irq_clear, epoch);
                epoch_adr_dev_handle = epoch->adapter;
                ibm5250_set_shared_clear(epoch->adapter,
                                         epoch_adr_shared_clear, epoch);
                io_sethandler(0x0722, 0x0002, NULL, NULL, NULL,
                              epoch_adr_ack_out, NULL, NULL, epoch);
                /* 0x1B0 (terminal type) and 0x1B6 (ICU release) are machine
                   resources, not adapter registers - keep them here. */
                io_sethandler(0x1B0, 0x0001, epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
                io_sethandler(0x1B6, 0x0001, epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
            } else {
                /* 只注册 5250 自检实际驱动的端口，避免抢掉启动早期要用的端口 */
                static const uint16_t adr_ports[] = {0x1B0, 0x1B2, 0x1B6, 0x1C0, 0x1C1, 0x1C2, 0x1C3, 0x1C6, 0x1C7, 0x1C8, 0x1C9, 0x1CA, 0x1CB, 0x1CC, 0x1CD, 0x1CE, 0x1CF, 0x1D0, 0x1D1, 0x1D6};
                for (int i = 0; i < (int) (sizeof(adr_ports) / sizeof(adr_ports[0])); i++) {
                    io_sethandler(adr_ports[i], 0x0001, epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
                    pclog("3477: ADAPTER port %03X registered\n", adr_ports[i]);
                }
            }
        }
        io_sethandler(0x2C2, 0x0001,
                      epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
        /* [TESTID] 无副作用探针端口 */
        io_sethandler(0x1CF, 0x0001,
                      epoch_misc_in, NULL, NULL, epoch_misc_out, NULL, NULL, epoch);
        io_sethandler(0x0CF, 0x0001,
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

    if (pb(PB_C000LOG))
        pclog("3477: rdb C000:%04X = %02X\n", addr & 0xfff, v);
    return v;
}

static uint16_t
i3477_ram_readw(uint32_t addr, void *priv)
{
    i3477_shared_t *dev = (i3477_shared_t *) priv;
    uint16_t        v   = (uint16_t) (dev->ram[addr & 0xfff] | (dev->ram[(addr + 1) & 0xfff] << 8));

    if (pb(PB_C000LOG))
        pclog("3477: rdw C000:%04X = %04X\n", addr & 0xfff, v);
    return v;
}

static void
i3477_ram_writeb(uint32_t addr, uint8_t val, void *priv)
{
    uint16_t off = addr & 0xfff;

    if ((off >= 0x1D) && (off <= 0x45)) { /* the burn-in's failure-code log */
        if (pb(PB_C000LOG))
            pclog("3477: wrb C000:%04X = %02X bp=%04X (pc=%04X:%04X)\n", off, val, BP, CS, cpu_state.pc);
    } else if (pb(PB_C000LOG))
        pclog("3477: wrb C000:%04X = %02X\n", off, val);
    ((i3477_shared_t *) priv)->ram[addr & 0xfff] = val;
}



static void
i3477_ram_writew(uint32_t addr, uint16_t val, void *priv)
{
    i3477_shared_t *dev = (i3477_shared_t *) priv;

    if (pb(PB_C000LOG))
        pclog("3477: wrw C000:%04X = %04X\n", addr & 0xfff, val);
    dev->ram[addr & 0xfff]         = val & 0xff;
    dev->ram[(addr + 1) & 0xfff]   = val >> 8;
}

static void
i3477_doorbell_out(uint16_t port, uint8_t val, void *priv)
{
    if (getenv("EPOCH3477_POST")) {
        uint32_t sl = ((uint32_t) SS << 4) + SP;
        int i;
        pclog("3477: POST port=%03X val=%02X pc=%04X:%04X sp=%04X stack:",
              port, val, CS, cpu_state.pc, SP);
        for (i = 0; i < 8; i++)
            pclog(" %04X", readmemwl(sl + 2 * i));
        pclog("\n");
    }
    if (pb(PB_C000LOG))
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

/* [PROBE] EPOCH3477_PCSAMPLE=1 时给 0x53B8/0x53B9（客人磁盘服务的完成标志）挂一个
   2 字节写监视：HB 直方图显示客人 ~63% 时间耗在等 [0x53B8] bit7 的超时环（0650:863F-8648）里，
   需要当场抓出「谁在什么时候置位」。只记写，不记读（读是热路径）。 */
static mem_mapping_t i3477_flagmap;
static long i3477_flag_reads = 0;
static uint32_t i3477_flag_last = 0xFFFFFFFF;
static uint8_t
i3477_flag_readb(uint32_t addr, void *priv)
{
    i3477_flag_reads++;
    if (i3477_flag_last == 0xFFFFFFFF) i3477_flag_last = ((uint32_t) CS << 16) | cpu_state.pc;
    return mem_readb_phys(addr);
}
static uint16_t
i3477_flag_readw(uint32_t addr, void *priv)
{
    return mem_readw_phys(addr);
}
static void
i3477_flag_writeb(uint32_t addr, uint8_t val, void *priv)
{
    static long wn = 0;

    mem_writeb_phys(addr, val);
    wn++;
    if ((wn <= 40) || ((val & 0x80) != 0))
        pclog("3477: FLAGW n=%ld [%04X] <- %02X pc=%04X:%04X\n",
              wn, addr, val, CS, cpu_state.pc);
}
static void
i3477_flag_writew(uint32_t addr, uint16_t val, void *priv)
{
    i3477_flag_writeb(addr, (uint8_t) (val & 0xff), priv);
    mem_writeb_phys(addr + 1, (uint8_t) (val >> 8));
}

static void
i3477_sample(void *priv)
{
    i3477_shared_t *dev = (i3477_shared_t *) priv;
    static int      n   = 0;
    static int      hb  = 0;
    static int      hbn = 0;

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
        hbn++;
        pclog("3477: HB PC %04X:%04X reads=%ld first=%04X:%04X\n", CS, cpu_state.pc,
              i3477_flag_reads, (unsigned) (i3477_flag_last >> 16), (unsigned) (i3477_flag_last & 0xFFFF));
        if (hbn >= 10) {
            uint8_t fv = mem_readb_phys(0x53B8);
            if ((fv & 0x80) == 0) {
                mem_writeb_phys(0x53B8, (uint8_t) (fv | 0x80));
                pclog("3477: FORCE [53B8] %02X -> %02X (hb=%d)\n", fv, fv | 0x80, hbn);
            }
        }

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

    /* 20 µs 的 PC 采样定时器是纯探针（每秒 5 万次回调）：要在 EPOCH3477_PCSAMPLE=1
       时才武装，否则连 timer 都不建。 */
    if (pb(PB_PCSAMPLE)) {
        timer_add(&dev->sample_timer, i3477_sample, dev, 0);
        timer_set_delay_u64(&dev->sample_timer, 50000ULL * TIMER_USEC);
        /* 同一开关下顺带装 0x53B8 写监视（2 字节，盖在 RAM 上） */
        mem_mapping_add(&i3477_flagmap, 0x53B8, 2, i3477_flag_readb, i3477_flag_readw, NULL,
                        i3477_flag_writeb, i3477_flag_writew, NULL, NULL, MEM_MAPPING_EXTERNAL, dev);
    }

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
    if (pb(PB_ROMPG)) {
        /* [ROMPG] 32 KB paged system ROS in the F8000 window.  Base mappings
           may not overlap in mem_mapping_add (three rom_init()s at the same
           address abort the emulator), so the pages live in plain buffers
           behind ONE external read-handler mapping and the switch is a
           pointer swap.  F0000-F7FFF is then left to the font card instead of
           being shadowed by the flat image's lower half. */
        /* Two pages only: both self-boot from their own 0xFFF50 stub and each
           carries a 0x390 writer.  card0.bin is NOT a page (no reset vector,
           no 0x7F40 template, zero 0x390 write sites) - it is a data image. */
        static const struct { const char *fn; int off; } epg[2] = {
            { "roms/machines/ibm3477/3477ipl.bin", 0x0000 }, /* page 0: IPL half */
            { "roms/machines/ibm3477/3477ipl.bin", 0x8000 }, /* page 1: ROS half (== 3477rom.bin) */
        };
        int i;

        ret = 0;
        epoch_rom_pages = 0;
        for (i = 0; i < 2; i++) {
            FILE *fp = rom_fopen(epg[i].fn, "rb");

            if (fp == NULL)
                break;
            fseek(fp, (long) epg[i].off, SEEK_SET);
            epoch_rom_pagebuf[i] = malloc(0x8000);
            if (fread(epoch_rom_pagebuf[i], 1, 0x8000, fp) != 0x8000) {
                free(epoch_rom_pagebuf[i]);
                fclose(fp);
                break;
            }
            fclose(fp);
            epoch_rom_pages = i + 1;
        }
        if (epoch_rom_pages) {
            mem_mapping_add(&epoch_rom_map, 0xf8000, 0x8000,
                            epoch_rom_readb, epoch_rom_readw, NULL,
                            NULL, NULL, NULL, NULL,
                            MEM_MAPPING_EXTERNAL, NULL);

            ret = 1;
            epoch_rom_cur = -1;
            {
            if (getenv("EPOCH3477_ROMX")) epoch_rom_trace = 0;
            const char *rpp = getenv("EPOCH3477_ROMPG_PAGE");
            epoch_rompg_switch((uint8_t) (rpp ? atoi(rpp) : 0));
        } /* [ROMPG] EPOCH3477_ROMPG_PAGE = raw 0x390 value at reset: 0 = page 0 (IPL half, default), 128 = page 1 (ROS half) */ /* reset-visible page = page 0 (IPL half) */
        }
        pclog("3477: ROMPG %d pages loaded, mapping %s\n", epoch_rom_pages,
              epoch_rom_pages ? "at F8000-FFFFF" : "NOT installed");
    } else {
        ret = bios_load_linear("roms/machines/ibm3477/3477ipl.bin",
                               0x000f0000, 65536, 0);
    }

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
    { const char *fx = getenv("EPOCH3477_ADRFIX");
      if (fx && atoi(fx)) { int i; for (i = 0; i < 256; i++) ram[0xE4000 + i] = (uint8_t) i; } }
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
            /* [EXPERIMENT] A/B the ROM bank from the shell: EPOCH3477_TESTMODE=0/1
               overrides whatever the config resolved.  Probe-only hook, must not
               reach the clean 80186 baseline. */
            if (getenv("EPOCH3477_TESTMODE") != NULL)
                epoch->testmode = atoi(getenv("EPOCH3477_TESTMODE"));
    /* [3477-S] Floppy turbo: EPOCH3477_TURBO=1 enables.  Default is OFF because
       the 3477-S ROS does not recognise the drive when turbo is active. */
            if (getenv("EPOCH3477_TURBO")) {
                fdd_set_turbo(&drives[0], 1);
            }
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

    /* The 3477-S card is a GB2312 kanji card.  Its 1 MB image (font.bin,
       32 pages) is a flat glyph array: g0..g862 = GB2312 rows 1-9 (symbols),
       g863..g7773 = rows 16-87 (hanzi), the rest = the card's own parts; every
       glyph is 128 bytes = 32 rows of 4 bytes = a 32x32 bitmap, MSB left.  Row
       3 carries the full-width ASCII set - its position n is code 0x20 + n and
       the row starts at g192, so a half-width code maps to glyph
       191 + (code - 0x20) (verified against the extracted font: g192 = full
       width '!', g207 = '0', g224 = 'A', g249 = 'Z').  The font-24 text
       renderer fetches the generator from EPOCH_VRAM_SBCS (0x38000), 128 bytes
       per code, and a text cell is 26 dots wide: the renderer cuts the bitmap
       in two down the middle and draws the halves side by side (the left 16
       columns' first 13 dots, then the right 16 columns' first 13 dots), so the
       machine cuts the 32-column bitmap instead of keeping a second, narrow
       font.  [Cloud server only, DO NOT USE!!!] */
    /* [FIX 2026-10-15] The FONT ROM is this machine's glyph source, not a
       crutch.  The 3477-S ROS never delivers glyphs to the generators: a
       backward slice of every "mov ds/es,imm" in 3477s_full.dis yields the
       segment bases 0x0000/0x00xx/0x1000/0x4000/0x8000/0xA000(8)/0xC000(33)/
       0xE000(29)/0xE100/0xE200(12)/0xE400(16)/0xE800/0xF000(14)/0xF800/0xFFF*,
       with no 0xD000/0xD800 at all - and 0xD0000/0xD8000 (the guests's view of
       EPOCH_VRAM_SBEX/SBCS through the 0xC1000-0xDFFFF window) is only
       reachable from a >= 0xD000 base.  Its only FONT ROM access is the
       burn-in checksum walk (000F04A0: out 0x400 <- 0xE0..0x00 stepping -0x20,
       ds = 0xE800, 16384 lodsw/add, 8 x 32 KB banks), which reads and never
       writes.  The only CRAM writers are the diagnostic status character
       (000F0599/000F05AA and the shared 000F093B/000F094C helper, es = 0xE000,
       di = 0xECC/0xE70).  Measured: probe EPOCH3477_VPAGE (4 KB page histogram
       in epoch_vram_write) shows a 150 s system-ROS run touching only pages
       0x00/0x01 of the plane (00:5248 01:2944 at n=8192) - pages 0x30..0x3F
       stay at zero.  So the generator must be filled from the machine side, and
       with it filled the diagnostic '0' (CELL probe: chr=0x30 attr=0x08 at
       ma=0x0E70) gains its 131 pixels (PIXEL nz 11440 -> 11571).
       A/B at runtime: compare with the load commented out, or git revert. */
    if ((epoch->fontcard_rom != NULL) && (epoch->fontcard_pages > 2)) {
        /* Half-width (ANK) generator: GB2312 row 3.  The position of a code in
           the row is n = code - 0x20 and the row starts at g192 (g192 is the
           full-width '!', verified against the extracted font: g192 = '!',
           g207 = '0', g224 = 'A', g249 = 'Z'), so glyph 191 + (code - 0x20) is
           the full-width form of that code.  This card carries one extra glyph
           between 'Z' and 'a' (verified: g257 = 'a', g258 = 'b', ... g263 = 'g'),
           so the lower-case block is shifted by one and uses 192 + (code - 0x20);
           reading it with 191 showed every lower-case letter as its predecessor. */
        for (uint32_t code = 0x20; code < 0x80; code++) {
            const uint32_t base = (code >= 0x61) ? 192u : 191u;
            uint8_t       *dst = epoch->vram + EPOCH_VRAM_SBCS + (code * 0x80);
            if (code == 0x20) {
                /* The card's glyph before the row-3 start is a filled block, not a
                   blank, so a space must be cleared explicitly. */
                memset(dst, 0, 128);
                continue;
            }
            const uint8_t *src = epoch->fontcard_rom + (base + (code - 0x20)) * 128u;

            /* Narrow it into the 13-dot text cell (see epoch_ank_narrow). */
            epoch_ank_narrow(src, dst);
        }
        /* The alternate generator (attribute bit 1: block and line graphics -
           e.g. the solid block the IBM logo is built from) still comes from
           card page 2, indexed from code 0x20. */
        {
            const uint8_t *page = epoch->fontcard_rom + 2 * EPOCH_FONTCARD_WIN_SIZE;
            uint8_t       *base = epoch->vram + EPOCH_VRAM_SBEX;

            for (uint32_t code = 0x20; code < 0x100; code++) {
                const uint8_t *src = page + (code - 0x20) * 0x80;
                uint8_t       *dst = base + (code * 0x80);

                /* Same narrow form as the text generator so the renderer reads
                   one layout for both. */
                epoch_ank_narrow(src, dst);
            }
        }
        pclog("3477: ANK generator <- FONT ROM GB2312 row 3 (narrowed to 13 dots); "
              "alternate generator <- FONT ROM page 2 (machine-side, guest never writes it)\n");
    } else
        pclog("3477: font ROM not installed, ANK generators left empty\n");

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
