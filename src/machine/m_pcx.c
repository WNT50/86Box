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
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/machine.h>
#include <86box/mem.h>
#include <86box/nmi.h>
#include <86box/pcx_mmu.h>
#include <86box/pic.h>
#include <86box/rom.h>
#include <86box/i8018x.h>
#include <86box/wd2791.h>
#include <86box/pcx_term.h>
#include <86box/plat_unused.h>
#include "cpu.h"

/* BIOS: two 8 KB halves of one 16-bit ROM.  The low byte half (.d43) is at
   even addresses, the high byte half (.d42) at odd addresses -- exactly what
   bios_load_interleaved() wants, first argument is the even one. */
#define PCX_BIOS_LOW      "roms/machines/pcx/361d0359.d43"
#define PCX_BIOS_HIGH     "roms/machines/pcx/361d0359.d42"
#define PCX_BIOS_ADDR     0x000fc000
#define PCX_BIOS_SIZE     0x00004000

/* Top of the address space the graphics board occupies.  With the standard
   monochrome board the BIOS reports the RAM ceiling as 0xf0000; the colour
   option moves it down to 0xc0000 and a third board to 0xd8000.  Values from
   the BIOS POST, see the file header. */
#define PCX_RAM_CEIL      0x000f0000
#define PCX_GRAPH_TOP     0x000fc000

/* I/O */
#define PCX_PIC1_PORT     0x00f800
#define PCX_PIC2_PORT     0x00f820
#define PCX_STAT_PORT     0x00f840
#define PCX_DIP_PORT      0x00f841
#define PCX_RTC_PORT      0x00f880
#define PCX_RTC_DATA      0x00f880
#define PCX_RTC_STATUS    0x00f88a
#define PCX_RTC_CMD       0x00f88b
#define PCX_NVRAM_PORT    0x00f000   /* 2 KB battery-backed config RAM */
#define PCX_NVRAM_SIZE    0x000800
#define PCX_VIDEO_DATA    0x00f980
#define PCX_VIDEO_STATUS  0x00f981
#define PCX_VIDEO_CTRL    0x00f982
#define PCX_VIDEO_EXEC    0x00f98f
#define PCX_TERM_ROM      "roms/machines/pcx/d12-graka.bin"
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
    mem_mapping_t ram_map;  /* RAM above 640 KB, up to the decode ceiling */
    mem_mapping_t hole_map; /* undecoded space: bus timeout -> NMI */
    rom_t         bios;

    uint8_t stat;    /* 0xf840: error/latch byte, write-1-to-clear */
    uint8_t dip;     /* 0xf841: read = S9 switches, write = LED latch */
    uint8_t led;     /* what the BIOS last wrote to 0xf841 */
    uint8_t dskctl;  /* 0xf904 */
    uint8_t dskctl2; /* 0xf905: second latch, read-modify-written by the POST */

    uint8_t nvram[PCX_NVRAM_SIZE];
    uint8_t rtc_data, rtc_status, rtc_cmd;

    uint8_t vid_data, vid_ctrl, vid_exec;

    pcx_term_t term; /* terminal side of the video board */
    uint8_t err_seg; /* A19..A12 of the last bus timeout / parity error */

    wd2791_t *fdc;   /* WD2791 formatter, 0xf900-0xf903 */
} pcx_t;

static pcx_t pcx;

/* ------------------------------------------------------------------------- */
/* Undecoded space                                                           */
/* ------------------------------------------------------------------------- */

/* The board has no bus error signal; a cycle to an address nothing answers is
   caught by a time-out after roughly 128 us (Servicehandbuch 5.5) and turned
   into an NMI.  MAME returns 0 from its fallback handlers; a real open bus
   reads 0xff, which is what we give the guest here -- if the POST turns out to
   look at the value, this is the place to change. */
static void
pcx_bus_timeout(uint32_t addr)
{
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
    pcx.led = val;
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
static uint8_t
pcx_rtc_read(uint16_t port, UNUSED(void *priv))
{
    switch (port) {
        case PCX_RTC_STATUS:
            return pcx.rtc_status;

        case PCX_RTC_CMD:
            return pcx.rtc_cmd;

        default:
            return pcx.rtc_data;
    }
}

static void
pcx_rtc_write(uint16_t port, uint8_t val, UNUSED(void *priv))
{
    switch (port) {
        case PCX_RTC_STATUS:
            pcx.rtc_status = val;
            break;

        case PCX_RTC_CMD:
            pcx.rtc_cmd = val;
            /* Any command completes immediately: clear the busy bit. */
            pcx.rtc_status &= 0x7f;
            break;

        default:
            pcx.rtc_data = val;
            break;
    }
}

/* Video board window.  0xf980 carries the command/data stream, 0xf981 is the
   status the POST polls (bit 1 = busy, v3 0xfce8d), 0xf982 and 0xf98f are
   set-up registers.  The board answers "always ready" until the SCN2672 model
   lands, which is enough for the POST to walk past the video step. */
static uint8_t
pcx_video_read(uint16_t port, UNUSED(void *priv))
{
    switch (port) {
        case PCX_VIDEO_STATUS:
            return 0x00; /* never busy */

        case PCX_VIDEO_CTRL:
            return pcx.vid_ctrl;

        case PCX_VIDEO_EXEC:
            return pcx.vid_exec;

        default:
            return pcx.vid_data;
    }
}

static void
pcx_video_write(uint16_t port, uint8_t val, UNUSED(void *priv))
{
    switch (port) {
        case PCX_VIDEO_CTRL:
            pcx.vid_ctrl = val;
            break;

        case PCX_VIDEO_EXEC:
            pcx.vid_exec = val;
            break;

        default:
            /* Everything below 0xf980+0 is the host -> 8031 stream: the POST
               sends a control byte and then the characters of its messages. */
            pcx.vid_data = val;
            if ((port - PCX_VIDEO_DATA) == 0)
                pcx_term_write(&pcx.term, val);
            break;
    }
}

/* Placeholders: return open bus so the POST at least does not hang on them.
   Each of these is a separate item in docs/86box/pcx-machine-plan. */
static uint8_t
pcx_stub_readb(UNUSED(uint16_t port), UNUSED(void *priv))
{
    return 0xff;
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

int
machine_pcx_init(const machine_t *model)
{
    int ret;

    /* 16 KB BIOS, assembled from the two 8 KB halves of the MAME dump. */
    ret = bios_load_interleaved(PCX_BIOS_LOW, PCX_BIOS_HIGH, PCX_BIOS_ADDR, PCX_BIOS_SIZE, 0);

    if (bios_only || !ret)
        return ret;

    pcx.stat   = 0x00;
    pcx.dskctl = 0x00;
    pcx.fdc    = NULL;
    /* 0xf841 read = S9.  Bit 0 is "no MMU" (set on the PC-D, clear on the
       PC-X); bits 4/5 ask the POST to drop into the monitor after/before POST,
       bit 6 skips the early RAM clear.  MAME hardcodes 0x01 because it models
       the PC-D.  Verified against the v3 BIOS: 0x3c reads it, tests 0x60. */
    pcx.dip  = 0x00;
    pcx.led  = 0x00;
    pcx.dskctl2 = 0x00;

    memset(pcx.nvram, 0x00, sizeof(pcx.nvram)); /* an unconfigured battery */
    pcx.rtc_data   = 0x00;
    pcx.rtc_status = 0x20; /* bits 4-6 = 0b010, idle */
    pcx.rtc_cmd    = 0x00;
    pcx_term_init(&pcx.term);
    /* A missing character generator only costs the glyphs, not the terminal:
       the page still fills in, it just renders blank. */
    pcx_term_load_font(&pcx.term, PCX_TERM_ROM);
    pcx.vid_data   = 0x00;
    pcx.vid_ctrl   = 0x00;
    pcx.vid_exec   = 0x00;
    pcx.err_seg = 0x00;

    /* RAM is contiguous up to the graphics board, unlike the PC layout that
       stops at A0000h: the machine keeps RAM all the way to 0xf0000 when it is
       fitted with 1 MB, and the BIOS sizes what is actually there. */
    {
        uint32_t ram_top   = (uint32_t) mem_size << 10;
        uint32_t ram_limit = (ram_top < PCX_RAM_CEIL) ? ram_top : PCX_RAM_CEIL;

        if (ram_limit > 0x000a0000) {
            uint32_t size = ram_limit - 0x000a0000;

            mem_set_mem_state_both(0x000a0000, size, MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);
            mem_mapping_add(&pcx.ram_map, 0x000a0000, size,
                            mem_read_ram, mem_read_ramw, mem_read_raml,
                            mem_write_ram, mem_write_ramw, mem_write_raml,
                            NULL, MEM_MAPPING_INTERNAL, NULL);
        }

        /* Everything the board does not decode: RAM above the fitted size, the
           graphics window when no board answers there, and any expansion-board
           hole.  MEM_MAPPING_CPU_ONLY keeps the on-chip DMA from raising NMIs
           through mem_readb_phys() when it walks into the hole. */
        if (ram_limit < PCX_GRAPH_TOP) {
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
        i8018x_pcb_set_ext_ack(pcb, 0, pcx_8259_ack, &pic);
        i8018x_pcb_set_ext_ack(pcb, 1, pcx_8259_ack, &pic2);
        pic_set_output_callback(&pic, pcx_icu_int0, pcb);
        pic_set_output_callback(&pic2, pcx_icu_int1, pcb);
    }

    io_sethandler(PCX_STAT_PORT, 0x0001, pcx_stat_read, NULL, NULL,
                  pcx_stat_write, NULL, NULL, NULL);
    io_sethandler(PCX_DIP_PORT, 0x0001, pcx_dip_read, NULL, NULL,
                  pcx_dip_write, NULL, NULL, NULL);
    io_sethandler(PCX_DSKCTL_PORT, 0x0001, pcx_dskctl_read, NULL, NULL,
                  pcx_dskctl_write, NULL, NULL, NULL);
    io_sethandler(PCX_DSKCTL2_PORT, 0x0001, pcx_dskctl2_read, NULL, NULL,
                  pcx_dskctl2_write, NULL, NULL, NULL);
    io_sethandler(PCX_NVRAM_PORT, PCX_NVRAM_SIZE, pcx_nvram_read, NULL, NULL,
                  pcx_nvram_write, NULL, NULL, NULL);
    io_sethandler(PCX_RTC_PORT, 0x000c, pcx_rtc_read, NULL, NULL,
                  pcx_rtc_write, NULL, NULL, NULL);
    io_sethandler(PCX_VIDEO_DATA, 0x0010, pcx_video_read, NULL, NULL,
                  pcx_video_write, NULL, NULL, NULL);

    /* WD2791 formatter: registers at 0xf900-0xf903, INTRQ on IR6, DRQ on the
       80186 DMA channel 1.  The 0xf904 latch above feeds it drive/side/DDEN. */
    pcx.fdc = wd2791_init();

    /* Not implemented yet -- see the TODO list below. */

    io_sethandler(PCX_SCSI_PORT, 0x0004, pcx_stub_readb, NULL, NULL,
                  pcx_stub_writeb, NULL, NULL, NULL);

    io_sethandler(PCX_USART1_PORT, 0x0004, pcx_stub_readb, NULL, NULL,
                  pcx_stub_writeb, NULL, NULL, NULL);
    io_sethandler(PCX_USART2_PORT, 0x0004, pcx_stub_readb, NULL, NULL,
                  pcx_stub_writeb, NULL, NULL, NULL);
    io_sethandler(PCX_USART3_PORT, 0x0004, pcx_stub_readb, NULL, NULL,
                  pcx_stub_writeb, NULL, NULL, NULL);

    /* The MMU sits on port 0x8000-0x8fff and translates every CPU access to the
       RAM window in 1 KB pages; it is also the place that catches the hole when
       the board layout later maps more RAM than the configuration has. */
    pcx_mmu_init(PCX_MMU_SINIX12);
    pcx_mmu_set_hole(0, 0); /* the mapping above already reports the hole */

    return ret;
}

const device_t pcx_device = {
    .name          = "Siemens PC-X",
    .internal_name = "pcx",
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
