/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          WD1002/WD1004 hard disk controller emulation, split into
 *          three layers: the WD1010/WD2010 task file, the WD1015
 *          buffer manager control processor and the WD11C00 PC/XT
 *          host interface (WD1100-11 on the Tandy 2000HD). WD2010
 *          extends WD1010 with a 32-bit ECC engine and one more
 *          cylinder address bit.
 *
 * Authors: Karl-Ludwig Deisenhofer
 *          Curt Coder
 *          WNT50
 *
 *          Copyright 2015-2017 Karl-Ludwig Deisenhofer.
 *          Copyright 2010-2026 Curt Coder.
 *          Copyright 2026 WNT50.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/hdc.h>
#include <86box/hdd.h>
#include <86box/io.h>
#include <86box/ui.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/dma.h>
#include <86box/pic.h>
#include <86box/hdc_st506_wd10.h>
#include <86box/plat_unused.h>

/* Sector geometry, shared by both layers. */
#define SECTOR_SIZE               512
#define MFM_SECTORS               17
#define RLL_SECTORS               26

/* ---- Layer 1: WD1010 / WD2010 task file ----------------------------- */

/* Task file commands. */
#define WD1010_CMD_RESTORE        0x10
#define WD1010_CMD_READ_SECTOR    0x20
#define WD1010_CMD_WRITE_SECTOR   0x30
#define WD1010_CMD_SECT_SCAN_ID   0x40
#define WD1010_CMD_WRITE_FORMAT   0x50
#define WD1010_CMD_SEEK           0x70
#define WD1010_CMD_DIAGS          0x90

/* Error register bits, WD1010/WD2010 data sheet (1986 handbook 3-119).
   These are bit conditions in the chip, not the codes the host sees. */
#define WD1010_ERR_DAM            0x01 /* data address mark not found */
#define WD1010_ERR_TK0            0x02 /* track 000 not found */
#define WD1010_ERR_AC             0x04 /* aborted command */
#define WD1010_ERR_ID             0x10 /* ID field not found */
#define WD1010_ERR_DATA           0x40 /* CRC or ECC error in the data field
                                        (bit 5 is reserved and always 0) */
#define WD1010_ERR_BB             0x80 /* ID field carried a Bad Block Mark */

/* Mechanism states. */
#define WD1010_STATE_IDLE         0
#define WD1010_STATE_READ_WAIT    1
#define WD1010_STATE_WRITE_WAIT   2
#define WD1010_STATE_FORMAT_SEEK  3
#define WD1010_STATE_FORMAT_SEND  4
#define WD1010_STATE_SEEK_WAIT    5

/* Time the watchdog allows for the ID field of one sector
   during FORMAT TRACK. The initial seek overhead for the head
   to reach the track is now supplied by wd1010_hdc_seek_delay(). */
#define WD1010_FORMAT_SECTOR      (1000 * TIMER_USEC)

/* Delay used when the selected drive isn't present. hdd_seek_get_time()
   cannot be asked about a missing drive, so the caller still gets a
   nonzero timer value for the seek to "complete". */
#define WD1010_SEEK_ABSENT_TIME   (20 * TIMER_USEC * 1000)

/* Command timer granularity. */
#define WD1015_TIME               (250 * TIMER_USEC)
#define WD1015_TIME_MS            (1000 * TIMER_USEC)

/* Status register. */
#define WD1015_STAT_REQ           0x01 /* controller ready */
#define WD1015_STAT_IO            0x02 /* input, data to host */
#define WD1015_STAT_CD            0x04 /* command mode (else data) */
#define WD1015_STAT_BSY           0x08 /* controller is busy */
#define WD1015_STAT_DRQ           0x10 /* controller needs DMA */
#define WD1015_STAT_IRQ           0x20 /* interrupt, we have info */

/* DMA/IRQ enable register. */
#define WD1015_DMA_ENA            0x01 /* DMA operation enabled */
#define WD1015_IRQ_ENA            0x02 /* IRQ operation enabled */

/* Error codes in sense report. */

/* No, um, errors.. */
#define WD1015_ERR_NONE           0x00

/* Group 0: drive errors. */
#define WD1015_ERR_NOT_READY      0x04 /* drive not ready */
#define WD1015_ERR_TRACK0         0x06 /* track 0 not found */

/* Group 1: controller errors. */
#define WD1015_ERR_UNC_ERR        0x11 /* uncorrectable data */
#define WD1015_ERR_DAM            0x12 /* data address mark not found */
#define WD1015_ERR_SEEK_ERROR     0x15 /* seek error- cyl not found */
#define WD1015_ERR_BAD_TRACK      0x19 /* track flagged bad */
#define WD1015_ERR_NO_RECOVERY    0x1f /* recovery mode not avail (vendor code) */

/* Group 2: command errors. */
#define WD1015_ERR_BAD_COMMAND    0x20 /* invalid command */
#define WD1015_ERR_ILLEGAL_ADDR   0x21 /* address beyond disk size */
#define WD1015_ERR_BAD_PARAMETER  0x22 /* invalid command parameter */

/* Group 3: misc errors. */

/* Controller commands. */
#define WD1015_CMD_TEST_DRIVE_READY   0x00
#define WD1015_CMD_RECALIBRATE        0x01
#define WD1015_CMD_STATUS             0x03
#define WD1015_CMD_FORMAT_DRIVE       0x04
#define WD1015_CMD_VERIFY             0x05
#define WD1015_CMD_FORMAT_TRACK       0x06
#define WD1015_CMD_FORMAT_BAD_TRACK   0x07
#define WD1015_CMD_READ               0x08
#define WD1015_CMD_WRITE              0x0a
#define WD1015_CMD_SEEK               0x0b
#define WD1015_CMD_SPECIFY            0x0c
#define WD1015_CMD_READ_ECC_BURST_LEN 0x0d
#define WD1015_CMD_READ_BUFFER        0x0e
#define WD1015_CMD_WRITE_BUFFER       0x0f
#define WD1015_CMD_RAM_DIAGNOSTIC     0xe0
#define WD1015_CMD_DRIVE_DIAGNOSTIC   0xe3
#define WD1015_CMD_CTRLR_DIAGNOSTIC   0xe4
#define WD1015_CMD_READ_LONG          0xe5
#define WD1015_CMD_WRITE_LONG         0xe6

/* ---- WD1015 / WD11C00 board types and ROM images --------------------- */

/* Board types. */
#define ST506_XT_TYPE_WDXT_GEN           0
#define ST506_XT_TYPE_WD1002A_WX1        1
#define ST506_XT_TYPE_WD1002A_WX1_NOBIOS 2
#define ST506_XT_TYPE_WD1004A_WX1        3
#define ST506_XT_TYPE_WD1002A_27X        4
#define ST506_XT_TYPE_WD1004_27X         5
#define ST506_XT_TYPE_WD1004A_27X        6

/* ROM images. */
#define WDXT_GEN_BIOS_FILE      "roms/hdd/st506/wdxt-gen/62-000128-000.bin"
#define WD1002A_WX1_BIOS_FILE   "roms/hdd/st506/wd1002a_wx1-62-000094-032.bin"
#define WD1002A_27X_BIOS_FILE   "roms/hdd/st506/wd1002a_27x-62-000094-032.bin"
#define WD1004A_BIOS_FILE       "roms/hdd/st506/western_digital_WD1004A-27X.bin"

/* Host transfer state, owned by the port layer. A command handler only
   reads it to find out whether the host finished moving a data block. */
enum {
    WD1015_BUS_IDLE,       /* no command in progress */
    WD1015_BUS_COMMAND,    /* collecting the six-byte command block */
    WD1015_BUS_POLL,       /* waiting on the chip or on a command timer */
    WD1015_BUS_OUT,        /* host is reading a data block */
    WD1015_BUS_IN,         /* host is writing a data block */
    WD1015_BUS_COMPLETION  /* waiting for the host to read the completion byte */
};

/* Phase inside the current command, advanced by its own handler. */
enum {
    WD1015_PHASE_START,      /* command accepted, first handler pass */
    WD1015_PHASE_XFER,       /* moving sectors or waiting on the chip */
    WD1015_PHASE_BLOCK_DONE, /* one data block finished (set by the port layer) */
    WD1015_PHASE_DONE        /* finished, waiting for the timer to retire it */
};

/* Chip status as the board sees it, returned by wd1015_chip_poll(). */
enum {
    WD1015_CHIP_DRQ,   /* the chip wants a sector moved */
    WD1015_CHIP_BUSY,  /* the mechanism is still working */
    WD1015_CHIP_DONE,  /* command finished without error */
    WD1015_CHIP_ERROR  /* command finished with an error */
};

/* ---- Types ---------------------------------------------------------- */

/* The WD2010 is the same task file with a 32-bit ECC engine (READ/WRITE
   with ECC is accepted) and one more cylinder address bit. The WD5011
   (RLL) keeps the same task file; its 10 MHz codec and 56-bit ECC
   live outside this model. */
typedef struct wd1010_variant_t {
    uint8_t cyl_mask; /* mask applied to the high cylinder register */
    uint8_t ecc;      /* accept READ/WRITE with ECC (command bit 1) */
} wd1010_variant_t;

/* The chip: task file registers, sector buffer and mechanism timers.
   The host interface around it lives in the board files. */
struct wd1010_t {
    const wd1010_variant_t *var;
    const char *name; /* model name for the log prefix */

    void  (*intrq)(void *priv, int level);
    void  *priv;
    int   irq_level;

    uint8_t  buf[SECTOR_SIZE];
    uint16_t buf_ptr;

    uint8_t  status;
    uint8_t  errors;
    uint8_t  sector;
    uint8_t  cyl_lo;
    uint8_t  cyl_hi;
    uint8_t  count;
    uint8_t  sdh;

    uint8_t  head;
    uint8_t  hs3; /* head bit 3, driven by the board, not by the chip */
    uint8_t  unit;

    uint8_t  present[MFM_NUM];
    uint8_t  hdd_num[MFM_NUM];
    uint32_t tracks[MFM_NUM];
    uint32_t spt[MFM_NUM];
    uint32_t hpc[MFM_NUM];

    uint32_t   xfers;   /* sectors left in the current transfer */
    uint32_t   fmt_bad; /* sectors the host marked as bad blocks */
    uint32_t   fmt_bytes;
    uint8_t   *bad_map[MFM_NUM]; /* Bad Block Marks recorded in the ID fields */
    uint32_t   bad_bits[MFM_NUM];
    uint32_t   lba_prev;
    uint32_t   sectors;
    int        state;
    pc_timer_t timer;
};

typedef struct wd1015_drive_t {
    int8_t  present;
    uint8_t hdd_num;

    uint8_t  spt; /* physical parameters */
    uint8_t  hpc;
    uint16_t tracks;

    uint8_t  cfg_spt; /* configured parameters */
    uint8_t  cfg_hpc;
    uint16_t cfg_cyl;
} wd1015_drive_t;

typedef struct wd1015_t {
    uint8_t type; /* controller type */

    uint8_t spt; /* sectors-per-track for controller */

    uint16_t base; /* controller configuration */
    int8_t   irq;
    int8_t   dma;
    uint8_t  switches;
    uint8_t  nr_err;
    uint8_t  err_bv;
    uint32_t bios_addr;
    rom_t    bios_rom;

    int        state; /* host transfer state (WD1015_BUS_*) */
    int        phase; /* phase of the current command (WD1015_PHASE_*) */
    uint8_t    irq_dma;
    uint8_t    error;
    uint8_t    status;
    pc_timer_t timer;

    uint8_t command[6]; /* current command request */
    int     drive_sel;
    int     sector;
    int     head;
    int     cylinder;
    int     count;
    int     left;  /* sectors still to verify */
    uint8_t bad;   /* format marks every sector as a bad block */
    uint8_t long_tail; /* READ/WRITE LONG is moving the four ECC bytes */
    uint8_t compl; /* current request completion code */

    int buff_pos;  /* pointers to the RAM buffer */
    int buff_cnt;

    wd1010_t *wd;     /* WD1010/WD2010 chip, NULL when the board runs its own state machine */
    uint8_t xfer_reg; /* bytes come from the chip's data register (DRQ stream)
                         rather than straight out of the sector buffer */
    uint8_t hs3;      /* board-driven head bit 3 */

    uint8_t *buff;    /* the chip's sector buffer; the board has no RAM of its
                         own (WD11C00 manual: one 1K RAM, four pages) */
    wd1015_drive_t drives[MFM_NUM];  /* the attached drives */
} wd1015_t;

/* The Tandy 2000HD option is a WD1010 with a one-sector buffer in front of it.
   The host moves a sector through the buffer at register 0 and drives the WD1010
   task file at registers 1-7; the board has no DOR and nothing raises INTRQ, so
   the host polls the status register. This wrapper supplies the IO mapping and
   the image lifecycle around the chip. */
typedef struct wd1100_tandy_t {
    wd1010_t  *wd;
    uint8_t  present[MFM_NUM];
    uint8_t  hdd_num[MFM_NUM];
} wd1100_tandy_t;

/* Supported drives table for the boards without a BIOS-set geometry. */
typedef struct wd1100_olivetti_t {
    uint16_t tracks;
    uint8_t  hpc;
    uint8_t  spt;
} wd1100_olivetti_t;

/* ---- Static data ---------------------------------------------------- */

static const wd1010_variant_t wd1010_variant_1010 = { 0x03, 0 };
static const wd1010_variant_t wd1010_variant_2010 = { 0x07, 1 };
static const wd1010_variant_t wd1010_variant_5011 = { 0x07, 1 };

static wd1100_olivetti_t hd_types_olivetti[16] = {
  // clang-format off
    {  697,  5, MFM_SECTORS },
    {  612,  4, MFM_SECTORS }, /* type 16 */
    {  612,  4, MFM_SECTORS }, /* type 16 */
    {  306,  4, MFM_SECTORS }, /* type 0  */
    {  612,  8, MFM_SECTORS },
    {  820,  6, MFM_SECTORS },
    {  820,  6, MFM_SECTORS },
    {  823, 10, MFM_SECTORS },
    {  981,  5, MFM_SECTORS },
    {  981,  5, MFM_SECTORS },
    { 1024,  8, MFM_SECTORS },
    { 1024,  9, MFM_SECTORS },
    {  872,  5, MFM_SECTORS },
    {  612,  4, MFM_SECTORS }, /* type 16 */
    {  612,  4, MFM_SECTORS }, /* type 16 */
    {  306,  4, MFM_SECTORS }  /* "not present" with the second hard disk */
  // clang-format on
};

#define ENABLE_WD_HDC_LOG 1
#ifdef ENABLE_WD_HDC_LOG
int wd_hdc_do_log = ENABLE_WD_HDC_LOG;

static void
wd_hdc_log(const char *fmt, ...)
{
    va_list ap;

    if (wd_hdc_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define wd_hdc_log(fmt, ...)
#endif

/* The chip only drives its INTRQ output; routing it to the host is up to
   the board that created the device. */
static void
wd1010_irq(wd1010_t *dev, int level)
{
    if (dev->irq_level == level)
        return;

    dev->irq_level = level;

    if (dev->intrq != NULL)
        dev->intrq(dev->priv, level);
}

static void
wd1010_hdc_led_off(void)
{
    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
}

static void
wd1010_hdc_led(wd1010_t *dev, int write)
{
    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, !write);
    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, write);
}

static void
wd1010_hdc_reset_state(wd1010_t *dev)
{
    timer_disable(&dev->timer);

    /* After a reset the error register reads 01h, the value host firmware
       probes for to confirm the controller came back. */
    dev->errors    = 0x01;
    dev->cyl_lo    = 0x00;
    dev->cyl_hi    = 0x00;
    dev->sector    = 0x00;
    dev->count     = 0x00;
    dev->head      = 0x00;
    dev->unit      = 0x00;
    dev->sdh       = 0x00;
    dev->hs3       = 0x00;
    dev->xfers     = 0;
    dev->buf_ptr   = 0;
    dev->sectors   = 0;
    dev->lba_prev  = 0;
    dev->fmt_bytes = 0;
    dev->state     = WD1010_STATE_IDLE;
    dev->status    = dev->present[0] ? (WD1010_ST_RDY | WD1010_ST_SC) : 0x00;
}

/* Turn the task file's CHS into a linear sector number, checking it
   against the geometry of the drive the SDH register selected. */
static int
wd1010_hdc_lba(wd1010_t *dev, uint32_t *lba)
{
    uint32_t cyl;
    uint32_t head;
    uint32_t sector;
    uint8_t  unit = dev->unit;

    if (!dev->present[unit])
        return 0;

    cyl    = ((uint32_t) dev->cyl_hi << 8) | dev->cyl_lo;
    head   = dev->head;
    sector = dev->sector;

    if ((sector < 1) || (sector > dev->spt[unit]) || (head >= dev->hpc[unit]) ||
        (cyl >= dev->tracks[unit]))
        return 0;

    *lba = ((cyl * dev->hpc[unit]) + head) * dev->spt[unit] + (sector - 1);

    return 1;
}

/* Calculate the delay (in 32:32 timer units) needed for the mechanism
   to reach the given linear sector. For the read and write operations
   hdd_seek_get_time() also adds the average rotational latency of the
   target sector. It updates the disk's "current head position"
   bookkeeping too, so subsequent calls can tell a same-track
   hop from a full cylinder seek. */
static uint64_t
wd1010_hdc_seek_delay(wd1010_t *dev, uint32_t dst_lba,
                    uint8_t continuous, uint8_t operation)
{
    double seek_us;

    if (!dev->present[dev->unit])
        return (uint64_t) WD1010_SEEK_ABSENT_TIME;

    seek_us = hdd_seek_get_time(&hdd[dev->hdd_num[dev->unit]],
                                dst_lba, operation, continuous, 0.0);

    wd_hdc_log("%s: seek to LBA %u, %s, %.0f us\n", dev->name,
                 dst_lba, (operation == HDD_OP_SEEK) ? "seek" : "access",
                 seek_us);

    return (uint64_t) (seek_us * TIMER_USEC);
}

static void
wd1010_hdc_command(wd1010_t *dev, uint8_t command)
{
    uint32_t lba;

    dev->buf_ptr = 0;
    dev->errors  = 0x00;
    dev->status  = WD1010_ST_RDY | WD1010_ST_SC;

    wd_hdc_log("%s: command %02X, unit %i, %i/%i/%i, count %i\n", dev->name,
                 command, dev->unit, ((uint32_t) dev->cyl_hi << 8) | dev->cyl_lo,
                 dev->head, dev->sector, dev->count);

    if (!dev->present[dev->unit]) {
        /* Nothing is attached to that unit. The controller cannot tell:
           selecting a drive is only a bit in the SDH register, so the head
           positioning commands complete as usual and only a command that needs
           an ID field fails. Answering every command with an error would make
           host firmware give up on the controller. */
        switch (command & 0xf0) {
            case WD1010_CMD_RESTORE:
            case WD1010_CMD_SEEK:
                dev->errors = 0x00;
                dev->status = WD1010_ST_RDY | WD1010_ST_SC;
                break;
            case WD1010_CMD_DIAGS:
                /* The diagnostic checks the board's own buffer RAM, so it
                   answers the same whatever is selected on the bus. */
                dev->errors = 0x01;
                dev->status = WD1010_ST_RDY | WD1010_ST_SC;
                break;
            default:
                dev->errors = WD1010_ERR_AC;
                dev->status = WD1010_ST_RDY | WD1010_ST_SC | WD1010_ST_ERR;
                break;
        }

        if (!(dev->status & WD1010_ST_BUSY))
            wd1010_irq(dev, 1);

        return;
    }

    switch (command & 0xf0) {
        case WD1010_CMD_RESTORE:
            /* The head starts from wherever it happened to sit in
               the previous command. Seek to track 0, wait the same seek
               time the other commands use, then announce completion. */
            dev->cyl_hi = dev->cyl_lo = 0x00;
            dev->status = WD1010_ST_RDY | WD1010_ST_SC | WD1010_ST_BUSY;
            dev->state  = WD1010_STATE_SEEK_WAIT;
            timer_set_delay_u64(&dev->timer,
                                wd1010_hdc_seek_delay(dev, 0, 0,
                                                    HDD_OP_SEEK));
            break;

        case WD1010_CMD_SEEK: {
            /* The cylinder/head registers hold the target. Wait for the
               head to arrive, then clear BUSY so the polling host sees
               the seek complete. */
            uint32_t cyl = ((uint32_t) dev->cyl_hi << 8) | dev->cyl_lo;
            uint32_t dst_lba = ((cyl * dev->hpc[dev->unit]) + dev->head)
                               * dev->spt[dev->unit];

            dev->status = WD1010_ST_RDY | WD1010_ST_SC | WD1010_ST_BUSY;
            dev->state  = WD1010_STATE_SEEK_WAIT;
            timer_set_delay_u64(&dev->timer,
                                wd1010_hdc_seek_delay(dev, dst_lba, 0,
                                                    HDD_OP_SEEK));
            break;
        }

        case WD1010_CMD_DIAGS:
            /* Execute Diagnostic. The model reports 01h in the error
               register, the "no error detected" result host firmware
               expects, instead of simulating the self test itself. */
            dev->errors = 0x01;
            dev->status = WD1010_ST_RDY | WD1010_ST_SC;
            break;

        case WD1010_CMD_READ_SECTOR:
        case WD1010_CMD_WRITE_SECTOR:
            /* Command bit 1 selects the ECC form of the transfer; the WD1010
               has no ECC engine and rejects it, the WD2010 accepts it (the
               ECC itself is not modelled, the data path is unchanged). */
            if ((command & 0x02) && !dev->var->ecc) {
                wd_hdc_log("%s: ECC form rejected\n", dev->name);
                dev->status |= WD1010_ST_ERR;
                dev->errors  = WD1010_ERR_AC;
                break;
            }

            if (!wd1010_hdc_lba(dev, &lba)) {
                wd_hdc_log("%s: sector past end of medium\n", dev->name);
                dev->status |= WD1010_ST_ERR;
                dev->errors  = WD1010_ERR_ID;
                break;
            }

            dev->lba_prev = lba;
            dev->status  |= WD1010_ST_BUSY;
            dev->xfers    = dev->count ? dev->count : 256;
            dev->state    = ((command & 0xf0) == WD1010_CMD_READ_SECTOR) ? WD1010_STATE_READ_WAIT : WD1010_STATE_WRITE_WAIT;

            wd1010_hdc_led(dev, dev->state == WD1010_STATE_WRITE_WAIT);

            timer_set_delay_u64(&dev->timer,
                                wd1010_hdc_seek_delay(dev, lba, 0,
                                    (command & 0xf0) == WD1010_CMD_READ_SECTOR ?
                                    HDD_OP_READ : HDD_OP_WRITE));
            break;

        case WD1010_CMD_WRITE_FORMAT: {
            /* The Tandy format utility sends one of these per track, with the
               sector count set to the sectors per track and the sector number
               register left at an unused value, so the track is numbered from
               sector one here. */
            uint32_t cyl     = ((uint32_t) dev->cyl_hi << 8) | dev->cyl_lo;
            uint32_t sectors = dev->count ? dev->count : 256;

            if ((dev->head >= dev->hpc[dev->unit]) || (cyl >= dev->tracks[dev->unit])) {
                dev->status |= WD1010_ST_ERR;
                dev->errors  = WD1010_ERR_ID;
                break;
            }

            if (sectors > dev->spt[dev->unit])
                sectors = dev->spt[dev->unit];

            dev->lba_prev  = ((cyl * dev->hpc[dev->unit]) + dev->head) * dev->spt[dev->unit];
            dev->fmt_bytes = 0;
            dev->fmt_bad   = 0;
            dev->sectors   = sectors;
            dev->status   |= WD1010_ST_BUSY;
            dev->state     = WD1010_STATE_FORMAT_SEEK;

            wd1010_hdc_led(dev, 1);

            /* The sector number register holds the Gap 1/3 length while the
               format runs (data sheet); the image model has no gaps, so it
               is accepted and ignored.
               A seek, and then the host supplies the per-sector ID data
               through the data register while DRQ is up: two bytes per
               sector, the Bad Block Mark and the logical sector number
               (WD2010 data sheet, format command). */
            timer_set_delay_u64(&dev->timer,
                                wd1010_hdc_seek_delay(dev, dev->lba_prev, 0,
                                                    HDD_OP_SEEK));
            break;
        }

        case WD1010_CMD_SECT_SCAN_ID: {
            /* Scan ID reads the cylinder number from the track under the
               heads and copies it into the present cylinder position register;
               it never seeks. Nothing moves in this model, so the head always
               sits on the track the cylinder registers name and the command
               only has to reject an address past the end of the medium. */
            uint32_t cyl = ((uint32_t) dev->cyl_hi << 8) | dev->cyl_lo;

            if (cyl >= dev->tracks[dev->unit]) {
                dev->status |= WD1010_ST_ERR;
                dev->errors  = WD1010_ERR_ID;
                break;
            }

            dev->cyl_hi = (cyl >> 8) & 0xff;
            dev->cyl_lo = cyl & 0xff;
            break;
        }

        default:
            /* Read/Write with ECC on the WD1010 and every opcode the chip
               does not define end here. No known host software issues them,
               so they answer with the same aborted command instead of
               carrying a half-model of each. */
            wd_hdc_log("%s: undefined command %02X\n", dev->name, command);
            dev->status |= WD1010_ST_ERR;
            dev->errors  = WD1010_ERR_AC;
            break;
    }

    /* Commands that do not hand the mechanism a timer have completed by
       now (seeks and transfers leave BUSY set and finish in the callback). */
    if (!(dev->status & WD1010_ST_BUSY))
        wd1010_irq(dev, 1);
}

/* Fills the track the format command selected with the E5 pattern the chip
   writes into data fields it is not given contents for. */
static void
wd1010_hdc_finish_format(wd1010_t *dev)
{
    wd_hdc_log("FMT done: sectors=%u bytes=%u bad=%u\n", dev->sectors, dev->fmt_bytes, dev->fmt_bad);
    dev->fmt_bytes = 0;
    dev->fmt_bad   = 0;
    dev->state    = WD1010_STATE_IDLE;
    dev->status  &= ~(WD1010_ST_BUSY | WD1010_ST_DRQ);

    /* Data fields the host did not supply are filled with FF (WD2010 data
       sheet, format command). */
    memset(dev->buf, 0xff, SECTOR_SIZE);

    for (uint32_t s = 0; s < dev->sectors; s++) {
        if (hdd_image_write(dev->hdd_num[dev->unit], dev->lba_prev + s, 1, dev->buf) < 0) {
            dev->status |= WD1010_ST_ERR;
            dev->errors  = WD1010_ERR_BB;
            break;
        }
    }
    wd1010_hdc_led_off();
}

/* Runs when the mechanism has finished the operation: a read or write
   releases the host with DRQ (or with the error the drive reported)
   and a seek only clears BUSY. */
static void
wd1010_hdc_callback(void *priv)
{
    wd1010_t *dev = (wd1010_t *) priv;

    wd_hdc_log("%s: callback, state %i, status %02X\n", dev->name,
                 dev->state, dev->status);

    switch (dev->state) {
        case WD1010_STATE_READ_WAIT:
            dev->state   = WD1010_STATE_IDLE;
            dev->status &= ~WD1010_ST_BUSY;

            /* A sector whose ID field carries a Bad Block Mark reports the
               detect bit instead of data; the host maps it and does not
               retry (WD1002S-WX2 table 5-5, code 19). */
            if ((dev->bad_map[dev->unit] != NULL) &&
                (dev->lba_prev < dev->bad_bits[dev->unit]) &&
                (dev->bad_map[dev->unit][dev->lba_prev >> 3] & (1 << (dev->lba_prev & 7)))) {
                dev->status |= WD1010_ST_ERR;
                dev->errors  = WD1010_ERR_DATA;
                dev->xfers   = 0;
                wd1010_hdc_led_off();
                break;
            }

            if (hdd_image_read(dev->hdd_num[dev->unit], dev->lba_prev, 1, dev->buf) < 0) {
                wd_hdc_log("%s: read error at LBA %u\n", dev->name, dev->lba_prev);
                dev->status |= WD1010_ST_ERR;
                dev->errors  = WD1010_ERR_DATA;
                dev->xfers   = 0;
                wd1010_hdc_led_off();
                break;
            }

            dev->status |= WD1010_ST_DRQ;
            break;

        case WD1010_STATE_WRITE_WAIT:
            dev->state   = WD1010_STATE_IDLE;
            dev->status &= ~WD1010_ST_BUSY;
            dev->status |= WD1010_ST_DRQ;
            break;

        case WD1010_STATE_FORMAT_SEEK:
            /* The head is over the track: request the ID of the first sector
               and let the host stream two bytes per sector through the data
               register. The watchdog covers hosts that ignore DRQ. */
            dev->fmt_bytes = 0;
            dev->buf_ptr   = 0;
            dev->status   |= WD1010_ST_DRQ;
            dev->state     = WD1010_STATE_FORMAT_SEND;

            timer_set_delay_u64(&dev->timer,
                                wd1010_hdc_seek_delay(dev, dev->lba_prev, 0,
                                                    HDD_OP_SEEK) +
                                (dev->sectors * WD1010_FORMAT_SECTOR));
            break;

        case WD1010_STATE_FORMAT_SEND:
            /* The host never supplied the IDs, so give it sequential ones. */
            wd1010_hdc_finish_format(dev);
            break;

        case WD1010_STATE_SEEK_WAIT:
            /* RESTORE or SEEK finally lands the head; clear BUSY so the
               polling host sees the seek complete. */
            dev->state   = WD1010_STATE_IDLE;
            dev->status &= ~WD1010_ST_BUSY;
            break;

        default:
            break;
    }

    /* A completed command with no data left in flight asserts INTRQ. */
    if ((dev->state == WD1010_STATE_IDLE) && (dev->xfers == 0))
        wd1010_irq(dev, 1);

    wd_hdc_log("%s: done, status %02X, error %02X\n", dev->name, dev->status, dev->errors);
}

static uint8_t
wd1010_hdc_read_data(wd1010_t *dev)
{
    uint8_t ret = dev->buf[dev->buf_ptr];

    dev->buf_ptr++;
    if (dev->buf_ptr >= SECTOR_SIZE) {
        dev->buf_ptr = 0;
        dev->status &= ~WD1010_ST_DRQ;
        wd1010_hdc_led_off();

        if (dev->xfers > 1) {
            /* Multi-sector transfer: the mechanism has to reach the next
               sector before the chip raises DRQ again. */
            dev->xfers--;
            dev->lba_prev++;
            dev->status |= WD1010_ST_BUSY;
            dev->state   = WD1010_STATE_READ_WAIT;
            timer_set_delay_u64(&dev->timer,
                                wd1010_hdc_seek_delay(dev, dev->lba_prev, 1,
                                                    HDD_OP_READ));
        } else {
            dev->xfers = 0;
            wd1010_irq(dev, 1);
        }
    }

    return ret;
}

static void
wd1010_hdc_write_data(wd1010_t *dev, uint8_t val)
{
    if (dev->state == WD1010_STATE_FORMAT_SEND) {
        /* Two bytes per sector: the Bad Block Mark first (00 normal, 80
           bad), then the logical sector number. */
        if (!(dev->fmt_bytes & 1)) {
            if (val & 0x80) {
                uint32_t lba = dev->lba_prev + (dev->fmt_bytes >> 1);

                dev->fmt_bad++;

                if ((lba < dev->bad_bits[dev->unit]) && (dev->bad_map[dev->unit] != NULL))
                    dev->bad_map[dev->unit][lba >> 3] |= (uint8_t) (1 << (lba & 7));
            }
        }

        dev->fmt_bytes++;

        if (dev->fmt_bytes >= (dev->sectors << 1)) {
            timer_disable(&dev->timer);
            wd1010_hdc_finish_format(dev);
        }

        return;
    }

    if (!(dev->status & WD1010_ST_DRQ))
        return;

    dev->buf[dev->buf_ptr] = val;

    dev->buf_ptr++;
    if (dev->buf_ptr >= SECTOR_SIZE) {
        dev->buf_ptr = 0;
        dev->status &= ~WD1010_ST_DRQ;
        wd1010_hdc_led_off();

        if (hdd_image_write(dev->hdd_num[dev->unit], dev->lba_prev, 1, dev->buf) < 0) {
            wd_hdc_log("%s: write error at LBA %u\n", dev->name, dev->lba_prev);
            dev->status |= WD1010_ST_ERR;
            dev->errors  = WD1010_ERR_DATA;
            dev->xfers   = 0;
            return;
        }

        if (dev->xfers > 1) {
            /* Multi-sector write: hand the buffer back to the host for the
               next sector straight away. */
            dev->xfers--;
            dev->lba_prev++;
            dev->buf_ptr = 0;
            dev->status |= WD1010_ST_DRQ;
        } else {
            dev->xfers = 0;
            wd1010_irq(dev, 1);
        }
    }
}

wd1010_t *
wd1010_create(const wd1010_config_t *cfg)
{
    wd1010_t *dev   = (wd1010_t *) calloc(1, sizeof(wd1010_t));
    int       units = 0;

    switch (cfg->variant) {
        case 2:
            dev->var  = &wd1010_variant_5011;
            dev->name = "WD5011";
            break;
        case 1:
            dev->var  = &wd1010_variant_2010;
            dev->name = "WD2010";
            break;
        default:
            dev->var  = &wd1010_variant_1010;
            dev->name = "WD1010";
            break;
    }
    dev->intrq = cfg->intrq;
    dev->priv  = cfg->priv;

    for (uint8_t unit = 0; unit < MFM_NUM; unit++) {
        if (cfg->unit[unit].hdd_num >= 0)
            units++;
    }

    wd_hdc_log("%s: %i unit(s)\n", dev->name, units);

    for (uint8_t unit = 0; unit < MFM_NUM; unit++) {
        if (cfg->unit[unit].hdd_num < 0)
            continue;

        dev->present[unit] = 1;
        dev->hdd_num[unit] = (uint8_t) cfg->unit[unit].hdd_num;
        dev->tracks[unit]  = cfg->unit[unit].tracks;
        dev->spt[unit]     = cfg->unit[unit].spt;
        dev->hpc[unit]     = cfg->unit[unit].hpc;

        dev->bad_bits[unit] = cfg->unit[unit].tracks * cfg->unit[unit].hpc *
                              cfg->unit[unit].spt;
        dev->bad_map[unit]  = (uint8_t *) calloc(1, (dev->bad_bits[unit] + 7) / 8);

        wd_hdc_log("%s: unit %i: hdd %i (%u/%u/%u)\n", dev->name,
                     unit, cfg->unit[unit].hdd_num, cfg->unit[unit].tracks,
                     cfg->unit[unit].hpc, cfg->unit[unit].spt);
    }

    timer_add(&dev->timer, wd1010_hdc_callback, dev, 0);

    wd1010_reset(dev);

    return dev;
}

void
wd1010_destroy(wd1010_t *dev)
{
    timer_disable(&dev->timer);

    for (uint8_t unit = 0; unit < MFM_NUM; unit++)
        free(dev->bad_map[unit]);

    free(dev);
}

void
wd1010_reset(wd1010_t *dev)
{
    wd_hdc_log("%s: reset\n", dev->name);

    wd1010_hdc_reset_state(dev);
    wd1010_irq(dev, 0);
}

void
wd1010_set_hs3(wd1010_t *dev, int level)
{
    dev->hs3  = level ? 1 : 0;
    dev->head = (dev->sdh & 0x07) | (dev->hs3 << 3);

    wd_hdc_log("%s: HS3 %i, head %i\n", dev->name, dev->hs3, dev->head);
}

uint8_t
wd1010_read(wd1010_t *dev, int reg)
{
    uint8_t ret;

    if (reg == WD1010_REG_STATUS)
        wd1010_irq(dev, 0);

    switch (reg) {
        case WD1010_REG_DATA:
            ret = wd1010_hdc_read_data(dev);
            break;
        case WD1010_REG_STATUS:
            ret = dev->status;
            break;
        case WD1010_REG_SECTOR:
            ret = dev->sector;
            break;
        case WD1010_REG_CYL_LO:
            ret = dev->cyl_lo;
            break;
        case WD1010_REG_CYL_HI:
            ret = dev->cyl_hi;
            break;
        case WD1010_REG_ERROR:
            ret = dev->errors;
            break;
        case WD1010_REG_COUNT:
            ret = dev->count;
            break;
        case WD1010_REG_SDH:
            ret = dev->sdh;
            break;
        default:
            ret = 0xff;
            break;
    }

    wd_hdc_log("%s: read(%i) = %02X\n", dev->name, reg, ret);

    return ret;
}

void
wd1010_write(wd1010_t *dev, int reg, uint8_t val)
{
    wd_hdc_log("%s: write(%i, %02X)\n", dev->name, reg, val);

    /* Writing the command register also acknowledges the previous INTRQ. */
    if (reg == WD1010_REG_STATUS)
        wd1010_irq(dev, 0);

    switch (reg) {
        case WD1010_REG_DATA:
            wd1010_hdc_write_data(dev, val);
            break;
        case WD1010_REG_ERROR:
            /* Write only: write precompensation, not modelled. */
            break;
        case WD1010_REG_COUNT:
            dev->count = val;
            break;
        case WD1010_REG_SECTOR:
            dev->sector = val;
            break;
        case WD1010_REG_CYL_LO:
            dev->cyl_lo = val;
            break;
        case WD1010_REG_CYL_HI:
            dev->cyl_hi = val & dev->var->cyl_mask;
            break;
        case WD1010_REG_SDH:
            dev->sdh  = val;
            dev->head = (val & 0x07) | (dev->hs3 << 3);
            /* The Tandy board uses bits 3-4 as a straight unit number. */
            dev->unit = (val >> 3) & 0x03;

            /* Selecting a drive clears a transfer that was left pending. */
            if (dev->status & WD1010_ST_DRQ)
                wd1010_hdc_led_off();

            dev->status &= ~WD1010_ST_DRQ;
            break;
        case WD1010_REG_STATUS:
            /* The chip ignores a new command while it is still busy with the
               previous one. */
            if (!(dev->status & WD1010_ST_BUSY))
                wd1010_hdc_command(dev, val);
            break;
        default:
            break;
    }
}

uint8_t *
wd1010_buffer(wd1010_t *dev)
{
    return dev->buf;
}

/* =====================================================================
 * Layer 2: WD1015 buffer manager control processor (CP / BMAC).
 *
 * The WD1015 executes the host command set, schedules the sector buffer,
 * keeps the per-drive task file copies, runs the ECC retry and recovery
 * logic and consolidates the status the host reads back. The host bus
 * side is the WD11C00 layer further down.
 * ===================================================================== */

/* ---- timing ------------------------------------------------ */

/* Advance the command timer by one controller tick. */
static void
wd1015_delay(wd1015_t *dev)
{
    timer_advance_u64(&dev->timer, WD1015_TIME);
}

/* Advance the command timer by a number of milliseconds. */
static void
wd1015_delay_ms(wd1015_t *dev, int ms)
{
    timer_advance_u64(&dev->timer, (uint64_t) ms * WD1015_TIME_MS);
}

/* Restart the command timer from now; used for chip polling and for the
   hand-off after the host finishes a data block. */
static void
wd1015_hold(wd1015_t *dev)
{
    timer_set_delay_u64(&dev->timer, WD1015_TIME);
}

/* ---- completion and errors ---- */

/* Retire the command: clear BUSY, raise DRQ off the bus and pulse IRQ. */
static void
wd1015_finish(wd1015_t *dev)
{
    dev->status = WD1015_STAT_REQ | WD1015_STAT_CD | WD1015_STAT_IO | WD1015_STAT_BSY;
    dev->state  = WD1015_BUS_COMPLETION;
    dev->phase  = WD1015_PHASE_DONE;

    if (dev->irq_dma & WD1015_DMA_ENA)
        dma_set_drq(dev->dma, 0);

    if (dev->irq_dma & WD1015_IRQ_ENA) {
        dev->status |= WD1015_STAT_IRQ;
        picint(1 << dev->irq);
    }
}

static void
wd1015_error(wd1015_t *dev, uint8_t err)
{
    dev->compl |= 0x02;
    dev->error = err;
}

/* Report an error and retire the command. */
static void
wd1015_finish_error(wd1015_t *dev, uint8_t err)
{
    wd1015_error(dev, err);
    wd1015_finish(dev);
}

/* Translate the chip's error register into the completion code the WD1015
   returns to the host.  The chip only reports bit conditions; turning them
   into the host codes of WD1002S-WX2 table 5-5 is the buffer manager's job,
   and passing the raw bits on gave the host codes that table does not
   define. */
static uint8_t
wd1015_chip_error(uint8_t chip)
{
    if (chip & WD1010_ERR_DATA)
        return WD1015_ERR_UNC_ERR;       /* 11h CRC/ECC error in the data field */
    if (chip & WD1010_ERR_ID)
        return WD1015_ERR_SEEK_ERROR;    /* 15h ID field not found */
    if (chip & WD1010_ERR_TK0)
        return WD1015_ERR_TRACK0;        /* 06h track 0 not found */
    if (chip & WD1010_ERR_DAM)
        return WD1015_ERR_DAM;           /* 12h data address mark not found */
    if (chip & WD1010_ERR_AC)
        return WD1015_ERR_BAD_COMMAND;   /* 20h invalid command */
    if (chip & WD1010_ERR_BB)
        return WD1015_ERR_BAD_TRACK;     /* 19h track flagged bad */

    /* The chip raised ERR without a bit this model understands. */
    return WD1015_ERR_UNC_ERR;
}

/* Read the chip status and say what the board has to do next. */
static int
wd1015_chip_poll(wd1015_t *dev)
{
    uint8_t st = wd1010_read(dev->wd, WD1010_REG_STATUS);

    if (st & WD1010_ST_ERR)
        return WD1015_CHIP_ERROR;
    if (st & WD1010_ST_BUSY)
        return WD1015_CHIP_BUSY;
    if (st & WD1010_ST_DRQ)
        return WD1015_CHIP_DRQ;

    return WD1015_CHIP_DONE;
}

/* Translate the chip's error and retire the command. */
static void
wd1015_chip_fail(wd1015_t *dev)
{
    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
    wd1015_finish_error(dev, wd1015_chip_error(wd1010_read(dev->wd, WD1010_REG_ERROR)));
}

/* ---- geometry conversion ---- */

/* Check the CHS in the command block against the configured geometry. The
   chip works the LBA out itself. */
static int
chs_valid(wd1015_t *dev, wd1015_drive_t *drive)
{
    if (!drive->present) {
        /* No need to log this. */
        dev->error = dev->nr_err;
        return 0;
    }

    if (dev->head >= drive->cfg_hpc) {
        wd_hdc_log("WD1015: CHS past end of configured heads\n");
        dev->error = WD1015_ERR_ILLEGAL_ADDR;
        return 0;
    }
    if (dev->sector >= drive->cfg_spt) {
        wd_hdc_log("WD1015: CHS past end of configured sectors\n");
        dev->error = WD1015_ERR_ILLEGAL_ADDR;
        return 0;
    }

    return 1;
}

static void
next_sector(wd1015_t *dev, wd1015_drive_t *drive)
{
    if (++dev->sector >= drive->cfg_spt) {
        dev->sector = 0;
        if (++dev->head >= drive->cfg_hpc) {
            dev->head = 0;
            if (++dev->cylinder >= drive->cfg_cyl) {
                /*
                 * This really is an error, we cannot move
                 * past the end of the drive, which should
                 * result in an WD1015_ERR_ILLEGAL_ADDR.  --FvK
                 */
                dev->cylinder = drive->cfg_cyl - 1;
            }
        }
    }
}

/* Extract the CHS info from a command block. Format commands always start at
   sector one, whatever the sector number register holds. */
static int
get_chs(wd1015_t *dev, wd1015_drive_t *drive, int format)
{
    dev->err_bv = 0x80;

    dev->head = dev->command[1] & 0x1f;
    /* 6 bits are used for the sector number even on the IBM PC controller. */
    dev->sector = format ? 1 : (dev->command[2] & 0x3f);
    dev->count  = dev->command[4];
    dev->cylinder = dev->command[3] | ((dev->command[2] & 0xc0) << 2);

    if (dev->cylinder >= drive->cfg_cyl) {
        /*
         * This really is an error, we cannot move
         * past the end of the drive, which should
         * result in an WD1015_ERR_ILLEGAL_ADDR.  --FvK
         */
        return 0;
    }

    return 1;
}

/* ---- WD1010/WD2010 chip bridge ---- */

/* WD1002A-WX1: the chip owns the transfer, the board only programs it and
   polls its status register. */

/* HS3 is the fourth head bit. It never passes through the three-bit SDH
   field of the chip, so the board drives it separately. */
static int
wd1015_program(wd1015_t *dev, uint8_t chip_cmd)
{
    int drive_sel = dev->drive_sel & 0x03;

    if ((dev->head > 15) || (dev->sector > 255))
        return 0;

    dev->hs3 = (dev->head >> 3) & 1;
    wd1010_set_hs3(dev->wd, dev->hs3);

    wd1010_write(dev->wd, WD1010_REG_COUNT, (uint8_t) dev->count);
    wd1010_write(dev->wd, WD1010_REG_SECTOR, (uint8_t) (dev->sector + 1));
    wd1010_write(dev->wd, WD1010_REG_CYL_LO, (uint8_t) (dev->cylinder & 0xff));
    wd1010_write(dev->wd, WD1010_REG_CYL_HI, (uint8_t) ((dev->cylinder >> 8) & 0x03));
    wd1010_write(dev->wd, WD1010_REG_SDH,
               (uint8_t) ((drive_sel << 3) | (dev->head & 0x07)));
    wd1010_write(dev->wd, WD1010_REG_STATUS, chip_cmd);

    return 1;
}

/* Pull the address of the last operation back out of the chip so that a
   following STATUS command reports it. */
static void
wd1015_sync(wd1015_t *dev)
{
    uint8_t  sdh   = wd1010_read(dev->wd, WD1010_REG_SDH);
    uint8_t  sec   = wd1010_read(dev->wd, WD1010_REG_SECTOR);

    dev->cylinder = ((wd1010_read(dev->wd, WD1010_REG_CYL_HI) & 0x03) << 8) |
                    wd1010_read(dev->wd, WD1010_REG_CYL_LO);
    dev->head     = (sdh & 0x07) | (dev->hs3 << 3);
    dev->sector   = sec ? (sec - 1) : 0;
}

static void
wd1015_chip_callback(wd1015_t *dev)
{
    wd1015_drive_t *drive;
    uint8_t       st;
    int           val;

    if (dev->wd == NULL) {
        wd1015_finish_error(dev, WD1015_ERR_BAD_COMMAND);
        return;
    }

    if (dev->drive_sel >= MFM_NUM) {
        wd1015_finish_error(dev, dev->nr_err);
        return;
    }

    drive = &dev->drives[dev->drive_sel];

    /* First pass: hand the command to the chip. */
    if (dev->phase == WD1015_PHASE_START) {
        if (!drive->present) {
            wd1015_finish_error(dev, dev->nr_err);
            return;
        }

        switch (dev->command[0]) {
            case WD1015_CMD_RECALIBRATE:
                dev->cylinder   = 0;
                dev->head       = 0;
                dev->sector     = 0;
                dev->count      = 0;

                if (!wd1015_program(dev, 0x10)) {
                    wd1015_finish_error(dev, WD1015_ERR_BAD_PARAMETER);
                    return;
                }
                break;

            case WD1015_CMD_SEEK:
                if (!get_chs(dev, drive, 0)) {
                    wd1015_finish_error(dev, WD1015_ERR_SEEK_ERROR);
                    return;
                }
                if (!chs_valid(dev, drive)) {
                    wd1015_finish_error(dev, WD1015_ERR_BAD_PARAMETER);
                    return;
                }
                dev->count = 0;

                if (!wd1015_program(dev, 0x70)) {
                    wd1015_finish_error(dev, WD1015_ERR_BAD_PARAMETER);
                    return;
                }
                break;

            case WD1015_CMD_READ:
            case WD1015_CMD_WRITE: {
                int write = (dev->command[0] == WD1015_CMD_WRITE);

                (void) get_chs(dev, drive, 0);
                wd_hdc_log("%s: %s %i/%i/%i, count %i\n",
                           dev->wd->name, write ? "WRITE" : "READ", dev->cylinder,
                           dev->head, dev->sector, dev->count);

                if (!chs_valid(dev, drive)) {
                    wd_hdc_log("%s: %s address out of range\n",
                               dev->wd->name, write ? "WRITE" : "READ");
                    /* READ has always reported the drive error, WRITE the
                       generic parameter code. */
                    wd1015_finish_error(dev, write ? WD1015_ERR_BAD_PARAMETER : dev->error);
                    return;
                }

                if (write)
                    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 1);
                else
                    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 1);

                if (!wd1015_program(dev, write ? 0x30 : 0x20)) {
                    if (write)
                        ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                    else
                        ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                    wd1015_finish_error(dev, WD1015_ERR_BAD_PARAMETER);
                    return;
                }
                break;
            }

            default:
                break;
        }

        dev->phase = WD1015_PHASE_XFER;
        dev->state = WD1015_BUS_POLL;
        wd1015_hold(dev);
        return;
    }

    /* Later passes: poll the chip and move the sector when it asks. */
    st = wd1015_chip_poll(dev);

    if (st == WD1015_CHIP_DRQ) {
        dev->xfer_reg = 1;
        dev->buff_pos  = 0;
        dev->buff_cnt  = SECTOR_SIZE;

        if (!(dev->irq_dma & WD1015_DMA_ENA)) {
            dev->status = WD1015_STAT_BSY | WD1015_STAT_REQ;
            if (dev->command[0] == WD1015_CMD_READ)
                dev->status |= WD1015_STAT_IO;
            dev->state = (dev->command[0] == WD1015_CMD_READ) ? WD1015_BUS_OUT : WD1015_BUS_IN;
            return;
        }

        dma_set_drq(dev->dma, 1);

        if (dev->command[0] == WD1015_CMD_READ) {
            for (dev->buff_pos = 0; dev->buff_pos < SECTOR_SIZE; dev->buff_pos++) {
                if (dma_channel_write(dev->dma,
                                      wd1010_read(dev->wd, WD1010_REG_DATA)) == DMA_NODATA) {
                    dma_set_drq(dev->dma, 0);
                    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                    wd1015_finish_error(dev, WD1015_ERR_NO_RECOVERY);
                    return;
                }
            }
        } else {
            for (dev->buff_pos = 0; dev->buff_pos < SECTOR_SIZE; dev->buff_pos++) {
                val = dma_channel_read(dev->dma);
                if (val == DMA_NODATA) {
                    dma_set_drq(dev->dma, 0);
                    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                    wd1015_finish_error(dev, WD1015_ERR_NO_RECOVERY);
                    return;
                }
                wd1010_write(dev->wd, WD1010_REG_DATA, (uint8_t) val);
            }
        }

        dma_set_drq(dev->dma, 0);

        dev->buff_pos = 0;
        dev->buff_cnt = 0;
        dev->state    = WD1015_BUS_POLL;
        wd1015_hold(dev);
        return;
    }

    if (st == WD1015_CHIP_BUSY) {
        wd1015_hold(dev);
        return;
    }

    if (st == WD1015_CHIP_ERROR) {
        wd1015_chip_fail(dev);
        return;
    }

    dev->xfer_reg = 0;
    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
    wd1015_sync(dev);
    wd1015_finish(dev);
}

/* ---- sector transfer helpers ---- */

/* Account for the access time of a transfer and raise DRQ. */
static int
wd1015_dma_out(wd1015_t *dev, const char *what)
{
    int val;

    for (; dev->buff_pos < dev->buff_cnt; dev->buff_pos++) {
        val = dma_channel_write(dev->dma, dev->buff[dev->buff_pos]);
        if (val == DMA_NODATA) {
            wd_hdc_log("WD1015: %s out of data!\n", what);
            return 0;
        }
    }

    return 1;
}

/* Fill the sector buffer from the host over the DMA channel. */
static int
wd1015_dma_in(wd1015_t *dev, const char *what)
{
    int val;

    for (; dev->buff_pos < dev->buff_cnt; dev->buff_pos++) {
        val = dma_channel_read(dev->dma);
        if (val == DMA_NODATA) {
            wd_hdc_log("WD1015: %s out of data!\n", what);
            return 0;
        }
        dev->buff[dev->buff_pos] = val & 0xff;
    }

    return 1;
}

/* ---- formatting and verification ------------------------------------ */

/* Start a Format Track on the chip for the track the cylinder and head
   registers name. The chip writes the ID and data fields itself (WD2010
   data sheet, format command) and asks the board for the per-sector ID
   bytes once the head is there. */
static int
wd1015_format_start(wd1015_t *dev, wd1015_drive_t *drive, int bad)
{
    if ((dev->cylinder >= drive->tracks) || (dev->head >= drive->hpc))
        return 0;

    dev->sector = 0;
    dev->count  = drive->spt;
    dev->bad    = bad ? 1 : 0;

    return wd1015_program(dev, 0x50);
}

/* Hand over the IDs the chip is waiting for: two bytes per sector, the Bad
   Block Mark first, then the logical sector number. */
static void
wd1015_format_feed(wd1015_t *dev)
{
    for (int s = 0; s < dev->count; s++) {
        wd1010_write(dev->wd, WD1010_REG_DATA, dev->bad ? 0x80 : 0x00);
        wd1010_write(dev->wd, WD1010_REG_DATA, (uint8_t) (s + 1));
    }
}

/* ---- command handling ---- */

static void
wd1015_cmd_test_ready(wd1015_t *dev)
{
    wd1015_drive_t *drive = &dev->drives[dev->drive_sel];

    wd_hdc_log("WD1015: TEST_READY(%i) = %i\n",
               dev->drive_sel, drive->present);
    if (!drive->present)
        wd1015_error(dev, dev->nr_err);
    wd1015_finish(dev);
}

static void
wd1015_cmd_status(wd1015_t *dev)
{
    switch (dev->phase) {
        case WD1015_PHASE_START:
            wd_hdc_log("WD1015: STATUS\n");
            dev->buff_pos = 0;
            dev->buff_cnt = 4;
            dev->buff[0]  = dev->err_bv | dev->error;
            dev->error    = 0;

            /* Give address of last operation. */
            dev->buff[1] = (dev->drive_sel ? 0x20 : 0) | dev->head;
            dev->buff[2] = ((dev->cylinder & 0x0300) >> 2) | dev->sector;
            dev->buff[3] = (dev->cylinder & 0xff);

            dev->status = WD1015_STAT_BSY | WD1015_STAT_IO | WD1015_STAT_REQ;
            dev->state  = WD1015_BUS_OUT;
            dev->phase  = WD1015_PHASE_XFER;
            break;

        case WD1015_PHASE_BLOCK_DONE:
            wd1015_finish(dev);
            break;

        default:
            break;
    }
}

/* FORMAT DRIVE / FORMAT TRACK / FORMAT BAD TRACK.  The chip formats one
   track; a whole drive walks the heads and cylinders and waits 20 ms per
   track, the timing the previous board-level implementation used. */
static void
wd1015_cmd_format(wd1015_t *dev, int whole, int bad)
{
    wd1015_drive_t *drive = &dev->drives[dev->drive_sel];
    uint8_t       st;

    switch (dev->phase) {
        case WD1015_PHASE_START:
            if (!drive->present) {
                wd1015_finish_error(dev, dev->nr_err);
                return;
            }

            (void) get_chs(dev, drive, 1);

            if (whole) {
                dev->cylinder = 0;
                dev->head     = 0;
            }

            wd_hdc_log("WD1015: FORMAT_%s(%i, %i/%i, %i sectors)\n",
                       whole ? "DRIVE" : (bad ? "BAD_TRACK" : "TRACK"),
                       dev->drive_sel, dev->cylinder, dev->head, drive->spt);
            ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 1);

            if (!wd1015_format_start(dev, drive, bad)) {
                ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_finish_error(dev, WD1015_ERR_ILLEGAL_ADDR);
                return;
            }

            dev->phase = WD1015_PHASE_XFER;
            wd1015_hold(dev);
            break;

        case WD1015_PHASE_XFER:
            st = wd1015_chip_poll(dev);

            if (st == WD1015_CHIP_DRQ) {
                wd1015_format_feed(dev);
                wd1015_hold(dev);
                break;
            }

            if (st == WD1015_CHIP_BUSY) {
                wd1015_hold(dev);
                break;
            }

            if (st == WD1015_CHIP_ERROR) {
                wd1015_chip_fail(dev);
                return;
            }

            if (!whole) {
                ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_finish(dev);
                return;
            }

            /* Next track of the drive: heads within a cylinder first. */
            if (++dev->head >= drive->hpc) {
                dev->head = 0;
                dev->cylinder++;
            }

            if (dev->cylinder >= drive->tracks) {
                ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_sync(dev);
                wd1015_finish(dev);
                return;
            }

            if (!wd1015_format_start(dev, drive, bad)) {
                ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_finish_error(dev, WD1015_ERR_ILLEGAL_ADDR);
                return;
            }

            wd1015_delay_ms(dev, 20);
            wd1015_hold(dev);
            break;

        default:
            break;
    }
}

static void
wd1015_cmd_format_drive(wd1015_t *dev)
{
    wd1015_cmd_format(dev, 1, 0);
}

/* READ VERIFY: WD1015-only command, the chip has no verify opcode, so the
   board reads the sectors through the chip and throws the data away; the
   chip's CRC/ECC check is what the command verifies. The chip advances the
   sectors on its own, the board only drains each one. */
static void
wd1015_cmd_verify(wd1015_t *dev)
{
    wd1015_drive_t *drive = &dev->drives[dev->drive_sel];
    uint8_t       st;

    switch (dev->phase) {
        case WD1015_PHASE_START:
            if (!drive->present) {
                wd1015_finish_error(dev, dev->nr_err);
                return;
            }

            if (!get_chs(dev, drive, 0)) {
                wd1015_finish_error(dev, WD1015_ERR_ILLEGAL_ADDR);
                return;
            }

            wd_hdc_log("WD1015: VERIFY(%i, %i/%i/%i, %i sectors)\n",
                       dev->drive_sel, dev->cylinder, dev->head,
                       dev->sector, dev->count ? dev->count : 256);
            ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 1);

            if (!wd1015_program(dev, 0x20)) {
                ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_finish_error(dev, WD1015_ERR_BAD_PARAMETER);
                return;
            }

            dev->phase = WD1015_PHASE_XFER;
            wd1015_hold(dev);
            break;

        case WD1015_PHASE_XFER:
            st = wd1015_chip_poll(dev);

            if (st == WD1015_CHIP_DRQ) {
                for (int i = 0; i < SECTOR_SIZE; i++)
                    (void) wd1010_read(dev->wd, WD1010_REG_DATA);
                wd1015_hold(dev);
                break;
            }

            if (st == WD1015_CHIP_BUSY) {
                wd1015_hold(dev);
                break;
            }

            if (st == WD1015_CHIP_ERROR) {
                wd1015_chip_fail(dev);
                return;
            }

            ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
            wd1015_sync(dev);
            wd1015_finish(dev);
            break;

        default:
            break;
    }
}

/* READ LONG / WRITE LONG: the data field plus the four ECC bytes the chip
   reads from (or writes to) the disk. The images carry no ECC, so the
   tail reads as zero and is discarded on writes. */
static void
wd1015_long_done(wd1015_t *dev, wd1015_drive_t *drive, int write)
{
    if (--dev->left <= 0) {
        ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
        ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
        wd1015_sync(dev);
        wd1015_finish(dev);
        return;
    }

    next_sector(dev, drive);
    dev->count = 1;

    if (!wd1015_program(dev, write ? 0x30 : 0x20)) {
        ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
        ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
        wd1015_finish_error(dev, WD1015_ERR_BAD_PARAMETER);
        return;
    }

    dev->long_tail = 0;
    dev->phase     = WD1015_PHASE_XFER;
    wd1015_hold(dev);
}

/* Hand the four ECC bytes over, in the direction the command moves data. */
static void
wd1015_long_tail(wd1015_t *dev, int write)
{
    if (!write) {
        for (int i = 0; i < 4; i++)
            dev->buff[i] = 0;
    }

    dev->long_tail = 1;
    dev->xfer_reg  = 0;
    dev->buff_pos  = 0;
    dev->buff_cnt  = 4;
    dev->status    = WD1015_STAT_BSY | WD1015_STAT_REQ;
    if (!write)
        dev->status |= WD1015_STAT_IO;
    dev->state = write ? WD1015_BUS_IN : WD1015_BUS_OUT;
}

static void
wd1015_cmd_long(wd1015_t *dev, int write)
{
    wd1015_drive_t *drive = &dev->drives[dev->drive_sel];
    uint8_t       st;
    int           val;

    switch (dev->phase) {
        case WD1015_PHASE_START:
            if (!drive->present) {
                wd1015_finish_error(dev, dev->nr_err);
                return;
            }

            if (!get_chs(dev, drive, 0)) {
                wd1015_finish_error(dev, WD1015_ERR_ILLEGAL_ADDR);
                return;
            }

            dev->left      = dev->count ? dev->count : 256;
            dev->count     = 1;
            dev->long_tail = 0;

            wd_hdc_log("WD1015: %s_LONG(%i, %i/%i/%i, %i sectors)\n",
                       write ? "WRITE" : "READ", dev->drive_sel, dev->cylinder,
                       dev->head, dev->sector, dev->left);

            if (write)
                ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 1);
            else
                ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 1);

            if (!wd1015_program(dev, write ? 0x30 : 0x20)) {
                ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_finish_error(dev, WD1015_ERR_BAD_PARAMETER);
                return;
            }

            dev->phase = WD1015_PHASE_XFER;
            wd1015_hold(dev);
            break;

        case WD1015_PHASE_XFER:
            st = wd1015_chip_poll(dev);

            if (st == WD1015_CHIP_BUSY) {
                wd1015_hold(dev);
                break;
            }

            if (st == WD1015_CHIP_ERROR) {
                wd1015_chip_fail(dev);
                return;
            }

            if (st != WD1015_CHIP_DRQ) {
                wd1015_hold(dev);
                break;
            }

            if (!write) {
                /* The chip has the sector in its buffer; draining the data
                   register releases DRQ and leaves the bytes where the host
                   transfer below picks them up. */
                for (int i = 0; i < SECTOR_SIZE; i++)
                    (void) wd1010_read(dev->wd, WD1010_REG_DATA);
            }

            if (dev->irq_dma & WD1015_DMA_ENA) {
                dma_set_drq(dev->dma, 1);

                for (int i = 0; i < SECTOR_SIZE; i++) {
                    val = write ? dma_channel_read(dev->dma)
                                : dma_channel_write(dev->dma, dev->buff[i]);
                    if (val == DMA_NODATA) {
                        dma_set_drq(dev->dma, 0);
                        ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                        ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                        wd1015_finish_error(dev, WD1015_ERR_NO_RECOVERY);
                        return;
                    }
                    if (write)
                        dev->buff[i] = (uint8_t) val;
                }

                for (int i = 0; i < 4; i++) {
                    val = write ? dma_channel_read(dev->dma)
                                : dma_channel_write(dev->dma, 0);
                    if (val == DMA_NODATA) {
                        dma_set_drq(dev->dma, 0);
                        ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                        ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
                        wd1015_finish_error(dev, WD1015_ERR_NO_RECOVERY);
                        return;
                    }
                }

                dma_set_drq(dev->dma, 0);

                if (write) {
                    for (int i = 0; i < SECTOR_SIZE; i++)
                        wd1010_write(dev->wd, WD1010_REG_DATA, dev->buff[i]);

                    if (wd1015_chip_poll(dev) == WD1015_CHIP_ERROR) {
                        wd1015_chip_fail(dev);
                        return;
                    }
                }

                wd1015_long_done(dev, drive, write);
                break;
            }

            dev->xfer_reg = 0;
            dev->buff_pos = 0;
            dev->buff_cnt = SECTOR_SIZE;
            dev->status   = WD1015_STAT_BSY | WD1015_STAT_REQ;
            if (!write)
                dev->status |= WD1015_STAT_IO;
            dev->state = write ? WD1015_BUS_IN : WD1015_BUS_OUT;
            break;

        case WD1015_PHASE_BLOCK_DONE:
            if (dev->long_tail) {
                dev->long_tail = 0;
                wd1015_long_done(dev, drive, write);
                break;
            }

            if (write) {
                /* The host has delivered the sector; hand it to the chip
                   before taking the ECC tail, which is not stored. */
                for (int i = 0; i < SECTOR_SIZE; i++)
                    wd1010_write(dev->wd, WD1010_REG_DATA, dev->buff[i]);

                if (wd1015_chip_poll(dev) == WD1015_CHIP_ERROR) {
                    wd1015_chip_fail(dev);
                    return;
                }
            }

            wd1015_long_tail(dev, write);
            break;

        default:
            break;
    }
}

static void
wd1015_cmd_read_long(wd1015_t *dev)
{
    wd1015_cmd_long(dev, 0);
}

static void
wd1015_cmd_write_long(wd1015_t *dev)
{
    wd1015_cmd_long(dev, 1);
}

static void
wd1015_cmd_format_track(wd1015_t *dev)
{
    wd1015_cmd_format(dev, 0, dev->command[0] == WD1015_CMD_FORMAT_BAD_TRACK);
}

static void
wd1015_cmd_specify(wd1015_t *dev)
{
    wd1015_drive_t *drive = &dev->drives[dev->drive_sel];

    switch (dev->phase) {
        case WD1015_PHASE_START:
            dev->buff_pos = 0;
            dev->buff_cnt = 8;
            dev->status   = WD1015_STAT_BSY | WD1015_STAT_REQ;
            dev->state    = WD1015_BUS_IN;
            dev->phase    = WD1015_PHASE_XFER;
            break;

        case WD1015_PHASE_BLOCK_DONE:
            drive->cfg_cyl = dev->buff[1] | (dev->buff[0] << 8);
            drive->cfg_hpc = dev->buff[2];
            /* For a 615/4/26 we get 666/2/31 geometry. */
            wd_hdc_log("WD1015: drive%i: cyls=%i, heads=%i\n",
                       dev->drive_sel, drive->cfg_cyl, drive->cfg_hpc);
            wd1015_finish(dev);
            break;

        default:
            break;
    }
}

static void
wd1015_cmd_read_ecc_burst(wd1015_t *dev)
{
    switch (dev->phase) {
        case WD1015_PHASE_START:
            wd_hdc_log("WD1015: READ_ECC_BURST_LEN\n");
            dev->buff_pos = 0;
            dev->buff_cnt = 1;
            dev->buff[0]  = 0; /* 0 bits */
            dev->status   = WD1015_STAT_BSY | WD1015_STAT_IO | WD1015_STAT_REQ;
            dev->state    = WD1015_BUS_OUT;
            dev->phase    = WD1015_PHASE_XFER;
            break;

        case WD1015_PHASE_BLOCK_DONE:
            wd1015_finish(dev);
            break;

        default:
            break;
    }
}

/* READ BUFFER / WRITE BUFFER: move one full sector between the host and the
   board's sector buffer. */
static void
wd1015_cmd_buffer(wd1015_t *dev, int write)
{
    switch (dev->phase) {
        case WD1015_PHASE_START:
            dev->buff_pos = 0;
            dev->buff_cnt = SECTOR_SIZE;
            wd_hdc_log("WD1015: %s_BUFFER (%i)\n", write ? "WRITE" : "READ",
                       dev->buff_cnt);

            dev->status = WD1015_STAT_BSY | WD1015_STAT_REQ;
            if (!write)
                dev->status |= WD1015_STAT_IO;
            if (dev->irq_dma & WD1015_DMA_ENA) {
                /* The DMA runs from a timer tick, so the host state has to
                   stay in poll mode for the callback guard to let it run. */
                wd1015_delay(dev);
                dma_set_drq(dev->dma, 1);
            } else {
                dev->state = write ? WD1015_BUS_IN : WD1015_BUS_OUT;
            }
            dev->phase = WD1015_PHASE_XFER;
            break;

        case WD1015_PHASE_XFER:
            if (write ? !wd1015_dma_in(dev, "WD1015_CMD_WRITE_BUFFER")
                      : !wd1015_dma_out(dev, "WD1015_CMD_READ_BUFFER")) {
                wd1015_finish_error(dev, WD1015_ERR_NO_RECOVERY);
                return;
            }

            dma_set_drq(dev->dma, 0);
            wd1015_delay(dev);
            dev->state = WD1015_BUS_POLL;
            dev->phase = WD1015_PHASE_BLOCK_DONE;
            break;

        case WD1015_PHASE_BLOCK_DONE:
            wd1015_finish(dev);
            break;

        default:
            break;
    }
}

static void
wd1015_cmd_read_buffer(wd1015_t *dev)
{
    wd1015_cmd_buffer(dev, 0);
}

static void
wd1015_cmd_write_buffer(wd1015_t *dev)
{
    wd1015_cmd_buffer(dev, 1);
}

/* EXECUTE DRIVE DIAGNOSTIC: reads sector one of every track and throws
   the data away, so the chip's ID and CRC/ECC check are what the command
   verifies. Track zero is read first, which seeks the head home the same
   way an explicit Restore does. No write is issued (WD1002S-WX2 5.19). */
static void
wd1015_cmd_drive_diag(wd1015_t *dev)
{
    wd1015_drive_t *drive = &dev->drives[dev->drive_sel];
    uint8_t       st;

    switch (dev->phase) {
        case WD1015_PHASE_START:
            if (!drive->present) {
                wd1015_finish_error(dev, WD1015_ERR_NOT_READY);
                return;
            }

            dev->cylinder = 0;
            dev->head     = 0;
            dev->sector   = 0;
            dev->count    = 1;
            dev->left     = drive->cfg_cyl ? drive->cfg_cyl : 1;

            wd_hdc_log("WD1015: DRIVE_DIAG(%i, %i tracks)\n", dev->drive_sel,
                       dev->left);
            ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 1);

            if (!wd1015_program(dev, 0x20)) {
                ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_finish_error(dev, WD1015_ERR_BAD_PARAMETER);
                return;
            }

            dev->phase = WD1015_PHASE_XFER;
            wd1015_hold(dev);
            break;

        case WD1015_PHASE_XFER:
            st = wd1015_chip_poll(dev);

            if (st == WD1015_CHIP_DRQ) {
                for (int i = 0; i < SECTOR_SIZE; i++)
                    (void) wd1010_read(dev->wd, WD1010_REG_DATA);
                wd1015_hold(dev);
                break;
            }

            if (st == WD1015_CHIP_BUSY) {
                wd1015_hold(dev);
                break;
            }

            if (st == WD1015_CHIP_ERROR) {
                wd1015_chip_fail(dev);
                return;
            }

            if (--dev->left <= 0) {
                ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_sync(dev);
                wd1015_finish(dev);
                return;
            }

            dev->cylinder++;
            dev->head   = 0;
            dev->sector = 0;
            dev->count  = 1;

            if (!wd1015_program(dev, 0x20)) {
                ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
                wd1015_finish_error(dev, WD1015_ERR_ILLEGAL_ADDR);
                return;
            }

            wd1015_hold(dev);
            break;

        default:
            break;
    }
}

static void
wd1015_cmd_diag(wd1015_t *dev)
{
    wd_hdc_log("WD1015: DIAG\n");
    wd1015_finish(dev);
}

typedef struct wd1015_op_t {
    uint8_t opcode;
    void  (*run)(wd1015_t *dev);
} wd1015_op_t;

static void wd1015_chip_callback(wd1015_t *dev);

static const wd1015_op_t wd1015_ops[] = {
    /* The four commands the board hands straight to the chip (WD1003-WA2
       5.2: Restore, Seek, Read, Write). Format Track also runs on the
       chip, but the board's format loop issues it per track. */
    { WD1015_CMD_RECALIBRATE,        wd1015_chip_callback      },
    { WD1015_CMD_READ,               wd1015_chip_callback      },
    { WD1015_CMD_WRITE,              wd1015_chip_callback      },
    { WD1015_CMD_SEEK,               wd1015_chip_callback      },
    { WD1015_CMD_TEST_DRIVE_READY,   wd1015_cmd_test_ready     },
    { WD1015_CMD_STATUS,             wd1015_cmd_status         },
    { WD1015_CMD_FORMAT_DRIVE,       wd1015_cmd_format_drive   },
    { WD1015_CMD_VERIFY,             wd1015_cmd_verify         },
    { WD1015_CMD_FORMAT_TRACK,       wd1015_cmd_format_track   },
    { WD1015_CMD_FORMAT_BAD_TRACK,   wd1015_cmd_format_track   },
    { WD1015_CMD_SPECIFY,            wd1015_cmd_specify        },
    { WD1015_CMD_READ_ECC_BURST_LEN, wd1015_cmd_read_ecc_burst },
    { WD1015_CMD_READ_BUFFER,        wd1015_cmd_read_buffer    },
    { WD1015_CMD_WRITE_BUFFER,       wd1015_cmd_write_buffer   },
    { WD1015_CMD_RAM_DIAGNOSTIC,     wd1015_cmd_diag           },
    { WD1015_CMD_DRIVE_DIAGNOSTIC,   wd1015_cmd_drive_diag     },
    { WD1015_CMD_CTRLR_DIAGNOSTIC,   wd1015_cmd_diag           },
    { WD1015_CMD_READ_LONG,          wd1015_cmd_read_long      },
    { WD1015_CMD_WRITE_LONG,         wd1015_cmd_write_long     }
};

static void
wd1015_callback(void *priv)
{
    wd1015_t *dev = (wd1015_t *) priv;

    /* A command step can arm a timer that outlives the command itself, so a
       tick may arrive while the host is still sending its command block or
       has already read the completion byte. Only act while the host is
       actually waiting on the controller. */
    if (dev->state != WD1015_BUS_POLL)
        return;

    /* Get the drive info. Note that the API supports up to 8 drives! */
    dev->drive_sel = (dev->command[1] >> 5) & 0x07;

    /* Preset the completion byte to "No error" and the selected drive. */
    dev->compl = (dev->drive_sel << 5) | WD1015_ERR_NONE;

    if (dev->command[0] != WD1015_CMD_STATUS)
        dev->err_bv = 0x00;

    for (size_t i = 0; i < sizeof(wd1015_ops) / sizeof(wd1015_ops[0]); i++) {
        if (wd1015_ops[i].opcode != dev->command[0])
            continue;

        wd1015_ops[i].run(dev);
        return;
    }

    wd_hdc_log("WD1015: unknown command:\n");
    wd_hdc_log("WD1015: %02x %02x %02x %02x %02x %02x\n",
               dev->command[0], dev->command[1], dev->command[2],
               dev->command[3], dev->command[4], dev->command[5]);
    wd1015_finish_error(dev, WD1015_ERR_BAD_COMMAND);
}

/* =====================================================================
 * Layer 3: WD11C00 PC/XT host interface.
 *
 * The WD11C00 decodes the four contiguous PC/XT ports and implements the
 * host side of the data and completion handshake, the status and DMA/IRQ
 * registers, the board select and the software reset. The sector buffer
 * address counter is the shared 512-byte buffer and the ECC engine is not
 * emulated.
 *
 * On the Tandy 2000HD the same host-side role is taken by the WD1100-11
 * gate array, with the task file at 270h-27Fh and a one-sector buffer.
 *
 * The option ROM window and the SW1 drive-type jumpers are decoded here
 * as well.
 * ===================================================================== */

/* ---- option ROM and SW1 drive-type jumpers ------------------------- */

/*
 * Set up and load the ROM BIOS for this controller.
 *
 * The option ROM is mapped as a window over the configured address; the
 * WD11C00 / WD1100-11 decodes ROMEN for it.
 */
static void
wd1100_ibmxt_loadrom(wd1015_t *dev, const char *fn)
{
    if (fn == NULL) {
        wd_hdc_log("WD1100: NULL BIOS ROM file pointer!\n");
        return;
    }

    /* "Disabled" leaves the option ROM unmapped. */
    if (dev->bios_addr == 0x00000)
        return;

    if (rom_init(&dev->bios_rom, fn, dev->bios_addr, 0x2000, 0x1fff, 0,
                 MEM_MAPPING_EXTERNAL) < 0)
        wd_hdc_log("WD1100: BIOS ROM '%s' not found!\n", fn);
}

/* Apply the SW1 drive-type straps and the IRQ2 jumper position. */
static void
wd1100_ibmxt_switch_defaults(wd1015_t *dev)
{
    switch (dev->type) {
        case ST506_XT_TYPE_WD1002A_WX1:
        case ST506_XT_TYPE_WD1004A_WX1:
            /* The switches are read in reverse: 0 = closed, 1 = open.
               Both open means MFM, 17 sectors per track. */
            dev->switches = 0x30; /* autobios */
            break;

        case ST506_XT_TYPE_WD1002A_27X:
        case ST506_XT_TYPE_WD1004_27X:
        case ST506_XT_TYPE_WD1004A_27X:
            /* The switches are read in reverse: 0 = closed, 1 = open.
               Both closed means translate 26 sectors per track to 17,
               SW6 closed, SW5 open means 26 sectors per track. */
            dev->switches = device_get_config_int("translate") ? 0x00 : 0x10;
            break;

        default:
            break;
    }

    if (dev->irq == 2)
        dev->switches |= 0x80;
}

/* Set the drive type switches from the geometry table. */
static void
wd1100_ibmxt_set_switches(wd1015_t *dev, wd1100_olivetti_t *hdt, int num)
{
    const wd1015_drive_t *drive;
    int            c;
    int            e;

    dev->switches = 0x00;

    for (uint8_t d = 0; d < MFM_NUM; d++) {
        drive = &dev->drives[d];

        if (!drive->present) {
            if (dev->type == ST506_XT_TYPE_WD1002A_WX1_NOBIOS)
                dev->switches |= (0x33 << (d ? 0 : 2));
            continue;
        }

        for (c = 0; c < num; c++) {
            /* Match the drive geometry against the type table. */
            if ((drive->spt == hdt[c].spt) && (drive->hpc == hdt[c].hpc) && (drive->tracks == hdt[c].tracks)) {
                /* Olivetti M24/M240: Move the upper 2 bites up by 2 bits, as the
                   layout is as follows: D0_3 D0_2 D1_3 D1_2 D0_1 D0_0 D1_1 D1_0. */
                if (dev->type == ST506_XT_TYPE_WD1002A_WX1_NOBIOS)
                    e = (c & 0x03) | ((c >> 2) << 4);
                else
                    e = c;
                dev->switches |= (e << (d ? 0 : 2));
                break;
            }
        }

        wd_hdc_log("WD1100: ");
        if (c == num)
            wd_hdc_log("*WARNING* drive%i unsupported", d);
        else
            wd_hdc_log("drive%i is type %i", d, c);
        wd_hdc_log(" (%i/%i/%i)\n", drive->tracks, drive->hpc, drive->spt);
    }
}

/* Read from one of the registers. */
static uint8_t
wd1100_ibmxt_read(uint16_t port, void *priv)
{
    wd1015_t  *dev = (wd1015_t *) priv;
    uint8_t ret = 0xff;

    switch (port & 3) {
        case 0: /* read data */
            dev->status &= ~WD1015_STAT_IRQ;
            switch (dev->state) {
                case WD1015_BUS_COMPLETION:
                    ret         = dev->compl;
                    dev->status = 0x00;
                    dev->state  = WD1015_BUS_IDLE;
                    dev->phase  = WD1015_PHASE_START;
                    break;

                case WD1015_BUS_OUT:
                    ret = dev->xfer_reg ? wd1010_read(dev->wd, WD1010_REG_DATA)
                                         : dev->buff[dev->buff_pos];
                    dev->buff_pos++;
                    if (dev->buff_pos == dev->buff_cnt) {
                        dev->buff_pos = 0;
                        dev->buff_cnt = 0;
                        dev->status   = WD1015_STAT_BSY;
                        dev->state    = WD1015_BUS_POLL;
                        if (!dev->xfer_reg)
                            dev->phase = WD1015_PHASE_BLOCK_DONE;
                        wd1015_hold(dev);
                    }
                    break;

                default:
                    break;
            }
            break;

        case 1: /* read status */
            ret = dev->status;
            if ((dev->irq_dma & WD1015_DMA_ENA) && dma_get_drq(dev->dma))
                ret |= WD1015_STAT_DRQ;
            break;

        case 2: /* read option jumpers */
            ret = dev->switches;
            break;

        default:
            break;
    }
    wd_hdc_log("WD1100: read(%04x) = %02x\n", port, ret);

    return ret;
}

/* Write to one of the registers. */
static void
wd1100_ibmxt_write(uint16_t port, uint8_t val, void *priv)
{
    wd1015_t *dev = (wd1015_t *) priv;

    wd_hdc_log("WD1100: write(%04x, %02x)\n", port, val);
    switch (port & 3) {
        case 0: /* write data */
            switch (dev->state) {
                case WD1015_BUS_COMMAND: /* command data */
                    /* Write directly to the command buffer to avoid overwriting
                       the data buffer. */
                    dev->command[dev->buff_pos++] = val;
                    if (dev->buff_pos == dev->buff_cnt) {
                        /* We have a new command. */
                        dev->buff_pos = 0;
                        dev->buff_cnt = 0;
                        dev->status   = WD1015_STAT_BSY;
                        dev->state    = WD1015_BUS_POLL;
                        dev->phase    = WD1015_PHASE_START;
                        wd1015_hold(dev);
                    }
                    break;

                case WD1015_BUS_IN: /* data */
                    if (dev->xfer_reg)
                        wd1010_write(dev->wd, WD1010_REG_DATA, val);
                    else
                        dev->buff[dev->buff_pos] = val;
                    dev->buff_pos++;
                    if (dev->buff_pos == dev->buff_cnt) {
                        dev->buff_pos = 0;
                        dev->buff_cnt = 0;
                        dev->status   = WD1015_STAT_BSY;
                        dev->state    = WD1015_BUS_POLL;
                        if (!dev->xfer_reg)
                            dev->phase = WD1015_PHASE_BLOCK_DONE;
                        wd1015_hold(dev);
                    }
                    break;

                default:
                    break;
            }
            break;

        case 1: /* controller reset */
            dev->status    = 0x00;
            dev->xfer_reg = 0;
            if (dev->wd != NULL)
                wd1010_reset(dev->wd);
            break;

        case 2: /* generate controller-select-pulse */
            dev->status    = WD1015_STAT_BSY | WD1015_STAT_CD | WD1015_STAT_REQ;
            dev->buff_pos  = 0;
            dev->buff_cnt  = sizeof(dev->command);
            dev->state     = WD1015_BUS_COMMAND;
            dev->xfer_reg  = 0;
            break;

        case 3: /* DMA/IRQ enable register */
            dev->irq_dma = val;

            if (!(dev->irq_dma & WD1015_DMA_ENA))
                dma_set_drq(dev->dma, 0);

            if (!(dev->irq_dma & WD1015_IRQ_ENA)) {
                dev->status &= ~WD1015_STAT_IRQ;
                picintc(1 << dev->irq);
            }
            break;

        default:
            break;
    }
}

/* Register the four PC/XT ports with the bus. */
static void
wd1100_ibmxt_io_init(wd1015_t *dev)
{
    io_sethandler(dev->base, 4,
                  wd1100_ibmxt_read, NULL, NULL, wd1100_ibmxt_write, NULL, NULL, dev);
}

/* ---- card assembly: drive images, lifecycle and device definitions --- */

static void
wd1100_ibmxt_loadhd(wd1015_t *dev, int c, int d)
{
    wd1015_drive_t *drive = &dev->drives[c];

    if (!hdd_image_load(d)) {
        drive->present = 0;
        return;
    }

    /* Make sure we can do this. */
    /* Allow 31 sectors per track on RLL controllers, for the
       ST225R, which is 667/2/31. */
    if ((hdd[d].spt != dev->spt) && (hdd[d].spt != 31) && (dev->spt != 26)) {
        /*
         * Uh-oh, MFM/RLL mismatch.
         *
         * Although this would be no issue in the code itself,
         * most of the BIOSes were hardwired to whatever their
         * native SPT setting was, so, do not allow this here.
         */
        wd_hdc_log("WD1100: drive%i: MFM/RLL mismatch (%i/%i)\n",
                     c, hdd[d].spt, dev->spt);
        hdd_image_close(d);
        drive->present = 0;
        return;
    }

    drive->spt    = (uint8_t) hdd[d].spt;
    drive->hpc    = (uint8_t) hdd[d].hpc;
    drive->tracks = (uint16_t) hdd[d].tracks;

    drive->hdd_num = d;
    drive->present = 1;
}

/* ---- lifecycle ---- */

/* The WD1002/WD1004 boards carry a WD1010 whose transfers are handled by
   the chip model above. The RLL variants are included because the 26 -> 17
   translation happens in their option ROM: the chip only ever sees physical
   CHS. */
static void *
wd1100_ibmxt_init(const device_t *info)
{
    const char *fn = NULL;
    wd1015_t      *dev;
    int         i;
    int         c;

    dev = (wd1015_t *) calloc(1, sizeof(wd1015_t));
    dev->type = info->local & 255;

    /* Set defaults for the controller. */
    dev->spt       = MFM_SECTORS;
    dev->base      = 0x0320;
    dev->irq       = 5;
    dev->dma       = 3;
    dev->bios_addr = 0xc8000;
    dev->nr_err    = WD1015_ERR_NOT_READY;

    switch (dev->type) {
        case ST506_XT_TYPE_WDXT_GEN: /* WDXT-GEN (MFM) */
            fn = WDXT_GEN_BIOS_FILE;
            break;

        case ST506_XT_TYPE_WD1002A_WX1: /* Western Digital WD1002A-WX1 (MFM) */
        case ST506_XT_TYPE_WD1004A_WX1: /* Western Digital WD1004A-WX1 (MFM) */
            fn = (dev->type == ST506_XT_TYPE_WD1004A_WX1) ? WD1004A_BIOS_FILE
                                                      : WD1002A_WX1_BIOS_FILE;
            break;

        case ST506_XT_TYPE_WD1002A_WX1_NOBIOS: /* Western Digital WD1002A-WX1 (MFM, No BIOS) */
            /* Supported base addresses: 320h, 324h, 328h, 32Ch. */
            fn = NULL;
            break;

        case ST506_XT_TYPE_WD1002A_27X: /* Western Digital WD1002A-27X (RLL) */
        case ST506_XT_TYPE_WD1004_27X:  /* Western Digital WD1004-27X (RLL) */
        case ST506_XT_TYPE_WD1004A_27X: /* Western Digital WD1004A-27X (RLL) */
            fn = (dev->type == ST506_XT_TYPE_WD1002A_27X) ? WD1002A_27X_BIOS_FILE
                                                      : WD1004A_BIOS_FILE;
            dev->spt      = RLL_SECTORS;
            break;

        default:
            break;
    }

    /* The boards with user-settable port, IRQ and ROM address. */
    switch (dev->type) {
        case ST506_XT_TYPE_WD1002A_WX1:
        case ST506_XT_TYPE_WD1004A_WX1:
        case ST506_XT_TYPE_WD1002A_27X:
        case ST506_XT_TYPE_WD1004_27X:
        case ST506_XT_TYPE_WD1004A_27X:
            dev->base      = device_get_config_hex16("base");
            dev->irq       = device_get_config_int("irq");
            dev->bios_addr = device_get_config_hex20("bios_addr");
            break;

        case ST506_XT_TYPE_WD1002A_WX1_NOBIOS:
            /* W4 and W7 jumpers: no ROM, so no BIOS address. */
            dev->base = device_get_config_hex16("base");
            dev->irq  = device_get_config_int("irq");
            break;

        default:
            break;
    }

    /* SW1 straps are part of the WD1100 host interface. */
    wd1100_ibmxt_switch_defaults(dev);

    /* Load the ROM BIOS. */
    wd1100_ibmxt_loadrom(dev, fn);

    /* Set up the I/O region. */
    wd1100_ibmxt_io_init(dev);

    /* Add the timer. */
    timer_add(&dev->timer, wd1015_callback, dev, 0);

    wd_hdc_log("WD1100: %s (I/O=%03X, IRQ=%i, DMA=%i, BIOS @0x%06lX, size %i)\n",
                 info->name, dev->base, dev->irq, dev->dma, dev->bios_addr, dev->bios_rom.sz);

    /* Load any drives configured for us. */
    wd_hdc_log("WD1100: looking for disks...\n");
    for (c = 0, i = 0; i < HDD_NUM; i++) {
        if ((hdd[i].bus_type == HDD_BUS_MFM) && (hdd[i].mfm_channel < MFM_NUM)) {
            wd_hdc_log("WD1100: disk '%s' on channel %i\n",
                         hdd[i].fn, hdd[i].mfm_channel);
            wd1100_ibmxt_loadhd(dev, hdd[i].mfm_channel, i);

            if (++c > MFM_NUM)
                break;
        }
    }
    wd_hdc_log("WD1100: %i disks loaded.\n", c);

    if (dev->type == ST506_XT_TYPE_WD1002A_WX1_NOBIOS)
        wd1100_ibmxt_set_switches(dev, (wd1100_olivetti_t *) hd_types_olivetti, 16);

    /* Initial "active" drive parameters. */
    for (c = 0; c < MFM_NUM; c++) {
        dev->drives[c].cfg_cyl = dev->drives[c].tracks;
        dev->drives[c].cfg_hpc = dev->drives[c].hpc;
        dev->drives[c].cfg_spt = dev->drives[c].spt;
    }

    /* All boards here carry a Winchester controller chip. The WD1002A-WX1
       boards use the WD1010A; the RLL boards use the WD5011, which shares
       the WD2010 task file (32-bit ECC). */
    {
        wd1010_config_t wcfg;
        int           ok = 1;

        memset(&wcfg, 0, sizeof(wcfg));

        switch (dev->type) {
            case ST506_XT_TYPE_WD1002A_27X:
            case ST506_XT_TYPE_WD1004_27X:
            case ST506_XT_TYPE_WD1004A_27X:
                wcfg.variant = 2; /* WD5011 (RLL): WD2010-compatible task file */
                break;

            default:
                wcfg.variant = 0; /* WD1010A */
                break;
        }

        wcfg.intrq = NULL;
        wcfg.priv  = NULL;

        for (int d = 0; d < MFM_NUM; d++) {
            wcfg.unit[d].hdd_num = -1;

            if (!dev->drives[d].present)
                continue;

            if (dev->drives[d].hpc > 16) {
                wd_hdc_log("WD1100: drive%i has too many heads for HS3 (%i)\n",
                             d, dev->drives[d].hpc);
                ok = 0;
            }

            wcfg.unit[d].hdd_num = dev->drives[d].hdd_num;
            wcfg.unit[d].spt     = dev->drives[d].spt;
            wcfg.unit[d].hpc     = dev->drives[d].hpc;
            wcfg.unit[d].tracks  = dev->drives[d].tracks;
        }

        if (ok) {
            dev->wd   = wd1010_create(&wcfg);
            dev->buff = wd1010_buffer(dev->wd);
        }
    }

    return dev;
}

static void
wd1100_ibmxt_close(void *priv)
{
    wd1015_t         *dev = (wd1015_t *) priv;
    const wd1015_drive_t *drive;

    if (dev->wd != NULL)
        wd1010_destroy(dev->wd);

    for (uint8_t d = 0; d < MFM_NUM; d++) {
        drive = &dev->drives[d];

        hdd_image_close(drive->hdd_num);
    }

    if (dev->bios_rom.rom != NULL) {
        free(dev->bios_rom.rom);
        dev->bios_rom.rom = NULL;
    }

    free(dev);
}

/* ---- Tandy 2000HD: WD1100-11 host interface ------------------------- */

/* On Tandy 2000HD the task file sits at 270h - 27Fh on even addresses:
   DATA 270h, ERROR 272h (write = precomp), COUNT 274h, SECTOR 276h,
   CYL_LO 278h, CYL_HI 27ah, SDH 27ch, STATUS 27eh (read = status,
   write = command). */
static int
wd1100_tandy_reg(uint16_t addr)
{
    return (addr - 0x0270) >> 1;
}

static uint8_t
wd1100_tandy_read(uint16_t addr, void *priv)
{
    wd1100_tandy_t *hdc = (wd1100_tandy_t *) priv;

    return wd1010_read(hdc->wd, wd1100_tandy_reg(addr));
}

static void
wd1100_tandy_write(uint16_t addr, uint8_t val, void *priv)
{
    wd1100_tandy_t *hdc = (wd1100_tandy_t *) priv;

    wd1010_write(hdc->wd, wd1100_tandy_reg(addr), val);
}

/* The board's reset port answers reads and takes writes; either one resets
   the controller, which is how the BIOS probes for the card. */
static uint8_t
wd1100_tandy_reset_r(UNUSED(uint16_t addr), void *priv)
{
    wd1100_tandy_t *hdc = (wd1100_tandy_t *) priv;

    wd1010_reset(hdc->wd);

    return 0xff;
}

static void
wd1100_tandy_reset_w(UNUSED(uint16_t addr), UNUSED(uint8_t val), void *priv)
{
    wd1100_tandy_t *hdc = (wd1100_tandy_t *) priv;

    wd1010_reset(hdc->wd);
}

static void
wd1100_tandy_reset(void *priv)
{
    wd1100_tandy_t *hdc = (wd1100_tandy_t *) priv;

    wd1010_hdc_led_off();

    wd1010_reset(hdc->wd);
}

/* Register the Tandy task file and reset ports with the bus. */
static void
wd1100_tandy_io_init(wd1100_tandy_t *hdc)
{
    io_sethandler(0x026c, 0x0002, wd1100_tandy_reset_r, NULL, NULL,
                  wd1100_tandy_reset_w, NULL, NULL, hdc);
    io_sethandler(0x0270, 0x0010, wd1100_tandy_read, NULL, NULL,
                  wd1100_tandy_write, NULL, NULL, hdc);
}

static void *
wd1100_tandy_init(const device_t *info)
{
    wd1100_tandy_t   *hdc = (wd1100_tandy_t *) calloc(1, sizeof(wd1100_tandy_t));
    wd1010_config_t cfg;

    memset(&cfg, 0, sizeof(cfg));
    cfg.variant = 0;
    cfg.intrq   = NULL;
    cfg.priv    = NULL;

    for (uint8_t i = 0; i < MFM_NUM; i++)
        cfg.unit[i].hdd_num = -1;

    for (uint8_t i = 0; i < HDD_NUM; i++) {
        if ((hdd[i].bus_type != HDD_BUS_MFM) || (hdd[i].mfm_channel >= MFM_NUM))
            continue;

        if (!hdd_image_load(i))
            continue;

        hdc->present[hdd[i].mfm_channel] = 1;
        hdc->hdd_num[hdd[i].mfm_channel] = i;

        cfg.unit[hdd[i].mfm_channel].hdd_num = i;
        cfg.unit[hdd[i].mfm_channel].spt     = hdd[i].spt;
        cfg.unit[hdd[i].mfm_channel].hpc     = hdd[i].hpc;
        cfg.unit[hdd[i].mfm_channel].tracks  = hdd[i].tracks;
    }

    /* The only board using this file is the Tandy 2000HD, which carries the
       WD1010 and leaves INTRQ unconnected. */
    hdc->wd = wd1010_create(&cfg);

    wd1100_tandy_io_init(hdc);

    return hdc;
}

static void
wd1100_tandy_close(void *priv)
{
    wd1100_tandy_t *hdc = (wd1100_tandy_t *) priv;

    wd1010_destroy(hdc->wd);

    for (uint8_t i = 0; i < MFM_NUM; i++) {
        if (hdc->present[i])
            hdd_image_close(hdc->hdd_num[i]);
    }

    free(hdc);
}

const device_t st506_wd1010_device = {
    .name          = "WD1010 Hard Disk Controller",
    .internal_name = "st506_wd1010",
    .flags         = 0,
    .local         = 0,
    .init          = wd1100_tandy_init,
    .close         = wd1100_tandy_close,
    .reset         = wd1100_tandy_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

/* ---- device availability ---- */

static int
wdxt_available(void)
{
    return (rom_present(WDXT_GEN_BIOS_FILE));
}

static int
wd1002a_wx1_available(void)
{
    return (rom_present(WD1002A_WX1_BIOS_FILE));
}

static int
wd1002a_27x_available(void)
{
    return (rom_present(WD1002A_27X_BIOS_FILE));
}

static int
wd1004a_available(void)
{
    return (rom_present(WD1004A_BIOS_FILE));
}

/* ---- device configuration ---- */

static const device_config_t wd_config[] = {
    {
        .name           = "bios_addr",
        .description    = "BIOS address",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xc8000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0x00000 },
            { .description = "C800H",    .value = 0xc8000 },
            { .description = ""                           }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0320,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "320H", .value = 0x0320 },
            { .description = "324H", .value = 0x0324 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 5,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 5", .value = 5 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

static const device_config_t wd_nobios_config[] = {
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0320,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "320H", .value = 0x0320 },
            { .description = "324H", .value = 0x0324 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 5,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 5", .value = 5 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

static const device_config_t wd_rll_config[] = {
    {
        .name           = "bios_addr",
        .description    = "BIOS address",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xc8000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0x00000 },
            { .description = "C800H",    .value = 0xc8000 },
            { .description = ""                           }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0320,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "320H", .value = 0x0320 },
            { .description = "324H", .value = 0x0324 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 5,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 5", .value = 5 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "translate",
        .description    = "Translate 26 -> 17",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Off", .value = 0 },
            { .description = "On",  .value = 1 },
            { .description = ""                }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

static const device_config_t wd1004_rll_config[] = {
    {
        .name           = "bios_addr",
        .description    = "BIOS address",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0xc8000,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0x00000 },
            { .description = "C800H",    .value = 0xc8000 },
            { .description = "CA00H",    .value = 0xca000 },
            { .description = "CC00H",    .value = 0xcc000 },
            { .description = "CE00H",    .value = 0xce000 },
            { .description = ""                           }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x0320,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "320H", .value = 0x0320 },
            { .description = "324H", .value = 0x0324 },
            { .description = "328H", .value = 0x0328 },
            { .description = "32CH", .value = 0x032c },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 5,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 2", .value = 2 },
            { .description = "IRQ 5", .value = 5 },
            { .description = ""                  }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "translate",
        .description    = "Translate 26 -> 17",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Off", .value = 0 },
            { .description = "On",  .value = 1 },
            { .description = ""                }
        },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

/* ---- devices ---- */

const device_t st506_xt_wdxt_gen_device = {
    .name          = "WDXT-GEN (MFM)",
    .internal_name = "st506_xt_gen",
    .flags         = DEVICE_ISA,
    .local         = (HDD_BUS_MFM << 8) | ST506_XT_TYPE_WDXT_GEN,
    .init          = wd1100_ibmxt_init,
    .close         = wd1100_ibmxt_close,
    .reset         = NULL,
    .available     = wdxt_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t st506_xt_wd1002a_wx1_device = {
    .name          = "WD1002A-WX1 (MFM)",
    .internal_name = "st506_xt_wd1002a_wx1",
    .flags         = DEVICE_ISA,
    .local         = (HDD_BUS_MFM << 8) | ST506_XT_TYPE_WD1002A_WX1,
    .init          = wd1100_ibmxt_init,
    .close         = wd1100_ibmxt_close,
    .reset         = NULL,
    .available     = wd1002a_wx1_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = wd_config
};

const device_t st506_xt_wd1002a_wx1_nobios_device = {
    .name          = "WD1002A-WX1 (MFM) (No BIOS)",
    .internal_name = "st506_xt_wd1002a_wx1",
    .flags         = 0,
    .local         = (HDD_BUS_MFM << 8) | ST506_XT_TYPE_WD1002A_WX1_NOBIOS,
    .init          = wd1100_ibmxt_init,
    .close         = wd1100_ibmxt_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = wd_nobios_config
};

const device_t st506_xt_wd1004a_wx1_device = {
    .name          = "WD1004A-WX1 (MFM)",
    .internal_name = "st506_xt_wd1004a_wx1",
    .flags         = DEVICE_ISA,
    .local         = (HDD_BUS_MFM << 8) | ST506_XT_TYPE_WD1004A_WX1,
    .init          = wd1100_ibmxt_init,
    .close         = wd1100_ibmxt_close,
    .reset         = NULL,
    .available     = wd1004a_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = wd_config
};

const device_t st506_xt_wd1002a_27x_device = {
    .name          = "WD1002A-27X (RLL)",
    .internal_name = "st506_xt_wd1002a_27x",
    .flags         = DEVICE_ISA,
    .local         = (HDD_BUS_MFM << 8) | ST506_XT_TYPE_WD1002A_27X,
    .init          = wd1100_ibmxt_init,
    .close         = wd1100_ibmxt_close,
    .reset         = NULL,
    .available     = wd1002a_27x_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = wd_rll_config
};

const device_t st506_xt_wd1004_27x_device = {
    .name          = "WD1004-27X (RLL)",
    .internal_name = "st506_xt_wd1004_27x",
    .flags         = DEVICE_ISA,
    .local         = (HDD_BUS_MFM << 8) | ST506_XT_TYPE_WD1004_27X,
    .init          = wd1100_ibmxt_init,
    .close         = wd1100_ibmxt_close,
    .reset         = NULL,
    .available     = wd1004a_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = wd1004_rll_config
};

const device_t st506_xt_wd1004a_27x_device = {
    .name          = "WD1004a-27X (RLL)",
    .internal_name = "st506_xt_wd1004a_27x",
    .flags         = DEVICE_ISA,
    .local         = (HDD_BUS_MFM << 8) | ST506_XT_TYPE_WD1004A_27X,
    .init          = wd1100_ibmxt_init,
    .close         = wd1100_ibmxt_close,
    .reset         = NULL,
    .available     = wd1004a_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = wd_rll_config
};
