/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Western Digital WD1010/WD2010 Winchester disk controller.
 *
 *          Eight register task file selected by A2-A0, no data register
 *          in the chip (A2-A0 = 0 tri-states the bus, the sector buffer
 *          lives on the board), restore/seek/read/write/verify/format/
 *          diagnose/set parameters.
 *
 *          The WD2010 is the same controller with a 32-bit ECC engine
 *          (SDH bit 7), the SDHLE output and one more cylinder address
 *          bit; it is software compatible with the WD1010. Both are
 *          selected through mfm_init()'s variant argument.
 *
 *          Boards embedding this controller (the IBM PC-AT WD1003 in
 *          hdc_st506_at.c, the DEC VAXmate expansion box) only decode
 *          I/O, wire the IRQ line and load the images.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Fred N. van Kempen, <decwiz@yahoo.com>
 *
 *          Copyright 2008-2019 Sarah Walker.
 *          Copyright 2017-2019 Fred N. van Kempen.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/pic.h>
#include "cpu.h"
#include <86box/machine.h>
#include <86box/timer.h>
#include <86box/plat.h>
#include <86box/ui.h>
#include <86box/hdc.h>
#include <86box/hdc_st506_wd10.h>
#include <86box/hdd.h>

#define MFM_TIME (TIMER_USEC * 10)

/* Controller variants. Both are the same task file; the WD2010 adds the
   32-bit ECC engine (SDH bit 7) and a wider cylinder register. */
typedef struct {
    uint8_t cyl_mask; /* high byte mask of the cylinder register */
    uint8_t ecc;      /* accepts READ/WRITE with ECC (SDH bit 7) */
} wd10_variant_t;

static const wd10_variant_t wd10_variant_1010 = { 0xff, 0 }; /* unmasked: keep historical behaviour */
static const wd10_variant_t wd10_variant_2010 = { 0x07, 1 };

/*Rough estimate - MFM drives spin at 3600 RPM, with 17 sectors per track,
  meaning (3600/60)*17 = 1020 sectors per second, or 980us per sector.

  This is required for OS/2 on slow 286 systems, as the hard drive formatter
  will crash with 'internal processing error' if write sector interrupts are too
  close in time*/
#define SECTOR_TIME        (TIMER_USEC * 980)

#define STAT_ERR           0x01
#define STAT_INDEX         0x02
#define STAT_ECC           0x04
#define STAT_DRQ           0x08 /* data request */
#define STAT_DSC           0x10
#define STAT_WRFLT         0x20
#define STAT_READY         0x40
#define STAT_BUSY          0x80

#define ERR_DAM_NOT_FOUND  0x01 /* Data Address Mark not found */
#define ERR_TR000          0x02 /* track 0 not found */
#define ERR_ABRT           0x04 /* command aborted */
#define ERR_ID_NOT_FOUND   0x10 /* ID not found */
#define ERR_DATA_CRC       0x40 /* data CRC error */
#define ERR_BAD_BLOCK      0x80 /* bad block detected */

#define CMD_RESTORE        0x10
#define CMD_READ           0x20
#define CMD_WRITE          0x30
#define CMD_VERIFY         0x40
#define CMD_FORMAT         0x50
#define CMD_SEEK           0x70
#define CMD_DIAGNOSE       0x90
#define CMD_SET_PARAMETERS 0x91

typedef struct drive_t {
    int8_t  present;  /* drive is present */
    int8_t  hdd_num;  /* drive number in system */
    int8_t  steprate; /* current servo step rate */
    int8_t  spt;      /* physical #sectors per track */
    int8_t  hpc;      /* physical #heads per cylinder */
    int8_t  pad;
    int16_t tracks;   /* physical #tracks per cylinder */

    int8_t cfg_spt;   /* configured #sectors per track */
    int8_t cfg_hpc;   /* configured #heads per track */

    int16_t curcyl;   /* current track number */
} drive_t;

struct mfm_t {
    const wd10_variant_t *var; /* WD1010 or WD2010 */

    uint8_t  precomp;  /* 1: precomp/error register */
    uint8_t  error;
    uint8_t  secount;  /* 2: sector count register */
    uint8_t  sector;   /* 3: sector number */
    uint8_t  head;     /* 6: head number + drive select */
    uint8_t  command;  /* 7: command/status */
    uint8_t  status;
    uint8_t  fdisk;    /* 8: control register */
    uint16_t cylinder; /* 4/5: cylinder LOW and HIGH */

    int8_t reset;      /* controller in reset */
    int8_t irqstat;    /* current IRQ status */
    int8_t drvsel;     /* current selected drive */
    int8_t pad;

    int        pos;            /* offset within data buffer */
    pc_timer_t callback_timer; /* callback delay timer */

    uint16_t buffer[256]; /* data buffer (16b wide) */

    drive_t drives[MFM_NUM]; /* attached drives */
};

uint8_t wd10_read(uint16_t port, void *priv);
void    wd10_write(uint16_t port, uint8_t val, void *priv);

#ifdef ENABLE_ST506_AT_LOG
int st506_at_do_log = ENABLE_ST506_AT_LOG;

static void
st506_at_log(const char *fmt, ...)
{
    va_list ap;

    if (st506_at_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define st506_at_log(fmt, ...)
#endif

static __inline void
irq_raise(mfm_t *mfm)
{
    mfm->irqstat = 1;
    if (!(mfm->fdisk & 2))
        picint_common(1 << 14, PIC_IRQ_EDGE, 1, NULL);
}

static __inline void
irq_lower(mfm_t *mfm)
{
    mfm->irqstat = 0;
    if (!(mfm->fdisk & 2))
        picint_common(1 << 14, PIC_IRQ_EDGE, 0, NULL);
}

static __inline void
irq_update(mfm_t *mfm)
{
    uint8_t set = !(mfm->fdisk & 2) && mfm->irqstat;
    picint_common(1 << 14, PIC_IRQ_EDGE, set, NULL);
}

/*
 * Return the sector offset for the current register values.
 *
 * According to the WD1002/WD1003 technical reference manual,
 * this is not done entirely correct. It specifies that the
 * parameters set with the SET_DRIVE_PARAMETERS command are
 * to be used only for multi-sector operations, and that any
 * such operation can only be executed AFTER these parameters
 * have been set. This would imply that for regular single
 * transfers, the controller uses (or, can use) the actual
 * geometry information...
 */
static int
get_sector(mfm_t *mfm, off64_t *addr)
{
    const drive_t *drive = &mfm->drives[mfm->drvsel];

    /* FIXME: See if this is even needed - if the code is present, IBM AT
              diagnostics v2.07 will error with: ERROR 152 - SYSTEM BOARD. */
    if (drive->curcyl != mfm->cylinder) {
        st506_at_log("WD1003(%d) sector: wrong cylinder\n");
        return 1;
    }

    if (mfm->head > drive->cfg_hpc) {
        st506_at_log("WD1003(%d) get_sector: past end of configured heads\n",
                     mfm->drvsel);
        return 1;
    }

    if (mfm->sector >= (drive->cfg_spt + 1)) {
        st506_at_log("WD1003(%d) get_sector: past end of configured sectors\n",
                     mfm->drvsel);
        return 1;
    }

    /* We should check this in the SET_DRIVE_PARAMETERS command!  --FvK */
    if (mfm->head > drive->hpc) {
        st506_at_log("WD1003(%d) get_sector: past end of heads\n", mfm->drvsel);
        return 1;
    }

    if (mfm->sector >= (drive->spt + 1)) {
        st506_at_log("WD1003(%d) get_sector: past end of sectors\n", mfm->drvsel);
        return 1;
    }

    *addr = ((((off64_t) mfm->cylinder * drive->cfg_hpc) + mfm->head) * drive->cfg_spt) + (mfm->sector - 1);

    return 0;
}

static int
get_sector_format(mfm_t *mfm, off64_t *addr)
{
    const drive_t *drive = &mfm->drives[mfm->drvsel];

    /* FIXME: See if this is even needed - if the code is present, IBM AT
              diagnostics v2.07 will error with: ERROR 152 - SYSTEM BOARD. */
    if (drive->curcyl != mfm->cylinder) {
        st506_at_log("WD1003(%d) sector: wrong cylinder\n");
        return 1;
    }

    if (mfm->head > drive->cfg_hpc) {
        st506_at_log("WD1003(%d) get_sector: past end of configured heads\n",
                     mfm->drvsel);
        return 1;
    }

    /* We should check this in the SET_DRIVE_PARAMETERS command!  --FvK */
    if (mfm->head > drive->hpc) {
        st506_at_log("WD1003(%d) get_sector: past end of heads\n", mfm->drvsel);
        return 1;
    }

    *addr = ((((off64_t) mfm->cylinder * drive->cfg_hpc) + mfm->head) * drive->cfg_spt);

    return 0;
}

/* Move to the next sector using CHS addressing. */
static void
next_sector(mfm_t *mfm)
{
    drive_t *drive = &mfm->drives[mfm->drvsel];

    if (++mfm->sector == (drive->cfg_spt + 1)) {
        mfm->sector = 1;
        if (++mfm->head == drive->cfg_hpc) {
            mfm->head = 0;
            mfm->cylinder++;
            if (drive->curcyl < drive->tracks)
                drive->curcyl++;
        }
    }
}

static void
mfm_cmd(mfm_t *mfm, uint8_t val)
{
    drive_t *drive = &mfm->drives[mfm->drvsel];

    if (!drive->present) {
        /* This happens if sofware polls all drives. */
        st506_at_log("WD1003(%d) command %02x on non-present drive\n",
                     mfm->drvsel, val);
        mfm->command = 0xff;
        mfm->status  = STAT_BUSY;
        timer_set_delay_u64(&mfm->callback_timer, 200 * MFM_TIME);
        return;
    }

    irq_lower(mfm);
    mfm->command = val;
    mfm->error   = 0;

    switch (val & 0xf0) {
        case CMD_RESTORE:
            drive->steprate = (val & 0x0f);
            mfm->command &= 0xf0;
            mfm->status = STAT_BUSY;
            timer_set_delay_u64(&mfm->callback_timer, 200 * MFM_TIME);
            break;

        case CMD_SEEK:
            drive->steprate = (val & 0x0f);
            mfm->command &= 0xf0;
            mfm->status = STAT_BUSY;
            timer_set_delay_u64(&mfm->callback_timer, 200 * MFM_TIME);
            break;

        default:
            mfm->command = val;
            switch (val) {
                case CMD_READ:
                case CMD_READ + 1:
                case CMD_READ + 2:
                case CMD_READ + 3:
                    st506_at_log("WD1003(%d) read, opt=%d\n",
                                 mfm->drvsel, val & 0x03);
                    mfm->command &= 0xfc;
                    if (val & 2) {
                        if (!mfm->var->ecc)
                            fatal("WD1003: READ with ECC\n");
                        /* WD2010: ECC is handled as CRC here; the 32-bit
                           ECC engine is not modelled. */
                    }
                    mfm->status = STAT_BUSY;
                    {
                        /* Calculate seek time based on CHS position */
                        off64_t seek_addr = ((((off64_t) mfm->cylinder * drive->cfg_hpc) + mfm->head) * drive->cfg_spt) + (mfm->sector - 1);
                        double  seek_us   = hdd_seek_get_time(&hdd[drive->hdd_num], (uint32_t) seek_addr, HDD_OP_READ, 0, 0.0);
                        uint64_t delay    = (seek_us > (200.0 * MFM_TIME / (double) TIMER_USEC))
                                            ? (uint64_t)(seek_us * TIMER_USEC) : 200 * MFM_TIME;
                        timer_set_delay_u64(&mfm->callback_timer, delay);
                    }
                    break;

                case CMD_WRITE:
                case CMD_WRITE + 1:
                case CMD_WRITE + 2:
                case CMD_WRITE + 3:
                    st506_at_log("WD1003(%d) write, opt=%d\n",
                                 mfm->drvsel, val & 0x03);
                    mfm->command &= 0xfc;
                    if (val & 2) {
                        if (!mfm->var->ecc)
                            fatal("WD1003: WRITE with ECC\n");
                    }
                    mfm->status = STAT_READY | STAT_DRQ | STAT_DSC;
                    mfm->pos    = 0;
                    break;

                case CMD_VERIFY:
                case CMD_VERIFY + 1:
                    mfm->command &= 0xfe;
                    mfm->status = STAT_BUSY;
                    timer_set_delay_u64(&mfm->callback_timer, 200 * MFM_TIME);
                    break;

                case CMD_FORMAT:
                    mfm->status = STAT_DRQ | STAT_BUSY;
                    mfm->pos    = 0;
                    break;

                case CMD_DIAGNOSE:
                    mfm->status = STAT_BUSY;
                    timer_set_delay_u64(&mfm->callback_timer, 200 * MFM_TIME);
                    break;

                case CMD_SET_PARAMETERS:
                    mfm->status = STAT_BUSY;
                    timer_set_delay_u64(&mfm->callback_timer, 200 * MFM_TIME);
                    break;

                default:
                    st506_at_log("WD1003: bad command %02X\n", val);
                    mfm->status = STAT_BUSY;
                    timer_set_delay_u64(&mfm->callback_timer, 200 * MFM_TIME);
                    break;
            }
    }
}

void
wd10_writew(uint16_t port, uint16_t val, void *priv)
{
    mfm_t *mfm = (mfm_t *) priv;

    if (port > 0x01f0) {
        wd10_write(port, val & 0xff, priv);
        if (port != 0x01f7)
            wd10_write(port + 1, (val >> 8) & 0xff, priv);
    } else {
        mfm->buffer[mfm->pos >> 1] = val;
        mfm->pos += 2;

        if (mfm->pos >= 512) {
            mfm->pos    = 0;
            mfm->status = STAT_BUSY;
            {
                drive_t *wr_drive = &mfm->drives[mfm->drvsel];
                off64_t  wr_addr  = ((((off64_t) mfm->cylinder * wr_drive->cfg_hpc) + mfm->head) * wr_drive->cfg_spt) + (mfm->sector - 1);
                double   seek_us  = hdd_seek_get_time(&hdd[wr_drive->hdd_num], (uint32_t) wr_addr, HDD_OP_WRITE, 1, 0.0);
                uint64_t delay    = (seek_us > (double)(SECTOR_TIME / TIMER_USEC))
                                    ? (uint64_t)(seek_us * TIMER_USEC) : SECTOR_TIME;
                timer_set_delay_u64(&mfm->callback_timer, delay);
            }
        }
    }
}

void
wd10_write(uint16_t port, uint8_t val, void *priv)
{
    mfm_t *mfm = (mfm_t *) priv;

    uint8_t old;

    st506_at_log("WD1003 write(%04x, %02x)\n", port, val);

    switch (port) {
        case 0x01f0: /* data */
            wd10_writew(port, val | (val << 8), priv);
            return;

        case 0x01f1: /* write precompenstation */
            mfm->precomp = val;
            return;

        case 0x01f2: /* sector count */
            mfm->secount = val;
            return;

        case 0x01f3: /* sector */
            mfm->sector = val;
            return;

        case 0x01f4: /* cylinder low */
            mfm->cylinder = (mfm->cylinder & 0xff00) | val;
            return;

        case 0x01f5: /* cylinder high */
            mfm->cylinder = (mfm->cylinder & 0xff) | ((val & mfm->var->cyl_mask) << 8);
            return;

        case 0x01f6: /* drive/head */
            mfm->head   = val & 0xF;
            mfm->drvsel = !!(val & 0x10);
            if (mfm->drives[mfm->drvsel].present)
                mfm->status = STAT_READY | STAT_DSC;
            else
                mfm->status = 0;
            return;

        case 0x01f7: /* command register */
            mfm_cmd(mfm, val);
            break;

        case 0x03f6: /* device control */
            val &= 0x0f;
            if ((mfm->fdisk & 0x04) && !(val & 0x04)) {
                timer_set_delay_u64(&mfm->callback_timer, 500 * MFM_TIME);
                mfm->reset  = 1;
                mfm->status = STAT_BUSY;
            } else if (!(mfm->fdisk & 0x04) && (val & 0x04)) {
                /* Drive held in reset. */
                timer_disable(&mfm->callback_timer);
                mfm->status = STAT_BUSY;
            }
            old = mfm->fdisk;
            mfm->fdisk = val;
            if (!(val & 0x02) && (old & 0x02))
                irq_update(mfm);
            break;

        default:
            break;
    }
}

uint16_t
wd10_readw(uint16_t port, void *priv)
{
    mfm_t   *mfm = (mfm_t *) priv;
    uint16_t ret;

    if (port > 0x01f0) {
        ret = wd10_read(port, priv);
        if (port == 0x01f7)
            ret |= 0xff00;
        else
            ret |= (wd10_read(port + 1, priv) << 8);
    } else {
        ret = mfm->buffer[mfm->pos >> 1];
        mfm->pos += 2;
        if (mfm->pos >= 512) {
            mfm->pos    = 0;
            mfm->status = STAT_READY | STAT_DSC;
            if (mfm->command == CMD_READ) {
                mfm->secount = (mfm->secount - 1) & 0xff;
                if (mfm->secount) {
                    next_sector(mfm);
                    mfm->status = STAT_BUSY | STAT_READY | STAT_DSC;
                    {
                        drive_t *rd_drive = &mfm->drives[mfm->drvsel];
                        off64_t  rd_addr  = ((((off64_t) mfm->cylinder * rd_drive->cfg_hpc) + mfm->head) * rd_drive->cfg_spt) + (mfm->sector - 1);
                        double   seek_us  = hdd_seek_get_time(&hdd[rd_drive->hdd_num], (uint32_t) rd_addr, HDD_OP_READ, 1, 0.0);
                        uint64_t delay    = (seek_us > (double)(SECTOR_TIME / TIMER_USEC))
                                            ? (uint64_t)(seek_us * TIMER_USEC) : SECTOR_TIME;
                        timer_set_delay_u64(&mfm->callback_timer, delay);
                    }
                } else
                    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
            }
        }
    }

    return ret;
}

uint8_t
wd10_read(uint16_t port, void *priv)
{
    mfm_t  *mfm = (mfm_t *) priv;
    uint8_t ret = 0xff;


    switch (port) {
        case 0x01f0: /* data */
            ret = wd10_readw(port, mfm) & 0xff;
            break;

        case 0x01f1: /* error */
            ret = mfm->error;
            break;

        case 0x01f2: /* sector count */
            ret = mfm->secount;
            break;

        case 0x01f3: /* sector */
            ret = mfm->sector;
            break;

        case 0x01f4: /* CYlinder low */
            ret = (uint8_t) (mfm->cylinder & 0xff);
            break;

        case 0x01f5: /* Cylinder high */
            ret = (uint8_t) (mfm->cylinder >> 8);
            break;

        case 0x01f6: /* drive/head */
            ret = (uint8_t) (0xa0 | mfm->head | (mfm->drvsel ? 0x10 : 0));
            break;

        case 0x01f7: /* Status */
            irq_lower(mfm);
            ret = mfm->status;
            break;

        default:
            break;
    }

    st506_at_log("WD1003 read(%04x) = %02x\n", port, ret);

    return ret;
}

static void
do_seek(mfm_t *mfm)
{
    drive_t *drive = &mfm->drives[mfm->drvsel];

    st506_at_log("WD1003(%d) seek(%d) max=%d\n",
                 mfm->drvsel, mfm->cylinder, drive->tracks);

    if (mfm->cylinder < drive->tracks)
        drive->curcyl = mfm->cylinder;
    else
        drive->curcyl = drive->tracks - 1;
}

static void
do_callback(void *priv)
{
    mfm_t   *mfm   = (mfm_t *) priv;
    drive_t *drive = &mfm->drives[mfm->drvsel];
    off64_t  addr;

    if (mfm->reset) {
        st506_at_log("WD1003(%d) reset\n", mfm->drvsel);

        mfm->status   = STAT_READY | STAT_DSC;
        mfm->error    = 1;
        mfm->secount  = 1;
        mfm->sector   = 1;
        mfm->head     = 0;
        mfm->cylinder = 0;

        drive->steprate = 0x0f; /* default steprate */
        drive->cfg_spt  = 0;    /* need new parameters */

        mfm->reset = 0;

        ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
        ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);

        return;
    }

    switch (mfm->command) {
        case CMD_RESTORE:
            st506_at_log("WD1003(%d) restore, step=%d\n",
                         mfm->drvsel, drive->steprate);
            drive->curcyl = 0;
            mfm->cylinder = 0;
            mfm->status   = STAT_READY | STAT_DSC;
            irq_raise(mfm);
            break;

        case CMD_SEEK:
            st506_at_log("WD1003(%d) seek, step=%d\n",
                         mfm->drvsel, drive->steprate);
            do_seek(mfm);
            mfm->status = STAT_READY | STAT_DSC;
            irq_raise(mfm);
            break;

        case CMD_READ:
            st506_at_log("WD1003(%d) read(%d,%d,%d)\n",
                         mfm->drvsel, mfm->cylinder, mfm->head, mfm->sector);
            do_seek(mfm);
            if (get_sector(mfm, &addr)) {
                mfm->error  = ERR_ID_NOT_FOUND;
read_error:
                mfm->status = STAT_READY | STAT_DSC | STAT_ERR;
                irq_raise(mfm);
                break;
            }

            if (hdd_image_read(drive->hdd_num, addr, 1, (uint8_t *) mfm->buffer) < 0) {
                mfm->error = ERR_BAD_BLOCK;
                goto read_error;
            }

            mfm->pos    = 0;
            mfm->status = STAT_DRQ | STAT_READY | STAT_DSC;
            irq_raise(mfm);
            ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 1);
            break;

        case CMD_WRITE:
            st506_at_log("WD1003(%d) write(%d,%d,%d)\n",
                         mfm->drvsel, mfm->cylinder, mfm->head, mfm->sector);
            do_seek(mfm);
            if (get_sector(mfm, &addr)) {
                mfm->error  = ERR_ID_NOT_FOUND;
write_error:
                mfm->status = STAT_READY | STAT_DSC | STAT_ERR;
                irq_raise(mfm);
                break;
            }

            if (hdd_image_write(drive->hdd_num, addr, 1, (uint8_t *) mfm->buffer) < 0) {
                mfm->error = ERR_BAD_BLOCK;
                goto write_error;
            }
            irq_raise(mfm);
            mfm->secount = (mfm->secount - 1) & 0xff;

            mfm->status = STAT_READY | STAT_DSC;
            if (mfm->secount) {
                /* More sectors to do.. */
                mfm->status |= STAT_DRQ;
                mfm->pos = 0;
                next_sector(mfm);
                ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 1);
            } else
                ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
            break;

        case CMD_VERIFY:
            st506_at_log("WD1003(%d) verify(%d,%d,%d)\n",
                         mfm->drvsel, mfm->cylinder, mfm->head, mfm->sector);
            do_seek(mfm);
            mfm->pos    = 0;
            mfm->status = STAT_READY | STAT_DSC;
            irq_raise(mfm);
            ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 1);
            break;

        case CMD_FORMAT:
            st506_at_log("WD1003(%d) format(%d,%d)\n",
                         mfm->drvsel, mfm->cylinder, mfm->head);
            do_seek(mfm);
            if (get_sector_format(mfm, &addr)) {
                mfm->error  = ERR_ID_NOT_FOUND;
                mfm->status = STAT_READY | STAT_DSC | STAT_ERR;
                irq_raise(mfm);
                break;
            }

            hdd_image_zero(drive->hdd_num, addr, mfm->secount);

            mfm->status = STAT_READY | STAT_DSC;
            irq_raise(mfm);
            ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 1);
            break;

        case CMD_DIAGNOSE:
            st506_at_log("WD1003(%d) diag\n", mfm->drvsel);

            /* This is basically controller diagnostics - it resets drive select to 0,
               and resets error and status to ready, DSC, and no error detected. */
            mfm->drvsel = 0;
            drive       = &mfm->drives[mfm->drvsel];

            drive->steprate = 0x0f;
            mfm->error      = 1;
            mfm->status     = STAT_READY | STAT_DSC;
            irq_raise(mfm);
            break;

        case CMD_SET_PARAMETERS:
            /*
             * NOTE:
             *
             * We currently just set these parameters, and
             * never bother to check if they "fit within"
             * the actual parameters, as determined by the
             * image loader.
             *
             * The difference in parameters is OK, and
             * occurs when the BIOS or operating system
             * decides to use a different translation
             * scheme, but either way, it SHOULD always
             * fit within the actual parameters!
             *
             * We SHOULD check that here!! --FvK
             */
            drive->cfg_spt = mfm->secount;
            drive->cfg_hpc = mfm->head + 1;

            st506_at_log("WD1003(%d) parameters: tracks=%d, spt=%i, hpc=%i\n",
                         mfm->drvsel, drive->tracks,
                         drive->cfg_spt, drive->cfg_hpc);

            mfm->command = 0x00;
            mfm->status  = STAT_READY | STAT_DSC;
            mfm->error   = 1;
            irq_raise(mfm);
            break;

        default:
            st506_at_log("WD1003(%d) callback on unknown command %02x\n",
                         mfm->drvsel, mfm->command);
            mfm->status = STAT_READY | STAT_ERR | STAT_DSC;
            mfm->error  = ERR_ABRT;
            irq_raise(mfm);
            break;
    }
}

/*
 * Allocate a controller. 'variant' is 0 for the WD1010 and 1 for the
 * WD2010; the only differences are the cylinder register width and
 * whether READ/WRITE with ECC (SDH bit 7) is accepted.
 */
mfm_t *
wd10_init(int variant)
{
    mfm_t *mfm;

    st506_at_log("WD10: ISA MFM/RLL Fixed Disk Adapter initializing ...\n");
    mfm = calloc(1, sizeof(mfm_t));

    mfm->var = variant ? &wd10_variant_2010 : &wd10_variant_1010;

    mfm->status = STAT_READY | STAT_DSC; /* drive is ready */
    mfm->error  = 1;                     /* no errors */

    timer_add(&mfm->callback_timer, do_callback, mfm, 0);

    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);

    return mfm;
}

/* Register a drive image (already loaded through hdd_image_load()). */
void
wd10_attach(mfm_t *mfm, int unit, int hdd_num, int spt, int hpc, int tracks)
{
    drive_t *drive = &mfm->drives[unit];

    drive->spt     = spt;
    drive->hpc     = hpc;
    drive->tracks  = tracks;
    drive->hdd_num = hdd_num;
    drive->present = 1;
}

void
wd10_close(mfm_t *mfm)
{
    if (mfm == NULL)
        return;

    timer_disable(&mfm->callback_timer);
    free(mfm);

    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
}
