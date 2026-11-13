/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the Western Digital WD279x floppy disk
 *          controller family.
 *
 *          The WD279x is a sector-level controller: four registers (status/
 *          command, track, sector, data), one command byte, and the host (or
 *          the on-chip DMA) moves every byte through the data register.  That
 *          is a different programming model from the uPD765 model in fdc.c,
 *          so this file implements the registers and the command state
 *          machine itself and reuses the drive layer (fdd_*) for the medium.
 *
 *          The family differs in the data bus and in the side select output:
 *          the WD2791 drives an inverted data bus, the WD2793 a true one, and
 *          the WD2795/WD2797 add a side select output.  None of that changes
 *          the command set implemented here; the board decides the polarity.
 *
 * Authors: Olivier Galibert, <galibert@pobox.com>
 *          WNT50
 *
 *          Copyright 2012-2015 Olivier Galibert.
 *          Copyright 2026 WNT50.
 */
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/plat.h>
#include <86box/mem.h>
#include <86box/device.h>

#include <86box/plat_unused.h>
#include <86box/timer.h>
#include <86box/pic.h>
#include <86box/dma.h>
#include <86box/fdd.h>
#include "cpu.h"
#include <86box/fdd_86f.h>
#include <86box/io.h>
#include <86box/log.h>
#include <86box/wd279x.h>


/* Status register, TABLE 4.  Bits 5/4/2/1 change meaning with the command
   type, which is what status_type_1 selects. */
#define WD_S_BUSY    0x01 /* command in progress */
#define WD_S_DRQ     0x02 /* Type II/III: data request */
#define WD_S_INDEX   0x02 /* Type I: index pulse */
#define WD_S_LOST    0x04 /* Type II/III: lost data */
#define WD_S_TRK0    0x04 /* Type I: head at track 0 */
#define WD_S_CRC     0x08 /* CRC error in the ID or data field */
#define WD_S_RNF     0x10 /* record not found */
#define WD_S_SEEKERR 0x10 /* Type I: seek error */
#define WD_S_HLD     0x20 /* Type I: head loaded */
#define WD_S_RECTYPE 0x20 /* Type II: deleted data address mark */
#define WD_S_WP      0x40 /* write protected */
#define WD_S_NOTRDY  0x80 /* not ready / no disk */

/* Sector length handed to the drive layer.  The 279x has no N register --
   the length comes from the ID field -- and every PC-X disk uses 512-byte
   sectors, so the drive layer is queried for those. */
#define WD_SECTOR_SIZE 512

/* Density index the drive layer expects: 250 kbit/s MFM at 300 rpm. */
#define WD_DENSITY 2

#define WD_CLOCK_MHZ 2

static wd279x_t *wd279x_cur = NULL;

static void wd279x_command(wd279x_t *dev, uint8_t val);
static void wd279x_timer(void *priv);

/* The real chip walks the rest of the track looking for the next ID before it
   gives up.  A chained sector past the end of the track never shows up, and a
   polled driver never asserts DMA terminal count either, so without this bound
   the command would sit in READ with BUSY set for ever.  One revolution at
   300 rpm is 200 ms. */
#define WD_RECORD_SEARCH_US (200 * 1000)

/* Shortest time a command stays BUSY.  Long enough for a polled driver to see
   it, short enough not to slow anything down. */
#define WD_INSTANT_CMD_US   (200)

/* ------------------------------------------------------------------------- */
/* helpers                                                                   */
/* ------------------------------------------------------------------------- */

static fdd_drive_t *
wd_drive(wd279x_t *dev)
{
    void *drv = dev->fdd[dev->drive & 3];

    return (fdd_drive_t *) drv;
}

/* The byte clock of the mounted medium.  fdd.c derives it from the format's
   own bit-cell timing and keeps that helper to itself, so ask the format.
   With nothing mounted fall back to the same idle period fdd.c uses. */
static uint64_t
wd_byte_period(fdd_drive_t *drv)
{
    if ((drv != NULL) && (drv->fmt.byteperiod != NULL))
        return drv->fmt.byteperiod(drv);

    return 32ULL * TIMER_USEC;
}


/* The bitcell period the drive layer samples with comes from the data rate the
   controller has selected, and the WD279x only has the one DDEN pin to say it.
   Leaving fdc.bitcell_period at zero made the bitstream layer search every
   sector at a zero-length bit cell, which is why every read ended in Record
   Not Found even with the right drive, track and sector. */
static int
wd279x_rate(wd279x_t *dev)
{
    fdd_drive_t *drv = wd_drive(dev);

    if ((drv != NULL) && !dev->dden) {
        /* Single density: same 250 kbps cell timing, FM framing. */
        return 2;
    }

    if ((drv != NULL) && (fdd_get_flags(drv) & 0x02)) {
        /* A 1.2M 5.25" drive spins at 360 rpm, so its double density rate is
           300 kbps rather than 250. */
        if (!fdd_hole(drv))
            return 1;
    }

    if ((drv != NULL) && fdd_hole(drv))
        return 0;   /* high density medium */

    return 2;
}

static void
wd279x_update_rate(wd279x_t *dev)
{
    dev->rate  = (uint8_t) wd279x_rate(dev);
    dev->mfm   = dev->dden ? 0 : 1;
    /* FM frames one data bit per two bit cells: at the same 250 kbps nominal
       rate a single density track is written at 125 kbps, so its cell lasts
       twice as long. The bitstream layer derives the medium's cell from the
       track's own flags (16000 ns for an FM/250 kbps track) and refuses the
       command outright when the controller disagrees, which is what made every
       FM read come back as NO IDAM without ever searching the track. */
    dev->bitcell_period = (uint32_t) fdd_get_bitcell_period(dev->rate) *
                              (dev->dden ? 2 : 1);
}

void
wd279x_set_post_us(wd279x_t *dev, uint32_t us)
{
    if (dev != NULL)
        dev->post_us = us;
}

/* A ROM that waits for BUSY to drop before it takes the sector's bytes cannot
   be told to wait for the buffer to drain: it will not read while BUSY is up,
   so the command must finish with the bytes still queued and stay readable. */
void
wd279x_set_busy_on_end(wd279x_t *dev, int enable)
{
    if (dev != NULL)
        dev->busy_on_end = enable ? 1 : 0;
}

void
wd279x_set_dma(wd279x_t *dev, int enable)
{
    if (dev == NULL)
        return;

    dev->dma_mode = enable ? 1 : 0;
}

static uint8_t wd_status(wd279x_t *dev);
static void    wd_fmt_put(wd279x_t *dev, uint8_t val);

static void
wd_irq_raise(wd279x_t *dev, UNUSED(const char *why))
{
    dev->intrq = 1;
    if (dev->irq_cb != NULL)
        dev->irq_cb(dev->irq_priv, 1);
    else
        picint(1 << WD279X_IRQ);
}

static void
wd_irq_lower(wd279x_t *dev)
{
    dev->intrq = 0;
    if (dev->irq_cb != NULL)
        dev->irq_cb(dev->irq_priv, 0);
    else
        picintc(1 << WD279X_IRQ);
}

/* Finish the command: drop busy, latch the interrupt and let the registers
   read back.  Every path out of the state machine ends here. */
static void
wd_finish(wd279x_t *dev)
{

    /* Post-byte gap: CRC and the status phase.  Without it the interrupt lands
       in the same emulated instant as the last data byte, which is the one
       moment a polled driver is not yet listening for it. */
    if (dev->post_us && !dev->latched) {
        dev->latched = 1;
        dev->state   = WD_ST_LATCH;
        timer_set_delay_u64(&dev->timer, (uint64_t) dev->post_us * TIMER_USEC);
        return;
    }

    dev->busy         = 0;
    dev->state        = WD_ST_IDLE;
    dev->drq          = 0;
    dev->done_pending  = 0;
    dev->chain_pending = 0;
    wd_irq_raise(dev, "finish");
}

static void
wd_done(wd279x_t *dev)
{
    /* No terminal count here.  The :YES BIOS arms the 80186 channel once for
       a whole multi-sector read and never programs the count, so the floppy's
       per-record TC is not what ends the block: stopping the channel after the
       first sector left the next command without a descriptor, it fell back to
       the polled queue, filled the record buffer and never finished -- which
       is the "0x40" retry the loader spins on. */

    /* Hold the command open until the host has read the bytes still queued
       behind the data register, or a polling driver would see BUSY drop in the
       middle of the sector. */
    if (!dev->busy_on_end && ((dev->buf_len > 0) || dev->dr_full)) {
        dev->done_pending = 1;
        return;
    }

    wd_finish(dev);
}


/* The drive layer wants the sector size as the N code of the ID field (0 = 128
   bytes, 2 = 512 bytes), not a byte count; passing 512 there set N to 0 and no
   sector on the medium ever matched. */
static int
wd_size_code(wd279x_t *dev)
{
    int n = 0;

    while ((n < 7) && ((128 << n) < dev->sector_size))
        n++;

    return n;
}

static uint8_t
wd_status(wd279x_t *dev)
{
    fdd_drive_t *drv = wd_drive(dev);
    uint8_t      ret;

    if (dev->type1) {
        /* Type I carries the mechanical state: S4 seek error, S2 track 0,
           S1 index pulse, S5 head loaded. */
        ret = dev->status & (WD_S_CRC | WD_S_RNF | WD_S_WP | WD_S_TRK0 | WD_S_LOST);

        if (!drv || !fdd_can_read_medium(drv))
            ret |= WD_S_NOTRDY;
        if (drv && fdd_track0(drv))
            ret |= WD_S_TRK0;
        if (drv && fdd_index(drv))
            ret |= WD_S_INDEX;
        ret |= WD_S_HLD;
    } else {
        /* Type II/III redefines two bits: S2 is LOST DATA, not track 0, and S6
           is write protect only while writing.  Letting a stale track-0 or
           protect bit from the Type I status fall through reported a lost byte
           on every plain read. */
        ret = dev->status & (WD_S_CRC | WD_S_RNF | WD_S_LOST);

        if ((dev->state == WD_ST_WRITE) || dev->wp_latched)
            ret |= dev->status & WD_S_WP;
        if (dev->deleted)
            ret |= WD_S_RECTYPE;
        /* DRQ describes the data register, and a byte waiting in the record
           buffer is reachable at the very next read: keep the two in step. */
        if (dev->drq || (dev->buf_len > 0))
            ret |= WD_S_DRQ;
    }

    if (dev->busy)
        ret |= WD_S_BUSY;

    return ret;
}

/* ------------------------------------------------------------------------- */
/* Type II/III data path                                                     */
/* ------------------------------------------------------------------------- */

static int
wd_buf_push(wd279x_t *dev, uint8_t data)
{
    if (dev->buf_len >= (int) sizeof(dev->buf))
        return 0;

    dev->buf[dev->buf_head] = data;
    dev->buf_head           = (dev->buf_head + 1) % (int) sizeof(dev->buf);
    dev->buf_len++;

    return 1;
}

/* A Read Sector command never carries the sector's size: the chip takes the N
   field out of the ID it just matched. Single density is the one case where the
   medium fixes it -- FM on 5.25" media always uses 128-byte sectors -- while a
   double density track may be 256, 512 or 1024 depending on how it was written,
   so there the command's own size code stays the best guess. */
static int
wd_sector_size_code(wd279x_t *dev)
{
    if (dev->dden)
        return 0;

    return wd_size_code(dev);
}

/* A *command* starts its scan at the index hole: a bitstream scan that begins
   inside a sync run never arms the address-mark detector.  Chained sectors of
   the same command keep scanning forward from where the previous one ended --
   rewinding the position for every sector would also rewind it before it ever
   reaches the index hole, so a scan that finds no address mark could never
   report the two revolutions it is allowed to take. */
/* The image numbers its sectors from 1 while a formatter that wrote the disk
   may have numbered them from 0; sector_base says which one the board sees. */
static int
wd_image_sector(wd279x_t *dev, int sector)
{
    return sector + 1 - dev->sector_base;
}

/* Hand the parameters of the command the chip is running down to the drive,
   the way the uPD765 adapter does: the medium works from its own copy of them
   and never asks the controller back. */
static void
wd_xfer_sync(wd279x_t *dev, fdd_drive_t *drv)
{
    if (drv == NULL)
        return;

    drv->xfer.dtl            = (uint8_t) dev->dtl;
    drv->xfer.dma            = dev->dma ? 1 : 0;
    drv->xfer.sk             = 0;
    drv->xfer.verify         = dev->verify ? 1 : 0;
    drv->xfer.deleted        = dev->deleted ? 1 : 0;
    drv->xfer.command        = dev->command;
    drv->xfer.compare        = dev->compare ? 1 : 0;
    drv->xfer.step_rate      = 0;
    drv->xfer.format_n       = (uint8_t) (dev->fmt_active ? dev->fmt_n : 0);
    drv->xfer.format_sectors = (uint8_t) (dev->fmt_active ? dev->fmt_count : 0);
    drv->xfer.format_id      = &dev->fmt_id;
}

static void
wd_sector_read(wd279x_t *dev, int sector)
{
    fdd_drive_t *drv = wd_drive(dev);

    if (drv == NULL)
        return;

    wd_xfer_sync(dev, drv);

    fdd_readsector(drv, wd_image_sector(dev, sector), dev->track_reg, dev->side,
                   dev->rate, wd_sector_size_code(dev));
}

/* Chained sectors: the controller looks for the next record once this one is
   over.  On the real chip that moment is the end of the data field, and the
   host still has the last byte to take, so the search starts when the data
   register is drained -- arming it earlier lets the whole chain race ahead of
   the driver.  The chip keeps going in ascending sector order until no record
   with the next number exists on the track, so there is no EOT register to
   stop at. */
static void
wd_chain_next(wd279x_t *dev)
{
    dev->chain_follow = 1;
    dev->sector_reg = (uint8_t) (dev->sector_reg + 1);
    timer_set_delay_u64(&dev->timer, WD_RECORD_SEARCH_US * TIMER_USEC);
    wd_sector_read(dev, dev->sector_reg);
}

/* One byte time has passed.  The chip moves the next byte of the record into
   the data register and raises DRQ; if the host has not taken the previous
   one, that byte is lost and S2 says so (WD279X-02, Read Sector). */
static void
wd_byte_tick(void *priv)
{
    wd279x_t    *dev = (wd279x_t *) priv;
    fdd_drive_t *drv = wd_drive(dev);

    if (dev->state == WD_ST_WRITETRACK) {
        int data = dma_channel_read(dev->dma_ch);

        if (data == DMA_NODATA)
            data = 0xff;

        wd_fmt_put(dev, (uint8_t) (data & 0xff));

        if (dev->state == WD_ST_WRITETRACK)
            timer_set_delay_u64(&dev->byte_timer, wd_byte_period(drv));
        return;
    }

    if ((dev->state != WD_ST_READ) && (dev->state != WD_ST_READID) &&
        (dev->state != WD_ST_READTRACK))
        return;

    /* The byte timer does not move bytes any more.  Two places promoting a
       byte into the data register consumed them out of order: the medium
       delivered "VOL1S", the guest received "VL1SN" and the SINIX POST
       rejected the disk.  wd_buf_pop() is the only reader of the queue. */
    timer_set_delay_u64(&dev->byte_timer, wd_byte_period(drv));
}

static int
wd_buf_pop(wd279x_t *dev)
{
    if (!dev->dr_full) {
        if (dev->buf_len <= 0) {
            dev->drq = 0;
            return -1;
        }

        /* The byte is already in the record buffer; the data register is
           empty only because the byte timer has not moved it across yet.  A
           polled driver reads the moment DRQ says there is data, so hand the
           byte over here: otherwise it read an empty register, saw LOST DATA
           and aborted the boot read. */
        dev->dr        = dev->buf[dev->buf_tail];
        dev->buf_tail  = (dev->buf_tail + 1) % (int) sizeof(dev->buf);
        dev->buf_len--;
        dev->dr_full   = 1;
        dev->drq       = 1;
    }

    dev->data_reg = dev->dr;
    dev->dr_full  = 0;
    dev->drq      = 0;

    /* A chained record clears byte_count when it ends, so waiting for
       byte_count to reach sector_size here never fired and the command stayed
       busy for ever.  The pending flag is the whole condition. */
    if (dev->chain_pending && (dev->buf_len == 0)) {
        dev->chain_pending = 0;
        wd_chain_next(dev);

        return dev->data_reg;
    }

    if ((dev->buf_len == 0) && (dev->byte_count >= dev->sector_size)) {
        if (dev->chain_pending) {
            /* The host has taken the last byte of this record, so the next one
               can be searched for. */
            dev->chain_pending = 0;
            wd_chain_next(dev);
        } else if (dev->done_pending) {
            wd_finish(dev);
        }
    }

    return dev->data_reg;
}

/* Called by the drive layer for every byte it takes off the medium.  Returns
   0 to keep the transfer going and -1 when the controller no longer wants
   bytes (which is how the drive layer learns that the sector is over). */
static int
wd_data_in(void *priv, uint8_t data, int last)
{
    wd279x_t *dev = (wd279x_t *) priv;
    int       res;



    /* A record starts with an empty data register.  Since the medium only
       queues bytes now, whatever the register held belongs to the previous
       record and handing it to the host shifted this one by a byte. */
    if ((dev->byte_count == 0) && !last) {
        dev->dr_full = 0;
        dev->drq     = 0;
    }

    if ((dev->state != WD_ST_READ) && (dev->state != WD_ST_READID) &&
        (dev->state != WD_ST_READTRACK)) {
        return -1;
    }

    timer_disable(&dev->timer);



    if ((dev->state == WD_ST_READID) && (dev->id_count < (int) sizeof(dev->id_buf)))
        dev->id_buf[dev->id_count] = data;

    if (dev->dma) {
        /* Put the byte where the hardware keeps it while DRQ is asserted: the
           data register.  The DMA controller fetches it through the source
           pointer (an I/O read of 0xF903), so without this the channel would
           read back whatever the port returned on its own. */
        dev->dr      = data;
        dev->dr_full = 1;
        dev->drq     = 1;
        dev->in_dma_fetch = 1;
        res = dma_channel_write(dev->dma_ch, data);
        dev->in_dma_fetch = 0;
        if (res != DMA_NODATA) {
            dev->dr_full = 0;
            dev->drq     = 0;
        } else {
            /* No descriptor armed: this command is a PIO transfer.  Clearing
               the flag here keeps every following byte on the queue path. */
            dev->dma = 0;
        }
        if (res == DMA_NODATA) {
            /* The guest has not armed the DMA channel: put the byte where a
               polling driver looks for it.  Filling the data register here,
               as the part does when the byte leaves the shifter, keeps the
               register in step with the media; queueing it for a byte timer
               put the driver a byte period behind and it read empty. */
            if (!dev->dr_full) {
                dev->dr        = data;
                dev->dr_full   = 1;
                dev->drq       = 1;
            } else if (!wd_buf_push(dev, data)) {
                dev->status |= WD_S_LOST;
                wd_done(dev);
                return -1;
            }
        } else if (res & DMA_OVER) {
            dev->tc = 1;
            dev->tc_seen = 1;
        }
    } else {
        /* One entry for the byte stream: the record buffer.  Letting the
           medium write the data register directly while the buffer held an
           older byte delivered them out of order -- the SINIX POST received
           "VL1SN" instead of "VOL1S". */
        if (!wd_buf_push(dev, data)) {
            dev->status |= WD_S_LOST;
            wd_done(dev);
            return -1;
        }
        dev->drq = 1;
    }

    dev->byte_count++;
    dev->id_count++;

    if (dev->state == WD_ST_READID) {
        /* C, H, R, N and the two CRC bytes. */
        if (dev->id_count >= 6) {
            if (dev->type1) {
                /* Verify (V) after a seek: the ID's cylinder must be the one
                   we asked for, otherwise S4 reports a seek error. */
                if (dev->id_buf[0] != (uint8_t) dev->target)
                    dev->status |= WD_S_SEEKERR;
                else
                    dev->status &= ~WD_S_SEEKERR;
                if ((dev->command & 0xf0) == 0x00)
                    dev->track_reg = 0x00;
                else if (dev->update)
                    dev->track_reg = (uint8_t) dev->target;
            }
            wd_done(dev);
        }
        return 0;
    }

    if (dev->state == WD_ST_READTRACK) {
        /* The drive layer walks the whole track and signals the end with
           last; a track is longer than one sector, so only last finishes. */
        wd_done(dev);
        return last ? -1 : 0;
    }

    if (last || (dev->byte_count >= dev->sector_size)) {
        dev->rec_done++;

        if (dev->multi) {
            dev->byte_count = 0;

            /* A real 2791 paces the chain with the data register: it only looks
               for the next record once the host has taken the last byte of this
               one.  Reading ahead turned a single-sector request into a run off
               the end of the track and a spurious record-not-found.

               A polled driver, however, never reads the data register just to
               let the chain advance -- it waits for DRQ, which the next record
               has to supply.  With the chain deferred such a driver and the
               controller wait for each other with BUSY still asserted, so the
               next record is searched for at once when no DMA channel is
               armed. */
            /* Both kinds of driver need the next record started here: a polled
               one because it waits for DRQ instead of making the extra data
               register read, a DMA one because the channel paces the transfer
               itself and would otherwise stall waiting for a record that is
               never searched for.  The command ends when the DMA channel
               reports terminal count. */
            if (!dev->tc_seen) {
                /* Data still queued for the host: the chain continues from
                   wd_buf_pop() once it has taken the last byte.  Only when the
                   queue is already empty (the host read ahead of the medium)
                   does the next record start here. */
                if (dev->buf_len > 0)
                    dev->chain_pending = 1;
                else
                    wd_chain_next(dev);
                return -1;
            }

            wd_done(dev);
            return -1;
            /* DRQ still reflects the byte queued behind the register. */
            return -1;
        }
        wd_done(dev);
        return -1;
    }

    return 0;
}

/* Called by the drive layer for every byte it needs while writing a sector. */
static int
wd_data_out(void *priv, int last)
{
    wd279x_t *dev = (wd279x_t *) priv;
    int       data;

    if (dev->state != WD_ST_WRITE)
        return 0xff;

    if (dev->dma) {
        data = dma_channel_read(dev->dma_ch);
        if (data == DMA_NODATA) {
            data = dev->data_reg;
        } else {
            if (data & DMA_OVER) {
                dev->tc  = 1;
                dev->tc_seen = 1;
            }
            data &= 0xff;
        }
    } else {
        data          = dev->data_reg;
        dev->drq      = 0;
    }

    dev->byte_count++;

    if (!last && (dev->byte_count < dev->sector_size))
        return data & 0xff;

    if (dev->multi && !dev->tc_seen) {
        dev->sector_reg = (uint8_t) (dev->sector_reg + 1);
        dev->byte_count = 0;
        wd_xfer_sync(dev, wd_drive(dev));
        fdd_writesector(wd_drive(dev), dev->sector_reg, dev->track_reg,
                        dev->side, dev->rate, wd_size_code(dev));
        return data & 0xff;
    }

    wd_done(dev);

    return data & 0xff;
}

/* ------------------------------------------------------------------------- */
/* Type I: seek family                                                       */
/* ------------------------------------------------------------------------- */

static void
wd_seek_finish(wd279x_t *dev)
{
    fdd_drive_t *drv = wd_drive(dev);

    if (drv != NULL) {
        if (fdd_current_track(drv) != dev->target) {
            if (dev->verify)
                dev->status |= WD_S_SEEKERR;
        } else {
            dev->status &= ~WD_S_SEEKERR;
        }
    }

    /* V (verify) is folded into the comparison above: the head has already
       been stepped to the target track, so the track the drive reports *is*
       the answer.  Reading the ID field here would mean waiting on an
       asynchronous Read Address that nothing completes for a Type I command,
       which leaves BUSY set and wedges the driver in its status poll. */

    if ((dev->command & 0xf0) == 0x00)
        dev->track_reg = 0x00;
    else if (dev->command & 0x10) /* T: update the track register */
        dev->track_reg = (uint8_t) dev->target;

    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[SEEK] cmd=%02X target=%d TR=%d drv_trk=%d\n", dev->command,
              dev->target, dev->track_reg,
              (wd_drive(dev) != NULL) ? fdd_current_track(wd_drive(dev)) : -1);

    dev->busy  = 0;
    wd_irq_raise(dev, "type1");
}


/* Record Not Found. */
static void
wd_rnf(wd279x_t *dev)
{
    dev->status |= WD_S_RNF;
}

static void
wd_type1(wd279x_t *dev, uint8_t val)
{
    fdd_drive_t *drv;
    int          target, diff;

    if ((val & 0x08) || (val & 0x04)) {
        /* h (head load) / V (verify) both need the head down. */
        if (wd_drive(dev) != NULL)
            fdd_set_motor_enable(wd_drive(dev), 1);
    }

    drv = wd_drive(dev);
    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        wd_rnf(dev);
        dev->track_reg = dev->track_reg;
        dev->busy      = 0;
        wd_irq_raise(dev, "nomedium");
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, 0);

    dev->verify = (val & 0x04) ? 1 : 0;
    dev->update = (val & 0x10) ? 1 : 0;

    switch (val & 0xf0) {
        case 0x00: /* Restore: step out until track 0 */
            target = 0;
            dev->step_dir = 1;
            break;
        case 0x10: /* Seek: target is in the data register */
            target = dev->data_reg;
            if (target < fdd_current_track(drv))
                dev->step_dir = 1;
            else
                dev->step_dir = 0;
            break;
        case 0x20:
        case 0x30: /* Step */
            target = fdd_current_track(drv) + (dev->step_dir ? -1 : 1);
            break;
        case 0x40:
        case 0x50: /* Step in */
            dev->step_dir = 0;
            target        = fdd_current_track(drv) + 1;
            break;
        default: /* Step out */
            dev->step_dir = 1;
            target        = fdd_current_track(drv) - 1;
            break;
    }

    if (target < 0)
        target = 0;

    dev->target = target;

    diff = target - fdd_current_track(drv);
    if (diff == 0) {
        /* Even when the head already sits on the target track the chip keeps
           BUSY up while it executes: a driver that polls the status right
           after writing the command byte has to see it, or it reads the
           vanished BUSY as "command not accepted" and fails the request (DOS
           3.1's disk helper does exactly that with its 25-pass loop). */
        dev->state = WD_ST_SEEK;
        timer_set_delay_u64(&dev->timer, (uint64_t) WD_INSTANT_CMD_US * TIMER_USEC);
        return;
    }

    dev->state = WD_ST_SEEK;
    fdd_seek(drv, diff);
}

/* ------------------------------------------------------------------------- */
/* Type II: read / write sector                                              */
/* ------------------------------------------------------------------------- */

static void
wd_start_read(wd279x_t *dev)
{
    fdd_drive_t *drv = wd_drive(dev);

    dev->chain_follow = 0;
    dev->rec_done     = 0;

    /* A command starts with an empty data path.  Bytes the guest never took
       from the previous command stayed queued and moved the new record one
       byte out of step, which is what made the SINIX POST read the volume
       label shifted by one. */
    dev->dr_full  = 0;
    dev->drq      = 0;
    dev->buf_len  = 0;
    dev->buf_head = dev->buf_tail = 0;

    if (drv == NULL)
        return;

    /* The density latch on this board does not describe the medium: the same
       value (0x05) boots an MFM DOS disk on the PC-D and an FM SINIX disk on
       the PC-X, and MAME boots both.  Take the encoding from the image -- that
       is what the head will actually find -- instead of from the latch, which
       only says what the last write put there. */
    if (drv->d86f_handler.side_flags != NULL) {
        /* Keep the latch's density.  The ROM's fallback path flips 0xf904
           bit 4 to single density right before it reads the SINIX boot record
           from track 0 sector 7, which only makes sense if the main path is
           MFM: on an FM-only medium that first read has to fail with no IDAM.
           Taking the encoding from the image instead made the read succeed and
           the fallback never ran. */
        
        /* The rate fields were last computed from the latch.  Setting the
           encoding here without recomputing them left the controller's bit
           cell period disagreeing with the medium's, and the drive layer
           refuses a command whose two bit cell periods differ -- the read
           died at position 0 without scanning a single cell. */
        wd279x_update_rate(dev);
    }

    dev->byte_count  = 0;
    dev->state       = WD_ST_READ;
    /* Whether the host moves the data with DMA or by polling is a property
       of the board, set before the command is issued. */
    dev->dma          = dev->dma_mode;
    dev->in_dma_fetch = 0;
    /* The record length comes from the ID the controller will match, not from
       the command: a Type II command carries no N, and the boot disk mixes
       128-byte sectors on cylinder 0 with 256-byte ones further in.  Look the
       sector up by C/H/R and size the transfer from what the track holds. */
    {
        int n = d86f_get_sector_size_code(drv, dev->side, dev->track_reg,
                                          dev->side, dev->sector_reg);
        if (n < 0)
            n = wd_sector_size_code(dev);
        dev->sector_size = 128 << n;
    }
    dev->was_first   = dev->first_read;
    dev->first_read  = 0;

    /* Start at the index hole.  The controller scans from there, and the
       estimate this used to compute from the track length landed further into
       the revolution than the driver is willing to wait: with a correct 100000
       cell track it put the head 78 ms in, while the PC-D read loop gives up
       after 28 ms and never saw a byte. */
    if (drv != NULL)
        d86f_set_track_pos(drv, 0);

    wd_sector_read(dev, dev->sector_reg);

    /* Two revolutions to find the record; wd_data_in() cancels this the moment
       the first byte arrives.  Without it a record that is never found leaves
       BUSY asserted for ever, and the host polls a status that never changes. */
    timer_set_delay_u64(&dev->timer, 2 * WD_RECORD_SEARCH_US * TIMER_USEC);

    if (drv != NULL)
        timer_set_delay_u64(&dev->byte_timer, wd_byte_period(drv));
}

static void
wd_start_write(wd279x_t *dev)
{
    fdd_drive_t *drv = wd_drive(dev);

    if (drv == NULL)
        return;

    if (drv->writeprot) {
        dev->status     |= WD_S_WP;
        dev->wp_latched  = 1;
        wd_done(dev);
        return;
    }

    dev->byte_count = 0;
    dev->state      = WD_ST_WRITE;

    /* Same board-level choice as a read. */
    dev->dma          = dev->dma_mode;
    dev->in_dma_fetch = 0;

    /* Same as a read: the length is whatever the ID says, not a default. */
    {
        int n = d86f_get_sector_size_code(drv, dev->side, dev->track_reg,
                                          dev->side, dev->sector_reg);
        if (n >= 0)
            dev->sector_size = 128 << n;
    }

    /* The 279x raises DRQ as soon as it wants the first data byte; a polled
       driver waits for exactly that bit before it writes 0xf903. */
    dev->drq        = 1;
    wd_xfer_sync(dev, drv);
    fdd_writesector(drv, wd_image_sector(dev, dev->sector_reg), dev->track_reg, dev->side,
                    dev->rate, wd_size_code(dev));
}

static void
wd_transfer_start(wd279x_t *dev)
{

    if ((dev->command & 0xf0) == 0x80 || (dev->command & 0xf0) == 0x90)
        wd_start_read(dev);
    else
        wd_start_write(dev);
}

static void
wd_rw_sector(wd279x_t *dev, uint8_t val)
{

    fdd_drive_t *drv = wd_drive(dev);

    dev->multi    = (val & 0x10) ? 1 : 0;
    dev->tc_seen  = 0;
    /* The side select on this board lives in the 0xf904 latch, not in the
       command byte: the BIOS issues a bare 0x94 for every read and flips the
       head through the latch.  Reading the command's bit 3 here would clear
       the head the latch just asked for. */
    if (!dev->side_latched)
        dev->side = (val & 0x08) ? 1 : 0;
    dev->settle   = (val & 0x04) ? 1 : 0;
    dev->compare  = (val & 0x02) ? 1 : 0;
    dev->deleted  = (val & 0x01) ? 1 : 0; /* a0 only on write */
    dev->sector_size = WD_SECTOR_SIZE;
    dev->byte_count  = 0;


    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        wd_rnf(dev);
        wd_done(dev);
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, dev->side);

    if (dev->settle) {
        dev->state = WD_ST_SETTLE;
        timer_set_delay_u64(&dev->timer, 15000 * TIMER_USEC);
        return;
    }

    wd_transfer_start(dev);
}

/* ------------------------------------------------------------------------- */
/* Type III: read address / read track / write track                          */
/* ------------------------------------------------------------------------- */

static void
wd_read_address(wd279x_t *dev, uint8_t val)
{
    fdd_drive_t *drv = wd_drive(dev);

    dev->settle = (val & 0x04) ? 1 : 0;

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        wd_rnf(dev);
        wd_done(dev);
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, dev->side);

    dev->id_count   = 0;
    dev->byte_count = 0;
    dev->state      = WD_ST_READID;
    wd_xfer_sync(dev, drv);
    fdd_readaddress(drv, dev->side, dev->rate);
}

static void
wd_read_track(wd279x_t *dev, uint8_t val)
{
    fdd_drive_t *drv = wd_drive(dev);

    dev->settle = (val & 0x04) ? 1 : 0;

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        wd_rnf(dev);
        wd_done(dev);
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, dev->side);

    dev->byte_count = 0;
    dev->state      = WD_ST_READTRACK;
    wd_xfer_sync(dev, drv);
    fdd_readsector(drv, SECTOR_FIRST, dev->track_reg, dev->side,
                   dev->rate, wd_size_code(dev));
}

/* ------------------------------------------------------------------------- */
/* Write Track                                                               */
/* ------------------------------------------------------------------------- */

static void wd_format_start_sector(wd279x_t *dev);

/* The host hands over the whole revolution as raw MFM bytes: gaps, sync
   words, address marks and CRCs included.  Rather than storing that stream
   untouched, the records are picked out of it and laid down through the same
   write path a Write Sector command uses, so the image ends up with exactly
   the layout the formatter asked for. */
static void
wd_format_parse(wd279x_t *dev)
{
    int i = 0;

    dev->fmt_count  = 0;
    dev->fmt_cur    = 0;
    dev->fmt_active = 0;

    while ((i + 8) <= dev->fmt_len) {
        if ((dev->fmt_buf[i] == 0xa1) && (dev->fmt_buf[i + 1] == 0xa1) &&
            (dev->fmt_buf[i + 2] == 0xa1) && (dev->fmt_buf[i + 3] == 0xfe)) {
            const uint8_t c   = dev->fmt_buf[i + 4];
            const uint8_t h   = dev->fmt_buf[i + 5];
            const uint8_t r   = dev->fmt_buf[i + 6];
            const uint8_t n   = dev->fmt_buf[i + 7];
            const int     len = 128 << (n & 3);
            int           j   = i + 8;

            /* Walk forward to this record's data address mark. */
            while ((j + 4) <= dev->fmt_len) {
                if ((dev->fmt_buf[j] == 0xa1) && (dev->fmt_buf[j + 1] == 0xa1) &&
                    (dev->fmt_buf[j + 2] == 0xa1)) {
                    if ((dev->fmt_buf[j + 3] == 0xfb) ||
                        (dev->fmt_buf[j + 3] == 0xf8))
                        break;
                    if (dev->fmt_buf[j + 3] == 0xfe)
                        break; /* next ID, this record has no data */
                }
                j++;
            }

            if (((j + 4 + len) <= dev->fmt_len) &&
                (dev->fmt_buf[j + 3] != 0xfe) &&
                (dev->fmt_count < WD_FMT_MAX_SECTORS)) {
                dev->fmt_sec[dev->fmt_count].c      = c;
                dev->fmt_sec[dev->fmt_count].h      = h;
                dev->fmt_sec[dev->fmt_count].r      = r;
                dev->fmt_sec[dev->fmt_count].n      = n;
                dev->fmt_sec[dev->fmt_count].offset = (uint16_t) (j + 4);
                dev->fmt_sec[dev->fmt_count].length = (uint16_t) len;
                dev->fmt_count++;

                i = j + 4 + len + 2; /* skip the data field and its CRC */
                continue;
            }
        }

        i++;
    }

    if (dev->fmt_count == 0) {
        wd_rnf(dev);
        wd_done(dev);
        return;
    }

    /* Lay out a track with the geometry the guest just described, then fill
       each record in turn.  The layout is what the drive layer reports back
       from wd_ctrl_sector_finishread(), so fmt_cur waits for that. */
    dev->fmt_n      = dev->fmt_sec[0].n;
    dev->fmt_cur    = -1;
    dev->fmt_id_idx = -1;

    /* Image formats (.img/.imd/...) have no formatter, and the callback that
       would end the command never comes; fail the way a bad track does. */
    if (wd_drive(dev)->fmt.format == NULL) {
        wd_rnf(dev);
        wd_done(dev);
        return;
    }

    dev->fmt_active = 1;
    dev->fmt_id_byte = 0;

    wd_xfer_sync(dev, wd_drive(dev));
    fdd_format(wd_drive(dev), dev->side, dev->rate, 0xe5);
}

static void
wd_format_start_sector(wd279x_t *dev)
{
    fdd_drive_t    *drv = wd_drive(dev);
    wd_fmt_sector_t *s;

    if (dev->fmt_cur >= dev->fmt_count) {
        dev->fmt_active = 0;
        wd_done(dev);
        return;
    }

    s = &dev->fmt_sec[dev->fmt_cur];

    dev->sector_reg = s->r;
    dev->byte_count = 0;
    dev->state      = WD_ST_WRITE;

    /* The next record's ID starts being collected from its first byte. */
    dev->fmt_id_byte = 0;

    /* Match the ID the format pass just laid down, not the track register:
       the guest may put another cylinder or head in the stream. */
    wd_xfer_sync(dev, drv);
    fdd_writesector(drv, s->r, s->c, s->h, dev->rate, s->n);
}

/* Put one raw byte into the revolution buffer.  DRQ stays up until the index
   hole comes round, which is what ends the capture. */
static void
wd_fmt_put(wd279x_t *dev, uint8_t val)
{
    if (dev->fmt_len < WD_FMT_BUF_SIZE)
        dev->fmt_buf[dev->fmt_len++] = val;

    if (dev->fmt_len >= dev->fmt_raw) {
        dev->drq = 0;
        wd_format_parse(dev);
        return;
    }

    dev->drq = dev->dma ? 0 : 1;
}

static void
wd_write_track(wd279x_t *dev, UNUSED(uint8_t val))
{
    fdd_drive_t *drv = wd_drive(dev);

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        wd_rnf(dev);
        wd_done(dev);
        return;
    }

    if (drv->writeprot) {
        dev->status    |= WD_S_WP;
        dev->wp_latched = 1;
        wd_done(dev);
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, dev->side);

    /* Same board-level choice as a read or a write. */
    dev->dma          = dev->dma_mode;
    dev->in_dma_fetch = 0;

    dev->fmt_len    = 0;
    dev->fmt_count  = 0;
    dev->fmt_cur    = 0;
    dev->fmt_active = 0;

    /* One revolution, converted from bit cells to bytes. */
    dev->fmt_raw = (int) (d86f_get_raw_size(drv->d86f_priv, dev->side) / 16);
    if (dev->fmt_raw <= 0)
        dev->fmt_raw = 6250;
    if (dev->fmt_raw > WD_FMT_BUF_SIZE)
        dev->fmt_raw = WD_FMT_BUF_SIZE;

    dev->state = WD_ST_WRITETRACK;
    /* As with Write Sector, DRQ comes up at once: a polled formatter waits
       for that bit before it hands over the first byte. */
    dev->drq   = 1;

    if (dev->dma)
        timer_set_delay_u64(&dev->byte_timer, wd_byte_period(drv));
}

/* ------------------------------------------------------------------------- */
/* Type IV: force interrupt                                                  */
/* ------------------------------------------------------------------------- */

static void
wd_force_interrupt(wd279x_t *dev, uint8_t val)
{
    uint8_t cond = val & 0x0f;

    /* The low nibble picks what ends the operation: i3 means "interrupt now",
       the others wait for ready/index (which this model does not follow), and
       zero terminates without raising INTRQ at all.  A command write already
       cleared the line, so 0xd0 must leave it low -- the PC-X POST issues
       exactly that pair, 0xd0 then 0xd8, and expects one interrupt from the
       second one (v3 BIOS 0xfcc20). */
    timer_disable(&dev->timer);

    /* Terminating a command has to reach the drive as well.  A Type II read
       that was left chaining into the next record keeps the bitstream engine
       spinning; it keeps searching, and the command it belongs to is gone, so
       the guest's next read starts on top of a record already in flight. */
    if (dev->busy) {
        fdd_drive_t *drv = wd_drive(dev);

        if (drv != NULL)
            fdd_stop(drv);

        /* A multi record read the host cuts short with Force Interrupt did
           not finish its last record; the chip reports Record Not Found.  The
           PC-X POST takes exactly that answer as "this is not a DOS boot
           sector" and falls back to the sector 7 check used by SINIX disks. */
        /* Only the probe read the POST issues right after selecting a drive is
           a failure when the host stops it: that is the "is this a DOS boot
           sector" test, and a Record Not Found is what sends the SINIX path to
           the volume label.  Every later read has the host's own byte count as
           its end, and reporting an error there aborts the loader. */
        if (dev->multi && dev->was_first && (dev->rec_done >= 2))
            wd_rnf(dev);

        dev->chain_pending = 0;
        dev->chain_follow  = 0;
        dev->done_pending  = 0;
        dev->multi         = 0;
        dev->buf_len       = 0;
        dev->buf_head      = 0;
        dev->buf_tail      = 0;
        dev->dr_full       = 0;
        /* No post-byte gap: this is an abort, not the end of a record. */
        dev->latched       = 1;
    }

    /* Force Interrupt is the one command that does not set BUSY.  If a command
       was executing it is terminated and the remaining status bits are left
       as they were; with nothing executing the status switches back to the
       Type I view, which is what a driver polling for a clean status expects. */
    if (!dev->busy) {
        dev->type1   = 1;
        dev->status &= ~(WD_S_RNF | WD_S_LOST);
    }

    dev->busy  = 0;
    dev->state = WD_ST_IDLE;
    dev->drq   = 0;

    if (cond & 0x08)
        wd_irq_raise(dev, "force");
}

/* ------------------------------------------------------------------------- */
/* registers                                                                 */
/* ------------------------------------------------------------------------- */

static void
wd279x_command(wd279x_t *dev, uint8_t val)
{
    /* A command written while the previous one is still running terminates it
       (WD279X-02 data sheet).  The drive's bitstream engine has to be stopped
       with it: left spinning it reaches the end of the record it was fetching
       and reports the completion through fdc_track_finishread(), which lands on
       whichever command is busy by then -- and that command ends with status 0
       and no data, looking like a successful read of nothing. */
    {
        fdd_drive_t *drv = wd_drive(dev);

        if (drv != NULL)
            fdd_stop(drv);
    }

    /* A new command abandons whatever the previous one left in the buffer. */
    dev->buf_head     = 0;
    dev->buf_tail     = 0;
    dev->buf_len      = 0;
    dev->dr_full      = 0;
    dev->done_pending  = 0;
    dev->chain_pending = 0;

    /* A new command abandons a Write Track that is still laying records
       down; the drive layer's callbacks find fmt_active clear and ignore
       them. */
    dev->fmt_active = 0;
    dev->fmt_cur    = -1;
    dev->fmt_id_idx = -1;

    /* A command register write clears INTRQ (die behaviour, see MAME
       cmd_w()). */
    wd_irq_lower(dev);
    dev->latched = 0;
    /* The write-protect latch belongs to the command that found it: it has to
       survive until the driver reads the status, but a later read command must
       not inherit it, or every read on a write-protected disk reports WP. */
    dev->wp_latched = 0;


    dev->command  = val;
    dev->busy    = 1;
    dev->status &= ~(WD_S_CRC | WD_S_RNF | WD_S_LOST);

    switch (val & 0xf0) {
        case 0x00:
        case 0x10:
        case 0x20:
        case 0x30:
        case 0x40:
        case 0x50:
        case 0x60:
        case 0x70:
            dev->type1 = 1;
            wd_type1(dev, val);
            break;

        case 0x80:
        case 0x90:
        case 0xa0:
        case 0xb0:
            dev->type1 = 0;
            wd_rw_sector(dev, val);
            break;

        case 0xc0:
            dev->type1 = 0;
            wd_read_address(dev, val);
            break;

        case 0xd0:
            dev->type1 = 0;
            wd_force_interrupt(dev, val);
            break;

        case 0xe0:
            dev->type1 = 0;
            wd_read_track(dev, val);
            break;

        default:
            dev->type1 = 0;
            wd_write_track(dev, val);
            break;
    }
}

static uint8_t
wd279x_reg_read(uint16_t port, void *priv)
{
    wd279x_t *dev = (wd279x_t *) priv;

    if (dev == NULL)
        return 0xff;


    switch ((port - dev->base) / (uint16_t) dev->reg_step) {
        case 0x00:
            /* Reading the status register drops INTRQ. */
            {
                uint8_t ret = wd_status(dev);

                wd_irq_lower(dev);
                return ret;
            }

        case 0x01:
            return dev->track_reg;

        case 0x02:
            return dev->sector_reg;

        default:
            /* The driver is polling the data register, so this transfer
               is PIO: leaving the DMA path armed as well fed the same
               record to two consumers and every byte after the first
               arrived shifted by one.  A read issued by the DMA engine
               itself comes through here too, so only a CPU read may
               switch the command over. */
            if (!dev->in_dma_fetch)
                dev->dma = 0;

            wd_buf_pop(dev);
            return dev->data_reg;
    }
}

static void
wd279x_reg_write(uint16_t port, uint8_t val, void *priv)
{
    wd279x_t *dev = (wd279x_t *) priv;

    if (dev == NULL)
        return;


    switch ((port - dev->base) / (uint16_t) dev->reg_step) {
        case 0x00:
            wd279x_command(dev, val);
            break;

        case 0x01:
            dev->track_reg = val;
            if (getenv("PCX_FDCLOG") != NULL)
                pclog("[REG] track <= %02X @%04X:%04X\n", val,
                      (uint16_t) cpu_state.temp_CS, (uint16_t) cpu_state.temp_pc);
            break;

        case 0x02:
            dev->sector_reg = val;
            if (getenv("PCX_FDCLOG") != NULL)
                pclog("[REG] sector <= %02X @%04X:%04X\n", val,
                      (uint16_t) cpu_state.temp_CS, (uint16_t) cpu_state.temp_pc);
            break;

        default:
            dev->data_reg = val;

            if (dev->state == WD_ST_WRITETRACK) {
                /* PIO Write Track: each byte the host hands over joins the
                   revolution buffer until the index hole comes round. */
                wd_fmt_put(dev, val);
            } else if (dev->state == WD_ST_WRITE) {
                /* PIO write: take the byte and ask for the next one.  When the
                   record is full, give the drive a moment to lay it down and
                   then end the command. */
                dev->byte_count++;

                /* The record length comes from the sector's ID, not from the
                   command's N: TDS writes 256-byte records while sending N=2.
                   The guest feeds bytes only as long as DRQ is up, so a short
                   gap with nothing arriving means the record is complete. */
                timer_set_delay_u64(&dev->timer,
                                    (uint64_t) 20000 * TIMER_USEC);

                if (dev->byte_count >= dev->sector_size) {
                    dev->drq   = 0;
                    dev->state = WD_ST_WRITE_END;
                    timer_set_delay_u64(&dev->timer,
                                        (uint64_t) WD_INSTANT_CMD_US * TIMER_USEC);
                }
            } else {
                dev->drq = 0;
            }
            break;
    }
}

/* ------------------------------------------------------------------------- */
/* timer                                                                     */
/* ------------------------------------------------------------------------- */

static void
wd279x_timer(void *priv)
{
    wd279x_t *dev = (wd279x_t *) priv;

    if (dev->state == WD_ST_SETTLE) {
        wd_transfer_start(dev);
        return;
    }

    if (dev->state == WD_ST_SEEK) {
        wd_seek_finish(dev);
        return;
    }

    if (dev->state == WD_ST_WRITE_END) {
        wd_done(dev);
        return;
    }

    if (dev->state == WD_ST_LATCH) {
        wd_finish(dev);
        return;
    }

    if (dev->state == WD_ST_WRITE) {
        /* One millisecond without a data byte: the record the drive engine
           wanted is complete, so drop BUSY and let the guest read the status. */
        dev->drq = 0;
        wd_done(dev);
        return;
    }

    if (dev->state == WD_ST_READ) {
        /* The next record never turned up.  Only say so when nothing of that
           record arrived at all: a host that has already taken the bytes it
           asked for stops reading and lets the controller sit on the last
           DRQ, exactly as the WD279x does (MAME keeps BUSY/DRQ there and
           waits for Force Interrupt), so a queued byte means the search
           succeeded and this bound must not turn it into an error. */
        if ((dev->byte_count > 0) || (dev->buf_len > 0))
            return;

        wd_rnf(dev);
        wd_done(dev);
        return;
    }
}

/* ------------------------------------------------------------------------- */
/* controller callbacks (installed into fdd_ctrl_t)                          */
/* ------------------------------------------------------------------------- */

static void
wd_noidam(void *priv)
{
    wd279x_t *dev = (wd279x_t *) priv;


    /* A Type I command has no data phase, so a "no address mark" report that
       arrives while one is running belongs to a stale sector operation.  Any
       other command that is still busy is waiting for exactly this answer --
       keying off the state instead left an FM scan that gave up while the
       controller sat in another state with BUSY stuck for ever. */
    if ((dev->type1) || (!dev->busy))
        return;

    /* A multi record read that has already transferred at least one sector
       ends the operation when the next sector is not on the track; only a
       command that found nothing at all reports Record Not Found. */
    if (dev->chain_follow) {
        dev->chain_follow = 0;
        wd_done(dev);
        return;
    }

    wd_rnf(dev);
    wd_done(dev);
}

/* Every error the drive layer reports for a data-phase command.  The generic
   path rewrites fdc->stat, which this controller never looks at, so without
   this hook a chained sector that fails left the command busy for ever. */
/* The record length of a read comes from the ID the controller matched, not
   from the density: an FM track can perfectly well hold 512 byte sectors, and
   assuming 128 for every FM sector cut the transfer short on the SINIX boot
   disk. */
static void
wd_ext_sectorid(void *priv, UNUSED(uint8_t c), UNUSED(uint8_t h),
                UNUSED(uint8_t r), uint8_t n)
{
    wd279x_t *dev = (wd279x_t *) priv;

    if (n > 7)
        return;

    dev->sector_size = 128 << n;
}

static void
wd_ext_error(void *priv, int st5, int st6)
{
    wd279x_t *dev = (wd279x_t *) priv;

    if ((dev->type1) || (!dev->busy))
        return;

    /* A multi record read that already transferred a whole sector and then
       found no such record after it has finished its job: the WD279x only
       reports Record Not Found when nothing was transferred.  The SINIX POST
       asks for sector 1 with m=1, took a bare RNF here and retried the boot
       for ever. */
    if (dev->chain_follow) {
        dev->chain_follow = 0;
        wd_done(dev);
        return;
    }

    /* fdc_error() carries the uPD765 numbering: 0x01 is "no ID address mark"
       and 0x04 is "ID marks found but not the sector asked for".  Both mean
       RNF to this chip, and leaving 0x04 unmapped made a read of a missing
       record finish with status 0 -- a successful transfer of nothing, which
       the guest then treated as a loaded boot area. */
    if (st5 & 0x01)
        wd_rnf(dev);
    if (st5 & 0x04)
        wd_rnf(dev);
    if (st5 & 0x10)
        dev->status |= WD_S_CRC;
    if (st6 & 0x20)
        dev->status |= WD_S_CRC;

    wd_done(dev);
}

static void
wd_seek_complete(void *priv, UNUSED(int drive))
{
    wd279x_t *dev = (wd279x_t *) priv;

    if (dev->state != WD_ST_SEEK)
        return;

    wd_seek_finish(dev);
}

static void
wd_diskchange(UNUSED(void *priv), UNUSED(int drive))
{
}

/* ------------------------------------------------------------------------- */
/* external interface                                                        */
/* ------------------------------------------------------------------------- */

static void
wd279x_reset_dev(wd279x_t *dev)
{
    if (dev == NULL)
        return;

    dev->buf_head     = 0;
    dev->buf_tail     = 0;
    dev->buf_len      = 0;
    dev->dr_full      = 0;
    dev->done_pending  = 0;
    dev->chain_pending = 0;

    timer_disable(&dev->timer);

    dev->command    = 0x00;
    dev->status     = 0x00;
    dev->wp_latched = 0;
    dev->track_reg  = 0x00;
    dev->sector_reg = 0x01; /* MR leaves the sector register at 1 (data book) */
    dev->data_reg   = 0x00;
    dev->dskctl     = 0x00;

    dev->state     = WD_ST_IDLE;
    dev->latched   = 0;
    dev->post_us   = 100;
    dev->type1     = 1;
    dev->busy      = 0;
    dev->drq       = 0;
    dev->intrq     = 0;
    dev->drive     = 0;
    dev->sector_base = 1;
    dev->side      = 0;
    dev->dden      = 0;
    dev->multi     = 0;
    dev->settle    = 0;
    dev->verify    = 0;
    dev->update    = 0;
    dev->compare   = 0;
    dev->deleted   = 0;
    dev->step_dir  = 1;
    dev->target    = 0;
    dev->byte_count = 0;
    dev->id_count   = 0;

    dev->sector_size = WD_SECTOR_SIZE;

    /* The rate is part of the controller state: leaving it at zero told
       d86f_export this was a 500 kbps medium, and the exported image then
       described tracks twice as long as the ones the drive actually turns. */
    wd279x_update_rate(dev);
}

void
wd279x_reset(void)
{
    wd279x_reset_dev(wd279x_cur);
}

void
wd279x_set_dskctl(wd279x_t *dev, uint8_t val)
{
    fdd_drive_t *drv;

    if (dev == NULL)
        return;

    dev->dskctl = val;

    dev->side_latched = 1;
    /* The drive select is one hot: the v3 boot loader writes 0x05 for drive A
       and 0x06 for drive B (0xfde1c: 1 << drive, or 0x04 for the motor), so
       reading the pair as a binary number picked drive B whenever A was
       asked for and every boot read ended in Record Not Found.  The :YES
       firmware also walks all four bits (0x21, 0x22, 0x24, 0x28 about equally
       often, two internal drives plus the two external sockets), which is why
       bit 3 is a fourth drive select and not a density select. */
    int         prev_drive = dev->drive;

    if (dev->dskctl_drive_onehot) {
        if (val & 0x08)
            dev->drive = 3;
        else if (val & 0x04)
            dev->drive = 2;
        else if (val & 0x02)
            dev->drive = 1;
        else if (val & 0x01)
            dev->drive = 0;
    } else
        dev->drive = (val & 0x01) ? 0 : ((val & 0x02) ? 1 : 0);

    dev->side = (val >> dev->dskctl_side_bit) & 1;
    dev->dden = (dev->dskctl_dden_bit < 0) ? 0 : ((val >> dev->dskctl_dden_bit) & 1);

    /* A fresh selection starts a fresh probe: the POST reads the boot sector
       of the drive it just picked and expects that one to be the read that can
       fail. */
    if (prev_drive != dev->drive)
        dev->first_read = 1;

    /* The latch cannot select the density for these machines: the same value
       (0x05) boots an MFM disk on the PC-D and an FM disk on the PC-X.  For
       bitstream media the encoding belongs to the image, and handing the latch
       to the drive layer changed the synthesised track length under a running
       scan -- the same track then measured 50000 cells on one command and
       100000 on the next.  Only controllers without a bitstream handler still
       need the latch. */
    drv = wd_drive(dev);
    if ((drv == NULL) || (drv->d86f_handler.side_flags == NULL))
        fdd_set_densel(0, dev->dden);
    wd279x_update_rate(dev);

    for (int i = 0; i < 2; i++) {
        void *d = dev->fdd[i];

        if (d == NULL)
            continue;

        /* Only the selected unit spins; bit 2 is the motor enable. */
        fdd_set_motor_enable(d, ((dev->drive == i) && (val & 0x04)) ? 1 : 0);
    }

    drv = wd_drive(dev);
    if (drv != NULL)
        fdd_set_head(drv, dev->side);
}

void
wd279x_set_reg_step(wd279x_t *dev, int step)
{
    if ((dev == NULL) || (step <= 0) || (step == dev->reg_step))
        return;

    /* Replace the window registered at init instead of adding a second one.
       The port layer appends handlers, so leaving the old range behind would
       make every write run twice on the overlapping ports. */
    io_removehandler(dev->base, 4 * dev->reg_step, wd279x_reg_read, NULL, NULL,
                     wd279x_reg_write, NULL, NULL, dev);

    dev->reg_step = step;

    io_sethandler(dev->base, 4 * dev->reg_step, wd279x_reg_read, NULL, NULL,
                  wd279x_reg_write, NULL, NULL, dev);
}

void
wd279x_set_sector_base(wd279x_t *dev, int base)
{
    if (dev != NULL)
        dev->sector_base = base;
}

uint8_t
wd279x_get_dskctl(wd279x_t *dev)
{
    return (dev != NULL) ? dev->dskctl : 0x00;
}

/* ------------------------------------------------------------------------- */
/* Controller interface                                                      */
/* ------------------------------------------------------------------------- */

static int
wd_ctrl_is_pcjx(UNUSED(void *priv))
{
    return 0;
}

static int
wd_ctrl_real_drive(UNUSED(void *priv), int drive)
{
    return drive & 3;
}

static int
wd_ctrl_get_bit_rate(void *priv)
{
    return ((wd279x_t *) priv)->rate;
}

static uint8_t
wd_ctrl_get_swwp(UNUSED(void *priv))
{
    return 0;
}

static int
wd_ctrl_get_bitcell_period(void *priv)
{
    return (int) ((wd279x_t *) priv)->bitcell_period;
}

static int
wd_ctrl_is_mfm(void *priv)
{
    return ((wd279x_t *) priv)->mfm;
}

static uint8_t
wd_ctrl_get_diswr(UNUSED(void *priv))
{
    return 0;
}

static int
wd_ctrl_get_gap(UNUSED(void *priv))
{
    return 0;
}

static int
wd_ctrl_get_gap2(UNUSED(void *priv), UNUSED(int drive))
{
    return 0;
}

static int
wd_ctrl_get_pcn(void *priv, UNUSED(int side))
{
    return ((wd279x_t *) priv)->track_reg;
}

static int
wd_ctrl_is_ibm5140(UNUSED(void *priv))
{
    return 0;
}

static int
wd_ctrl_is_interface_gated(UNUSED(void *priv))
{
    return 0;
}

static int
wd_ctrl_data(void *priv, uint8_t data, int last)
{
    return wd_data_in(priv, data, last);
}

static int
wd_ctrl_getdata(void *priv, int last)
{
    wd279x_t *dev = (wd279x_t *) priv;

    /* While a track is being laid out the drive layer collects each record's
       four ID bytes from here.  Hand it the ID parsed out of the guest's
       stream -- the same one fdd_writesector() is later given -- so the
       address marks on the medium match the data fields.  The data itself is
       filled through fdd_writesector(), never through this hook. */
    if (dev->fmt_active) {
        wd_fmt_sector_t *s;
        int              i = dev->fmt_cur;

        if (i < 0)
            i = 0;
        if (i >= dev->fmt_count)
            i = dev->fmt_count - 1;
        if (i < 0)
            return 0xff;

        s = &dev->fmt_sec[i];

        switch (dev->fmt_id_byte++) {
            case 0:
                return s->c;
            case 1:
                return s->h;
            case 2:
                return s->r;
            default:
                return s->n;
        }
    }

    return wd_data_out(priv, last);
}

static int
wd_ctrl_data_available(UNUSED(const void *priv))
{
    return 0;
}

/* Write Track writes one record at a time; the drive layer says when the
   current one is on the medium and the next record takes over. */
static void
wd_ctrl_sector_finishread(void *priv)
{
    wd279x_t *dev = (wd279x_t *) priv;

    if (!dev->fmt_active)
        return;

    dev->fmt_cur++;

    wd_format_start_sector(dev);
}

static void
wd_ctrl_sector_finishcompare(UNUSED(void *priv), UNUSED(int satisfying))
{
}

static void
wd_ctrl_track_finishread(UNUSED(void *priv), UNUSED(int condition))
{
}

static void
wd_ctrl_noidam(void *priv)
{
    wd_noidam(priv);
}

static void
wd_ctrl_nosector(void *priv)
{
    wd_ext_error(priv, 4, 0);
}

static void
wd_ctrl_nodataam(void *priv)
{
    wd_ext_error(priv, 1, 1);
}

static void
wd_ctrl_cannotformat(void *priv)
{
    wd279x_t *dev = (wd279x_t *) priv;

    if (dev->fmt_active) {
        /* The medium will not take the layout: end the command with RNF
           instead of waiting for a write-back that will never start. */
        dev->fmt_active = 0;
        wd_rnf(dev);
        wd_done(dev);
        return;
    }

    wd_ext_error(priv, 0, 0);
}

static void
wd_ctrl_datacrcerror(void *priv)
{
    wd_ext_error(priv, 0x20, 0x20);
}

static void
wd_ctrl_headercrcerror(void *priv)
{
    wd_ext_error(priv, 0x20, 0);
}

static void
wd_ctrl_writeprotect(void *priv)
{
    wd279x_t *dev = (wd279x_t *) priv;

    if (dev->fmt_active) {
        /* Write Track on a protected medium: S6 set and the command ends. */
        dev->fmt_active = 0;
        dev->wp_latched  = 1;
        dev->status     |= WD_S_WP;
        wd_done(dev);
        return;
    }

    wd_ext_error(priv, 0x02, 0);
}

static void
wd_ctrl_wrongcylinder(void *priv)
{
    wd_ext_error(priv, 0x04, 0x10);
}

static void
wd_ctrl_badcylinder(void *priv)
{
    wd_ext_error(priv, 4, 0x02);
}

static void
wd_ctrl_set_wrong_am(UNUSED(void *priv))
{
}

static void
wd_ctrl_sectorid(void *priv, uint8_t track, uint8_t side, uint8_t sector,
                 uint8_t size, UNUSED(uint8_t crc1), UNUSED(uint8_t crc2))
{
    wd_ext_sectorid(priv, track, side, sector, size);
}

static fdd_sector_id_t
wd_ctrl_get_read_track_sector(UNUSED(void *priv))
{
    fdd_sector_id_t ret = { 0 };

    return ret;
}

static void
wd_ctrl_request_next_sector_id(void *priv)
{
    wd279x_t *dev = (wd279x_t *) priv;

    /* The formatter asks for the ID of each record as it reaches it. */
    if (dev->fmt_active && (dev->fmt_id_idx < dev->fmt_count))
        dev->fmt_id_idx++;
}

static void
wd_ctrl_stop_id_request(UNUSED(void *priv))
{
}

static void
wd_ctrl_seek_complete_interrupt(void *priv, int drive)
{
    wd_seek_complete(priv, drive);
}

static void
wd_ctrl_diskchange_interrupt(void *priv, int drive)
{
    wd_diskchange(priv, drive);
}

static const fdd_ctrl_t wd279x_ctrl = {
    .is_pcjx                 = wd_ctrl_is_pcjx,
    .real_drive              = wd_ctrl_real_drive,
    .get_swwp                = wd_ctrl_get_swwp,
    .get_bit_rate            = wd_ctrl_get_bit_rate,
    .get_bitcell_period      = wd_ctrl_get_bitcell_period,
    .is_mfm                  = wd_ctrl_is_mfm,
    .get_diswr               = wd_ctrl_get_diswr,
    .get_gap                 = wd_ctrl_get_gap,
    .get_gap2                = wd_ctrl_get_gap2,
    .get_pcn                 = wd_ctrl_get_pcn,
    .is_ibm5140              = wd_ctrl_is_ibm5140,
    .is_interface_gated      = wd_ctrl_is_interface_gated,

    .data                    = wd_ctrl_data,
    .getdata                 = wd_ctrl_getdata,
    .data_available          = wd_ctrl_data_available,

    .sector_finishread       = wd_ctrl_sector_finishread,
    .sector_finishcompare    = wd_ctrl_sector_finishcompare,
    .track_finishread        = wd_ctrl_track_finishread,
    .noidam                  = wd_ctrl_noidam,
    .nosector                = wd_ctrl_nosector,
    .nodataam                = wd_ctrl_nodataam,
    .cannotformat            = wd_ctrl_cannotformat,
    .datacrcerror            = wd_ctrl_datacrcerror,
    .headercrcerror          = wd_ctrl_headercrcerror,
    .writeprotect            = wd_ctrl_writeprotect,
    .wrongcylinder           = wd_ctrl_wrongcylinder,
    .badcylinder             = wd_ctrl_badcylinder,
    .set_wrong_am            = wd_ctrl_set_wrong_am,
    .sectorid                = wd_ctrl_sectorid,
    .get_read_track_sector   = wd_ctrl_get_read_track_sector,
    .request_next_sector_id  = wd_ctrl_request_next_sector_id,
    .stop_id_request         = wd_ctrl_stop_id_request,

    .seek_complete_interrupt = wd_ctrl_seek_complete_interrupt,
    .diskchange_interrupt    = wd_ctrl_diskchange_interrupt
};

wd279x_t *
wd279x_init_at(uint16_t base)
{
    wd279x_t *dev;
    int       i;

    dev = (wd279x_t *) calloc(1, sizeof(wd279x_t));
    if (dev == NULL)
        return NULL;

    /* The PC-X mapping until the board says otherwise. */
    dev->dskctl_side_bit     = 3;
    dev->dskctl_drive_onehot = 0;
    dev->dskctl_dden_bit     = 4;

    /* Bind the four drives to this controller. The drive layer reports back
       through wd279x_ctrl, so no fdc_t is involved.
       TODO: take a drive offset instead of assuming drives[0] once a machine
       can instantiate more than one controller. */
    for (i = 0; i < 4; i++) {
        dev->fdd[i]         = &(drives[i]);
        drives[i].ctrl      = &wd279x_ctrl;
        drives[i].ctrl_priv = dev;
    }

    dev->dma      = 1;
    dev->dma_mode = 0;
    dev->dma_ch   = WD279X_DMA;

    /* The WD279x has no DTL register: when the ID says the sector is 128 bytes
       the transfer is exactly one sector. fdd_86f works the length out as
       128 << N, but only when DTL is not smaller than 128, so leave it there
       instead of the zero a reset leaves behind. */
    dev->dtl = 128;

    timer_add(&dev->timer, wd279x_timer, dev, 0);
    timer_add(&dev->byte_timer, wd_byte_tick, dev, 0);

    dev->base     = base;
    dev->reg_step = 1;
    io_sethandler(base, 4 * dev->reg_step, wd279x_reg_read, NULL, NULL,
                  wd279x_reg_write, NULL, NULL, dev);

    wd279x_cur = dev;
    wd279x_reset_dev(dev);

    return dev;
}

/* Boards differ in how the drive/side/density latch is wired; the defaults are
   the mapping this driver shipped with (head on bit 3, density on bit 4). */
void
wd279x_set_dskctl_bits(wd279x_t *dev, int side_bit, int drive_onehot, int dden_bit)
{
    if (dev == NULL)
        return;

    dev->dskctl_side_bit     = ((side_bit >= 0) && (side_bit <= 7)) ? side_bit : 3;
    dev->dskctl_drive_onehot = drive_onehot ? 1 : 0;
    dev->dskctl_dden_bit     = (dden_bit <= 7) ? dden_bit : -1;
}

wd279x_t *
wd279x_init(void)
{
    /* The PC-X board's formatter sits at 0xf900. */
    return wd279x_init_at(0xf900);
}

void
wd279x_set_irq_handler(wd279x_t *dev, void (*cb)(void *priv, int level),
                       void *priv)
{
    if (dev == NULL)
        return;

    dev->irq_cb   = cb;
    dev->irq_priv = priv;
}

static void
wd279x_close_dev(wd279x_t *dev)
{
    if (dev == NULL)
        return;

    io_removehandler(dev->base, 4 * dev->reg_step, wd279x_reg_read, NULL, NULL,
                     wd279x_reg_write, NULL, NULL, dev);

    timer_disable(&dev->timer);

    for (int i = 0; i < 4; i++) {
        drives[i].ctrl      = NULL;
        drives[i].ctrl_priv = NULL;
        dev->fdd[i]         = NULL;
    }

    free(dev);

    if (wd279x_cur == dev)
        wd279x_cur = NULL;
}

void
wd279x_close(void)
{
    wd279x_close_dev(wd279x_cur);
}

/* ------------------------------------------------------------------------- */
/* Device model                                                              */
/* ------------------------------------------------------------------------- */

static void *
wd279x_device_init(const device_t *info)
{
    (void) info;

    return wd279x_init();
}

static void
wd279x_device_close(void *priv)
{
    wd279x_close_dev((wd279x_t *) priv);
}

static void
wd279x_device_reset(void *priv)
{
    wd279x_reset_dev((wd279x_t *) priv);
}

/* TODO: the device path still uses the PC-X defaults -- base 0xf900 and the
   board latch bits set in the init above. A board that adds it through
   device_add_params() should pass its own base and DSKCTL mapping. */
const device_t wd279x_device = {
    .name          = "Western Digital WD279x FDC",
    .internal_name = "wd279x",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = wd279x_device_init,
    .close         = wd279x_device_close,
    .reset         = wd279x_device_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
