/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through very early
 *          IBM PC compatible systems (prior to the PC720).
 *
 *          This program is free software; you can redistribute it and/or
 *          modify it under the terms of the GNU General Public License
 *          as published by the Free Software Foundation; either version 2
 *          of the License, or (at your option) any later version.
 *
 *          This program is distributed in the hope that it will be useful,
 *          but WITHOUT ANY WARRANTY; without even the implied warranty of
 *          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *          GNU General Public License for more details.
 *
 *          Siemens PC-X: Western Digital WD2791 floppy disk formatter.
 *
 *          Command set and status layout follow the WD279X-02 data book
 *          (TABLE 1 / TABLE 4) and the decode MAME uses for the same chip in
 *          its wd_fdc device; the medium is the 86Box drive layer, which is
 *          already sector based, so none of the flux/PLL machinery is needed
 *          here.  A WD2793 behaves identically (its data bus is not inverted),
 *          and the PC-X board carries the WD2791 variant.
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/plat.h>
#include <86box/mem.h>

#include <86box/plat_unused.h>
#include <86box/timer.h>
#include <86box/pic.h>
#include <86box/dma.h>
#include <86box/fdd.h>
#include "cpu.h"
#include <86box/fdd_86f.h>
#include <86box/fdc.h>
#include <86box/io.h>
#include <86box/log.h>
#include <86box/wd2791.h>


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

static wd2791_t *wd2791_cur = NULL;

static void wd2791_command(wd2791_t *dev, uint8_t val);
static void wd2791_timer(void *priv);

/* The real chip walks the rest of the track looking for the next ID before it
   gives up.  A chained sector past the end of the track never shows up, and a
   polled driver never asserts DMA terminal count either, so without this bound
   the command would sit in READ with BUSY set for ever.  One revolution at
   300 rpm is 200 ms. */
#define WD_RECORD_SEARCH_US (200 * 1000)

/* Experiment hook: the driver gives up on a read well before the fixed
   full-revolution wait this model uses for the record search. */
static uint64_t
wd_search_us(void)
{
    static uint64_t v;
    static int      init;

    if (!init) {
        const char *e = getenv("PCX_SEARCH_US");

        v    = e ? (uint64_t) strtoull(e, NULL, 0) : WD_RECORD_SEARCH_US;
        init = 1;
    }
    return v;
}

/* Shortest time a command stays BUSY.  Long enough for a polled driver to see
   it, short enough not to slow anything down. */
#define WD_INSTANT_CMD_US   (200)

/* ------------------------------------------------------------------------- */
/* helpers                                                                   */
/* ------------------------------------------------------------------------- */

static fdd_drive_t *
wd_drive(wd2791_t *dev)
{
    void *drv = dev->fdc.fdd[dev->drive & 3];

    return (fdd_drive_t *) drv;
}


/* The bitcell period the drive layer samples with comes from the data rate the
   controller has selected, and the WD279x only has the one DDEN pin to say it.
   Leaving fdc.bitcell_period at zero made the bitstream layer search every
   sector at a zero-length bit cell, which is why every read ended in Record
   Not Found even with the right drive, track and sector. */
static int
wd2791_rate(wd2791_t *dev)
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
wd2791_update_rate(wd2791_t *dev)
{
    dev->fdc.rate  = (uint8_t) wd2791_rate(dev);
    dev->fdc.mfm   = dev->dden ? 0 : 1;
    /* FM frames one data bit per two bit cells: at the same 250 kbps nominal
       rate a single density track is written at 125 kbps, so its cell lasts
       twice as long. The bitstream layer derives the medium's cell from the
       track's own flags (16000 ns for an FM/250 kbps track) and refuses the
       command outright when the controller disagrees, which is what made every
       FM read come back as NO IDAM without ever searching the track. */
    dev->fdc.bitcell_period = (uint32_t) fdd_get_bitcell_period(dev->fdc.rate) *
                              (dev->dden ? 2 : 1);
}

static uint8_t wd_status(wd2791_t *dev);

static void
wd_irq_raise(wd2791_t *dev, const char *why)
{
    if (getenv("PCX_FDCIQ") != NULL) {
        static int n;
        if (n < 400) {
            n++;
            pclog("[FDCIQ] t=%u why=%s cmd=%02X st=%02X state=%d busy=%d intrq=%d bc=%d @%04X:%04X\n",
                  plat_get_ticks(), why, dev->command, wd_status(dev), dev->state, dev->busy,
                  dev->intrq, dev->byte_count, cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
    }
    if (dev->intrq && (getenv("PCX_FDCIQ") != NULL))
        pclog("[FDCIQ] DOUBLE raise why=%s cmd=%02X t=%u @%04X:%04X\n", why,
              dev->command, plat_get_ticks(), cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
    dev->intrq = 1;
    picint(1 << WD2791_IRQ);
}

static void
wd_irq_lower(wd2791_t *dev)
{
    dev->intrq = 0;
    picintc(1 << WD2791_IRQ);
}

/* Bytes the host actually took out of the data register for the command now
   running; [DONE] prints it so "found the sector" and "delivered the sector"
   cannot be confused again. */
static int wd_dbg_pops = 0;
static int wd_dbg_why  = 0;
static uint32_t wd_dbg_cmd_t = 0;

/* Finish the command: drop busy, latch the interrupt and let the registers
   read back.  Every path out of the state machine ends here. */
static void
wd_finish(wd2791_t *dev)
{
    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[DONE] t=%u cmd=%02X bc=%d pops=%d state=%d st=%02X why=%d trk=%d sec=%d side=%d\n",
              plat_get_ticks(), dev->command, dev->byte_count, wd_dbg_pops, dev->state,
              dev->status, wd_dbg_why, dev->track_reg, dev->sector_reg, dev->side);
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
wd_done(wd2791_t *dev)
{
    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[WDONE] bc=%d size=%d buf=%d dr=%d state=%d\n", dev->byte_count,
              dev->sector_size, dev->buf_len, dev->dr_full, dev->state);
    /* Hold the command open until the host has read the bytes still queued
       behind the data register, or a polling driver would see BUSY drop in the
       middle of the sector. */
    if ((dev->buf_len > 0) || dev->dr_full) {
        dev->done_pending = 1;
        return;
    }

    wd_finish(dev);
}


/* The drive layer wants the sector size as the N code of the ID field (0 = 128
   bytes, 2 = 512 bytes), not a byte count; passing 512 there set N to 0 and no
   sector on the medium ever matched. */
static int
wd_size_code(wd2791_t *dev)
{
    int n = 0;

    while ((n < 7) && ((128 << n) < dev->sector_size))
        n++;

    return n;
}

static uint8_t
wd_status(wd2791_t *dev)
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

        if (getenv("PCX_FDCLOG") != NULL && (ret & WD_S_DRQ) &&
            !dev->dr_full && (dev->buf_len <= 0)) {
            static unsigned gh;
            if ((gh++ % 20) == 0)
                pclog("[DRQGHOST] n=%u bc=%d drq=%d full=%d len=%d state=%d\n",
                      gh, dev->byte_count, dev->drq, dev->dr_full,
                      dev->buf_len, dev->state);
        }
    }

    if (dev->busy)
        ret |= WD_S_BUSY;

    return ret;
}

/* ------------------------------------------------------------------------- */
/* Type II/III data path                                                     */
/* ------------------------------------------------------------------------- */

static int
wd_buf_push(wd2791_t *dev, uint8_t data)
{
    if (dev->buf_len >= (int) sizeof(dev->buf))
        return 0;

    dev->buf[dev->buf_head] = data;
    dev->buf_head           = (dev->buf_head + 1) % (int) sizeof(dev->buf);
    dev->buf_len++;

    if (getenv("PCX_FDCLOG") != NULL && (dev->byte_count <= 8))
        pclog("[Q] push %02X head=%d tail=%d len=%d bc=%d\n",
              data, dev->buf_head, dev->buf_tail, dev->buf_len, dev->byte_count);

    return 1;
}

/* A Read Sector command never carries the sector's size: the chip takes the N
   field out of the ID it just matched. Single density is the one case where the
   medium fixes it -- FM on 5.25" media always uses 128-byte sectors -- while a
   double density track may be 256, 512 or 1024 depending on how it was written,
   so there the command's own size code stays the best guess. */
static int
wd_sector_size_code(wd2791_t *dev)
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
static void
wd_sector_read(wd2791_t *dev, int sector)
{
    fdd_drive_t *drv = wd_drive(dev);

    if (drv == NULL)
        return;

    fdd_readsector(drv, sector, dev->track_reg, dev->side, dev->fdc.rate,
                   wd_sector_size_code(dev));
}

/* Chained sectors: the controller looks for the next record once this one is
   over.  On the real chip that moment is the end of the data field, and the
   host still has the last byte to take, so the search starts when the data
   register is drained -- arming it earlier lets the whole chain race ahead of
   the driver. */
static void
wd_chain_next(wd2791_t *dev)
{
    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[CHAIN] next=%d bc=%d len=%d size=%d busy=%d\n",
              dev->sector_reg + 1, dev->byte_count, dev->buf_len,
              dev->sector_size, dev->busy);
    dev->chain_follow = 1;
    dev->sector_reg = (uint8_t) (dev->sector_reg + 1);
    timer_set_delay_u64(&dev->timer, wd_search_us() * TIMER_USEC);
    wd_sector_read(dev, dev->sector_reg);
}

/* One byte time has passed.  The chip moves the next byte of the record into
   the data register and raises DRQ; if the host has not taken the previous
   one, that byte is lost and S2 says so (WD279X-02, Read Sector). */
static void
wd_byte_tick(void *priv)
{
    wd2791_t    *dev = (wd2791_t *) priv;
    fdd_drive_t *drv = wd_drive(dev);

    if ((dev->state != WD_ST_READ) && (dev->state != WD_ST_READID) &&
        (dev->state != WD_ST_READTRACK))
        return;

    /* The byte timer does not move bytes any more.  Two places promoting a
       byte into the data register consumed them out of order: the medium
       delivered "VOL1S", the guest received "VL1SN" and the SINIX POST
       rejected the disk.  wd_buf_pop() is the only reader of the queue. */
    timer_set_delay_u64(&dev->byte_timer,
                        (drv != NULL) ? fdd_get_byte_period(drv) : (32ULL * TIMER_USEC));
}

static int
wd_buf_pop(wd2791_t *dev)
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

        if (getenv("PCX_FDCLOG") != NULL && (dev->byte_count <= 8))
            pclog("[Q] fill %02X head=%d tail=%d len=%d bc=%d\n",
                  dev->dr, dev->buf_head, dev->buf_tail, dev->buf_len,
                  dev->byte_count);
    }

    if (getenv("PCX_FDCLOG") != NULL) {
        static unsigned seq;
        static unsigned lim = 400;

        if (dev->byte_count <= 8)
            pclog("[POP] seq=%u data=%02X bc=%d state=%d\n", seq, dev->dr,
                  dev->byte_count, dev->state);
        seq++;
    }

    dev->data_reg = dev->dr;
    dev->dr_full  = 0;
    dev->drq      = 0;
    wd_dbg_pops++;

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
            wd_dbg_why = __LINE__; wd_finish(dev);
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
    wd2791_t *dev = (wd2791_t *) priv;
    int       res;

    /* A record starts with an empty data register.  Since the medium only
       queues bytes now, whatever the register held belongs to the previous
       record and handing it to the host shifted this one by a byte. */
    if ((dev->byte_count == 0) && !last) {
        dev->dr_full = 0;
        dev->drq     = 0;
    }

    if (getenv("PCX_DATADUMP") != NULL) {
        static int  n;
        static char line[160];
        int         k = n % 16;

        snprintf(&line[k * 3], 4, "%02X ", data);
        n++;
        if ((n % 16) == 0)
            pclog("[DATA] %s st=%d\n", line, dev->state);
    }

    if ((dev->state != WD_ST_READ) && (dev->state != WD_ST_READID) &&
        (dev->state != WD_ST_READTRACK)) {
        if (getenv("PCX_FDCLOG") != NULL && (dev->byte_count < 8))
            pclog("[DROP] data=%02X last=%d state=%d bc=%d\n",
                  data, last, dev->state, dev->byte_count);
        return -1;
    }

    timer_disable(&dev->timer);



    if ((dev->state == WD_ST_READID) && (dev->id_count < (int) sizeof(dev->id_buf)))
        dev->id_buf[dev->id_count] = data;

    if (dev->fdc.dma) {
        /* Put the byte where the hardware keeps it while DRQ is asserted: the
           data register.  The DMA controller fetches it through the source
           pointer (an I/O read of 0xF903), so without this the channel would
           read back whatever the port returned on its own. */
        dev->dr      = data;
        dev->dr_full = 1;
        dev->drq     = 1;
        res = dma_channel_write(dev->fdc.dma_ch, data);
        if (res != DMA_NODATA) {
            dev->dr_full = 0;
            dev->drq     = 0;
        }
        if (getenv("PCX_DMADBG") != NULL) {
            static int n;
            if (n++ < 24)
                pclog("[DMADBG] ch=%d data=%02X res=%d\n", dev->fdc.dma_ch, data, res);
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
                wd_dbg_why = __LINE__; wd_done(dev);
                return -1;
            }
        } else if (res & DMA_OVER) {
            dev->fdc.tc = 1;
            dev->tc_seen = 1;
        }
    } else {
        /* One entry for the byte stream: the record buffer.  Letting the
           medium write the data register directly while the buffer held an
           older byte delivered them out of order -- the SINIX POST received
           "VL1SN" instead of "VOL1S". */
        if (!wd_buf_push(dev, data)) {
            dev->status |= WD_S_LOST;
            wd_dbg_why = __LINE__; wd_done(dev);
            return -1;
        }
        dev->drq = 1;
    }

    if (getenv("PCX_FDCLOG") != NULL && (dev->byte_count < 6))
        pclog("[BYTES-IN] n=%d data=%02X last=%d drfull=%d len=%d\n",
              dev->byte_count, data, last, dev->dr_full, dev->buf_len);

    dev->byte_count++;
    dev->id_count++;

    if ((dev->byte_count == 1) && (getenv("PCX_FDCLOG") != NULL))
        pclog("[WD1ST] cmd=%02X dt=%u ms pos=%u state=%d last=%d\n", dev->command,
              plat_get_ticks() - wd_dbg_cmd_t, d86f_get_track_pos(wd_drive(dev)),
              dev->state, last);

    if (getenv("PCX_DATALOG") != NULL &&
        (((dev->byte_count & 63) == 0) || (dev->byte_count <= 8)))
        pclog("[WDIN] bc=%d last=%d size=%d state=%d dma=%d pos=%u\n", dev->byte_count,
              last, dev->sector_size, dev->state, dev->fdc.dma,
              d86f_get_track_pos(wd_drive(dev)));

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
            wd_dbg_why = __LINE__; wd_done(dev);
        }
        return 0;
    }

    if (dev->state == WD_ST_READTRACK) {
        /* The drive layer walks the whole track and signals the end with
           last; a track is longer than one sector, so only last finishes. */
        if (last)
            wd_dbg_why = __LINE__; wd_done(dev);
        return last ? -1 : 0;
    }

    if (getenv("PCX_FDCLOG") != NULL && (last || (dev->byte_count >= dev->sector_size)))
        pclog("[LASTB] bc=%d size=%d last=%d multi=%d tc=%d chainp=%d donep=%d len=%d\n",
              dev->byte_count, dev->sector_size, last, dev->multi, dev->tc_seen,
              dev->chain_pending, dev->done_pending, dev->buf_len);

    if (last || (dev->byte_count >= dev->sector_size)) {
        {
            static int cl2 = -1;
            if (cl2 < 0)
                cl2 = getenv("PCX_CMDN") ? 1 : 0;
            if (cl2)
                pclog("[CMDN] C=%d H=%d R=%d 交付=%d size=%d last=%d\n",
                      dev->track_reg, dev->side, dev->sector_reg,
                      dev->byte_count, dev->sector_size, last);
        }
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

            wd_dbg_why = __LINE__; wd_done(dev);
            return -1;
            /* DRQ still reflects the byte queued behind the register. */
            return -1;
        }
        wd_dbg_why = __LINE__; wd_done(dev);
        return -1;
    }

    return 0;
}

/* Called by the drive layer for every byte it needs while writing a sector. */
static int
wd_data_out(void *priv, int last)
{
    wd2791_t *dev = (wd2791_t *) priv;
    int       data;

    if (dev->state != WD_ST_WRITE)
        return 0xff;

    if (dev->fdc.dma) {
        data = dma_channel_read(dev->fdc.dma_ch);
        if (data == DMA_NODATA) {
            data = dev->data_reg;
        } else {
            if (data & DMA_OVER)
                dev->fdc.tc = 1;
            data &= 0xff;
        }
    } else {
        data          = dev->data_reg;
        dev->drq      = 0;
    }

    dev->byte_count++;

    if (!last && (dev->byte_count < dev->sector_size))
        return data & 0xff;

    if (dev->multi) {
        dev->sector_reg = (uint8_t) (dev->sector_reg + 1);
        dev->byte_count = 0;
        fdd_writesector(wd_drive(dev), dev->sector_reg, dev->track_reg,
                        dev->side, dev->fdc.rate, dev->sector_size);
        return data & 0xff;
    }

    wd_dbg_why = __LINE__; wd_done(dev);

    return data & 0xff;
}

/* ------------------------------------------------------------------------- */
/* Type I: seek family                                                       */
/* ------------------------------------------------------------------------- */

static void
wd_seek_finish(wd2791_t *dev)
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

    dev->busy  = 0;
    wd_irq_raise(dev, "type1");
}


/* Record Not Found, with the site that raised it: eight places set it and the
   boot path depends on which one did. */
static void
wd_rnf(wd2791_t *dev, int line)
{
    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[RNFSET] line=%d cmd=%02X state=%d bc=%d busy=%d chain=%d\n",
              line, dev->command, dev->state, dev->byte_count, dev->busy,
              dev->chain_follow);
    dev->status |= WD_S_RNF;
}

static void
wd_type1(wd2791_t *dev, uint8_t val)
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
        wd_rnf(dev, __LINE__);
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
wd_start_read(wd2791_t *dev)
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
        int mfm = ((drv->d86f_handler.side_flags(drv) & 0x18) == 0x08);

        if (getenv("PCX_FDCLOG") != NULL)
            pclog("[WDENC] flags=%04X mfm=%d side=%d head=%d\n",
                  drv->d86f_handler.side_flags(drv), mfm, dev->side,
                  fdd_get_head(drv));

        dev->fdc.mfm = mfm;
        dev->dden    = mfm ? 0 : 1;
        /* The rate fields were last computed from the latch.  Setting the
           encoding here without recomputing them left the controller's bit
           cell period disagreeing with the medium's, and the drive layer
           refuses a command whose two bit cell periods differ -- the read
           died at position 0 without scanning a single cell. */
        wd2791_update_rate(dev);
    }

    dev->byte_count  = 0;
    dev->state       = WD_ST_READ;
    /* The record length the command asks for follows N (or the medium, in
       single density); the transfer is that long, not a fixed 512 bytes. */
    dev->sector_size = 128 << wd_sector_size_code(dev);
    dev->was_first   = dev->first_read;
    dev->first_read  = 0;

    {
        static int fl = -1;

        if (fl < 0)
            fl = (getenv("PCX_FDCLOG") != NULL);
        if (fl)
            pclog("[FDC] read drv=%d C=%d H=%d R=%d drv_track=%d pos=%u raw=%u rate=%d cmd=%02X size=%d n=%d from %04X:%04X dst=%04X:%04X dden=%d\n",
                  dev->drive, dev->track_reg, dev->side, dev->sector_reg, fdd_current_track(drv),
                  d86f_get_track_pos(drv), d86f_get_raw_size(drv, dev->side),
                  dev->fdc.rate, dev->command, dev->sector_size, wd_sector_size_code(dev),
                  CS, cpu_state.pc, ES, DI, dev->dden);
    }

    /* Start at the index hole.  The controller scans from there, and the
       estimate this used to compute from the track length landed further into
       the revolution than the driver is willing to wait: with a correct 100000
       cell track it put the head 78 ms in, while the PC-D read loop gives up
       after 28 ms and never saw a byte. */
    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[SEEKPOS] pos=%u (left where the drive is)\n",
              d86f_get_track_pos(drv));

    wd_sector_read(dev, dev->sector_reg);

    /* Two revolutions to find the record; wd_data_in() cancels this the moment
       the first byte arrives.  Without it a record that is never found leaves
       BUSY asserted for ever, and the host polls a status that never changes. */
    timer_set_delay_u64(&dev->timer, 2 * wd_search_us() * TIMER_USEC);

    if (drv != NULL)
        timer_set_delay_u64(&dev->byte_timer, fdd_get_byte_period(drv));
}

static void
wd_start_write(wd2791_t *dev)
{
    fdd_drive_t *drv = wd_drive(dev);

    if (drv == NULL)
        return;

    if (drv->writeprot) {
        dev->status     |= WD_S_WP;
        dev->wp_latched  = 1;
        wd_dbg_why = __LINE__; wd_done(dev);
        return;
    }

    dev->byte_count = 0;
    dev->state      = WD_ST_WRITE;
    /* The 279x raises DRQ as soon as it wants the first data byte; a polled
       driver waits for exactly that bit before it writes 0xf903. */
    dev->drq        = 1;
    fdd_writesector(drv, dev->sector_reg, dev->track_reg, dev->side,
                    dev->fdc.rate, wd_size_code(dev));
}

static void
wd_transfer_start(wd2791_t *dev)
{

    if ((dev->command & 0xf0) == 0x80 || (dev->command & 0xf0) == 0x90)
        wd_start_read(dev);
    else
        wd_start_write(dev);
}

static void
wd_rw_sector(wd2791_t *dev, uint8_t val)
{

    fdd_drive_t *drv = wd_drive(dev);

    if (((val & 0xf0) >= 0xa0) && (getenv("PCX_FDCLOG") != NULL))
        pclog("[WR] cmd=%02X C=%d H=%d R=%d N=%d from %04X:%04X\n", val,
              dev->track_reg, dev->side, dev->sector_reg, wd_size_code(dev), CS, cpu_state.pc);

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

    dev->fdc.deleted = dev->deleted ? 3 : 0;

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        wd_rnf(dev, __LINE__);
        wd_dbg_why = __LINE__; wd_done(dev);
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
wd_read_address(wd2791_t *dev, uint8_t val)
{
    fdd_drive_t *drv = wd_drive(dev);

    dev->settle = (val & 0x04) ? 1 : 0;

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        wd_rnf(dev, __LINE__);
        wd_dbg_why = __LINE__; wd_done(dev);
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, dev->side);

    dev->id_count   = 0;
    dev->byte_count = 0;
    dev->state      = WD_ST_READID;
    fdd_readaddress(drv, dev->side, dev->fdc.rate);
}

static void
wd_read_track(wd2791_t *dev, uint8_t val)
{
    fdd_drive_t *drv = wd_drive(dev);

    dev->settle = (val & 0x04) ? 1 : 0;

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        wd_rnf(dev, __LINE__);
        wd_dbg_why = __LINE__; wd_done(dev);
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, dev->side);

    dev->byte_count = 0;
    dev->state      = WD_ST_READTRACK;
    fdd_readsector(drv, SECTOR_FIRST, dev->track_reg, dev->side,
                   dev->fdc.rate, wd_size_code(dev));
}

static void
wd_write_track(wd2791_t *dev, UNUSED(uint8_t val))
{
    /* Formatting needs the MFM encoder (A1 sync words, gap and CRC layout).
       Until that is written the command completes without touching the
       medium so a formatter that probes the controller does not hang. */
    wd_dbg_why = __LINE__; wd_done(dev);
}

/* ------------------------------------------------------------------------- */
/* Type IV: force interrupt                                                  */
/* ------------------------------------------------------------------------- */

static void
wd_force_interrupt(wd2791_t *dev, uint8_t val)
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

        if (getenv("PCX_FDCLOG") != NULL)
            pclog("[ABORT] cmd=%02X state=%d bc=%d buf=%d chain=%d multi=%d\n",
                  dev->command, dev->state, dev->byte_count, dev->buf_len,
                  dev->chain_pending, dev->multi);

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
            wd_rnf(dev, __LINE__);

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
wd2791_command(wd2791_t *dev, uint8_t val)
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

    /* A command register write clears INTRQ (die behaviour, see MAME
       cmd_w()). */
    wd_irq_lower(dev);
    dev->latched = 0;
    /* The write-protect latch belongs to the command that found it: it has to
       survive until the driver reads the status, but a later read command must
       not inherit it, or every read on a write-protected disk reports WP. */
    dev->wp_latched = 0;

    dev->command  = val;
    wd_dbg_pops   = 0;
    wd_dbg_cmd_t  = plat_get_ticks();
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
wd2791_reg_read(uint16_t port, UNUSED(void *priv))
{
    wd2791_t *dev = wd2791_cur;

    if (dev == NULL)
        return 0xff;


    switch (port & 3) {
        case 0x00:
            /* Reading the status register drops INTRQ. */
            {
                uint8_t ret = wd_status(dev);

                {
                    static int  fl = -1;
                    static int  last = -1;
                    static long n = 0;

                    if (fl < 0)
                        fl = (getenv("PCX_FDCLOG") != NULL);
                    n++;
                    if (fl && (ret != last)) {
                        pclog("[STS] %02X n=%ld state=%d intrq=%d\n", ret, n, dev->state, dev->intrq);
                        last = ret;
                    }
                }

                {
                    static int st = -1;
                    static long sn;
                    if (st < 0)
                        st = (getenv("PCX_FDCSTAT") != NULL);
                    if (st && (sn < 400)) {
                        sn++;
                        pclog("[STAT] n=%ld %02X t=%u @%04X:%04X cmd=%02X state=%d intrq=%d busy=%d drq=%d\n",
                              sn,
                              ret, plat_get_ticks(), cpu_state.seg_cs.seg,
                              (uint16_t) cpu_state.pc, dev->command, dev->state,
                              dev->intrq, dev->busy, dev->drq);
                    }
                }

                wd_irq_lower(dev);
                return ret;
            }

        case 0x01:
            return dev->track_reg;

        case 0x02:
            return dev->sector_reg;

        default:

            {
                static int l = -1;
                int        r;

                if (l < 0)
                    l = getenv("PCX_DATALOG") ? 1 : 0;
                /* The driver is polling the data register, so this transfer
                   is PIO: leaving the DMA path armed as well fed the same
                   record to two consumers and every byte after the first
                   arrived shifted by one. */
                dev->fdc.dma = 0;

                r = wd_buf_pop(dev);
                if (l) {
                    static int once;
                    if (!once) {
                        unsigned k;
                        uint16_t sss = (uint16_t) cpu_state.seg_ss.seg;
                        uint16_t ssp = (uint16_t) cpu_state.regs[4].w;
                        uint16_t bpp = (uint16_t) cpu_state.regs[5].w;

                        once = 1;
                        pclog("[DATAFRAME] sp=%04X bp=%04X\n", ssp, bpp);
                        for (k = 0; k < 8; k++) {
                            uint32_t a = ((uint32_t) sss << 4) + ((ssp + k * 2) & 0xffff);
                            pclog("[DATAFRAME] sp+%02X = %04X\n", k * 2,
                                  (uint16_t) readmembl(a) | (uint16_t) (readmembl(a + 1) << 8));
                        }
                        for (k = 0; k < 8; k++) {
                            uint32_t a = ((uint32_t) sss << 4) + ((bpp + k * 2) & 0xffff);
                            pclog("[DATAFRAME] bp+%02X = %04X\n", k * 2,
                                  (uint16_t) readmembl(a) | (uint16_t) (readmembl(a + 1) << 8));
                        }
                    }
                }
                if (l)
                    pclog("[DATARD] pop=%d data=%02X st=%02X drq=%d full=%d len=%d bc=%d ES:DI=%04X:%04X @%04X:%04X\n", r,
                          dev->data_reg, dev->status, dev->drq, dev->dr_full,
                          dev->buf_len, dev->byte_count,
                          (uint16_t) cpu_state.seg_es.seg,
                          (uint16_t) cpu_state.regs[7].w, (uint16_t) cpu_state.temp_CS,
                          (uint16_t) cpu_state.temp_pc);
                (void) r;
            }
            return dev->data_reg;
    }
}

static void
wd2791_reg_write(uint16_t port, uint8_t val, UNUSED(void *priv))
{
    wd2791_t *dev = wd2791_cur;

    if (dev == NULL)
        return;


    switch (port & 3) {
        case 0x00:
            if (getenv("PCX_FDCLOG") != NULL)
                pclog("[CMD] t=%u %02X C=%d H=%d R=%d N=%d @%04X:%04X\n", plat_get_ticks(), val, dev->track_reg, dev->side,
                      dev->sector_reg, wd_size_code(dev),
                      (uint16_t) cpu_state.temp_CS, (uint16_t) cpu_state.temp_pc);
            wd2791_command(dev, val);
            break;

        case 0x01:
            dev->track_reg = val;
            break;

        case 0x02:
            dev->sector_reg = val;
            break;

        default:
            dev->data_reg = val;

            if (dev->state == WD_ST_WRITE) {
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
wd2791_timer(void *priv)
{
    wd2791_t *dev = (wd2791_t *) priv;

    if (dev->state == WD_ST_SETTLE) {
        wd_transfer_start(dev);
        return;
    }

    if (dev->state == WD_ST_SEEK) {
        wd_seek_finish(dev);
        return;
    }

    if (dev->state == WD_ST_WRITE_END) {
        wd_dbg_why = __LINE__; wd_done(dev);
        return;
    }

    if (dev->state == WD_ST_LATCH) {
        wd_dbg_why = __LINE__; wd_finish(dev);
        return;
    }

    if (dev->state == WD_ST_WRITE) {
        /* One millisecond without a data byte: the record the drive engine
           wanted is complete, so drop BUSY and let the guest read the status. */
        if (getenv("PCX_FDCLOG"))
            pclog("[WRIDLE] bytes=%d\n", dev->byte_count);
        dev->drq = 0;
        wd_dbg_why = __LINE__; wd_done(dev);
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

        if (getenv("PCX_FDCLOG") != NULL)
            pclog("[TIMEOUT] sec=%d bc=%d buf=%d\n", dev->sector_reg, dev->byte_count, dev->buf_len);
        wd_rnf(dev, __LINE__);
        wd_dbg_why = __LINE__; wd_done(dev);
        return;
    }
}

/* ------------------------------------------------------------------------- */
/* controller callbacks (installed into fdc_t)                               */
/* ------------------------------------------------------------------------- */

static void
wd_noidam(void *priv)
{
    wd2791_t *dev = (wd2791_t *) priv;


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
        if (getenv("PCX_FDCLOG") != NULL)
            pclog("[CHAINEND] sec=%d bc=%d\n", dev->sector_reg, dev->byte_count);
        dev->chain_follow = 0;
        wd_done(dev);
        return;
    }

    wd_rnf(dev, __LINE__);
    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[RNFAT] cmd=%02X sec=%d state=%d pos=%u raw=%u bc=%d chain=%d type1=%d drive=%d\n",
              dev->command, dev->sector_reg, dev->state,
              d86f_get_track_pos(wd_drive(dev)),
              d86f_get_raw_size(wd_drive(dev), dev->side),
              dev->byte_count, dev->chain_follow, dev->type1, dev->drive);
    wd_dbg_why = __LINE__; wd_done(dev);
}

/* Every error the drive layer reports for a data-phase command.  The generic
   path rewrites fdc->stat, which this controller never looks at, so without
   this hook a chained sector that fails left the command busy for ever. */
/* The record length of a read comes from the ID the controller matched, not
   from the density: an FM track can perfectly well hold 512 byte sectors, and
   assuming 128 for every FM sector cut the transfer short on the SINIX boot
   disk. */
static void
wd_ext_sectorid(void *priv, uint8_t c, uint8_t h,
                uint8_t r, uint8_t n)
{
    wd2791_t *dev = (wd2791_t *) priv;

    if (n > 7)
        return;

    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[WDSID] id %d/%d/%d/%d -> size=%d\n", c, h, r, n, 128 << n);

    dev->sector_size = 128 << n;
}

static void
wd_ext_error(void *priv, int st5, int st6)
{
    wd2791_t *dev = (wd2791_t *) priv;

    if ((dev->type1) || (!dev->busy))
        return;

    if (getenv("PCX_FDCLOG") != NULL)
        pclog("[EXTERR] st5=%02X st6=%02X chain=%d bc=%d sec=%d\n",
              st5, st6, dev->chain_follow, dev->byte_count, dev->sector_reg);

    /* A multi record read that already transferred a whole sector and then
       found no such record after it has finished its job: the WD279x only
       reports Record Not Found when nothing was transferred.  The SINIX POST
       asks for sector 1 with m=1, took a bare RNF here and retried the boot
       for ever. */
    if (dev->chain_follow) {
        if (getenv("PCX_FDCLOG") != NULL)
            pclog("[CHAINEND] sec=%d bc=%d\n", dev->sector_reg, dev->byte_count);
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
        wd_rnf(dev, __LINE__);
    if (st5 & 0x04)
        wd_rnf(dev, __LINE__);
    if (st5 & 0x10)
        dev->status |= WD_S_CRC;
    if (st6 & 0x20)
        dev->status |= WD_S_CRC;

    wd_dbg_why = __LINE__; wd_done(dev);
}

static void
wd_seek_complete(void *priv, UNUSED(int drive))
{
    wd2791_t *dev = (wd2791_t *) priv;

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

void
wd2791_reset(void)
{
    wd2791_t *dev = wd2791_cur;

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
    {
        const char *e = getenv("PCX_POST_US");
        dev->post_us  = e ? atoi(e) : 100;
    }
    dev->type1     = 1;
    dev->busy      = 0;
    dev->drq       = 0;
    dev->intrq     = 0;
    dev->drive     = 0;
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
}

void
wd2791_set_dskctl(wd2791_t *dev, uint8_t val)
{
    fdd_drive_t *drv;

    if (dev == NULL)
        return;

    dev->dskctl = val;

    dev->side_latched = 1;
    /* The drive select is one hot: the v3 boot loader writes 0x05 for drive A
       and 0x06 for drive B (0xfde1c: 1 << drive, or 0x04 for the motor), so
       reading the pair as a binary number picked drive B whenever A was
       asked for and every boot read ended in Record Not Found. */
    int         prev_drive = dev->drive;

    dev->drive  = (val & 0x01) ? 0 : ((val & 0x02) ? 1 : 0);
    dev->side   = (val & 0x08) ? 1 : 0;
    /* The 0xf904 latch selects single density when bit 4 is set: the POST
       reads the boot sector with the latch at 0x05, which has to be MFM on a
       360k medium. */
    dev->dden   = (val & 0x10) ? 1 : 0;

    /* A fresh selection starts a fresh probe: the POST reads the boot sector
       of the drive it just picked and expects that one to be the read that can
       fail. */
    if (prev_drive != dev->drive)
        dev->first_read = 1;

    if (getenv("PCX_DSKLOG") != NULL)
        pclog("[DSKSET] val=%02X -> drive=%d side=%d dden=%d\n", val,
              dev->drive, dev->side, dev->dden);

    if ((prev_drive != dev->drive) && (getenv("PCX_DSKLOG") != NULL)) {
        fdd_drive_t *nd = wd_drive(dev);
        pclog("[DRVSEL] %d -> %d med=%d pos=%u trk=%d\n", prev_drive,
              dev->drive, (nd != NULL) ? fdd_can_read_medium(nd) : 0,
              (nd != NULL) ? d86f_get_track_pos(nd) : 0,
              (dev->track_reg & 0xFF));
    }

    /* The latch cannot select the density for these machines: the same value
       (0x05) boots an MFM disk on the PC-D and an FM disk on the PC-X.  For
       bitstream media the encoding belongs to the image, and handing the latch
       to the drive layer changed the synthesised track length under a running
       scan -- the same track then measured 50000 cells on one command and
       100000 on the next.  Only controllers without a bitstream handler still
       need the latch. */
    drv = wd_drive(dev);
    if ((drv == NULL) || (drv->d86f_handler.side_flags == NULL))
        fdd_set_densel(dev->fdc.bus, dev->dden);
    wd2791_update_rate(dev);

    for (int i = 0; i < 2; i++) {
        void *d = dev->fdc.fdd[i];

        if (d == NULL)
            continue;

        /* Only the selected unit spins; bit 2 is the motor enable. */
        fdd_set_motor_enable(d, ((dev->drive == i) && (val & 0x04)) ? 1 : 0);
    }

    drv = wd_drive(dev);
    if (drv != NULL)
        fdd_set_head(drv, dev->side);
}

uint8_t
wd2791_get_dskctl(wd2791_t *dev)
{
    return (dev != NULL) ? dev->dskctl : 0x00;
}

wd2791_t *
wd2791_init(void)
{
    wd2791_t *dev;
    int       i;

    dev = (wd2791_t *) calloc(1, sizeof(wd2791_t));
    if (dev == NULL)
        return NULL;

    /* The drive layer hands a fdc_t * back to us, so bind the four drives the
       same way fdc_init() does and keep the controller state in the struct
       that embeds it. */
    for (i = 0; i < 4; i++) {
        dev->fdc.fdd[i]          = &(drives[i]);
        drives[i].fdc            = &dev->fdc;
    }

    dev->fdc.irq    = WD2791_IRQ;
    dev->fdc.dma    = 1;
    dev->fdc.dma_ch = WD2791_DMA;

    dev->fdc.bus    = 0;

    /* The WD279x has no DTL register: when the ID says the sector is 128 bytes
       the transfer is exactly one sector. fdd_86f works the length out as
       128 << N, but only when DTL is not smaller than 128, so leave it there
       instead of the zero a reset leaves behind. */
    dev->fdc.dtl = 128;

    dev->fdc.ext          = dev;
    dev->fdc.ext_data     = wd_data_in;
    dev->fdc.ext_getdata  = wd_data_out;
    dev->fdc.ext_noidam   = wd_noidam;
    dev->fdc.ext_sectorid = wd_ext_sectorid;
    dev->fdc.ext_error    = wd_ext_error;
    dev->fdc.ext_seek_done = wd_seek_complete;
    dev->fdc.ext_diskchange = wd_diskchange;

    timer_add(&dev->timer, wd2791_timer, dev, 0);
    timer_add(&dev->byte_timer, wd_byte_tick, dev, 0);

    io_sethandler(0xf900, 4, wd2791_reg_read, NULL, NULL,
                  wd2791_reg_write, NULL, NULL, dev);

    wd2791_cur = dev;
    wd2791_reset();

    return dev;
}

void
wd2791_close(void)
{
    wd2791_t *dev = wd2791_cur;

    if (dev == NULL)
        return;

    io_removehandler(0xf900, 4, wd2791_reg_read, NULL, NULL,
                     wd2791_reg_write, NULL, NULL, dev);

    timer_disable(&dev->timer);

    for (int i = 0; i < 4; i++) {
        drives[i].fdc = NULL;
        dev->fdc.fdd[i] = NULL;
    }

    free(dev);
    wd2791_cur = NULL;
}
