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
#include <86box/plat_unused.h>
#include <86box/timer.h>
#include <86box/pic.h>
#include <86box/dma.h>
#include <86box/fdd.h>
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

/* ------------------------------------------------------------------------- */
/* helpers                                                                   */
/* ------------------------------------------------------------------------- */

static fdd_drive_t *
wd_drive(wd2791_t *dev)
{
    void *drv = dev->fdc.fdd[dev->drive & 3];

    return (fdd_drive_t *) drv;
}

static void
wd_irq_raise(wd2791_t *dev)
{
    dev->intrq = 1;
    picint(1 << WD2791_IRQ);
}

static void
wd_irq_lower(wd2791_t *dev)
{
    dev->intrq = 0;
    picintc(1 << WD2791_IRQ);
}

/* Finish the command: drop busy, latch the interrupt and let the registers
   read back.  Every path out of the state machine ends here. */
static void
wd_done(wd2791_t *dev)
{
    dev->busy  = 0;
    dev->state = WD_ST_IDLE;
    dev->drq   = 0;
    wd_irq_raise(dev);
}

static uint8_t
wd_status(wd2791_t *dev)
{
    fdd_drive_t *drv = wd_drive(dev);
    uint8_t      ret = dev->status & (WD_S_CRC | WD_S_RNF | WD_S_WP | WD_S_TRK0 | WD_S_LOST);

    if (dev->busy)
        ret |= WD_S_BUSY;

    if (dev->type1) {
        /* Type I reports the mechanical state instead of the transfer state. */
        if (!drv || !fdd_can_read_medium(drv))
            ret |= WD_S_NOTRDY;
        if (drv && fdd_track0(drv))
            ret |= WD_S_TRK0;
        if (drv && fdd_index(drv))
            ret |= WD_S_INDEX;
        ret |= WD_S_HLD;
    } else {
        if (dev->deleted)
            ret |= WD_S_RECTYPE;
        if (dev->drq)
            ret |= WD_S_DRQ;
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/* Type II/III data path                                                     */
/* ------------------------------------------------------------------------- */

/* Called by the drive layer for every byte it takes off the medium.  Returns
   0 to keep the transfer going and -1 when the controller no longer wants
   bytes (which is how the drive layer learns that the sector is over). */
static int
wd_data_in(void *priv, uint8_t data, int last)
{
    wd2791_t *dev = (wd2791_t *) priv;
    int       res;

    if ((dev->state != WD_ST_READ) && (dev->state != WD_ST_READID) &&
        (dev->state != WD_ST_READTRACK))
        return -1;

    if ((dev->state == WD_ST_READID) && (dev->id_count < (int) sizeof(dev->id_buf)))
        dev->id_buf[dev->id_count] = data;

    if (dev->fdc.dma) {
        res = dma_channel_write(dev->fdc.dma_ch, data);
        if (res == DMA_NODATA) {
            /* The guest has not armed the DMA channel: fall back to the data
               register so a polling driver still gets its bytes. */
            if (dev->drq) {
                dev->status |= WD_S_LOST;
                wd_done(dev);
                return -1;
            }
            dev->data_reg = data;
            dev->drq      = 1;
        } else if (res & DMA_OVER) {
            dev->fdc.tc = 1;
        }
    } else {
        if (dev->drq) {
            dev->status |= WD_S_LOST;
            wd_done(dev);
            return -1;
        }
        dev->data_reg = data;
        dev->drq      = 1;
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
        if (last)
            wd_done(dev);
        return last ? -1 : 0;
    }

    if (last || (dev->byte_count >= dev->sector_size)) {
        if (dev->multi) {
            dev->multi      = 1;
            dev->sector_reg = (uint8_t) (dev->sector_reg + 1);
            dev->byte_count = 0;
            dev->drq        = 0;
            fdd_readsector(wd_drive(dev), dev->sector_reg, dev->track_reg,
                           dev->side, WD_DENSITY, dev->sector_size);
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
                        dev->side, WD_DENSITY, dev->sector_size);
        return data & 0xff;
    }

    wd_done(dev);

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

    if (dev->verify && ((dev->status & WD_S_SEEKERR) == 0)) {
        /* V verifies the target track by reading its first ID field. */
        if (drv != NULL) {
            fdd_set_head(drv, 0);
            fdd_readaddress(drv, 0, WD_DENSITY);
            dev->state       = WD_ST_READID;
            dev->id_count    = 0;
            dev->byte_count  = 0;
            return;
        }
    }

    if ((dev->command & 0xf0) == 0x00)
        dev->track_reg = 0x00;
    else if (dev->command & 0x10) /* T: update the track register */
        dev->track_reg = (uint8_t) dev->target;

    dev->busy  = 0;
    wd_irq_raise(dev);
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
        dev->status |= WD_S_RNF;
        dev->track_reg = dev->track_reg;
        dev->busy      = 0;
        wd_irq_raise(dev);
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
        wd_seek_finish(dev);
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

    if (drv == NULL)
        return;

    dev->byte_count = 0;
    dev->state      = WD_ST_READ;
    fdd_readsector(drv, dev->sector_reg, dev->track_reg, dev->side,
                   WD_DENSITY, dev->sector_size);
}

static void
wd_start_write(wd2791_t *dev)
{
    fdd_drive_t *drv = wd_drive(dev);

    if (drv == NULL)
        return;

    if (drv->writeprot) {
        dev->status |= WD_S_WP;
        wd_done(dev);
        return;
    }

    dev->byte_count = 0;
    dev->state      = WD_ST_WRITE;
    dev->drq        = 0;
    fdd_writesector(drv, dev->sector_reg, dev->track_reg, dev->side,
                    WD_DENSITY, dev->sector_size);
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

    dev->multi    = (val & 0x10) ? 1 : 0;
    dev->side     = (val & 0x08) ? 1 : 0;
    dev->settle   = (val & 0x04) ? 1 : 0;
    dev->compare  = (val & 0x02) ? 1 : 0;
    dev->deleted  = (val & 0x01) ? 1 : 0; /* a0 only on write */
    dev->sector_size = WD_SECTOR_SIZE;
    dev->byte_count  = 0;

    dev->fdc.deleted = dev->deleted ? 3 : 0;

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        dev->status |= WD_S_RNF;
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
wd_read_address(wd2791_t *dev, uint8_t val)
{
    fdd_drive_t *drv = wd_drive(dev);

    dev->settle = (val & 0x04) ? 1 : 0;

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        dev->status |= WD_S_RNF;
        wd_done(dev);
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, dev->side);

    dev->id_count   = 0;
    dev->byte_count = 0;
    dev->state      = WD_ST_READID;
    fdd_readaddress(drv, dev->side, WD_DENSITY);
}

static void
wd_read_track(wd2791_t *dev, uint8_t val)
{
    fdd_drive_t *drv = wd_drive(dev);

    dev->settle = (val & 0x04) ? 1 : 0;

    if ((drv == NULL) || !fdd_can_read_medium(drv)) {
        dev->status |= WD_S_RNF;
        wd_done(dev);
        return;
    }

    fdd_set_motor_enable(drv, 1);
    fdd_set_head(drv, dev->side);

    dev->byte_count = 0;
    dev->state      = WD_ST_READTRACK;
    fdd_readsector(drv, SECTOR_FIRST, dev->track_reg, dev->side,
                   WD_DENSITY, dev->sector_size);
}

static void
wd_write_track(wd2791_t *dev, UNUSED(uint8_t val))
{
    /* Formatting needs the MFM encoder (A1 sync words, gap and CRC layout).
       Until that is written the command completes without touching the
       medium so a formatter that probes the controller does not hang. */
    wd_done(dev);
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
    dev->busy  = 0;
    dev->state = WD_ST_IDLE;
    dev->drq   = 0;

    if (cond & 0x08)
        wd_irq_raise(dev);
}

/* ------------------------------------------------------------------------- */
/* registers                                                                 */
/* ------------------------------------------------------------------------- */

static void
wd2791_command(wd2791_t *dev, uint8_t val)
{
    /* A command register write clears INTRQ (die behaviour, see MAME
       cmd_w()). */
    wd_irq_lower(dev);

    dev->command = val;
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
                wd_irq_lower(dev);
                return ret;
            }

        case 0x01:
            return dev->track_reg;

        case 0x02:
            return dev->sector_reg;

        default:
            dev->drq = 0;
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
            dev->drq      = 0;
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
}

/* ------------------------------------------------------------------------- */
/* controller callbacks (installed into fdc_t)                               */
/* ------------------------------------------------------------------------- */

static void
wd_noidam(void *priv)
{
    wd2791_t *dev = (wd2791_t *) priv;

    if ((dev->state != WD_ST_READ) && (dev->state != WD_ST_READID) &&
        (dev->state != WD_ST_WRITE) && (dev->state != WD_ST_READTRACK))
        return;

    dev->status |= WD_S_RNF;
    wd_done(dev);
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

    timer_disable(&dev->timer);

    dev->command    = 0x00;
    dev->status     = 0x00;
    dev->track_reg  = 0x00;
    dev->sector_reg = 0x01; /* MR leaves the sector register at 1 (data book) */
    dev->data_reg   = 0x00;
    dev->dskctl     = 0x00;

    dev->state     = WD_ST_IDLE;
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
    dev->drive  = val & 0x03;
    dev->side   = (val & 0x08) ? 1 : 0;
    dev->dden   = (val & 0x10) ? 1 : 0;

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

    dev->fdc.ext          = dev;
    dev->fdc.ext_data     = wd_data_in;
    dev->fdc.ext_getdata  = wd_data_out;
    dev->fdc.ext_noidam   = wd_noidam;
    dev->fdc.ext_seek_done = wd_seek_complete;
    dev->fdc.ext_diskchange = wd_diskchange;

    timer_add(&dev->timer, wd2791_timer, dev, 0);

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
