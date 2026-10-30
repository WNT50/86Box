/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          WD1010 and WD2010 hard disk controller emulation. WD2010
 *          extends WD1010 controller with a 32-bit ECC engine and
 *          one more cylinder address bit.
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
#include <86box/hdc.h>
#include <86box/hdd.h>
#include <86box/io.h>
#include <86box/ui.h>
#include <86box/timer.h>
#include <86box/pic.h>
#include <86box/plat_unused.h>

/* The 2000HD option is a WD1010 with a one-sector buffer in front of it. The
   host moves a sector through the buffer at register 0 and drives the WD1010
   task file at registers 1-7; the board has no DOR and nothing raises INTRQ,
   so the host polls the status register. */
#define REG_DATA         0 /* 270h */
#define REG_ERROR        1 /* 272h, write = precompensation */
#define REG_COUNT        2 /* 274h */
#define REG_SECTOR       3 /* 276h */
#define REG_CYL_LO       4 /* 278h */
#define REG_CYL_HI       5 /* 27ah */
#define REG_SDH          6 /* 27ch */
#define REG_STATUS       7 /* 27eh, write = command */

#define ST_ERR           0x01
#define ST_DRQ           0x08
#define ST_SC            0x10
#define ST_RDY           0x40
#define ST_BUSY          0x80

#define ERR_AC           0x04
#define ERR_ID           0x10
#define ERR_BB           0x80

#define CMD_RESTORE      0x10
#define CMD_READ_SECTOR  0x20
#define CMD_WRITE_SECTOR 0x30
#define CMD_SECT_SCAN_ID 0x40
#define CMD_WRITE_FORMAT 0x50
#define CMD_SEEK         0x70
#define CMD_SET_PARAMETERS 0x91

#define SECTOR_SIZE      512

/* The drive has to seek and then wait for the sector to come under the head
   before it can move data, so the board stays busy - and its activity light
   stays on - for as long as the real mechanism would take. */
#define ACCESS_TIME      (20 * TIMER_USEC * 1000)
#define FORMAT_SECTOR    (1000 * TIMER_USEC)

#define STATE_IDLE        0
#define STATE_READ_WAIT   1
#define STATE_WRITE_WAIT  2
#define STATE_FORMAT_SEEK 3
#define STATE_FORMAT_SEND 4

/* The WD2010 is the same task file with a 32-bit ECC engine (READ/WRITE
   with ECC is accepted) and one more cylinder address bit. */
typedef struct wd10_variant_t {
    uint8_t cyl_mask; /* mask applied to the high cylinder register */
    uint8_t ecc;      /* accept READ/WRITE with ECC (command bit 1) */
} wd10_variant_t;

static const wd10_variant_t wd10_variant_1010 = { 0xff, 0 };
static const wd10_variant_t wd10_variant_2010 = { 0x07, 1 };

#ifdef ENABLE_WD10_HDC_LOG
int wd10_hdc_do_log = ENABLE_WD10_HDC_LOG;

static void
wd10_hdc_log(const char *fmt, ...)
{
    va_list ap;

    if (wd10_hdc_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define wd10_hdc_log(fmt, ...)
#endif

typedef struct wd10_hdc_t {
    const wd10_variant_t *var;

    /* The WD1010 sits on the Tandy 2000HD board (task file on even addresses
       of 270h, no interrupt). The WD2010 is modelled where it actually
       shipped: an AT adapter with the task file at 1F0h, a device control
       register at 3F6h and IRQ 14. */
    uint8_t  at;

    uint8_t  buf[SECTOR_SIZE];
    uint16_t buf_ptr;

    uint8_t  status;
    uint8_t  errors;
    uint8_t  sector;
    uint8_t  cyl_lo;
    uint8_t  cyl_hi;
    uint8_t  count;
    uint8_t  sdh;
    uint8_t  fdisk;  /* AT device control register */
    uint8_t  irqstat;

    uint8_t  head;
    uint8_t  unit;

    uint8_t  present[MFM_NUM];
    uint8_t  hdd_num[MFM_NUM];
    uint32_t tracks[MFM_NUM];
    uint32_t spt[MFM_NUM];
    uint32_t hpc[MFM_NUM];
    uint32_t cfg_spt[MFM_NUM]; /* SET DRIVE PARAMETERS overrides */
    uint32_t cfg_hpc[MFM_NUM];

    uint32_t   xfers; /* sectors left in the current transfer */
    uint32_t   fmt_bytes;
    uint32_t   lba_prev;
    uint32_t   sectors;
    int        state;
    pc_timer_t timer;
} wd10_hdc_t;

static void
wd10_hdc_led_off(void)
{
    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, 0);
    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, 0);
}

static void
wd10_hdc_led(wd10_hdc_t *dev, int write)
{
    ui_sb_update_icon(SB_HDD | HDD_BUS_MFM, !write);
    ui_sb_update_icon_write(SB_HDD | HDD_BUS_MFM, write);
}

/* INTRQ is a level the host clears by reading the status register. On the
   Tandy board nothing is wired to it. */
static void
wd10_hdc_irq_update(wd10_hdc_t *dev)
{
    if (!dev->at)
        return;

    /* Device control bit 1 (nIEN) masks the line. */
    picint_common(1 << 14, PIC_IRQ_EDGE, !(dev->fdisk & 0x02) && dev->irqstat, NULL);
}

static void
wd10_hdc_irq_raise(wd10_hdc_t *dev)
{
    dev->irqstat = 1;
    wd10_hdc_irq_update(dev);
}

static void
wd10_hdc_irq_lower(wd10_hdc_t *dev)
{
    dev->irqstat = 0;
    wd10_hdc_irq_update(dev);
}

/* SET DRIVE PARAMETERS can override the geometry the image reports; the
   physical values stay untouched, so a reset cannot lose them. */
static uint32_t
wd10_hdc_spt(wd10_hdc_t *dev, uint8_t unit)
{
    return dev->cfg_spt[unit] ? dev->cfg_spt[unit] : dev->spt[unit];
}

static uint32_t
wd10_hdc_hpc(wd10_hdc_t *dev, uint8_t unit)
{
    return dev->cfg_hpc[unit] ? dev->cfg_hpc[unit] : dev->hpc[unit];
}

static void
wd10_hdc_reset_state(wd10_hdc_t *dev)
{
    timer_disable(&dev->timer);

    dev->sector    = 0x00;
    /* After a reset the error register reads 01h, which is how an AT BIOS
       knows the controller came back (the IBM POST compares it with 1). */
    dev->errors    = 0x01;
    dev->cyl_lo    = 0x00;
    dev->cyl_hi    = 0x00;
    dev->count     = 0x00;
    dev->head      = 0x00;
    dev->unit      = 0x00;
    dev->sdh       = 0x00;
    dev->buf_ptr   = 0;
    dev->sectors   = 0;
    dev->lba_prev  = 0;
    dev->fmt_bytes = 0;
    dev->state     = STATE_IDLE;

    dev->xfers = 0;
    wd10_hdc_irq_lower(dev);

    dev->status = dev->present[0] ? (ST_RDY | ST_SC) : 0x00;
}

/* Turn the task file's CHS into a linear sector number, checking it
   against the geometry of the drive the SDH register selected. */
static int
wd10_hdc_lba(wd10_hdc_t *dev, uint32_t *lba)
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

    if ((sector < 1) || (sector > wd10_hdc_spt(dev, unit)) || (head >= wd10_hdc_hpc(dev, unit)) ||
        (cyl >= dev->tracks[unit]))
        return 0;

    *lba = ((cyl * wd10_hdc_hpc(dev, unit)) + head) * wd10_hdc_spt(dev, unit) + (sector - 1);

    return 1;
}

static void
wd10_hdc_command(wd10_hdc_t *dev, uint8_t command)
{
    uint32_t lba;

    dev->buf_ptr = 0;
    dev->errors  = 0x00;
    dev->status  = ST_RDY | ST_SC;

    wd10_hdc_log("WD10 HDC: command %02X, unit %i, %i/%i/%i, count %i\n",
                 command, dev->unit, ((uint32_t) dev->cyl_hi << 8) | dev->cyl_lo,
                 dev->head, dev->sector, dev->count);

    if (!dev->present[dev->unit]) {
        /* Nothing on that unit: the command still completes and asserts INTRQ,
           otherwise a host that waits for the interrupt hangs until it times
           out (the VAXmate BIOS probes both drives this way at boot). */
        dev->errors = ERR_AC;
        dev->status = ST_RDY | ST_SC | ST_ERR;
        wd10_hdc_irq_raise(dev);
        return;
    }

    switch (command & 0xf0) {
        case CMD_RESTORE:
            dev->cyl_lo = 0x00;
            dev->cyl_hi = 0x00;
            dev->status = ST_RDY | ST_SC;
            wd10_hdc_irq_raise(dev);
            break;

        case CMD_SEEK:
            /* The cylinder registers already hold the target; the head is
               there once the command completes, which is what INTRQ reports. */
            dev->status = ST_RDY | ST_SC;
            wd10_hdc_irq_raise(dev);
            break;

        case 0x90:
            /* SET DRIVE PARAMETERS (91h): the AT BIOS announces the geometry
               it is about to use before the first transfer. */
            if (command == CMD_SET_PARAMETERS) {
                uint8_t unit = dev->unit;

                dev->cfg_spt[unit] = dev->count ? dev->count : dev->spt[unit];
                dev->cfg_hpc[unit] = (dev->sdh & 0x07) + 1;
                break;
            }

            dev->status |= ST_ERR;
            dev->errors  = ERR_AC;
            break;

        case CMD_READ_SECTOR:
        case CMD_WRITE_SECTOR:
            /* Command bit 1 selects the ECC form of the transfer; the WD1010
               has no ECC engine and aborts, the WD2010 accepts it (the ECC
               itself is not modelled, the data path is unchanged). */
            if ((command & 0x02) && !dev->var->ecc) {
                dev->status |= ST_ERR;
                dev->errors  = ERR_AC;
                break;
            }

            if (!wd10_hdc_lba(dev, &lba)) {
                dev->status |= ST_ERR;
                dev->errors  = ERR_ID;
                break;
            }

            dev->lba_prev = lba;
            dev->xfers    = dev->count ? dev->count : 256;
            dev->status  |= ST_BUSY;
            dev->state    = ((command & 0xf0) == CMD_READ_SECTOR) ? STATE_READ_WAIT : STATE_WRITE_WAIT;

            wd10_hdc_led(dev, dev->state == STATE_WRITE_WAIT);

            timer_set_delay_u64(&dev->timer, ACCESS_TIME);
            break;

        case CMD_WRITE_FORMAT: {
            /* The Tandy format utility sends one of these per track, with the
               sector count set to the sectors per track and the sector number
               register left at an unused value, so the track is numbered from
               sector one here. */
            uint32_t cyl     = ((uint32_t) dev->cyl_hi << 8) | dev->cyl_lo;
            uint32_t sectors = dev->count ? dev->count : 256;

            if ((dev->head >= wd10_hdc_hpc(dev, dev->unit)) || (cyl >= dev->tracks[dev->unit])) {
                dev->status |= ST_ERR;
                dev->errors  = ERR_ID;
                break;
            }

            if (sectors > wd10_hdc_spt(dev, dev->unit))
                sectors = wd10_hdc_spt(dev, dev->unit);

            dev->lba_prev  = ((cyl * wd10_hdc_hpc(dev, dev->unit)) + dev->head) * wd10_hdc_spt(dev, dev->unit);
            dev->fmt_bytes = 0;
            dev->sectors   = sectors;
            dev->status   |= ST_BUSY;
            dev->state     = STATE_FORMAT_SEEK;

            wd10_hdc_led(dev, 1);

            /* A seek, and then the host supplies one four byte ID per sector
               through the data register while DRQ is up. */
            timer_set_delay_u64(&dev->timer, ACCESS_TIME);
            break;
        }

        case CMD_SECT_SCAN_ID:
        default:
            dev->status |= ST_ERR;
            dev->errors  = ERR_AC;
            break;
    }

    /* A command that ends without a data transfer interrupts the host. */
    if (!(dev->status & (ST_BUSY | ST_DRQ)))
        wd10_hdc_irq_raise(dev);
}

/* Fills the track the format command selected with the E5 pattern the chip
   writes into data fields it is not given contents for. */
static void
wd10_hdc_finish_format(wd10_hdc_t *dev)
{
    dev->fmt_bytes = 0;
    dev->state    = STATE_IDLE;
    dev->status  &= ~(ST_BUSY | ST_DRQ);

    memset(dev->buf, 0xe5, SECTOR_SIZE);

    for (uint32_t s = 0; s < dev->sectors; s++) {
        if (hdd_image_write(dev->hdd_num[dev->unit], dev->lba_prev + s, 1, dev->buf) < 0) {
            dev->status |= ST_ERR;
            dev->errors  = ERR_BB;
            break;
        }
    }
    wd10_hdc_led_off();
}

/* Runs when the mechanism has finished moving the data; only then does the
   board release the host with DRQ (or with the error the drive reported). */
static void
wd10_hdc_callback(void *priv)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) priv;

    switch (dev->state) {
        case STATE_READ_WAIT:
            dev->state   = STATE_IDLE;
            dev->status &= ~ST_BUSY;

            if (hdd_image_read(dev->hdd_num[dev->unit], dev->lba_prev, 1, dev->buf) < 0) {
                dev->status |= ST_ERR;
                dev->errors  = ERR_BB;
                wd10_hdc_led_off();
                break;
            }

            dev->status |= ST_DRQ;
            wd10_hdc_irq_raise(dev);
            break;

        case STATE_WRITE_WAIT:
            dev->state   = STATE_IDLE;
            dev->status &= ~ST_BUSY;
            dev->status |= ST_DRQ;
            wd10_hdc_irq_raise(dev);
            break;

        case STATE_FORMAT_SEEK:
            /* The head is over the track: request the ID of the first sector
               and let the host stream four bytes per sector through the data
               register. The watchdog covers hosts that ignore DRQ. */
            dev->fmt_bytes = 0;
            dev->buf_ptr   = 0;
            dev->status   |= ST_DRQ;
            dev->state     = STATE_FORMAT_SEND;

            timer_set_delay_u64(&dev->timer, ACCESS_TIME + (dev->sectors * FORMAT_SECTOR));
            break;

        case STATE_FORMAT_SEND:
            /* The host never supplied the IDs, so give it sequential ones. */
            wd10_hdc_finish_format(dev);
            break;

        default:
            break;
    }

    wd10_hdc_log("WD10 HDC: done, status %02X, error %02X\n", dev->status, dev->errors);
}

static uint8_t
wd10_hdc_read_data(wd10_hdc_t *dev)
{
    uint8_t ret = dev->buf[dev->buf_ptr];

    dev->buf_ptr++;
    if (dev->buf_ptr >= SECTOR_SIZE) {
        dev->buf_ptr = 0;
        dev->status &= ~ST_DRQ;
        wd10_hdc_led_off();
        wd10_hdc_irq_lower(dev);

        if (dev->xfers > 1) {
            /* Multi-sector transfer: the mechanism has to reach the next
               sector before the chip raises DRQ again. */
            dev->xfers--;
            dev->lba_prev++;
            dev->status |= ST_BUSY;
            dev->state   = STATE_READ_WAIT;
            timer_set_delay_u64(&dev->timer, ACCESS_TIME);
        } else {
            dev->xfers = 0;
        }
    }

    return ret;
}

static void
wd10_hdc_write_data(wd10_hdc_t *dev, uint8_t val)
{
    if (dev->state == STATE_FORMAT_SEND) {
        /* Four ID bytes per sector arrive in one burst; the IDs themselves
           do not matter here because the sector numbers follow the track. */
        dev->fmt_bytes++;

        if (dev->fmt_bytes >= (dev->sectors << 2)) {
            timer_disable(&dev->timer);
            wd10_hdc_finish_format(dev);
        }

        return;
    }

    if (!(dev->status & ST_DRQ))
        return;

    dev->buf[dev->buf_ptr] = val;

    dev->buf_ptr++;
    if (dev->buf_ptr >= SECTOR_SIZE) {
        dev->buf_ptr = 0;
        dev->status &= ~ST_DRQ;
        wd10_hdc_led_off();
        wd10_hdc_irq_lower(dev);

        if (hdd_image_write(dev->hdd_num[dev->unit], dev->lba_prev, 1, dev->buf) < 0) {
            dev->status |= ST_ERR;
            dev->errors  = ERR_BB;
            dev->xfers   = 0;
            return;
        }

        if (dev->xfers > 1) {
            dev->xfers--;
            dev->lba_prev++;
            dev->status |= ST_BUSY;
            dev->state   = STATE_WRITE_WAIT;
            timer_set_delay_u64(&dev->timer, ACCESS_TIME);
        } else {
            dev->xfers = 0;
        }
    }
}

static int
wd10_hdc_reg(wd10_hdc_t *dev, uint16_t addr)
{
    /* The Tandy task file spreads the eight registers over 270h - 27Fh, the
       AT task file packs them into 1F0h - 1F7h. */
    if (dev->at)
        return addr - 0x01f0;

    return (addr - 0x0270) >> 1;
}

static uint8_t
wd10_hdc_read(uint16_t addr, void *priv)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) priv;

    switch (wd10_hdc_reg(dev, addr)) {
        case REG_DATA:
            return wd10_hdc_read_data(dev);
        case REG_STATUS:
            /* Reading the status register acknowledges the interrupt. */
            wd10_hdc_irq_lower(dev);
            return dev->status;
        case REG_SECTOR:
            return dev->sector;
        case REG_CYL_LO:
            return dev->cyl_lo;
        case REG_CYL_HI:
            return dev->cyl_hi;
        case REG_ERROR:
            return dev->errors;
        case REG_COUNT:
            return dev->count;
        case REG_SDH:
            /* An AT adapter reports the drive select back in bits 4 and 0-2. */
            if (dev->at)
                return 0xa0 | dev->head | (dev->unit ? 0x10 : 0x00);
            return dev->sdh;
        default:
            break;
    }

    return 0xff;
}

static void
wd10_hdc_write(uint16_t addr, uint8_t val, void *priv)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) priv;

    switch (wd10_hdc_reg(dev, addr)) {
        case REG_DATA:
            wd10_hdc_write_data(dev, val);
            break;
        case REG_ERROR:
            /* Write only: write precompensation, not modelled. */
            break;
        case REG_COUNT:
            dev->count = val;
            break;
        case REG_SECTOR:
            dev->sector = val;
            break;
        case REG_CYL_LO:
            dev->cyl_lo = val;
            break;
        case REG_CYL_HI:
            dev->cyl_hi = val & dev->var->cyl_mask;
            break;
        case REG_SDH:
            dev->sdh  = val;
            dev->head = val & 0x07;
            /* The AT task file selects the drive with bit 4; the Tandy board
               uses bits 3-4 as a straight unit number. */
            dev->unit = dev->at ? ((val >> 4) & 0x01) : ((val >> 3) & 0x03);

            /* Selecting a drive clears a transfer that was left pending. */
            if (dev->status & ST_DRQ) {
                wd10_hdc_led_off();
                wd10_hdc_irq_lower(dev);
            }

            dev->status &= ~ST_DRQ;
            break;
        case REG_STATUS:
            /* The chip ignores a new command while it is still busy with the
               previous one. */
            if (!(dev->status & ST_BUSY))
                wd10_hdc_command(dev, val);
            break;
        default:
            break;
    }
}

/* The board's reset port answers reads and takes writes; either one resets the
   controller, which is how the BIOS probes for the card. */
static uint8_t
wd10_hdc_reset_r(UNUSED(uint16_t addr), void *priv)
{
    wd10_hdc_reset_state((wd10_hdc_t *) priv);

    return 0xff;
}

static void
wd10_hdc_reset_w(UNUSED(uint16_t addr), UNUSED(uint8_t val), void *priv)
{
    wd10_hdc_reset_state((wd10_hdc_t *) priv);
}

/* The AT BIOS moves a whole sector with REP INSW / REP OUTSW, so the data
   register has to answer 16-bit accesses with two bytes at a time. */
static uint16_t
wd10_hdc_readw(uint16_t addr, void *priv)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) priv;

    uint16_t ret;

    if (wd10_hdc_reg(dev, addr) != REG_DATA) {
        ret = wd10_hdc_read(addr, priv);
        if (addr == 0x01f7)
            ret |= 0xff00;
        else if (addr < 0x01f7)
            ret |= (wd10_hdc_read(addr + 1, priv) << 8);
        return ret;
    }

    /* The order of the two fetches is part of the data stream, so they must
       be sequenced explicitly: C leaves the operand order unspecified. */
    ret  = wd10_hdc_read_data(dev);
    ret |= (uint16_t) (wd10_hdc_read_data(dev) << 8);

    return ret;
}

static void
wd10_hdc_writew(uint16_t addr, uint16_t val, void *priv)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) priv;

    if (wd10_hdc_reg(dev, addr) != REG_DATA) {
        wd10_hdc_write(addr, (uint8_t) val, priv);
        return;
    }

    wd10_hdc_write_data(dev, (uint8_t) val);
    wd10_hdc_write_data(dev, (uint8_t) (val >> 8));
}

/* AT device control register (3F6h write): bit 2 holds the controller in
   reset, bit 1 masks the interrupt. 3F7h mirrors the status. */
static void
wd10_hdc_ctrl_w(UNUSED(uint16_t addr), uint8_t val, void *priv)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) priv;
    uint8_t     old = dev->fdisk;

    val &= 0x0f;

    if (!(old & 0x04) && (val & 0x04))
        dev->status = ST_BUSY; /* held in reset */
    else if ((old & 0x04) && !(val & 0x04))
        wd10_hdc_reset_state(dev);

    dev->fdisk = val;

    if (!(val & 0x02) && (old & 0x02))
        wd10_hdc_irq_update(dev);
}

static uint8_t
wd10_hdc_status_r(UNUSED(uint16_t addr), void *priv)
{
    return ((wd10_hdc_t *) priv)->status;
}

static void
wd10_hdc_reset(void *priv)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) priv;

    wd10_hdc_led_off();

    wd10_hdc_reset_state(dev);
}

static void *
wd10_hdc_init(const device_t *info)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) calloc(1, sizeof(wd10_hdc_t));

    /* The WD1010 keeps the Tandy 2000HD board; the WD2010 is wired as the AT
       adapter it shipped on (task file 1F0h-1F7h, control 3F6h, IRQ 14). */
    dev->var = info->local ? &wd10_variant_2010 : &wd10_variant_1010;
    dev->at  = info->local ? 1 : 0;

    for (uint8_t i = 0; i < HDD_NUM; i++) {
        if ((hdd[i].bus_type != HDD_BUS_MFM) || (hdd[i].mfm_channel >= MFM_NUM))
            continue;

        if (!hdd_image_load(i))
            continue;

        dev->present[hdd[i].mfm_channel] = 1;
        dev->hdd_num[hdd[i].mfm_channel] = i;
        dev->spt[hdd[i].mfm_channel]     = hdd[i].spt;
        dev->hpc[hdd[i].mfm_channel]     = hdd[i].hpc;
        dev->tracks[hdd[i].mfm_channel]  = hdd[i].tracks;
    }

    if (dev->at) {
        io_sethandler(0x03f6, 0x0001, NULL, NULL, NULL,
                      wd10_hdc_ctrl_w, NULL, NULL, dev);
        io_sethandler(0x03f7, 0x0001, wd10_hdc_status_r, NULL, NULL,
                      NULL, NULL, NULL, dev);

        io_sethandler(0x01f0, 0x0001, wd10_hdc_read, wd10_hdc_readw, NULL,
                      wd10_hdc_write, wd10_hdc_writew, NULL, dev);
        io_sethandler(0x01f1, 0x0007, wd10_hdc_read, wd10_hdc_readw, NULL,
                      wd10_hdc_write, wd10_hdc_writew, NULL, dev);
    } else {
        io_sethandler(0x026c, 0x0002, wd10_hdc_reset_r, NULL, NULL,
                      wd10_hdc_reset_w, NULL, NULL, dev);
        io_sethandler(0x0270, 0x0010, wd10_hdc_read, NULL, NULL,
                      wd10_hdc_write, NULL, NULL, dev);
    }

    timer_add(&dev->timer, wd10_hdc_callback, dev, 0);

    wd10_hdc_reset_state(dev);

    return dev;
}

static void
wd10_hdc_close(void *priv)
{
    wd10_hdc_t *dev = (wd10_hdc_t *) priv;

    for (uint8_t i = 0; i < MFM_NUM; i++) {
        if (dev->present[i])
            hdd_image_close(dev->hdd_num[i]);
    }

    free(dev);
}

const device_t st506_wd1010_device = {
    .name          = "WD1010 Hard Disk Controller",
    .internal_name = "st506_wd1010",
    .flags         = 0,
    .local         = 0,
    .init          = wd10_hdc_init,
    .close         = wd10_hdc_close,
    .reset         = wd10_hdc_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t st506_wd2010_device = {
    .name          = "WD2010 Hard Disk Controller",
    .internal_name = "st506_wd2010",
    .flags         = 0,
    .local         = 1,
    .init          = wd10_hdc_init,
    .close         = wd10_hdc_close,
    .reset         = wd10_hdc_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
