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
 *          Western Digital WD279x floppy disk controller family: registers
 *          and command state.
 *
 * Authors: Olivier Galibert, <galibert@pobox.com>
 *          WNT50
 *
 *          Copyright 2012-2015 Olivier Galibert.
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_WD279X_H
#define EMU_WD279X_H

#include <stdint.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/fdd.h>

/* Interrupt and DMA line the PC-X wires the chip to. */
#define WD279X_IRQ 6
#define WD279X_DMA 1

/* Command state machine. */
enum {
    WD_ST_IDLE = 0,
    WD_ST_SEEK,  /* waiting for the drive to report the seek complete */
    WD_ST_SETTLE,/* 15 ms head settle before a read/write */

    WD_ST_READ,
    WD_ST_WRITE,
    WD_ST_READID,
    WD_ST_READTRACK,
    WD_ST_WRITETRACK,

    /* The last data byte of a write has gone in; the drive still needs a
       moment to put it on the track before the command may finish. */
    WD_ST_WRITE_END,

    /* The data bytes are gone; the chip still shifts out the CRC and latches
       the status before it lifts INTRQ.  A polled driver that has just read
       its last byte needs that window to issue Force Interrupt, which clears
       the line, instead of being interrupted for a command it already owns. */
    WD_ST_LATCH
};

#define WD_FMT_BUF_SIZE     16384
#define WD_FMT_MAX_SECTORS  64

/* One record found in a Write Track stream. */
typedef struct wd_fmt_sector {
    uint8_t  c;
    uint8_t  h;
    uint8_t  r;
    uint8_t  n;
    uint16_t offset; /* first data byte in fmt_buf */
    uint16_t length; /* data bytes */
} wd_fmt_sector_t;

typedef struct wd279x_t {
    /* The four drives this controller serves. The drive layer reports back
       through the fdd_ctrl_t hooks, so there is no fdc_t here. */
    fdd_drive_t *fdd[4];

    /* State the drive layer asks about. */
    uint8_t  rate;
    int      mfm;
    uint32_t bitcell_period;
    int      dma;      /* the transfer runs through the DMA channel */
    int      dma_mode; /* board setting: transfers may use the DMA channel */
    int      dma_ch;
    int      tc;       /* terminal count was reported */
    int      dtl;      /* transfer length the drive layer works from */

    uint8_t command;   /* 0xf900 write */
    uint8_t status;    /* 0xf900 read */
    uint8_t track_reg; /* 0xf901 */
    uint8_t sector_reg;/* 0xf902 */
    uint16_t base;       /* first register port */
    int     reg_step;    /* 1 = byte spacing, 2 = the 16-bit bus on the :YES */
    int     sector_base; /* 1 = the image numbers sectors from 1 */
    uint8_t data_reg;  /* 0xf903 */

    /* Microseconds between the last data byte and INTRQ (CRC + status phase).
       PCX_POST_US overrides it; 0 makes the completion immediate again. */
    int post_us;
    int latched;
    uint8_t dskctl;    /* 0xf904 latch: drives, motor, side, DDEN */

    int state;
    int type1;    /* current command belongs to Type I */
    int busy;
    int drq;

    /* Write Track: the host hands over a whole revolution as raw MFM bytes,
       address marks and CRCs included.  The stream is kept as it arrives, then
       parsed into records and laid down through the normal write path. */
    uint8_t  fmt_buf[WD_FMT_BUF_SIZE];
    int      fmt_len;    /* bytes collected so far */
    int      fmt_raw;    /* bytes in one revolution on this track */
    int      fmt_count;  /* records found in the stream */
    int      fmt_cur;    /* record being written back */
    int      fmt_n;      /* N of the laid-out records */
    int      fmt_active; /* the write-back is in progress */
    int      fmt_id_idx; /* record whose ID the formatter is laying down */
    int      fmt_id_byte; /* byte of that ID the formatter is collecting */
    /* ID bytes the drive layer collects while it lays a track out; the
       drive layer owns drv->xfer.format_id and writes here. */
    fdd_sector_id_t fmt_id;
    wd_fmt_sector_t fmt_sec[WD_FMT_MAX_SECTORS];
    /* A write that stopped on the protect tab keeps S6 set until the next
       command; the guest often only looks at the status after BUSY drops. */
    int wp_latched;
    int intrq;

    /* Board hook: when set, INTRQ is reported here instead of through the
       PC's PIC.  The PC-X wires it to IR6; the Philips :YES feeds the
       i8256 MUART's interrupt input instead. */
    void (*irq_cb)(void *priv, int level);
    void     *irq_priv;

    int drive;
    int side;
    int dden;     /* 1 = FM single density */
    int side_latched; /* side comes from the 0xf904 latch on this board */
    /* The meaning of the dskctl latch is board wiring, not part behaviour: the
       PC-X puts the head on bit 3 and density on bit 4, the :YES puts the head
       on bit 4 and has no density bit at all.  Defaults are the PC-X (and
       common ancestor) mapping; a board that differs says so at init. */
    /* Record length the medium turned out to hold, per track and side, learned
       from the ID a read matched.  -1 means "not seen yet".  The drive layer
       cannot answer this for an image that was never written through it, and
       guessing from the command reads 256-byte records as 512. */
    int8_t   medium_n[64][2];
    int      chain_wrapped;

    int dskctl_side_bit;    /* latch bit that flips the head */
    int dskctl_drive_onehot;/* drives one hot in the low nibble, else bit 0/1 */
    int dskctl_dden_bit;    /* latch bit for density, -1 when the board has none */

    int multi;    /* m bit */
    int settle;   /* E bit */
    int verify;   /* V bit */
    int update;   /* T bit */
    int compare;  /* C bit */
    int deleted;  /* a0 bit / F8 address mark */

    int step_dir; /* 1 = towards track 0 */
    int target;

    int sector_size;
    int byte_count;
    int first_read;   /* set when the drive is selected, cleared by the first read */
    int was_first;    /* value of first_read when the running command started */
    int id_count;
    uint8_t id_buf[8];

    /* The one byte data register has a sector buffer behind it: a polling
       driver reads 0xf903 a byte at a time, while the bitstream layer may hand
       the whole sector over in a single burst, so the bytes queue up here and
       BUSY stays asserted until the host has drained them. */
    uint8_t buf[1024];
    int     buf_head;
    int     buf_tail;
    int     buf_len;
    /* The data register proper: one byte, filled from the medium at the byte
       rate and emptied by the host.  A whole sector must not pile up in front
       of it -- the 279x raises DRQ per byte and reports Lost Data when the
       host misses one. */
    uint8_t dr;
    int     dr_full;
    pc_timer_t byte_timer;
    int     done_pending;
    int     chain_pending;
    int     chain_follow;  /* a chained read is looking for its next sector */
    int     rec_done;      /* records this command has completed */
    int     tc_seen;      /* the DMA channel reported terminal count */

    pc_timer_t timer;
    pc_timer_t ts_timer;   /* PCX_CHAINDBG sampler, see wd279x.c */
    int        in_dma_fetch; /* the data-port read comes from the DMA engine */
} wd279x_t;

extern wd279x_t *wd279x_init(void);
extern wd279x_t *wd279x_init_at(uint16_t base);
extern void      wd279x_set_irq_handler(wd279x_t *dev,
                                        void (*cb)(void *priv, int level),
                                        void *priv);
/* How long the controller waits after a command completes before it drops
   INTRQ, in microseconds. */
extern void wd279x_set_post_us(wd279x_t *dev, uint32_t us);
/* Whether data transfers use the DMA channel. Boards that move the bytes with
   their own DMA controller enable this before issuing a read or write. */
extern void wd279x_set_dma(wd279x_t *dev, int enable);
extern void      wd279x_close(void);
extern void      wd279x_reset(void);
extern void      wd279x_set_dskctl(wd279x_t *dev, uint8_t val);
extern void      wd279x_set_dskctl_bits(wd279x_t *dev, int side_bit, int drive_onehot,
                                         int dden_bit);
extern uint8_t   wd279x_get_dskctl(wd279x_t *dev);
extern void      wd279x_set_sector_base(wd279x_t *dev, int base);
extern void      wd279x_set_reg_step(wd279x_t *dev, int step);

extern const device_t wd279x_device;

#endif /*EMU_WD279X_H*/
