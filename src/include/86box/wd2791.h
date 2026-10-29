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
 *          The WD2791 is a sector-level controller: four registers (status/
 *          command, track, sector, data), one command byte, and the host (or
 *          the on-chip DMA) moves every byte through the data register.  That
 *          is a different programming model from the uPD765 model in fdc.c,
 *          so this file implements the registers and the command state
 *          machine itself and reuses the drive layer (fdd_*) for the medium.
 *
 *          Wiring on the PC-X board (MAME pcd.cpp:190-214 and the
 *          Servicehandbuch agree): INTRQ -> master 8259 IR6, DRQ -> 80186
 *          on-chip DMA channel 1, registers at 0xf900-0xf903, drive/motor/
 *          side/DDEN latch at 0xf904, 2 MHz clock (16 MHz / 8).
 */

#ifndef EMU_WD2791_H
#define EMU_WD2791_H

#include <stdint.h>
#include <86box/timer.h>
#include <86box/fdd.h>
#include <86box/fdc.h>

/* Interrupt and DMA line the PC-X wires the chip to. */
#define WD2791_IRQ 6
#define WD2791_DMA 1

/* Command state machine. */
enum {
    WD_ST_IDLE = 0,
    WD_ST_SEEK,  /* waiting for the drive to report the seek complete */
    WD_ST_SETTLE,/* 15 ms head settle before a read/write */

    WD_ST_READ,
    WD_ST_WRITE,
    WD_ST_READID,
    WD_ST_READTRACK,
    WD_ST_WRITETRACK
};

typedef struct wd2791_t {
    /* Must stay first: the drive layer hands a fdc_t * back to us through
       fdc_data()/fdc_getdata() and the controller callbacks. */
    fdc_t fdc;

    uint8_t command;   /* 0xf900 write */
    uint8_t status;    /* 0xf900 read */
    uint8_t track_reg; /* 0xf901 */
    uint8_t sector_reg;/* 0xf902 */
    uint8_t data_reg;  /* 0xf903 */

    uint8_t dskctl;    /* 0xf904 latch: drives, motor, side, DDEN */

    int state;
    int type1;    /* current command belongs to Type I */
    int busy;
    int drq;
    int intrq;

    int drive;
    int side;
    int dden;     /* 1 = FM single density */

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
    int id_count;
    uint8_t id_buf[8];

    pc_timer_t timer;
} wd2791_t;

extern wd2791_t *wd2791_init(void);
extern void      wd2791_close(void);
extern void      wd2791_reset(void);
extern void      wd2791_set_dskctl(wd2791_t *dev, uint8_t val);
extern uint8_t   wd2791_get_dskctl(wd2791_t *dev);

#endif /*EMU_WD2791_H*/
