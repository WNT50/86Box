/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          WD1010 and WD2010 Winchester disk controller chip model.
 *
 * Authors: Karl-Ludwig Deisenhofer
 *          Curt Coder
 *          WNT50
 *
 *          Copyright 2015-2017 Karl-Ludwig Deisenhofer.
 *          Copyright 2010-2026 Curt Coder.
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_HDC_ST506_WD1010_H
#define EMU_HDC_ST506_WD1010_H

#include <stdint.h>
#include <86box/hdc.h>

typedef struct wd1010_t wd1010_t;

enum {
    WD1010_REG_DATA = 0,
    WD1010_REG_ERROR,
    WD1010_REG_COUNT,
    WD1010_REG_SECTOR,
    WD1010_REG_CYL_LO,
    WD1010_REG_CYL_HI,
    WD1010_REG_SDH,
    WD1010_REG_STATUS
};

/* Status register bits. */
#define WD1010_ST_ERR  0x01
#define WD1010_ST_DRQ  0x08
#define WD1010_ST_SC   0x10
#define WD1010_ST_RDY  0x40
#define WD1010_ST_BUSY 0x80

typedef struct {
    uint8_t variant; /* 0 = WD1010, 1 = WD2010, 2 = WD5011 */
    struct {
        int      hdd_num; /* negative: no drive on this unit */
        uint32_t spt;
        uint32_t hpc;
        uint32_t tracks;
    } unit[MFM_NUM];
    void (*intrq)(void *priv, int level); /* NULL: INTRQ not connected */
    void *priv;
} wd1010_config_t;

extern wd1010_t  *wd1010_create(const wd1010_config_t *cfg);
extern void    wd1010_destroy(wd1010_t *dev);
extern void    wd1010_reset(wd1010_t *dev);
extern void    wd1010_write(wd1010_t *dev, int reg, uint8_t val);
extern uint8_t wd1010_read(wd1010_t *dev, int reg);
extern uint8_t *wd1010_buffer(wd1010_t *dev);

/* HS3: the fourth head bit. It never reaches the chip itself; the board
   drives it straight onto the drive interface. The model needs it to
   compute the sector address. */
extern void    wd1010_set_hs3(wd1010_t *dev, int level);

#endif /*EMU_HDC_ST506_WD1010_H*/
