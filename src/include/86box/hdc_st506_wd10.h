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
#ifndef EMU_HDC_ST506_WD10_H
#define EMU_HDC_ST506_WD10_H

#include <stdint.h>
#include <86box/hdc.h>

typedef struct wd10_t wd10_t;

enum {
    WD10_REG_DATA = 0,
    WD10_REG_ERROR,
    WD10_REG_COUNT,
    WD10_REG_SECTOR,
    WD10_REG_CYL_LO,
    WD10_REG_CYL_HI,
    WD10_REG_SDH,
    WD10_REG_STATUS
};

typedef struct {
    uint8_t variant; /* 0 = WD1010, 1 = WD2010 */
    int     units;
    struct {
        int      hdd_num; /* negative: no drive on this unit */
        uint32_t spt;
        uint32_t hpc;
        uint32_t tracks;
    } unit[MFM_NUM];
    void (*intrq)(void *priv, int level); /* NULL: INTRQ not connected */
    void *priv;
} wd10_config_t;

extern wd10_t  *wd10_create(const wd10_config_t *cfg);
extern void    wd10_destroy(wd10_t *dev);
extern void    wd10_reset(wd10_t *dev);
extern void    wd10_write(wd10_t *dev, int reg, uint8_t val);
extern uint8_t wd10_read(wd10_t *dev, int reg);
extern uint8_t *wd10_buffer(wd10_t *dev);

#endif /*EMU_HDC_ST506_WD10_H*/
