/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Driver for the IBM PC-AT MFM/RLL Fixed Disk controller.
 *
 *          This controller was a 16bit ISA card, and it used a WD1003
 *          based design. Most cards were WD1003-WA2 or -WAH, where the
 *          -WA2 cards had a floppy controller as well (to save space.)
 *
 *          The card carries a WD1010 class controller; the chip itself
 *          (task file, command set) lives in hdc_st506_wd10.c, together
 *          with the WD2010 variant. This file only decodes I/O, wires
 *          IRQ14 and loads the drive images.
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

typedef struct {
    mfm_t *mfm;
    int    hdd_num[MFM_NUM];
} st506_at_t;

static void *
st506_at_init(const device_t *info)
{
    st506_at_t *board;
    int         c = 0;

    board = calloc(1, sizeof(st506_at_t));

    for (int i = 0; i < MFM_NUM; i++)
        board->hdd_num[i] = -1;

    board->mfm = wd10_init(info->local);

    for (uint8_t d = 0; d < HDD_NUM; d++) {
        if ((hdd[d].bus_type == HDD_BUS_MFM) && (hdd[d].mfm_channel < MFM_NUM)) {
            if (!hdd_image_load(d))
                continue;

            wd10_attach(board->mfm, hdd[d].mfm_channel, d, hdd[d].spt, hdd[d].hpc, hdd[d].tracks);
            board->hdd_num[hdd[d].mfm_channel] = d;

            if (++c >= MFM_NUM)
                break;
        }
    }

    io_sethandler(0x01f0, 1,
                  wd10_read, wd10_readw, NULL, wd10_write, wd10_writew, NULL, board->mfm);
    io_sethandler(0x01f1, 7,
                  wd10_read, wd10_readw, NULL, wd10_write, wd10_writew, NULL, board->mfm);
    io_sethandler(0x03f6, 1,
                  NULL, NULL, NULL, wd10_write, NULL, NULL, board->mfm);

    return board;
}

static void
st506_at_close(void *priv)
{
    st506_at_t *board = (st506_at_t *) priv;

    if (board == NULL)
        return;

    for (int i = 0; i < MFM_NUM; i++) {
        if (board->hdd_num[i] >= 0)
            hdd_image_close(board->hdd_num[i]);
    }

    wd10_close(board->mfm);
    free(board);
}

const device_t st506_at_wd2010_device = {
    .name          = "WD2010 (MFM/RLL)",
    .internal_name = "st506_at_wd2010",
    .flags         = DEVICE_ISA16,
    .local         = WD10_VARIANT_2010,
    .init          = st506_at_init,
    .close         = st506_at_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t st506_at_wd1003_device = {
    .name          = "WD1003-WAH (MFM/RLL)",
    .internal_name = "st506_at",
    .flags         = DEVICE_ISA16,
    .local         = WD10_VARIANT_1010,
    .init          = st506_at_init,
    .close         = st506_at_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
