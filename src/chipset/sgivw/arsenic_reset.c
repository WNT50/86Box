/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Arsenic Display ASIC device initialization.
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 2024 Dmitry Borisov
 */
#include <86box/sgivw/vid_sgi_arsenic.h>
#include <86box/sgivw/vid_sgi_arsenic_reset.h>

void
arsenic_reset_hard(ars_t *dev)
{
    memset(dev->regs, 0, sizeof(dev->regs));

    dev->regs[VW_ARS_REG_GENERAL_CTRL] = 0x23FFFFE2;
    dev->regs[VW_ARS_REG_ID] = 0x00000666;
    dev->regs[VW_ARS_REG_POWER_CONFIG] = 0x000000F7;
    dev->regs[VW_ARS_REG_RETRACE_POS] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_POS_MAX] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_VSYNC] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_HSYNC] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_VBLANK] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_HBLANK] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_CTRL] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_CLK] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_INTR_CTRL_1] = 0x80FFFFFF;
    dev->regs[VW_ARS_REG_RETRACE_INTR_CTRL_2] = 0x80FFFFFF;
    dev->regs[VW_ARS_REG_LCD_HDRV] = 0x80000000;
    dev->regs[VW_ARS_REG_LCD_VDRV] = 0x80000000;
    dev->regs[VW_ARS_REG_LCD_DATA_ENABLE] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_HPIX_ENABLE] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_VPIX_ENABLE] = 0x80000000;
    dev->regs[VW_ARS_REG_RETRACE_HCOLOR_MAP] = 0x80000FFF;
    dev->regs[VW_ARS_REG_RETRACE_VCOLOR_MAP] = 0x80000FFF;
    dev->regs[VW_ARS_REG_DID_START_POS] = 0x80000000;
    dev->regs[VW_ARS_REG_CURSOR_START_POS] = 0x80000000;
    dev->regs[VW_ARS_REG_VC_START_POS] = 0x80000000;
    dev->regs[VW_ARS_REG_VIDEO_CAPTURE_CTRL_2] = 0x00000006;
    dev->regs[VW_ARS_REG_VIDEO_CAPTURE_CTRL_3] = 0x00000010;

    memcpy(&dev->regs[VW_ARS_REG_MODE_REGS_START], ars_default_mode_regs, sizeof(ars_default_mode_regs));
    memcpy(&dev->regs[VW_ARS_REG_COLOR_MAP_START], ars_default_color_map, sizeof(ars_default_color_map));
    memcpy(&dev->regs[VW_ARS_REG_GAMMA_MAP_START], ars_default_gamma_map, sizeof(ars_default_gamma_map));
    memcpy(&dev->regs[VW_ARS_REG_GAMMA_MAP_10_START], ars_default_gamma_map10, sizeof(ars_default_gamma_map10));
    memcpy(&dev->regs[VW_ARS_REG_CURSOR_DATA_START], ars_default_cursor_data, sizeof(ars_default_cursor_data));
}
