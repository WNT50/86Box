/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Hardware definitions for the Arsenic Display ASIC device.
 *
 * Authors: Jeffrey Newquist, <newquist@engr.sgi.som>
 *          Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 1999 Jeffrey Newquist
 *          Copyright 2024 Dmitry Borisov
 */
#pragma once

#define VW_ARS_IO_BASE                0xD0000000
#define VW_ARS_IO_SIZE                0x01000000
#define VW_ARS_IO_DECODE_MASK         0x000FFFFC
#define VW_ARS_IO_DECODE_MAX          0x00080020

/* These definitions derived from the Linux dbe driver source code written by Jeffrey Newquist */
#define VW_ARS_REG_GENERAL_CTRL           (0x00000000 / 4)
#define VW_ARS_REG_DOT_CLOCK              (0x00000004 / 4)
#define VW_ARS_REG_CRT_I2C_CTRL           (0x00000008 / 4)
#define VW_ARS_REG_SYSCLK_CTRL            (0x0000000C / 4)
#define VW_ARS_REG_LCD_I2C_CTRL           (0x00000010 / 4)
#define VW_ARS_REG_ID                     (0x00000014 / 4)
#define VW_ARS_REG_POWER_CONFIG           (0x00000018 / 4)
#define VW_ARS_REG_BIST                   (0x0000001C / 4)
#define VW_ARS_REG_RETRACE_POS            (0x00010000 / 4)
#define VW_ARS_REG_RETRACE_POS_MAX        (0x00010004 / 4)
#define VW_ARS_REG_RETRACE_VSYNC          (0x00010008 / 4)
#define VW_ARS_REG_RETRACE_HSYNC          (0x0001000C / 4)
#define VW_ARS_REG_RETRACE_VBLANK         (0x00010010 / 4)
#define VW_ARS_REG_RETRACE_HBLANK         (0x00010014 / 4)
#define VW_ARS_REG_RETRACE_CTRL           (0x00010018 / 4)
#define VW_ARS_REG_RETRACE_CLK            (0x0001001C / 4)
#define VW_ARS_REG_RETRACE_INTR_CTRL_1    (0x00010020 / 4)
#define VW_ARS_REG_RETRACE_INTR_CTRL_2    (0x00010024 / 4)
#define VW_ARS_REG_LCD_HDRV               (0x00010028 / 4)
#define VW_ARS_REG_LCD_VDRV               (0x0001002C / 4)
#define VW_ARS_REG_LCD_DATA_ENABLE        (0x00010030 / 4)
#define VW_ARS_REG_RETRACE_HPIX_ENABLE    (0x00010034 / 4)
#define VW_ARS_REG_RETRACE_VPIX_ENABLE    (0x00010038 / 4)
#define VW_ARS_REG_RETRACE_HCOLOR_MAP     (0x0001003C / 4)
#define VW_ARS_REG_RETRACE_VCOLOR_MAP     (0x00010040 / 4)
#define VW_ARS_REG_DID_START_POS          (0x00010044 / 4)
#define VW_ARS_REG_CURSOR_START_POS       (0x00010048 / 4)
#define VW_ARS_REG_VC_START_POS           (0x0001004C / 4)
#define VW_ARS_REG_OVERLAY_PARAMS         (0x00020000 / 4)
#define VW_ARS_REG_OVERLAY_STATUS         (0x00020004 / 4)
#define VW_ARS_REG_OVERLAY_CTRL           (0x00020008 / 4)
#define VW_ARS_REG_FB_PARAMS              (0x00030000 / 4)
#define VW_ARS_REG_FB_HEIGHT              (0x00030004 / 4)
#define VW_ARS_REG_FB_STATUS              (0x00030008 / 4)
#define VW_ARS_REG_FB_CTRL                (0x0003000C / 4)
#define VW_ARS_REG_DID_STATUS             (0x00040000 / 4)
#define VW_ARS_REG_DID_CTRL               (0x00040004 / 4)
#define VW_ARS_REG_MODE_REGS_START        (0x00048000 / 4)
#define VW_ARS_REG_MODE_REGS_END          (0x0004807C / 4)
#define VW_ARS_REG_COLOR_MAP_START        (0x00050000 / 4)
#define VW_ARS_REG_COLOR_MAP_END          (0x00055FFC / 4)
#define VW_ARS_REG_COLOR_MAP_FIFO_STATUS  (0x00058000 / 4)
#define VW_ARS_REG_GAMMA_MAP_START        (0x00060000 / 4)
#define VW_ARS_REG_GAMMA_MAP_END          (0x000603FC / 4)
#define VW_ARS_REG_GAMMA_MAP_10_START     (0x00068000 / 4)
#define VW_ARS_REG_GAMMA_MAP_10_END       (0x00068FFC / 4)
#define VW_ARS_REG_CURSOR_POS             (0x00070000 / 4)
#define VW_ARS_REG_CURSOR_CTRL            (0x00070004 / 4)
#define VW_ARS_REG_CURSOR_MAP_START       (0x00070008 / 4)
#define VW_ARS_REG_CURSOR_MAP_END         (0x00070010 / 4)
#define VW_ARS_REG_CURSOR_DATA_START      (0x00078000 / 4)
#define VW_ARS_REG_CURSOR_DATA_END        (0x000780FC / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_0   (0x00080000 / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_1   (0x00080004 / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_2   (0x00080008 / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_3   (0x0008000C / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_4   (0x00080010 / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_5   (0x00080014 / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_6   (0x00080018 / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_7   (0x0008001C / 4)
#define VW_ARS_REG_VIDEO_CAPTURE_CTRL_8   (0x00080020 / 4)

#define VW_ARS_MODE_REGS_SIZE             (0x80 / 4)
#define VW_ARS_COLOR_MAP_REGS_SIZE        (0x6000 / 4)
#define VW_ARS_COLOR_GAMMA_MAP_SIZE       (0x400 / 4)
#define VW_ARS_COLOR_GAMMA_MAP_10_SIZE    (0x1000 / 4)
#define VW_ARS_COLOR_CURSOR_DATA_SIZE     (0x100 / 4)

#define VW_ARS_I2C_SDA_LOW     0x01
#define VW_ARS_I2C_SCL_LOW     0x02
