/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitions for the Lithium IEEE 1394 Controller.
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 2024 Dmitry Borisov
 */
#pragma once

#define VW_LI_1394_IO_BASE                     0xFE000000
#define VW_LI_1394_IO_SIZE                     0x00E00000
#define VW_LI_1394_IO_DECODE_MASK              0x00000FFC
#define VW_LI_1394_REGS_SIZE                   0x00001000

#define VW_LI_1394_REG_000                     (0x000 / 4)
#define VW_LI_1394_REG_010                     (0x010 / 4)
#define VW_LI_1394_REG_PHY_CTRL_0              (0x018 / 4)
#define VW_LI_1394_REG_020                     (0x020 / 4)
#define VW_LI_1394_REG_028                     (0x028 / 4)
#define VW_LI_1394_REG_030                     (0x030 / 4)
#define VW_LI_1394_REG_038                     (0x038 / 4)

#define VW_LI_1394_REG_040                     (0x040 / 4)
#define VW_LI_1394_REG_050                     (0x050 / 4)
#define VW_LI_1394_REG_PHY_CTRL_1              (0x058 / 4)
#define VW_LI_1394_REG_060                     (0x060 / 4)
#define VW_LI_1394_REG_068                     (0x068 / 4)
#define VW_LI_1394_REG_070                     (0x070 / 4)
#define VW_LI_1394_REG_078                     (0x078 / 4)

#define VW_LI_1394_REG_080                     (0x080 / 4)
#define VW_LI_1394_REG_090                     (0x090 / 4)
#define VW_LI_1394_REG_PHY_CTRL_2              (0x098 / 4)
#define VW_LI_1394_REG_0A0                     (0x0A0 / 4)
#define VW_LI_1394_REG_0A8                     (0x0A8 / 4)
#define VW_LI_1394_REG_0B0                     (0x0B0 / 4)
#define VW_LI_1394_REG_0B8                     (0x0B8 / 4)

#define VW_LI_1394_REG_0C0                     (0x0C0 / 4)
#define VW_LI_1394_REG_0D0                     (0x0D0 / 4)
#define VW_LI_1394_REG_PHY_CTRL_3              (0x0D8 / 4)
#define VW_LI_1394_REG_0E0                     (0x0E0 / 4)
#define VW_LI_1394_REG_0E8                     (0x0E8 / 4)
#define VW_LI_1394_REG_0F0                     (0x0F0 / 4)
#define VW_LI_1394_REG_0F8                     (0x0F8 / 4)

#define VW_LI_1394_REG_108                     (0x108 / 4)
#define VW_LI_1394_REG_110                     (0x110 / 4)
#define VW_LI_1394_REG_128                     (0x128 / 4)
#define VW_LI_1394_REG_130                     (0x130 / 4)
#define VW_LI_1394_REG_148                     (0x148 / 4)
#define VW_LI_1394_REG_150                     (0x150 / 4)
#define VW_LI_1394_REG_168                     (0x168 / 4)
#define VW_LI_1394_REG_170                     (0x170 / 4)
#define VW_LI_1394_REG_188                     (0x188 / 4)
#define VW_LI_1394_REG_190                     (0x190 / 4)
#define VW_LI_1394_REG_1A8                     (0x1A8 / 4)
#define VW_LI_1394_REG_1B0                     (0x1B0 / 4)
#define VW_LI_1394_REG_1C8                     (0x1C8 / 4)
#define VW_LI_1394_REG_1D0                     (0x1D0 / 4)
#define VW_LI_1394_REG_1E8                     (0x1E8 / 4)
#define VW_LI_1394_REG_1F0                     (0x1F0 / 4)

#define VW_LI_1394_REG_200                     (0x200 / 4)
#define VW_LI_1394_REG_280                     (0x280 / 4)
#define VW_LI_1394_REG_300                     (0x300 / 4)
#define VW_LI_1394_REG_380                     (0x380 / 4)
#define VW_LI_1394_REG_400                     (0x400 / 4)
#define VW_LI_1394_REG_480                     (0x480 / 4)
#define VW_LI_1394_REG_500                     (0x500 / 4)
#define VW_LI_1394_REG_580                     (0x580 / 4)
#define VW_LI_1394_REG_600                     (0x600 / 4)
#define VW_LI_1394_REG_680                     (0x680 / 4)
#define VW_LI_1394_REG_700                     (0x700 / 4)
#define VW_LI_1394_REG_780                     (0x780 / 4)

/*
 * PHY Control register
 */
#define VW_LI_1394_PHY_CTRL_WRITE_DATA_MASK    0x000000FF
#define VW_LI_1394_PHY_CTRL_REG_ADDR_MASK      0x00000F00
#define VW_LI_1394_PHY_CTRL_WRITE_ACTION       0x00004000
#define VW_LI_1394_PHY_CTRL_READ_ACTION        0x00008000
#define VW_LI_1394_PHY_CTRL_READ_DATA_MASK     0x00FF0000
#define VW_LI_1394_PHY_CTRL_READ_ADDR_MASK     0x0F000000
#define VW_LI_1394_PHY_CTRL_READ_DONE          0x80000000

#define VW_LI_1394_PHY_CTRL_REG_ADDR_SHIFT     8
#define VW_LI_1394_PHY_CTRL_READ_DATA_SHIFT    16
#define VW_LI_1394_PHY_CTRL_READ_ADDR_SHIFT    24

