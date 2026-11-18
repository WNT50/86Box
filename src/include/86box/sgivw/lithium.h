/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitions for the Lithium I/O Bus ASIC device emulation.
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 2024 Dmitry Borisov
 */
#pragma once

#define VW_LI_BRIDGE_A_IO_BASE                 0xFC000000
#define VW_LI_BRIDGE_B_IO_BASE                 0xFD000000
#define VW_LI_BRIDGE_IO_SIZE                   0x00400000
#define VW_LI_BRIDGE_IO_DECODE_MASK            0x000000FF
#define VW_LI_BRIDGE_REGS_SIZE                 0x00000100

#define VW_LI_DMA_1_IO_BASE                    0xFF001000
#define VW_LI_DMA_1_IO_SIZE                    0x00800000
#define VW_LI_DMA_1_IO_DECODE_MASK             0x007FFFFF

#define VW_LI_DMA_2_IO_BASE                    0xFF00F000
#define VW_LI_DMA_2_IO_SIZE                    0x00001000
#define VW_LI_DMA_2_IO_DECODE_MASK             0x00000FFF

#define VW_LI_DMA_3_IO_BASE                    0xFF010000
#define VW_LI_DMA_3_IO_SIZE                    0x00800000
#define VW_LI_DMA_3_IO_DECODE_MASK             0x00000FFF

extern const device_t lithium_device;
extern const device_t lithium_bridge_a_device;
extern const device_t lithium_bridge_b_device;
extern const device_t lithium_1394_device;

typedef struct li_t
{
    struct {
        uint8_t placeholder;
    } regs;
    uint64_t reset_tsc;
    mem_mapping_t dma_mappings[3];
    mem_mapping_t ieee1394_mapping;
} li_t;

void
lithium_dma_init(li_t *dev);
