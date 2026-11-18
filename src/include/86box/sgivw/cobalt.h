/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitions for the Cobalt I/O ASIC device emulation.
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 2024 Dmitry Borisov
 */
#pragma once

/* 100 MHz */
#define VW_COBALT_CLOCK_FREQ    100000000

#define VW_CO_CPU_IO_BASE                   0xC2000000
#define VW_CO_CPU_IO_SIZE                   0x02000000
#define VW_CO_CPU_IO_DECODE_MASK            0x0000007C
#define VW_CO_CPU_REGS_SIZE                 0x00000080

#define VW_CO_CPU_REG_00                    (0x00 / 4)
#define VW_CO_CPU_REG_04                    (0x04 / 4)
#define VW_CO_CPU_REG_REVISION              (0x08 / 4)
#define VW_CO_CPU_REG_CTRL                  (0x10 / 4)
#define VW_CO_CPU_REG_18                    (0x18 / 4)
#define VW_CO_CPU_REG_28                    (0x28 / 4)
#define VW_CO_CPU_REG_TIMER_AUTO_RELOAD     (0x30 / 4)
#define VW_CO_CPU_REG_TIMER_VALUE           (0x38 / 4)
#define VW_CO_CPU_REG_40                    (0x40 / 4)
#define VW_CO_CPU_REG_48                    (0x48 / 4)

#define VW_CO_CPU_REV_A4   0xA4
#define VW_CO_CPU_REV_A5   0xA5

#define VW_CO_CPU_START_TIMER   0x8

/* ****************************************************************************/

#define VW_CO_APIC_IO_BASE                   0xC4000000
#define VW_CO_APIC_IO_SIZE                   0x02000000
#define VW_CO_APIC_IO_DECODE_MASK            0x00000FFF // TODO
#define VW_CO_APIC_REGS_SIZE                 0x00001000

#define VW_CO_APIC_IRQ_DISABLED              0x00010000

/* ****************************************************************************/

#define VW_CO_MEM_IO_BASE                   0xC6000000
#define VW_CO_MEM_IO_SIZE                   0x02000000
#define VW_CO_MEM_IO_DECODE_MASK            0x000000FC
#define VW_CO_MEM_REGS_SIZE                 0x00000100

#define VW_CO_MEM_REG_RAM_BUS_CTRL       (0x00 / 4)
#define VW_CO_MEM_REG_TIMER_AUTO_RELOAD  (0x08 / 4)
#define VW_CO_MEM_REG_TIMER_VALUE        (0x10 / 4)
#define VW_CO_MEM_REG_ERROR_STATUS       (0x18 / 4)
#define VW_CO_MEM_REG_38                 (0x38 / 4)
#define VW_CO_MEM_REG_40                 (0x40 / 4)
#define VW_CO_MEM_REG_DIMM_STATUS_CTRL   (0x48 / 4)
#define VW_CO_MEM_REG_BANK_A_128_CTRL    (0x50 / 4)
#define VW_CO_MEM_REG_BANK_A_256_CTRL    (0x58 / 4)
#define VW_CO_MEM_REG_BANK_A_384_CTRL    (0x60 / 4)
#define VW_CO_MEM_REG_BANK_A_512_CTRL    (0x68 / 4)
#define VW_CO_MEM_REG_BANK_B_128_CTRL    (0x70 / 4)
#define VW_CO_MEM_REG_BANK_B_256_CTRL    (0x78 / 4)
#define VW_CO_MEM_REG_BANK_B_384_CTRL    (0x80 / 4)
#define VW_CO_MEM_REG_BANK_B_512_CTRL    (0x88 / 4)
#define VW_CO_MEM_REG_BANK_C_128_CTRL    (0x90 / 4)
#define VW_CO_MEM_REG_BANK_C_256_CTRL    (0x98 / 4)
#define VW_CO_MEM_REG_BANK_C_384_CTRL    (0xA0 / 4)
#define VW_CO_MEM_REG_BANK_C_512_CTRL    (0xA8 / 4)
#define VW_CO_MEM_REG_BANK_D_128_CTRL    (0xB0 / 4)
#define VW_CO_MEM_REG_BANK_D_256_CTRL    (0xB8 / 4)
#define VW_CO_MEM_REG_BANK_D_384_CTRL    (0xC0 / 4)
#define VW_CO_MEM_REG_BANK_D_512_CTRL    (0xC8 / 4)

#define VW_CO_MEM_STATUS_CLEAR             0x00000000

/* ****************************************************************************/

/* For some reason, this 4-byte only register is spread over 32 MB of MMIO space */
#define VW_CO_GFX_INT_STATUS_IO_BASE                   0xCA000000
#define VW_CO_GFX_INT_STATUS_IO_SIZE                   0x02000000
#define VW_CO_GFX_INT_STATUS_IO_DECODE_MASK            0x00000004

/* ****************************************************************************/

typedef struct co_t
{
    mem_mapping_t cpu_mapping;
    mem_mapping_t apic_mapping;
    mem_mapping_t mem_mapping;
    mem_mapping_t gfx_mapping;
    mem_mapping_t gfx_mapping_interrupt_status;
    struct {
        uint32_t regs[VW_CO_CPU_REGS_SIZE / sizeof(uint32_t)];
        uint64_t last_timer_tsc;
        pc_timer_t countdown_timer;
    } cpu;
    struct {
        uint32_t regs[VW_CO_APIC_REGS_SIZE / sizeof(uint32_t)];
    } apic;
    struct {
        uint32_t regs[VW_CO_MEM_REGS_SIZE / sizeof(uint32_t)];
        pc_timer_t countdown_timer;
    } mem;
    struct {
        uint32_t interrupt_status;
    } gfx;
} co_t;

extern const device_t cobalt_device;

void
cobalt_apic_init(co_t *dev);

void
cobalt_cpu_init(co_t *dev);

void
cobalt_gfx_init(co_t *dev);

void
cobalt_mem_init(co_t *dev);
