/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the STMicroelectronics M29F400T flash chip
 *          (4 Mbit, 512K x8 / 256K x16, top boot).
 *
 *          The SGI Visual Workstation 320/540 boot PROM is wired in
 *          16-bit mode: command words go to word addresses 5555h/2AAAh
 *          (byte offsets AAAAh/5554h), and the firmware's flash programmer
 *          checks the autoselect signature 20h/D5h before programming.
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *          (original M29F400 device in the disean/86Box sgi_320 branch)
 *
 *          Copyright 2024-2025 Dmitry Borisov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/nvr.h>
#include <86box/machine.h>
#include <86box/flash.h>
#include <86box/plat_unused.h>

#define ST_FLASH_SIZE       0x00080000 /* 512 KiB */
#define ST_FLASH_BASE_LOW   0x00080000
#define ST_FLASH_BASE_HIGH  0xFFF80000
#define ST_FLASH_MASK       (ST_FLASH_SIZE - 1)

/* Command addresses are word addresses; the byte offsets are doubled. */
#define ST_CMD_ADDR1        0xAAAA
#define ST_CMD_ADDR2        0x5554

#define ST_CMD_UNLOCK1      0x00AA
#define ST_CMD_UNLOCK2      0x0055
#define ST_CMD_AUTOSELECT   0x0090
#define ST_CMD_PROGRAM      0x00A0
#define ST_CMD_ERASE        0x0080
#define ST_CMD_CHIP_ERASE   0x0010
#define ST_CMD_SECTOR_ERASE 0x0030
#define ST_CMD_RESET        0x00F0

#define ST_MANUFACTURER_ID  0x0020 /* STMicroelectronics */
#define ST_DEVICE_ID        0x00D5 /* M29F400T (top boot) */

#define ST_BOOT_BLOCK_START 0x7C000 /* 16 KiB boot block, write protected */

enum {
    ST_STATE_IDLE = 0,
    ST_STATE_UNLOCK1,
    ST_STATE_UNLOCK2,
    ST_STATE_PROGRAM,
    ST_STATE_ERASE1,
    ST_STATE_ERASE2,
    ST_STATE_ERASE3
};

enum {
    ST_MODE_ARRAY = 0,
    ST_MODE_AUTOSELECT
};

typedef struct st_flash_t {
    uint8_t array[ST_FLASH_SIZE];

    uint8_t state;
    uint8_t mode;
    uint8_t dirty;

    mem_mapping_t mapping_low;
    mem_mapping_t mapping_high;

    char path[1024];
} st_flash_t;

static uint8_t
st_flash_read_byte(const st_flash_t *dev, uint32_t off)
{
    if (dev->mode == ST_MODE_AUTOSELECT) {
        uint16_t id = ((off & ~1) == 0x0000) ? ST_MANUFACTURER_ID : ST_DEVICE_ID;
        return (off & 1) ? (uint8_t) (id >> 8) : (uint8_t) id;
    }
    return dev->array[off];
}

static uint8_t
st_flash_read8(uint32_t addr, void *priv)
{
    const st_flash_t *dev = (const st_flash_t *) priv;
    uint32_t          off = addr & ST_FLASH_MASK;

    return st_flash_read_byte(dev, off);
}

static uint16_t
st_flash_read16(uint32_t addr, void *priv)
{
    const st_flash_t *dev = (const st_flash_t *) priv;
    uint32_t          off = addr & ST_FLASH_MASK;

    return (uint16_t) st_flash_read_byte(dev, off) |
           ((uint16_t) st_flash_read_byte(dev, off + 1) << 8);
}

static uint32_t
st_flash_read32(uint32_t addr, void *priv)
{
    const st_flash_t *dev = (const st_flash_t *) priv;
    uint32_t          off = addr & ST_FLASH_MASK;

    return (uint32_t) st_flash_read_byte(dev, off) |
           ((uint32_t) st_flash_read_byte(dev, off + 1) << 8) |
           ((uint32_t) st_flash_read_byte(dev, off + 2) << 16) |
           ((uint32_t) st_flash_read_byte(dev, off + 3) << 24);
}

static void
st_flash_sector_erase(st_flash_t *dev, uint32_t off)
{
    static const struct {
        uint32_t start;
        uint32_t size;
    } blocks[] = {
        { 0x00000, 0x10000 }, { 0x10000, 0x10000 }, { 0x20000, 0x10000 },
        { 0x30000, 0x10000 }, { 0x40000, 0x10000 }, { 0x50000, 0x10000 },
        { 0x60000, 0x10000 }, { 0x70000, 0x08000 }, { 0x78000, 0x02000 },
        { 0x7A000, 0x02000 }, { 0x7C000, 0x04000 }
    };
    uint32_t i;

    for (i = 0; i < (sizeof(blocks) / sizeof(blocks[0])); i++) {
        if ((off >= blocks[i].start) && (off < (blocks[i].start + blocks[i].size))) {
            /* The boot block is protected on the Visual Workstation. */
            if (blocks[i].start < ST_BOOT_BLOCK_START) {
                memset(&dev->array[blocks[i].start], 0xFF, blocks[i].size);
                dev->dirty = 1;
            }
            return;
        }
    }
}

static void
st_flash_chip_erase(st_flash_t *dev)
{
    memset(&dev->array[0], 0xFF, ST_BOOT_BLOCK_START);
    dev->dirty = 1;
}

static void
st_flash_write_word(st_flash_t *dev, uint32_t addr, uint16_t val)
{
    uint32_t off = addr & ST_FLASH_MASK;

    switch (dev->state) {
        case ST_STATE_IDLE:
            if ((val == ST_CMD_UNLOCK1) && (off == ST_CMD_ADDR1))
                dev->state = ST_STATE_UNLOCK1;
            else if (val == ST_CMD_RESET)
                dev->mode = ST_MODE_ARRAY;
            break;

        case ST_STATE_UNLOCK1:
            if ((val == ST_CMD_UNLOCK2) && (off == ST_CMD_ADDR2))
                dev->state = ST_STATE_UNLOCK2;
            else
                dev->state = ST_STATE_IDLE;
            break;

        case ST_STATE_UNLOCK2:
            if (off != ST_CMD_ADDR1) {
                dev->state = ST_STATE_IDLE;
                break;
            }

            switch (val) {
                case ST_CMD_AUTOSELECT:
                    dev->mode  = ST_MODE_AUTOSELECT;
                    dev->state = ST_STATE_IDLE;
                    break;
                case ST_CMD_PROGRAM:
                    dev->state = ST_STATE_PROGRAM;
                    break;
                case ST_CMD_ERASE:
                    dev->state = ST_STATE_ERASE1;
                    break;
                case ST_CMD_RESET:
                    dev->mode  = ST_MODE_ARRAY;
                    dev->state = ST_STATE_IDLE;
                    break;
                default:
                    dev->state = ST_STATE_IDLE;
                    break;
            }
            break;

        case ST_STATE_PROGRAM:
            /* Program one word; the boot block stays write protected. */
            if (off < ST_BOOT_BLOCK_START) {
                dev->array[off]     = (uint8_t) val;
                dev->array[off + 1] = (uint8_t) (val >> 8);
                dev->dirty          = 1;
            }
            dev->state = ST_STATE_IDLE;
            break;

        case ST_STATE_ERASE1:
            if ((val == ST_CMD_UNLOCK1) && (off == ST_CMD_ADDR1))
                dev->state = ST_STATE_ERASE2;
            else
                dev->state = ST_STATE_IDLE;
            break;

        case ST_STATE_ERASE2:
            if ((val == ST_CMD_UNLOCK2) && (off == ST_CMD_ADDR2))
                dev->state = ST_STATE_ERASE3;
            else
                dev->state = ST_STATE_IDLE;
            break;

        case ST_STATE_ERASE3:
            if (val == ST_CMD_CHIP_ERASE)
                st_flash_chip_erase(dev);
            else if (val == ST_CMD_SECTOR_ERASE)
                st_flash_sector_erase(dev, off);
            dev->state = ST_STATE_IDLE;
            break;

        default:
            dev->state = ST_STATE_IDLE;
            break;
    }
}

static void
st_flash_write8(uint32_t addr, uint8_t val, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;

    /* Commands are issued as words; a byte write only fills one lane. */
    st_flash_write_word(dev, addr & ~1, (addr & 1) ? ((uint16_t) val << 8) : val);
}

static void
st_flash_write16(uint32_t addr, uint16_t val, void *priv)
{
    st_flash_write_word((st_flash_t *) priv, addr, val);
}

static void
st_flash_write32(uint32_t addr, uint32_t val, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;

    st_flash_write_word(dev, addr, (uint16_t) val);
    st_flash_write_word(dev, addr + 2, (uint16_t) (val >> 16));
}

static void
st_flash_reset(void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;

    dev->state = ST_STATE_IDLE;
    dev->mode  = ST_MODE_ARRAY;
}

static void *
st_flash_init(UNUSED(const device_t *info))
{
    st_flash_t *dev = (st_flash_t *) calloc(1, sizeof(st_flash_t));
    FILE       *fp;
    size_t      bytes = 0;

    snprintf(dev->path, sizeof(dev->path), "%s.bin",
             machine_get_internal_name_ex(machine));

    fp = nvr_fopen(dev->path, "rb");
    if (fp != NULL) {
        bytes = fread(dev->array, 1, ST_FLASH_SIZE, fp);
        fclose(fp);
    } else {
        bytes = MIN((size_t) (biosmask + 1), (size_t) ST_FLASH_SIZE);
        if (rom != NULL)
            memcpy(dev->array, rom, bytes);
    }

    if (bytes < ST_FLASH_SIZE)
        memset(&dev->array[bytes], 0xFF, ST_FLASH_SIZE - bytes);

    /* The ROM is only mapped through the high alias; the whole low 1 MiB is RAM. */

    mem_mapping_add(&dev->mapping_high, ST_FLASH_BASE_HIGH, ST_FLASH_SIZE,
                    st_flash_read8, st_flash_read16, st_flash_read32,
                    st_flash_write8, st_flash_write16, st_flash_write32,
                    dev->array, MEM_MAPPING_EXTERNAL, dev);

    /* The full 512 KiB image is mapped by this device, not by rom.c. */
    mem_mapping_disable(&bios_mapping);

    st_flash_reset(dev);

    return dev;
}

static void
st_flash_close(void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;
    FILE       *fp;

    fp = nvr_fopen(dev->path, "wb");
    if (fp != NULL) {
        fwrite(dev->array, ST_FLASH_SIZE, 1, fp);
        fclose(fp);
    }

    free(dev);
}

const device_t st_flash_29f400t_device = {
    .name          = "ST M29F400T Flash BIOS",
    .internal_name = "st_flash_29f400t",
    .flags         = 0,
    .local         = 0,
    .init          = st_flash_init,
    .close         = st_flash_close,
    .reset         = st_flash_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
