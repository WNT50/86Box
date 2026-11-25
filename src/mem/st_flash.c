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
#include "cpu.h"
#include <86box/flash.h>
#include <86box/plat_unused.h>

#ifdef ENABLE_ST_FLASH_LOG
int st_flash_do_log = ENABLE_ST_FLASH_LOG;

static void
st_flash_log(const char *fmt, ...)
{
    va_list ap;

    if (st_flash_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define st_flash_log(fmt, ...)
#endif

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
    ST_STATE_ERASE3,
    ST_STATE_ERASE_MORE
};

enum {
    ST_MODE_ARRAY = 0,
    ST_MODE_AUTOSELECT
};

/* Operation state machine: what the P/E.C. is doing right now, as opposed to
   the command state machine above, which tracks how far into a command cycle
   the writes are. */
enum {
    ST_OP_NONE = 0,
    ST_OP_PROGRAM,
    ST_OP_ERASE,
    ST_OP_ERASE_CHIP
};

/* The updater polls the block in a tight loop with no delay and gives up
   after a handful of reads, so the P/E.C. has to report completion inside
   that budget.  A wall-clock erase (about a second on the real part) would
   never be seen; the operation therefore advances on polled reads. */
#define ST_OP_BUSY_POLLS  32
#define ST_OP_TIMER_POLLS 2
#define ST_OP_ERR_POLLS   2

typedef struct st_flash_t {
    uint8_t array[ST_FLASH_SIZE];

    uint8_t  state;      /* command state machine */
    uint8_t  op;         /* operation state machine */
    uint8_t  op_polls;
    uint8_t  op_reads;
    uint8_t  op_toggle;
    uint8_t  err_polls;
    uint8_t  mode;
    uint8_t  dirty;
    uint8_t  err;
    uint16_t op_val;
    uint32_t op_off;

    mem_mapping_t mapping_low;
    mem_mapping_t mapping_high;

    uint8_t shadow[0x10000];

    char path[1024];
} st_flash_t;

static void st_flash_op_finish(st_flash_t *dev);

static uint8_t
st_flash_read_byte(st_flash_t *dev, uint32_t off)
{
    if (dev->mode == ST_MODE_AUTOSELECT) {
        /* Autoselect answers three different reads, keyed by A0/A1 of the
           word address (M29F400T, note 6): 00 manufacturer, 10 device,
           01 the protection status of the block that holds the address. */
        uint32_t wa = off >> 1;
        uint32_t a0 = wa & 1;
        uint32_t a1 = (wa >> 1) & 1;
        uint16_t val;

        if (!a0 && !a1)
            val = ST_MANUFACTURER_ID;
        else if (a0 && !a1)
            val = ST_DEVICE_ID;
        else if (!a0 && a1)
            val = (off >= ST_BOOT_BLOCK_START) ? 0x0001 : 0x0000;
        else
            val = 0x0000;

        st_flash_log("FLASH: autoselect %06X -> %04X\n", off, val);
        return (off & 1) ? (uint8_t) (val >> 8) : (uint8_t) val;
    }
    /* While the P/E.C. runs, data polling is what the host sees: DQ7 = 0 and
       DQ6 toggling.  Once it ends the array answers again. */
    if (dev->op != ST_OP_NONE) {
        /* DQ3 says the erase timer has started, DQ2 toggles while the block
           is being erased, and during programming DQ7 carries the complement
           of the bit being programmed.  Reporting zero here reads as "the
           command never took effect". */
        uint16_t val  = 0x0000;
        uint8_t  byte;

        /* DQ3 is the erase timer: it stays low for a short while after the
           command and only then reports that the timer has started.  The
           firmware reads it once right after the command and treats a high
           value there as "the command never took". */
        if (dev->op_reads > ST_OP_TIMER_POLLS)
            val |= 0x0008;

        if (dev->op_toggle & 1) {
            val |= 0x0040;
            if ((dev->op == ST_OP_ERASE) || (dev->op == ST_OP_ERASE_CHIP))
                val |= 0x0004;
        }

        if ((dev->op == ST_OP_PROGRAM) && !(dev->op_val & 0x0080))
            val |= 0x0080;

        byte = (off & 1) ? (uint8_t) (val >> 8) : (uint8_t) val;

        if (!(off & 1)) {
            dev->op_toggle ^= 1;
            if (dev->op_reads < 0xFF)
                dev->op_reads++;
            if (!--dev->op_polls)
                st_flash_op_finish(dev);
        }

        if ((off < 0x20) || ((off >= 0x70000) && (off < 0x80000)))
            st_flash_log("FLASH: read %06X -> %02X (busy)\n", off, byte);
        return byte;
    }

    /* A failed operation leaves DQ5 set long enough to be read back. */
    if (dev->err_polls) {
        uint16_t val  = 0x00A0 | 0x0040 | 0x0008 | 0x0004;
        uint8_t  byte = (off & 1) ? (uint8_t) (val >> 8) : (uint8_t) val;

        if (!(off & 1))
            dev->err_polls--;

        if ((off < 0x20) || ((off >= 0x70000) && (off < 0x80000)))
            st_flash_log("FLASH: read %06X -> %02X (error)\n", off, byte);
        return byte;
    }

    if ((off < 0x20) || ((off >= 0x70000) && (off < 0x80000))) {
        static int read_log_n;
        if (read_log_n < 400) {
            read_log_n++;
            st_flash_log("FLASH: read %06X -> %02X\n", off, dev->array[off]);
        }
    }

    return dev->array[off];
}

static uint8_t
st_flash_read8(uint32_t addr, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;
    uint32_t          off = addr & ST_FLASH_MASK;

    return st_flash_read_byte(dev, off);
}

static uint16_t
st_flash_read16(uint32_t addr, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;
    uint32_t          off = addr & ST_FLASH_MASK;

    return (uint16_t) st_flash_read_byte(dev, off) |
           ((uint16_t) st_flash_read_byte(dev, off + 1) << 8);
}

static uint32_t
st_flash_read32(uint32_t addr, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;
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

    st_flash_log("FLASH: erase block at %06X\n", off);

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
    st_flash_log("FLASH: chip erase\n");
    memset(&dev->array[0], 0xFF, ST_BOOT_BLOCK_START);
    dev->dirty = 1;
}

static void
st_flash_op_finish(st_flash_t *dev)
{
    switch (dev->op) {
        case ST_OP_PROGRAM: {
            /* Programming can only clear bits.  A 0 -> 1 request, or a
               program aimed at the protected block, fails and sets DQ5. */
            if (dev->op_off < ST_BOOT_BLOCK_START) {
                uint16_t old = (uint16_t) dev->array[dev->op_off] |
                               ((uint16_t) dev->array[dev->op_off + 1] << 8);

                if (dev->op_val & ~old) {
                    st_flash_log("FLASH: program %06X %04X over %04X fails\n",
                                 dev->op_off, dev->op_val, old);
                    dev->err = 1;
                } else {
                    st_flash_log("FLASH: program %06X <- %04X\n",
                                 dev->op_off, dev->op_val);
                    dev->array[dev->op_off]     = (uint8_t) (old & dev->op_val);
                    dev->array[dev->op_off + 1] = (uint8_t) ((old & dev->op_val) >> 8);
                    dev->dirty                  = 1;
                }
            } else
                dev->err = 1;
            break;
        }

        case ST_OP_ERASE:
            if (dev->op_off >= ST_BOOT_BLOCK_START)
                dev->err = 1;
            else {
                st_flash_sector_erase(dev, dev->op_off);
                dev->err = 0;
            }
            break;

        case ST_OP_ERASE_CHIP:
            st_flash_chip_erase(dev);
            dev->err = 0;
            break;

        default:
            break;
    }

    dev->op = ST_OP_NONE;

    if (dev->err)
        dev->err_polls = ST_OP_ERR_POLLS;
}

static void
st_flash_start_op(st_flash_t *dev, uint8_t op, uint32_t off, uint16_t val)
{
    dev->op        = op;
    dev->op_off    = off;
    dev->op_val    = val;
    dev->op_polls  = ST_OP_BUSY_POLLS;
    dev->op_reads  = 0;
    dev->op_toggle = 1;
    dev->err       = 0;
    dev->err_polls = 0;
}

static void
st_flash_write_word(st_flash_t *dev, uint32_t addr, uint16_t val)
{
    uint32_t off = addr & ST_FLASH_MASK;

#ifdef ENABLE_ST_FLASH_LOG
    /* Report the first command write together with the code around it: the
       flash driver is not in the image that can be read from the ROM file. */
    {
        static int first = 1;
        if (first) {
            uint32_t lin = cpu_state.seg_cs.base + cpu_state.pc;
            int      i;

            first = 0;
            st_flash_log("FLASH: first write from CS base %08X, pc %08X, linear %08X\n",
                         cpu_state.seg_cs.base, cpu_state.pc, lin);
            for (i = 0; i < 64; i++) {
                st_flash_log("FLASH: code %08X: %02X\n", lin - 32 + i,
                             mem_readb_phys(lin - 32 + i));
            }
        }
    }
#endif

    /* The part ignores writes while the P/E.C. runs. */
    if (dev->op != ST_OP_NONE)
        return;

    st_flash_log("FLASH: write %06X (off %06X) <- %04X (state %i)\n",
                 addr, off, val, dev->state);

#ifdef ENABLE_ST_FLASH_LOG
    /* Which routine issued the command, so a block that fails can be told
       from one that works. */
    if ((val == 0x0030) || (val == 0x00A0) || (val == 0x0080) || (val == 0x0010)) {
        static int cmd_log_n;
        if (cmd_log_n < 40) {
            cmd_log_n++;
            st_flash_log("FLASH: cmd %04X at %06X from %08X\n",
                         val, off, cpu_state.seg_cs.base + cpu_state.pc);
        }
    }
#endif

handle:
    switch (dev->state) {
        case ST_STATE_IDLE:
            if ((val == ST_CMD_UNLOCK1) && (off == ST_CMD_ADDR1))
                dev->state = ST_STATE_UNLOCK1;
            else if (val == ST_CMD_RESET) {
                dev->mode      = ST_MODE_ARRAY;
                dev->err       = 0;
                dev->err_polls = 0;
            }
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
                    dev->mode      = ST_MODE_ARRAY;
                    dev->err       = 0;
                    dev->err_polls = 0;
                    dev->state     = ST_STATE_IDLE;
                    break;
                default:
                    dev->state = ST_STATE_IDLE;
                    break;
            }
            break;

        case ST_STATE_PROGRAM:
            st_flash_start_op(dev, ST_OP_PROGRAM, off, val);
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
                st_flash_start_op(dev, ST_OP_ERASE_CHIP, off, 0);
            else if (val == ST_CMD_SECTOR_ERASE)
                st_flash_start_op(dev, ST_OP_ERASE, off, 0);
            else {
                dev->err       = 1;
                dev->err_polls = ST_OP_ERR_POLLS;
            }
            /* Further 30h writes within the erase timeout erase more
               blocks without repeating the command prefix (table 8). */
            dev->state = ST_STATE_ERASE_MORE;
            break;

        case ST_STATE_ERASE_MORE:
            if (val == ST_CMD_SECTOR_ERASE) {
                st_flash_start_op(dev, ST_OP_ERASE, off, 0);
                break;
            }
            /* Not another block: this write starts a new command cycle. */
            dev->state = ST_STATE_IDLE;
            goto handle;

        default:
            dev->state = ST_STATE_IDLE;
            break;
    }
}

static uint8_t
st_shadow_read8(uint32_t addr, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;

    return dev->shadow[addr & 0xFFFF];
}

static uint16_t
st_shadow_read16(uint32_t addr, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;
    uint32_t    off = addr & 0xFFFF;

    return (uint16_t) (dev->shadow[off] | (dev->shadow[(off + 1) & 0xFFFF] << 8));
}

static uint32_t
st_shadow_read32(uint32_t addr, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;
    uint32_t    off = addr & 0xFFFF;

    return (uint32_t) dev->shadow[off] |
           ((uint32_t) dev->shadow[(off + 1) & 0xFFFF] << 8) |
           ((uint32_t) dev->shadow[(off + 2) & 0xFFFF] << 16) |
           ((uint32_t) dev->shadow[(off + 3) & 0xFFFF] << 24);
}

static void
st_shadow_write8(uint32_t addr, uint8_t val, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;

    dev->shadow[addr & 0xFFFF] = val;
}

static void
st_shadow_write16(uint32_t addr, uint16_t val, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;
    uint32_t    off = addr & 0xFFFF;

    dev->shadow[off]                 = (uint8_t) val;
    dev->shadow[(off + 1) & 0xFFFF]  = (uint8_t) (val >> 8);
}

static void
st_shadow_write32(uint32_t addr, uint32_t val, void *priv)
{
    st_flash_t *dev = (st_flash_t *) priv;
    uint32_t    off = addr & 0xFFFF;

    dev->shadow[off]                 = (uint8_t) val;
    dev->shadow[(off + 1) & 0xFFFF]  = (uint8_t) (val >> 8);
    dev->shadow[(off + 2) & 0xFFFF]  = (uint8_t) (val >> 16);
    dev->shadow[(off + 3) & 0xFFFF]  = (uint8_t) (val >> 24);
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

    /* The top 64 KiB is shadow RAM: it starts as a copy of the ROM and the
       firmware may write to it (there is no ROMCS switch on this board). */
    memcpy(dev->shadow, &dev->array[0x70000], sizeof(dev->shadow));

    mem_mapping_add(&dev->mapping_low, 0x000F0000, sizeof(dev->shadow),
                    st_shadow_read8, st_shadow_read16, st_shadow_read32,
                    st_shadow_write8, st_shadow_write16, st_shadow_write32,
                    dev->shadow, MEM_MAPPING_EXTERNAL, dev);

    mem_set_mem_state_both(0x000A0000, 0x00050000, MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);

    mem_mapping_add(&dev->mapping_high, ST_FLASH_BASE_HIGH, ST_FLASH_SIZE,
                    st_flash_read8, st_flash_read16, st_flash_read32,
                    st_flash_write8, st_flash_write16, st_flash_write32,
                    dev->array, MEM_MAPPING_EXTERNAL, dev);

    /* The full 512 KiB image is mapped by this device, not by rom.c. */
    mem_mapping_disable(&bios_mapping);
    mem_mapping_disable(&bios_high_mapping);


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
