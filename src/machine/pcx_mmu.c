/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-X MMU (I/O ports 0x8000-0x8fff).
 *
 *          Layout, taken from MAME's pcd.cpp:
 *            0x8000        control: bit 5 = translate, bits 0-4 = which
 *                          32-entry table group the register file edits,
 *                          bits 3-4 also take part in the index (window)
 *            0x8200-0x83FF  entry window, one slot every 4 bytes; the slot
 *                          number is (off >> 2) & 0x1F, so the 32 distinct
 *                          entries repeat four times across the window, and
 *                          only a word access at the slot address is decoded
 *            0x8400        "table updated" latch, written/read to hand the
 *                          change over to the driver as a PIC interrupt
 */
#include <stdint.h>
#include <string.h>

#include <86box/86box.h>
#include <86box/plat_unused.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/nmi.h>
#include <86box/mem.h>
#include "cpu.h"

#include <86box/pcx_mmu.h>

#define PCX_MMU_BASE     0x8000
#define PCX_MMU_SIZE     0x1000
#define PCX_MMU_CTL      0x000
#define PCX_MMU_REG      0x200
#define PCX_MMU_ACK      0x400
#define PCX_MMU_PAGES    1024                      /* 1 MB / 1 KB */
#define PCX_MMU_PGSIZE   0x400
#define PCX_MMU_WINDOW   0x80000                   /* MAME translates 0-0x7FFFF */

typedef struct {
    uint16_t ctl;
    uint16_t pg[PCX_MMU_PAGES];
    int      type;
    int      ack;                                  /* 0x8400 latch */
    uint32_t fault;                                /* last faulted address */
} pcx_mmu_t;

static pcx_mmu_t pcx_mmu;

/* Addresses this machine does not decode at all (PC-X: 0x80000-0xFBFFF, the
   region between the RAM behind the MMU and the ROM).  MAME models it as a
   two-instruction fallback in the address map that just raises NMI; the same
   check can live here because this function already runs on every access.
   Disabled when hi <= lo. */
static uint32_t pcx_mmu_hole_lo;
static uint32_t pcx_mmu_hole_hi;

/* Table group the register file edits while ctl bit 5 is set.  MAME indexes
   the same array as ((ctl & 0x1f) << 5) | ((offset >> 2) & 0x1f). */
static int
pcx_mmu_slot(uint16_t ctl, uint16_t off)
{
    return ((ctl & 0x1f) << 5) | ((off >> 2) & 0x1f);
}

/* Page number seen by a translation for the given CPU address.  The two SINIX
   releases use 8-bit and 7-bit page numbers, with the remaining index bits
   coming from the control register -- i.e. the mapping is a banked window,
   not a flat 1 MB table.  Kept identical to MAME on purpose. */
static int
pcx_mmu_index(uint16_t ctl, uint32_t addr)
{
    uint32_t page = addr >> 10;

    if (pcx_mmu.type == PCX_MMU_SINIX12)
        return (int) ((page & 0xff) | ((ctl & 0x18) << 5));
    return (int) ((page & 0x7f) | ((ctl & 0x1c) << 5));
}

/* Returns the physical address, or 0xffffffff when the access faults.  Called
   from the memory access helpers (mem_machine_translate), so it must be cheap:
   one table read, no allocation, no logging in the normal path. */
static uint32_t
pcx_mmu_translate(uint32_t addr, int rw)
{
    uint16_t entry;
    int      idx;

    /* Undecoded space first: it is a hole whether the MMU is on, off, or not
       fitted at all -- the board decodes nothing there. */
    if ((addr >= pcx_mmu_hole_lo) && (addr < pcx_mmu_hole_hi)) {
        pcx_mmu.fault = addr;
        nmi_raise();
        return 0xffffffffu;
    }

    if ((pcx_mmu.type == PCX_MMU_NONE) || !(pcx_mmu.ctl & 0x20) || (addr >= PCX_MMU_WINDOW))
        return addr;

    idx   = pcx_mmu_index(pcx_mmu.ctl, addr);
    entry = pcx_mmu.pg[idx];

    /* entry bit 1 permits reads, bit 0 permits writes; bits 15-7 are the 1 KB
       page number.  A denied or absent entry is not an x86 exception: the
       board raises NMI, like the rest of the machine's error handling. */
    if (!(entry & (rw ? 0x0001 : 0x0002))) {
        pcx_mmu.fault = addr;
        nmi_raise();
        return 0xffffffffu;
    }

    return ((uint32_t) (entry << 3) & 0x7fc00u) | (addr & (PCX_MMU_PGSIZE - 1));
}

static uint8_t
pcx_mmu_inb(UNUSED(uint16_t port), UNUSED(void *priv))
{
    /* MAME declares only 16-bit handlers for this window: byte accesses are
       not decoded by the part. */
    return 0xff;
}

static void
pcx_mmu_outb(uint16_t port, uint8_t val, UNUSED(void *priv))
{
    uint16_t off = (uint16_t) ((port - PCX_MMU_BASE) >> 1);

    /* A byte write to the control register carries only the low half, which is
       where every control bit lives. */
    if (off == PCX_MMU_CTL)
        pcx_mmu.ctl = (uint16_t) ((pcx_mmu.ctl & 0xff00) | val);
}

static uint16_t
pcx_mmu_inw(uint16_t port, UNUSED(void *priv))
{
    uint16_t off = (uint16_t) ((port - PCX_MMU_BASE) >> 1);

    if (off == PCX_MMU_CTL)
        return pcx_mmu.ctl;

    if ((off >= PCX_MMU_REG) && (off < PCX_MMU_ACK) && !(off & 3)) {
        uint16_t entry = pcx_mmu.pg[pcx_mmu_slot(pcx_mmu.ctl, off)];

        /* The entry is written and read back through a rotated data path; the
           0xC0 is what MAME reports in the top of the first slot's read-back
           while its "table changed" latch is still set. */
        return (uint16_t) ((entry << 4) | (entry >> 12) |
                           ((pcx_mmu.ack && (off == PCX_MMU_REG)) ? 0x00c0 : 0x0000));
    }

    if (off == PCX_MMU_ACK) {
        pcx_mmu.ack = 0;
        picintc(0x0001);
        return 0;
    }

    return 0;
}

static void
pcx_mmu_outw(uint16_t port, uint16_t val, UNUSED(void *priv))
{
    uint16_t off = (uint16_t) ((port - PCX_MMU_BASE) >> 1);

    if (off == PCX_MMU_CTL) {

        /* Bit 5 turns translation on.  Nothing has to be rebuilt: the entry is
           read on every access, unlike a mapping-based model. */
        pcx_mmu.ctl = val;
        return;
    }

    if ((off >= PCX_MMU_REG) && (off < PCX_MMU_ACK) && !(off & 3)) {
        pcx_mmu.pg[pcx_mmu_slot(pcx_mmu.ctl, off)] = (uint16_t) ((val >> 4) | (val << 12));
        return;
    }

    if (off == PCX_MMU_ACK) {
        /* "The table changed": the driver is told through IRQ0 of the first
           controller rather than by polling. */
        pcx_mmu.ack = 1;
        picint(0x0001);
    }
}

void
pcx_mmu_reset(void)
{
    pcx_mmu.ctl   = 0;
    pcx_mmu.ack   = 0;
    pcx_mmu.fault = 0;
    memset(pcx_mmu.pg, 0x00, sizeof(pcx_mmu.pg));
}

/* Undecoded range, hi exclusive.  Call with hi <= lo to turn it off.  Only one
   of the two ways to catch the hole should be live per machine: this one, or a
   CPU-only catch-all mapping over the same range (MEM_MAPPING_CPU_ONLY), which
   composes with maps added later while this one does not. */
void
pcx_mmu_set_hole(uint32_t lo, uint32_t hi)
{
    pcx_mmu_hole_lo = lo;
    pcx_mmu_hole_hi = hi;
}

uint32_t
pcx_mmu_fault_addr(void)
{
    return pcx_mmu.fault;
}

void
pcx_mmu_init(int type)
{
    pcx_mmu_reset();
    pcx_mmu_set_hole(0, 0);
    pcx_mmu.type = type;

    io_sethandler(PCX_MMU_BASE, PCX_MMU_SIZE,
                  pcx_mmu_inb, pcx_mmu_inw, NULL,
                  pcx_mmu_outb, pcx_mmu_outw, NULL, NULL);

    /* Hooked only while this machine runs; every other machine leaves it
       NULL and pays one test per access. */
    mem_machine_translate = pcx_mmu_translate;
}

void
pcx_mmu_close(void)
{
    mem_machine_translate = NULL;
    io_removehandler(PCX_MMU_BASE, PCX_MMU_SIZE,
                     pcx_mmu_inb, pcx_mmu_inw, NULL,
                     pcx_mmu_outb, pcx_mmu_outw, NULL, NULL);
}
