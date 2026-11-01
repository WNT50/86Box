/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This program is free software; you can redistribute it and/or
 *          modify it under the terms of the GNU General Public License as
 *          published by the Free Software Foundation; either version 2 of
 *          the License, or (at your option) any later version.
 *
 *          This program is distributed in the hope that it will be useful,
 *          but WITHOUT ANY WARRANTY; without even the implied warranty of
 *          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *          GNU General Public License for more details.
 *
 *          You should have received a copy of the GNU General Public
 *          License along with this program; if not, write to the Free
 *          Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 *          Boston, MA 02110-1301, USA.
 *
 *          The Siemens PC-X / PC-D MMU card (S26361-D240).
 *
 * The card is 3 x 4096-bit static RAM, one 12-bit entry per 2 KB page, sitting
 * on ports 0x8400-0x87ff.  Everything in the model below comes from the kernel
 * driver in SINIX 1.0 and from the loader in the BIOS, not from MAME (whose
 * pcd.cpp is marked MACHINE_NOT_WORKING and decodes 512 KB only):
 *
 *   the driver in the kernel (physical 0xdf40..0xe16b) does
 *
 *     mov  bx,[0xf82e] / mov al,[bx+0x16] / shl al,1 / shl al,1   ; task * 4
 *     mov  cx,[bx+4] / and cx,0x7ff      ; 2 KB inside the page
 *     xor  cx,bx / xchg bh,bl / or bh,al ; page number, task in the high half
 *     and  bx,0xf8                       ; 8 bytes per entry
 *     add  bx,0x8400                     ; entry port = 0x8400 + page * 8
 *     mov  dx,0x84f0 / xchg ax,bx / out ax,dx   ; index latch
 *     add  dx,8        / xchg ax,cx / out ax,dx ; data (0x84f8)
 *
 *   and the loader in the BIOS fills the table with
 *
 *     ctl=0    ports 0x8400,0x8408-0x8420 <- 0xd820,0xe020,0xe820,0xf020,0xf820
 *              i.e. logical 0x0000/0x0800/0x1000/0x1800/0x2000 -> physical
 *              0xd800/0xe000/0xe800/0xf000/0xf800, one 2 KB frame per entry,
 *              the frame number in bits 15..8 and 0x20 = present.
 *
 * so: page = address >> 11, entry = table[page & 0x7f], physical address =
 * (entry & 0xff00) | (address & 0x7ff).  A zero entry means the page was never
 * programmed and passes through untranslated -- the loader and the kernel both
 * execute out of the RAM below the card's window without ever writing an entry
 * for it.
 *
 * 0x8000  control: bit 5 enables translation, the low bits are the task /
 *         segment selector the board compares the entries against.
 * 0x8800  system call: a read drops the request and clears the latch.
 *
 * Protection faults (bit 4 = write allowed, bit 5 = read allowed / present,
 * bit 6 = fault latched, bit 7 = system call) are decoded but not enforced
 * yet: the kernel takes the fault as IRQ0 of the first controller and its
 * handler is not reachable until the kernel boots, so enforcing it only
 * produced an abort loop over the IVT writes.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <86box/86box.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/pic.h>
#include <86box/nmi.h>
#include <86box/plat_unused.h>
#include "cpu.h"

#include <86box/pcx_mmu.h>

#define PCX_MMU_BASE     0x8000
#define PCX_MMU_SIZE     0x1000
#define PCX_MMU_CTL      0x0000                    /* 0x8000 */
#define PCX_MMU_IDX      0x00f0                    /* 0x84f0 index latch */
#define PCX_MMU_DAT      0x00f8                    /* 0x84f8 data */
#define PCX_MMU_ENTRY_LO 0x0400                    /* 0x8400 first entry port */
#define PCX_MMU_ACK      0x0800                    /* 0x8800 system call */
#define PCX_MMU_PAGES    128                       /* 0x8400-0x87f8 / 8 */
#define PCX_MMU_PGSIZE   0x800                     /* 2 KB */
#define PCX_MMU_WINDOW   0x80000                   /* what the card decodes */

typedef struct {
    uint16_t ctl;
    uint16_t idx;                                  /* index latch */
    uint16_t pg[PCX_MMU_PAGES];
    int      type;
    int      ack;                                  /* system call latched */
    int      fpend;                                /* protection fault latched */
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

/* Entry port -> table slot, or -1 when the port is not an entry.  0x84f0 and
   0x84f8 are the index / data pair the kernel driver uses, so they are not
   entry ports even though they sit inside the window. */
static int
pcx_mmu_slot(uint16_t port)
{
    uint16_t off = (uint16_t) (port - PCX_MMU_BASE);
    int      idx;

    if (off < PCX_MMU_ENTRY_LO)
        return -1;
    if (off >= PCX_MMU_ACK)
        return -1;
    if (off & 7)
        return -1;

    idx = (off - PCX_MMU_ENTRY_LO) >> 3;
    if ((idx == (PCX_MMU_IDX >> 3)) || (idx == (PCX_MMU_DAT >> 3)))
        return -1;

    return idx;
}

/* Slot the index latch points at, or -1. */
static int
pcx_mmu_latched(void)
{
    int idx = pcx_mmu_slot(pcx_mmu.idx);

    return (idx < 0) ? -1 : idx;
}

/* Returns the physical address.  Called from the memory access helpers
   (mem_machine_translate), so it must be cheap: one table read, no
   allocation, no logging in the normal path. */
static uint32_t
pcx_mmu_translate(uint32_t addr, UNUSED(int rw))
{
    uint16_t entry;
    uint32_t page;

    /* Undecoded space first: it is a hole whether the MMU is on, off, or not
       fitted at all -- the board decodes nothing there. */
    if ((addr >= pcx_mmu_hole_lo) && (addr < pcx_mmu_hole_hi)) {
        pcx_mmu.fault = addr;
        nmi_raise();
        return 0xffffffffu;
    }

    if ((pcx_mmu.type == PCX_MMU_NONE) || !(pcx_mmu.ctl & 0x0020) ||
        (addr >= PCX_MMU_WINDOW))
        return addr;

    page  = addr >> 11;
    entry = pcx_mmu.pg[page & (PCX_MMU_PAGES - 1)];

    if (entry == 0x0000)
        return addr;

    return ((uint32_t) (entry & 0xff00u)) | (addr & (PCX_MMU_PGSIZE - 1));
}

static uint16_t pcx_mmu_inw(uint16_t port, void *priv);
static void     pcx_mmu_outw(uint16_t port, uint16_t val, void *priv);

/* The registers are 16 bits wide on a 16-bit bus, and the loader does read some
   of them a byte at a time -- a byte access has to answer with the matching
   half, not 0xff, or a poll loop never sees the bit it is waiting for. */
static uint8_t
pcx_mmu_inb(uint16_t port, void *priv)
{
    uint16_t word = pcx_mmu_inw((uint16_t) (port & ~1), priv);

    return (uint8_t) ((port & 1) ? (word >> 8) : (word & 0xff));
}

static void
pcx_mmu_outb(uint16_t port, uint8_t val, void *priv)
{
    uint16_t word = pcx_mmu_inw((uint16_t) (port & ~1), priv);

    if (port & 1)
        word = (uint16_t) ((word & 0x00ff) | ((uint16_t) val << 8));
    else
        word = (uint16_t) ((word & 0xff00) | (uint16_t) val);

    pcx_mmu_outw((uint16_t) (port & ~1), word, priv);
}

static uint16_t
pcx_mmu_inw(uint16_t port, UNUSED(void *priv))
{
    uint16_t off = (uint16_t) (port - PCX_MMU_BASE);
    int      idx;

    if (off == PCX_MMU_CTL)
        return pcx_mmu.ctl;

    if (off == PCX_MMU_IDX)
        return pcx_mmu.idx;

    if (off == PCX_MMU_DAT) {
        idx = pcx_mmu_latched();

        return (idx < 0) ? 0x0000 : pcx_mmu.pg[idx];
    }

    if (off == PCX_MMU_ACK) {
        /* Reading the acknowledge port drops the request and clears whichever
           fault latched it. */
        pcx_mmu.ack   = 0;
        pcx_mmu.fpend = 0;
        picintc(0x0001);
        return 0;
    }

    idx = pcx_mmu_slot(port);

    return (idx < 0) ? 0x0000 : pcx_mmu.pg[idx];
}

static void
pcx_mmu_outw(uint16_t port, uint16_t val, UNUSED(void *priv))
{
    uint16_t off = (uint16_t) (port - PCX_MMU_BASE);
    int      idx;

    if (off == PCX_MMU_CTL) {
        /* Bit 5 turns translation on.  Nothing has to be rebuilt: the entry is
           read on every access, unlike a mapping-based model. */
        pcx_mmu.ctl = val;
        return;
    }

    if (off == PCX_MMU_IDX) {
        /* The driver writes the entry port here, not a table slot. */
        pcx_mmu.idx = val;
        return;
    }

    if (off == PCX_MMU_DAT) {
        idx = pcx_mmu_latched();
        if (idx >= 0)
            pcx_mmu.pg[idx] = val;
        return;
    }

    if (off == PCX_MMU_ACK) {
        /* "The table changed": the driver is told through IRQ0 of the first
           controller rather than by polling. */
        pcx_mmu.ack = 1;
        picint(0x0001);
        return;
    }

    idx = pcx_mmu_slot(port);
    if (idx >= 0)
        pcx_mmu.pg[idx] = val;
}

void
pcx_mmu_reset(void)
{
    pcx_mmu.ctl   = 0;
    pcx_mmu.idx   = 0;
    pcx_mmu.ack   = 0;
    pcx_mmu.fpend = 0;
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
