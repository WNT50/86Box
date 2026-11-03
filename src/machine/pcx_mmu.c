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
 * (address & 0xf0000) | (entry & 0xff00) | (address & 0x7ff) -- the entry
 * rewrites A15..A8 only, the 64 KB block (A19..A16) passes through.  A zero entry means the page was never
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
#define PCX_MMU_ENTRY_LO 0x0400                    /* 0x8400 first entry port */
#define PCX_MMU_ACK      0x0800                    /* 0x8800 system call */
#define PCX_MMU_IOOFF    0x0a00                    /* 0x8a00 read and all I/O is trapped */
#define PCX_MMU_PAGES    1024                      /* 32 banks x 32 slots */
#define PCX_MMU_PGSIZE   0x800                     /* 2 KB */
#define PCX_MMU_WINDOW   0x100000                  /* the card sits under the whole 1 MB */

typedef struct {
    uint16_t ctl;
    int      defer;                                /* mapping waits for the next cs load */
    uint16_t pg[PCX_MMU_PAGES];
    int      type;
    int      ack;                                  /* system call latched */
    int      fpend;                                /* protection fault latched */
    int      iotrap;                               /* 0x8a00 trap raised the nmi */
    int      ioguard;                              /* 0x8a00 read: I/O is trapped */
    uint32_t fault;                                /* last faulted address */
} pcx_mmu_t;

static pcx_mmu_t pcx_mmu;

/* Bit 4 / bit 5 enforcement.  Off by default: the model's table content still
   loses the loader's own map to its self test, so enforcing now only moves the
   failure.  Set PCX_PROT=1 to enable it (see the notes, section 250). */
static int pcx_mmu_prot = -1;
static int pcx_mmu_encs = -1;
static int pcx_mmu_absf  = -1;

/* Addresses this machine does not decode at all (PC-X: 0x80000-0xFBFFF, the
   region between the RAM behind the MMU and the ROM).  MAME models it as a
   two-instruction fallback in the address map that just raises NMI; the same
   check can live here because this function already runs on every access.
   Disabled when hi <= lo. */
static uint32_t pcx_mmu_hole_lo;
static uint32_t pcx_mmu_hole_hi;

/* RAM wrap mask: the entry can name pages of a 2 MB space, the board holds at
   most 1 MB, so the high half of that range folds back onto the low half. */
static uint32_t pcx_mmu_ram_mask = 0x000fffffu;

/* Entry port -> table slot, or -1 when the port is not an entry.  Every port
   from 0x8400 to 0x84f8 is one: the POST writes 0x5015 to all thirty-two of
   them, 0x84f0 and 0x84f8 included, and reads the value back (v3 BIOS
   0xfe296-0xfe2c0), so the kernel driver's use of the last two is an ordinary
   pair of entries, not an index / data register pair. */

/* The X8 jumper ("MMU Type" on the machine) picks which SINIX release the card
   is strapped for.  What the jumper changes is still open: the POST's own self
   test only ever walks thirty-two banks of thirty-two slots, which is what the
   table is built as here, so both releases take the same layout until something
   shows a difference. */

/* PCX_EN_CS=1: the mapping the control register installs only applies once the
   cpu has made a far transfer into another segment.  The routine at 0x103a
   writes the control register, patches the return frame's segment and does an
   lret; it can only survive its own write if the new mapping starts applying
   after that lret. */
/* 0x8a00: reading it turns every further I/O cycle into a non maskable
   interrupt.  The card is meant to keep user mode out of the hardware; SINIX
   never arms it, and neither the loader nor the kernel ever touch the port, so
   this path only runs if a guest asks for it.  One cycle is refused, the NMI
   goes out and the trap opens again: the POST arms it, touches a port and then
   expects its own handler -- which does I/O itself -- to have run (v3 BIOS
   0xfe35e-0xfe36e). */
void (*pcx_mmu_nmi_status)(uint8_t bits) = NULL;

static int
pcx_mmu_io_guard(UNUSED(uint16_t port), UNUSED(uint8_t write))
{
    if (!pcx_mmu.ioguard)
        return 0;

    if (!pcx_mmu.iotrap) {
        pcx_mmu.iotrap = 1;
        pcx_mmu.fault = PCX_MMU_BASE + PCX_MMU_IOOFF;
        /* The handler the POST installs looks at the status byte before it does
           anything else: with no bit set it treats the interrupt as a debug
           trap and drops into the ROM monitor (v3 BIOS 0xfe361-0xfe373, the
           handler it builds at 0x879).  The trap reports as 0x08 -- the "not a
           debug trap" line -- and leaves the time-out bit clear, which is the
           combination the test then accepts. */
        if (pcx_mmu_nmi_status != NULL)
            pcx_mmu_nmi_status(0x08);
        nmi_raise();
    }

    /* One refused cycle, then the line is open again: the POST arms the trap,
       touches one port and expects its NMI handler -- which does I/O itself --
       to have run (v3 BIOS 0xfe35e-0xfe36e). */
    pcx_mmu.ioguard = 0;
    io_guard        = NULL;

    return 1;
}

static void
pcx_mmu_cs_load(void)
{
    pcx_mmu.defer = 0;
}

static int
pcx_mmu_abs(void)
{
    if (pcx_mmu_absf < 0) {
        const char *env = getenv("PCX_ABS_NMI");

        pcx_mmu_absf = (env && (env[0] == '1')) ? 1 : 0;
    }

    return pcx_mmu_absf;
}

static int
pcx_mmu_def_cs(void)
{
    if (pcx_mmu_encs < 0) {
        const char *env = getenv("PCX_EN_CS");

        pcx_mmu_encs = (env && (env[0] == '0')) ? 0 : 1;
    }

    return pcx_mmu_encs;
}

/* The table is thirty-two banks of thirty-two slots.  The control register's
   low five bits pick the bank: the POST walks it through 0..0x1f, writes
   0x20 + n into slot n of every bank, then insists each bank reads back what
   was written into it (v3 BIOS 0xfe2d3-0xfe31c).  The slot number comes from
   the port or from the address, five bits of it, so one bank covers one 64 KB
   block and the block number itself is not part of the index. */
static int
pcx_mmu_index(uint32_t slot)
{
    return (int) (((pcx_mmu.ctl & 0x1fu) << 5) | (slot & 0x1fu));
}

/* Which table slot a linear address goes through.  The two releases split the
   entry address differently (MAME pcd.cpp mem_r/mem_w):
     SINIX 1.0: eight 256 KB banks from ctl bits 2..4, 128 slots of 2 KB
     SINIX 1.2: four 512 KB banks from ctl bits 3..4, 256 slots of 2 KB */
static int
pcx_mmu_page_index(uint32_t addr)
{
    uint32_t page = (addr >> 11) & 0x3ffu;

    if (pcx_mmu.type == PCX_MMU_SINIX12)
        return (int) (((page & 0xffu) | ((pcx_mmu.ctl & 0x18u) << 5)) & (PCX_MMU_PAGES - 1));
    return (int) (((page & 0x7fu) | ((pcx_mmu.ctl & 0x1cu) << 5)) & (PCX_MMU_PAGES - 1));
}

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

    idx = (off - PCX_MMU_ENTRY_LO) >> 3;      /* 0..31 inside the bank */

    /* The control register picks the bank, which is what keeps the loader's self
       test (it walks the control register through 4,8,...0x1c and rewrites slots
       0 and 2) from destroying the map the loader itself built under 0-2. */
    return pcx_mmu_index((uint32_t) idx);
}


/* Returns the physical address.  Called from the memory access helpers
   (mem_machine_translate), so it must be cheap: one table read, no
   allocation, no logging in the normal path. */
static uint32_t
pcx_mmu_translate(uint32_t addr, UNUSED(int rw))
{
    uint16_t entry;
    uint32_t page, frame;

    /* Undecoded space first: it is a hole whether the MMU is on, off, or not
       fitted at all -- the board decodes nothing there. */
    if ((addr >= pcx_mmu_hole_lo) && (addr < pcx_mmu_hole_hi)) {
        pcx_mmu.fault = addr;
        nmi_raise();
        return 0xffffffffu;
    }

    if (pcx_mmu_prot < 0) {
        const char *env = getenv("PCX_PROT");

        pcx_mmu_prot = (env && (env[0] == '1')) ? 1 : 0;
    }

    if ((pcx_mmu.type == PCX_MMU_NONE) || !(pcx_mmu.ctl & 0x0020) ||
        (addr >= PCX_MMU_WINDOW))
        return addr;

    if (pcx_mmu.defer)
        return addr;

    page  = addr >> 11;
    entry = pcx_mmu.pg[pcx_mmu_page_index(addr)];

    if (entry == 0x0000)
        return addr;

    /* An entry without the read allow bit is a null entry: the card does not
       let the access reach memory, it raises the non maskable interrupt so the
       firmware can report the segment.  PCX_ABS_NMI=1 to model that instead of
       letting the access through. */
    if (pcx_mmu_abs() && !(entry & 0x0020)) {
        pcx_mmu.fault = addr;
        pcx_mmu.fpend = 1;
        nmi_raise();
        return 0xffffffffu;
    }

    /* Protection, as the card does it: bit 5 is the present bit (an absent page
       faults on any access) and bit 4 is the write allow bit (a write to a page
       without it faults).  Neither is an x86 exception -- the card raises IRQ0
       of the first controller and reports the status in bits 7-6 of the entry
       register for the handler at INT 0x20, and the access does not take
       effect. */
    if (pcx_mmu_prot) {
        if (!(entry & 0x0020) || (rw && !(entry & 0x0010))) {
            pcx_mmu.fault = addr;
            pcx_mmu.fpend = 1;
            picint(0x0001);
            return 0xffffffffu;
        }
    }

    {
        static int rl = -1, n;
        if (rl < 0)
            rl = getenv("PCX_RL") ? 1 : 0;
        if (rl && (n < 40) && (pcx_mmu.ctl != 0)) {
            n++;
            pclog("[RL] ctl=%04X addr=%05X idx=%03X entry=%04X -> %05X rw=%d @%04X:%04X\n",
                  pcx_mmu.ctl, addr & 0xfffffu,
                  pcx_mmu_page_index(addr), entry,
                  (((uint32_t) (((entry & 0x001fu) << 5) |
                                ((entry >> 11) & 0x1fu)) << 11) |
                   (addr & (PCX_MMU_PGSIZE - 1))) & pcx_mmu_ram_mask, rw,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
    }

    /* Entry layout, read back out of the loader's own table (PCX_TLOG capture,
       1108 writes): the page number is split -- bits 15..11 hold its low five
       bits, bits 4..0 its high five -- and bit 5 is present.  The loader writes
       identity entries 0x0030, 0x0830, ... 0xd030 for pages 0..0x1a, and the
       next value it writes for page 0x1b in a write-allowed bank is 0xd830|.1:
       crossing 0x1f carries out of bit 15 and lands in bit 0, which is only
       possible if those five bits are the high half of the page number.  Ten
       bits of page at 2 KB each covers the 2 MB the POST walks; the board holds
       1 MB, so the address wraps there (0x1f7800 -> 0xf7800). */
    /* Entry layout, straight out of the loader's own table writes
       ([TLOG], 1106 writes): the page number sits in bits 15..7 and the
       access flags in bits 5..0 --
           0x0030 -> page 0x000,  0x0830 -> page 0x010,  0x1030 -> page 0x020
           0xB020 -> page 0x160 (0xB0000),  0x5022 -> page 0x0A0 (0x50000)
       2 KB per page, so the physical address is (entry >> 7) << 11. */
    frame = (uint32_t) (((entry & 0x001fu) << 5) | ((entry >> 11) & 0x1fu));

    {
        static int km = -1, kmn;
        if (km < 0)
            km = getenv("PCX_KMAP") ? 1 : 0;
        if (km && (kmn < 200) && (addr >= 0x80000) && (addr < 0x90000)) {
            kmn++;
            pclog("[KM] v=%05X idx=%03X entry=%04X -> p=%05X @%04X:%04X\n",
                  addr & 0xfffffu, pcx_mmu_page_index(addr), entry,
                  ((frame << 11) | (addr & (PCX_MMU_PGSIZE - 1))) & pcx_mmu_ram_mask,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
    }

    {
        static int kl = -1, kn;
        if (kl < 0)
            kl = getenv("PCX_KLOG") ? 1 : 0;
        if (kl && ((cpu_state.seg_cs.seg == 0x1000) || (cpu_state.seg_cs.seg == 0x8000)) && (kn < 60)) {
            kn++;
            pclog("[KL] %05X idx=%03X entry=%04X -> %05X rw=%d @%04X:%04X\n",
                  addr & 0xfffffu, pcx_mmu_page_index(addr), entry,
                  ((frame << 11) | (addr & (PCX_MMU_PGSIZE - 1))) & pcx_mmu_ram_mask,
                  rw, cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
    }

    return ((frame << 11) | (addr & (PCX_MMU_PGSIZE - 1))) & pcx_mmu_ram_mask;
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

    if (off == PCX_MMU_IOOFF) {
        /* Reading the trap port arms it: from here on every I/O cycle is kept
           from its handler and the first one raises NMI. */
        pcx_mmu.ioguard = 1;
        io_guard        = pcx_mmu_io_guard;
        return 0;
    }

    if (off == PCX_MMU_ACK) {
        /* Reading the acknowledge port drops the request and clears whichever
           fault latched it. */
        pcx_mmu.ack    = 0;
        pcx_mmu.fpend  = 0;
        pcx_mmu.iotrap = 0;
        picintc(0x0001);
        return 0;
    }

    idx = pcx_mmu_slot(port);
    if (idx < 0)
        return 0x0000;

    /* Slot 0 of the active bank carries the two status bits the handler watches:
       bit 7 says the last trap was a system call rather than a protection
       fault, bit 6 that one is pending at all.  A pending system call sets
       both, which is what the 0xc0 on the first entry port means. */
    if (idx == pcx_mmu_index(0)) {
        if (pcx_mmu.ack)
            return pcx_mmu.pg[idx] | 0xc0;   /* bit 7 system call, bit 6 pending */
        if (pcx_mmu.fpend)
            return pcx_mmu.pg[idx] | 0x40;   /* bit 6: a fault is latched */
        if (pcx_mmu.iotrap)
            return pcx_mmu.pg[idx] | 0x80;   /* the 0x8a00 trap, not a fault */
    }

    return pcx_mmu.pg[idx];
}

static void
pcx_mmu_outw(uint16_t port, uint16_t val, UNUSED(void *priv))
{
    uint16_t off = (uint16_t) (port - PCX_MMU_BASE);
    int      idx;

    {
        static int on = -1, n;

        if (on < 0)
            on = getenv("PCX_PLOG") ? 1 : 0;
        if (on && (n < 2400) && (off < PCX_MMU_ACK)) {
            n++;
            pclog("[PLOG] %04X %04X ctl=%02X @%04X:%04X\n", port, val,
                  pcx_mmu.ctl, cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
    }

    if (off == PCX_MMU_CTL) {
        /* Bit 5 turns translation on.  Nothing has to be rebuilt: the entry is
           read on every access, unlike a mapping-based model. */
        /* Only the low six bits are storage: the POST writes 0x0a and 0x15 and
           agrees with the read back value after masking with 0x3f (v3 BIOS
           0xfe266-0xfe27e). */
        pcx_mmu.ctl = val & 0x003f;
        /* The mapping the register installs applies from the next instruction
           stream on: the routine that flips the bit keeps running out of the
           prefetch queue, which is why it can patch its own return frame and
           lret into the paged view. */
        if (val & 0x0020)
            pcx_mmu.defer = pcx_mmu_def_cs();
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
    if (idx >= 0) {
        /* PCX_TLOG=1 dumps every table write, with the controlling register and
           the instruction that issued it: the loader builds the kernel's map
           through these ports and the encoding has to be read back out of them
           (caps at 6000 lines so a runaway loop cannot flood the log). */
        static int tl = -1, tn;

        if (tl < 0)
            tl = getenv("PCX_TLOG") ? 1 : 0;
        if (tl && (tn < 6000)) {
            tn++;
            pclog("[TLOG] w port=%04X slot=%02X val=%04X ctl=%02X @%04X:%04X\n",
                  port, idx, val, pcx_mmu.ctl,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }

        /* Bits 0..5 hold the flags and 11..15 the page; 6..10 are the status
           lines the handler reads back, not storage (the POST at 0xfe29a
           compares through the mask 0x3f:0xf800). */
        pcx_mmu.pg[idx] = val & 0xf83f;
    }
}

void
pcx_mmu_reset(void)
{
    pcx_mmu.ctl   = 0;
    pcx_mmu.defer = 0;
    pcx_mmu.ack   = 0;
    pcx_mmu.fpend  = 0;
    pcx_mmu.iotrap = 0;
    pcx_mmu.fault  = 0;
    pcx_mmu.ioguard = 0;
    io_guard        = NULL;
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
    pclog("[MMU] type=%d\n", type);

    /* The entry names a page in a 2 MB space; fold anything past the installed
       RAM back onto it (1 MB machine: 0x1f7800 reads as 0xf7800). */
    if (mem_size > 0)
        pcx_mmu_ram_mask = (mem_size << 10) - 1u;

    /* "MMU Type = None" is the card-less PC-D arrangement: the window is not
       decoded at all, so 0x8000-0x8fff answers like the rest of the hole. */
    if (type == PCX_MMU_NONE)
        return;

    io_sethandler(PCX_MMU_BASE, PCX_MMU_SIZE,
                  pcx_mmu_inb, pcx_mmu_inw, NULL,
                  pcx_mmu_outb, pcx_mmu_outw, NULL, NULL);

    /* Hooked only while this machine runs; every other machine leaves it
       NULL and pays one test per access. */
    mem_machine_translate = pcx_mmu_translate;
    mem_machine_cs_load   = pcx_mmu_cs_load;
}

void
pcx_mmu_close(void)
{
    mem_machine_translate = NULL;
    mem_machine_cs_load   = NULL;
    io_guard              = NULL;

    if (pcx_mmu.type == PCX_MMU_NONE)
        return;

    io_removehandler(PCX_MMU_BASE, PCX_MMU_SIZE,
                     pcx_mmu_inb, pcx_mmu_inw, NULL,
                     pcx_mmu_outb, pcx_mmu_outw, NULL, NULL);
}
