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
#include <86box/plat.h>
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
#define PCX_MMU_PGSIZE   0x400                     /* 1 KB */
#define PCX_MMU_WINDOW   0x080000                  /* MAME translates 0x00000-0x7ffff only */

typedef struct {
    uint16_t ctl;
    int      defer;                                /* mapping waits for the next cs load */
    uint16_t pg[PCX_MMU_PAGES];
    int      type;
    int      ack;                                  /* system call latched */
    int      fpend;                                /* protection fault latched */
    uint32_t fault;                                /* last faulted address */
} pcx_mmu_t;

static pcx_mmu_t pcx_mmu;

/* Bit 4 / bit 5 enforcement.  Off by default: the model's table content still
   loses the loader's own map to its self test, so enforcing now only moves the
   failure.  Set PCX_PROT=1 to enable it (see the notes, section 250). */
static int pcx_mmu_encs = -1;

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


static void
pcx_mmu_cs_load(void)
{
    pcx_mmu.defer = 0;
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
    static int slotoff = 0x7fffffff;
    uint32_t   page;

    /* PCX_MMUSLOT=<n>: 查表时把页号减 n。实测硬件是 n=5：
       虚拟页 0x42 必须读到槽 0x3D（页 0x38 = 0xE32E 的端口桩），
       而槽 0x40 里放的页 0x3B 是 +0x1400 错位的那一条。 */
    if (slotoff == 0x7fffffff)
        slotoff = getenv("PCX_MMUSLOT") ? atoi(getenv("PCX_MMUSLOT")) : 0;

    /* 页大小 2 KB：内核在 1B58:4025 用 >>11 算条目数（0xDB7F>>11 = 27，
       27 x 2 KB 正好覆盖 0x0000-0xD7FF），模块在 0x0E1C 也拿 0x800 做自检。
       PCX_PG1K=1 可切回旧的 1 KB 行为做对照。 */
    static int pg2k = -1;
    if (pg2k < 0)
        pg2k = getenv("PCX_PG1K") ? 0 : 1;

    page = (((addr >> (pg2k ? 11 : 10)) - (uint32_t) slotoff)) & 0x3ffu;

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
pcx_mmu_translate(uint32_t addr, int rw)
{
    /* Whoever writes the module's far-call slot decides where the kernel's
       dispatch lands; log the writer for the slot bytes only. */
    if (rw && (addr >= 0x1060u) && (addr < 0x1064u)) {
        static int n;
        if (n < 200) {
            n++;
            int      ix = pcx_mmu_page_index(addr);
            uint16_t en = pcx_mmu.pg[ix];
            uint16_t uv = (uint16_t) ((en << 4) | (en >> 12));
            pclog("[SLOT] w %05X idx=%03X entry=%04X -> phys=%05X @%04X:%04X ctl=%04X ds=%04X es=%04X ax=%04X bx=%04X cx=%04X si=%04X di=%04X\n",
                  addr & 0xfffffu, ix, en,
                  (((uint32_t) (uv & 0xff00u)) | (addr & 0x7ffu)) & pcx_mmu_ram_mask,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.oldpc,
                  pcx_mmu.ctl, cpu_state.seg_ds, cpu_state.seg_es.seg,
                  cpu_state.regs[0].w, cpu_state.regs[3].w, cpu_state.regs[1].w,
                  cpu_state.regs[6].w, cpu_state.regs[7].w);
        }
    }
    /* PCX_MODW=1: the far-call operand at 1DD8:1BF5 is patched at run time;
       find the writer. */
    if (rw && (addr >= 0x1f960u) && (addr < 0x1f990u) && (getenv("PCX_MODW") != NULL) &&
        (cpu_state.seg_cs.seg != 0xfc00)) {
        static int n;
        if (n < 200) {
            n++;
            pclog("[MODW] w %05X @%04X:%04X ds=%04X es=%04X ax=%04X bx=%04X cx=%04X dx=%04X si=%04X di=%04X\n",
                  addr & 0xfffffu, cpu_state.seg_cs.seg, (uint16_t) cpu_state.oldpc,
                  cpu_state.seg_ds.seg, cpu_state.seg_es.seg,
                  cpu_state.regs[0].w, cpu_state.regs[3].w, cpu_state.regs[1].w,
                  cpu_state.regs[2].w, cpu_state.regs[6].w, cpu_state.regs[7].w);
        }
    }

    uint16_t entry;
    uint32_t phys;

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

    /* MAME applies a new mapping immediately (mem_r reads the table on every
       access); the deferred mode is kept behind PCX_DEFER=1 because the
       kernel's "patch the return frame and lret into the paged view" trick
       relies on the old behaviour. */
    {
        static int df = -1;
        if (df < 0)
            df = getenv("PCX_DEFER") ? 1 : 0;
        if (df && pcx_mmu.defer)
            return addr;
    }

    /* PCX_MAP42=<phys>: 把逻辑页 0x42（0x10800-0x10BFF）改映射到指定物理页。
       实测模块在这一页上只执行 0B2E/0B31/0B34/0B35/0B37/0B39/0B3D/0B40
       八个地址，即三处端口桩的调用点；而桩表本体在物理 0xE000 页内。
       用它验证“不改代码、改 MMU”这条路线。 */
    {
        static long m42 = -2;
        if (m42 == -2) {
            const char *e = getenv("PCX_MAP42");
            m42 = e ? strtol(e, NULL, 16) : -1;
        }
        if ((m42 >= 0) && ((addr >> 10) == 0x42u))
            return (((uint32_t) m42) & 0x7FC00u) | (addr & (PCX_MMU_PGSIZE - 1));
    }

    entry = pcx_mmu.pg[pcx_mmu_page_index(addr)];

    {
        static int t1 = -1, done;
        if (t1 < 0)
            t1 = getenv("PCX_CTL") ? 1 : 0;
        if (t1 && !done && (addr >= 0x10000) && (addr < 0x10100)) {
            done = 1;
            pclog("[T1] addr=%05X idx=%03X entry=%04X defer=%04X ctl=%02X "
                  "at %04X:%04X\n", addr, pcx_mmu_page_index(addr), entry,
                  pcx_mmu.defer, pcx_mmu.ctl,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
    }

    /* Null entry, exactly as the card does it (MAME pcd.cpp mem_r/mem_w): a
       read needs bit 1, a write needs bit 0, and without it the access never
       reaches memory -- the card pulses NMI and the firmware reports the
       segment.  PCX_NULLPASS=1 restores the old "let it through" behaviour so
       the two can be compared. */
    {
        static int np = -1;

        if (np < 0)
            np = getenv("PCX_NULLPASS") ? 1 : 0;

        if (!np && !(entry & (rw ? 0x0001u : 0x0002u))) {
            static int nn;

            if (nn < 400) {
                nn++;
                pclog("[NUL] t=%u addr=%05X idx=%03X entry=%04X rw=%d ctl=%02X ds=%04X @%04X:%04X\n",
                      plat_get_ticks(), addr & 0xfffffu, pcx_mmu_page_index(addr), entry, rw,
                      pcx_mmu.ctl, (uint16_t) cpu_state.seg_ds.seg,
                      (uint16_t) cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
            }
            if (nn <= 2) {
                int k;
                pclog("[NUL] table:");
                for (k = -3; k <= 3; k++) {
                    int q = pcx_mmu_page_index(addr) + k;
                    if ((q >= 0) && (q < PCX_MMU_PAGES))
                        pclog(" [%03X]=%04X", q, pcx_mmu.pg[q]);
                }
                pclog("\n");
            }
            pcx_mmu.fault = addr;
            pcx_mmu.fpend = 1;
            nmi_raise();
            return 0xffffffffu;
        }
    }

    /* MAME moves the 9 KB kernel block from 0x0DBE0 to 0x103E0 between 20 s and
       40 s; log the first write per 256-byte block of that destination. */
    if (rw && (addr >= 0x103E0u) && (addr < 0x12800u)) {
        static int mv;
        static uint8_t seen[0x40];
        uint32_t blk = (addr - 0x103E0u) >> 8;

        if (blk < sizeof(seen) && !seen[blk]) {
            seen[blk] = 1;
            if (mv < 48) {
                mv++;
                pclog("[MV] w %05X @%04X:%04X ctl=%04X\n", addr & 0xfffffu,
                      cpu_state.seg_cs.seg, (uint16_t) cpu_state.oldpc, pcx_mmu.ctl);
            }
        }
    }

    /* Kernel code page (phys 0x0D100-0x0D300) being written means something
       is trampling the code the kernel is running. */
    if (rw && (addr >= 0x1F900u) && (addr < 0x1FA80u)) {
        static int kn;
        if (kn < 24) {
            kn++;
            pclog("[KWR] lin=%05X phys=%05X entry=%04X @%04X:%04X ax=%04X di=%04X cx=%04X\n",
                  addr & 0xfffffu,
                  (((uint32_t) ((uint16_t) ((pcx_mmu.pg[pcx_mmu_page_index(addr)] << 4) |
                                            (pcx_mmu.pg[pcx_mmu_page_index(addr)] >> 12)) & 0xff00u)) |
                   (addr & 0x7ffu)) & pcx_mmu_ram_mask,
                  pcx_mmu.pg[pcx_mmu_page_index(addr)],
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.oldpc,
                  cpu_state.regs[0].w, cpu_state.regs[7].w, cpu_state.regs[1].w);
        }
    }

    /* Is the per-task copy of the module (0x39300 for task 1) ever written? */
    if (rw && (addr >= 0x39000u) && (addr < 0x39500u)) {
        static int cpn;
        if (cpn < 30) {
            cpn++;
            pclog("[CPY] w %05X entry=%04X @%04X:%04X\n", addr & 0xfffffu,
                  pcx_mmu.pg[pcx_mmu_page_index(addr)],
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.oldpc);
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
    {
        static int pg2k3 = -1;
        uint32_t poff;

        if (pg2k3 < 0)
            pg2k3 = getenv("PCX_PG1K") ? 0 : 1;

        if (pg2k3) {
            /* PCX_MAP20=<phys>: 2 KB 页下把逻辑槽 0x20（0x10000-0x107FF）
               改指到指定物理基址。装载器把这一槽写成 0xD800（库基址），
               而模块入口 1000:002C 需要它指向 0xEC00（模块代码）。 */
            static long m20 = -2;
            if (m20 == -2) {
                const char *e = getenv("PCX_MAP20");
                m20 = e ? strtol(e, NULL, 16) : -1;
            }
            if ((m20 >= 0) && ((addr >> 11) == 0x20u))
                return (((uint32_t) m20) & 0x7FC00u) | (addr & 0x7ffu);

            /* 2 KB 页：条目按“表里存的形式”解，物理基址 = 回转后的 bits 15..8。
               依据：装载器写的恒等条目是 0x0030,0x0830,0x1030,…（步长 0x800），
               即 bits 15..8 = 物理地址 bits 15..8，低 6 位是标志（0x30 = 存在+可写）。*/
            uint16_t val  = (uint16_t) ((pcx_mmu.pg[pcx_mmu_page_index(addr)] << 4) |
                                        (pcx_mmu.pg[pcx_mmu_page_index(addr)] >> 12));

            /* PCX_MAMEPHYS=1: MAME's formula verbatim, in its own word
               addressing: phys = ((reg << 3) & 0x7fc00) | (offset & 0x3ff),
               then back to a byte address.  Kept as a switch because the
               identity entries carry flags in their low bits, which this
               formula does not mask away. */
            static int mp = -1;
            if (mp < 0)
                mp = getenv("PCX_MAMEPHYS") ? 1 : 0;
            if (mp) {
                uint32_t w = ((((uint32_t) pcx_mmu.pg[pcx_mmu_page_index(addr)] << 3) & 0x7fc00u) |
                              ((addr >> 1) & 0x3ffu));
                phys = (w << 1) | (addr & 1u);
            } else {
                /* PCX_PGDEC=1: decode the entry as "physical page base in bits
                   15..10, flags in the low bits" instead of the rotated form.
                   Identity entries read back as 0x0070, 0x0830, 0x1030, ... so
                   the page base sits directly in the entry. */
                /* Default: 32-bit rotate with the full 19-bit physical mask.
                   The old 8-bit form (PCX_PGDEC=0) could only express pages
                   below 64 KB, which folded every module/task page down into
                   low memory and made the CPU execute zeros. */
                static int pdec = -1;
                if (pdec < 0)
                    pdec = getenv("PCX_PGDEC") ? atoi(getenv("PCX_PGDEC")) : 5;
                if (pdec == 1) {
                    /* PCX_PGDEC=1: 16-bit rotate then mask 0x7FC00. */
                    uint32_t v32 = ((uint32_t) ((uint16_t) ((pcx_mmu.pg[pcx_mmu_page_index(addr)] << 4) |
                                                           (pcx_mmu.pg[pcx_mmu_page_index(addr)] >> 12)))) & 0x7fc00u;
                    phys = v32 | (addr & 0x7ffu);
                } else if (pdec == 2) {
                    /* PCX_PGDEC=2: 32-bit rotate, keep bits 18..10 (1 MB space). */
                    uint16_t en = pcx_mmu.pg[pcx_mmu_page_index(addr)];
                    uint32_t v32 = (((uint32_t) en << 4) | ((uint32_t) en >> 12)) & 0x7fc00u;
                    phys = v32 | (addr & 0x7ffu);
                } else if (pdec == 5) {
                    /* MAME's lookup formula in byte units, applied to the entry
                       exactly as the guest wrote it:
                       phys = ((reg << 4) & 0xff800) | (byte & 0x7ff).
                       Calibration: the loader's identity entry 0x0830 for page 1
                       decodes to 0x800, which is the whole point. */
                    phys = (((uint32_t) pcx_mmu.pg[pcx_mmu_page_index(addr)] << 4) & 0xff800u) |
                           (addr & 0x7ffu);
                } else if (pdec == 4) {
                    /* MAME verbatim (pcd.cpp:279 store, :431 lookup):
                       stored = (data >> 4) | (data << 12)
                       word offset = ((stored << 3) & 0x7fc00) | (offset & 0x3ff)
                       -> byte address = (stored << 4) & 0xff800, plus the low bits. */
                    uint16_t en = pcx_mmu.pg[pcx_mmu_page_index(addr)];
                    uint16_t st = (uint16_t) ((en >> 4) | (en << 12));
                    phys = (((uint32_t) st << 4) & 0xff800u) | (addr & 0x7ffu);
                } else if (pdec == 3)
                    phys = ((uint32_t) (pcx_mmu.pg[pcx_mmu_page_index(addr)] & 0xfc00u)) |
                           (addr & 0x7ffu);
                else
                    phys = ((uint32_t) (val & 0xff00u)) | (addr & 0x7ffu);
            }
        } else {
            phys = ((uint32_t) (entry << 3) & 0x7fc00u) | (addr & (PCX_MMU_PGSIZE - 1));
        }
    }

    {
        static int km = -1, kmn;
        if (km < 0)
            km = getenv("PCX_KMAP") ? 1 : 0;
        if (km && (kmn < 200) && (addr >= 0x80000) && (addr < 0x90000)) {
            kmn++;
            pclog("[KM] v=%05X idx=%03X entry=%04X -> p=%05X @%04X:%04X\n",
                  addr & 0xfffffu, pcx_mmu_page_index(addr), entry,
                  phys & pcx_mmu_ram_mask,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
    }

    {
        static int kl = -1, kn;
        if (kl < 0)
            kl = getenv("PCX_KLOG") ? 1 : 0;
        if (kl && (addr >= 0x10000) && (addr < 0x11000) && (kn < 60)) {
            kn++;
            pclog("[KL] %05X idx=%03X entry=%04X -> %05X rw=%d @%04X:%04X\n",
                  addr & 0xfffffu, pcx_mmu_page_index(addr), entry,
                  phys & pcx_mmu_ram_mask,
                  rw, cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
    }

    /* PCX_MAPT=1: ground truth from the model itself for the addresses the
       module's device-table scan and dispatch run through. */
    if (getenv("PCX_MAPT") && ((addr == 0x02dbcu) || (addr == 0x01060u) || (addr == 0x1f97eu))) {
        static int n;
        if (n < 60) {
            n++;
            int ix = pcx_mmu_page_index(addr);
            pclog("[MAPT] %s %05X entry=%04X phys=%05X ctl=%04X cs=%04X:%04X ds=%04X\n",
                  rw ? "w" : "r", addr, pcx_mmu.pg[ix], phys & pcx_mmu_ram_mask,
                  pcx_mmu.ctl, cpu_state.seg_cs.seg, (uint16_t) cpu_state.oldpc,
                  cpu_state.seg_ds.seg);
        }
    }

    return phys & pcx_mmu_ram_mask;
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

    if (off == PCX_MMU_ACK) {
        /* Reading the acknowledge port drops the request and clears whichever
           fault latched it. */
        pcx_mmu.ack    = 0;
        pcx_mmu.fpend  = 0;
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
    /* The card hands the entry back rotated four bits the other way; the
       table stores it that way because the page number has to line up with
       the address lines the card drives (MAME pcd.cpp mmu_r/mmu_w). */
    {
        uint16_t data = (uint16_t) ((pcx_mmu.pg[idx] << 4) | (pcx_mmu.pg[idx] >> 12));

        if (idx == pcx_mmu_index(0)) {
            if (pcx_mmu.ack)
                return data | 0xc0;   /* bit 7 system call, bit 6 pending */
            if (pcx_mmu.fpend)
                return data | 0x40;   /* bit 6: a fault is latched */
        }

        {
            static int e84 = -1, n;
            if (e84 < 0)
                e84 = getenv("PCX_E84") ? 1 : 0;
            if (e84 && (n < 40) && (off == 0x04f8u)) {
                n++;
                pclog("[E84R] port=%04X idx=%03X pg=%04X -> %04X ctl=%02X @%04X:%04X\n",
                      port, idx, pcx_mmu.pg[idx], data, pcx_mmu.ctl,
                      cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
            }
        }

        return data;
    }
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
        {
            static int cl = -1;
            if (cl < 0)
                cl = getenv("PCX_CTL") ? 1 : 0;
            if (cl)
                pclog("[CTL] %02X -> %02X defer=%04X at %04X:%04X\n",
                      pcx_mmu.ctl, val & 0x003f, pcx_mmu.defer,
                      cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }
        pcx_mmu.ctl = val;
        /* The mapping the register installs applies from the next instruction
           stream on: the routine that flips the bit keeps running out of the
           prefetch queue, which is why it can patch its own return frame and
           lret into the paged view. */
        if (val & 0x0020) {
            pcx_mmu.defer = pcx_mmu_def_cs();
        }
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

        /* PCX_PTE3F=1: the page that holds linear 0x1F800 (slot 0x3F) is the
           window the loader writes the module through; it stops pointing at
           the module after 36 s.  Log every write that touches it. */
        {
            static int p3 = -1, n3;
            if (p3 < 0)
                p3 = getenv("PCX_PTE3F") ? 1 : 0;
            if (p3 && ((idx & 0x7f) == 0x3f) && (n3 < 300)) {
                n3++;
                pclog("[PTE3F] idx=%03X old=%04X val=%04X ctl=%04X @%04X:%04X\n",
                      idx, pcx_mmu.pg[idx], val, pcx_mmu.ctl,
                      cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
            }
        }

        /* Bits 0..5 hold the flags and 11..15 the page; 6..10 are the status
           lines the handler reads back, not storage (the POST at 0xfe29a
           compares through the mask 0x3f:0xf800). */
        pcx_mmu.pg[idx] = (uint16_t) ((val >> 4) | (val << 12));

        {
            static int e84w = -1, n;
            if (e84w < 0)
                e84w = getenv("PCX_E84") ? 1 : 0;
            if (e84w && (n < 40) && (off == 0x04f8u)) {
                n++;
                pclog("[E84W] port=%04X idx=%03X val=%04X store=%04X ctl=%02X @%04X:%04X\n",
                      port, idx, val, pcx_mmu.pg[idx], pcx_mmu.ctl,
                      cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
            }
        }
    }
}


/* Debug helper: read a byte through the table without the null-entry NMI, so a
   probe can look at memory the guest is about to touch. */
uint8_t
pcx_mmu_peek8(uint32_t addr)
{
    uint16_t e;
    uint32_t ph;

    if ((pcx_mmu.type == PCX_MMU_NONE) || !(pcx_mmu.ctl & 0x0020) ||
        (addr >= PCX_MMU_WINDOW))
        return mem_readb_phys(addr);

    e = pcx_mmu.pg[pcx_mmu_page_index(addr)];
    if (e == 0x0000)
        return 0xff;

    ph = ((uint32_t) ((uint16_t) ((e << 4) | (e >> 12)) & 0xff00u)) | (addr & 0x7ffu);
    return mem_readb_phys(ph & pcx_mmu_ram_mask);
}

uint16_t
pcx_mmu_pg_get(int idx)
{
    if ((idx < 0) || (idx >= PCX_MMU_PAGES))
        return 0xFFFF;
    return pcx_mmu.pg[idx];
}

void
pcx_mmu_reset(void)
{
    pcx_mmu.ctl   = 0;
    pcx_mmu.defer = 0;
    pcx_mmu.ack   = 0;
    pcx_mmu.fpend  = 0;
    pcx_mmu.fault  = 0;
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

    if (pcx_mmu.type == PCX_MMU_NONE)
        return;

    io_removehandler(PCX_MMU_BASE, PCX_MMU_SIZE,
                     pcx_mmu_inb, pcx_mmu_inw, NULL,
                     pcx_mmu_outb, pcx_mmu_outw, NULL, NULL);
}
