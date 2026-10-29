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
 *          The shape follows MAME's src/mame/siemens/pcd.cpp (pcd_state::
 *          mmu_r / mmu_w plus the translation inside mem_r / mem_w), which
 *          is the only existing model of this part.
 *
 *          Why this is not built out of memory mappings: the pages are
 *          1 KB, and 86Box rebuilds its mapping tables on a 4 KB grid
 *          (MEM_GRANULARITY_BITS == 12), so four independently mapped
 *          1 KB pages cannot share one granule.  Translation therefore
 *          happens in the access path, through mem_machine_translate()
 *          (see src/mem/mem.c), which also covers instruction fetches --
 *          the 808x BIU reads code with read_mem_b()/read_mem_w().
 */
#ifndef EMU_PCX_MMU_H
#define EMU_PCX_MMU_H

/* Software selectable at machine level, the way MAME exposes it through the
   "mmu" input port: the two SINIX releases use different page-number widths. */
#define PCX_MMU_NONE 0
#define PCX_MMU_SINIX10 1
#define PCX_MMU_SINIX12 2

extern void pcx_mmu_init(int type);
/* Undecoded address range (hi exclusive) that raises NMI on access; hi <= lo
   disables it.  Alternative to a CPU-only catch-all mapping -- use one, not
   both. */
extern void     pcx_mmu_set_hole(uint32_t lo, uint32_t hi);
extern uint32_t pcx_mmu_fault_addr(void);
extern void pcx_mmu_close(void);
extern void pcx_mmu_reset(void);

#endif /*EMU_PCX_MMU_H*/
