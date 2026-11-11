/*
 * 86Box    A hypervisor and IBM PC system emulator.
 *
 * MCS-48 (8048/8049/8042) interpreter, written for the Mindset's system
 * controller.
 *
 * Instruction encodings follow Intel's "MCS-48 Family of Single Chip
 * Microcomputers User's Manual" (1980).  The decoding was cross-checked
 * against MAME's mcs48 core (src/devices/cpu/mcs48/mcs48.cpp, copyright
 * 1998-2025 MAMEdev, BSD-3-Clause); this is an independent minimal
 * implementation, not a port of that code.
 */
#ifndef MCS48_H
#define MCS48_H

#include <stdint.h>

typedef struct mcs48 mcs48;

typedef struct {
    uint8_t (*p1_in)(mcs48 *);
    uint8_t (*p2_in)(mcs48 *);
    void    (*p1_out)(mcs48 *, uint8_t);
    void    (*p2_out)(mcs48 *, uint8_t);
    uint8_t (*bus_in)(mcs48 *);
    void    (*bus_out)(mcs48 *, uint8_t);
    int     (*t0_in)(mcs48 *);
    int     (*t1_in)(mcs48 *);
} mcs48_hooks;

extern void mcs48_reset(mcs48 *c);
extern int  mcs48_step(mcs48 *c);
extern void mcs48_run(mcs48 *c, int cycles);

#endif /* MCS48_H */
