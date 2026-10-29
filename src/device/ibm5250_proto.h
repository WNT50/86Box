/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          IBM 3477-S 5250 adapter: pure protocol tables.
 *
 *          These helpers carry no state and touch no emulator globals, so
 *          they can be unit-tested on the host (tests/device/
 *          ibm5250_proto_test.cpp) without booting a machine.
 *
 *          Everything here follows the two sources that were verified
 *          against the ROM/EMDIAG bit-banger and the manuals:
 *            - SA21-9247-6 (5250 Functions Reference) §frame format
 *            - National 1992 handbook §3.1.1 (start sequence)
 *            - EMDIAG disassembly at 0x840 / 0x720 (em2.bin)
 */
#ifndef EMU_IBM5250_PROTO_H
#define EMU_IBM5250_PROTO_H

#include <stdint.h>

/*  Block 0xAB0's clock-read pattern (one sample per read at 0x8B1).
 *
 *  c0..c2   0        the first data read already found the sync, and
 *                    0x865 rejects the line if the clock sample is high
 *  c3..c10  1,0,...  0x86C's eight alternation samples
 *  c11..c13 1,1,1    0x882: code violation, positive half 1.5 bit times
 *  c14..    0        0x88F: violation's negative half, then idle
 */
static inline uint8_t
ibm5250_proto_clk_bit(int c)
{
    if ((c >= 3) && (c <= 10))
        return (uint8_t) (((c - 3) & 1) ? 0 : 1);
    if ((c >= 11) && (c <= 13))
        return 1;
    return 0;
}

/*  Block 0xAB0's 16-bit frame, bit k = the k-th bit after the sync (B14 => k 0).
 *
 *  0x740 shifts each received bit into bit15 of BL:BH and shifts the pair
 *  right, so bit k lands in W bit (k+1); 0x761 rebuilds AL from W bits 1..8.
 *  AL is therefore the first eight bits (k 0..7), LSB first — the data byte.
 *
 *  k 0..7    data bit k            (B14..B7)
 *  k 8..10   station bit (k-8)     (B6..B4)
 *  k 11      even parity over B15..B4
 *  k 12..14  0                     (B2..B0 fill)
 */
static inline uint8_t
ibm5250_proto_frame_bit(int k, int data, int station)
{
    int ones;

    if ((k >= 0) && (k <= 7))
        return (uint8_t) ((data >> k) & 1);
    if ((k >= 8) && (k <= 10))
        return (uint8_t) ((station >> (k - 8)) & 1);
    if (k == 11) {
        ones = 1;                       /* the sync bit is always 1 */
        for (int t = 0; t < 8; t++)
            ones += (data >> t) & 1;
        for (int t = 0; t < 3; t++)
            ones += (station >> t) & 1;
        return (uint8_t) (ones & 1);    /* 1 => make the 12-bit group even */
    }
    return 0;
}

/*  0x1C2 state selector: the code written selects which status bits the
 *  following 0x1C6 read must show.  SEL_PHASE means "follow dev->phase".
 */
#define IBM5250_SEL_PHASE (-1)
#define IBM5250_SEL_KEEP  (-2)

typedef struct ibm5250_sel_s {
    uint8_t code;
    int     b1;      /* -1 = phase, else forced value */
    int     b0;      /* -2 = keep previous, else forced value */
} ibm5250_proto_sel_t;

static const ibm5250_proto_sel_t ibm5250_proto_sels[] = {
    { 0x01, 1,            0            },  /* 0x7E5 -> 0x7ED jz: bit0 = 0 */
    { 0x04, IBM5250_SEL_PHASE, IBM5250_SEL_KEEP },  /* bit1 = phase */
    { 0x08, 1,            1            },  /* 0x7FA -> 0x808 jc: bit0 = 1 */
    { 0x09, 1,            IBM5250_SEL_KEEP }, /* 0x9D7 -> 0x9E8 jnz: bit1 = 1 */
};

#endif /* EMU_IBM5250_PROTO_H */
