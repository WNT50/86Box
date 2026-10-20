/*
 * Host-side unit tests for the IBM 3477-S 5250 adapter protocol tables.
 *
 * No gtest, no emulator: this is a plain C program so that it builds with
 * a single compiler invocation and runs in milliseconds, replacing the
 * four-minute "boot EMDIAG and read the log" cycle for the protocol layer.
 *
 * Every rule asserted here was established from em2.bin (0x840 / 0x720) and
 * the 5250 manuals, so a failure means a real protocol regression.
 */
#include <stdio.h>
#include <stdint.h>

#include "device/ibm5250_proto.h"

static int failures;
static int checks;

#define CHECK(cond)                                                           \
    do {                                                                      \
        checks++;                                                             \
        if (!(cond)) {                                                        \
            failures++;                                                       \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
        }                                                                     \
    } while (0)

#define CHECK_EQ(a, b)                                                        \
    do {                                                                      \
        long _a = (long) (a), _b = (long) (b);                                \
        checks++;                                                             \
        if (_a != _b) {                                                       \
            failures++;                                                       \
            printf("FAIL %s:%d  %s == %s  (%ld != %ld)\n",                    \
                   __FILE__, __LINE__, #a, #b, _a, _b);                       \
        }                                                                     \
    } while (0)

/* ------------------------------------------------------------------ */
/* 0x840: start sequence                                              */
/* ------------------------------------------------------------------ */

static void
test_clock_prologue(void)
{
    /* 0x865 rejects the line when the clock sample right after the sync
       hunt is high, so the first three samples must be 0. */
    for (int c = 0; c < 3; c++)
        CHECK_EQ(ibm5250_proto_clk_bit(c), 0);
}

static void
test_clock_alternation(void)
{
    /* 0x86C compares each sample with the previous one and fails when two
       consecutive samples are equal, so c3..c10 must alternate. */
    for (int c = 3; c < 10; c++)
        CHECK(ibm5250_proto_clk_bit(c) != ibm5250_proto_clk_bit(c + 1));

    CHECK_EQ(ibm5250_proto_clk_bit(3), 1);
}

static void
test_clock_violation(void)
{
    /* 0x882 wants three consecutive 1s (1.5 bit times), 0x88F three 0s. */
    for (int c = 11; c <= 13; c++)
        CHECK_EQ(ibm5250_proto_clk_bit(c), 1);
    for (int c = 14; c <= 16; c++)
        CHECK_EQ(ibm5250_proto_clk_bit(c), 0);
}

/* ------------------------------------------------------------------ */
/* 0x720: 16-bit frame                                                */
/* ------------------------------------------------------------------ */

static void
test_frame_data(void)
{
    const int data = 0xA5;

    /* AL is rebuilt from W bits 1..8; bit k lands in W bit (k+1). */
    for (int k = 0; k < 8; k++)
        CHECK_EQ(ibm5250_proto_frame_bit(k, data, 0), (data >> k) & 1);
}

static void
test_frame_station(void)
{
    for (int k = 8; k <= 10; k++)
        CHECK_EQ(ibm5250_proto_frame_bit(k, 0, 5), (5 >> (k - 8)) & 1);
}

static void
test_frame_parity(void)
{
    for (int data = 0; data < 256; data++) {
        for (int station = 0; station < 8; station++) {
            int ones = 1;               /* sync */

            for (int k = 0; k < 8; k++)
                ones += ibm5250_proto_frame_bit(k, data, station);
            for (int k = 8; k <= 10; k++)
                ones += ibm5250_proto_frame_bit(k, data, station);
            ones += ibm5250_proto_frame_bit(11, data, station);

            CHECK_EQ(ones & 1, 0);
        }
    }
}

static void
test_frame_fill(void)
{
    for (int k = 12; k <= 14; k++)
        CHECK_EQ(ibm5250_proto_frame_bit(k, 0xFF, 7), 0);
}

/* ------------------------------------------------------------------ */
/* EMDIAG 判据的不变式                                                  */
/* ------------------------------------------------------------------ */

/* 0x761 rebuilds AL from the frame; AL must equal the data byte, which is
   what 0xAE7's `cmp al,ch` checks across the 255-frame sequence. */
static void
test_emdiag_al_equals_data(void)
{
    for (int data = 0; data < 256; data++) {
        int al = 0;

        for (int k = 0; k < 8; k++)
            al |= ibm5250_proto_frame_bit(k, data, 7) << k;
        CHECK_EQ(al, data);
    }
}

/* 0xB11: `mov al,bh; or al,0xF1; inc al; jz` requires BH | 0xF1 == 0xFF,
   i.e. W bits 8..11 all set: data bit 7 plus the three station bits. */
static void
test_emdiag_bh(void)
{
    int bh = 0;

    for (int k = 7; k <= 11; k++)
        bh |= ibm5250_proto_frame_bit(k, 0xFF, 7) << (k - 7);

    CHECK_EQ(bh | 0xF1, 0xFF);
    CHECK_EQ(bh & 0x0F, 0x0F);
}

/* 0xB03: `mov al,[0x2A]; or al,0xB7; inc al; jz` needs bits 6 and 3, which
   0x1C1's union value 0xDA supplies. */
static void
test_emdiag_status_union(void)
{
    CHECK_EQ((uint8_t) ((0xDA | 0xB7) + 1), 0);
}

/* ------------------------------------------------------------------ */
/* 0x1C2 selector table                                               */
/* ------------------------------------------------------------------ */

static const ibm5250_proto_sel_t *
find_sel(uint8_t code)
{
    for (size_t i = 0; i < sizeof(ibm5250_proto_sels) / sizeof(ibm5250_proto_sels[0]); i++) {
        if (ibm5250_proto_sels[i].code == code)
            return &ibm5250_proto_sels[i];
    }
    return NULL;
}

static void
test_selectors(void)
{
    const ibm5250_proto_sel_t *s;

    s = find_sel(0x01);
    CHECK(s != NULL);
    if (s) CHECK_EQ(s->b0, 0);          /* 0x7ED jz */

    s = find_sel(0x08);
    CHECK(s != NULL);
    if (s) { CHECK_EQ(s->b1, 1); CHECK_EQ(s->b0, 1); }  /* 0x808 jc */

    s = find_sel(0x09);
    CHECK(s != NULL);
    if (s) CHECK_EQ(s->b1, 1);          /* 0x9E8 jnz */

    s = find_sel(0x04);
    CHECK(s != NULL);
    if (s) {
        CHECK_EQ(s->b1, IBM5250_SEL_PHASE);
        CHECK_EQ(s->b0, IBM5250_SEL_KEEP);
    }

    CHECK(find_sel(0x0A) == NULL);      /* 0x0A 必须保持命令内局部 */
}

int
main(void)
{
    test_clock_prologue();
    test_clock_alternation();
    test_clock_violation();
    test_frame_data();
    test_frame_station();
    test_frame_parity();
    test_frame_fill();
    test_emdiag_al_equals_data();
    test_emdiag_bh();
    test_emdiag_status_union();
    test_selectors();

    printf("ibm5250 proto: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
