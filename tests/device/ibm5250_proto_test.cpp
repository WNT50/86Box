/*
 * Host-side unit tests for the IBM 3477-S 5250 adapter protocol tables.
 *
 * These replace the slow "boot EMDIAG and read the log" loop for the
 * protocol layer: every rule below was originally established by
 * disassembling em2.bin (0x840 / 0x720) and single-stepping the guest,
 * so a regression here means a real protocol regression.
 */
#include <gtest/gtest.h>

#include "device/ibm5250_proto.h"

/* ------------------------------------------------------------------ */
/* 0x840: start sequence                                              */
/* ------------------------------------------------------------------ */

TEST(Ibm5250Proto, ClockPrologueIsLow)
{
    /* 0x865 rejects the line when the clock sample right after the sync
       hunt is high, so the first three samples must be 0. */
    for (int c = 0; c < 3; c++)
        EXPECT_EQ(ibm5250_proto_clk_bit(c), 0) << "c=" << c;
}

TEST(Ibm5250Proto, ClockAlternatesForEightSamples)
{
    /* 0x86C compares each sample with the previous one and fails when two
       consecutive samples are equal, so c3..c10 must alternate. */
    for (int c = 3; c < 10; c++) {
        EXPECT_NE(ibm5250_proto_clk_bit(c), ibm5250_proto_clk_bit(c + 1))
            << "c=" << c;
    }
    EXPECT_EQ(ibm5250_proto_clk_bit(3), 1);
}

TEST(Ibm5250Proto, ClockViolationIsThreeOnesThenThreeZeros)
{
    /* 0x882 wants three consecutive 1s (1.5 bit times), 0x88F three 0s. */
    for (int c = 11; c <= 13; c++)
        EXPECT_EQ(ibm5250_proto_clk_bit(c), 1) << "c=" << c;
    for (int c = 14; c <= 16; c++)
        EXPECT_EQ(ibm5250_proto_clk_bit(c), 0) << "c=" << c;
}

/* ------------------------------------------------------------------ */
/* 0x720: 16-bit frame                                                */
/* ------------------------------------------------------------------ */

TEST(Ibm5250Proto, DataByteIsFirstEightBitsLsbFirst)
{
    const int data = 0xA5;

    for (int k = 0; k < 8; k++)
        EXPECT_EQ(ibm5250_proto_frame_bit(k, data, 0), (data >> k) & 1)
            << "k=" << k;
}

TEST(Ibm5250Proto, StationOccupiesThreeBits)
{
    for (int k = 8; k <= 10; k++)
        EXPECT_EQ(ibm5250_proto_frame_bit(k, 0, 5), (5 >> (k - 8)) & 1)
            << "k=" << k;
}

TEST(Ibm5250Proto, ParityMakesTheTwelveBitGroupEven)
{
    for (int data = 0; data < 256; data++) {
        for (int station = 0; station < 8; station++) {
            int ones = 1;               /* sync */

            for (int k = 0; k < 8; k++)
                ones += ibm5250_proto_frame_bit(k, data, station);
            for (int k = 8; k <= 10; k++)
                ones += ibm5250_proto_frame_bit(k, data, station);
            ones += ibm5250_proto_frame_bit(11, data, station);

            EXPECT_EQ(ones & 1, 0)
                << "data=" << data << " station=" << station;
        }
    }
}

TEST(Ibm5250Proto, FillBitsAreZero)
{
    for (int k = 12; k <= 14; k++)
        EXPECT_EQ(ibm5250_proto_frame_bit(k, 0xFF, 7), 0) << "k=" << k;
}

/* ------------------------------------------------------------------ */
/* EMDIAG 判据的不变式                                                  */
/* ------------------------------------------------------------------ */

/* 0x761 rebuilds AL from W bits 1..8; bit k sits in W bit (k+1), so AL is
   exactly the data byte.  This is what 0xAE7's `cmp al,ch` checks for the
   255-frame incrementing sequence. */
TEST(Ibm5250Proto, EmdiagAlEqualsDataByte)
{
    for (int data = 0; data < 256; data++) {
        int al = 0;

        for (int k = 0; k < 8; k++)
            al |= ibm5250_proto_frame_bit(k, data, 7) << k;
        EXPECT_EQ(al, data) << "data=" << data;
    }
}

/* 0xB11: `mov al,bh; or al,0xF1; inc al; jz` requires BH | 0xF1 == 0xFF,
   i.e. W bits 8..11 all set: data bit 7 plus the three station bits. */
TEST(Ibm5250Proto, EmdiagBhRequiresStationSevenAndDataBit7)
{
    int bh = 0;

    for (int k = 7; k <= 11; k++)
        bh |= ibm5250_proto_frame_bit(k, 0xFF, 7) << (k - 7);   /* W8..W12 */

    EXPECT_EQ(bh | 0xF1, 0xFF);
    EXPECT_EQ(bh & 0x0F, 0x0F);
}

/* 0xB03: `mov al,[0x2A]; or al,0xB7; inc al; jz` requires the ISR sample to
   supply bits 6 and 3 (0x1C1's union value 0xDA does). */
TEST(Ibm5250Proto, EmdiagStatusUnionSatisfiesB03)
{
    const uint8_t status = 0xDA;

    EXPECT_EQ((uint8_t) ((status | 0xB7) + 1), 0);
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
    return nullptr;
}

TEST(Ibm5250Proto, SelectorTableMatchesEmdiagChecks)
{
    const ibm5250_proto_sel_t *s;

    s = find_sel(0x01);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->b0, 0);            /* 0x7ED jz */

    s = find_sel(0x08);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->b1, 1);            /* 0x808 jc needs bit0 */
    EXPECT_EQ(s->b0, 1);

    s = find_sel(0x09);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->b1, 1);            /* 0x9E8 jnz */

    s = find_sel(0x04);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(s->b1, IBM5250_SEL_PHASE);
    EXPECT_EQ(s->b0, IBM5250_SEL_KEEP);

    EXPECT_EQ(find_sel(0x0A), nullptr);   /* 0x0A must stay command-scoped */
}
