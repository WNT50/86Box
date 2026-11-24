/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          Implementation of the National Semiconductor DP8393x
 *          SONIC/SONIC-T Ethernet controller and the IBM LAN Adapter/A
 *          for Ethernet (Micro Channel).
 *
 *          The file is organised in four sections:
 *            1. common definitions and the controller instance
 *            2. the SONIC/SONIC-T controller core
 *            3. the MCA board (POS, shared memory window, boot ROM)
 *            4. device registration and life cycle
 */

#include <stdarg.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/thread.h>
#include <86box/timer.h>
#include <86box/network.h>
#include <86box/net_dp8393x.h>

/* =========================================================================
 * Section 1: Common definitions
 * ========================================================================= */

/* Chip constants and dp8393x_t live in <86box/net_dp8393x.h>; board-level
 * code lives in net_ibmlana.c, so this file holds no board state. */

#define ENABLE_DP8393X_LOG 1
#if ENABLE_DP8393X_LOG
int dp8393x_do_log = ENABLE_DP8393X_LOG;

static void
dp8393x_log(const char *fmt, ...)
{
    va_list ap;

    if (dp8393x_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define dp8393x_log(fmt, ...)
#endif

/* =========================================================================
 * Section 2: SONIC/SONIC-T controller core
 *
 * P1 implements the register array, write masks, reset defaults, CAM load and
 * interrupt aggregation. The TX/RX state machines arrive in P2/P3; here only
 * the command and status bit landing points are kept.
 * ========================================================================= */

enum {
    SONIC_CR   = 0x00,
    SONIC_DCR  = 0x02,
    SONIC_RCR  = 0x04,
    SONIC_TCR  = 0x06,
    SONIC_IMR  = 0x08,
    SONIC_ISR  = 0x0A,
    SONIC_UTDA = 0x0C,
    SONIC_CTDA = 0x0E,
    SONIC_URDA = 0x1A,
    SONIC_CRDA = 0x1C,
    SONIC_EOBC = 0x26,
    SONIC_URRA = 0x28,
    SONIC_RSA  = 0x2A,
    SONIC_REA  = 0x2C,
    SONIC_RRP  = 0x2E,
    SONIC_RWP  = 0x30,
    SONIC_CEP  = 0x42,
    SONIC_CAP2 = 0x44,
    SONIC_CAP1 = 0x46,
    SONIC_CAP0 = 0x48,
    SONIC_CE   = 0x4A,
    SONIC_CDP  = 0x4C,
    SONIC_CDC  = 0x4E,
    SONIC_SR   = 0x50,
    SONIC_WT0  = 0x52,
    SONIC_WT1  = 0x54,
    SONIC_RSC  = 0x56,
    SONIC_CRCT = 0x58,
    SONIC_FAET = 0x5A,
    SONIC_MPT  = 0x5C,
    SONIC_DCR2 = 0x7E
};

#define SONIC_CR_HTX   0x0001
#define SONIC_CR_TXP   0x0002
#define SONIC_CR_RXDIS 0x0004
#define SONIC_CR_RXEN  0x0008
#define SONIC_CR_STP   0x0010
#define SONIC_CR_ST    0x0020
#define SONIC_CR_RST   0x0080
#define SONIC_CR_RRRA  0x0100
#define SONIC_CR_LCAM  0x0200

#define SONIC_ISR_MP    0x0002
#define SONIC_ISR_FAE   0x0004
#define SONIC_ISR_CRC   0x0008
#define SONIC_ISR_RBAE  0x0010
#define SONIC_ISR_RBE   0x0020
#define SONIC_ISR_RDE   0x0040
#define SONIC_ISR_TC    0x0080
#define SONIC_ISR_TXER  0x0100
#define SONIC_ISR_PTDN  0x0200
#define SONIC_ISR_PKTRX 0x0400
#define SONIC_ISR_PINT  0x0800
#define SONIC_ISR_LCD   0x1000

/* The watchdog decrements at 1/2 TXC = 5 MHz; a 1 ms timer drops 5000 counts each tick. */
#define SONIC_WT_TICK_US     1000
#define SONIC_WT_TICK_COUNTS 5000u

/* Receive Control Register: configuration bits 15-9, status bits 8-0. */
#define SONIC_RCR_ERR  0x8000
#define SONIC_RCR_RNT  0x4000
#define SONIC_RCR_BRD  0x2000
#define SONIC_RCR_PRO  0x1000
#define SONIC_RCR_AMC  0x0800

/* Loopback mode select (LB1-0, bits 10-9) and status bit LBK. */
#define SONIC_RCR_LB_MASK 0x0600
#define SONIC_RCR_MC   0x0100
#define SONIC_RCR_BC   0x0080
#define SONIC_RCR_LPKT 0x0040
#define SONIC_RCR_CRS  0x0020
#define SONIC_RCR_COL  0x0010
#define SONIC_RCR_CRCR 0x0008
#define SONIC_RCR_FAER 0x0004
#define SONIC_RCR_LBK  0x0002
#define SONIC_RCR_PRX  0x0001
#define SONIC_RCR_STATUS_MASK 0x01FF

/* Receive descriptor (RDA) layout, 28-byte slots. */
#define SONIC_RDA_STATUS  0x00
#define SONIC_RDA_CONFIG  0x02
#define SONIC_RDA_BYTES   0x04
#define SONIC_RDA_ADDR_LO 0x08
#define SONIC_RDA_ADDR_HI 0x0C
#define SONIC_RDA_LINK    0x14
#define SONIC_RDA_SEQ     0x18

/* Receive resource (RRA) layout, 16-byte slots. */
#define SONIC_RRA_ADDR_LO 0x00
#define SONIC_RRA_ADDR_HI 0x04
#define SONIC_RRA_WORDS   0x08
#define SONIC_RRA_STEP    16

/* Transmit descriptor (TDA) layout, 32-byte slots. */
#define SONIC_TDA_STATUS       0x00
#define SONIC_TDA_CONFIG       0x04
#define SONIC_TDA_SIZE         0x08
#define SONIC_TDA_FRAGS        0x0C
#define SONIC_TDA_FRAG         0x10
#define SONIC_TDA_FRAG_STEP    12
#define SONIC_TDA_CONFIG_PINTR 0x8000

/* Transmit status bits written back to TXpkt.status. */
#define SONIC_TSR_PTX  0x0001
#define SONIC_TSR_BCM  0x0002
#define SONIC_TSR_FU   0x0004
#define SONIC_TSR_PMB  0x0008
#define SONIC_TSR_OWC  0x0020
#define SONIC_TSR_EXC  0x0040
#define SONIC_TSR_CRSL 0x0080
#define SONIC_TSR_NCRS 0x0100
#define SONIC_TSR_DEF  0x0200
#define SONIC_TSR_EXD  0x0400
#define SONIC_TSR_ERR  (SONIC_TSR_BCM | SONIC_TSR_FU | SONIC_TSR_EXC | SONIC_TSR_EXD)

static uint16_t
dp8393x_ram_read16(const dp8393x_t *dev, uint16_t off)
{
    uint32_t size = dev->ram_size ? dev->ram_size : SONIC_RAM_SIZE;
    uint32_t o    = (uint32_t) (off & (size - 1)) & (SONIC_RAM_SIZE - 1);

    return (uint16_t) (dev->ram[o] | (dev->ram[(o + 1) & (SONIC_RAM_SIZE - 1)] << 8));
}

uint8_t
dp8393x_ram_read8(const dp8393x_t *dev, uint16_t off)
{
    uint32_t size = dev->ram_size ? dev->ram_size : SONIC_RAM_SIZE;
    uint32_t o    = (uint32_t) (off & (size - 1)) & (SONIC_RAM_SIZE - 1);

    return dev->ram[o];
}

static uint32_t
dp8393x_ram_read32(const dp8393x_t *dev, uint16_t off)
{
    return (uint32_t) dp8393x_ram_read16(dev, off)
         | ((uint32_t) dp8393x_ram_read16(dev, (uint16_t) (off + 2)) << 16);
}

void
dp8393x_ram_write8(dp8393x_t *dev, uint16_t off, uint8_t val)
{
    uint32_t size = dev->ram_size ? dev->ram_size : SONIC_RAM_SIZE;
    uint32_t o    = (uint32_t) (off & (size - 1)) & (SONIC_RAM_SIZE - 1);

    dev->ram[o] = val;
}

static void
dp8393x_ram_write16(dp8393x_t *dev, uint16_t off, uint16_t val)
{
    uint32_t size = dev->ram_size ? dev->ram_size : SONIC_RAM_SIZE;
    uint32_t o    = (uint32_t) (off & (size - 1)) & (SONIC_RAM_SIZE - 1);

    dev->ram[o]                             = (uint8_t) (val & 0xFF);
    dev->ram[(o + 1) & (SONIC_RAM_SIZE - 1)] = (uint8_t) (val >> 8);
}

static void
dp8393x_ram_write32(dp8393x_t *dev, uint16_t off, uint32_t val)
{
    dp8393x_ram_write16(dev, off, (uint16_t) val);
    dp8393x_ram_write16(dev, (uint16_t) (off + 2), (uint16_t) (val >> 16));
}

static void
dp8393x_update_irq(dp8393x_t *dev)
{
    int level = ((dev->regs[SONIC_ISR >> 1] & dev->regs[SONIC_IMR >> 1]) != 0) ? 1 : 0;

    if (level == dev->irq_level)
        return;

    dev->irq_level = (uint8_t) level;
    if (dev->irq_cb != NULL)
        dev->irq_cb(dev->irq_priv, level);
}

static void
dp8393x_raise_isr(dp8393x_t *dev, uint16_t bits)
{
    dev->regs[SONIC_ISR >> 1] |= bits;
    dp8393x_update_irq(dev);
}

/* Generic watchdog: counts down from the WT1/WT0 load value and sets
 * ISR.TC on the 0 -> 0xFFFFFFFF wrap.
 * Manual 2.7.14: clocked by 1/2 TXC, started by ST, stopped by STP/RST. */
static void
dp8393x_wt_tick(void *priv)
{
    dp8393x_t *dev = (dp8393x_t *) priv;

    if (!dev->wt_running)
        return;

    if (dev->wt_count < SONIC_WT_TICK_COUNTS) {
        dev->wt_count = 0xFFFFFFFFu - (SONIC_WT_TICK_COUNTS - dev->wt_count - 1u);
        dp8393x_raise_isr(dev, SONIC_ISR_TC);
    } else {
        dev->wt_count -= SONIC_WT_TICK_COUNTS;
    }

    timer_set_delay_u64(&dev->wt_timer, SONIC_WT_TICK_US * TIMER_USEC);
}

static void
dp8393x_wt_start(dp8393x_t *dev)
{
    if (dev->wt_running)
        return;

    dev->wt_running = 1;
    timer_set_delay_u64(&dev->wt_timer, SONIC_WT_TICK_US * TIMER_USEC);
}

static void
dp8393x_wt_stop(dp8393x_t *dev)
{
    if (!dev->wt_running)
        return;

    dev->wt_running = 0;
    timer_disable(&dev->wt_timer);
}

void
dp8393x_reset(dp8393x_t *dev)
{
    memset(dev->regs, 0x00, sizeof(dev->regs));
    memset(dev->cam, 0x00, sizeof(dev->cam));

    dev->cam_enable = 0x0000;
    dev->cep        = 0x0000;
    dev->cam_valid  = 0x00;
    dev->rx_enabled = 0;
    dev->stopped    = 1;

    dev->rx_buf      = 0;
    dev->rx_words    = 0;
    dev->rx_rba_full = 0;
    dev->rx_rbe      = 0;
    dev->rx_rde      = 0;
    dev->rx_pkt_seq  = 0;
    dev->rx_rba_seq  = 0;

    dp8393x_wt_stop(dev);
    dev->wt_count = 0;

    /* Hardware reset: RST|STP|RXDIS = 0094h. */
    dev->regs[SONIC_CR >> 1] = SONIC_CR_RST | SONIC_CR_STP | SONIC_CR_RXDIS;

    dp8393x_update_irq(dev);
}

/* Software reset (CR.RST=1). Manual 2.7.1: only the internal state machines
 * are reset; the CRC generator is disabled and the tallies are paused but not
 * cleared. Pointers such as CTDA/CRDA/RRP/RWP, CAM/CE, CDC/CEP and RSC are
 * unaffected by a software reset (they are cleared only by a hardware reset). */
static void
dp8393x_sw_reset(dp8393x_t *dev)
{
    dev->tx_active   = 0;
    dev->rx_buf      = 0;
    dev->rx_words    = 0;
    dev->rx_rba_full = 0;
    dev->rx_rbe      = 0;
    dev->rx_rde      = 0;
    dev->rx_enabled  = 0;

    /* Manual 2.7.14: reset stops the timer but does not clear it. */
    dp8393x_wt_stop(dev);

    /* Software reset clears bits 9/8/1/0 (LCAM/RRRA/TXP/HTX) and sets bits 7/2
     * (RST/RXDIS); ST, STP and the other bits are preserved. */
    dev->regs[SONIC_CR >> 1] = (uint16_t) ((dev->regs[SONIC_CR >> 1]
                                           & (uint16_t) ~(SONIC_CR_LCAM | SONIC_CR_RRRA
                                                          | SONIC_CR_TXP | SONIC_CR_HTX
                                                          | SONIC_CR_RXEN))
                                          | SONIC_CR_RST | SONIC_CR_RXDIS);
}

/* LCAM: CDP points to the CAM descriptor area inside the window and CDC is the
 * descriptor count. Each descriptor is 16 bytes: +0 control/entry number,
 * +4/+8/+0C the 6-byte address. */
static void
dp8393x_cam_load(dp8393x_t *dev)
{
    uint16_t cdp = dev->regs[SONIC_CDP >> 1];
    uint16_t cdc = dev->regs[SONIC_CDC >> 1] & 0x001F;
    uint16_t i;

    memset(dev->cam, 0x00, sizeof(dev->cam));
    dev->cam_valid = 0x00;

    for (i = 0; i < cdc; i++) {
        uint16_t base = (uint16_t) (cdp + (i << 4));
        uint16_t ctrl = dp8393x_ram_read16(dev, base);
        uint16_t slot = ctrl & 0x000F;

        if (slot == 0x0000)
            slot = (uint16_t) (i & 0x000F);

        dev->cam[(slot * 3) + 0] = dp8393x_ram_read16(dev, (uint16_t) (base + 0x04));
        dev->cam[(slot * 3) + 1] = dp8393x_ram_read16(dev, (uint16_t) (base + 0x08));
        dev->cam[(slot * 3) + 2] = dp8393x_ram_read16(dev, (uint16_t) (base + 0x0C));
        dev->cam_valid |= (uint16_t) (1u << slot);
    }

    /* The CAM Enable field immediately follows the last descriptor. */
    dev->cam_enable = dp8393x_ram_read16(dev, (uint16_t) (cdp + (cdc << 4)));

    /* Manual: after LCAM completes, CDP points past the CE field and CDC is
     * cleared. TX/RX are synchronous in this model, so CAM DMA does not need to
     * wait for a transfer to finish. */
    dev->regs[SONIC_CDP >> 1] = (uint16_t) (cdp + ((cdc + 1) << 4));
    dev->regs[SONIC_CDC >> 1] = 0x0000;

    dev->regs[SONIC_CR >> 1] &= (uint16_t) ~SONIC_CR_LCAM;
    dp8393x_raise_isr(dev, SONIC_ISR_LCD);
}

/* -------------------------------------------------------------------------
 * Receive path (P3)
 *
 * Descriptor layout is taken from IBMENIIN.SYS: an RDA is a 28-byte slot where
 * status/byte_count/packet address are words and the link is a dword whose LSB
 * is EOL; an RRA is a 16-byte slot with a 24-bit buffer address and a size in
 * 16-bit words. The manual names RDA +0x18 RXpkt.seq_no, but the driver treats
 * it as a "descriptor occupied" flag (initialized to 1, processed only when 0,
 * written back as 1 when done). Driver behaviour is the reference here: the
 * packet is delivered with 0. See C6 in p3/spec.md.
 * ------------------------------------------------------------------------- */

static uint32_t
dp8393x_crc32(const uint8_t *buf, int len)
{
    uint32_t crc = 0xFFFFFFFF;
    int      i;
    int      b;

    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t) -(int32_t) (crc & 1));
    }

    return ~crc;
}

/* 48-bit CAM compare: the first byte on the wire is the low address byte, so
 * assemble little-endian words. */
static int
dp8393x_cam_match(const dp8393x_t *dev, const uint8_t *addr)
{
    uint16_t w0 = (uint16_t) (addr[0] | (addr[1] << 8));
    uint16_t w1 = (uint16_t) (addr[2] | (addr[3] << 8));
    uint16_t w2 = (uint16_t) (addr[4] | (addr[5] << 8));
    uint16_t enabled = (uint16_t) (dev->cam_enable & dev->cam_valid);
    int      i;

    for (i = 0; i < 16; i++) {
        if (!(enabled & (1u << i)))
            continue;

        if ((dev->cam[(i * 3) + 0] == w0) && (dev->cam[(i * 3) + 1] == w1)
            && (dev->cam[(i * 3) + 2] == w2))
            return 1;
    }

    return 0;
}

/* Manual 2.7.3 / 2.7.12 filter order: PRO -> AMC -> BRD -> CAM (RJCM only inverts CAM). */
static int
dp8393x_rx_accept(const dp8393x_t *dev, const uint8_t *buf, int len, uint16_t *status)
{
    uint16_t rcr = dev->regs[SONIC_RCR >> 1];
    int      broadcast = (buf[0] == 0xFF) && (buf[1] == 0xFF) && (buf[2] == 0xFF)
                  && (buf[3] == 0xFF) && (buf[4] == 0xFF) && (buf[5] == 0xFF);
    int      multicast = !broadcast && (buf[0] & 0x01);
    int      accept    = 0;

    *status = 0;

    /* RNT=0 rejects short frames; fewer than 64 bytes on the wire means fewer
     * than 60 bytes of payload. */
    if (!(rcr & SONIC_RCR_RNT) && ((len + 4) < 64))
        return 0;

    if (broadcast)
        accept = (rcr & (SONIC_RCR_BRD | SONIC_RCR_AMC)) != 0;
    else if (multicast)
        accept = (rcr & SONIC_RCR_AMC) != 0;
    else
        accept = (rcr & SONIC_RCR_PRO) != 0;

    if (!accept && dp8393x_cam_match(dev, buf))
        accept = (dev->regs[SONIC_DCR2 >> 1] & 0x0001) ? 0 : 1; /* RJCM */

    if (!accept)
        return 0;

    *status = SONIC_RCR_PRX;
    if (broadcast)
        *status |= SONIC_RCR_BC;
    else if (multicast)
        *status |= SONIC_RCR_MC;

    return 1;
}

/* RRRA: read the next RRA at RRP and load CRBA/RBWC. RRP wraps to RSA after REA. */
static int
dp8393x_read_rra(dp8393x_t *dev)
{
    uint16_t rrp = (uint16_t) (dev->regs[SONIC_RRP >> 1] & 0xFFFE);
    uint16_t rwp = (uint16_t) (dev->regs[SONIC_RWP >> 1] & 0xFFFE);
    uint16_t rsa = (uint16_t) (dev->regs[SONIC_RSA >> 1] & 0xFFFE);
    uint16_t rea = (uint16_t) (dev->regs[SONIC_REA >> 1] & 0xFFFE);
    uint32_t lo  = dp8393x_ram_read32(dev, rrp) & 0xFFFF;
    uint32_t hi  = dp8393x_ram_read32(dev, (uint16_t) (rrp + SONIC_RRA_ADDR_HI)) & 0xFF;
    uint32_t words = dp8393x_ram_read32(dev, (uint16_t) (rrp + SONIC_RRA_WORDS)) & 0xFFFF;

    dev->rx_buf      = (lo & 0xFFFE) | (hi << 16);
    dev->rx_words    = words;
    dev->rx_rba_full = 0;
    dev->rx_pkt_seq  = 0;
    dev->rx_rba_seq++;

    rrp = (uint16_t) (rrp + SONIC_RRA_STEP);
    if ((rea != 0x0000) && (rrp >= rea))
        rrp = rsa;
    dev->regs[SONIC_RRP >> 1] = rrp;

    /* Manual: after reading the last RRA, RRP == RWP sets RBE. */
    if ((rrp & 0xFFFE) == rwp) {
        dev->rx_rbe = 1;
        dp8393x_raise_isr(dev, SONIC_ISR_RBE);
    }

    return (dev->rx_words != 0) ? 1 : 0;
}

/* Tally increment: sets the matching ISR bit when the counter wraps 0000 -> FFFF
 * (manual 2.7.11). */
static void
dp8393x_tally_inc(dp8393x_t *dev, uint16_t reg, uint16_t isr_bit)
{
    uint16_t val = (uint16_t) (dev->regs[reg >> 1] + 1);

    dev->regs[reg >> 1] = val;
    if (val == 0x0000)
        dp8393x_raise_isr(dev, isr_bit);
}

/* Receive: filter -> take RBA -> write data + FCS -> write RDA -> advance CRDA / raise IRQ. */
int
dp8393x_rx(dp8393x_t *dev, const uint8_t *buf, int len, int loopback)
{
    uint16_t status;
    uint16_t rcr;
    uint16_t rda;
    uint16_t link;
    uint32_t pkt_addr;
    uint32_t used;
    uint32_t crc;
    int      i;

    if (!dev->rx_enabled || (len < 14) || (len > NET_MAX_FRAME)) {
        dp8393x_tally_inc(dev, SONIC_MPT, SONIC_ISR_MP);
        return 0;
    }

    if (!dp8393x_rx_accept(dev, buf, len, &status)) {
        dp8393x_tally_inc(dev, SONIC_MPT, SONIC_ISR_MP);
        return 0;
    }

    /* Reception stops once the RDA ring is exhausted, until software clears RDE. */
    if (dev->rx_rde) {
        dp8393x_tally_inc(dev, SONIC_MPT, SONIC_ISR_MP);
        return 0;
    }

    if (dev->rx_rba_full || (dev->rx_words == 0)) {
        if (dev->rx_rbe || !dp8393x_read_rra(dev)) {
            dp8393x_tally_inc(dev, SONIC_MPT, SONIC_ISR_MP);
            return 0;
        }
    }

    /* Data + 4-byte FCS, occupying the RBA aligned to 32 bits. */
    used = ((uint32_t) len + 4u + 3u) & ~3u;

    if ((used / 2u) > dev->rx_words) {
        /* The buffer cannot hold it: drop the packet without writing an RDA and
         * move to the next RRA. */
        dp8393x_raise_isr(dev, SONIC_ISR_RBAE);
        dp8393x_tally_inc(dev, SONIC_MPT, SONIC_ISR_MP);
        if (!dev->rx_rbe)
            dp8393x_read_rra(dev);
        return 0;
    }

    pkt_addr = dev->rx_buf;

    for (i = 0; i < len; i++)
        dp8393x_ram_write8(dev, (uint16_t) (pkt_addr + (uint32_t) i), buf[i]);

    /* 86Box backend frames carry no FCS; the driver expects the length to include
     * it, so append CRC32 at the end of the RBA. */
    crc = dp8393x_crc32(buf, len);
    for (i = 0; i < 4; i++)
        dp8393x_ram_write8(dev, (uint16_t) (pkt_addr + (uint32_t) len + (uint32_t) i),
                           (uint8_t) (crc >> (i * 8)));

    dev->rx_buf += used;
    dev->rx_words -= used / 2u;

    if (loopback)
        status |= SONIC_RCR_LBK;

    /* LPKT: the remaining word count is below one maximum frame, so mark this
     * packet as the last one in the current RBA. */
    if (dev->rx_words <= (uint32_t) (dev->regs[SONIC_EOBC >> 1] & 0xFFFF)) {
        status |= SONIC_RCR_LPKT;
        dev->rx_rba_full = 1;
    }

    /* The low 9 bits of RCR are the receive status of the latest packet and are
     * then written into RXpkt.status. */
    rcr = dev->regs[SONIC_RCR >> 1];
    dev->regs[SONIC_RCR >> 1] = (uint16_t) ((rcr & (uint16_t) ~SONIC_RCR_STATUS_MASK) | status);

    rda  = (uint16_t) (dev->regs[SONIC_CRDA >> 1] & 0xFFFE);
    dp8393x_ram_write16(dev, (uint16_t) (rda + SONIC_RDA_STATUS), status);
    dp8393x_ram_write16(dev, (uint16_t) (rda + SONIC_RDA_BYTES), (uint16_t) (len + 4));
    dp8393x_ram_write16(dev, (uint16_t) (rda + SONIC_RDA_ADDR_LO), (uint16_t) (pkt_addr & 0xFFFF));
    dp8393x_ram_write16(dev, (uint16_t) (rda + SONIC_RDA_ADDR_HI), (uint16_t) (pkt_addr >> 16));

    /* The manual names +0x18 RXpkt.seq_no, but the driver treats it as a
     * "descriptor occupied" flag: it initializes it to 1, processes only when 0
     * (0x12BC0), and writes 1 back when done (0x12EAD). Driver behaviour is the
     * reference here, so 0 is written on delivery. See C6 in p3/spec.md. */
    dp8393x_ram_write32(dev, (uint16_t) (rda + SONIC_RDA_SEQ), 0x00000000);

    dev->rx_pkt_seq++;
    dev->regs[SONIC_RSC >> 1] = (uint16_t) ((dev->rx_rba_seq << 8) | dev->rx_pkt_seq);

    link = (uint16_t) dp8393x_ram_read32(dev, (uint16_t) (rda + SONIC_RDA_LINK));
    dev->regs[SONIC_CRDA >> 1] = link;

    dp8393x_raise_isr(dev, SONIC_ISR_PKTRX);

    if (link & 0x0001) {
        dev->rx_rde = 1;
        dp8393x_raise_isr(dev, SONIC_ISR_RDE);
    }

    return 1;
}

/* TXP: walk the transmit descriptor chain starting at CTDA.
 *
 * The descriptor layout is the one measured in IBMENIIN.SYS: a 32-byte slot
 * with dword fields, fragments starting at +0x10 and 12 bytes apart, and the
 * link field after the last fragment (bit 0 is EOL; bit 15 of the config word
 * is PINTR, not EOL).  The real driver always terminates each TDA with EOL and
 * re-issues TXP per packet, but the chain walk follows the manual.
 *
 * Multi-fragment descriptors place the link beyond the 32-byte slot; this is
 * kept for compatibility because the driver only ever uses one fragment.
 */
static void
dp8393x_tx(dp8393x_t *dev)
{
    uint16_t tda;

    if (dev->tx_active) {
        /* TXP must not be set while a transmission is in progress. */
        dp8393x_log("SONIC: TXP set again during transmission, ignored\n");
        return;
    }

    dev->tx_active = 1;
    tda = (uint16_t) (dev->regs[SONIC_CTDA >> 1] & 0xFFFE);

    for (;;) {
        uint16_t config = dp8393x_ram_read16(dev, (uint16_t) (tda + SONIC_TDA_CONFIG));
        uint32_t size   = dp8393x_ram_read32(dev, (uint16_t) (tda + SONIC_TDA_SIZE));
        uint32_t frags  = dp8393x_ram_read32(dev, (uint16_t) (tda + SONIC_TDA_FRAGS));
        uint32_t gather = 0;
        uint32_t i;
        uint16_t status = 0x0000;
        uint16_t link;
        uint8_t  abort = 0;

        /* TXpkt.config carries the TCR configuration bits; the status bits
         * start cleared for every packet. */
        dev->regs[SONIC_TCR >> 1] = (uint16_t) (config & 0xF000);

        /* PINTR raises PINT as soon as the descriptor has been read. */
        if (config & SONIC_TDA_CONFIG_PINTR)
            dp8393x_raise_isr(dev, SONIC_ISR_PINT);

        if ((frags == 0) || (frags > 16) || (size > SONIC_TX_FRAME_MAX)) {
            status = SONIC_TSR_BCM;
            abort  = 1;
        } else {
            for (i = 0; i < frags; i++) {
                uint16_t frag = (uint16_t) (tda + SONIC_TDA_FRAG + (i * SONIC_TDA_FRAG_STEP));
                uint32_t addr = (uint32_t) (dp8393x_ram_read32(dev, frag) & 0xFFFF);
                uint32_t len  = dp8393x_ram_read32(dev, (uint16_t) (frag + 0x08));
                uint32_t b;

                addr |= (uint32_t) dp8393x_ram_read8(dev, (uint16_t) (frag + 0x04)) << 16;

                if ((len > (size - gather)) || ((gather + len) > SONIC_TX_FRAME_MAX)) {
                    status = SONIC_TSR_BCM;
                    abort  = 1;
                    break;
                }

                for (b = 0; b < len; b++)
                    dev->tx_frame[gather + b] = dp8393x_ram_read8(dev, (uint16_t) (addr + b));

                gather += len;
            }

            if (!abort && (gather != size)) {
                status = SONIC_TSR_BCM;
                abort  = 1;
            }
        }

        if (!abort) {
            uint16_t lb = (uint16_t) (dev->regs[SONIC_RCR >> 1] & SONIC_RCR_LB_MASK);

            status = SONIC_TSR_PTX;

            if (lb != 0) {
                /* Loopback diagnostics (used by the Boot ROM POST): data does not
                 * go out but enters the receive engine directly with LBK in the
                 * RDA status. The manual requires CAM/RCR to accept the
                 * destination address. */
                dp8393x_rx(dev, dev->tx_frame, (int) size, 1);
            } else if (dev->card != NULL) {
                /* 86Box backend frames carry no FCS; descriptor data is handed up as-is. */
                network_tx(dev->card, dev->tx_frame, (int) size);
            }
        }

        /* Write TXpkt.status and clear the TCR status bits after write-back
         * (this is also what clears OWC/DEF).  Bits 15-11 are the collision
         * count read by the driver as status >> 11; 86Box injects no
         * collisions, so they stay 0. */
        dp8393x_ram_write32(dev, (uint16_t) (tda + SONIC_TDA_STATUS), status);
        dev->regs[SONIC_TCR >> 1] = (uint16_t) ((dev->regs[SONIC_TCR >> 1] & 0xF000) | status);

        if (status & SONIC_TSR_ERR) {
            dp8393x_raise_isr(dev, SONIC_ISR_TXER);
            dp8393x_raise_isr(dev, SONIC_ISR_PTDN);
            break;
        }

        link = dp8393x_ram_read16(dev,
                                  (uint16_t) (tda + SONIC_TDA_FRAG + (frags * SONIC_TDA_FRAG_STEP)));

        if (link & 0x0001) {
            dev->regs[SONIC_CTDA >> 1] = link;
            dp8393x_raise_isr(dev, SONIC_ISR_PTDN);
            break;
        }

        if (dev->regs[SONIC_CR >> 1] & SONIC_CR_HTX) {
            /* HTX: stop after this packet; CTDA points at the last
             * transmitted descriptor. */
            dev->regs[SONIC_CTDA >> 1] = tda;
            dp8393x_raise_isr(dev, SONIC_ISR_PTDN);
            break;
        }

        dev->regs[SONIC_CTDA >> 1] = (uint16_t) (link & 0xFFFE);
        tda = (uint16_t) (link & 0xFFFE);
    }

    /* TXP is a command bit; it is cleared on EOL, HTX or abort. */
    dev->regs[SONIC_CR >> 1] &= (uint16_t) ~(SONIC_CR_TXP | SONIC_CR_HTX);
    dev->tx_active = 0;
}

uint16_t
dp8393x_reg_read(dp8393x_t *dev, uint16_t off)
{
    switch (off) {
        case SONIC_SR:
            /* Silicon version; manual/QEMU both use 0004h and the Boot ROM POST can see it. */
            return 0x0004;

        case SONIC_CAP2:
            return dev->cam[((dev->cep & 0x000F) * 3) + 2];

        case SONIC_CAP1:
            return dev->cam[((dev->cep & 0x000F) * 3) + 1];

        case SONIC_CAP0:
            return dev->cam[(dev->cep & 0x000F) * 3];

        case SONIC_CE:
            return dev->cam_enable;

        case SONIC_WT0:
            return (uint16_t) (dev->wt_count & 0xFFFF);

        case SONIC_WT1:
            return (uint16_t) (dev->wt_count >> 16);

        default:
            break;
    }

    if (off >= 0x80)
        return 0xFFFF;

    return dev->regs[off >> 1];
}

void
dp8393x_reg_write(dp8393x_t *dev, uint16_t off, uint16_t val)
{
    switch (off) {
        case SONIC_CR: {
            uint8_t txp;
            uint8_t htx;
            uint8_t release = 0;

            /* RST is a level command: writing 1 enters software reset, writing 0 releases it. */
            if (val & SONIC_CR_RST) {
                dp8393x_sw_reset(dev);
                return;
            }

            /* The write that releases software reset only latches status bits and
             * executes no command bits. */
            if (dev->regs[SONIC_CR >> 1] & SONIC_CR_RST) {
                release                     = 1;
                dev->regs[SONIC_CR >> 1]    = 0x0000;
            }

            txp = (val & SONIC_CR_TXP) ? 1 : 0;
            htx = (val & SONIC_CR_HTX) ? 1 : 0;

            /* RRRA completes synchronously, so the command bit self-clears
             * immediately and the driver polls CR bit 8 as 0. The recovery path
             * after clearing RBE reuses this as well. */
            if (!release && (val & SONIC_CR_RRRA)) {
                dev->rx_rbe = 0;
                dp8393x_read_rra(dev);
            }

            if (!release && (val & SONIC_CR_LCAM))
                dp8393x_cam_load(dev);

            if (!release && (val & SONIC_CR_RXEN))
                dev->rx_enabled = 1;
            if (!release && (val & SONIC_CR_RXDIS))
                dev->rx_enabled = 0;

            /* ST/STP are mutually exclusive; asserting both stops the timer (manual 2.7.1). */
            if ((val & (SONIC_CR_ST | SONIC_CR_STP)) == (SONIC_CR_ST | SONIC_CR_STP))
                dev->stopped = 1;
            else if (val & SONIC_CR_ST)
                dev->stopped = 0;
            else if (val & SONIC_CR_STP)
                dev->stopped = 1;

            /* ST/STP directly control the generic watchdog; reset stops the timer
             * without clearing it. */
            if (dev->stopped)
                dp8393x_wt_stop(dev);
            else
                dp8393x_wt_start(dev);

            /* TXP/RRRA/LCAM are command bits and are not retained; STP/RXEN/RXDIS/HTX
             * are. HTX must stay visible during transmission and is cleared by
             * dp8393x_tx() afterwards. */
            dev->regs[SONIC_CR >> 1] = (uint16_t) ((dev->stopped ? SONIC_CR_STP : SONIC_CR_ST)
                                                 | (dev->rx_enabled ? SONIC_CR_RXEN : SONIC_CR_RXDIS)
                                                 | (htx ? SONIC_CR_HTX : 0x0000));

            if (!release && txp)
                dp8393x_tx(dev);
            break;
        }

        case SONIC_DCR:
            /* Bits 9-8 and 6 are fixed at 1; bit 7 and bits 15-10 are fixed at 0. */
            dev->regs[SONIC_DCR >> 1] = (uint16_t) ((val & 0x003F) | 0x0360);
            break;

        case SONIC_RCR:
            dev->regs[SONIC_RCR >> 1] = val;
            break;

        case SONIC_TCR:
            /* The upper 4 configuration bits are writable; the low 12 bits are
             * hardware-maintained status. */
            dev->regs[SONIC_TCR >> 1] = (uint16_t) ((dev->regs[SONIC_TCR >> 1] & 0x0FFF)
                                                  | (val & 0xF000));
            break;

        case SONIC_IMR:
            /* POST writes 0xBCFF and reads it back unchanged, so no bit masking. */
            dev->regs[SONIC_IMR >> 1] = val;
            dp8393x_update_irq(dev);
            break;

        case SONIC_ISR: {
            uint8_t rbe = (val & SONIC_ISR_RBE) ? 1 : 0;
            uint8_t rde = (val & SONIC_ISR_RDE) ? 1 : 0;

            dev->regs[SONIC_ISR >> 1] &= (uint16_t) ~(val & 0x7FFF);

            if (rbe)
                dev->rx_rbe = 0;
            if (rde)
                dev->rx_rde = 0;

            /* Manual: after RBE is cleared, if software has supplied more RRAs the
             * adapter re-reads the resource descriptor at RRP, equivalent to an
             * RRRA. */
            if (rbe && !dev->rx_rbe && dev->rx_rba_full)
                dp8393x_read_rra(dev);

            dp8393x_update_irq(dev);
            break;
        }

        case SONIC_CDC:
            dev->regs[SONIC_CDC >> 1] = val & 0x001F;
            break;

        case SONIC_CEP:
            dev->cep = val & 0x000F;
            dev->regs[SONIC_CEP >> 1] = dev->cep;
            break;

        case SONIC_CE:
            dev->cam_enable = val;
            break;

        case SONIC_SR:
        case SONIC_RSC:
            break;

        case SONIC_CRCT:
        case SONIC_FAET:
        case SONIC_MPT:
            /* Manual 2.7.11: the written value is latched inverted, so writing FFFFh clears it. */
            dev->regs[off >> 1] = (uint16_t) ~val;
            break;

        case SONIC_WT0:
            dev->wt_count = (dev->wt_count & 0xFFFF0000u) | val;
            break;

        case SONIC_WT1:
            dev->wt_count = (dev->wt_count & 0x0000FFFFu) | ((uint32_t) val << 16);
            break;

        case SONIC_DCR2:
            /* Not modelled per measurements: the driver never accesses DCR2. Only
             * bit 0 (RJCM) is kept for receive filtering; the other bits have no
             * effect. */
            dev->regs[SONIC_DCR2 >> 1] = val;
            break;

        default:
            if (off < 0x80)
                dev->regs[off >> 1] = val;
            break;
    }
}

/* =========================================================================
 * Section 3: Chip external interface and device registration
 * ========================================================================= */

void
dp8393x_set_ram_size(dp8393x_t *dev, uint32_t size)
{
    dev->ram_size = size ? size : SONIC_RAM_SIZE;
}

void
dp8393x_init(dp8393x_t *dev)
{
    timer_add(&dev->wt_timer, dp8393x_wt_tick, dev, 0);
    dp8393x_reset(dev);
}

static void *
dp8393x_device_init(const device_t *info)
{
    dp8393x_t *dev = (dp8393x_t *) calloc(1, sizeof(dp8393x_t));

    (void) info;
    dp8393x_init(dev);

    return dev;
}

static void
dp8393x_device_close(void *priv)
{
    dp8393x_t *dev = (dp8393x_t *) priv;

    if (dev == NULL)
        return;

    timer_disable(&dev->wt_timer);
    free(dev);
}

static void
dp8393x_device_reset(void *priv)
{
    dp8393x_reset((dp8393x_t *) priv);
}

int dp8393x_inst = 0;

const device_t dp8393x_device = {
    .name          = "DP8393X SONIC Ethernet Controller",
    .internal_name = "dp8393x",
    .flags         = 0,
    .local         = 0,
    .init          = dp8393x_device_init,
    .close         = dp8393x_device_close,
    .reset         = dp8393x_device_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
