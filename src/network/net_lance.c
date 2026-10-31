/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          Emulation of the AMD Am7990 LANCE Ethernet controller in the
 *          16-bit byte-swapped bus mode used by the DEC VAXmate.
 *
 *          Register map (VAXmate TRM Vol.1, Table 13-1):
 *            0x0C60 (alias 0x0C64)  16-bit  RDP - register data port
 *            0x0C62 (alias 0x0C66)  16-bit  RAP - register address port
 *            0x0C68 (0x0C69-0x0C6F)  8-bit  NICSR - DEC interface logic
 *
 *          CSR3 must be programmed with BSWP (bit 2) set, see TRM Table 13-2:
 *          the LANCE sits on the byte-swapped 16-bit I/O bus of the VAXmate,
 *          so every word it hands to / takes from system memory has its byte
 *          lanes exchanged.
 *
 *          Descriptor and initialization block layouts follow the Am7990
 *          datasheet; the ring walking code is modelled on MAME's am79c90.
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/thread.h>
#include <86box/mem.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/pic.h>
#include <86box/timer.h>
#include <86box/dma.h>
#include <86box/plat.h>
#include <86box/network.h>
#include <86box/net_lance.h>

/* Defaults used when no board parameters are supplied. */
#define LANCE_BASE          0x0c60
#define LANCE_IRQ           10

#define LANCE_BUF_MAX       1600

/* CSR0 bits */
#define CSR0_INIT          0x0001
#define CSR0_STRT          0x0002
#define CSR0_STOP          0x0004
#define CSR0_TDMD          0x0008
#define CSR0_TXON          0x0010
#define CSR0_RXON          0x0020
#define CSR0_INEA          0x0040
#define CSR0_INTR          0x0080
#define CSR0_IDON          0x0100
#define CSR0_TINT          0x0200
#define CSR0_RINT          0x0400
#define CSR0_MERR          0x0800
#define CSR0_MISS          0x1000
#define CSR0_CERR          0x2000
#define CSR0_BABL          0x4000
#define CSR0_ERR           0x8000

/* CSR3 bits (VAXmate: BSWP = 1) */
#define CSR3_BSWP          0x0004
#define CSR3_ACON          0x0002
#define CSR3_BCON          0x0001

/* NICSR bits (TRM page 13-17, write only, set/clear style) */
#define NICSR_INT_SET      0x01
#define NICSR_INT_CLEAR    0x04
#define NICSR_LED_CLEAR    0x08
#define NICSR_LED_SET      0x10

/* MODE word of the initialization block */
#define MODE_DRX           0x0001
#define MODE_DTX           0x0002
#define MODE_LOOP          0x0004
#define MODE_DTCR          0x0008
#define MODE_PROM          0x8000

/* Transmit message descriptor word 1 */
#define TMD1_ENP           0x0100
#define TMD1_STP           0x0200
#define TMD1_ERR           0x4000
#define TMD1_OWN           0x8000

/* Transmit message descriptor word 3 */
#define TMD3_BUFF          0x8000
#define TMD3_UFLO          0x4000

/* Receive message descriptor word 1 */
#define RMD1_ENP           0x0100
#define RMD1_STP           0x0200
#define RMD1_BUFF          0x0400
#define RMD1_ERR           0x4000
#define RMD1_OWN           0x8000

/* Receive message descriptor word 3 */
#define RMD3_MCNT          0x0fff

#define INIT_ADDR_MASK     0xfffffe
#define RING_ADDR_MASK     0xfffff8

typedef struct lance_t {
    uint16_t   csr[4];
    uint16_t   base;
    int        irq_num;       /* 该板卡使用的中断线 */
    uint8_t    rap;
    int        irq_gate;      /* NICSR interrupt enable (power-on: off) */
    int        irq;
    uint8_t    mac[6];
    uint8_t    padr[6];
    uint8_t    ladrf[8];
    uint16_t   mode;
    uint32_t   init_addr;
    uint32_t   rx_ring_base;
    uint32_t   tx_ring_base;
    uint8_t    rx_ring_mask;
    uint8_t    tx_ring_mask;
    uint8_t    rx_ring_pos;
    uint8_t    tx_ring_pos;
    uint16_t   tx_md[4];
    uint16_t   rx_md[4];
    pc_timer_t tx_timer;
    pc_timer_t inj_timer;     /* LANCE_INJ 调试注入 */
    int        injected;
    netcard_t *card;
} lance_t;

static int lance_log = -1;

static void
lance_dbg(const char *fmt, ...)
{
    va_list ap;

    if (lance_log <= 0)
        return;

    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fflush(stderr);
}

static void
lance_update_irq(lance_t *dev)
{
    int active = 0;

    if (dev->irq_gate && (dev->csr[0] & CSR0_INEA)) {
        /* INTR is raised for any of the error/completion flags. */
        if (dev->csr[0] & (CSR0_TINT | CSR0_RINT | CSR0_MERR | CSR0_MISS |
                           CSR0_CERR | CSR0_BABL | CSR0_ERR))
            active = 1;
    }

    if (active && !dev->irq) {
        dev->irq = 1;
        lance_dbg("[LANCE] IRQ10 assert (csr0=%04X gate=%d)\n", dev->csr[0], dev->irq_gate);
        picint(1 << dev->irq_num);
    } else if (!active && dev->irq) {
        dev->irq = 0;
        lance_dbg("[LANCE] IRQ10 clear\n");
        picintc(1 << dev->irq_num);
    }
}

/* The byte lanes of the 16-bit bus are swapped on the VAXmate (CSR3 BSWP). */
static void
lance_bswap(uint8_t *buf, int len)
{
    int i;

    for (i = 0; i + 1 < len; i += 2) {
        uint8_t t = buf[i];
        buf[i]     = buf[i + 1];
        buf[i + 1] = t;
    }
}

static void
lance_dma_read(lance_t *dev, uint32_t addr, uint8_t *buf, int len)
{
    dma_bm_read(addr, buf, len, 2);

    if (dev->csr[3] & CSR3_BSWP)
        lance_bswap(buf, len);
}

static void
lance_dma_write(lance_t *dev, uint32_t addr, const uint8_t *buf, int len)
{
    uint8_t tmp[LANCE_BUF_MAX];

    if (len > (int) sizeof(tmp))
        len = sizeof(tmp);

    memcpy(tmp, buf, len);

    if (dev->csr[3] & CSR3_BSWP)
        lance_bswap(tmp, len);

    dma_bm_write(addr, tmp, len, 2);
}

static uint16_t
lance_md_read(lance_t *dev, uint32_t addr)
{
    uint8_t b[2];

    lance_dma_read(dev, addr, b, 2);

    return (uint16_t) (b[0] | (b[1] << 8));
}

static void
lance_md_write(lance_t *dev, uint32_t addr, uint16_t val)
{
    uint8_t b[2];

    b[0] = (uint8_t) (val & 0xff);
    b[1] = (uint8_t) (val >> 8);

    lance_dma_write(dev, addr, b, 2);
}

static uint32_t
lance_md_addr(uint16_t md0, uint16_t md1)
{
    return (uint32_t) (((md1 & 0x00ff) << 16) | md0);
}

/* Address filter: physical address, broadcast, or multicast through LADRF. */
static int
lance_address_filter(lance_t *dev, const uint8_t *buf, int len)
{
    static const uint8_t bc[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    uint64_t hash;
    uint8_t  bit;

    if (len < 6)
        return 0;

    if (dev->mode & MODE_PROM)
        return 1;

    if (!memcmp(buf, dev->padr, 6))
        return 1;

    if (!memcmp(buf, bc, 6))
        return 1;

    if (buf[0] & 0x01) {
        hash = (uint64_t) dev->ladrf[0] | ((uint64_t) dev->ladrf[1] << 8) |
               ((uint64_t) dev->ladrf[2] << 16) | ((uint64_t) dev->ladrf[3] << 24) |
               ((uint64_t) dev->ladrf[4] << 32) | ((uint64_t) dev->ladrf[5] << 40) |
               ((uint64_t) dev->ladrf[6] << 48) | ((uint64_t) dev->ladrf[7] << 56);
        bit = (uint8_t) ((buf[2] ^ buf[4] ^ buf[5]) & 0x3f);
        if (hash & (1ULL << bit))
            return 1;
    }

    return 0;
}

/* Initialization block: 12 words, see TRM chapter 13. */
static void
lance_init_block(lance_t *dev)
{
    uint8_t ib[24];
    int     i;

    dev->init_addr = ((uint32_t) dev->csr[1] | ((uint32_t) dev->csr[2] << 16)) & INIT_ADDR_MASK;

    memset(ib, 0x00, sizeof(ib));
    lance_dma_read(dev, dev->init_addr, ib, sizeof(ib));

    dev->mode = (uint16_t) (ib[0] | (ib[1] << 8));

    for (i = 0; i < 6; i++)
        dev->padr[i] = ib[2 + i];
    for (i = 0; i < 8; i++)
        dev->ladrf[i] = ib[8 + i];

    dev->rx_ring_base = ((uint32_t) ib[16] | ((uint32_t) ib[17] << 8) |
                         ((uint32_t) ib[18] << 16)) & RING_ADDR_MASK;
    dev->tx_ring_base = ((uint32_t) ib[20] | ((uint32_t) ib[21] << 8) |
                         ((uint32_t) ib[22] << 16)) & RING_ADDR_MASK;
    dev->rx_ring_mask = (uint8_t) ~(1 << ((ib[19] >> 5) & 7));
    dev->tx_ring_mask = (uint8_t) ~(1 << ((ib[23] >> 5) & 7));

    dev->rx_ring_pos = 0;
    dev->tx_ring_pos = 0;

    memcpy(dev->mac, dev->padr, 6);

    lance_dbg("[LANCE] init: mode=%04X mac=%02X:%02X:%02X:%02X:%02X:%02X "
              "rx=%05X/%d tx=%05X/%d\n",
              dev->mode, dev->padr[0], dev->padr[1], dev->padr[2],
              dev->padr[3], dev->padr[4], dev->padr[5],
              dev->rx_ring_base, 1 << ((ib[19] >> 5) & 7),
              dev->tx_ring_base, 1 << ((ib[23] >> 5) & 7));

    if (!dev->injected && (getenv("LANCE_INJ") != NULL)) {
        dev->injected = 1;
        timer_set_delay_u64(&dev->inj_timer, 50000 * TIMER_USEC);
    }

    /* INIT is cleared by the chip, IDON reports completion. */
    dev->csr[0] &= (uint16_t) ~CSR0_INIT;
    dev->csr[0] |= CSR0_IDON;
}

static void lance_tx_poll(void *priv);

static int  lance_receive(lance_t *dev, uint8_t *buf, int length);
static void lance_rx_complete(lance_t *dev, int result);

/* 调试用：INIT 之后注入一帧，验证接收环 + RINT 通路。 */
static void
lance_inject(void *priv)
{
    lance_t     *dev = (lance_t *) priv;
    static const uint8_t frame[64] = {
        0x08, 0x00, 0x2b, 0x12, 0x34, 0x56,   /* 目的：本站 MAC（INIT 之后再填）*/
        0x08, 0x00, 0x2b, 0xaa, 0xbb, 0xcc,   /* 源 */
        0x08, 0x00,                            /* EtherType IP */
        0x45, 0x00, 0x00, 0x2e
    };
    uint8_t buf[64];

    if (dev->card == NULL)
        return;

    memcpy(buf, frame, sizeof(buf));
    memcpy(buf, dev->padr, 6);

    lance_dbg("[LANCE] inject %d bytes\n", (int) sizeof(buf));
    lance_rx_complete(dev, lance_receive(dev, buf, sizeof(buf)));
}

static void
lance_tx_complete(lance_t *dev, int result)
{
    uint32_t ring_address = (dev->tx_ring_base + (dev->tx_ring_pos << 3)) & RING_ADDR_MASK;

    if (result < 0)
        dev->tx_md[1] |= TMD1_ERR;

    /* Hand the descriptor back to the driver and move on. */
    lance_md_write(dev, ring_address | 6, dev->tx_md[3]);
    lance_md_write(dev, ring_address | 2, dev->tx_md[1]);

    dev->tx_ring_pos = (uint8_t) ((dev->tx_ring_pos + 1) & dev->tx_ring_mask);

    dev->csr[0] |= CSR0_TINT;

    /* TX completion must reach the host: the driver waits on IRQ10. */
    lance_update_irq(dev);

    /* Back-to-back transmit: poll again shortly. */
    timer_set_delay_u64(&dev->tx_timer, 20 * TIMER_USEC);
}

static void
lance_transmit(lance_t *dev)
{
    uint32_t ring_address = (dev->tx_ring_base + (dev->tx_ring_pos << 3)) & RING_ADDR_MASK;
    uint8_t  buf[LANCE_BUF_MAX];
    int      length = 0;

    memset(buf, 0x00, sizeof(buf));

    for (;;) {
        int tx_buf_length;
        int remaining;

        dev->tx_md[0] = lance_md_read(dev, ring_address | 0);
        dev->tx_md[2] = lance_md_read(dev, ring_address | 4);
        dev->tx_md[3] = 0x0000;

        tx_buf_length = dev->tx_md[2] & 0x0fff;
        remaining     = LANCE_BUF_MAX - length;
        if (tx_buf_length > remaining)
            tx_buf_length = remaining;

        if (tx_buf_length > 0)
            lance_dma_read(dev, lance_md_addr(dev->tx_md[0], dev->tx_md[1]),
                           &buf[length], tx_buf_length);
        length += tx_buf_length;

        dev->tx_md[1] &= (uint16_t) ~TMD1_OWN;

        if (!(dev->tx_md[1] & TMD1_ENP)) {
            uint8_t  next_pos  = (uint8_t) ((dev->tx_ring_pos + 1) & dev->tx_ring_mask);
            uint32_t next_addr = (dev->tx_ring_base + (next_pos << 3)) & RING_ADDR_MASK;
            uint16_t next_md1  = (next_addr != ring_address) ?
                                 lance_md_read(dev, next_addr | 2) : dev->tx_md[1];

            if (next_md1 & TMD1_OWN) {
                lance_md_write(dev, ring_address | 2, dev->tx_md[1]);
                dev->tx_ring_pos = next_pos;
                ring_address     = next_addr;
                dev->tx_md[1]    = next_md1;
                continue;
            }

            /* The chain ran out of descriptors. */
            dev->tx_md[1] |= TMD1_ERR;
            dev->tx_md[3] |= TMD3_BUFF | TMD3_UFLO;
            dev->csr[0] &= (uint16_t) ~CSR0_TXON;
            break;
        }

        break;
    }

    if (length > 1518)
        dev->csr[0] |= CSR0_BABL | CSR0_ERR;

    if (length >= 14) {
        if (!(dev->mode & MODE_LOOP) && (dev->card != NULL)) {
            network_tx(dev->card, buf, length);
            lance_dbg("[LANCE] tx %d bytes\n", length);
        }
        lance_tx_complete(dev, length);
    } else {
        lance_tx_complete(dev, -1);
    }
}

static void
lance_tx_poll(void *priv)
{
    lance_t *dev = (lance_t *) priv;
    uint32_t ring_address;

    if (!(dev->csr[0] & CSR0_TXON))
        return;

    dev->csr[0] &= (uint16_t) ~CSR0_TDMD;

    ring_address  = (dev->tx_ring_base + (dev->tx_ring_pos << 3)) & RING_ADDR_MASK;
    dev->tx_md[1] = lance_md_read(dev, ring_address | 2);

    if (!(dev->tx_md[1] & TMD1_OWN))
        return;

    if (!(dev->tx_md[1] & TMD1_STP)) {
        /* Descriptor does not start a packet: release it. */
        lance_md_write(dev, ring_address | 2, dev->tx_md[1] & (uint16_t) ~TMD1_OWN);
        dev->tx_ring_pos = (uint8_t) ((dev->tx_ring_pos + 1) & dev->tx_ring_mask);
        timer_set_delay_u64(&dev->tx_timer, 20 * TIMER_USEC);
        return;
    }

    lance_transmit(dev);
}

static int
lance_receive(lance_t *dev, uint8_t *buf, int length)
{
    uint32_t ring_address;
    int      offset = 0;

    if (!(dev->csr[0] & CSR0_RXON) || (dev->mode & MODE_DRX))
        return -1;

    if (!lance_address_filter(dev, buf, length))
        return -1;

    ring_address  = (dev->rx_ring_base + (dev->rx_ring_pos << 3)) & RING_ADDR_MASK;
    dev->rx_md[1] = lance_md_read(dev, ring_address | 2);

    if (!(dev->rx_md[1] & RMD1_OWN))
        return -2;

    dev->rx_md[1] |= RMD1_STP;

    while (offset < length) {
        int rx_buf_length;
        int count;

        dev->rx_md[0] = lance_md_read(dev, ring_address | 0);
        dev->rx_md[2] = lance_md_read(dev, ring_address | 4);

        rx_buf_length = dev->rx_md[2] & 0x0fff;
        count         = length - offset;
        if (count > rx_buf_length)
            count = rx_buf_length;
        if (count < 0)
            count = 0;

        if (count > 0)
            lance_dma_write(dev, lance_md_addr(dev->rx_md[0], dev->rx_md[1]),
                            &buf[offset], count);
        offset += count;

        dev->rx_md[1] &= (uint16_t) ~RMD1_OWN;

        if (offset < length) {
            uint8_t  next_pos  = (uint8_t) ((dev->rx_ring_pos + 1) & dev->rx_ring_mask);
            uint32_t next_addr = (dev->rx_ring_base + (next_pos << 3)) & RING_ADDR_MASK;
            uint16_t next_md1  = (next_addr != ring_address) ?
                                 lance_md_read(dev, next_addr | 2) : dev->rx_md[1];

            if (next_md1 & RMD1_OWN) {
                lance_md_write(dev, ring_address | 2, dev->rx_md[1]);
                dev->rx_ring_pos = next_pos;
                ring_address     = next_addr;
                dev->rx_md[1]    = next_md1;
            } else {
                dev->rx_md[1] |= RMD1_ERR | RMD1_BUFF;
                break;
            }
        }
    }

    if (offset == length)
        dev->rx_md[1] |= RMD1_ENP;

    return offset;
}

static void
lance_rx_complete(lance_t *dev, int result)
{
    uint32_t ring_address;

    if (result == -1)
        return;

    ring_address = (dev->rx_ring_base + (dev->rx_ring_pos << 3)) & RING_ADDR_MASK;

    if (result == -2) {
        dev->csr[0] |= CSR0_ERR | CSR0_MISS;
    } else {
        lance_md_write(dev, ring_address | 2, dev->rx_md[1]);
        lance_md_write(dev, ring_address | 6,
                       (dev->rx_md[1] & RMD1_ERR) ? 0x0000 : (result & RMD3_MCNT));
        dev->rx_ring_pos = (uint8_t) ((dev->rx_ring_pos + 1) & dev->rx_ring_mask);
    }

    dev->csr[0] |= CSR0_RINT;
    lance_update_irq(dev);
}

static int
lance_rx(void *card_drv, uint8_t *buf, int length)
{
    lance_t *dev = (lance_t *) card_drv;

    lance_dbg("[LANCE] rx %d bytes\n", length);

    lance_rx_complete(dev, lance_receive(dev, buf, length));

    return 0;
}

static uint16_t
lance_csr_read(lance_t *dev, int reg)
{
    uint16_t ret;

    switch (reg) {
        case 0:
            ret = dev->csr[0];
            if ((ret & CSR0_INEA) && (ret & (CSR0_TINT | CSR0_RINT |
                                            CSR0_MERR | CSR0_MISS |
                                            CSR0_CERR | CSR0_BABL | CSR0_ERR)))
                ret |= CSR0_INTR;
            /* reading CSR0 clears INTR */
            dev->csr[0] &= (uint16_t) ~CSR0_INTR;
            break;

        case 3:
            /* CSR3 is only accessible while STOP is set. */
            if (!(dev->csr[0] & CSR0_STOP))
                ret = 0x0000;
            else
                ret = dev->csr[3];
            break;

        default:
            ret = dev->csr[reg & 3];
            break;
    }

    return ret;
}

static void
lance_csr_write(lance_t *dev, int reg, uint16_t val)
{
    switch (reg) {
        case 0:
            /* INIT and STRT are write-one-to-act bits. */
            if (val & CSR0_STOP) {
                /* STOP clears the chip state, CSR1/2/3 included. */
                dev->csr[0] = CSR0_STOP;
                dev->csr[1] = 0x0000;
                dev->csr[2] = 0x0000;
                dev->csr[3] = 0x0000;
                timer_disable(&dev->tx_timer);
                break;
            }

            /* TRM 1.9.2: command bits are write-one only ("the LANCE updates
               CSR0 by ORing the previous and present value"), so writing a
               zero must not clear INEA or TDMD.  STOP is the only bit that
               clears the register. */
            dev->csr[0] |= (uint16_t) (val & (CSR0_INEA | CSR0_TDMD));

            lance_dbg("[LANCE] CSR0 write %04X (IADR=%05X)\n", val, dev->init_addr);

            /* 状态位写 1 清零（TRM：IDON/TINT/RINT/MERR/MISS/CERR/BABL/ERR） */
            dev->csr[0] &= (uint16_t) ~(val & (CSR0_IDON | CSR0_TINT | CSR0_RINT |
                                               CSR0_MERR | CSR0_MISS | CSR0_CERR |
                                               CSR0_BABL | CSR0_ERR));

            if (val & CSR0_STRT) {
                /* STRT 必须读回 1（ROM 在 F6BA9 轮询 bit1 判定启动完成） */
                dev->csr[0] |= CSR0_STRT;
                dev->csr[0] &= (uint16_t) ~(CSR0_STOP | CSR0_INIT);
                if (dev->mode & MODE_DRX)
                    dev->csr[0] &= (uint16_t) ~CSR0_RXON;
                else
                    dev->csr[0] |= CSR0_RXON;
                if (dev->mode & MODE_DTX)
                    dev->csr[0] &= (uint16_t) ~CSR0_TXON;
                else
                    dev->csr[0] |= CSR0_TXON;
                timer_set_delay_u64(&dev->tx_timer, 20 * TIMER_USEC);
            }
            if (val & CSR0_INIT) {
                /* Setting INIT (or STRT) also clears STOP, TRM 1.9.2. */
                dev->csr[0] &= (uint16_t) ~CSR0_STOP;
                dev->csr[0] |= CSR0_INIT;
                dev->init_addr = ((uint32_t) dev->csr[1] |
                                  ((uint32_t) dev->csr[2] << 16)) & INIT_ADDR_MASK;
                lance_init_block(dev);
            }
            if ((val & CSR0_TDMD) && (dev->csr[0] & CSR0_TXON))
                timer_set_delay_u64(&dev->tx_timer, 20 * TIMER_USEC);
            break;

        case 1:
        case 2:
            dev->csr[reg] = val;
            break;

        case 3:
            if (dev->csr[0] & CSR0_STOP) {
                dev->csr[3] = val;
                lance_dbg("[LANCE] CSR3 = %04X\n", val);
            } else
                lance_dbg("[LANCE] CSR3 write ignored (STOP=0)\n");
            break;
    }

    lance_update_irq(dev);
}

static uint16_t
lance_rdp_read(uint16_t addr, void *priv)
{
    lance_t *dev = (lance_t *) priv;

    (void) addr;

    lance_dbg("[LANCE] RDP read  csr%d\n", dev->rap & 3);
    return lance_csr_read(dev, dev->rap & 3);
}

static void
lance_rdp_write(uint16_t addr, uint16_t val, void *priv)
{
    lance_t *dev = (lance_t *) priv;

    (void) addr;

    lance_dbg("[LANCE] RDP write csr%d = %04X\n", dev->rap & 3, val);
    lance_csr_write(dev, dev->rap & 3, val);
}

static uint16_t
lance_rap_read(uint16_t addr, void *priv)
{
    lance_t *dev = (lance_t *) priv;

    (void) addr;

    lance_dbg("[LANCE] RAP read -> %d\n", dev->rap);
    return dev->rap;
}

static void
lance_rap_write(uint16_t addr, uint16_t val, void *priv)
{
    lance_t *dev = (lance_t *) priv;

    (void) addr;

    dev->rap = (uint8_t) (val & 0x03);
    lance_dbg("[LANCE] RAP write %04X -> %d\n", val, dev->rap);
}

static void
lance_reset(void *priv)
{
    lance_t *dev = (lance_t *) priv;

    if (dev == NULL)
        return;

    memset(dev->csr, 0x00, sizeof(dev->csr));
    dev->csr[0] = CSR0_STOP;
    dev->rap    = 0;
    dev->irq    = 0;
    dev->mode   = 0x0000;
    memset(dev->padr, 0x00, sizeof(dev->padr));
    memset(dev->ladrf, 0x00, sizeof(dev->ladrf));
    dev->rx_ring_pos = 0;
    dev->tx_ring_pos = 0;
    dev->rx_ring_base = 0x000000;
    dev->tx_ring_base = 0x000000;
    dev->rx_ring_mask = 0xff;
    dev->tx_ring_mask = 0xff;
    /* NICSR interrupt enable and diagnostic LED are both off after reset. */
    dev->irq_gate = 0;

    timer_disable(&dev->tx_timer);

    picintc(1 << dev->irq_num);
}


static void *
lance_init(const device_t *info)
{
    lance_t            *dev;
    const lance_board_t *board;

    dev = (lance_t *) calloc(1, sizeof(lance_t));

    (void) info;

    if (dev == NULL)
        return NULL;

    if (lance_log < 0)
        lance_log = (getenv("LANCE_LOG") != NULL) ? 1 : 0;

    /* Board parameters: the machine decides where the chip lives. */
    board = (const lance_board_t *) device_get_priv(info);
    if (board != NULL) {
        dev->base = board->base;
        dev->irq_num = board->irq;
        memcpy(dev->mac, board->mac, 6);
    } else {
        dev->base                          = LANCE_BASE;
        dev->irq_num                       = LANCE_IRQ;
        dev->mac[0] = 0x08;
        dev->mac[1] = 0x00;
        dev->mac[2] = 0x2b;
        dev->mac[3] = 0x12;
        dev->mac[4] = 0x34;
        dev->mac[5] = 0x56;
    }

    timer_add(&dev->tx_timer, lance_tx_poll, dev, 0);
    timer_add(&dev->inj_timer, lance_inject, dev, 0);
    lance_reset(dev);

    /* RDP is a single 16-bit port; it must not be registered over 0x0C62,
       or the RAP handler at that address is shadowed by the wider range. */
    io_sethandler(dev->base, 0x0002,
                  NULL, lance_rdp_read, NULL,
                  NULL, lance_rdp_write, NULL, dev);
    io_sethandler(dev->base + 0x0004, 0x0002,
                  NULL, lance_rdp_read, NULL,
                  NULL, lance_rdp_write, NULL, dev);
    io_sethandler(dev->base + 0x0002, 0x0002,
                  NULL, lance_rap_read, NULL,
                  NULL, lance_rap_write, NULL, dev);
    io_sethandler(dev->base + 0x0006, 0x0002,
                  NULL, lance_rap_read, NULL,
                  NULL, lance_rap_write, NULL, dev);

    dev->card = network_attach(dev, dev->mac, lance_rx, NULL);

    return dev;
}

static void
lance_close(void *priv)
{
    lance_t *dev = (lance_t *) priv;

    if (dev != NULL)
        free(dev);
}

/* DEC interface logic (board glue) gates the interrupt line. */
void
lance_set_irq_gate(void *priv, int on)
{
    lance_t *dev = (lance_t *) priv;

    if (dev == NULL)
        return;

    dev->irq_gate = on ? 1 : 0;
    lance_update_irq(dev);
}

int
lance_get_irq_gate(void *priv)
{
    lance_t *dev = (lance_t *) priv;

    return (dev != NULL) ? dev->irq_gate : 0;
}

const device_t lance_device = {
    .name          = "AMD Am7990 LANCE",
    .internal_name = "lance_am7990",
    .flags         = DEVICE_ISA16,
    .local         = 0,
    .init          = lance_init,
    .close         = lance_close,
    .reset         = lance_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
