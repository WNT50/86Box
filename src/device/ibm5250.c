/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          Emulation of the 5250 Work Station Adapter of the IBM 3477-S
 *          (5550 family).
 *
 *          Ports owned by this device:
 *            0x1C0  command (0x08 poll / 0x40 Clear Unit)
 *            0x1C1  status latch
 *            0x1C2  strobe (bit2 = 0x04)
 *            0x1C3, 0x1C7, 0x1C8-0x1CF, 0x1D0, 0x1D1, 0x1D6
 *            0x1C6  bit-serial data + handshake phase (bit1) + response (bit7)
 *
 *          0x1B0 / 0x1B2 / 0x1B6 stay in the machine: they are the terminal
 *          type latch, the memory-window selector and the ICU release port,
 *          none of which belong to the adapter.
 *
 *          The bit-serial protocol follows the ROM/EMDIAG bit-banger:
 *            - guest writes bit6 = data, bit4 = level
 *            - 0x1C0 command 0x08 (poll) or 0x40 (Clear Unit) arms a reply
 *            - the guest reads 0x1C6, 19 handshake reads then 32 frame reads
 *              (2 reads per bit, 16 bits: start + 8 data + 3 addr + parity + 3 fill)
 *            - bit1 mirrors the 0x1C2 phase (always 0 for command 0x40)
 *
 *          The payload for 0x40 is d=0x55 with address 111 and odd parity
 *          over the 12 bits B15..B4; for 0x08 it is d=0xFF with even parity.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include "cpu.h"
#include <86box/ibm5250.h>

/* 19-read handshake, measured on the real EMDIAG/0x840 sequence.  idx 18 must
   stay 0: giving it the start bit breaks the 0x500 subtest (AH=0x13). */
static const uint8_t ibm5250_handshake[19] = {
    0x00, 0x00, 0x00, 0x80, 0x00,
    0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
    0x80, 0x80, 0x80,
    0x00, 0x00, 0x00
};

struct ibm5250_t {
    uint8_t  latch;      /* 0x1C1 status latch, mirrored into 0x1C6 */
    uint8_t  cmdlat;     /* command latched at 0x1C0 */
    int      phase;      /* 0x1C6 bit1 for every command except 0x40 */

    int      ph;         /* 1 = handshake, 2 = frame */
    int      rd;         /* read index inside the current phase */
    int      serve;      /* this poll gets a reply */
    int      done40;     /* the 0x40 frame has been served once */
    int      fno;        /* frame counter (log only) */
    int      dummy;      /* optional leading idle read */

    uint32_t hist;       /* pulse history for the 0x2AA frame preamble */
    int      inframe;
    int      pcnt;
    uint16_t word;       /* decoded command word */

    int      irq;
    ibm5250_irq_t irq_raise;
    void    *irq_priv;
    pc_timer_t timer;
    int      timer_armed;
};

static void
ibm5250_irq_cb(void *priv)
{
    ibm5250_t *dev = (ibm5250_t *) priv;

    dev->timer_armed = 0;
    if (dev->irq_raise != NULL)
        dev->irq_raise(dev->irq_priv);
}

static void
ibm5250_arm_irq(ibm5250_t *dev)
{
    if ((dev->irq_raise == NULL) || dev->timer_armed)
        return;

    dev->timer_armed = 1;
    timer_set_delay_u64(&dev->timer, TIMER_USEC * 80);
}

static uint8_t
ibm5250_base(ibm5250_t *dev)
{
    /* bit1 is the 0x1C2 phase, but command 0x40 always reads it as 0 -
       that is what the 0x890 register handshake in EMDIAG checks. */
    return (uint8_t) ((dev->latch & 0x7d) |
                      ((dev->cmdlat == 0x40) ? 0x00 : (dev->phase ? 0x02 : 0x00)));
}

static uint8_t
ibm5250_frame_bit(ibm5250_t *dev, int k)
{
    int cmd40 = (dev->cmdlat == 0x40);
    int d     = cmd40 ? 0x55 : 0xFF;
    int ones  = 0;
    int o12;

    if (k == 0)
        return 1;                       /* start / sync */
    if (k <= 8)
        return (uint8_t) ((d >> (k - 1)) & 1);
    if (k <= 11)
        return 1;                       /* station address 111 */
    if (k == 12) {
        /* B15..B4 = start + 8 data + 3 address */
        ones = 1 + 3;
        for (int t = 0; t < 8; t++)
            ones += (d >> t) & 1;
        o12 = ones + 1;                 /* + the parity bit itself */
        return (uint8_t) (cmd40 ? ((o12 & 1) ? 0 : 1) : ((o12 & 1) ? 1 : 0));
    }
    return 0;                           /* fill */
}

static uint8_t
ibm5250_in_1c6(ibm5250_t *dev)
{
    uint8_t base = ibm5250_base(dev);
    uint8_t ret  = (uint8_t) (base & 0x3f);

    if (!dev->serve)
        return ret;

    if (dev->ph != 2) {
        ret = (uint8_t) ((base & 0x3f) | ibm5250_handshake[dev->rd]);
        if (++dev->rd >= 19) {
            dev->ph = 2;
            dev->rd = 0;
            dev->dummy = 0;
        }
        return ret;
    }

    if (dev->dummy) {
        /* optional extra idle read, kept behind EPOCH3477_ADR_RW0 so the
           0x500 timing stays untouched unless explicitly requested */
        dev->dummy = 0;
        return ret;
    }

    {
        int k  = dev->rd / 2;
        int fb = ibm5250_frame_bit(dev, k);

        if (getenv("EPOCH5250_TRACE")) {
            static int n;
            if (n < 200) {
                pclog("5250: TRACE f=%d k=%2d fb=%d rd=%2d at=%04X:%04X bx=%04X bl=%02X bh=%02X\n",
                      dev->fno, k, fb, dev->rd,
                      ((CS > 0xFFFF) ? (int) (CS >> 4) : (int) CS), cpu_state.pc,
                      BX, (unsigned) (BX & 0xff), (unsigned) ((BX >> 8) & 0xff));
                n++;
            }
        }

        ret = (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
        if (++dev->rd >= 32) {
            dev->rd = 0;
            if (dev->cmdlat == 0x40)
                dev->done40 = 1;
        }
    }

    return ret;
}

static uint8_t
ibm5250_in(uint16_t port, void *priv)
{
    ibm5250_t *dev = (ibm5250_t *) priv;

    if (getenv("EPOCH5250_LOGIO")) {
        static int n;
        if (n < 300) {
            pclog("5250: IN %03X -> (state latch=%02X cmd=%02X ph=%d rd=%d)\n",
                  port, dev->latch, dev->cmdlat, dev->ph, dev->rd);
            n++;
        }
    }

    switch (port) {
        case 0x1C1:
            /* bit1|bit6 are the adapter signature the ROM handshake checks */
            return (uint8_t) (dev->latch | 0x42);

        case 0x1C2:
        case 0x1C3:
            return 0xff;

        case 0x1C6:
            return ibm5250_in_1c6(dev);

        default:
            return 0xff;
    }
}

static void
ibm5250_out(uint16_t port, uint8_t val, void *priv)
{
    ibm5250_t *dev = (ibm5250_t *) priv;

    if (getenv("EPOCH5250_LOGIO")) {
        static int n;
        if (n < 300) {
            pclog("5250: OUT %03X = %02X\n", port, val);
            n++;
        }
    }

    switch (port) {
        case 0x1C0:
            if ((val == 0x40) || (val == 0x08)) {
                /* keep the 0x40 reply in place while the subtest is running */
                int hold = (val == 0x08) && (dev->cmdlat == 0x40) && !dev->done40;

                if (!hold) {
                    dev->cmdlat = val;
                    dev->ph     = 1;
                    dev->rd     = 0;
                    dev->serve  = 1;
                    dev->fno++;
                    dev->dummy  = getenv("EPOCH3477_ADR_RW0") ? 1 : 0;
                    if (val == 0x40)
                        dev->done40 = 0;
                    dev->word    = 0;
                    dev->inframe = 0;
                    dev->hist    = 0;
                    pclog("5250: ARM %02X (frame %d)\n", val, dev->fno);
                } else {
                    pclog("5250: ARM %02X ignored (0x40 pending)\n", val);
                }
            }
            ibm5250_arm_irq(dev);
            break;

        case 0x1C1:
            dev->latch = (uint8_t) (val | 0x42);
            if ((val & 0x42) == 0x42)
                dev->phase = 0;
            break;

        case 0x1C2:
            if (val & 0x04)
                dev->phase = ((dev->latch & 0x42) == 0x42) ? 0 : 1;
            break;

        case 0x1C6:
            if (val & 0x40) {
                int d = (val >> 4) & 1;

                dev->hist = ((dev->hist << 1) | (uint32_t) d) & 0x3FF;
                if (!dev->inframe) {
                    if (dev->hist == 0x2AA) {
                        dev->inframe = 1;
                        dev->pcnt    = 0;
                        dev->word    = 0;
                    }
                } else {
                    if ((dev->pcnt >= 6) && (((dev->pcnt - 6) & 1) == 0)) {
                        int n = (dev->pcnt - 6) >> 1;

                        if ((n < 16) && d)
                            dev->word |= (uint16_t) (1u << n);
                    }
                    dev->pcnt++;
                    if (dev->pcnt >= 38) {
                        dev->inframe = 0;
                        /* the 0x500 subtest polls eight stations and only
                           answers the one whose address word is 0x1221 */
                        if (getenv("EPOCH5250_NODS")) {
                            dev->serve = (dev->word == 0x1221);
                        } else {
                            uint32_t dsa = ((uint32_t) cpu_state.seg_ds.seg << 4) + 0x2F;
                            dev->serve = (dsa < 0x1000000)
                                       ? ((mem_readb_phys(dsa) == 1) ? (dev->word == 0x1221) : 1)
                                       : 1;
                        }
                        pclog("5250: FRAME %04X serve=%d\n", dev->word, dev->serve);
                    }
                }
            }
            ibm5250_arm_irq(dev);
            break;

        default:
            break;
    }
}

void
ibm5250_reset(ibm5250_t *dev)
{
    dev->latch    = 0x00;
    dev->cmdlat   = 0x00;
    dev->phase    = 1;
    dev->ph       = 1;
    dev->rd       = 0;
    dev->serve    = 1;
    dev->done40   = 0;
    dev->fno      = 0;
    dev->dummy    = 0;
    dev->hist     = 0;
    dev->inframe  = 0;
    dev->pcnt     = 0;
    dev->word     = 0;
}

/* Ports the adapter itself owns; 0x1B0/0x1B2/0x1B6 stay with the machine. */
static const uint16_t ibm5250_ports[] = {
    0x1C0, 0x1C1, 0x1C2, 0x1C3, 0x1C6, 0x1C7,
    0x1C8, 0x1C9, 0x1CA, 0x1CB, 0x1CC, 0x1CD, 0x1CE, 0x1CF,
    0x1D0, 0x1D1, 0x1D6
};

ibm5250_t *
ibm5250_init(int irq, ibm5250_irq_t irq_raise, void *irq_priv)
{
    ibm5250_t *dev = calloc(1, sizeof(ibm5250_t));

    if (dev == NULL)
        return NULL;

    dev->irq       = irq;
    dev->irq_raise = irq_raise;
    dev->irq_priv  = irq_priv;

    ibm5250_reset(dev);

    timer_add(&dev->timer, ibm5250_irq_cb, dev, 0);

    for (int i = 0; i < (int) (sizeof(ibm5250_ports) / sizeof(ibm5250_ports[0])); i++)
        io_sethandler(ibm5250_ports[i], 0x0001,
                      ibm5250_in, NULL, NULL, ibm5250_out, NULL, NULL, dev);

    pclog("5250: adapter initialised (irq=%d, %d ports)\n",
          irq, (int) (sizeof(ibm5250_ports) / sizeof(ibm5250_ports[0])));

    return dev;
}

void
ibm5250_close(ibm5250_t *dev)
{
    if (dev == NULL)
        return;

    for (int i = 0; i < (int) (sizeof(ibm5250_ports) / sizeof(ibm5250_ports[0])); i++)
        io_removehandler(ibm5250_ports[i], 0x0001,
                         ibm5250_in, NULL, NULL, ibm5250_out, NULL, NULL, dev);

    free(dev);
}
