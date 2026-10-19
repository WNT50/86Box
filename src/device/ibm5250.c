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
    int      pre;        /* 帧前空槽（每槽 2 读）*/
    uint8_t  data8;      /* 0x1C8 数据寄存器 */
    int      seq64;      /* 0x64 命令的帧序号 */
    int      fno;        /* frame counter (log only) */
    int      dummy;      /* optional leading idle read */

    uint32_t hist;       /* pulse history for the 0x2AA frame preamble */
    int      inframe;
    int      pcnt;
    uint16_t word;       /* decoded command word */

    int      irq;
    ibm5250_irq_t irq_raise;
    void    *irq_priv;
    ibm5250_irq_t irq_clear;
    void    *irq_clear_priv;
    int      irq_level;
    int      pair_pend;      /* 一次位槽的第 1 读已发生 */
    uint64_t pair_tsc;
    pc_timer_t timer;
    int      timer_armed;
    int      irq_burst;
};

void
ibm5250_irq_ack(ibm5250_t *dev)
{
    if (dev == NULL || !dev->irq_level)
        return;

    dev->irq_level = 0;
    if (dev->irq_clear != NULL)
        dev->irq_clear(dev->irq_clear_priv);
}

static void
ibm5250_irq_cb(void *priv)
{
    ibm5250_t *dev = (ibm5250_t *) priv;

    dev->timer_armed = 0;
    if (dev->irq_raise != NULL)
        dev->irq_raise(dev->irq_priv);

    /* EMDIAG 的 ISR（0x2D0）要定期被叫到才会刷新 [0x2A]/[0x2B]：
       收发期间每 200 µs 再来一次，最多 64 次。 */
    if (dev->serve && (dev->irq_burst < 64)) {
        dev->irq_burst++;
        dev->timer_armed = 1;
        timer_set_delay_u64(&dev->timer, TIMER_USEC * 200);
    }
}

static void
ibm5250_arm_irq(ibm5250_t *dev)
{
    if ((dev->irq_raise == NULL) || dev->timer_armed || getenv("EPOCH5250_NOIRQ"))
        return;

    /* 电平型“数据就绪”：0x64 一武装就拉线，客人写 0x722 才消。
       0x08/0x40/0xAA 不投中断（会破坏对应测试）。 */
    if ((dev->cmdlat != 0x64) || (getenv("EPOCH5250_LEVELIRQ") == NULL))
        return;

    if (dev->irq_level)
        return;

    dev->irq_level = 1;
    dev->irq_raise(dev->irq_priv);
}

static uint8_t
ibm5250_base(ibm5250_t *dev)
{
    /* bit1 is the 0x1C2 phase, but command 0x40 always reads it as 0 -
       that is what the 0x890 register handshake in EMDIAG checks. */
    uint8_t v = (uint8_t) ((dev->latch & 0x7d) |
                           ((dev->cmdlat == 0x40) ? 0x00 : (dev->phase ? 0x02 : 0x00)));

    /* 0x4BD 写 0x1C1=0xFF 后要求 (0x1C6 & 3) == 0 */
    if (dev->latch == 0xFF)
        v &= 0xFC;

    return v;
}

static uint8_t
ibm5250_frame_bit(ibm5250_t *dev, int k)
{
    int cmd40 = (dev->cmdlat == 0x40);
    int d     = cmd40 ? 0x55
              : ((dev->cmdlat == 0x64)
                    ? (dev->seq64 == 0 ? 0x14 : (dev->seq64 == 1 ? 0xAA : 0xDB))
                    : 0xFF);

    if (cmd40 && getenv("EPOCH5250_D40"))         /* 实验用：覆盖 0x40 帧数据字节 */
        d = (int) strtoul(getenv("EPOCH5250_D40"), NULL, 16);
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
        /* B15..B4 = 同步位 1 + 8 数据 + 3 地址 = 12 位；校验位按这 12 位的 1 的个数算，
           不能把校验位自己再数进去（旧代码 +1 等于把结果整体翻转：
           实测 0x500 收到校验 1 → BH=0x3F（偶）与 BL=0xFE（奇）不同 → AH=0x02） */
        ones = 1 + 3;
        for (int t = 0; t < 8; t++)
            ones += (d >> t) & 1;
        o12 = ones;
        /* EPOCH5250_P40 可强制 0x40 帧的校验位取 0（实测 BH/BL 同奇偶才通过） */
        if (cmd40 && getenv("EPOCH5250_P40"))
            return (uint8_t) atoi(getenv("EPOCH5250_P40"));
        return (uint8_t) (cmd40 ? ((o12 & 1) ? 0 : 1) : ((o12 & 1) ? 1 : 0));
    }
    return 0;                           /* fill */
}

static uint8_t
ibm5250_in_1c6(ibm5250_t *dev)
{
    uint8_t base = ibm5250_base(dev);
    uint8_t ret  = (uint8_t) (base & 0x3f);

    if (!dev->serve) {
        /* 只有 EMDIAG 的 0x40/0x64 收发要求静默期读回 0x00
           （0x720 超时返回 AL=(读值<<1)，必须为 0）；
           其它情形（IPL 早期轮询）保持 base，否则 POST 会卡住。 */
        if ((dev->cmdlat == 0x40) || (dev->cmdlat == 0x64) || (dev->cmdlat == 0xAA))
            return 0x00;
        return ret;
    }

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
        /* 帧前多一拍。EPOCH5250_LEAD1=1 时给 1（客人把它当同步位消费掉，
           随后 15 位正好落在 k0..k14）；EPOCH3477_ADR_RW0=1 时给 0（空闲）。 */
        if (getenv("EPOCH5250_LEAD1")) {
            /* 两拍前导：第 1 拍给 1（结束 0x720 的等 1 循环），
               第 2 拍给 0（成为客人 15 位移位窗口的第 1 位）。
               这样窗口整体后移一位，BL.bit1 由起始位 1 变成数据 d0。 */
            dev->dummy--;
            return (uint8_t) ((dev->dummy == 1) ? (ret | 0x80) : (ret & 0x7f));
        }
        dev->dummy = 0;
        return ret;
    }

    if (dev->pre > 0) {
        dev->pre--;
        return (uint8_t) (base & 0x3f);
    }

    {
        int k  = dev->rd / 2;
        int fb = ibm5250_frame_bit(dev, k);

        /* 读配对：0x8A0 每个位槽连读两次（同一 bit），而 ISR(0x2D0) 只单读一次。
           只在第 2 读上推 rd，单读不推进，ISR 采样就不会打乱帧边界。
           以 tsc 间隔区分：同槽两读相隔几十个周期，中断间隔远大于此。 */
        if ((dev->cmdlat == 0x64) && getenv("EPOCH5250_PAIR")) {
            int paired = dev->pair_pend
                      && ((tsc - dev->pair_tsc) <= (uint64_t) atoi(getenv("EPOCH5250_PAIRW") ? getenv("EPOCH5250_PAIRW") : "600"));

            if (!paired) {
                dev->pair_pend = 1;
                dev->pair_tsc  = tsc;
                return (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
            }
            dev->pair_pend = 0;
        }

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
        /* 配对模式下本读已是第 2 读：rd 一次 +2，保持 k=rd/2 与 32 读阈值不变 */
        dev->rd += (((dev->cmdlat == 0x64) && getenv("EPOCH5250_PAIR")) ? 2 : 1);
        if (dev->rd >= 32) {
            dev->rd = 0;
            if (dev->cmdlat == 0x40)
                dev->done40 = 1;
            /* 0x40/0x64 只应答一帧：之后 0x720 必须"无应答"超时
               （BL=0 → AL=0）。0x08 等其它命令保持连续应答，IPL 需要。 */
            if (getenv("EPOCH5250_CODEDUMP") && (dev->cmdlat == 0x64)) {
                uint32_t base = ((uint32_t) cpu_state.seg_ds.seg << 4);
                uint32_t csa  = ((uint32_t) CS > 0xFFFF) ? (uint32_t) CS : ((uint32_t) CS << 4);
                pclog("5250: CODE @CS:6C0 =");
                for (int i = 0; i < 28; i++)
                    pclog(" %02X", mem_readb_phys(csa + 0x6C0 + i));
                pclog("   DS:%04X CS:%04X BX=%04X\n", base >> 4, csa >> 4, BX);
            }
            if (dev->cmdlat == 0x64) {
                dev->seq64 = (dev->seq64 + 1) & 3;
                if (dev->seq64 == 0)
                    dev->serve = 0;      /* 三帧发完再静默 */
            } else if ((dev->cmdlat == 0x40) || (dev->cmdlat == 0xAA)) {
                dev->serve = 0;
            }
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
            /* bit6|bit1 = 适配器签名；bit4 = 0x4B2 的 (al|0xAF)+1==0 检查要求
               (0x50 两位都为 1) */
            return (uint8_t) (dev->latch | 0x52);

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
        case 0x1C8:
            /* 数据寄存器：只存字节，不重新武装（0x6A0 先写 0xDB 再写 0xAA） */
            dev->data8 = val;
            break;

        case 0x1C9:      /* EMDIAG 的 [0x12]：写 0xAA 是“触发一帧应答”，命令号不变 */
        case 0x1CA:
        case 0x1D0:
        case 0x1D1:
            if (val != 0xFF) {
                /* 不要把 val 当命令：0x6A0 的命令在 0x1C0（0x64），
                   0x1C9 只是启动这次收发。 */
                if (port != 0x1C9)
                    dev->cmdlat = val;

                dev->ph     = 1;
                dev->rd     = 0;
                dev->serve  = 1;
                dev->pre    = getenv("EPOCH5250_PRE") ? atoi(getenv("EPOCH5250_PRE")) : 0;
                dev->fno++;
                {
                    uint32_t dsa = ((uint32_t) cpu_state.seg_ds.seg << 4);
                    pclog("5250: CMD %03X = %02X (frame %d) DS:[0x10]=%04X [0x12]=%04X [0x14]=%04X\n",
                          port, val, dev->fno,
                          (dsa < 0x1000000) ? mem_readw_phys(dsa + 0x10) : 0,
                          (dsa < 0x1000000) ? mem_readw_phys(dsa + 0x12) : 0,
                          (dsa < 0x1000000) ? mem_readw_phys(dsa + 0x14) : 0);
                }
            }
            break;

        case 0x1C0:
            if ((val == 0x40) || (val == 0x08) || (val == 0x64)) {
                /* keep the 0x40 reply in place while the subtest is running */
                int hold = (val == 0x08) && (dev->cmdlat == 0x40) && !dev->done40;

                if (!hold) {
                    dev->cmdlat = val;
                    dev->ph     = 1;
                    dev->rd     = 0;
                    dev->serve  = 1;
                    dev->irq_burst = 0;
                    dev->pre    = getenv("EPOCH5250_PRE") ? atoi(getenv("EPOCH5250_PRE")) : 0;
                    dev->fno++;
                    dev->dummy  = getenv("EPOCH5250_LEAD1") ? 2
                                : (getenv("EPOCH3477_ADR_RW0") ? 1 : 0);
                    if (val == 0x40)
                        dev->done40 = 0;
                    if (val == 0x64)
                        dev->seq64 = 0;
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
ibm5250_init(int irq, ibm5250_irq_t irq_raise, void *irq_priv,
             ibm5250_irq_t irq_clear, void *irq_clear_priv)
{
    ibm5250_t *dev = calloc(1, sizeof(ibm5250_t));

    if (dev == NULL)
        return NULL;

    dev->irq       = irq;
    dev->irq_raise = irq_raise;
    dev->irq_priv  = irq_priv;
    dev->irq_clear = irq_clear;
    dev->irq_clear_priv = irq_clear_priv;

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
