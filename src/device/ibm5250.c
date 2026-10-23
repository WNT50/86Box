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

#include "ibm5250_proto.h"
#include "cpu.h"
#include <86box/ibm5250.h>

/* 19-read handshake, measured on the real EMDIAG/0x840 sequence.
   0x8A0 falls through into 0x8AA, so ONE call 0x8A0 reads the data line
   (0x8A7) and then the clock line (0x8B1); the CF from the data read
   survives while AL is overwritten by the clock sample.
   Slot layout, verified against the stack return address of every read:
     rd 0     0x845 data   (warm-up; CF unused)
     rd 1     0x845 clock  (warm-up)
     rd 2     0x848 clock
     rd 3     0x84B data   -> 0x84E jb: bit7=1
     rd 4     0x84B clock  -> 0x865 jb: bit7 must be 0
     rd 5..12 0x86C eight clock samples, adjacent samples must differ
                            (first sample must be 1: [0x2E] holds 0)
     rd 13..15 0x882 station bits, all bit7=1
     rd 16..18 0x88F fill bits, all bit7=0
   After the 19th read the device flips to the frame phase and 0x720
   shifts its 15 bits out of the data line. */
static const uint8_t ibm5250_handshake[19] = {
    0x00, 0x00, 0x00, 0x80,
    0x00,
    0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00,
    0x80, 0x80, 0x80,
    0x00, 0x00, 0x00
};

/* 协议形式的帧描述：B15=sync、B14..B7=data、B6..B4=station、
   B3=parity（覆盖 B15..B4 共 12 位）、B2..B0=fill≥3。
   每条命令一帧或多帧，数据与站号不再散落在三元表达式里。 */
typedef struct {
    uint8_t cmd;
    uint8_t station;    /* 0..6 设备站号；7 = EOM，只能出现在末帧 */
    uint8_t odd;        /* 1 = 奇校验，0 = 偶校验 */
    uint8_t nframes;
    uint8_t data[3];
} ibm5250_frame_desc_t;

static const ibm5250_frame_desc_t ibm5250_frames[] = {
    /* 0x40 Clear Unit：实测用奇校验（手册写偶，待真机/ROM 复核） */
    { 0x40, 7, 1, 1, { 0x55, 0x00, 0x00 } },
    /* 0x64：三帧，数据 14/AA/DB */
    { 0x64, 7, 0, 3, { 0x14, 0xAA, 0xDB } },
    /* 0x08 轮询：单帧 FF */
    { 0x08, 7, 0, 1, { 0x00, 0x00, 0x00 } },
    /* 0x00：块 0x500 的 0x720 收帧要求第一位 B14=1（0x840 用 BL.bit0
       判"本站在线"），数据 0x00 会让 BL.bit0 恒 0；其它块走 ab0 分支
       自带 data 参数，不受这里影响。 */
    { 0x00, 7, 0, 1, { 0x00, 0x00, 0x00 } },
    /* 0x61：0x440 组要求 BL&0x7C==0x54 -> station=5, p=0, B2=1 */
    { 0x61, 0, 0, 1, { 0x2A, 0x00, 0x00 } },
    /* 0x30：0xCA0 线上实际的数据字节（bl.bit0 被当 sync，0x61>>1=0x30）。
       CMDSYNC 用 (word>>1)&0xFF 解出来的就是 0x30，必须能选到同一份帧内容。 */
    { 0x30, 0, 0, 2, { 0x2A, 0x00, 0x00 } },
};

static const ibm5250_frame_desc_t *
ibm5250_frame_desc(uint8_t cmd)
{
    for (size_t i = 0; i < sizeof(ibm5250_frames) / sizeof(ibm5250_frames[0]); i++) {
        if (ibm5250_frames[i].cmd == cmd)
            return &ibm5250_frames[i];
    }
    return NULL;
}

/* 0x1C2 低 4 位是状态选择码：0xFF = 保持原值，SEL_PHASE = 跟 phase。 */
#define SEL_KEEP   0xFF
#define SEL_PHASE  0xFE
typedef struct {
    uint8_t code;
    uint8_t b1;
    uint8_t b0;
} ibm5250_sel_t;

static const ibm5250_sel_t ibm5250_sels[] = {
    { 0x01, 1,         0         },  /* 0x7E5 → 0x7ED jz 要求 bit0=0 */
    { 0x04, SEL_PHASE, SEL_KEEP  },  /* 0x8E1/0x8FF：bit1=phase */
    { 0x08, 1,         1         },  /* 0x7FA → 0x808 jc 要求 bit0=1 */
    { 0x09, 1,         SEL_KEEP  },  /* 0x9D7 → 0x9E8 jnz 要求 bit1=1 */
};

struct ibm5250_t {
    uint8_t  latch;      /* 0x1C1 status latch, mirrored into 0x1C6 */
    uint8_t  cmdlat;     /* command latched at 0x1C0 */
    int      phase;      /* 0x1C6 bit1 for every command except 0x40 */

    int      ph;         /* 1 = handshake, 2 = frame */
    int      hsk;
    int      cklevel;
    int      cksync;
    uint8_t  lastport;
    uint8_t  lastval;
    int      rd;         /* read index inside the current phase */
    int      p500_round; /* 块 0x500 的 8 轮轮询计数：只让其中一轮应答 */
    int      last18;     /* 上一次 0x1C6 写是 0x18（C40 起始脉冲） */
    int      p5resp;
    uint16_t base;       /* 适配器 I/O 基址（0x1C0 或 0x100） */
    int      lastrd_pc;
    int      p5hi;  /* 上一次 0x1C6 读的 pc（判定 0x840 新一轮起点） */     /* 0x500 被寻址轮：回一条能让 0x720 收下的应答帧 */
    int      p500_mark;  /* 本轮已计数（serve=0 时 rd 不前进，避免重复计） */
    int      p500_on;    /* 已进入块 0x500（0x505=08 + 0x515=00 标记） */
    /* ── 协议线路模型（EPOCH5250_PROTO）：以半位为单位推进，读端口只返回线电平 ── */
    int      txpos;      /* 起始序列/帧内的半位位置 */
    int      respond;    /* 本轮收到的站地址 == 本机站号 */
    int      rx_word;    /* 最近一次 0x1C6 写脉冲解码出的接收字 */
    int      my_station; /* 本机站号（0..7） */
    int      tx_reset;   /* 本轮已经由 0xCA0 发送重置过 txpos */
    int      serve;      /* this poll gets a reply */
    int      done40;     /* the 0x40 frame has been served once */
    int      pre;        /* 帧前空槽（每槽 2 读）*/
    uint8_t  data8;      /* 0x1C8 数据寄存器 */
    int      seq64;      /* 0x64 命令的帧序号 */
    int      sub30;      /* 0x30 帧：0=0x840 内部那次，1=0x476 直接调用 */
    int      n720;       /* 0x30 命令下 0x720 完成的握手轮数（ph 1->2 次数） */
    int      p64;        /* 当前块自己 out 0x1C0=0x64（0x6A0）：cmdlat 不许被线上字节回填 */
    int      fno;        /* frame counter (log only) */
    int      u_bit0;     /* 收帧完成事件位（0x1C1 bit0，ISR 采样后清） */
    int      t13n;       /* 实验：块 0x500 的帧位窗口剩余读次数 */
    int      r8_arm;     /* 实验：块 0x500 的 8 轮轮询窗口 */
    int      r8_skip;    /* 需要压掉的同步机会数 */
    int      r8_cnt;     /* 窗口内 0x1C0 写计数（每轮 22 次） */
    int      r8b_round;  /* 0x840 轮次（以栈返回地址 0x848 计） */
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
    int      ready;
    int      win_irq;      /* 0x8C0 判据链还要一次中断采样（块 0x7A0） */
    int      w42;
    int      bit0;
    int      c3;          /* 0x1C3 写入值（块 0xAB0 写 0x08） */
    int      ab0;         /* 处于块 0xAB0 的 0x840 收帧上下文 */
    int      fhunt;       /* 0x720 搜寻同步位时已数到的空闲拍 */
    int      fsync;       /* 0x720 已锁定同步位，进入 15 位收帧 */
    int      fbit;        /* 0x720 已收的位数 0..14（对应 B14..B0） */
    int      flast;       /* 0x720 上一位的值（时钟读沿用） */
    int      c1_cnt;      /* 0x5B0 循环里 0x1C1=0x80 的计数（每轮一次） */
    int      a7_pend;     /* 延迟中断：等 0xCA0 清完 [0x2A] 再拉 */
    int      c1_ff;       /* 0x1C1 bit7 回读触发器：写 bit6=1 → 置 1，写 bit6=0 → 清 0 */
    int      clkc;        /* 块 0xAB0 时钟读计数（每次时钟读翻转） */
    int      fseq;        /* 0x720 已收完的帧号（第 k 帧数据字节 = k+1） */
    int      samp;        /* pc 采样剩余次数（0xAB0 诊断） */
    int      b1;
    int      b1_pulse;      /* 0x1C2=0x0A 后只喂 0xBB6 一次 bit1 */          /* 0x1C6.bit1 的选择器输出 */
    void   (*shared_clear)(void *priv);
    void    *shared_priv;
    int      poll_left;   /* 0x8C0 窗口轮询计数 */
    int      caller;          /* 数据有效：ISR 采样时 bit1=1 */
    int      pair_pend;      /* 一次位槽的第 1 读已发生 */
    uint64_t pair_tsc;
    uint64_t last_tsc;
    pc_timer_t timer;
    int      timer_armed;
    int      irq_burst;
};

void
ibm5250_irq_ack(ibm5250_t *dev)
{
    if (dev == NULL || !dev->irq_level)
        return;

    if (getenv("EPOCH5250_PCLOG"))
        pclog("5250: IRQACK level->0 (seq=%d)\n", dev->seq64);

    dev->irq_level = 0;
    if (dev->irq_clear != NULL)
        dev->irq_clear(dev->irq_clear_priv);
}

static void
ibm5250_irq_cb(void *priv)
{
    ibm5250_t *dev = (ibm5250_t *) priv;

    dev->timer_armed = 0;

    if (dev->a7_pend) {
        dev->a7_pend = 0;
        if (getenv("EPOCH5250_PCLOG"))
            pclog("5250: A7PEND raise pc=%04X lvl=%d cmd=%02X\n",
                  cpu_state.pc, dev->irq_level, dev->cmdlat);
        /* 先真正拉低再拉高：上一轮中断若没走 0x722，8259 的 IRQ5 还在服务中，
           直接 picint 不会产生新的边沿（0x7A0 的重武装也是这么做的）。 */
        if (dev->irq_clear != NULL)
            dev->irq_clear(dev->irq_clear_priv);
        dev->irq_level = 1;
        if (dev->irq_raise != NULL)
            dev->irq_raise(dev->irq_priv);
        return;
    }

    if (dev->samp > 0) {
        static long sn = 0;
        dev->samp--;
        sn++;
        if ((cpu_state.pc >= 0x0840) && (cpu_state.pc <= 0x08A0) && (sn <= 400))
            pclog("5250: PCTR n=%ld pc=%04X\n", sn, cpu_state.pc);
        if (dev->samp > 0) {
            dev->timer_armed = 1;
            timer_set_delay_u64(&dev->timer, 1 * TIMER_USEC);
        }
        return;
    }

    if (getenv("EPOCH5250_POLLOG"))
        pclog("5250: CB pc=%04X lvl=%d poll=%d raise=%d\n",
              cpu_state.pc, dev->irq_level, dev->poll_left, dev->irq_raise != NULL);
    if (dev->irq_raise == NULL || dev->irq_level) {
        /* 线还高（等客人写 0x722 应答）：继续等窗口，别让轮询断掉。 */
        if (dev->irq_raise != NULL && getenv("EPOCH5250_LEVELIRQ") &&
            ((dev->cmdlat == 0x00) || (dev->cmdlat == 0x10) || dev->win_irq) &&
            (dev->c3 != 0x08) && (dev->poll_left > 0)) {
            dev->timer_armed = 1;
            timer_set_delay_u64(&dev->timer, 1 * TIMER_USEC);
        }
        return;
    }

    /* 块 0x7A0：等客人真正进入 0x8C0 且已过 0xCA0 的清零（pc>=0x8C3）
       再拉线，避免中断样本被 [0x2A]/[0x2B] 的清零抹掉。 */
    if (dev->poll_left > 0) {
        uint16_t pc = cpu_state.pc;
        int in_win = ((pc >= 0x8C3) && (pc <= 0x912)) ||
                     ((pc >= 0xC48) && (pc <= 0xC63));

        {
            static int dbgn;
            if (getenv("EPOCH5250_POLLOG") && (dbgn < 25)) {
                pclog("5250: POLL pc=%04X win=%d left=%d\n", pc, in_win, dev->poll_left);
                dbgn++;
            }
        }
        if (!in_win) {
            dev->poll_left--;
            if (dev->poll_left > 0) {
                dev->timer_armed = 1;
                timer_set_delay_u64(&dev->timer, 1 * TIMER_USEC);
            }
            return;
        }
        dev->poll_left = 0;
    }

    dev->irq_level = 1;
    dev->irq_raise(dev->irq_priv);
    if (getenv("EPOCH5250_PCLOG"))
        pclog("5250: IRQ ASSERT (level) after trigger\n");

    /* 0x8C0 可能被同一块多次调用：保持窗口轮询，直到客人换命令。 */
    if (getenv("EPOCH5250_LEVELIRQ") &&
        ((dev->cmdlat == 0x00) || (dev->cmdlat == 0x10) || dev->win_irq) &&
        (dev->c3 != 0x08)) {
        dev->poll_left = 2000;
        dev->timer_armed = 1;
        timer_set_delay_u64(&dev->timer, 1 * TIMER_USEC);
    }

    /* 诊断：确认 ISR 的 [0x2B] 是否写进了判据所用的 DS */
    if (getenv("EPOCH5250_FORCE2B")) {
        uint32_t dsa = ((uint32_t) cpu_state.seg_ds.seg << 4);
        if (dsa < 0x1000000) {
            pclog("5250: DSAT ds=%04X mem2b=%02X mem2a=%02X\n",
                  cpu_state.seg_ds.seg, mem_readb_phys(dsa + 0x2B),
                  mem_readb_phys(dsa + 0x2A));
        }
    }
}

/* EMDIAG 的两份 ISR（0x2D0 / 0x2EC）里对 0x1C1 / 0x1C6 的采样读 */
#define ISR_PC(pc) (((pc) == 0x2D7) || ((pc) == 0x2DF) || \
                    ((pc) == 0x2F7) || ((pc) == 0x2FB))

static void
ibm5250_arm_irq(ibm5250_t *dev)
{
    if ((dev->irq_raise == NULL) || getenv("EPOCH5250_NOIRQ"))
        return;

    /* 电平型“数据就绪”：0x64 一武装就拉线，客人写 0x722 才消。
       0x08/0x40/0xAA 不投中断（会破坏对应测试）。 */
    /* 只有 0x64 在触发时拉线；0x00（块 0x7A0）必须等进入 0x8C0 的窗口，
       由 ibm5250_irq_cb 的 pc 轮询投递，不能在这里立即拉。 */
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
    /* 协议（见 references/3477s-emdiag-protocol-table.md）：
       0x1C6 bit7 = 数据位、bit1 = 错误位（正常必须为 0）、bit0 = 就绪。
       phase 不是 0x1C6 的内容，绝不映射到 bit1 —— 之前正是它让每个块
       的 `1C6 & 2` 判据直接失败（AH=0x03 / 0xA1 …）。 */
    uint8_t v = (uint8_t) (dev->latch & 0x7d);

    if (dev->b1_pulse && (cpu_state.pc == 0x0BB6)) {
        dev->b1_pulse = 0;
        v |= 0x02;
    }

    /* 0x8C0 的判据链证明 bit1 = phase（0x1C2 写入 0x04 时更新）：
       0x8E8 要求 bit1=1、0x904 在写 0x1C1=0x42 之后要求 bit1=0。 */
    if (dev->b1)
        v |= 0x02;
    /* 0x1C2 是状态选择器：写 0x01 → bit0=0，写 0x08 → bit0=1
       （0x7ED `jz` / 0x808 `jc` 两条字节级判据）。 */
    if (dev->bit0)
        v |= 0x01;

    /* 0x4BD 写 0x1C1=0xFF 后要求 (0x1C6 & 3) == 0 */
    if (dev->latch == 0xFF)
        v &= 0xFC;

    return v;
}

static uint8_t
ibm5250_ab0_frame_bit(ibm5250_t *dev, int k, int data)
{
    /* 站号固定 7（EOM）：0xB11 的 (BH|0xF1)+1==0 要求 W8..W11 全 1。 */
    if (k == 14)
        dev->u_bit0 = 1;        /* 一帧收完 → 状态寄存器 bit0 事件 */
    return ibm5250_proto_frame_bit(k, data, 7);
}

static uint8_t
ibm5250_frame_bit(ibm5250_t *dev, int k)
{
    const ibm5250_frame_desc_t *f = ibm5250_frame_desc(dev->cmdlat);
    int d, ones, o12;

    if (dev->t13n > 0)
        dev->t13n--;

    if (f == NULL)
        f = ibm5250_frame_desc(0x08);

    {
        /* 0x30 也是两帧序列：第一次（0x840 内的 0x720）要 bl&0x7C==0x54，
           第二次（0x476 的 0x720）要 al==0，即 bl 为 0/1。 */
        int fi;
        if (dev->cmdlat == 0x64)
            fi = dev->seq64 % f->nframes;
        else if (dev->cmdlat == 0x30)
            fi = dev->sub30 % f->nframes;
        else
            fi = 0;
        d = f->data[fi];
        /* 协议：多帧应答前 n-1 帧用本站地址，只有末帧用 7（EOM） */
        if (getenv("EPOCH5250_EOM") && (f->nframes > 1))
            f = &(ibm5250_frame_desc_t){ f->cmd, (uint8_t) ((fi == (f->nframes - 1)) ? 7 : 2),
                                         f->odd, f->nframes, { f->data[0], f->data[1], f->data[2] } };
    }

    if (dev->cmdlat == 0x40 && getenv("EPOCH5250_D40"))   /* 实验覆盖 */
        d = (int) strtoul(getenv("EPOCH5250_D40"), NULL, 16);

    if (k == 15) {
        dev->u_bit0 = 1;
        /* 实验：块 0x500 的帧补位改成 1，让 15 次 rcr 后 BL.bit0=1 */
        if (getenv("EPOCH5250_T13") && (dev->fno == 7) && (dev->cmdlat == 0x64))
            return 1;
    }

    if (k == 0)                 return 1;                 /* B15 sync */
    /* nat-1992 3.1.1: 8-bit data byte (B7-B14), LSB first。
       位流从 B15 往 B0 发，所以 B14 = data bit0 ... B7 = data bit7。 */
    if (k <= 8)                 return (uint8_t) ((d >> (k - 1)) & 1);  /* B14..B7 */
    if (k <= 11)                return (uint8_t) ((f->station >> (k - 9)) & 1); /* B6..B4 */
    if (k == 12) {
        /* 0x820: bl 偶 -> 直接通过；bl 奇则要求 bh 奇。等价于 16 位
           (sync + B14..B0) 偶数个 1，sync 参与校验。 */
        ones = 1;                                          /* sync(B15) 参与 12 位校验 */
        for (int t = 0; t < 8; t++)
            ones += (d >> t) & 1;                          /* data */
        for (int t = 0; t < 3; t++)
            ones += (f->station >> t) & 1;                 /* station */
        o12 = ones;
        if (dev->cmdlat == 0x40 && getenv("EPOCH5250_P40"))
            return (uint8_t) atoi(getenv("EPOCH5250_P40"));
        /* 偶校验：12 位里 1 的个数为奇数时补 1；奇校验反过来 */
        {
            uint8_t p = (uint8_t) (f->odd ? ((o12 & 1) ? 0 : 1) : ((o12 & 1) ? 1 : 0));
            if (getenv("EPOCH5250_T13") && (dev->fno == 7) && (dev->cmdlat == 0x64))
                p ^= 1;
            return p;
        }
    }
    if (dev->cmdlat == 0x61) {
        /* emu440.py 枚举结果：data=0x2A、station=0、fill=0b011
           -> 0x720 返回 bh=0xC0 bl=0x55，四个判据同时成立 */
        if (k == 14) return 1;
        if (k == 15) return 1;
    }
    return 0;                                              /* B2..B0 fill */
}

/* 协议线路模型：每次 0x1C6 读采样一个半位。
 *
 *   n 0..2    0        暖机（0x845 两读 + 0x848 一读）
 *   n 3..12   1,0,...  5 个 quiesce 位（半位交替，奇拍为 1）
 *   n 13..15  1        编码违例正半位（1.5 位）
 *   n 16..18  0        编码违例负半位（1.5 位）
 *   n 19      1/0      sync：被寻址给 1，否则 0
 *   n >= 20   B14..B0  每两个半位一位：电平 = 位值 ^ 半位号
 *
 * 0x840 正好消费 n 0..18，0x720 的搜寻读落在 n=19，随后 15 次
 * call 0x8A0（每次两读）覆盖 30 个半位 = B14..B0。
 */
static int
ibm5250_line_level(ibm5250_t *dev)
{
    int n = dev->txpos;

    /* 实验（doc：槽位 0 起算 R=0x1C03 奇数→校验翻位；槽位 1 起算 R=0x0E01
       偶数→通过）：把 sync 之后的整段（sync + 15 位）统一后移一个位槽，即
       2 个半位，客人的 15 位窗口就从槽位 1 起算。 */
    if (getenv("EPOCH5250_P500S2") && (n >= 19))
        n -= 2;

    if (n < 3)
        return 0;
    if (n < 13)
        return (n & 1);
    if (n < 16)
        return 1;
    if (n < 19)
        return 0;
    if (n == 19)
        return dev->respond ? 1 : 0;
    if (!dev->respond)
        return 0;

    {
        /* sync 占 n=19,20 两个半位；B14 的第一个半位从 n=21 起。 */
        /* 实测：0x720 的数据读落在 n=21,23,...（每次 call 的第二读），
           sync 占 n=19,20，所以帧从 n=21 起。 */
        int m  = n - 21;
        int bi = m >> 1;
        int  bit;

        if (bi >= 15)
            return 1;                       /* 帧后回到空闲 */
        /* 实验：EMDIAG 0x820/0x597 要求 bh=bl=0xFF（data 全 1），
           即轮询应答的 15 位全 1。协议上等同“无数据、全 1 空闲码”。 */
        if (getenv("EPOCH5250_FRAMEFF") &&
            ((dev->cmdlat == 0x00) || (dev->cmdlat == 0x08)))
            return 1;
        /* 0x500 被寻址轮（站地址 0x1221）的应答帧：data=0x00、station=111、
           parity=0、fill=0,0,1（离线枚举：15 位 00000000 111 0 001 经
           15 次 rcr 得 bx=0x8E01，bl bit0=1（0x55A 计数），且
           PF(bl)=1 odd != PF(bh)=4~ 的偶数 ⇒ 0x820 不翻位、0x75C 过）。真实终端的应答帧
           内容还没从手册核出来，这里按两条判据反解。 */
        if (dev->p5resp && dev->p500_on) {
            static const uint8_t p5b[16] = { 0,0,0,0,0,0,0,0,0, 1,1,1,0, 0,0,0 };
            return (uint8_t) (p5b[bi + 1] ^ (m & 1));
        }
        bit = ibm5250_frame_bit(dev, bi + 1);
        return bit ^ (m & 1);
    }
}


static uint8_t
ibm5250_in_1c6(ibm5250_t *dev)
{
    uint8_t base = ibm5250_base(dev);

    /* 0xBA0/AH=0x03 的时序线：0xC40 清 [0x2B] 是内存写，设备看不见，
       但它前后必经这些 0x1C6 的 I/O 访问。按顺序记下来就能看出
       0x8C7 之前最后一次写 [0x2B] 的是 ISR 还是 0xC40。 */
    if (getenv("EPOCH5250_TLINE") && (dev->cmdlat == 0x64)) {
        static int tl;
        const char *ev = NULL;
        switch (cpu_state.pc) {
        case 0x0A73: ev = "C40-clear"; break;
        case 0x0A81: ev = "C40-send1"; break;
        case 0x0C4D: case 0x0C56: case 0x0C83: ev = "C90-tx"; break;
        case 0x08A8: case 0x08B2: ev = "720-read"; break;
        case 0x02D7: case 0x02DF: ev = "ISR-read"; break;
        }
        if (ev && (tl++ < 240))
            pclog("5250: TLINE %-10s pc=%04X base=%02X serve=%d\n",
                  ev, cpu_state.pc, base, dev->serve);
    }

    /* 0xBA0 的 AH=0x03 来自 0x8C7 的 [0x2B]&2，而 [0x2B] 是 ISR 里读 0x1C6
       采样的。这里直接记录 ISR 的读点 pc 与返回值，别再猜地址。 */
    if (getenv("EPOCH5250_ISRLOG") && (cpu_state.pc >= 0x02D0) && (cpu_state.pc <= 0x0300)) {
        static int isn;
        if (isn++ < 40)
            pclog("5250: ISR pc=%04X base=%02X b1p=%d b1=%d serve=%d cmd=%02X\n",
                  cpu_state.pc, base, dev->b1_pulse, dev->b1, dev->serve, dev->cmdlat);
    }

    /* 0x440 的第二次 0x720（0x476 直接调用）：不给帧，让 0x720 的 255 次
       搜寻全部超时、走 0x732 ret，AL 保持 0 —— 0x479 的 `or al,al` 才能过。
       0x840 内部那次（sub30=0）仍然正常应答，供 0x45E/0x469 检查 bl。 */
    if ((dev->cmdlat == 0x30) && (dev->sub30 == 1)) {
        if (getenv("EPOCH5250_SUB30")) {
            static int sn30;
            if (sn30++ < 8)
                pclog("5250: SUB30-ZERO pc=%04X rd=%d\n", cpu_state.pc, dev->rd);
        }
        return 0x00;
    }

    if (getenv("EPOCH5250_POLLOG") && (dev->cmdlat == 0x00) && (dev->c3 == 0x08)) {
        static long rn = 0; rn++;
        if (rn <= 200)
            pclog("5250: ABR n=%ld pc=%04X ph=%d rd=%d serve=%d\n",
                  rn, cpu_state.pc, dev->ph, dev->rd, dev->serve);
    }

    if (ISR_PC(cpu_state.pc) && getenv("EPOCH5250_POLLOG"))
        pclog("5250: ISRRD pc=%04X cmd=%02X ph=%d rd=%d base=%02X serve=%d ds=%04X\n",
              cpu_state.pc, dev->cmdlat, dev->ph, dev->rd, base, dev->serve,
              cpu_state.seg_ds.seg);

    if (getenv("EPOCH5250_PCLOG") && (dev->cmdlat == 0x64)) {
        static int n;
        if (n < 400) {
            pclog("5250: PCLOG rd=%02d pc=%04X serve=%d seq=%d\n",
                  dev->rd, cpu_state.pc, dev->serve, dev->seq64);
            n++;
        }
    }
    uint8_t ret  = (uint8_t) (base & 0x3f);

    if (getenv("EPOCH5250_STK720") &&
        ((cpu_state.pc == 0x08A8) || (cpu_state.pc == 0x08B2))) {
        static long sn = 0;
        uint32_t spx = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
        {
            uint16_t s0 = mem_readw_phys(spx);
            uint16_t s2 = mem_readw_phys(spx + 2);
            /* 0x440 组（0x476 call 0x720，返回 0x479）：栈顶 0x72C=搜寻读，
               0x743=收帧读。0x500 的 0x840→0x720 栈顶 0x743、SP+2=0x89D。 */
            if ((sn++ < 600) && ((s2 == 0x0479) || ((s0 == 0x0743) && (s2 == 0x089D))))
                pclog("5250: STK720 n=%ld pc=%04X ph=%d serve=%d cmd=%02X "
                      "r1=%04X r2=%04X r3=%04X r4=%04X\n",
                      sn, cpu_state.pc, dev->ph, dev->serve, dev->cmdlat,
                      s0, s2, mem_readw_phys(spx + 4), mem_readw_phys(spx + 6));
        }
    }

    /* 块 0x500 是 8 轮站地址轮询，只有一轮该应答；每轮的第一个数据读
       (0x845, pc=0x8A8) 作为轮次标记。必须放在 serve 检查之前：
       未被寻址的轮次 serve=0，否则计数永远不涨。 */
    if (!dev->serve) {
        /* 只有 EMDIAG 的 0x40/0x64 收发要求静默期读回 0x00
           （0x720 超时返回 AL=(读值<<1)，必须为 0）；
           其它情形（IPL 早期轮询）保持 base，否则 POST 会卡住。 */
        if ((dev->cmdlat == 0x40) || (dev->cmdlat == 0x64) || (dev->cmdlat == 0xAA))
            return 0x00;
        /* 0x500 的未寻址轮次：读回 0 让 0x720 的搜寻超时（0x732 ret），
           不进收帧分支 —— 否则会拿一条无效帧走 0x75C，整机报 AH=0x02 提前退出。 */
        if (dev->p500_on && !getenv("EPOCH5250_LEGACY"))
            return 0x00;
        return ret;
    }

    /* 协议线路模型（块 0x500）：不查 pc、不查 rd，只按半位输出线电平。 */
    if (dev->p500_on && !getenv("EPOCH5250_LEGACY")) {
        int lvl;

        if (ISR_PC(cpu_state.pc)) {
            dev->txpos++;      /* 中断里的读同样消耗一个线上半位 */
            return (uint8_t) ((base & 0x3d) | (dev->ready ? 0x02 : 0x00));
        }

        /* 线路模型默认应答本轮；站地址匹配留给 0x1C6 解码出的 rx_word。 */
        if (getenv("EPOCH5250_BITLOG") && (cpu_state.pc == 0x08A8)) {
            uint32_t spb = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
            uint16_t rcb = (spb < 0x1000000) ? mem_readw_phys(spb) : 0;
            if (rcb == 0x0743)
                pclog("5250: BIT n=%02d lvl=%d cmd=%02X\n", dev->txpos, ibm5250_line_level(dev), dev->cmdlat);
        }
        /* 0x500 的一轮 = 一次 0x840：它的同步搜寻循环头在 pc=0x0849。
           上一读还在搜寻/移位（0x0849/0x08A8/0x08B2）时不动，否则就是
           新的一轮开始 —— 用 51 读窗口对齐会与客人的轮次错拍。 */
        if (dev->p500_on && (cpu_state.pc == 0x0849) &&
            (dev->lastrd_pc != 0x0849) && (dev->lastrd_pc != 0x08A8) &&
            (dev->lastrd_pc != 0x08B2)) {
            dev->txpos = 0;
            dev->p500_round++;
        }
        dev->lastrd_pc = cpu_state.pc;
        if (dev->p500_on && (dev->txpos == 0) && (cpu_state.pc != 0x0849)) {
            dev->p500_round++;                 /* 51 读窗口兜底（非 0x840 起点） */
            if (dev->p500_round > 9) {
                dev->p500_on = 0;              /* 8 轮轮询结束 */
                dev->txpos   = 0;
                return (uint8_t) (base & 0x3f);
            }
        }
        /* 应答窗口：p5resp 命中的那一轮（0x580 要求它是第 8 轮，[0x30]==7），
           再加上前 3 轮兼容旧路径。 */
        dev->respond = dev->p500_on && (dev->p5resp || (dev->p500_round <= 3));
        /* 一轮 = 0x840 的 19 读 + sync + 15 位×2 + 1 = 51 个读；
           靠读计数闭环，不依赖客人写到哪个端口。 */
        if (dev->p500_on && (dev->txpos >= 51))
            dev->txpos = 0;
        lvl = ibm5250_line_level(dev);
        if (getenv("EPOCH5250_P5R") && (cpu_state.pc == 0x08A8)) {
            static int r5;
            if (r5++ < 90)
            {
                uint32_t d3 = ((uint32_t) cpu_state.seg_ds.seg << 4);
                pclog("5250: P5R tx=%d round=%d resp=%d serve=%d p5r=%d lvl=%d [26]=%04X [28]=%02X [2F]=%02X\n",
                      dev->txpos, dev->p500_round, dev->respond, dev->serve, dev->p5resp,
                      ibm5250_line_level(dev),
                      mem_readw_phys(d3 + 0x26), mem_readb_phys(d3 + 0x28), mem_readb_phys(d3 + 0x2F));
            }
        }
        if (getenv("EPOCH5250_P500SHIFT") && dev->serve &&
            ((cpu_state.pc == 0x08A8) || (cpu_state.pc == 0x08B2))) {
            static int sn5;
            if (sn5++ < 130)
                pclog("5250: P5S n=%d pc=%04X lvl=%d\n", dev->txpos, cpu_state.pc, lvl);
        }
        dev->txpos++;
        if (dev->txpos >= 19)
            dev->tx_reset = 0;
        if (getenv("EPOCH5250_LOGPROTO"))
            pclog("5250: PROTO n=%02d lvl=%d resp=%d pc=%04X\n",
                  dev->txpos - 1, lvl, dev->respond, cpu_state.pc);
        return (uint8_t) ((base & 0x3f) | (lvl ? 0x80 : 0x00));
    }

    /* 协议：0x64 的响应在 0x1C9 触发后才就绪；0xCA0 清 [0x2A] 之后、
       0x840 开始收帧时拉 IRQ4，ISR 采样 0x1C1 写入 [0x2A]（0x6F8 检查）。 */
    if (dev->ph != 2) {
        /* ISR 的采样读不能消耗握手槽 */
        if (ISR_PC(cpu_state.pc)) {
            dev->txpos++;      /* 中断里的读同样消耗一个线上半位 */
            return (uint8_t) ((base & 0x3d) | (dev->ready ? 0x02 : 0x00));
        }
        /* 协议：接收方以 sync=1 对齐；帧前必须是空闲 0。
           0x64 的 19 槽握手是实测拟合值，只用于它自己；其它命令（0x00/0x08…）
           给纯空闲，否则 0x840 的同步搜索会提前命中握手里的 1 而错位。 */
        /* 每轮 0x840 的第一次 0x8A0 调用：栈顶返回地址 = 0x848 */
        if (dev->r8_arm && (cpu_state.pc == 0x8A8)) {
            uint32_t spa = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
            if (mem_readw_phys(spa) == 0x848) {
                dev->r8b_round++;
                if (getenv("EPOCH5250_R8LOG") && (dev->r8b_round < 40))
                    pclog("5250: R8 round=%d cnt=%d rd=%d ph=%d\n",
                          dev->r8b_round, dev->r8_cnt, dev->rd, dev->ph);
            }
        }

        if (getenv("EPOCH5250_RDPORT")) {   /* 本函数即 0x1C6 读路径 */
            static int rdp_n;
            if (rdp_n < 200) {
                rdp_n++;
                pclog("5250: RDP pc=%04X port=1C6 ph=%d rd=%d ab0=%d base=%02X\n",
                      cpu_state.pc, dev->ph, dev->rd, dev->ab0, base);
            }
        }

        /* 一个位时间被读两次（数据 0x8A7 + 时钟 0x8B1），两次必须给同一位；
           否则 0x84E 的 CF 与 0x865 的 CF 来自不同槽，同步永远判不出来。 */
        int pair2 = getenv("EPOCH5250_PAIR2") ? 1 : 0;
        int pair3 = getenv("EPOCH5250_PAIR3") ? 1 : 0;
        int pair4 = getenv("EPOCH5250_PAIR4") ? 1 : 0;
        int hsi = pair2 ? (dev->rd >> 1) : dev->rd;

        if (pair4) {
            /* 数据线 / 时钟线分离：
               - 0x8A7（数据）：按握手序列给出位，0x80 = 1
               - 0x8B1（时钟）：逐拍翻转；紧跟同步位的那一拍给 0，
                 使 0x865 的 CF=0 且 0x867 的 8 拍比较能看到变化 */
            if (cpu_state.pc == 0x08A7) {
                int b = dev->ab0 ? 0 : (ibm5250_handshake[dev->hsk % 19] ? 1 : 0);
                ret = (uint8_t) ((base & 0x3f) | (b ? 0x80 : 0x00));
                dev->hsk++;
                if (dev->hsk >= 19)
                    dev->hsk = 0;
                dev->cksync = 1;
            } else {
                if (dev->cksync) {
                    dev->cklevel = 0;
                    dev->cksync  = 0;
                } else {
                    dev->cklevel ^= 1;
                }
                ret = (uint8_t) ((base & 0x3f) | (dev->cklevel ? 0x80 : 0x00));
            }
        } else if (pair3) {
            /* 数据读（pc=0x8A7）给握手位，时钟读（pc=0x8B1）给 0：
               0x84E 判数据 bit7、0x865 判时钟 bit7，两者不能同时为 1。 */
            if (cpu_state.pc == 0x08A7) {
                ret = (uint8_t) ((base & 0x3f)
                                 | (dev->ab0 ? 0x00 : ibm5250_handshake[dev->hsk % 19]));
                dev->hsk++;
                if (dev->hsk >= 19)
                    dev->hsk = 0;
            } else {
                ret = (uint8_t) (base & 0x3f);
            }
        } else
        ret = (uint8_t) ((base & 0x3f)
                         | (dev->ab0 ? 0x00
                                                  : ibm5250_handshake[hsi % 19]));
        if (dev->r8_arm && (dev->r8b_round < 8) && (ret & 0x80))
            ret &= (uint8_t) ~0x80;

        if (getenv("EPOCH5250_LOG19") && (dev->rd < 19)) {
            static int l19;
            uint32_t s19 = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
            uint16_t r19 = (s19 < 0x1000000) ? mem_readw_phys(s19) : 0;
            l19++;
            pclog("5250: L19 n=%d rd=%02d pc=%04X ret=%02X ph=%d r1=%04X rnd=%d [29]=%02X\n",
                  l19, dev->rd, cpu_state.pc, ret, dev->ph, r19, dev->p500_round,
                  (cpu_state.seg_ds.seg < 0x10000) ? mem_readb_phys(((uint32_t) cpu_state.seg_ds.seg << 4) + 0x29) : 0);
        }

        if (++dev->rd >= (pair2 ? 38 : 19)) {
            dev->ph = 2;
            dev->rd = 0;
            dev->p500_mark = 0;
            dev->dummy = 0;
            if (dev->cmdlat == 0x30)
                dev->n720++;
        }
        return ret;
    }

    /* 0x440 的第二次 0x720（0x476）：ph 已经在第一次 0x840 里推到 2，
       这里若继续给帧，0x720 会正常收出一条帧、AL 非 0（AH=0x23）。
       让线路整段为 0，搜寻 255 次超时，0x732 ret 时 AL 保持 0。 */
    /* 实验：0x440 的 0x720 在 ph==2 时 rd 恒为 0，15 位窗口形不成。
       这里按 pc 推进 rd，数据源沿用 19 槽握手数组（与握手期同构）。 */
    if (getenv("EPOCH5250_PH2RD") && (dev->cmdlat == 0x30) && (dev->n720 >= 1) &&
        ((cpu_state.pc == 0x08A7) || (cpu_state.pc == 0x08A8) ||
         (cpu_state.pc == 0x08B1) || (cpu_state.pc == 0x08B2))) {
        /* rd 0..29 = 第一次 0x720（0x840 内）：给 data=0x2A 的帧
           rd >= 30 = 第二次 0x720（0x476）：整段 0 → 搜寻超时 → AL=0
           位序按 emu440.py 已验证的 15 次 rcr 模型：第 i 次数据读送
           frame_bit(i+1)，紧跟着的时钟读送反相位（half_bit 的 m&1）。 */
        int is_data = (cpu_state.pc == 0x08A7) || (cpu_state.pc == 0x08A8);
        int fb2 = 0, fdata = 0x2A, r = dev->rd;
        if (dev->rd >= 44) {
            /* 第二次 0x720（0x476）：前 247 位搜寻失败 → 第 248 次命中
               (cl=0xF7，对应 0x482 的 [0x2D]==0xF7)，帧内容全 0 → bl<=1。 */
            r = dev->rd - 44;
            fdata = 0x00;
        }
        int sync_at = (dev->rd >= 44) ? 494 : 12;
        int frm_at  = (dev->rd >= 44) ? 496 : 14;
        int seg = (r < sync_at) ? 0 : (r < sync_at + 2) ? 1 : (r < frm_at + 30) ? 2 : 3;
        if (seg == 1) {
            fb2 = is_data ? 1 : 0;                /* sync */
        } else if (seg == 2) {
            int k = ((r - frm_at) >> 1) + 1;
            int ones = 1;
            for (int i = 0; i < 8; i++) ones += (fdata >> i) & 1;
            if (k >= 1 && k <= 8)        fb2 = (fdata >> (k - 1)) & 1;
            else if (k >= 9 && k <= 11)  fb2 = 0;
            else if (k == 12)            fb2 = ones & 1;
            else if (k >= 13 && k <= 15) fb2 = (0x03 >> (15 - k)) & 1;
            if (!is_data) fb2 ^= 1;
        }
        uint8_t r2b = (uint8_t) ((base & 0x3f) | (fb2 ? 0x80 : 0x00));
        if (dev->rd < 700)
            dev->rd++;
        if (getenv("EPOCH5250_PH2LOG") && (dev->rd < 700)) {
            static int p2n;
            if (p2n++ < 240)
                pclog("5250: PH2 rd=%d pc=%04X fb=%d ret=%02X\n",
                      dev->rd, cpu_state.pc, fb2, r2b);
        }
        return r2b;
    }

    if (getenv("EPOCH5250_CMD30") && (dev->cmdlat == 0x30)) {
        static int c30;
        if (c30++ < 60)
            pclog("5250: CMD30 pc=%04X ph=%d rd=%d n720=%d sub30=%d serve=%d\n",
                  cpu_state.pc, dev->ph, dev->rd, dev->n720, dev->sub30, dev->serve);
    }
    if ((dev->cmdlat == 0x30) && (dev->n720 >= 2)) {
        if (getenv("EPOCH5250_SUB30")) {
            static int sn30b;
            if (sn30b++ < 6)
                pclog("5250: SUB30-NOFEED pc=%04X n720=%d rd=%d\n",
                      cpu_state.pc, dev->n720, dev->rd);
        }
        return 0x00;
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

    /* 块 0xAB0：按手册的起始序列（5 quiesce 位 + 3 位违规 + 帧）。
       实测第一拍落在 pc=0x8B2（rd=0），quiesce 的 1 必须在下一拍
       （pc=0x8A8，数据读）出现；之后每个读都是半位样本，逐拍翻转。
         rd=1..8    1,0,1,0,…（quiesce，相邻样本交替）
         rd=9..11   1,1,1   （违规正半位，1.5 位）
         rd=12..14  0,0,0   （违规负半位，1.5 位）
         rd=15..28  0       （0x720 等 sync，前 7 次尝试）
         rd=29      1       （第 8 次尝试 ⇒ [0x2D]=7）
         rd=30..59  15 位，s8=1、s7=0 ⇒ AL=(BL>>1)=1 */
    if (dev->ab0) {
        int rd = dev->rd;
        int fb;

        if (getenv("EPOCH5250_RLOG")) {
            static int rn;
            if ((rn < 120) && (cpu_state.pc >= 0x08A0) && (cpu_state.pc <= 0x08C0)) {
                pclog("5250: R pc=%04X rd=%d\n", cpu_state.pc, dev->rd);
                rn++;
            }
        }

        /* 只让 0x840 的两个读点（数据 0x8A7、时钟 0x8B1）推进序列；
           块内 0x71 检查等其它读不占位，否则同步位会错一拍。 */
        /* ISR(0x2D6) 读 0x1C6 并写入 [0x2A]；块 0xAB0 的 0xB03 要求
           ([0x2A] | 0xB7) + 1 == 0 ⇒ 至少要有 bit6 与 bit3。 */
        /* ISR 两次读：0x2D6 读 0x1C1 → [0x2A]，0x2DE 读 0x1C6 → [0x2B]。
           块 0x920 的 0x95E 要求 [0x2B]&2，所以这一次也要给 bit1。 */
        /* 块 0x5B0 的 A1：0x5E8 读 0x1C6 必须 bit1=1。 */
        if ((cpu_state.pc == 0x05E8) || (cpu_state.pc == 0x05E9))
            return (uint8_t) ((base & 0x3f) | 0x02);

        if ((cpu_state.pc == 0x02D6) || (cpu_state.pc == 0x02D7) ||
            (cpu_state.pc == 0x02DE) || (cpu_state.pc == 0x02DF) ||
            (cpu_state.pc == 0x02F2) || (cpu_state.pc == 0x02F3) ||
            (cpu_state.pc == 0x02FA) || (cpu_state.pc == 0x02FB))
            /* 块 0x5B0 的 A7 要求 [0x2A]&4：最后一次写 0x1C1 是 0x80(bit6=0)
               时，ISR 采样带上 bit2；其它情形仍是 0x4A。 */
            return (uint8_t) (0x4A | (dev->c1_ff ? 0x00 : 0x04));

        if ((cpu_state.pc != 0x08A7) && (cpu_state.pc != 0x08A8) &&
            (cpu_state.pc != 0x08B1) && (cpu_state.pc != 0x08B2))
            return (uint8_t) (base & 0x3f);

        /* 实测读序：rd=0 是 0x8A7 的数据读（0x84E 取它的 CF），
           rd=1 是 0x8B1 的时钟读，rd=2 起是 0x8AA。
           同步位必须落在 rd=0；之后每拍翻转 = quiesce 的半位样本。 */
        /* rd=0 是 0x84B 前的 0x8AA 时钟读；rd=1 是 0x8A0 的数据读
           （0x84E 取这一次的 CF）；rd=2 是它的时钟读（0x865 判 bit6）。 */
        /* 实测读点（反汇编）：rd0=数据读(0x8A7)，rd1=它的时钟读，
           0x86C 的 8 次交替是 rd2..9，0x882 的三个 1 是 rd10..12，
           0x88F 的三个 0 是 rd13..15，之后 0x720 从 rd16 起收 16 位。 */
        /* 0x720 的读：0x729 的 call 0x8A0 返回地址是 0x72C。
           它有自己的搜寻/收帧计数，命中同步位后重锁位边界，
           不受 0x840 前段 rd 漂移影响。 */
        {
            uint32_t ssp = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
            uint16_t r1  = (uint16_t) (mem_readb_phys(ssp) |
                                       (mem_readb_phys(ssp + 1) << 8));
            uint16_t r2  = (uint16_t) (mem_readb_phys(ssp + 2) |
                                       (mem_readb_phys(ssp + 3) << 8));
            uint16_t r3  = (uint16_t) (mem_readb_phys(ssp + 4) |
                                       (mem_readb_phys(ssp + 5) << 8));
            int is_data  = ((cpu_state.pc == 0x08A7) || (cpu_state.pc == 0x08A8));

            if ((r1 == 0x072C) || (r1 == 0x0743)) {   /* 0x729 搜寻 / 0x740 收帧 */
                /* 来源必须在**搜寻阶段**就判定：0x476 的 0x720 先跑 255 次
                   0x729，等进到 0x740 才判就晚了，前 255 读已经给出帧。 */
                if (r1 == 0x072C) {
                    dev->sub30 = (r2 == 0x0479) ? 1 : 0;
                    if (getenv("EPOCH5250_SUB30"))
                        pclog("5250: SUB30 r2=%04X -> %d cmd=%02X\n",
                              r2, dev->sub30, dev->cmdlat);
                }
                if (r1 == 0x0743) {
                    dev->fsync = 1;          /* 已进入 15 位收帧循环 */
                    /* 0x440 的两次 0x720：0x840 内部（r2=0x89D）用第 0 帧，
                       0x476 直接调用（r2=0x479）用第 1 帧。 */
                    if (r2 == 0x089D)
                        dev->sub30 = 0;
                    else if (r2 != 0)
                        dev->sub30 = 1;
                }
                if (dev->fsync && (dev->fbit >= 15)) {
                    /* 上一帧收完，0x720 又回来搜寻下一帧 */
                    if (r2 != 0x089D) {     /* 0x840 内部那一帧不占序号 */
                        dev->fseq++;
                        /* 255 帧收完：补投一次中断，让 ISR 从 0x1C1(=0xDA)
                           采到 [0x2A]，0xB03 的 (al|0xB7)+1==0 才能成立。 */
                        if ((dev->fseq == 255) && (dev->irq_raise != NULL))
                            dev->irq_raise(dev->irq_priv);
                    }
                    dev->fsync = 0;
                    dev->fhunt = 0;
                    dev->fbit  = 0;
                }
                if (!dev->fsync) {
                    /* 块 0x920 的 0x970 那次调用必须立刻给出同步位，
                       否则 0x720 的 256 次搜寻全部超时、返回 BX=0
                       （gdb 0x976 现场 BX=0000，AH=0x93）。 */
                    fb = ((dev->fhunt >= 7) || (r3 == 0x0973)) ? 1 : 0;
                    if (is_data) {
                        if (fb) {
                            dev->fsync = 1;
                            dev->fbit  = 0;
                            if (getenv("EPOCH5250_POLLOG"))
                                pclog("5250: 720 SYNC fhunt=%d rd=%d\n",
                                      dev->fhunt, dev->rd);
                        } else {
                            dev->fhunt++;
                        }
                    }
                } else if (is_data) {
                    /* 0x840 自己那次 call 0x720(返回 0x89D) 必须给 0；
                       块里 0xAE4 的调用才走递增序列 1,2,3… */
                    fb = ibm5250_ab0_frame_bit(dev, dev->fbit,
                                               /* 0x976 查 BH bit0 = B8 = 数据位1，
                                                  0x981 查 BL bit7 = B7 = 数据位0
                                                  ⇒ 数据位 7 必须为 1（data=0x80）。 */
                                               ((r2 == 0x089D) && (r3 == 0x0973))
                                                   ? 0xC0
                                                   : ((r2 == 0x089D)
                                                          ? 0
                                                          : ((dev->fseq + 1) & 0xFF)));
                    dev->flast = fb;
                    dev->fbit++;
                    if (getenv("EPOCH5250_POLLOG") && (dev->fbit >= 15))
                        pclog("5250: 720 FRAME seq=%d data=%02X r2=%04X\n",
                              dev->fseq, (r2 == 0x089D) ? 0
                                                        : ((dev->fseq + 1) & 0xFF), r2);
                } else {
                    fb = dev->flast;
                }

                dev->rd++;
                return (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
            }
        }

        if (rd <= 9)                       fb = ((rd & 1) == 0);
        else if ((rd >= 10) && (rd <= 12)) fb = 1;
        else if ((rd >= 13) && (rd <= 15)) fb = 0;
        /* 0x720：7 个空闲拍（每次 0x8A0 两拍）→ B15=1 → B14..B0。
           B3 是偶校验位：sync=1、数据/站号全 0 时它必须为 1。 */
        else if (rd == 30)                 fb = 1;
        else if ((rd >= 32) && (rd <= 61)) fb = (((rd - 32) / 2) == 11);
        else                               fb = 0;

        /* 同一拍里 0x8A7 是数据读、0x8B1 是时钟读，biphase 编码下两者互补：
           0x84E 用数据读的 CF 判“找到同步”，随后 0x865 用时钟读的 bit7
           判“线路仍为高则失败”，所以时钟读必须取反。 */
        if ((cpu_state.pc == 0x08B1) || (cpu_state.pc == 0x08B2)) {
            /* 时钟读按位格推进：
               c0..c2   0（0x865 要求线路静默）
               c3..c10  1,0,1,0…（0x86C 的 8 拍交替）
               c11..c13 1,1,1（0x882 违规正半位）
               c14..c16 0,0,0（0x88F 违规负半位）
               之后交给 0x720 的搜寻/收帧逻辑。 */
            int c = dev->clkc++;

            fb = ibm5250_proto_clk_bit(c);
        } else {
            fb = 1;              /* 数据读：一直为高，0x84E 首个循环即判到同步 */
        }

        if (dev->rd == 0)
            dev->ready = 1;
        if (getenv("EPOCH5250_POLLOG")) {
            static long an = 0; an++;
            if (an <= 200)
                {
                    uint32_t ssp = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
                    uint16_t r1  = (uint16_t) (mem_readb_phys(ssp) |
                                               (mem_readb_phys(ssp + 1) << 8));
                    uint16_t r2  = (uint16_t) (mem_readb_phys(ssp + 2) |
                                               (mem_readb_phys(ssp + 3) << 8));
                    pclog("5250: AB0 n=%ld pc=%04X rd=%d bit=%d ph=%d "
                          "r1=%04X r2=%04X\n",
                          an, cpu_state.pc, dev->rd, fb, dev->ph, r1, r2);
                }
        }
        dev->rd++;
        return (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
    }

    {
        /* 块 0x500（cmdlat=0x00）的 0x720 收帧：0x720 每个数据读就是一位，
           不能再按 rd/2 折算，否则 15 位只走到 k=7，BL.bit0（=sync）永远是 0。
           栈顶是 0x720 的返回地址：0x072C = 0x729 搜寻，0x0743 = 0x740 收帧。 */
        uint32_t sp720 = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
        uint16_t rr1   = (sp720 < 0x1000000) ? mem_readw_phys(sp720) : 0;

        if ((dev->cmdlat == 0x00) &&
            ((rr1 == 0x072C) || (rr1 == 0x0743))) {
            int fb;

            if (getenv("EPOCH5250_LOG720") && (dev->fbit < 20))
                pclog("5250: L720 r1=%04X fbit=%d rd=%d pc=%04X\n",
                      rr1, dev->fbit, dev->rd, cpu_state.pc);

            if (rr1 == 0x072C) {
                dev->fbit  = 0;          /* 新一帧的搜寻位 */
                dev->fsync = 0;
                /* 8 轮轮询里只有一轮给同步位（本站被寻址） */
                if (dev->p500_round != 1)
                    return (uint8_t) (base & 0x3f);
                return (uint8_t) ((base & 0x3f) | 0x80);
            }

            if (dev->p500_round != 1) {   /* 未被寻址：整轮无应答 */
                dev->fsync = 0;
                dev->rd++;
                return (uint8_t) (base & 0x3f);
            }

            /* 一次 call 0x8A0 = 数据读(0x8A7) + 时钟读(0x8B1)，
               只有数据读消耗一个位；时钟读重放同一位，否则 15 次
               循环会吃掉 30 位、校验位错位。 */
            if (cpu_state.pc == 0x08A8) {
                /* 0x840 已经把 sync(B15) 消费掉；0x720 的 15 次循环收的是
                   B14..B0，所以位索引从 1 开始（k=1 是数据 bit7）。 */
                fb = ibm5250_frame_bit(dev, dev->fbit + 1);
                dev->flast = (uint8_t) fb;
                dev->fbit++;
                if (getenv("EPOCH5250_LOG720") && (dev->fbit >= 15))
                    pclog("5250: L720 DONE fbit=%d BX=%04X bh=%02X bl=%02X fb=%d\n",
                          dev->fbit, BX, (unsigned) ((BX >> 8) & 0xff),
                          (unsigned) (BX & 0xff), fb);
            } else {
                fb = dev->flast;
            }
            dev->fsync = 1;
            dev->rd++;
            return (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
        }
    }

    {
        int k  = dev->rd / 2;
        int fb = ibm5250_frame_bit(dev, k);

        if (dev->rd == 0)
            dev->ready = 1;

        /* EMDIAG 的 ISR(0x2D0) 在 0x2D6 与 0x2DD 处各读一次 0x1C6 做状态采样。
           这些读不是帧位，不能推进 rd/配对。 */
        if (ISR_PC(cpu_state.pc)) {
            /* ISR 插进同一槽的两次读之间时，把配对时间戳顺延，
               否则第 2 读会被当成新槽的首读，整帧错一位。 */
            if (dev->pair_pend)
                dev->pair_tsc = tsc;
            if (getenv("EPOCH5250_PCLOG"))
                pclog("5250: ISR SAMPLE 0x1C6 -> %02X (cmdlat=%02X ph=%d rd=%d)\n",
                      (unsigned) ((base & 0x3d) | (fb ? 0x80 : 0x00)),
                      dev->cmdlat, dev->ph, dev->rd);
            /* 0x8CC `jnz` → 帧前采样 bit1=1（数据有效）；0x712 要求帧后
               采样 bit1=0 ⇒ 由 dev->ready 决定 */
            return (uint8_t) ((base & 0x3d) | (dev->ready ? 0x02 : 0x00)
                                           | (fb ? 0x80 : 0x00));
        }

        /* 读配对：0x8A0 每个位槽连读两次（同一 bit），而 ISR(0x2D0) 只单读一次。
           只在第 2 读上推 rd，单读不推进，ISR 采样就不会打乱帧边界。
           以 tsc 间隔区分：同槽两读相隔几十个周期，中断间隔远大于此。 */
        if ((dev->cmdlat == 0x64) && getenv("EPOCH5250_PAIR")) {
            int paired = dev->pair_pend
                      && ((tsc - dev->pair_tsc) <= (uint64_t) atoi(getenv("EPOCH5250_PAIRW") ? getenv("EPOCH5250_PAIRW") : "850"));

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
                pclog("5250: TRACE f=%d cmd=%02X k=%2d fb=%d rd=%2d dt=%llu at=%04X:%04X bx=%04X bl=%02X bh=%02X\n",
                      dev->fno, dev->cmdlat, k, fb, dev->rd,
                      (unsigned long long) (tsc - dev->last_tsc),
                      ((CS > 0xFFFF) ? (int) (CS >> 4) : (int) CS), cpu_state.pc,
                      BX, (unsigned) (BX & 0xff), (unsigned) ((BX >> 8) & 0xff));
                dev->last_tsc = tsc;
                n++;
            }
        }

        ret = (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
        /* 配对模式下本读已是第 2 读：rd 一次 +2，保持 k=rd/2 与 32 读阈值不变 */
        dev->rd += (((dev->cmdlat == 0x64) && getenv("EPOCH5250_PAIR")) ? 2 : 1);

        /* 帧长按调用者区分：0x840 内部调用 0x720（返回 0x89D）收 16 槽；
           0x6A0 直接调 0x720（返回 0x6DB / 0x6EC）收 15 槽。 */
        {
            uint32_t spa = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
            int caller = (spa < 0x1000000) ? mem_readw_phys(spa) : -1;
            dev->caller = caller;
            if (getenv("EPOCH5250_PCLOG"))
                pclog("5250: FRAMEDONE caller=%04X rd=%d seq=%d\n", caller, dev->rd, dev->seq64);
        }

        /* 0x720 收帧期间不能按 rd>=32 清帧：块 0x500 的 8 次调用累计读早已
           越过 32，中途清零会把 fbit 打回 0，15 位移位走不满，BL bit0 永远
           是 0（实测 scanM4：k 只到 6~7 就重开）。 */
        if ((dev->rd >= 32) && !(dev->fsync && (dev->fbit < 15))) {
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
                dev->seq64 = (dev->seq64 + 1) & 7;
                if (dev->seq64 >= 3) {
                    if (getenv("EPOCH5250_SRVRST")) pclog("5250: SRV0 pc=%04X cmd=%02X (三帧发完)\n", cpu_state.pc, dev->cmdlat);
                    dev->serve = 0;      /* 三帧发完再静默 */
                    /* 0x64 三帧结束：第二次采样 bit1=0（0x70D 要 [0x2B]&3==0） */
                    dev->ready = 0;
                    if (getenv("EPOCH5250_LEVELIRQ") && dev->irq_raise) {
                        dev->irq_level = 0;
                        dev->irq_raise(dev->irq_priv);
                        dev->irq_level = 1;
                    }
                }
            } else if ((dev->cmdlat == 0x40) || (dev->cmdlat == 0xAA)) {
                if (getenv("EPOCH5250_SRVRST")) pclog("5250: SRV0 pc=%04X cmd=%02X (40/AA)\n", cpu_state.pc, dev->cmdlat);
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

    if ((port >= 0x0100) && (port < 0x0120))
        port = (uint16_t) (0x1C0 + (port - 0x0100));

    /* HS 取证：块 0x500 的 0x8A0 数据读（pc=0x8A8）看握手表/ab0 实际返回。 */
    if (getenv("EPOCH5250_HS") && (cpu_state.pc == 0x8A8)) {
        static int hn;
        if (hn < 60) {
            pclog("5250: HS port=%03X rd=%d ab0=%d ph=%d cmd=%02X val=%02X\n",
                  port, dev->rd, dev->ab0, dev->ph, dev->cmdlat,
                  (unsigned) (((dev->ph != 2) ? ((dev->ab0 ? 0x00 : ibm5250_handshake[dev->rd % 19])) : 0) | 0));
            hn++;
        }
    }

    /* ISR 取证：EMDIAG 的 0x2D2/0x2EE 用 mov dx,[0x6] 取中断状态口。 */
    if (getenv("EPOCH5250_ISR") && ((cpu_state.pc == 0x2D7) || (cpu_state.pc == 0x2F3)))
        pclog("5250: ISR read port=%03X pc=%04X\n", port, cpu_state.pc);

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
            if (getenv("EPOCH5250_PCLOG"))
                pclog("5250: IN 0x1C1 = %02X pc=%04X (ISR=%d)\n",
                      (unsigned) (dev->latch | 0x52), cpu_state.pc,
                      (cpu_state.pc == 0x2D7));
            /* 全部 0x1C1 判据的并集：bit7|6|4|3|1 = 1、bit2 = 0 ⇒ 0xDA
               （0x4AD 要 6|4、0x616 要 7、0x683 要 2=0、0x8D3 要 6|1、0xA17 要 6|3） */
            {
                uint8_t u = (uint8_t) (dev->latch | 0xDA);
                /* 只在 A3/A4 的回读点生效：0x616 读要 bit7=1、0x62B 读要 bit7=0。
                   全局套用会破坏 ISR(0x2D6) 采样（A2 要 [0x2A]&0x80）。 */
                if ((cpu_state.pc == 0x061A) || (cpu_state.pc == 0x061B))
                    u |= 0x80;
                if ((cpu_state.pc == 0x062B) || (cpu_state.pc == 0x062C))
                    u &= 0x7F;
                /* 块 0x5B0 的 A7：ISR 采样 0x1C1 → [0x2A] 必须带 bit2。
                   最后一次写 0x1C1 是 0x80(bit6=0) 时给出该位。 */
                /* 块 0x5B0 的 A9：写 0x1C1=0x84 之后 0x687 读回 bit2 必须为 0
                   （bit2 是不可写状态位；全局屏蔽会误伤块 0x500 的 0x13）。 */
                if ((cpu_state.pc == 0x0687) || (cpu_state.pc == 0x0688))
                    u &= (uint8_t) ~0x04;
                if (((cpu_state.pc == 0x02D6) || (cpu_state.pc == 0x02D7) ||
                     (cpu_state.pc == 0x02F2) || (cpu_state.pc == 0x02F3)) && !dev->c1_ff)
                    u |= 0x04;
                if (dev->u_bit0 && (cpu_state.pc == 0x02D7)) {
                    dev->u_bit0 = 0;
                    u |= 0x01;
                }
                return u;
            }

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

    if ((port >= 0x0100) && (port < 0x0120))
        port = (uint16_t) (0x1C0 + (port - 0x0100));

    dev->lastport = (uint8_t) (port & 0xFF);
    dev->lastval  = val;

    if (getenv("EPOCH5250_LOGIO")) {
        static int n;
        if (n < 300) {
            pclog("5250: OUT %03X = %02X\n", port, val);
            n++;
        }
    }

    switch (port) {
        case 0x1C3:
            dev->c3 = val;
            /* 块 0xAB0 的 0x840 采样预算只有约 20 拍，不能先耗在 19 拍握手上：
               quiesce 紧跟在空闲之后，直接进帧阶段。 */
            if ((val == 0x08) && (dev->cmdlat == 0x00)) {
                dev->ph = 2;
                dev->rd = 0;
                if (getenv("EPOCH5250_POLLOG")) {
                    dev->samp = 40000;
                    dev->timer_armed = 1;
                    timer_set_delay_u64(&dev->timer, 1 * TIMER_USEC);
                }
            }
            if (getenv("EPOCH5250_POLLOG"))
                pclog("5250: OUT1C3 %02X pc=%04X cmd=%02X ph=%d rd=%d\n",
                      val, cpu_state.pc, dev->cmdlat, dev->ph, dev->rd);
            break;

        case 0x1C8:
            /* 数据寄存器：只存字节，不重新武装（0x6A0 先写 0xDB 再写 0xAA） */
            dev->data8 = val;
            break;

        case 0x1C9:      /* EMDIAG 的 [0x12]：写 0xAA 是“触发一帧应答”，命令号不变 */
        case 0x1CA:
            if ((val == 0x01) && (cpu_state.pc == 0x051B) && getenv("EPOCH5250_T13"))
                dev->t13n = 0;     /* 块 0x500 的收帧窗口（暂关） */
            if ((val == 0x01) && (cpu_state.pc == 0x051B) && getenv("EPOCH5250_R8")) {
                dev->r8_arm  = 1;
                dev->r8_skip = 0;
                dev->r8_cnt  = 0;  /* 从触发点开始按轮计数 */
                dev->r8b_round = 0;
            }
        case 0x1D0:
        case 0x1D1:
            if (val != 0xFF) {
                /* 不要把 val 当命令：0x6A0 的命令在 0x1C0（0x64），
                   0x1C9 只是启动这次收发。 */
                /* 命令只能由 0x1C0 设定；0x1C9/0x1CA/0x1D0/0x1D1 只是
                   触发或参数（0x6A0 末尾写 0x1CA=0x01 曾把 cmdlat 冲成 0x01）。 */

                /* 0x00（块 0xAB0）：0x840 的搜寻样本本身就是这些读，
                   不能再叠加一层 19 槽握手，否则预算在找到 quiesce 前就耗尽。 */
                dev->ab0    = ((dev->cmdlat == 0x00) && dev->ab0) ? 1 : 0;
                dev->ph     = dev->ab0 ? 2 : 1;
                dev->rd     = 0;
                dev->serve  = 1;
                dev->pre    = ((dev->cmdlat == 0x40) && getenv("EPOCH5250_PRE"))
                              ? atoi(getenv("EPOCH5250_PRE")) : 0;
                /* 协议：数据就绪中断在“触发”后一个响应窗口才来（twinax 30–80 µs） */
                if ((dev->cmdlat == 0x64) && (port == 0x1C9) && getenv("EPOCH5250_LEVELIRQ")) {
                    dev->ready       = 1;
                    dev->irq_level   = 0;
                    dev->timer_armed = 1;
                    timer_set_delay_u64(&dev->timer, 60 * TIMER_USEC);
                }
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
            if (getenv("EPOCH5250_P5R") && (val == 0x08)) {
                static int e5;
                if (e5++ < 24)
                    pclog("5250: W1C0 08 pc=%04X round=%d on=%d tsc=%08X\n",
                          cpu_state.pc, dev->p500_round, dev->p500_on, (unsigned) tsc);
            }
            /* 块 0x500 的第一条写（0x505 out dx,0x08）：记录入口状态 */
            if (getenv("EPOCH5250_ENTRY") && (cpu_state.pc == 0x0508)) {
                uint32_t dsa = ((uint32_t) cpu_state.seg_ds.seg << 4);
                pclog("5250: ENTRY ph=%d rd=%d ab0=%d fsync=%d fbit=%d cmd=%02X [27]=%02X [28]=%02X\n",
                      dev->ph, dev->rd, dev->ab0, dev->fsync, dev->fbit, dev->cmdlat,
                      mem_readb_phys(dsa + 0x27), mem_readb_phys(dsa + 0x28));
            }
            if (dev->r8_arm) {
                dev->r8_cnt++;
                /* 每轮开始（22 次写）都把相位重置回握手态 */
                if ((dev->r8_cnt % 22) == 1) {
                    if (getenv("EPOCH5250_PHSRC")) pclog("5250: PHSRC->1 pc=%04X lp=%02X lv=%02X\n", cpu_state.pc, dev->lastport, dev->lastval);
                    dev->ph   = 1;
                    dev->rd   = 0;
                    dev->fsync = 0;
                    dev->fbit = 0;
                }
            }
            pclog("5250: CMDW %02X pc=%04X\n", val, cpu_state.pc);
            /* 协议表：每个测试块都先写一条命令（0x00/0x08/0x10/0x40/0x64/0xAA…），
               之前只放行 0x40/0x08/0x64，导致 0x7A0 起步就被忽略。 */
            if (val != 0xFF) {
                /* 只有块 0x7A0 自己的 0x1C0 写（pc≈0x7C6）才保留 0x8C0 链的
                   中断窗口标记；别的块一写命令就清掉，避免轮询串块。 */
                if ((cpu_state.pc < 0x07C0) || (cpu_state.pc > 0x07C8))
                    dev->win_irq = 0;
                /* 相位黏着（实验）：已经进入帧相位时，位脉冲写不得把设备
                   打回握手态，否则 0xCA0 每轮的 22 次 0x1C0 写会让
                   ph 永远停在 1，0x720 拿不到任何帧。 */
                if (getenv("EPOCH5250_PHSTICK")
                    && (cpu_state.pc >= 0x0CA0) && (cpu_state.pc <= 0x0D20)) {
                    dev->serve = 1;
                    break;
                }
                /* keep the 0x40 reply in place while the subtest is running */
                /* 0x6A0 自己 out 0x1C0=0x64，块内 0xCA0 线上发的是 0x61(=0x30)，
                   但 0x840 必须按 0x64 选帧表。p64 是粘滞标志，直到下一块
                   写自己的 0x1C0 值才清掉。 */
                dev->p64 = (val == 0x64) ? 1 : 0;
                int hold = (val == 0x08) && (dev->cmdlat == 0x40) && !dev->done40;

                if (!hold) {
                    dev->cmdlat = val;
                    if (getenv("EPOCH5250_PHSRC")) pclog("5250: PHSRC->1 pc=%04X lp=%02X lv=%02X\n", cpu_state.pc, dev->lastport, dev->lastval);
                    dev->ph     = 1;
                    dev->rd     = 0;
                    dev->txpos  = 0;      /* 新一轮起始序列从半位 0 开始 */
                    /* 0x720 的收帧计数器必须跟着每轮命令一起复位，
                       否则上一轮停在 fbit=15，下一轮搜寻位从 0 位重新开始时错位。 */
                    if ((val == 0x08) && ((cpu_state.pc == 0x0508) || (cpu_state.pc == 0x0505))) {
                        dev->p500_round = 0;   /* 块 0x500 入口：轮询计数归零 */
                        dev->p500_on    = 1;
                    }
                    if ((val != 0x00) && (val != 0x08))
                        dev->p500_on    = 0;   /* 离开 0x500：别影响后续块 */
                    dev->fbit   = 0;
                    dev->fsync  = 0;
                    /* 块 0x920 的命令写在 pc=0x0927；它要自己走一遍
                       0x840，必须从干净序列开始（实测宽条件会误伤
                       0x7A0 的 0x7C6）。 */
                    if ((val == 0x00) && (cpu_state.pc == 0x0927)) {
                        dev->fhunt = 0;
                        dev->fsync = 0;
                        dev->fbit  = 0;
                        dev->flast = 0;
                        dev->c1_ff = 1;
    dev->c1_cnt = 0;
    dev->a7_pend = 0;
    dev->clkc  = 0;
                        dev->fseq  = 0;
                        dev->ab0   = 1;
                        /* 必须同时把相位推进到 2：ph!=2 的读一律返回
                           base&0x3f（bit7=0）且不推进 clkc，0x840 的 8 次
                           同步搜寻会全部落空（实测 RD 日志 rd 0..18、
                           clkc 恒 0、随后 OUT1B0 00/01）。 */
                        dev->ph    = 2;
                    }
                    dev->serve  = 1;
                    dev->irq_burst = 0;
                    dev->pre    = ((val == 0x40) && getenv("EPOCH5250_PRE"))
                                  ? atoi(getenv("EPOCH5250_PRE")) : 0;
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
                    dev->w42     = 0;   /* 新命令：清掉上一块遗留的 1C1=42 */
                    dev->latch   = 0;   /* 上一块末尾的 1C1=0xFF 会把 bit1 掩掉 */
                    dev->bit0    = 0;
                    pclog("5250: ARM %02X (frame %d)\n", val, dev->fno);
                } else {
                    pclog("5250: ARM %02X ignored (0x40 pending)\n", val);
                }
            }
            /* 块 0x7A0 在 0x7C6 写 1C0=0x00，随后 0x7F7 调 0x8C0 检查
               [0x2B]&2（要 bit1=1）——按同一响应窗口投一次中断 */
            if ((val == 0x00) && (cpu_state.pc == 0x7C6) &&
                getenv("EPOCH5250_LEVELIRQ") && dev->irq_raise) {
                dev->win_irq     = 1;
                dev->ready       = 1;
                /* 上一次（0x64）的中断可能还挂在 8259 上：先真正拉低，
                   否则再拉高没有边沿，ISR 不会进。 */
                if (dev->irq_clear != NULL)
                    dev->irq_clear(dev->irq_clear_priv);
                dev->irq_level   = 0;
                dev->poll_left   = 2000;   /* 最多等 2 ms */
                dev->timer_armed = 1;
                timer_set_delay_u64(&dev->timer, 1 * TIMER_USEC);
            }

            ibm5250_arm_irq(dev);
            break;

        case 0x1C1:
            /* 块 0x5B0 的 A3/A4：写 0xC0 后读回 bit7=1、写 0x80 后读回 bit7=0
               ⇒ bit6 是 bit7 回读的“置位/清除”控制。 */
            dev->c1_ff = ((val & 0x40) != 0);
            /* 块 0x5B0：0x602 写 C0 起一轮，循环里每轮写一次 80（共 15 轮）。
               第 15 次写之后客人先查 A5/A6（[0x2A]==0），再 call 0xCA0 清、
               然后 call 0xB90 延时约 1ms 才查 A7（[0x2A]&4）。
               所以中断必须延迟到 0xCA0 之后，不能在写点立刻拉。 */
            if (val == 0xC0)
                dev->c1_cnt = 0;
            else if (val == 0x80) {
                if (++dev->c1_cnt >= 15) {
                    dev->c1_cnt  = 0;
                    dev->a7_pend = 1;
                    dev->timer_armed = 1;
                    timer_set_delay_u64(&dev->timer, 200 * TIMER_USEC);
                }
            }
            dev->latch = (uint8_t) (val | 0x42);
            /* 任何一次 0x1C1 写入都会让下一次 1C2=04 的 bit1 变成 0：
               0x8F6 写 0x42、0xA2B 写 0x48 都是这个效果（0x904 / 0xA48 两条判据）。 */
            dev->w42 = 1;
            /* 0x6A0 末尾：写 0x1C1=0xFF 之后要求 (0x1C6 & 3) == 0
               ⇒ 视为复位/静默，停止继续供帧，0x1C6 读回 0 */
            if (val == 0xFF) {
                if (getenv("EPOCH5250_SRVRST")) pclog("5250: SRV0 pc=%04X cmd=%02X (1C1=FF)\n", cpu_state.pc, dev->cmdlat);
                dev->serve = 0;
                dev->rd    = 0;
                dev->ph    = 0;
                dev->seq64 = 0;
            }
            break;

        case 0x1C2:
            if (getenv("EPOCH5250_POLLOG")) {
                int nz = 0, k, first = -1;
                uint32_t dsa = ((uint32_t) cpu_state.seg_ds.seg << 4);
                int off24 = (int) (mem_readb_phys(dsa + 0x24)
                                   | (mem_readb_phys(dsa + 0x25) << 8));
                for (k = 0; k < 256; k++) {
                    if (mem_readb_phys(0xE4000 + off24 + k) != 0) {
                        nz++;
                        if (first < 0) first = k;
                    }
                }
                pclog("5250: OUT1C2 %02X pc=%04X [24]=%04X nz=%d first=%d v=%02X%02X%02X%02X %02X%02X%02X%02X\n",
                      val, cpu_state.pc, off24, nz, first,
                      mem_readb_phys(0xE4000), mem_readb_phys(0xE4001),
                      mem_readb_phys(0xE4002), mem_readb_phys(0xE4003),
                      mem_readb_phys(0xE4004), mem_readb_phys(0xE4005),
                      mem_readb_phys(0xE4006), mem_readb_phys(0xE4007));
            }
            /* 每块开头都重开起始序列：0xAB0 写 08、0xBA0 写 0A，
               否则第二块会接着上一块的 rd 继续，序列整体错位。 */
            /* 只在进入该块时重置一次；块内还会多次写 0x1C2（04/09 等），
               每次都重置会把序列在第 13 拍反复打断。 */
            /* 块 0x5B0：0x5B7 写 1C2=10、0x5D4 写 1C2=00，随后 0x5E9 要求
               0x1C6 bit1=1（否则 AH=0xA1）。 */
            if ((val == 0x00) && ((cpu_state.pc == 0x05D4) || (cpu_state.pc == 0x05D5) ||
                                  (cpu_state.pc == 0x05D6))) {
                dev->ready = 1;
                dev->ph    = 2;
                /* 块 0x5B0 的 A2（0x5F2）要求 [0x2A]&0x80，而 [0x2A] 只能由
                   ISR 采样 0x1C1 写回；紧随其后的 0x5DD call 0xCA0 会先清 [0x2A]。
                   在这个写点后 ~150µs 投一次中断：落在 0xCA0 之后（清已完成）、
                   0x5F2 判据之前。A1 的 0x1C6 读点不受影响（ISR 读不消耗位）。 */
                if (!dev->a7_pend && (dev->irq_raise != NULL) &&
                    getenv("EPOCH5250_LEVELIRQ")) {
                    dev->a7_pend     = 1;
                    dev->timer_armed = 1;
                    timer_set_delay_u64(&dev->timer, 150 * TIMER_USEC);
                }
            }

            /* 块 0x920 第二阶段：0x999 写 1C2=04 之后紧跟 0x9A2 call 0x840，
               此时 rd 已经走到 51，不重开起始序列就必然 AH=01。 */
            if ((val == 0x04) && ((cpu_state.pc == 0x099A) || (cpu_state.pc == 0x0999) ||
                                  (cpu_state.pc == 0x099B))) {
                dev->rd    = 0;
                dev->clkc  = 0;
                dev->fhunt = 0;
                dev->fsync = 0;
                dev->fbit  = 0;
                dev->flast = 0;
                dev->ph    = 2;
            }

            if ((dev->cmdlat == 0x00) && !dev->ab0) {
                dev->ab0   = 1;
                dev->ph    = 2;
                dev->rd    = 0;
                dev->fhunt = 0;
                dev->fsync = 0;
                dev->fbit  = 0;
                dev->flast = 0;
                dev->clkc  = 0;
                dev->fseq  = 0;
            }

            /* 块 0xBA0 在 0xBA5 写 0x1C2=0x0A，随后 0xBB6 读 0x1C6
               要求 bit1=1；而且它自己要走一遍 0x840（0xBBF 的 call），
               上一块已把 rd 用到 33，必须在这里重开序列。 */
            if (((val & 0x0F) == 0x0A) && (dev->cmdlat == 0x00)) {
                dev->rd      = 0;
                dev->txpos   = 0;
                dev->p500_mark = 0;
                dev->fhunt   = 0;
                dev->fsync   = 0;
                dev->fbit    = 0;
                dev->flast   = 0;
                dev->clkc    = 0;
                dev->fseq    = 0;
                dev->b1_pulse = 1;
            }
            /* 块 0xBA0 的 0x84 判据（0xBED）要求 [0x2A] 同时带 0x40|0x08，而
               [0x2A] 只能由 ISR 采样 0x1C1 写回；0xCA0 的 0xC40 又会把它清零。
               在 0x1C2=0x0A 写点后 ~300µs 投一次延迟中断：落点必在 0xC40 之后、
               0xBED 之前。注意不能用 cmdlat==0x00 门控——0xAB0 的帧会把 cmdlat
               解成 0x10。 */
            if ((val == 0x0A) && (cpu_state.pc >= 0x0BA0) && (cpu_state.pc <= 0x0BB0) &&
                (dev->irq_raise != NULL) && getenv("EPOCH5250_LEVELIRQ")) {
                dev->a7_pend     = 1;
                dev->timer_armed = 1;
                if (getenv("EPOCH5250_PCLOG"))
                    pclog("5250: BA0 arm irq 300us pc=%04X lvl=%d cmd=%02X\n",
                          cpu_state.pc, dev->irq_level, dev->cmdlat);
                timer_set_delay_u64(&dev->timer, 300 * TIMER_USEC);
            }
            if (val & 0x04) {
                dev->phase = dev->w42 ? 0 : 1;
                dev->w42   = 0;
            }
            /* 选择器：带 bit0（0x01）的写（如 0x09）→ bit1=1；否则 bit1=phase */

            /* 0x09 = 选中适配器共享 RAM 窗口：块 0x9D0 写完立刻就要求
               0xE400:0000..00FF 为 0（0xA5B），而它自己从不写这块内存。 */
            if (((val & 0x0F) == 0x09) && (dev->shared_clear != NULL))
                dev->shared_clear(dev->shared_priv);

            {
                const ibm5250_sel_t *sel = NULL;
                for (size_t si = 0; si < sizeof(ibm5250_sels) / sizeof(ibm5250_sels[0]); si++) {
                    if (ibm5250_sels[si].code == (val & 0x0F)) {
                        sel = &ibm5250_sels[si];
                        break;
                    }
                }
                if (sel != NULL) {
                    dev->b1   = (sel->b1 == SEL_PHASE) ? dev->phase : sel->b1;
                    if (sel->b0 != SEL_KEEP)
                        dev->bit0 = sel->b0;
                } else {
                    dev->b1 = dev->phase;
                }
            }

            /* 必须放在选择器之后：0x0A 不在表里，上面的 else 会用 phase
               覆盖。块 0xBA0 在 0xBA5（取指后报 0xBA8）写 0x0A，随后
               0xBB6 读 0x1C6 要求 bit1=1。 */
            if ((val == 0x0A) && (cpu_state.pc >= 0x0BA0) && (cpu_state.pc <= 0x0BB0))
                dev->b1 = 1;

            /* 块 0x7A0 每次写 0x1C2 之后都会进 0x8C0：每次都重新武装
               中断窗口，否则第二次调用没有中断、[0x2B] 被 0xCA0 清掉。
               注意 0xCA0 发帧会把 cmdlat 覆写成帧里的命令字节（0x1203→0x01），
               所以这里不能只看 cmdlat==0x00，要看块 0x7A0 的窗口标记。 */
            if (((cpu_state.pc >= 0x07C0) && (cpu_state.pc <= 0x0812)) ||
                ((cpu_state.pc >= 0x0B6F) && (cpu_state.pc <= 0x0B95)))
                dev->win_irq = 1;
            else
                dev->win_irq = 0;   /* 离开 0x8C0 的两个调用块：别把窗口轮询带进下一块 */
            if (dev->win_irq && getenv("EPOCH5250_LEVELIRQ") &&
                dev->irq_raise) {
                dev->ready = 1;
                if (dev->irq_clear != NULL)
                    dev->irq_clear(dev->irq_clear_priv);
                dev->irq_level = 0;
                dev->poll_left = 2000;
                dev->timer_armed = 1;
                timer_set_delay_u64(&dev->timer, 1 * TIMER_USEC);
            }
            break;

        case 0x1C6:
            /* 只有 0xCA0 的命令帧发送（0xC70 循环 call 0xA70/0xA7E）才推进
               帧解码；0x840 读期间的位时钟（call 0x8A0 → 0xA73）不算数据。 */
            {
            uint32_t sp1c6 = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
            uint16_t rc1c6 = (sp1c6 < 0x1000000) ? mem_readw_phys(sp1c6 + 2) : 0;
            /* 镜像等长探针：patch_probe2b.py 把 0x8C7 改成
               mov al,[0x2B]; out dx,al; nop nop，out 之后 pc=0x8CC。
               这里把客人真实的 [0x2B] 打出来。 */
            /* 0x8C0 每次被调用都会先走 0xC40 的起始序列（5 个 biphase 1，
               首拍 pc=0x0A91 val=0x18 rc1=0x0A73）。数这些事件就能知道
               0x8C0 一共被调几次、每次在哪条命令（cmdlat）下。 */
            if (getenv("EPOCH5250_W1C6") && dev->p500_on) {
                static int w5;
                if (w5++ < 70)
                    pclog("5250: W1C6 val=%02X pc=%04X rc=%04X pcnt=%d hist=%04X\n",
                          val, cpu_state.pc, rc1c6, dev->pcnt, dev->hist);
            }
            /* 0x500 组的第二轮可能跳过 0x1C0 入口直接进 0x547，p500_on 因此
               为 0、serve 永远不置 -> 计数 0 -> AH=0x13。这里按调用链识别：
               C40 起始拍时栈上第 4 个字是 0x054D（0x54A call 0xCA0 的返址）。 */
            if ((rc1c6 == 0x0A73) && (sp1c6 < 0x1000000)) {
                uint16_t r6 = mem_readw_phys(sp1c6 + 8);
                if (r6 == 0x054D) {
                    if (!dev->p500_on || (dev->p500_round > 8)) {
                        dev->p500_on    = 1;
                        dev->p500_round = 0;
                        dev->txpos      = 0;
                    }
                }
            }
            if (getenv("EPOCH5250_P5R") && (rc1c6 == 0x0A73)) {
                static int s7;
                if ((val & 0x10) && !dev->p5hi && (sp1c6 < 0x1000000) && (s7++ < 30))
                    pclog("5250: STK5 on=%d 2=%04X 4=%04X 6=%04X 8=%04X A=%04X C=%04X\n",
                          dev->p500_on, mem_readw_phys(sp1c6+2), mem_readw_phys(sp1c6+4),
                          mem_readw_phys(sp1c6+6), mem_readw_phys(sp1c6+8),
                          mem_readw_phys(sp1c6+10), mem_readw_phys(sp1c6+12));
                dev->p5hi = (val & 0x10) ? 1 : 0;
            }
            if (getenv("EPOCH5250_P5R") && (rc1c6 == 0x0A73)) {
                static int s6;
                if ((val & 0x10) && !dev->p5hi) {
                    uint32_t d6 = ((uint32_t) cpu_state.seg_ds.seg << 4);
                    if (s6++ < 40)
                        pclog("5250: SEND pc=%04X val=%02X on=%d [28]=%02X [29]=%02X [2F]=%02X\n",
                              cpu_state.pc, val, dev->p500_on, mem_readb_phys(d6 + 0x28),
                              mem_readb_phys(d6 + 0x29), mem_readb_phys(d6 + 0x2F));
                }
                dev->p5hi = (val & 0x10) ? 1 : 0;
            }
            if (getenv("EPOCH5250_C40N") && (rc1c6 == 0x0A73) && (val == 0x18)) {
                static int c40n;
                uint16_t rc4x = (sp1c6 < 0x1000000) ? mem_readw_phys(sp1c6 + 4) : 0;
                uint16_t rc6x = (sp1c6 < 0x1000000) ? mem_readw_phys(sp1c6 + 6) : 0;
                pclog("5250: C40N #%d cmdlat=%02X pc=%04X serve=%d seq=%d rc4=%04X rc6=%04X\n",
                      ++c40n, dev->cmdlat, cpu_state.pc, dev->serve, dev->seq64, rc4x, rc6x);
            }
            if (getenv("EPOCH5250_W2B") && (cpu_state.pc == 0x08CC))
                pclog("5250: W2B val=%02X cmdlat=%02X serve=%d seq=%d\n",
                      val, dev->cmdlat, dev->serve, dev->seq64);
            if (getenv("EPOCH5250_W2B") && (cpu_state.pc == 0x08CF))
                pclog("5250: W2BE val=%02X cmdlat=%02X serve=%d seq=%d\n",
                      val, dev->cmdlat, dev->serve, dev->seq64);
            if (getenv("EPOCH5250_TOUT") && (dev->cmdlat == 0x64)) {
                static int to;
                if (to++ < 240)
                    pclog("5250: TOUT pc=%04X val=%02X rc1=%04X txr=%d\n",
                          cpu_state.pc, val, rc1c6, dev->tx_reset);
            }
            uint16_t rc4   = (sp1c6 < 0x1000000) ? mem_readw_phys(sp1c6 + 4) : 0;
            uint16_t rc6   = (sp1c6 < 0x1000000) ? mem_readw_phys(sp1c6 + 6) : 0;
            uint16_t rc8   = (sp1c6 < 0x1000000) ? mem_readw_phys(sp1c6 + 8) : 0;
            uint16_t rcA   = (sp1c6 < 0x1000000) ? mem_readw_phys(sp1c6 + 10) : 0;
            /* 只有 0x840/0x720 的“读”会经 0x8A0 调 0xA73（返回地址
               0x8A3/0x8AD/0x8B4）；其余写脉冲都是 0xCA0 的发送。 */
            int from_tx = (rc1c6 != 0x08A3) && (rc1c6 != 0x08AD) &&
                          (rc1c6 != 0x08B4) && (rc1c6 != 0x0B95);

            if (getenv("EPOCH5250_TXLOG") && from_tx) {
                static int tn;
                if (tn < 400)
                    pclog("5250: TXLOG %03d val=%02X rc1=%04X hist=%03X inf=%d pcnt=%d word=%04X cmd=%02X\n",
                          tn++, val, rc1c6, dev->hist, dev->inframe, dev->pcnt,
                          dev->word, dev->cmdlat);
            }

            /* 块 0x920：0x93E call 0xC40 之后紧跟 16 位发送（0xC70），0x944 sti，
               0x952 检查 [0x2A]&1。在起始序列（val 是 [0x31]|0x10，此块为 0x10，
               不是 0x7A0 的 0x18）任一拍就置"发送完成"事件位并拉线：中断会等到
               0x944 的 sti 才进，ISR 采样 0x1C1 就带 bit0。栈返回地址 0x0941/
               0x0944 锁定块，避免影响其它同样调 0xC40 的路径。 */
            if ((rc4 == 0x0941) || (rc6 == 0x0941) || (rc8 == 0x0941) ||
                (rc4 == 0x0944) || (rc6 == 0x0944) || (rc8 == 0x0944)) {
                /* 每一拍都补事件位（0x720 收帧也会置它，会被 ISR 采样清掉）；
                   线只武装一次，避免把 30µs 定时器反复推后。 */
                dev->u_bit0 = 1;
                if (!dev->a7_pend && (dev->irq_raise != NULL) &&
                    getenv("EPOCH5250_LEVELIRQ")) {
                    dev->a7_pend     = 1;
                    dev->timer_armed = 1;
                    timer_set_delay_u64(&dev->timer, 30 * TIMER_USEC);
                }
            }

            /* 0xC40 起始序列首拍（val=0x18 即 [0x31]|0x10，调用点 0x0A73）：
               每次 0xCA0 发送的协议边界，帧解码器必须在这里重新对齐，否则
               上一块 19 拍握手的 hist/pcnt 残留会把命令字节解成噪声。 */
            if ((val == 0x18) && (rc1c6 == 0x0A73) && !dev->last18) {
                dev->hist    = 0;
                dev->inframe = 0;
                dev->pcnt    = 0;
                dev->word    = 0;
            }
            dev->last18 = (val == 0x18) ? 1 : 0;

            /* 0xCA0 在每轮轮询前先发前导+命令帧；上一轮 0x840/0x720 已经把
               txpos 推到 19 以上，所以这里就是协议上的轮次边界。 */
            if ((val & 0x40) && from_tx) {
                int d = (val >> 4) & 1;

                dev->hist = ((dev->hist << 1) | (uint32_t) d) & 0x3FF;
                if (!dev->inframe) {
                    if (dev->hist == 0x2AA) {
                        dev->inframe = 1;
                        dev->pcnt    = 0;
                        dev->word    = 0;
                        if (dev->p500_on) {
                            /* 0xCA0 每次轮询先发前导，再发命令帧：这是新一轮的
                               唯一协议边界（块内不写 0x1C0）。 */
                            dev->txpos = 0;
                            dev->p500_round++;
                        }
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
                        /* 0x580 之后要求 [0x30]==7：成功轮次必须是第 8 轮
                           （0x568 把 ch 写进 [0x30]，0x586 与 7 比较）。
                           实测按 word==0x1221 应答落在第 2 轮（[0x30]=01）。 */
                        if (dev->p500_on)
                            dev->serve = (dev->word == 0x1E21);   /* 第 8 轮：0x580 要求 [0x30]==7 */
                        else if (getenv("EPOCH5250_NODS"))
                            dev->serve = (dev->word == 0x1221);
                        else {
                            uint32_t dsa = ((uint32_t) cpu_state.seg_ds.seg << 4) + 0x2F;
                            dev->serve = (dsa < 0x1000000)
                                       ? ((mem_readb_phys(dsa) == 1) ? (dev->word == 0x1221) : 1)
                                       : 1;
                        }
                        dev->p5resp = dev->p500_on && (dev->word == 0x1E21);
                        if (getenv("EPOCH5250_P5R") && dev->p500_on)
                            { uint32_t d4 = ((uint32_t) cpu_state.seg_ds.seg << 4);
                              pclog("5250: P5RESP %d word=%04X [26]=%04X [28]=%02X [29]=%02X\n",
                                    dev->p5resp, dev->word, mem_readw_phys(d4 + 0x26),
                                    mem_readb_phys(d4 + 0x28), mem_readb_phys(d4 + 0x29)); }
                        if (getenv("EPOCH5250_W1C6") && dev->p500_on) {
                            uint32_t ds2 = ((uint32_t) cpu_state.seg_ds.seg << 4) + 0x2F;
                            pclog("5250: P5W word=%04X serve=%d [2F]=%02X DS=%04X ds=%04X cmdlat=%02X round=%d\n",
                                  dev->word, dev->serve,
                                  (ds2 < 0x1000000) ? mem_readb_phys(ds2) : 0xFF,
                                  cpu_state.seg_ds.seg, (unsigned) ds2, dev->cmdlat, dev->p500_round);
                        }
                        dev->rx_word = dev->word;
                        dev->respond = dev->p500_on;
                        dev->tx_reset = 1;
                        /* 0xCA0 发的帧里低字节就是命令字节（0x500 是 0x21，
                           0x440 是 0x61）。块内不写 0x1C0，cmdlat 只能从这里来，
                           否则 0x840 内部的 0x720 会拿上一块的命令选错帧表。
                           0x500 自己靠 0x1C0 的轮询语义，不动它的 cmdlat。 */
                        if (!dev->p500_on && !dev->p64 && (dev->word & 1u)) {
                            /* nat-1992 3.1.1.3：word 按接收顺序组装，bit0=B15(sync)，
                               所以命令字节在 (word>>1)&0xFF，不是 word&0xFF。 */
                            uint8_t cmd = (uint8_t) ((dev->word >> 1) & 0xFF);
                            if (cmd != dev->cmdlat) {
                                if (getenv("EPOCH5250_CMDSYNC"))
                                    pclog("5250: CMDSYNC %02X -> %02X (word=%04X)\n",
                                          dev->cmdlat, cmd, dev->word);
                                dev->cmdlat = cmd;
                            }
                        }
                        pclog("5250: FRAME %04X serve=%d station=%d my=%d data=%02X\n",
                              dev->word, dev->serve, (dev->word >> 9) & 7,
                              dev->my_station, (dev->word >> 1) & 0xFF);
                    }
                }
            }
            }
            break;

        default:
            break;
    }
}

/* 块 0xAB0 正在跑 0x840 的起始序列/收帧：此时不允许任何 IRQ 打断
   （ISR 会覆盖 0x882 循环的 AL/CF，导致直接走 0x854 失败）。 */
int
ibm5250_in_ab0(ibm5250_t *dev)
{
    return (dev != NULL) && dev->ab0 && (dev->cmdlat == 0x00);
}

void
ibm5250_reset(ibm5250_t *dev)
{
    dev->latch    = 0x00;
    dev->cmdlat   = 0x00;
    dev->phase    = 1;
    dev->b1       = 1;
    if (getenv("EPOCH5250_PHSRC")) pclog("5250: PHSRC->1 pc=%04X lp=%02X lv=%02X\n", cpu_state.pc, dev->lastport, dev->lastval);
    dev->ph       = 1;
    dev->rd       = 0;
    dev->p500_round = 0;
    dev->p500_mark  = 0;
    dev->p500_on    = 0;
    dev->txpos      = 0;
    dev->respond    = 0;
    dev->rx_word    = 0;
    dev->my_station = getenv("EPOCH5250_STATION") ? atoi(getenv("EPOCH5250_STATION")) : 7;
    dev->tx_reset   = 0;
    dev->serve    = 1;
    dev->done40   = 0;
    dev->fno      = 0;
    dev->u_bit0   = 0;
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

    {
        const char *eb = getenv("EPOCH5250_BASE");
        dev->base = eb ? (uint16_t) strtol(eb, NULL, 0) : 0x1C0;
    }
    dev->irq       = irq;
    dev->irq_raise = irq_raise;
    dev->irq_priv  = irq_priv;
    dev->irq_clear = irq_clear;
    dev->irq_clear_priv = irq_clear_priv;

    ibm5250_reset(dev);

    timer_add(&dev->timer, ibm5250_irq_cb, dev, 0);

    for (int i = 0; i < (int) (sizeof(ibm5250_ports) / sizeof(ibm5250_ports[0])); i++)
        io_sethandler((uint16_t) (dev->base + (ibm5250_ports[i] - 0x1C0)), 0x0001,
                      ibm5250_in, NULL, NULL, ibm5250_out, NULL, NULL, dev);

    pclog("5250: adapter initialised (irq=%d, %d ports)\n",
          irq, (int) (sizeof(ibm5250_ports) / sizeof(ibm5250_ports[0])));

    return dev;
}

void
ibm5250_set_shared_clear(ibm5250_t *dev, void (*fn)(void *priv), void *priv)
{
    dev->shared_clear = fn;
    dev->shared_priv  = priv;
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
