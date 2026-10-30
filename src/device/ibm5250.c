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
 *          Parity is even over the 12 bits B15..B4 for every command, as the
 *          protocol requires (audit item: the earlier odd-parity fit for 0x40
 *          is gone). The payloads are d=0x2A for 0x40 and d=0xFF for 0x08,
 *          both with station 111 (7) = EOM on single-frame replies.
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

/* 适配器有两个寄存器窗口：0x1C0 与 0x100（偏移相同）。
   EMDIAG 的 0x500 第二遍把基址切到 0x100。 */
#define IBM5250_ALT_BASE  0x0100
#define IBM5250_ALT_DELTA 0x00C0

static uint16_t
ibm5250_norm(uint16_t port)
{
    if ((port >= IBM5250_ALT_BASE) && (port < (IBM5250_ALT_BASE + 0x20)))
        return (uint16_t) (port + IBM5250_ALT_DELTA);
    return port;
}

/* 临时探针总开关：默认关闭，pclog 同步写会拖慢时序 */
#define TRACE_ON (getenv("EPOCH5250_TRACE") != NULL)

/* 19-read handshake, measured on the real EMDIAG/0x840 sequence.  idx 18 must
   stay 0: giving it the start bit breaks the 0x500 subtest (AH=0x13). */
/* 0x1C1 状态位：客机判据的并集，逐位都有出处（不是猜测的硬件语义）
   bit7 = 本帧服务完成  —— 写 0x1C1=0x80 清（0x62B 判 0）、帧发完置 1（0x616 判 1）
                          独立字段 dev->c1_b7，不参与强制掩码
   bit6 = 适配器签名    —— 0x4AD 的 (al|0xAF)+1==0、0x8D3 的 (al|0xBD)+1==0 都要求它
   bit4 = 适配器签名    —— 同上 0x4AD
   bit3 = 命令就绪      —— 0xA17（0x1C1 read 后 & 8）
   bit1 = 本机被轮询    —— 0x8D3 要求 bit6|bit1；各块 READY 读法同上
   bit2 = 恒 0          —— 0x683 判 bit2=0
   逐位出处见 references/3477s-emdiag-protocol-table.md；这里只做并集常量。 */
#define IBMR_ST_SIGNATURE  (0x40 | 0x10 | 0x08 | 0x02)
/* 写 0x1C1 时锁存进 latch 的位：bit6/bit4 会被 0x1C6 的 base()（0x7C 掩码）读到，
   所以必须保留；bit1 在 latch 里没有任何读者（base() 的掩码已排除它，读 0x1C1
   时由 IBMR_ST_SIGNATURE 补上），因此不再写进去 —— 之前 val|0x42 多置的正是它。 */
#define IBMR_LATCH_SIG     (0x40 | 0x10)




/* 写侧识别点：同一件事由多个 EMDIAG 写指令地址触发，收成集合判据。 */
static int
ibm5250_pc_in(uint16_t pc, const uint16_t *set, unsigned n)
{
    unsigned i;

    for (i = 0; i < n; i++)
        if (set[i] == pc)
            return 1;
    return 0;
}

/* 写 0x1C2=0x04 选择相位、随后立刻读 0x1C6 要求 bit1=0 的判据点：
   0xA41(0x9D0) / 0xB3A(0xAB0) / 0xC1B(0xBA0)。这些点要结束供帧，
   否则读回的是响应帧位，bit1 由帧内容决定。 */
static const uint16_t ibm_pc_sel4[] = { 0x0A41, 0x0B3A, 0x0C1B, 0x0675 };

/* 0x1C6 的读点分类：数据读 / 时钟读 / 其它。
   ★ 端口层不可判：0x8A0 与 0x8AA 都是“写 3 次 0x1C6 → in 0x1C6”，端口地址、
   前导写序列、返回值来源完全相同；0x840 的调用序列还是 D C C D C（不是严格交替），
   所以读序号、写计数都替代不了指令地址。要改只能上指令层挂钩，那本质上仍是
   “认代码地址”，并没有变成协议判据。详见 NOTES-5250-0x500.md。 */
enum { IBM_RD_NONE = 0, IBM_RD_DATA = 1, IBM_RD_CLOCK = 2 };
static int
ibm5250_read_kind(uint16_t pc)
{
    if ((pc == 0x08A7) || (pc == 0x08A8))
        return 1;
    if ((pc == 0x08B1) || (pc == 0x08B2))
        return 2;
    return 0;
}

/* 0x1C6 的“发送位”写点（0xA73 里的三次 out 中的第一次）。
   ★ 同样不可由端口层区分：0xB90 的 54 拍时钟与 0xCA0 的 16 位发送都是
   “call 0xA73” 的重复，写序列 v / v|0x40 / v 完全同构，只有次数不同。 */
static int
ibm5250_is_tx_wr_pc(uint16_t pc)
{
    return (pc == 0x0A91) || (pc == 0x0A9A) || (pc == 0x0AA3);
}

static const uint8_t ibm5250_handshake[19] = {
    0x00, 0x00, 0x00, 0x80, 0x00,
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
    /* 0x40 Clear Unit：偶校验（EMDIAG 0x820 判定 sync+data+station+parity 须为偶数个 1） */
    { 0x40, 7, 0, 1, { 0x2A, 0x00, 0x00 } },   /* 回显 [0xE] 写入的字节；0x460 判 bit1=0 / 0x46B 判 0x54 */
    /* 0x64：三帧，数据 14/AA/DB */
    { 0x64, 7, 0, 3, { 0x14, 0xAA, 0xDB } },
    /* 0x08 轮询：单帧 FF */
    { 0x08, 7, 0, 1, { 0xFF, 0x00, 0x00 } },
    /* 0x00：块 0xAB0 / 0x920 需要一帧带 sync 的应答（数据待 0x840/0x720 反推） */
    { 0x00, 7, 0, 1, { 0x00, 0x00, 0x00 } },
    /* 0x440 组的 0x22/0x2A 项：离线枚举 0x720 的 15 次 rcr 得到
       data=0x2A,st=7 -> BX=0x0E55，BL&0x7C==0x54 判据成立。 */
    { 0x61, 7, 0, 1, { 0x2A, 0x00, 0x00 } },
    { 0x30, 7, 0, 2, { 0x2A, 0x00, 0x00 } },
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
    { 0x09, 1,         1         },  /* 0x9D7 → 0x9E8 jnz 要 bit1=1；0xB7F → 0xB85 jc 要 bit0=1 */
};

struct ibm5250_t {
    uint8_t  latch;      /* 0x1C1 status latch, mirrored into 0x1C6 */
    uint8_t  cmdlat;     /* command latched at 0x1C0 */
    int      phase;      /* 0x1C6 bit1 for every command except 0x40 */

    int      ph;         /* 1 = handshake, 2 = frame */
    int      rd;         /* read index inside the current phase */
    int      serve;
    int      isr_active;
    int      isr_samp;   /* 已投中断、ISR 尚未读 0x1C1（采样窗口） */
    int      isr_c6;     /* ISR 采样后紧跟的那次 0x1C6 读 */
    uint8_t  c1_wval;
    int      c1_42;        /* 写 0x1C1=0x42 后，下一次 0x1C6 读 bit1=0（一次性） */
    int      c2_rst;       /* 写完 0x1C2=0x04 后 0x1C1 bit1 必须读 0 */      /* 最后一次写 0x1C1 的值 */   /* IRQ 已投递、尚未 ack：ISR 采样窗口 */
    int      txn_station;   /* 本帧站号（协议事务层） */
    int      txn_station_valid; /* 收到过轮询帧：站号才可用作非末帧的 B6–B4 */
    int      txn_cmd;       /* 本帧命令字 */      /* this poll gets a reply */
    int      done40;     /* the 0x40 frame has been served once */
    int      pre;        /* 帧前空槽（每槽 2 读）*/
    uint8_t  data8;      /* 0x1C8 数据寄存器 */
    int      seq64;      /* 0x64 命令的帧序号 */
    int      idle_rd;    /* 0x40 空闲期连续读计数（诊断用，判定已改为按 CL） */
    int      idle_sync;  /* 本次搜寻已找到 sync */
    int      idle_bits;  /* sync 之后已读的有效位个数（13 = 校验位） */
    int      irq_pending;/* 0xAA 触发后等发送例程跑完再投中断 */
    int      irq_from_aa;/* 本次中断由 0x1C9=0xAA 触发（决定 ISR 采到的状态值） */
    int      send_bits;  /* 本次发送已写出的位数（16 位一帧） */
    int      send_via_aa;/* 本次发送由 0x1C9=0xAA 发起 */
    int      aa_block;   /* 本块由 0xAA 触发（ISR 服务一次后清） */
    int      aa_latch;   /* 块级锁存：直到下一条 0x1C0 命令 */
    int      h00;        /* cmd=0x00 的 0x840 搜寻计数 */
    int      q00;        /* cmd=0x00 的 quiesce 交替位 */
    int      b5_raised;  /* 0x5B0 已投过的中断次数 */
    int      b5_clr;     /* 0x5B0：0x1C1=0x80 写了多少次（=轮询帧序号） */
    int      b5_sends;   /* 0x5B0 已完成的发送次数（决定 ISR 采样值） */
    int      blk5b0;    /* 0x5B0 块：ISR 采样要 bit7（0x5F2 的 and 0x80） */
    int      blk920;    /* 0x920 块：ISR 采样要 bit0=1（0x952 判据） */
    int      rx_budget;
    int      rx_zero;
    int      isr_val;
    int      prev_cmd;
    int      win_sel;    /* 0x1CA/0x10A 的窗口选择锁存 */
    int      cnt_pri;    /* 主窗口 0x1C0 组的访问计数 */
    int      cnt_alt;    /* 第二窗口 0x100 组的访问计数 */
    int      rx_on;      /* 0xAB0 块的接收序列在用 */
    int      rx_phase;   /* 0 = 0x840 头，1 = 0x720 收帧 */
    int      rx_d;       /* 头：数据读计数 */
    int      rx_c;       /* 头：时钟读计数 */
    int      rx_hunt;    /* 0x720：搜寻计数（sync 时须等于 hunt_lim） */
    int      hunt_lim;   /* 本次 0x720 的搜寻上限：常规 7，0x476 处 0xF7 */
    int      c1_b7;      /* 0x1C1 bit7 握手：写 0x80 清、发送完成置 1 */
    int      ba0_cnt;    /* 0xBA0 的 31 帧扫描 1..0x1F */
    int      ab0_cnt;    /* 0xAB0 的 255 帧数据扫描：1..255 */
    int      blk7a0;     /* cmd 0x10 @pc=0x07C6 → 0x7A0 块 */
    int      hdr_idx;    /* 0x840 头：当前拍号 0..16 */
    int      w6;         /* 0x840 头：本次入口以来的 0x1C6 写次数 */
    int      w6_last;    /* 上次读时的 w6（用于合并重复读） */
    int      hdr_last;   /* 上次返回的头位值 */
    int      hdr_site;   /* 0x840 头：当前调用点（[SP]） */
    int      hdr_n;      /* 该调用点内的读序号 */
    int      rx_bit;     /* 0x720：已收位数 */
    int      fno;        /* frame counter (log only) */
    int      u_bit0;     /* 收帧完成事件位（0x1C1 bit0，ISR 采样后清） */
    int      dummy;      /* optional leading idle read */

    /* ── 设备侧定位手段（替代 pc 判据的基础设施）── */
    int      rd_c0;      /* 上次 0x1C0 命令以来，0x1C0 的读次数 */
    int      rd_c1;      /* 上次 0x1C1 写以来，0x1C1 的读次数 */
    int      rd_c2;      /* 上次 0x1C2 写以来，0x1C2 的读次数 */
    int      rd_c6;      /* 上次 0x1C6 写以来，0x1C6 的读次数 */
    int      g_state;    /* 文法：0 空闲，1 收到命令，2 收到控制，3 收到选择 */
    uint8_t  g_cmd;      /* 最近一次 0x1C0 写 */
    uint8_t  g_ctl;      /* 最近一次 0x1C1 写 */
    uint8_t  g_sel;      /* 最近一次 0x1C2 写 */
    uint8_t  g_ctl3;     /* 最近一次 0x1C3 写 */

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
    int      w42;
    int      bit0;
    int      st3;        /* 0x1C2=0x09：共享 RAM 窗口选中 → 0x1C1 bit3 */
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
    int      a1_given;     /* 块 0x5B0 的 A1 bit1 只给第一次读 */
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

static void ibm5250_raise(ibm5250_t *dev);


void
ibm5250_irq_ack(ibm5250_t *dev)
{
    dev->isr_active  = 0;
    dev->irq_pending = 0;
    dev->c1_b7      = 1;
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
        if (dev->irq_raise != NULL) {
            dev->isr_active = 1;
            ibm5250_raise(dev);
        }
        return;
    }

    if (dev->samp > 0) {
        static long sn = 0;
        dev->samp--;
        sn++;
        if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x0840) &&
            (cpu_state.pc <= 0x08A0) && (sn <= 400))
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
            ((dev->cmdlat == 0x00) || (dev->cmdlat == 0x10)) &&
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
    dev->isr_active = 1;
            ibm5250_raise(dev);
    if (getenv("EPOCH5250_PCLOG"))
        pclog("5250: IRQ ASSERT (level) after trigger\n");

    /* 0x8C0 可能被同一块多次调用：保持窗口轮询，直到客人换命令。 */
    if (getenv("EPOCH5250_LEVELIRQ") &&
        ((dev->cmdlat == 0x00) || (dev->cmdlat == 0x10)) &&
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
    dev->isr_active = 1;
            ibm5250_raise(dev);
}

static uint8_t
ibm5250_base(ibm5250_t *dev)
{
    /* 0x1C6 是状态/数据混合口：bit7 = 数据位、bit0 = 0x1C2 选择器的结果、
       其余位镜像 0x1C1 的锁存（0x7C 掩码，bit1 除外）。
       bit1 是**同一根线**，只是有两个采样点：
         · 普通读（这里）→ dev->b1，由 0x1C1 写入置 1、0x1C2=0x04 时按 b1_pulse 重算
           （0xA2C/0x8E8 要求 1、0xA48 要求 0）
         · ISR 采样窗口 → dev->ready，见 in 0x1C6 的 ISR 分支
       之前把这两处写成“错误位/phase 两个字段”，是同一信号的两种说法，
       不是两条线；值本身两边都各自有判据，所以没有合并成一个来源。 */
    uint8_t v = (uint8_t) (dev->latch & 0x7C);

    /* 0x1C2←0x0A 之后的一次性 bit1 脉冲：给下一次非 ISR 的 0x1C6 读 */
    if (dev->b1_pulse && !dev->isr_c6) {
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
    if (k == 14) {
        dev->u_bit0 = 1;        /* 一帧收完 → 状态寄存器 bit0 事件 */
        /* 0x40 的应答发完就拉中断：客机 ISR 在 0x2D7 采样 0x1C1 写 [0x2A]，
           而 0x4AD 的 (al|0xAF)+1==0 检查依赖它（早于 0x1C9=0xAA 那次触发）。 */
        if ((dev->cmdlat == 0x40) && (dev->irq_raise != NULL))
            ibm5250_raise(dev);
    }
    return ibm5250_proto_frame_bit(k, data, 7);
}

static uint8_t
ibm5250_frame_bit(ibm5250_t *dev, int k)
{
    const ibm5250_frame_desc_t *f = ibm5250_frame_desc(dev->cmdlat);
    int d, ones, o12, st;

    if (f == NULL)
        f = ibm5250_frame_desc(0x08);

    d = f->data[(dev->cmdlat == 0x64) ? (dev->seq64 % f->nframes) : 0];

    /* B6–B4 是站号 0–6，末帧用 7 表示 EOM。表里恒填 7 是 EMDIAG 的实测值
       （0xB11 的 (BH|0xF1)+1==0 要求 W8..W11 全 1），单帧命令因此保持不变；
       多帧命令（0x64 三帧）的非末帧改用本次轮询的站号，且只在确实收到过
       轮询帧时生效，未收到时退回表值。 */
    st = f->station;
    if ((f->nframes > 2) && dev->txn_station_valid
        && (((dev->cmdlat == 0x64) ? (dev->seq64 % f->nframes) : 0) < (f->nframes - 1)))
        st = dev->txn_station & 7;

    if (dev->cmdlat == 0x40 && getenv("EPOCH5250_D40"))   /* 实验覆盖 */
        d = (int) strtoul(getenv("EPOCH5250_D40"), NULL, 16);

    if (k == 15)
        dev->u_bit0 = 1;
    /* 注意：0x40 帧结束【不能】投中断。0x495 的判据是 [0x2A]==0
       （确认 ISR 没留下旧采样），0x4AD 要的是端口 0x1C1 自己返回 0x50。
       提前投中断会把 [0x2A] 写成 0x5F，直接触发 AH=0x26。 */

    if (k == 0)                 return 1;                 /* B15 sync */
    if (k <= 8)                 return (uint8_t) ((d >> (k - 1)) & 1);  /* B14..B7 */
    if (k <= 11)                return (uint8_t) ((st >> (k - 9)) & 1); /* B6..B4 */
    if (k == 12) {
        ones = 1;                                          /* sync */
        for (int t = 0; t < 8; t++)
            ones += (d >> t) & 1;                          /* data */
        for (int t = 0; t < 3; t++)
            ones += (st >> t) & 1;                         /* station */
        o12 = ones;
        if (dev->cmdlat == 0x40 && getenv("EPOCH5250_P40"))
            return (uint8_t) atoi(getenv("EPOCH5250_P40"));
        /* 偶校验：12 位里 1 的个数为奇数时补 1；奇校验反过来 */
        return (uint8_t) (f->odd ? ((o12 & 1) ? 0 : 1) : ((o12 & 1) ? 1 : 0));
    }
    return 0;                                              /* B2..B0 fill */
}



static unsigned long g_rd_n;
static unsigned long g_isr_n;

static uint8_t
ibm5250_in_1c6(ibm5250_t *dev)
{
    if (getenv("EPOCH5250_IDLELOG") && (cpu_state.pc == 0x0485)) {
        static int once;
        if (!once) {
            once = 1;
            uint32_t dsb = ((uint32_t) cpu_state.seg_ds.seg) << 4;
            pclog("IDLE @485: [2D]=%02X [2A]=%02X [2B]=%02X CL=%02X idle_rd=%d\n",
                  mem_readb_phys(dsb + 0x2D), mem_readb_phys(dsb + 0x2A),
                  mem_readb_phys(dsb + 0x2B), CL, dev->idle_rd);
        }
    }
    {
        g_rd_n++;
        if (dev->isr_active)
            g_isr_n++;
        if (TRACE_ON && (dev->isr_active || (cpu_state.pc == 0x02DF) || (cpu_state.pc == 0x02D7)))
            pclog("5250: RD#%lu pc=%04X isr=%d ph=%d serve=%d\n",
                  g_rd_n, cpu_state.pc, dev->isr_active, dev->ph, dev->serve);
    }
    /* 0x40 首帧发完后，后续 0x720 一律走空闲图案：
       第 247 次读给 sync（→[0x2D]=0xF7，0x482 判据），
       第 260 次给 parity（sync+parity=2，偶校验通过），其余全 0（AL=0）。 */
    if ((dev->cmdlat == 0x40) && dev->done40) {
        uint8_t iv;
        if (getenv("EPOCH5250_TRACE") && ((dev->idle_rd < 3) || (dev->idle_rd == 495) ||
            (dev->idle_rd == 521) || (dev->idle_rd % 50 == 0))) {
            uint32_t dsb2 = ((uint32_t) cpu_state.seg_ds.seg) << 4;
            pclog("IDLEPATH rd=%d pc=%04X CL=%02X ds=%04X [2D]=%02X\n", dev->idle_rd,
                  cpu_state.pc, CL, cpu_state.seg_ds.seg, mem_readb_phys(dsb2 + 0x2D));
        }
        /* 0x8A0 每位读两次口（0x8A8 采用 / 0x8B2 丢弃）：
           搜寻循环 cl 从 0 起算，cl=0xF7 ⇒ sync 在第 495 次读（采用位）；
           随后第 13 位（parity）= 495+2*13 = 521。
           注意：判定必须用“本次读”的序号，不能先自增再比 —— 否则脉冲会错位
           到前一次（被丢弃的 0x8B2 读），采用的 0x8A8 读永远收到 0。 */
        int rd_now = dev->idle_rd;
        dev->idle_rd = rd_now + 1;
        iv = 0x00;
        /* 只认“采用位”那一次读（pc=0x8A8，值会过 rol al,1 进 CF）。
           0x8B2 是丢弃读，给它任何值都不会被看到。
           sync：客机搜寻循环 cl 走到 0xF7 时（0x737 会把 cl 存进 [0x2D]）；
           校验：sync 之后第 13 个有效位（sync + 8 data + 3 station + parity = 13 位偶校验）。 */
        if (ibm5250_read_kind(cpu_state.pc) == IBM_RD_DATA) {
            if (!dev->idle_sync && (CL == 0xF7)) {
                dev->idle_sync = 1;
                dev->idle_bits = 0;
                iv = 0x80;
            } else if (dev->idle_sync) {
                dev->idle_bits++;
                if (dev->idle_bits == 13)
                    iv = 0x80;
            }
        }
        if (getenv("EPOCH5250_IDLELOG") && (dev->idle_rd <= 300))
            pclog("IDLE rd=%d v=%02X pc=%04X\n", dev->idle_rd, iv, cpu_state.pc);
        return iv;
    }
    uint8_t base = ibm5250_base(dev);

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

    if (!dev->serve) {
        /* 只有 EMDIAG 的 0x40/0x64 收发要求静默期读回 0x00
           （0x720 超时返回 AL=(读值<<1)，必须为 0）；
           其它情形（IPL 早期轮询）保持 base，否则 POST 会卡住。 */
        if (dev->cmdlat == 0x40) {
            /* 0x720 的同步位搜索：空转第 247 次给一个 1，记入 [0x2D]=0xF7
               （0x482 判据）；之后仍返回 0，15 位数据全 0 → AL=0。 */
            dev->idle_rd++;
            if (dev->idle_rd == 247)
                return 0x80;    /* sync：客机把它当作第 247 位（[0x2D]=0xF7） */
            if (dev->idle_rd == 259)
                return 0x80;    /* 该伪帧的 parity 位：sync=1 + parity=1 → 偶校验通过 */
            return 0x00;        /* 数据/站号全 0 → AL=0（0x476 判据） */
        }
        if ((dev->cmdlat == 0x64) || (dev->cmdlat == 0xAA))
            return 0x00;
        return ret;
    }

    /* 协议：0x64 的响应在 0x1C9 触发后才就绪；0xCA0 清 [0x2A] 之后、
       0x840 开始收帧时拉 IRQ4，ISR 采样 0x1C1 写入 [0x2A]（0x6F8 检查）。 */
    if (dev->ph != 2) {
        /* ISR 的采样读不能消耗握手槽 */
        if (ISR_PC(cpu_state.pc))
            { uint8_t r = (uint8_t) ((base & 0x7d) | (dev->ready ? 0x02 : 0x00));
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }
        /* 协议：接收方以 sync=1 对齐；帧前必须是空闲 0。
           0x64 的 19 槽握手是实测拟合值，只用于它自己；其它命令（0x00/0x08…）
           给纯空闲，否则 0x840 的同步搜索会提前命中握手里的 1 而错位。 */
        ret = (uint8_t) ((base & 0x3f)
                         | (dev->ab0 ? 0x00
                                                  : ibm5250_handshake[dev->rd]));
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
            { uint8_t r = (uint8_t) ((dev->dummy == 1) ? (ret | 0x80) : (ret & 0x7f));
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }
        }
        dev->dummy = 0;
        return ret;
    }

    if (dev->pre > 0) {
        dev->pre--;
        { uint8_t r = (uint8_t) (base & 0x3f);
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }
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
        if (dev->blk5b0 && dev->a1_given && !dev->isr_c6)
            {   dev->a1_given = 0;
                uint8_t r = (uint8_t) ((base & 0x3f) | 0x02);
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }

        /* 块 0x5B0 的 A7 要求 [0x2A]&4：最后一次写 0x1C1 是 0x80(bit6=0)
           时，ISR 采样带上 bit2；其它情形仍是 0x4A。 */
        if (TRACE_ON && (dev->isr_active || (cpu_state.pc == 0x02D7) ||
            (cpu_state.pc == 0x02DF) || (cpu_state.pc == 0x02FB))) {
            pclog("5250: ISRRD pc=%04X ret=%02X isr=%d serve=%d ph=%d rd=%d\n",
                  cpu_state.pc, (unsigned) (0x4A | (dev->c1_ff ? 0x00 : 0x04)), dev->isr_active,
                  dev->serve, dev->ph, dev->rd);
        }
        { uint8_t r = (uint8_t) (0x4A | (dev->c1_ff ? 0x00 : 0x04));
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }

        if (ibm5250_read_kind(cpu_state.pc) == IBM_RD_NONE)
            

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
            int is_data  = (ibm5250_read_kind(cpu_state.pc) == IBM_RD_DATA);

            if ((r1 == 0x072C) || (r1 == 0x0743)) {   /* 0x729 搜寻 / 0x740 收帧 */
                if (r1 == 0x0743)
                    dev->fsync = 1;          /* 已进入 15 位收帧循环 */
                if (dev->fsync && (dev->fbit >= 15)) {
                    /* 上一帧收完，0x720 又回来搜寻下一帧 */
                    if (r2 != 0x089D) {     /* 0x840 内部那一帧不占序号 */
                        dev->fseq++;
                        /* 255 帧收完：补投一次中断，让 ISR 从 0x1C1(=0xDA)
                           采到 [0x2A]，0xB03 的 (al|0xB7)+1==0 才能成立。 */
                        if ((dev->fseq == 255) && (dev->irq_raise != NULL))
                            dev->isr_active = 1;
            ibm5250_raise(dev);
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
                { uint8_t r = (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }
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
        if (ibm5250_read_kind(cpu_state.pc) == IBM_RD_CLOCK) {
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
        { uint8_t r = (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }
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
            { uint8_t r = (uint8_t) ((base & 0x3d) | (dev->ready ? 0x02 : 0x00)
                                           | (fb ? 0x80 : 0x00));
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }
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
                { uint8_t r = (uint8_t) ((base & 0x3f) | (fb ? 0x80 : 0x00));
            /* 0x1C6 bit1 = 状态可读：写过 0x1C1 且 bit6=1（c1_ff）后
               表示服务位已被取走 → 读 0；否则读 1。0x8EC 要 1、0x908 要 0。 */
            if (ISR_PC(cpu_state.pc))
                r |= 0x02;                            /* ISR 采样：服务位在 */
            else if (dev->c1_42) {
                r &= (uint8_t) ~0x02;                 /* 0x1C1=0x42 的应答：读 0 */
                dev->c1_42 = 0;
            } else
                r |= 0x02;
            return r; }
            }
            dev->pair_pend = 0;
        }

        if (getenv("EPOCH5250_TRACE")) {
            static int n;
            if (n < 200) {
                pclog("5250: TRACE f=%d k=%2d fb=%d rd=%2d dt=%llu at=%04X:%04X bx=%04X bl=%02X bh=%02X\n",
                      dev->fno, k, fb, dev->rd,
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
            if (dev->cmdlat == 0x40) {
                if (!dev->done40) {
                    dev->idle_rd   = 0;
                    dev->idle_sync = 0;
                    dev->idle_bits = 0;
                }
                dev->done40 = 1;
            }
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
                    dev->serve = 0;      /* 三帧发完再静默 */
                    /* 0x64 三帧结束：第二次采样 bit1=0（0x70D 要 [0x2B]&3==0） */
                    dev->ready = 0;
                    if (getenv("EPOCH5250_LEVELIRQ") && dev->irq_raise) {
                        dev->irq_level = 0;
                        dev->isr_active = 1;
            ibm5250_raise(dev);
                        dev->irq_level = 1;
                    }
                }
            } else if ((dev->cmdlat == 0x40) || (dev->cmdlat == 0xAA)) {
                dev->serve = 0;
            }
        }
    }

    return ret;
}

static uint8_t ibm5250_in_real(uint16_t port, void *priv);

/* 所有中断投递都走这里：打开“ISR 采样窗口”。
   这样就不必再用 pc==0x02D7 去认 ISR 的那次读。 */
static void
ibm5250_raise(ibm5250_t *dev)
{
    dev->isr_samp = 1;
    if (dev->irq_raise != NULL)
        dev->irq_raise(dev->irq_priv);
}

/* 0x840 的 17 拍头：按调用点（[SP]）给值。0x84B 的搜寻循环每次产生两次读
   （0x8A8 + 0x8B2），所以 0x86F 的交替按“读序号 / 2”取奇偶。 */
static int
ibm5250_hdr_val(ibm5250_t *dev, uint16_t site)
{
    /* idx: 0 丢弃 / 1 采样 / 2 sync / 3 重复读吃掉 / 4..11 八拍交替
       / 12..14 三个 1 / 15..17 三个 0 / >=18 进帧 */
    static const uint8_t h17[18] = {
        0x00,0x00,0x80, 0x00,
        0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,
        0x80,0x80,0x80,
        0x00,0x00,0x00 };
    int idx;

    (void) site;
    /* 每位 = 3 次 0x1C6 写；w6 没变说明是同一个位的重复读，返回同一个值 */
    if (dev->w6 == dev->w6_last)
        return dev->hdr_last;
    dev->w6_last = dev->w6;
    idx = (dev->w6 > 0) ? ((dev->w6 - 1) / 3) : 0;
    if (idx >= 18) {
        dev->rx_phase = 1;
        dev->hdr_last = 0x00;
        return 0x00;
    }
    dev->hdr_last = h17[idx];
    return dev->hdr_last;
}

static uint8_t
ibm5250_in(uint16_t port, void *priv)
{
    if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x2D0) && (cpu_state.pc <= 0x308))
        pclog("ISRIN port=%03X pc=%04X cs=%04X\n", port, cpu_state.pc, CS);
    {
        ibm5250_t *d = (ibm5250_t *) priv;
        if ((port >= 0x0100) && (port < 0x0120)) d->cnt_alt++;
        else if ((port >= 0x01C0) && (port < 0x01E0)) d->cnt_pri++;
    }
    return ibm5250_in_real(ibm5250_norm(port), priv);
}

static uint8_t
ibm5250_in_real(uint16_t port, void *priv)
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

    /* 每口读计数：设备侧定位用的基础量，不参与任何判据 */
    switch (port) {
        case 0x1C0: dev->rd_c0++; break;
        case 0x1C1: dev->rd_c1++; break;
        case 0x1C2: dev->rd_c2++; break;
        case 0x1C6: dev->rd_c6++; break;
        default: break;
    }

    if (getenv("EPOCH5250_TRACE") && (port == 0x1C6)) {
        uint32_t ssp = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
        uint16_t rr  = (uint16_t) (mem_readb_phys(ssp) |
                                   (mem_readb_phys(ssp + 1) << 8));
        pclog("R6 pc=%04X rs=%04X cmd=%02X ph=%d b=%d\n", cpu_state.pc, rr,
              dev->cmdlat, dev->rx_phase, dev->blk7a0);
    }
    if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x0906) &&
        (cpu_state.pc <= 0x090A))
        pclog("5250: IN@%04X port=%03X\n", cpu_state.pc, port);
    switch (port) {
        case 0x1C1:
            /* bit1|bit6 are the adapter signature the ROM handshake checks */
            /* bit6|bit1 = 适配器签名；bit4 = 0x4B2 的 (al|0xAF)+1==0 检查要求
               (0x50 两位都为 1) */
            if (getenv("EPOCH5250_PCLOG"))
                pclog("5250: IN 0x1C1 = %02X pc=%04X (ISR=%d)\n",
                      (unsigned) (dev->latch | IBMR_ST_SIGNATURE), cpu_state.pc,
                      dev->isr_active);
            /* 全部 0x1C1 判据的并集：bit7|6|4|3|1 = 1、bit2 = 0 ⇒ 0xDA
               （0x4AD 要 6|4、0x616 要 7、0x683 要 2=0、0x8D3 要 6|1、0xA17 要 6|3） */
            {
                uint8_t u = (uint8_t) (dev->latch | IBMR_ST_SIGNATURE);
                if (getenv("EPOCH5250_TRACE") &&
                    ((cpu_state.pc == 0x02D7) || (cpu_state.pc == 0x02DF)))
                    pclog("ISRS pc=%04X port=%03X cmd=%02X latch=%02X b7=%d\n",
                          cpu_state.pc, port, dev->cmdlat, dev->latch,
                          dev->c1_b7);
                if (getenv("EPOCH5250_TRACE") &&
                    (cpu_state.pc >= 0x05B0) && (cpu_state.pc <= 0x06A0))
                    pclog("B5 C1 pc=%04X u=%02X b7=%d lat=%02X\n",
                          cpu_state.pc, u, dev->c1_b7, dev->latch);
                if (dev->c1_b7)
                    u |= 0x80;
                else
                    u &= (uint8_t) ~0x80;
                /* 采样窗口：投递中断后 ISR 的第一次 0x1C1 读。
                   用事件标志代替 pc==0x02D7。 */
                int is_samp = 0;
                if (dev->isr_samp) {
                    dev->isr_samp = 0;
                    dev->isr_c6   = 1;   /* 紧接着那次 0x1C6 读也属于 ISR */
                    is_samp       = 1;
                }
                if (getenv("EPOCH5250_TRACE") && (cpu_state.pc == 0x02D7))
                    pclog("ISRVAL u=%02X latch=%02X c1w=%02X isr=%d\n",
                          u, dev->latch, dev->c1_wval, dev->isr_active);
                if (getenv("EPOCH5250_IDLELOG") && ((cpu_state.pc == 0x02D6) || (cpu_state.pc == 0x02F2)))
                    pclog("ISRST pc=%04X u=%02X isr=%d c1w=%02X ff=%d\n",
                          cpu_state.pc, u, dev->isr_active, dev->c1_wval, dev->c1_ff);
                /* 只在 A3/A4 的回读点生效：0x616 读要 bit7=1、0x62B 读要 bit7=0。
                   全局套用会破坏 ISR(0x2D6) 采样（A2 要 [0x2A]&0x80）。 */
                /* 读点由最后一次写入 0x1C1 的值区分（0xC0 读要 bit7=1、
                   0x80 读要 bit7=0），不再依赖 EMDIAG 的读指令地址。 */
                /* bit6 是 bit7 回读的置位/清除控制：
                   0xC0（bit6=1）→ 回读 bit7=1；0x80（bit6=0）→ 回读 bit7=0。 */
                if (dev->c1_wval & 0x40)
                    u |= 0x80;
                else
                    u &= (uint8_t) ~0x80;
                /* 块 0x5B0 的 A7：ISR 采样 0x1C1 → [0x2A] 必须带 bit2。
                   最后一次写 0x1C1 是 0x80(bit6=0) 时给出该位。 */
                /* 块 0x5B0 的 A9：写 0x1C1=0x84 之后 0x687 读回 bit2 必须为 0
                   （bit2 是不可写状态位；全局屏蔽会误伤块 0x500 的 0x13）。 */
                if (dev->c1_wval & 0x04)
                    u &= (uint8_t) ~0x04;
                if (dev->c2_rst)
                    u &= (uint8_t) ~0x02;

                if (dev->isr_active && !dev->c1_ff)
                    u |= 0x04;   /* 写 0x1C1=0x84 之后 A9 要求 bit2=0，不能被这里加回来 */
                if (dev->u_bit0) {
                    dev->u_bit0 = 0;
                    u |= 0x01;
                }
                /* A9：写 0x1C1=0x84 后 0x687 的回读要求 bit2=0。
                   必须放在 isr_active 那行之后，否则 bit2 会被重新置上。
                   只认 0x688 这个读点，块 0x500 的 0x13 判据不受影响。 */
                /* A9：写 0x1C1=0x84 之后的第一次回读要求 bit2=0。
                   用“写值 + 读序号”定位，不再看 pc。 */
                if (dev->blk5b0 && (dev->c1_wval == 0x84) && (dev->rd_c1 == 1))
                    u &= (uint8_t) ~0x04;
                if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x0610) && (cpu_state.pc <= 0x0630))
                    pclog("R1C1 pc=%04X u=%02X c1w=%02X latch=%02X\n",
                          cpu_state.pc, u, dev->c1_wval, dev->latch);
                /* ISR 采样点：按触发来源给值（必须放在所有调整之后）。
                   AA 触发 → 0xDA（含 0x50，0x6F8 要）；
                   其它触发 → 0x02（0x8D3 要求 [0x2A] ⊆ 0x42，不能有 0x40/0x04）。 */
                if (is_samp)
                    /* 0x8D3 要 bit6|bit1；0x9D0 的 0xA17 还要 bit3（选了共享窗口） */
                    /* 0x8D3 要 bit6|bit1；0x9D0 的 0xA17 还要 bit3（选了共享窗口）；
                       0x920 的 0x952 要 bit0=1，而 0x7A0 的 0x8D3 明确要 bit0=0。 */
                    u = dev->irq_from_aa ? 0xDA
                                         : (dev->blk5b0 ? ((dev->b5_clr == 0) ? 0xDA : 0x04)
                                            : (uint8_t) ((dev->blk920 ? 0x43 : 0x42) |
                                                         (dev->st3 ? 0x08 : 0)));
                if (getenv("EPOCH5250_TRACE") && ((u & 0x02) == 0) &&
                    (cpu_state.pc >= 0x0800) && (cpu_state.pc <= 0x0A00))
                    pclog("5250: NOBIT1 pc=%04X v=%02X latch=%02X c1w=%02X isr=%d serve=%d ph=%d rd=%d\n",
                          cpu_state.pc, u, dev->latch, dev->c1_wval,
                          dev->isr_active, dev->serve, dev->ph, dev->rd);
                return u;
            }

        case 0x1C2:
        case 0x1C3:
            return 0xff;

        case 0x1C6: {
            uint8_t rv;
            /* ISR 自己读 0x1C6 时不再投中断（isr_c6 就是那次读的标记） */
            if (dev->irq_pending && !dev->isr_c6) {
                if (getenv("EPOCH5250_TRACE"))
                    pclog("IRQFIRE pc=%04X cmd=%02X aa=%d bits=%d\n",
                          cpu_state.pc, dev->cmdlat, dev->irq_from_aa, dev->send_bits);
                dev->irq_pending = 0;
                dev->isr_active  = 1;
                dev->irq_level   = 1;
                if (dev->irq_raise != NULL)
                    ibm5250_raise(dev);
            }
            if (dev->isr_c6) {
                dev->isr_c6 = 0;
                return (uint8_t) dev->isr_val;
            }
            rv = ibm5250_in_1c6(dev);
            /* ── 0xAB0 块的接收序列（块级状态机）─────────────────────────
               0x840 头：0x845 数据(0) → 0x848 时钟 → 0x84B 数据(sync)
                         → 8 拍交替 → 3×1 → 3×0
               0x720 收帧：7 拍空转后 sync（[0x2D]=7）→ 15 位帧，
                         数据字节 = fseq+1（0xAE7 的 al == ch）
               状态由 0x1C3=0x08（0xABE）起算，不受 ISR 端口读干扰。 */
            if (getenv("EPOCH5250_TRACE") && dev->rx_on &&
                (ibm5250_read_kind(cpu_state.pc) != IBM_RD_NONE))
                pclog("RX %c phase=%d d=%d c=%d hunt=%d bit=%d fseq=%d v=%02X\n",
                      (ibm5250_read_kind(cpu_state.pc) == IBM_RD_DATA) ? 'D' : 'C',
                      dev->rx_phase, dev->rx_d, dev->rx_c, dev->rx_hunt,
                      dev->rx_bit, dev->fseq,
                      (unsigned)((cpu_state.pc == 0x08A8) ? 0 : 0));
            if (dev->rx_on && (ibm5250_read_kind(cpu_state.pc) != IBM_RD_NONE)) {
                uint16_t rs = 0;
                {   uint32_t se = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
                    rs = (uint16_t) (mem_readb_phys(se) |
                                              (mem_readb_phys(se + 1) << 8));
                    uint16_t rc = (uint16_t) (mem_readb_phys(se + 2) |
                                              (mem_readb_phys(se + 3) << 8));
                    if ((rs == 0x0848) &&
                        ((ibm5250_read_kind(cpu_state.pc) == IBM_RD_DATA) ||
                         (dev->blk7a0 && (dev->rx_phase != 0)))) {
                        /* 0x845 的 call 0x8A0 返回 0x0848：0x840 入口，头重来 */
                        dev->rx_phase = 0;
                        dev->hdr_idx  = 0;
                        dev->w6       = 0;
                        dev->w6_last  = -1;
                        if (getenv("EPOCH5250_TRACE"))
                            pclog("H840RST pc=%04X rs=%04X b=%d\n",
                                  cpu_state.pc, rs, dev->blk7a0);
                    }
                    /* 0x476 的 0x720 要 [0x2D]==0xF7（247 次空转） */
                    dev->hunt_lim = (rc == 0x0479) ? 0xF7 : 7;
                    if ((dev->cmdlat == 0x40) && getenv("EPOCH5250_TRACE")) {
                        static int nT = 0;
                        if (nT < 26) { nT++;
                            pclog("T#%02d pc=%04X rs=%04X rc=%04X ph=%d hi=%d hunt=%d bit=%d\n",
                                  nT, cpu_state.pc, rs, rc, dev->rx_phase,
                                  dev->hdr_idx, dev->rx_hunt, dev->rx_bit);
                            if (nT > 0) pclog("   w6=%d idx=%d\n", dev->w6,
                                              (dev->w6 > 0) ? ((dev->w6 - 1) / 3) : 0); }
                    }
                }
                if (ibm5250_read_kind(cpu_state.pc) == IBM_RD_DATA) {   /* 数据读：bit7 = 线路值 */
                    if (dev->rx_phase == 0) {
                        uint8_t v = 0x00;
                        if ((dev->cmdlat == 0x40) || dev->blk7a0) {
                            v = (uint8_t) ibm5250_hdr_val(dev, rs);
                            dev->rx_d++;
                            return v;
                        }
                        v = (dev->rx_d == 1) ? 0x80 : 0x00;
                        dev->rx_d++;
                        return v;
                    }
                    if (dev->rx_hunt < dev->hunt_lim) {                /* 未命中：0x72E inc cl */
                        dev->rx_hunt++;
                        return 0x00;
                    }
                    if (dev->rx_hunt == dev->hunt_lim) {   /* sync：[0x2D] = lim */
                        dev->rx_hunt = dev->hunt_lim + 1;
                        dev->rx_bit  = 0;
                        return 0x80;
                    }
                    {
                        /* 0x840 自己在 0x89A 也收一帧（返回地址 0x089D）：
                           它必须解出 0（0xAD9 的 or al,al），且不占帧号；
                           块内 0xAE4 的 255 帧从 data=1 起。 */
                        uint32_t ssp = ((uint32_t) cpu_state.seg_ss.seg << 4) + SP;
                        uint16_t ra  = (uint16_t) (mem_readb_phys(ssp + 2) |
                                                   (mem_readb_phys(ssp + 3) << 8));
                        uint16_t ra2 = (uint16_t) (mem_readb_phys(ssp + 4) |
                                                   (mem_readb_phys(ssp + 5) << 8));
                        int internal = (ra == 0x089D);
                        /* 0x920 块两段 0x840，内部帧要求相反：
                           第一段（0x973 调用）BH.bit0=1/BL.bit7=1 → 0xC0；
                           第二段（0x9A5 调用）两者都 0 → 0x00。 */
                        int data = internal ? ((dev->blk920 && (ra2 == 0x0973))
                                                  ? 0xC0 : 0x00)
                                            : ((dev->fseq + 1) & 0xFF);
                        if (dev->rx_zero && !internal)
                            data = 0x00;
                        /* 0x440：首帧 0x2A（0x720 的 15 次 rcr 后 BL&0x7C==0x54），
                           之后一帧 0x00（0x479 的 or al,al 要求 0）。 */
                        if (ra == 0x0AE7) {
                            /* 0xAB0 的 255 帧数据扫描（必须优先于 cmdlat 规则） */
                            if (dev->rx_bit == 0)
                                dev->ab0_cnt = (dev->ab0_cnt % 255) + 1;
                            data = dev->ab0_cnt;
                        }
                        else if (ra == 0x0BD0) {
                            if (dev->rx_bit == 0)
                                dev->ba0_cnt = (dev->ba0_cnt % 0x1F) + 1;
                            data = dev->ba0_cnt;
                        }
                        else if ((dev->cmdlat == 0x40) && !dev->rx_zero)
                            data = (dev->fseq == 0) ? 0x2A : 0x00;
                        else if (dev->blk7a0)
                            data = 0x00;
                        int b = ibm5250_proto_frame_bit(dev->rx_bit, data, 7);
                        if (getenv("EPOCH5250_TRACE") && (ra == 0x0AE7))
                            pclog("FB bit=%2d data=%02X v=%d\n", dev->rx_bit, data, b);

                        if (++dev->rx_bit >= 15) {
                            dev->rx_bit  = 0;
                            dev->rx_hunt = 0;              /* 下一帧重新搜寻 */
                            if (!internal || (dev->cmdlat == 0x40)) {
                                dev->fseq++;
                                if (dev->rx_budget > 0 && --dev->rx_budget == 0)
                                    dev->rx_zero = 1;
                            }
                        }
                        return (uint8_t) (b ? 0x80 : 0x00);
                    }
                } else {                                   /* 时钟读 */
                    if (dev->rx_phase == 0) {
                        uint8_t v;
                        if ((dev->cmdlat == 0x40) || dev->blk7a0) {
                            return (uint8_t) ibm5250_hdr_val(dev, rs);
                        }
                        v = (uint8_t) (ibm5250_proto_clk_bit(dev->rx_c) ? 0x80 : 0x00);
                        if (++dev->rx_c >= 17)
                            dev->rx_phase = 1;             /* 头结束 → 0x720 */
                        return v;
                    }
                    return 0x80;                           /* 0x735 的 jnc 要求 CF=1 */
                }
            }
            if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x08A0) && (cpu_state.pc <= 0x08B8))
                pclog("RD8A pc=%04X v=%02X cmd=%02X serve=%d ph=%d rd=%d fsync=%d fbit=%d fno=%d ab0=%d\n",
                      cpu_state.pc, rv, dev->cmdlat, dev->serve, dev->ph, dev->rd,
                      dev->fsync, dev->fbit, dev->fno, dev->ab0);
            if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x0B70) && (cpu_state.pc <= 0x0B90))
                pclog("RDB7 pc=%04X v=%02X bit0=%d b1=%d cmd=%02X ph=%d rd=%d latch=%02X\n",
                      cpu_state.pc, rv, dev->bit0, dev->b1, dev->cmdlat, dev->ph,
                      dev->rd, dev->latch);
            if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x0A40) && (cpu_state.pc <= 0x0A50))
                pclog("RDA4 pc=%04X v=%02X b1=%d ph=%d w42=%d latch=%02X bp=%d cmd=%02X done40=%d ph=%d rd=%d serve=%d ab0=%d\n",
                      cpu_state.pc, rv, dev->b1, dev->phase, dev->w42, dev->latch,
                      dev->b1_pulse, dev->cmdlat, dev->done40, dev->ph, dev->rd,
                      dev->serve, dev->ab0);
            /* 0x1C2 选择器的 bit0 输出：选 0x08/0x09 期间读回 bit0=1，
               选 0x01 读回 0。不再按指令地址白名单。 */
            if (dev->bit0)
                rv |= 0x01;
            if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x07F8) && (cpu_state.pc <= 0x0812))
                pclog("RD805 v=%02X bit0=%d bit1=%d cmd=%02X done40=%d\n",
                      rv, dev->bit0, dev->b1, dev->cmdlat, dev->done40);
            return rv;
        }

        default:
            return 0xff;
    }
}

static void
ibm5250_out(uint16_t port, uint8_t val, void *priv)
{
    ibm5250_t *dev = (ibm5250_t *) priv;

    if ((port >= 0x0100) && (port < 0x0120)) dev->cnt_alt++;
    else if ((port >= 0x01C0) && (port < 0x01E0)) dev->cnt_pri++;

    /* 0x1CA / 0x10A 是窗口/通道选择锁存（两遍都会写，顺序固定）。 */
    if ((port == 0x01CA) || (port == 0x010A))
        dev->win_sel = val;

    port = ibm5250_norm(port);

    /* (命令, 控制, 选择) 文法：只记录转移，不改变行为 */
    switch (port) {
        case 0x1C0:
            dev->g_cmd = val; dev->g_state = 1;
            dev->rd_c0 = dev->rd_c1 = dev->rd_c2 = dev->rd_c6 = 0;
            break;
        case 0x1C1:
            dev->g_ctl = val; dev->g_state = 2; dev->rd_c1 = 0;
            break;
        case 0x1C2:
            dev->g_sel = val; dev->g_state = 3; dev->rd_c2 = 0;
            break;
        case 0x1C3:
            dev->g_ctl3 = val; dev->rd_c6 = 0;
            break;
        case 0x1C6:
            dev->rd_c6 = 0;
            if (((dev->cmdlat == 0x40) || dev->blk7a0) && (dev->rx_phase == 0))
                dev->w6++;
            break;
        default:
            break;
    }
    if (getenv("EPOCH5250_TRACE") && (getenv("EPOCH5250_GRAMMAR")))
        pclog("G st=%d cmd=%02X ctl=%02X sel=%02X c3=%02X | port=%03X v=%02X pc=%04X\n",
              dev->g_state, dev->g_cmd, dev->g_ctl, dev->g_sel, dev->g_ctl3,
              port, val, cpu_state.pc);

    if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x05B0) && (cpu_state.pc <= 0x0700))
        pclog("OUT5B pc=%04X port=%03X v=%02X\n", cpu_state.pc, port, val);
    if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x7A0) && (cpu_state.pc <= 0x800))
        pclog("OUT7A pc=%04X port=%03X v=%02X cmd=%02X\n", cpu_state.pc, port, val, dev->cmdlat);
    if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0xA00) && (cpu_state.pc <= 0xC00))
        pclog("OUTA pc=%04X port=%03X v=%02X\n", cpu_state.pc, port, val);

    if (getenv("EPOCH5250_LOGIO")) {
        static int n;
        if (n < 300) {
            pclog("5250: OUT %03X = %02X\n", port, val);
            n++;
        }
    }

    switch (port) {
        case 0x1C3:
            /* 0xAB0 块在 0xABE 写 0x1C3=0x08 作为"开始接收"的标记：
               0x840 的搜寻/静默序列必须从这里重新起算，否则上一块
               （或 ISR 的端口读）留下的 rd/clkc 会让 8 次搜寻全部落空。 */
            if (dev->cmdlat == 0x00) {   /* 只有 0xAB0 这块走这套序列 */
            dev->rd     = 0;
            dev->clkc   = 0;
            dev->fhunt  = 0;
            dev->fbit   = 0;
            dev->fsync  = 0;
            dev->serve  = 1;
            dev->ph     = 2;
            dev->rx_on    = 1;
            dev->rx_phase = 0;
            dev->rx_d     = 0;
            dev->rx_c     = 0;
            dev->rx_hunt  = 0;
            dev->rx_bit   = 0;
            dev->fseq     = 0;   /* 帧计数从 0 起：数据字节 = fseq+1 = ch */
            dev->isr_val   = 0x04;
            dev->rx_zero   = 0;
            dev->rx_budget = 0xFF;
            }
            
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
            if (val == 0xAA) {   /* 触发一次收发：发送完成即拉中断，
                                    ISR 在 0xB90 延时窗口内写 [0x2A]/[0x2B] */
                /* 同时重开一帧序列：0x440 之后的 0xAA 要一帧新的应答，
                   不能被 0x40 的空闲图案挡住（0x840 只搜 8 次）。 */
                dev->done40  = 0;
                dev->idle_rd = 0;
                dev->rd      = 0;
                dev->fhunt   = 0;
                dev->fsync   = 0;
                dev->fbit    = 0;
                /* 不能在这里投：0xCA0 = call 0xC40（清 [0x2A]/[0x2B]）
                   + call 0xC90（发送），ISR 若此刻跑，采样值会被 0xC40 清掉。
                   挂起，等发送结束后的第一次 0x1C6 读再投。 */
                dev->send_via_aa = 1;
                dev->aa_block    = 1;
                dev->aa_latch    = 1;
                dev->send_bits   = 0;   /* 本次发送重新计数，别被上一帧的余数提前触发 */
            }
        case 0x1CA:
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
            pclog("5250: CMDW %02X pc=%04X [pri=%d alt=%d sel=%d]\n",
                  val, cpu_state.pc, dev->cnt_pri, dev->cnt_alt, dev->win_sel);
            /* 协议表：每个测试块都先写一条命令（0x00/0x08/0x10/0x40/0x64/0xAA…），
               之前只放行 0x40/0x08/0x64，导致 0x7A0 起步就被忽略。 */
            if (val != 0xFF) {
                /* keep the 0x40 reply in place while the subtest is running */
                int hold = (val == 0x08) && (dev->cmdlat == 0x40) && !dev->done40;

                if (!hold) {
                    dev->prev_cmd = dev->cmdlat;
                    dev->cmdlat = val;
                    if ((val == 0x10) && (dev->prev_cmd == 0x64))
                        dev->blk7a0 = 1;                  /* 进入 0x7A0 块（0x64 之后的 0x10） */
                    else if ((val == 0x40) || (val == 0x64) || (val == 0x08))
                        dev->blk7a0 = 0;                  /* 其它块的命令 */
                    dev->ph     = 1;
                    dev->rd     = 0;
                    dev->idle_rd = 0;
                    dev->done40  = 0;   /* 新命令到来：结束 0x40 空闲图案 */
                    dev->idle_sync   = 0;
                    dev->idle_bits   = 0;
                    dev->irq_pending = 0;   /* 新命令作废上一块挂起的中断 */
                    dev->aa_latch    = 0;
                    dev->h00         = 0;
                    dev->q00         = 0;
                    /* 收摊写不算新纪元：0x00（主窗口）与 0x10（0x100 窗口，
                       0x50C 的 cmp dx,0x100 分支）都不得清掉刚起的接收序列。 */
                    if ((val != 0x00) && !((val == 0x10) && (dev->prev_cmd == 0x08))) {
                        dev->rx_on = 0;
                        /* 预算状态只属于 0x500 那个窗口：新命令必须清零，
                           否则后续块的 0x840 sync 搜寻拿到全 0 数据。 */
                        dev->rx_zero = 0;
                        dev->rx_budget = 0xFF;
                    }
                    /* 0x7A0 块（cmd 0x10 @pc=0x07C6）必须重新开接收序列：
                       否则 rx_on 被上面清掉，0x8C0 链里的 0x840 一行数据都拿不到。 */
                    if ((val == 0x10) && (dev->prev_cmd == 0x64)) {
                        dev->isr_val = 0x02;
                        dev->rx_on = 1; dev->rx_phase = 0; dev->rx_d = 0;
                        dev->rx_c = 0; dev->rx_hunt = 0; dev->rx_bit = 0;
                        dev->serve = 1; dev->rx_zero = 0; dev->rx_budget = 0xFF;
                        dev->w6 = 0; dev->w6_last = -1;
                    }
                    /* 同一签名：0x1C0←0x00 且 (ctl=48, sel=04) */
                    dev->blk920      = ((dev->cmdlat == 0x00) && (dev->prev_cmd == 0x00) &&
                                        (dev->g_ctl == 0x48) && (dev->g_sel == 0x04));
                    if (dev->blk920)
                        dev->isr_val = 0x02;
                    dev->blk5b0      = 0;
                    dev->b5_raised   = 0;
                    dev->b5_sends    = 0;
                    if (val == 0x40) {
                        dev->isr_val = 0x02;
                        /* 0x440 链在 0x45B 有它自己的 0x840 sync 搜寻 */
                        dev->rx_on = 1; dev->rx_phase = 0; dev->rx_d = 0; dev->rx_c = 0;
                        dev->rx_hunt = 0; dev->rx_bit = 0; dev->serve = 1;
                        dev->rx_zero = 0; dev->rx_budget = 0xFF;
                        dev->fseq = 0;
                    }
                    if (val == 0x64) dev->isr_val = 0x04;
                    if ((val == 0x10) && (dev->prev_cmd == 0x64))
                        dev->isr_val = 0x02;      /* 只给 0x7A0 块，0x500 收摊仍用旧值 */
                    if (val == 0x00 && dev->prev_cmd == 0x64) dev->isr_val = 0x02;
                    if (val == 0x08) {
                        dev->isr_val = 0x04;
                        dev->rd = 0; dev->clkc = 0; dev->fhunt = 0; dev->fbit = 0;
                        dev->fsync = 0; dev->serve = 1; dev->ph = 2;
                        dev->rx_on = 1; dev->rx_phase = 0; dev->rx_d = 0; dev->rx_c = 0;
                        dev->rx_hunt = 0; dev->rx_bit = 0; dev->fseq = 0;
                        dev->rx_zero = 0; dev->rx_budget = 1;
                    }
                    if (dev->blk920) {   /* 0x920 也走同一套接收序列（立即 sync） */
                        dev->rd = 0; dev->clkc = 0; dev->fhunt = 0; dev->fbit = 0;
                        dev->fsync = 0; dev->serve = 1; dev->ph = 2;
                        dev->rx_on = 1; dev->rx_phase = 0; dev->rx_d = 0;
                        dev->rx_c = 0; dev->rx_hunt = 0; dev->rx_bit = 0; dev->fseq = 0;
                    }
                    dev->st3         = 0;
                    dev->send_bits   = 0;
                    dev->send_via_aa = 0;
                    /* 块 0x920 的命令写在 pc=0x0927；它要自己走一遍
                       0x840，必须从干净序列开始（实测宽条件会误伤
                       0x7A0 的 0x7C6）。 */
                    /* 设备侧签名：0x927 这次 0x1C0←0x00 发生在 (ctl=48, sel=04)
                       之后；0x7A0 的 0x7C6 是 (ctl=FF, sel=00)，天然分开。 */
                    if ((val == 0x00) && (dev->g_ctl == 0x48) && (dev->g_sel == 0x04)) {
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
            if ((val == 0x00) && (dev->prev_cmd == 0x10) &&
                getenv("EPOCH5250_LEVELIRQ") && dev->irq_raise) {
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
            if (getenv("EPOCH5250_TRACE") && (val == 0xFF)) {
                uint32_t dsb = ((uint32_t) cpu_state.seg_ds.seg) << 4;
                pclog("CHK2A pc=%04X ds=%04X [2A]=%02X [2B]=%02X [2D]=%02X\n",
                      cpu_state.pc, cpu_state.seg_ds.seg,
                      mem_readb_phys(dsb + 0x2A), mem_readb_phys(dsb + 0x2B),
                      mem_readb_phys(dsb + 0x2D));
            }
            
            if (getenv("EPOCH5250_TRACE") && (cpu_state.pc >= 0x05F0) && (cpu_state.pc <= 0x0700))
                pclog("W1C1 pc=%04X v=%02X\n", cpu_state.pc, val);
            dev->c1_wval = val;
            /* 0x5B0 握手：写 0x80 到 0x1C1 → 下一次读 bit7 必须变 0；
               写 0x84 的 bit2 是“确认并清除” → latch 的 bit2 也要清 */
            if (val & 0x80)
                dev->c1_b7 = 0;
            if (val & 0x04)
                dev->latch &= (uint8_t) ~0x04;
            if (val == 0x42)
                dev->c1_42 = 1;
            dev->c2_rst   = 0;
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
            /* 写 0x1C1=0x80 是"清 bit7"：0x5B0 在 0x626 写它，
               0x62B 随即回读要求 bit7=0（AH=0xA4）。 */
            dev->latch = (val == 0x80) ? (uint8_t) IBMR_LATCH_SIG : (uint8_t) (val | IBMR_LATCH_SIG);
            if ((val == 0x80) && dev->blk5b0)
                dev->b5_clr++;   /* 每轮询帧一次：0 → 首帧, 1..15 → 循环内, ≥15 → 循环后 */
            dev->b1    = 1;   /* 0x1C1 写入即置状态 bit1（0xA2C/0x8E8 判据） */
            /* 任何一次 0x1C1 写入都会让下一次 1C2=04 的 bit1 变成 0：
               0x8F6 写 0x42、0xA2B 写 0x48 都是这个效果（0x904 / 0xA48 两条判据）。 */
            dev->w42 = 1;
            if (getenv("EPOCH5250_TRACE"))
                pclog("W42SET pc=%04X cmd=%02X\n", cpu_state.pc, dev->cmdlat);
            /* 0x6A0 末尾：写 0x1C1=0xFF 之后要求 (0x1C6 & 3) == 0
               ⇒ 视为复位/静默，停止继续供帧，0x1C6 读回 0 */
            if (val == 0xFF) {
                dev->serve = 0;
                dev->rd    = 0;
                dev->ph    = 0;
                dev->seq64 = 0;
            }
            break;

        case 0x1C2:
            /* 新的操作开始：结束 0x40 的空闲图案，否则后面的块每读 0x1C6
               都会命中空闲分支（0x7A0 的 0x808 因此拿不到 bit0 → AH=0x43）。 */
            dev->done40 = 0;
            /* 0x1C1 曾写 0xFF（0x4BD/0x706 的自检）→ base() 会把 bit0/bit1
               一律压成 0；新的一次状态选择开始，残值必须清掉，否则
               0x7D8/0x806 这些按选择器取 bit0 的判据永远拿不到 1。 */
            if (dev->latch == 0xFF)
                dev->latch = 0;
            if ((val & 0x0F) == 0x09)
                dev->st3 = 1;   /* 块级：选中共享窗口后整块都算（0x8C0 链里的 0x04 不能清掉它） */
            /* 状态重选 = 新的一次操作：握手序列从头开始。
               否则 0xA48（写 0x1C2=0x04 后要求 bit1=0）会读到握手表里的 0x42。 */
            dev->ph    = 1;
            dev->rd    = 0;
            /* 相位选择（0x1C2=0x04）表示“这一轮响应到此为止”：
               0xA48 随后的 0x1C6 读必须回到状态寄存器，而不是继续送帧。 */
            /* 只有 0xA40 那次相位选择（0xA48 判据）结束供帧；
               其它 0x1C2=0x04 仍需继续送帧（0x9D0 前两项）。 */
            /* 块起点判据改为设备侧：0x1C2←0x10 全片只出现在 A 组开头
               （文法日志里 (ctl=01, sel=10) 共 4 次，全在本块内）。 */
            if (val == 0x10) {
                dev->blk5b0   = 1;   /* 0x5B0 块开始 */
                dev->a1_given = 1;
                dev->b5_sends = 0;
                dev->b5_raised = 0;
                dev->b5_clr   = 0;
            }
            /* 设备侧签名：(ctl=01, sel=04) 全片唯一，即 0x920 的第二段重启 */
            if ((val & 0x04) && (dev->g_ctl == 0x01) && dev->rx_on) {
                /* 0x920 块的第二段：0x999 选相位后重新走一遍 0x840 头序列。 */
                dev->rx_phase = 0; dev->rx_d = 0; dev->rx_c = 0;
                dev->rx_hunt = 0;  dev->rx_bit = 0;
            }
            if ((val & 0x04) && ibm5250_pc_in(cpu_state.pc, ibm_pc_sel4,
                                              sizeof(ibm_pc_sel4) / sizeof(ibm_pc_sel4[0])))
                dev->serve = 0;

            
            
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
            if ((val == 0x00)) {
                dev->ready = 1;
                dev->ph    = 2;
            }

            /* 块 0x920 第二阶段：0x999 写 1C2=04 之后紧跟 0x9A2 call 0x840，
               此时 rd 已经走到 51，不重开起始序列就必然 AH=01。 */
            if ((val == 0x04)) {
                if (TRACE_ON) pclog("5250: BOUND 1C2=04 rd=%lu isr=%lu pc=%04X\n", g_rd_n, g_isr_n, cpu_state.pc);
                dev->c2_rst = 1;

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
                dev->fhunt   = 0;
                dev->fsync   = 0;
                dev->fbit    = 0;
                dev->flast   = 0;
                dev->clkc    = 0;
                dev->fseq    = 0;
                dev->rx_zero = 0;
                dev->rx_budget = 0xFF;
                dev->b1_pulse = 1;
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
                    /* 不在选择器表里的写（如 0x1C2=0x42/0x01）是状态重选：
                       清 bit1（0x8FA → 0x904 要求 bit1=0）。 */
                    dev->b1 = 0;
                }
            }

            /* 必须放在选择器之后：0x0A 不在表里，上面的 else 会用 phase
               覆盖。块 0xBA0 在 0xBA5（取指后报 0xBA8）写 0x0A，随后
               0xBB6 读 0x1C6 要求 bit1=1。 */
            /* 块标记改用协议值：0x1C3←0x0A 只有 0xBA0 块会写，不必看指令地址 */
            if (val == 0x0A) {
                dev->b1 = 1;
                /* 块 0xBA0 与 0xAB0 同构（31 帧，ch=1..0x1F），用 0x1C2=0x0A 起算。 */
                dev->rd = 0; dev->clkc = 0; dev->fhunt = 0; dev->fbit = 0;
                dev->fsync = 0; dev->serve = 1; dev->ph = 2;
                dev->rx_on = 1; dev->rx_phase = 0; dev->rx_d = 0; dev->rx_c = 0;
                dev->rx_hunt = 0; dev->rx_bit = 0; dev->fseq = 0;
            }

            /* 块 0x7A0 每次写 0x1C2 之后都会进 0x8C0：每次都重新武装
               中断窗口，否则第二次调用没有中断、[0x2B] 被 0xCA0 清掉。 */
            if ((dev->cmdlat == 0x00) && getenv("EPOCH5250_LEVELIRQ") &&
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
            if (ibm5250_is_tx_wr_pc(cpu_state.pc)) {
                if (++dev->send_bits >= 16) {
                    dev->c1_b7 = 1;      /* 一帧发完：bit7 重新就绪 */
                    dev->send_bits = 0;
                    /* 0x440 回显块不投：它要求 [0x2A]==0（0x495），中断会把
                       采样写进去。其余块（0x6A0 / 0x7A0）在发送完成后要一次中断。 */
                    if (dev->blk5b0) {
                        dev->c1_wval |= 0x40;   /* 一次发送 → 状态 bit7 重新置位（0x5B0 的 A3 循环） */
                        dev->b5_sends++;
                    }
                    /* 0x5B0：第 1 次发送要中断（A2/A3），中间 15 帧轮询不留 [0x2A]（A5），
                       循环后的发送再要一次（A7 的 bit2）。 */
                    if (dev->blk5b0 && (dev->b5_clr >= 1) && (dev->b5_clr <= 14)) {
                        /* 15 帧轮询期内不投（A5 要求循环结束 [0x2A]==0） */
                    } else if (dev->cmdlat != 0x40) {
                        if (dev->blk5b0)
                            dev->b5_raised++;
                        /* 发送完成立即投：0x8C0 链发完就直接进 0xB90 延时，
                           期间不读 0x1C6，挂起到“下一次读”会永远不触发。 */
                        dev->irq_from_aa = dev->aa_latch;   /* 块级锁存：一块内多帧都算同一语境 */
                        if (getenv("EPOCH5250_TRACE"))
                            pclog("IRQRAISE cmd=%02X pc=%04X aa=%d\n",
                                  dev->cmdlat, cpu_state.pc, dev->irq_from_aa);
                        if (dev->irq_raise != NULL) {
                            dev->isr_active = 1;
                            dev->irq_level  = 1;
                            ibm5250_raise(dev);
                        }
                    }
                    /* send_via_aa 不在这里清：一块里会连发多帧，
                       只有下一条 0x1C0 命令才算新语境。 */
                }
            }
            
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
                        /* 事务层：按命令字区分轮询语义
                           0x21 = 扫描（0x500 的 8 站轮询）-> 只答本机站 7
                           其它（0x30/0x61 单站命令）-> 地址解析到本机即答 */
                        dev->txn_station = (dev->word >> 9) & 7;
                        dev->txn_station_valid = 1;
                        dev->txn_cmd     = (dev->word >> 1) & 0xFF;
                        dev->serve = (dev->txn_cmd == 0x21) ? (dev->txn_station == 7) : 1;
                        pclog("5250: FRAME %04X serve=%d\n", dev->word, dev->serve);
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
    dev->ph       = 1;
    dev->rd       = 0;
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
    0x1D0, 0x1D1, 0x1D6,
    0x100, 0x101, 0x102, 0x103, 0x106, 0x107,
    0x108, 0x109, 0x10A, 0x10B, 0x10C, 0x10D, 0x10E, 0x10F,
    0x110, 0x111, 0x116
};

/* 只观测：0x100..0x11F 上的访问（第二窗口）。返回 0xFF、吞掉写，不影响行为。 */
static uint8_t
ibm5250_alt_probe_in(uint16_t port, void *priv)
{
    (void) priv;
    pclog("ALTIN  p=%03X pc=%04X\n", port, cpu_state.pc);
    return 0xFF;
}

static void
ibm5250_alt_probe_out(uint16_t port, uint8_t val, void *priv)
{
    (void) priv;
    pclog("ALTOUT p=%03X v=%02X pc=%04X\n", port, val, cpu_state.pc);
}

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

    if (getenv("EPOCH5250_ALT_PROBE"))
        io_sethandler(0x0100, 0x0020,
                      ibm5250_alt_probe_in, NULL, NULL,
                      ibm5250_alt_probe_out, NULL, NULL, dev);

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
