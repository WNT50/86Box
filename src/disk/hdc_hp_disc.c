/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          HP Vectra hard disc adapter (the 45816A/45817A generation of
 *          HP "intelligent" controllers).
 *
 *          The model is taken from the OS/2 1.1 driver HP shipped for this
 *          adapter (DISKHP1.SYS), disassembled and documented in
 *          documents/86box/os2drv/REPORT.md, and from the INT 13h front end
 *          that lives in the adapter's own option ROM (hp_disc_rom.asm).
 *
 *          Register map (base = 0x0300):
 *
 *            base + 0   write command (0x00 reset, 0x20 read, 0x30 write)
 *                       read status (bit 7 = busy, bit 0 = error)
 *            base + 1   write sector count, read error code
 *            base + 2   write LBA bits 7:0    (also the legacy packet port)
 *            base + 3   write LBA bits 15:8
 *            base + 4   write LBA bits 23:16
 *            base + 5   write drive
 *            base + 6   data port, 16-bit (insw / outsw)
 *
 *          The 0x02..0x04 command range keeps the pass-through "packet"
 *          behaviour the OS/2 driver uses once it starts talking to the
 *          adapter; the 0x20/0x30 range is what the option ROM uses.
 *
 *          The driver finds the adapter by scanning the option ROM area
 *          (0xC8000-0xDFFFF, 8 KB steps) for a "WINSTON" signature at
 *          window+0x1848 and then reads the port base, the drive count and
 *          the drive parameter table out of the ROM.  This device supplies
 *          that ROM (see roms/machines/hpvectra/hp_disc_option_rom.bin and
 *          tools/mk_hp_disc_rom.py), so the model can be validated against
 *          the real discovery path.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/config.h>
#include <86box/ini.h>
#include <86box/mem.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include "../cpu/cpu.h"   /* 调试探针：读 cpu_state.seg_ds.base */
#include <86box/rom.h>
#include <86box/timer.h>
#include <86box/pic.h>
#include <86box/hdc.h>
#include <86box/hdd.h>
#include <86box/plat.h>

#define HP_DISC_ROM_PATH   "roms/machines/hpvectra/hp_disc_option_rom.bin"
#define HP_DISC_ROM_ADDR   0x000c8000
#define HP_DISC_ROM_SIZE   0x4000

/* The option ROM carries no IRQ field: DISKHP1.SYS masks IRQ14 itself
   (0x276D opens "master IRQ2 + slave IRQ6" before every command).  The
   port base it does read, from ROM +0x186A, is whatever the generator
   writes there - see mk_hp_disc_rom.py. */

#define HP_DISC_PACKET_WORDS 10
#define HP_DISC_PACKET_BYTES (HP_DISC_PACKET_WORDS * 2)
#define HP_DISC_MAX_SECTORS  255  /* FORMAT 用 17/40 扇区，拷贝用 125 扇区(64000B) */

enum {
    HP_DISC_CMD_RESET = 0x00,
    HP_DISC_CMD_READ  = 0x20,
    HP_DISC_CMD_WRITE = 0x30,
    HP_DISC_CMD_DATA  = 0x03
};

typedef struct hp_disc_t {
    uint16_t base;
    int      irq;

    uint8_t status;
    uint8_t cmd;
    uint8_t error;

    /* block interface (option ROM / INT 13h) */
    uint8_t  count;
    uint32_t lba;
    uint8_t  drive;
    uint8_t  buffer[512 * HP_DISC_MAX_SECTORS];
    uint32_t buffer_len;
    uint32_t buffer_pos;
    int      hdd_id;
    int      present;
    int      spt, hpc, tracks;

    /* legacy packet interface (OS/2 driver) */
    uint16_t packet[HP_DISC_PACKET_WORDS];
    uint8_t  packet_pos;
    uint8_t  wbuf[512 * HP_DISC_MAX_SECTORS];
    uint32_t wpos;
    uint8_t  phase;
    int      xfer_left;
    int      req_len;
    int      block_ok;   /* 命令块里声明的整笔字节数（只做日志/参考） */   /* 本笔请求还剩多少扇区 */

    /* 短命令（2 字节，如 0xFF0D）：应答是 20 字节状态块，不走扇区通路 */
    int      short_len;
    uint8_t  shortbuf[20];
    uint8_t  data_lo;
    uint8_t  data_hi;

    int      trace_ops;
    int      data_ready;
    uint8_t  unit;
    uint8_t  opcode;
    int      trace_all;
    int      tracing;
    uint64_t packets;
    uint64_t reads;
    uint64_t writes;
    uint64_t irqs;

    pc_timer_t timer;
    pc_timer_t pc_timer;   /* HP_DISC_PC=<us>：周期性采 CS:IP */
    int        pc_us;
    uint8_t      shadow[HP_DISC_ROM_SIZE];
    mem_mapping_t mapping;
} hp_disc_t;

static void
hp_disc_raise_irq(hp_disc_t *dev)
{
    /* HP_DISC_NOIRQ breaks the line for debugging: the option ROM polls
       status, but a guest that never reads it would otherwise leave the
       PIC line asserted and storm IRQ7 */
    if (getenv("HP_DISC_NOIRQ"))
        return;

    dev->irqs++;
    picint(1 << dev->irq);
}

static uint8_t
hp_disc_rom_read(uint32_t addr, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    return dev->shadow[(addr - HP_DISC_ROM_ADDR) & (HP_DISC_ROM_SIZE - 1)];
}

static void
hp_disc_rom_write(uint32_t addr, uint8_t val, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    dev->shadow[(addr - HP_DISC_ROM_ADDR) & (HP_DISC_ROM_SIZE - 1)] = val;
}

static void
hp_disc_complete(void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    /* busy clear, done set, "ok" pattern in bits 3:2 */
    dev->status = 0x40 | 0x0c;

    if (dev->tracing)
        pclog("hp_disc: command %02x completed, status %02x\n", dev->cmd, dev->status);

    hp_disc_raise_irq(dev);
}


/* DISKHP1 的命令块（20 字节，按字节序）：
   b0      = 0x20 | 单元号
   b1      = 0x10
   b2,3    = 0
   b4..b7  = LBA，32 位大端（扇区）
   b8      = 0x18（长度字段标签）
   b9..b12 = 长度，32 位大端（字节）
   b13..   = 状态相关尾字                                              */
/* HP_DISC_PIO：逐次打印 base+0/base+2 上的读写（方向、字节数、当时 phase），
   用来判定 DISKHP1 的命令/数据方向。 */
static int
hp_disc_pio_on(void)
{
    static int on = -1;
    if (on < 0)
        on = (getenv("HP_DISC_PIO") != NULL) ? 1 : 0;
    return on;
}

static void
hp_disc_parse_packet(hp_disc_t *dev)
{
    uint8_t *p = (uint8_t *) dev->packet;
    uint32_t len;

    dev->unit   = p[0] & 0x0f;
    dev->opcode = p[0] & 0xf0;
    dev->lba    = ((uint32_t) p[4] << 24) | ((uint32_t) p[5] << 16) |
                  ((uint32_t) p[6] << 8) | (uint32_t) p[7];
    len = ((uint32_t) p[9] << 24) | ((uint32_t) p[10] << 16) |
          ((uint32_t) p[11] << 8) | (uint32_t) p[12];
    /* 实测：驱动每一笔都是「一个扇区 + 一次命令」，长度字段只是整笔请求
       的字节数（FORMAT 会填 8192）。按长度去等 8192 字节会让驱动在第一个
       512 字节后就卡在等中断上（SYS0049）。所以每笔固定一扇区。 */
    /* 合法命令块的三处常量（0x262D 的构造代码固定在 b1=0x10、b8=0x18），
       再加长度/LBA 范围检查。实测格式化完成后的拷贝阶段，偶尔会把一段
       8086 代码（b0=0x2B、b1=0x10…）当命令块送进来，若不拦，LBA 会跑到
       0x81FBF7xx，把整块镜像写坏。 */
    dev->block_ok = ((p[1] == 0x10) && (p[8] == 0x18) &&
                     (len != 0) && ((len & 511) == 0) &&
                     (len <= (uint32_t) (HP_DISC_MAX_SECTORS * 512)) &&
                     (dev->hdd_id >= 0) &&
                     (dev->lba < (uint32_t) (dev->spt * dev->hpc * dev->tracks)));
    if (!dev->block_ok) {
        if (dev->tracing)
            pclog("hp_disc: invalid block (b1=%02x b8=%02x len=%u lba=%06x) -> ignored\n",
                  p[1], p[8], len, dev->lba);
        dev->count = 1;
        return;
    }

    dev->req_len = (int) len;   /* 仅日志用 */
    dev->count   = (int) (len / 512);
    if (dev->count == 0)
        dev->count = 1;
    if (dev->count > HP_DISC_MAX_SECTORS)
        dev->count = HP_DISC_MAX_SECTORS;
}

static void
hp_disc_pc_cb(void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;
    static uint32_t last = 0xffffffff;
    uint32_t pc = (cpu_state.seg_cs.base << 4) + cpu_state.pc;

    if (pc != last) {
        last = pc;
        fprintf(stderr, "PC %05x:%04x fl34=%02x model[ph=%d st=%02x left=%d pos=%u reads=%u irq=%u]\n",
                cpu_state.seg_cs.base, cpu_state.pc,
                mem_readb_phys(cpu_state.seg_ds.base + 0x0e34),
                dev->phase, dev->status, dev->xfer_left, dev->packet_pos,
                dev->reads, dev->irqs);
    }
    timer_set_delay_u64(&dev->pc_timer, (uint64_t) dev->pc_us * TIMER_USEC);
}

static int
hp_disc_floppy_media(void)
{
    /* HP_DISC_FORCEA: bit0 = 把引导权交还 BIOS；bit1 = 同时不挂钩 INT 13h */
    {
        const char *e = getenv("HP_DISC_FORCEA");
        if (e != NULL) {
            int v = atoi(e);
            return v ? v : 1;
        }
    }

    {
        /* the section moved between 86Box versions; accept both */
        const char *fn = config_get_string("Floppy and CD-ROM drives", "fdd_01_fn", "");
        if ((fn == NULL) || (fn[0] == '\0'))
            fn = config_get_string("General", "fdd_01_fn", "");
        return (fn != NULL) && (fn[0] != '\0');
    }
}

static void
hp_disc_block(hp_disc_t *dev)
{
    uint32_t sectors = dev->count ? dev->count : 1;

    if (sectors > HP_DISC_MAX_SECTORS)
        sectors = HP_DISC_MAX_SECTORS;

    dev->error      = 0;
    dev->buffer_len = sectors * 512;
    dev->buffer_pos = 0;

    /* The drive list is filled after the controller is instantiated, so the
       attachment is resolved on first use rather than at init time. */
    if (dev->hdd_id < 0) {
        dev->error  = 0x04;
        dev->status = 0x01;
        return;
    }

    if (dev->cmd != HP_DISC_CMD_WRITE) {
        if (hdd_image_read((uint8_t) dev->hdd_id, dev->lba, sectors, dev->buffer) < 0) {
            dev->error  = 0x04;
            dev->status = 0x01;
            return;
        }
        dev->reads++;
    } else {
        memset(dev->buffer, 0, dev->buffer_len);
        dev->writes++;
    }

    dev->status = 0x00;
}

/* ------------------------------------------------------------------
 * 状态机
 *
 *   IDLE   等命令
 *   PACKET base+2 收 10 word 命令包
 *   ARMED  包已解析，等「第一个数据口访问」来定向（读还是写）
 *   IN     主机读：从 buffer 喂数据
 *   OUT    主机写：往 wbuf 收数据
 *
 * 方向不靠猜包尾 trailer：DISKHP1 的读走 0x25CA(rep insw)、写走
 * 0x249E(rep outsw)，口上先出现哪种访问就是哪个方向。
 * ------------------------------------------------------------------ */
enum {
    HP_PHASE_IDLE = 0,
    HP_PHASE_PACKET,
    HP_PHASE_ARMED,
    HP_PHASE_IN,
    HP_PHASE_OUT
};

/* 数据口一次搬动的字节数。DISKHP1 的搬运用例（0x249E / 0x25CA）里
   cx 固定是 0x100（512 字节），一个扇区一次，所以这里不按包里的长度字段走
   —— 那个字段在写请求里给的是整笔请求的长度（实测 0x2000=16 扇区），
   跟每次实际搬动的量不一致。 */
static int
hp_disc_xfer_len(hp_disc_t *dev)
{
    if (dev->short_len > 0)
        return dev->short_len;

    return 512;
}

/* 包解析完 → ARMED。bit7 = 数据口可用（驱动 0x2575 / 0x2606 都只看这一位）；
   同时 0xC0 & 0x4C = 0x40 != 0x4C，不会被 0x2727 误判成「已完成」。 */
static void
hp_disc_arm(hp_disc_t *dev)
{
    dev->phase      = HP_PHASE_ARMED;
    dev->buffer_pos = 0;
    dev->wpos       = 0;
    dev->data_ready = 0;
    dev->xfer_left  = dev->count ? dev->count : 1;
}

/* 读一个扇区进 buffer（每次数据口访问只搬 512 字节） */
static void
hp_disc_load_sector(hp_disc_t *dev)
{
    dev->error      = 0;
    dev->buffer_len = 512;
    dev->buffer_pos = 0;

    if (dev->hdd_id < 0) {
        dev->error  = 0x04;
        dev->status = 0x01;
        return;
    }

    if (hdd_image_read((uint8_t) dev->hdd_id, dev->lba, 1, dev->buffer) < 0) {
        dev->error  = 0x04;
        dev->status = 0x01;
        return;
    }
    dev->reads++;
    dev->status = 0x00;
}

static void
hp_disc_begin_in(hp_disc_t *dev)
{
    if (dev->short_len > 0) {
        /* 短命令应答：20 字节状态块（DISKHP1 0x22CE 里 rep insw cx=0x0A） */
        memcpy(dev->buffer, dev->shortbuf, dev->short_len);
        dev->buffer_len = (uint32_t) dev->short_len;
        dev->buffer_pos = 0;
        dev->data_ready = 1;
        dev->status     = 0xcc;
        if (dev->tracing)
            pclog("hp_disc: short reply %i bytes\n", dev->short_len);
        return;
    }

    hp_disc_load_sector(dev);

    dev->phase      = HP_PHASE_IN;
    dev->buffer_pos = 0;
    dev->data_ready = 1;
    dev->status     = 0xcc;   /* bit7 数据就绪 | 0x0C */

    if (dev->tracing)
        pclog("hp_disc: data in 512 bytes lba=%06x\n", dev->lba);
}

static void
hp_disc_begin_out(hp_disc_t *dev)
{
    dev->phase      = HP_PHASE_OUT;
    dev->wpos       = 0;
    dev->buffer_pos = 0;
    dev->status     = 0xc0;

    if (dev->tracing)
        pclog("hp_disc: data out %i bytes lba=%06x\n", hp_disc_xfer_len(dev), dev->lba);
}

/* 一个 512 字节块搬完。整笔请求还有扇区就继续留在 OUT，
   全部搬完才回 IDLE + 完成状态 + 中断。 */
static void
hp_disc_end_out(hp_disc_t *dev)
{
    if ((dev->hdd_id >= 0) &&
        (dev->lba < (uint32_t) (dev->spt * dev->hpc * dev->tracks)))
        hdd_image_write((uint8_t) dev->hdd_id, dev->lba, 1, dev->wbuf);

    dev->writes++;
    dev->lba++;

    if (dev->tracing)
        pclog("hp_disc: write 512 bytes lba=%06x done (left %i)\n",
              dev->lba - 1, dev->xfer_left - 1);

    if (--dev->xfer_left > 0) {
        dev->wpos   = 0;               /* 继续同一笔请求的下一个扇区 */
        /* 0xCC = bit7（数据口就绪，0x2575 要）+ 0x4C（0x2727 要的完成位）。
           只给 0xC0 会让 0x2727 判「没完成」，驱动立刻转状态 0x17 报错。
           ISR(0x1B6F) 驱动的状态机每步只搬一个扇区，中断是推进的唯一手段
           （0x27D4 只是重试计数，不阻塞），所以每个扇区都要给一次中断。 */
        dev->status = 0xcc;
        if (dev->tracing)
            pclog("hp_disc: >> IRQ(sector) lba=%06x left=%i\n",
                  dev->lba, dev->xfer_left);
        if (!getenv("HP_DISC_NOIRQ"))
            hp_disc_raise_irq(dev);
        return;
    }

    dev->phase      = HP_PHASE_IDLE;
    dev->packet_pos = 0;
    /* 整笔写完：bit7 表示「命令完成」，驱动 0x2575 就等这一位；
       给 0x4C 会让它空转 180 万次后置 [0x292]=0xFF 报错。 */
    dev->status     = 0xcc;

    if (!getenv("HP_DISC_NOIRQ"))
        hp_disc_raise_irq(dev);
}

static uint8_t
hp_disc_in(uint16_t addr, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;
    if (hp_disc_pio_on()) pclog("PIO %04x:%04x inb%i ph=%d pos=%u left=%d\n", cpu_state.seg_cs.base, cpu_state.pc, addr - dev->base, dev->phase, dev->packet_pos, dev->xfer_left);
    int        len;

    switch (addr - dev->base) {
        case 0:
            /* reading the status port acknowledges the interrupt。
               HP_DISC_NOACK：不在这里撤销，留给 EOI——驱动在发命令前也会
               读一次状态（0x2727），那一下会把还没被 CPU 取走的中断撤掉。 */
            if (!getenv("HP_DISC_NOACK"))
                picintc(1 << dev->irq);
            if (dev->tracing && (dev->trace_ops > 0)) {
                dev->trace_ops--;
                pclog("hp_disc:   in  %02x -> %02x\n", addr - dev->base, dev->status);
            }
            return dev->status;
        case 1:
            return dev->error;
        case 2:
        case 3:
            if (dev->phase == HP_PHASE_ARMED)
                hp_disc_begin_in(dev);
            if (dev->phase == HP_PHASE_IN) {
                uint8_t v = 0xff;

                len = hp_disc_xfer_len(dev);
                if (dev->buffer_pos < (uint32_t) len)
                    v = dev->buffer[dev->buffer_pos++];
                if (dev->buffer_pos >= (uint32_t) len) {
                    dev->lba++;
                    if (--dev->xfer_left > 0) {
                        hp_disc_load_sector(dev);
                    } else {
                        dev->phase      = HP_PHASE_IDLE;
                        dev->data_ready = 0;
                        dev->packet_pos = 0;
                        dev->status     = 0xcc;
                    }
                }
                return v;
            }
            if (dev->tracing)
                pclog("hp_disc:   inb%i (phase %i)\n", addr - dev->base, dev->phase);
            return (addr - dev->base) ? dev->data_hi : dev->data_lo;
        case 6:
            if (dev->buffer_pos + 1 < dev->buffer_len) {
                uint8_t v = dev->buffer[dev->buffer_pos];
                dev->buffer_pos += 2;
                return v;
            }
            return 0xff;
        case 7:
            /* live: the option ROM's int19 asks this right before it decides
               whether to chain to the BIOS (A: bootable) or boot the MBR */
            return hp_disc_floppy_media() ? 0x01 : 0x00;
        default:
            break;
    }

    return 0xff;
}

static uint16_t
hp_disc_inw(uint16_t addr, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;
    if (hp_disc_pio_on()) pclog("PIO %04x:%04x inw%i ph=%d pos=%u left=%d\n", cpu_state.seg_cs.base, cpu_state.pc, addr - dev->base, dev->phase, dev->packet_pos, dev->xfer_left);
    int        len;

    if (dev->tracing && (dev->trace_all < 4000)) {
        dev->trace_all++;
        pclog("hp_disc: INW%i (phase %i)\n", addr - dev->base, dev->phase);
    }

    switch (addr - dev->base) {
        case 2:
            if (dev->phase == HP_PHASE_ARMED)
                hp_disc_begin_in(dev);
            if (dev->phase != HP_PHASE_IN)
                return 0xffff;

            len = hp_disc_xfer_len(dev);
            {
                uint16_t v = 0xffff;

                if (dev->buffer_pos + 1 < (uint32_t) len)
                    v = (uint16_t) (dev->buffer[dev->buffer_pos] |
                                    (dev->buffer[dev->buffer_pos + 1] << 8));
                dev->buffer_pos += 2;
                if (dev->tracing && (dev->buffer_pos <= 8 || dev->buffer_pos >= (uint32_t) len))
                    pclog("hp_disc:   data[%04x] = %04x\n", dev->buffer_pos - 2, v);
                if (dev->buffer_pos >= (uint32_t) len) {
                    dev->lba++;
                    if (dev->short_len > 0) {
                        dev->short_len  = 0;
                        dev->phase      = HP_PHASE_IDLE;
                        dev->data_ready = 0;
                        dev->packet_pos = 0;
                        dev->status     = 0xcc;
                    } else if (--dev->xfer_left > 0) {
                        hp_disc_load_sector(dev);   /* 下一扇区，仍留在 IN */
                        dev->status = 0xcc;
                        /* 读侧与写侧同构：每搬完一个扇区都要中断一次，
                           驱动的状态机靠 ISR 才能进到下一扇区。 */
                        if (dev->tracing)
                            pclog("hp_disc: >> IRQ(read) lba=%06x left=%i\n",
                                  dev->lba, dev->xfer_left);
                        if (!getenv("HP_DISC_NOIRQ"))
                            hp_disc_raise_irq(dev);
                    } else {
                        dev->phase      = HP_PHASE_IDLE;
                        dev->data_ready = 0;
                        dev->packet_pos = 0;
                        dev->status     = 0xcc;
                        if (dev->tracing)
                            pclog("hp_disc: read request done at lba %06x\n", dev->lba - 1);
                    }
                }
                return v;
            }
        case 6:
            if (dev->buffer_pos + 1 < dev->buffer_len) {
                uint16_t v = (uint16_t) (dev->buffer[dev->buffer_pos] |
                                         (dev->buffer[dev->buffer_pos + 1] << 8));
                dev->buffer_pos += 2;
                if (dev->tracing && (dev->trace_ops > 0)) {
                    dev->trace_ops--;
                    pclog("hp_disc:   inw 6 -> %04x\n", v);
                }
                return v;
            }
            return 0xffff;
        default:
            break;
    }

    return 0xffff;
}

static void
hp_disc_out(uint16_t addr, uint8_t val, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;
    if (hp_disc_pio_on()) pclog("PIO %04x:%04x outb%i <= %02x ph=%d pos=%u left=%d\n", cpu_state.seg_cs.base, cpu_state.pc, addr - dev->base, val, dev->phase, dev->packet_pos, dev->xfer_left);
    int        len;
    uint16_t   shmem_len = 0;

    if (dev->tracing && (dev->trace_all < 260)) {
        dev->trace_all++;
        pclog("hp_disc:   W%i <- %02x\n", addr - dev->base, val);
    }

    switch (addr - dev->base) {
        case 0:
            if (dev->tracing) {
                uint32_t dseg = cpu_state.seg_ds.base;
                pclog("hp_disc: >CS base=%05x ip=%04x DS st=%02x len=%04x err=%02x blk0=%04x"
                      " unit=%02x lba=%02x%02x%02x%02x sec=%02x fl=%04x irqs=%04x flags=%02x\n",
                      cpu_state.seg_cs.base, cpu_state.pc,

                      mem_readb_phys(dseg + 0x210),
                      mem_readw_phys(dseg + 0x282),
                      mem_readb_phys(dseg + 0x292),
                      mem_readw_phys(dseg + 0x264),
                      mem_readb_phys(dseg + 0x22d),
                      mem_readb_phys(dseg + 0x22e),
                      mem_readb_phys(dseg + 0x22f),
                      mem_readb_phys(dseg + 0x230),
                      mem_readb_phys(dseg + 0x231),
                      mem_readb_phys(dseg + 0x232),
                      mem_readw_phys(dseg + 0x218),
                      mem_readw_phys(dseg + 0x1f8),
                      mem_readb_phys(dseg + 0x34e));
            }
            dev->cmd   = val;
            dev->error = 0;
            /* 任何命令都终止上一笔数据请求：驱动每搬完一个扇区都会发
               0x04 之类收尾命令，然后再送下一个命令包。不在这里复位，
               下一个包的字节会被当成数据吃掉（口失步）。 */
            dev->xfer_left = 0;
            if ((val != 0x02) && (val != HP_DISC_CMD_DATA) && (val != 0x01)) {
                dev->phase      = HP_PHASE_IDLE;
                dev->packet_pos = 0;   /* 收尾命令 = 包边界，清掉残留 */
            }

            if (dev->tracing) {
                pclog("hp_disc: command %02x (unit %i op %02x count %i lba %06x)\n",
                      val, dev->unit, dev->opcode, dev->count, dev->lba);
                dev->trace_ops = 40;
            }

            switch (val) {
                case HP_DISC_CMD_RESET:
                    dev->phase      = HP_PHASE_IDLE;
                    dev->packet_pos = 0;
                    dev->status     = 0x00;
                    break;

                case 0x01:
                    /* DISKHP1 的顺序是「先 rep outsw 送包、再 out 01」
                       （0x26F9），所以包字节在 IDLE 阶段就已经暂存好了，
                       这里只负责解析 + 完成 + 进入 ARMED 等数据方向。 */
                    /* 按内容判断「短命令」，不要按长度猜：正常的命令块是
                       15 或 20 字节（[0x282] 由 0x262D 动态算），只有
                       状态/请求类命令才是 2 字节的 0xFF0D。 */
                    if ((dev->packet_pos == 2) &&
                        (((uint8_t *) dev->packet)[1] == 0xff)) {
                        /* 2 字节短命令（0xFF0D，DISKHP1 0x2223）：应答 20 字节状态块。
                           全 0 表示「命令接受、无错误」——驱动的错误位图在 0x23AA 解析，
                           8 个状态字节全 0 走的就是无错分支。 */
                        uint16_t sc = dev->packet[0];

                        memset(dev->shortbuf, 0, sizeof(dev->shortbuf));
                        dev->short_len  = 20;
                        dev->xfer_left  = 1;
                        dev->packet_pos = 0;
                        dev->phase      = HP_PHASE_ARMED;
                        dev->status     = 0xcc;

                        if (dev->tracing)
                            pclog("hp_disc: short command %04x -> 20-byte status\n", sc);
                        if (!getenv("HP_DISC_NOIRQ"))
                            hp_disc_raise_irq(dev);
                        break;
                    }
                    if (dev->tracing) {
                        uint8_t *pb = (uint8_t *) dev->packet;
                        /* 用 stderr：pclog 的重复行压缩会把不同字节的块合并掉 */
                        fprintf(stderr, "hp_disc: BLK");
                        for (int i = 0; i < dev->packet_pos && i < 20; i++)
                            fprintf(stderr, " %02x", pb[i]);
                        fprintf(stderr, "\n");
                    }
                    if (dev->tracing) {
                        pclog("hp_disc: PKT pos=%i:", dev->packet_pos);
                        for (int i = 0; i < HP_DISC_PACKET_WORDS; i++)
                            pclog(" %04x", dev->packet[i]);
                        pclog("\n");
                    }
                    if (dev->tracing) {
                        /* 同一时刻把驱动自己的命令块（DS:0x264，长度 [0x282]）打出来，
                           与我们收到的字节逐字节对照：判断「b8=0xC3 的代码块」到底是
                           驱动真送了它，还是我们暂存错位。 */
                        uint32_t dseg = cpu_state.seg_ds.base;
                        shmem_len = mem_readw_phys(dseg + 0x282);
                        if (shmem_len > 32)
                            shmem_len = 32;
                        fprintf(stderr, "hp_disc: DRV %04x:", shmem_len);
                        for (int i = 0; i < shmem_len; i++)
                            fprintf(stderr, " %02x", mem_readb_phys(dseg + 0x264 + i));
                        fprintf(stderr, "\n");
                    }
                    if (getenv("HP_DISC_BLK")) {
                        uint8_t *b = (uint8_t *) dev->packet;
                        uint32_t dseg2 = cpu_state.seg_ds.base;
                        fprintf(stderr, " [reads=%u writes=%u status=%02x phase=%d left=%d]",
                                dev->reads, dev->writes, dev->status, dev->phase, dev->xfer_left);
                        fprintf(stderr, "C1 %06x ip=%04x pp=%i drv_unit=%02x drv_lba=%02x%02x%02x%02x :",
                                cpu_state.pc, cpu_state.pc, dev->packet_pos,
                                mem_readb_phys(dseg2 + 0x22d),
                                mem_readb_phys(dseg2 + 0x22e), mem_readb_phys(dseg2 + 0x22f),
                                mem_readb_phys(dseg2 + 0x230), mem_readb_phys(dseg2 + 0x231));
                        for (int i = 0; i < 16; i++)
                            fprintf(stderr, " %02x", b[i]);
                        fprintf(stderr, "\n");
                    }
                    hp_disc_parse_packet(dev);
                    dev->packet_pos = 0;
                    dev->packets++;
                    dev->status = 0xcc;
                    if (!getenv("HP_DISC_NOIRQ"))
                        hp_disc_raise_irq(dev);
                    if (dev->block_ok)
                        hp_disc_arm(dev);  /* 状态仍是 0x4C，驱动 0x2727 能过 */
                    break;

                case 0x02:
                case HP_DISC_CMD_DATA:
                    /* 短命令（0xFF0D）的应答也走 cmd 03：这里必须把 bit7 立起来，
                       否则驱动 0x2575 会空转几十次才硬读（日志里 *** 39 repeats ***）。 */
                    if (dev->short_len > 0) {
                        dev->phase      = HP_PHASE_ARMED;
                        dev->buffer_pos = 0;
                        dev->status     = 0xc0;
                        if (!getenv("HP_DISC_NOIRQ"))
                            hp_disc_raise_irq(dev);
                        break;
                    }
                    /* 数据阶段：只 arm，方向等第一个数据口访问。
                       非法块（block_ok=0）不 arm，否则会拿垃圾 LBA 去写磁盘。 */
                    if (!dev->block_ok) {
                        dev->status = 0x4c;
                        if (!getenv("HP_DISC_NOIRQ"))
                            hp_disc_raise_irq(dev);
                        break;
                    }
                    hp_disc_arm(dev);
                    dev->status = 0xc0;
                    if (!getenv("HP_DISC_NOIRQ"))
                        hp_disc_raise_irq(dev);
                    break;

                case HP_DISC_CMD_READ:
                    dev->phase = HP_PHASE_IDLE;
                    hp_disc_block(dev);
                    break;

                case HP_DISC_CMD_WRITE:
                    dev->phase = HP_PHASE_IDLE;
                    hp_disc_block(dev);
                    break;

                default:
                    dev->phase = HP_PHASE_IDLE;
                    timer_set_delay_u64(&dev->timer, 20 * TIMER_USEC);
                    break;
            }
            break;

        case 1:
            dev->count = val;
            break;

        case 2:
        case 3:
            /* 写数据 */
            if (dev->phase == HP_PHASE_ARMED)
                hp_disc_begin_out(dev);
            if (dev->phase == HP_PHASE_OUT) {
                len = hp_disc_xfer_len(dev);
                if (dev->wpos < (uint32_t) len)
                    dev->wbuf[dev->wpos++] = val;
                if (dev->wpos >= (uint32_t) len)
                    hp_disc_end_out(dev);
                break;
            }

            /* IDLE：既当 LBA 寄存器（ROM 前端），也当命令包暂存区 */
            dev->lba = (dev->lba & ~(0xffu << (8 * (addr - dev->base - 2)))) |
                       ((uint32_t) val << (8 * (addr - dev->base - 2)));
            if (dev->packet_pos < HP_DISC_PACKET_BYTES) {
                ((uint8_t *) dev->packet)[dev->packet_pos] = val;
                dev->packet_pos++;
            }
            break;

        case 4:
            dev->lba = (dev->lba & ~0x00ff0000u) | ((uint32_t) val << 16);
            break;

        case 5:
            dev->drive = val;
            break;

        case 7:
            /* base+7 doubles as the "A: mounted" status the option ROM reads
               before it picks a bootstrap device, so a write here must not
               clobber what the read side reports. */
            break;

        case 6: {
            /* The data port is 16-bit: the option ROM's `rep outsw` and the
               OS/2 driver both move one word per access, so the byte pair
               goes to wpos and wpos+1. */
            if (dev->buffer_pos + 1 < dev->buffer_len) {
                dev->buffer[dev->buffer_pos]     = (uint8_t) (val & 0xff);
                dev->buffer[dev->buffer_pos + 1] = (uint8_t) (val >> 8);
                dev->buffer_pos += 2;
                if ((dev->buffer_pos >= dev->buffer_len) &&
                    (dev->cmd == HP_DISC_CMD_WRITE)) {
                    if (dev->hdd_id >= 0)
                        hdd_image_write((uint8_t) dev->hdd_id, dev->lba,
                                        dev->buffer_len / 512, dev->buffer);
                    dev->status = 0x00;
                }
            }
            break;
        }

        default:
            break;
    }
}

static void
hp_disc_outw(uint16_t addr, uint16_t val, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;
    int        len;
    uint16_t   shmem_len = 0;

    if (dev->tracing && (dev->trace_all < 4000)) {
        dev->trace_all++;
        pclog("hp_disc: OUTW%i (phase %i) <- %04x\n", addr - dev->base, dev->phase, val);
    }

    switch (addr - dev->base) {
        case 2:
            /* 命令包的首字永远是 0x1020（字节 20 10 = op 0x20|unit + tag 0x10）。
               只要看到它而且暂存指针还在 0，就无条件按「新命令包开始」处理——
               不管当前处于哪个阶段。否则上一笔请求残留的 OUT 会把包头吃掉，
               后面的字节全部错位（实测会解析出 lba=0x121800 这类垃圾）。 */
            /* 中断里驱动会往数据口写一批 0x0000（应答/清缓冲），它不是命令块；
               在暂存指针为 0 时只接受两种首字：0x1020（正常命令块）和
               0xFF0D（2 字节状态请求）。其它一律丢掉，避免 20 个 0 把包边界
               挤掉，导致后面的真块被解析成 lba=0。 */
            /* ARMED 且还没搬过任何字节时，数据口来的字不可能是搬运起点：
               DISKHP1 的搬运用例固定 cx=0x100（一个扇区一次）。上一笔读命令
               收尾后驱动会直接发下一个命令包（0xFF0D 状态查询），若把它当写
               数据，这笔读就永远收不了尾、包边界也跟着错位（实测 pos 停在
               14、后续每包只剩 1 字节）。 */
            if ((dev->phase == HP_PHASE_ARMED) && (dev->wpos == 0) &&
                (dev->buffer_pos == 0) &&
                (((val & 0xfff0) == 0x1020) || (val == 0xff0d))) {
                if (dev->tracing)
                    pclog("hp_disc: ARMED -> new packet %04x\n", val);
                dev->phase      = HP_PHASE_IDLE;
                dev->packet_pos = 0;
                dev->wpos       = 0;
                dev->xfer_left  = 0;
            }

            if ((dev->phase == HP_PHASE_IDLE) && (dev->packet_pos == 0) &&
                (((val & 0xfff0) != 0x1020) && (val != 0xff0d))) {
                if (dev->tracing)
                    pclog("hp_disc: drop %04x (not a block start)\n", val);
                break;
            }

            /* 注意：0x1020 的掩码必须包含 unit 低半字节，但**不能**在 OUT 阶段
               生效。文件数据里随时可能出现 0x102X 这样的字（实测就是
               `2b 10` = 0x102b，一段 8086 代码），一旦被当成新块首字就把
               传输中剩下的数据全灌进命令包暂存，解析出 lba=0x81FBF70F。 */
            if (((val & 0xfff0) == 0x1020) && (dev->packet_pos == 0) &&
                (dev->phase != HP_PHASE_IN) && (dev->phase != HP_PHASE_OUT)) {
                dev->phase     = HP_PHASE_IDLE;
                dev->xfer_left = 0;
                dev->wpos      = 0;
            }

            /* IDLE：命令包暂存（DISKHP1 用 rep outsw 先送包，末字节单独 out） */
            if (dev->phase == HP_PHASE_IDLE) {
                uint8_t *pp = (uint8_t *) dev->packet;

                if (dev->tracing && (dev->trace_all < 400))
                    pclog("hp_disc: PKTW pos=%i <- %04x\n", dev->packet_pos, val);
                if (dev->packet_pos + 1 < HP_DISC_PACKET_BYTES) {
                    pp[dev->packet_pos]     = (uint8_t) (val & 0xff);
                    pp[dev->packet_pos + 1] = (uint8_t) (val >> 8);
                    dev->packet_pos += 2;
                }
                break;
            }

            /* 写数据（0x249E 的 rep outsw） */
            if (dev->phase == HP_PHASE_ARMED)
                hp_disc_begin_out(dev);
            if (dev->phase == HP_PHASE_OUT) {
                len = hp_disc_xfer_len(dev);
                if (dev->wpos < (uint32_t) len)
                    dev->wbuf[dev->wpos++] = (uint8_t) (val & 0xff);
                if (dev->wpos < (uint32_t) len)
                    dev->wbuf[dev->wpos++] = (uint8_t) (val >> 8);
                if (dev->wpos >= (uint32_t) len)
                    hp_disc_end_out(dev);
            }
            break;

        case 6:
            if (dev->buffer_pos + 1 < dev->buffer_len) {
                dev->buffer[dev->buffer_pos]     = (uint8_t) (val & 0xff);
                dev->buffer[dev->buffer_pos + 1] = (uint8_t) (val >> 8);
                dev->buffer_pos += 2;
            }
            if ((dev->buffer_pos >= dev->buffer_len) && (dev->cmd == HP_DISC_CMD_WRITE)) {
                if (dev->hdd_id >= 0)
                    hdd_image_write((uint8_t) dev->hdd_id, dev->lba,
                                    dev->buffer_len / 512, dev->buffer);
                dev->status = 0x00;
            }
            break;

        default:
            break;
    }
}

static void *
hp_disc_init(const device_t *info)
{
    hp_disc_t *dev = (hp_disc_t *) calloc(1, sizeof(hp_disc_t));

    dev->base    = (uint16_t) info->local;
    /* DISKHP1 每次发命令前 0x276D 打开的是「主片 bit2（级联）+ 从片 bit6」，
       命令收尾再把 IRQ14 掩回去——即这块适配器的中断线是 IRQ14，不是 IRQ5。
       默认给 5 时驱动侧 [0x1f8] 中断计数永远是 0（实测），命令全部超时。 */
    dev->irq     = getenv("HP_DISC_IRQ") ? atoi(getenv("HP_DISC_IRQ")) : 14;
    dev->status  = 0x00;
    dev->tracing = (getenv("HP_DISC_TRACE") != NULL);
    dev->hdd_id  = -1;

    /* 认领挂在 MFM 通道上的驱动器。做法与 hdc_st506_wd10.c 一致：
       bus_type 必须是 MFM、mfm_channel 在范围内，然后 hdd_image_load()
       真正打开镜像 —— 没有这一步 hdd_image_read() 一定失败。 */
    for (uint8_t i = 0; i < HDD_NUM; i++) {
        if ((hdd[i].bus_type != HDD_BUS_MFM) || (hdd[i].mfm_channel >= MFM_NUM))
            continue;

        if (!hdd_image_load(i))
            continue;

        dev->hdd_id  = i;
        dev->present = 1;
        dev->spt     = hdd[i].spt;
        dev->hpc     = hdd[i].hpc;
        dev->tracks  = hdd[i].tracks;
        break;
    }

    timer_add(&dev->timer, hp_disc_complete, dev, 0);
    timer_add(&dev->pc_timer, hp_disc_pc_cb, dev, 0);
    if (getenv("HP_DISC_PC")) {
        dev->pc_us = getenv("HP_DISC_PC")[0] ? atoi(getenv("HP_DISC_PC")) : 10000;
        if (dev->pc_us <= 0)
            dev->pc_us = 10000;
        timer_set_delay_u64(&dev->pc_timer, (uint64_t) dev->pc_us * TIMER_USEC);
    }

    io_sethandler(dev->base, 8,
                  hp_disc_in, hp_disc_inw, NULL,
                  hp_disc_out, hp_disc_outw, NULL,
                  dev);

    /* The option ROM carries the discovery structure the OS/2 driver looks
       for and the INT 13h front end.  It lives in a writable shadow copy so
       the front end can stash the vectors it replaces, exactly like a board
       with a RAM-backed ROM socket. */
    {
        FILE *f = rom_fopen(HP_DISC_ROM_PATH, "rb");
        if (f == NULL)
            pclog("hp_disc: option ROM %s missing, the adapter will not be found\n",
                  HP_DISC_ROM_PATH);
        else {
            size_t got = fread(dev->shadow, 1, HP_DISC_ROM_SIZE, f);
            fclose(f);
            if (got != HP_DISC_ROM_SIZE)
                pclog("hp_disc: option ROM short read (%i bytes)\n", (int) got);
        }
    }

    mem_mapping_add(&dev->mapping, HP_DISC_ROM_ADDR, HP_DISC_ROM_SIZE,
                    hp_disc_rom_read, NULL, NULL,
                    hp_disc_rom_write, NULL, NULL,
                    dev->shadow, MEM_MAPPING_EXTERNAL, dev);

    pclog("hp_disc: HP hard disc adapter at %04Xh, IRQ %i, hdd %i\n",
          dev->base, dev->irq, dev->hdd_id);

    return dev;
}

static void
hp_disc_close(void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    timer_disable(&dev->timer);

    if (dev->present)
        hdd_image_close(dev->hdd_id);

    io_removehandler(dev->base, 8,
                     hp_disc_in, hp_disc_inw, NULL,
                     hp_disc_out, hp_disc_outw, NULL,
                     dev);

    free(dev);
}

const device_t hp_disc_device = {
    .name          = "HP Hard Disc Adapter (45816A)",
    .internal_name = "st506_hp_disc",
    .flags         = DEVICE_ISA,
    .local         = 0x0300,
    .init          = hp_disc_init,
    .close         = hp_disc_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
