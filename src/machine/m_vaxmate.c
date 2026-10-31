/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through very early
 *          IBM PC/AT machines and compatibles.
 *
 *          DEC VAXmate (PC500 series) - 80286 AT-class diskless workstation.
 *
 *          Hardware per AA-HD92A-TK TRM Vol.1 chapter 2:
 *            80286 @ 8 MHz, 1 MB on board, up to 3 MB with PC50X-AA
 *            2 x 8259A, 8254-2, MC146818, 8237A, 8042 (DEC firmware)
 *            uPD765A floppy controller (03F0-03F7, IRQ6, DMA ch2)
 *            System CSR 1 at 0C00-0C1F, special register at 0C80
 *            DIGITAL video controller (6845 + translation ROM + VLT)
 *            Am7990 LANCE at 0C60/0C62/0C68 (not yet emulated)
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/mem.h>
#include <86box/io.h>
#include <86box/fdc.h>
#include <86box/hdc.h>
#include <86box/nvr.h>
#include <86box/machine.h>
#include <86box/thread.h>
#include <86box/network.h>
#include <86box/rom.h>
#include "cpu.h"
#include <86box/vid_vaxmate.h>
#include <86box/net_lance.h>
#include <86box/mouse_vaxmate.h>

#include <86box/dma.h>

extern dma_t dma[8];
extern uint8_t dma_m;
extern void vaxmate_video_dump(int w, int h);

#define VAXMATE_BIOS "roms/machines/vaxmate/vaxmate_310_311.bin"

/*
 * System CSR 1 (0x0C00) - bit-serial port to the 256-bit NVM (TRM Vol.1 p7-23,
 * TD §1.2.1).  Bit roles: 0 = 8254 counter 2 gate, 1 = NVM write enable,
 * 2 = diagnostic LED, 3 = "NVR valid" (read only), 4 = DI, 5 = CLK,
 * 6 = /CS, 7 = DO.
 *
 * Measured on the 3.10/3.11 ROM: the BIOS never shifts a frame at all - it
 * only writes the write-enable bit once (F0681) and reads the valid bit
 * (F04EB) and the LED bit (F409A).  The configuration the POST acts on comes
 * from CMOS, not from here, so the frame decoder below is only a model of the
 * protocol a SETUP utility would use.
 *
 * The BIOS never shifts data back in - it only drives frames out - but the
 * frame format is fully decoded here so that a future SETUP utility (and any
 * real read path) sees a consistent device:
 *
 *   b0        start bit, always 1
 *   b1..b7    low nibble,  7-step waveform from the ROM table at 0xF5998
 *   b8        separator, 0
 *   b9..b15   high nibble, same table
 *   b16..b35  20 trailing clocks (program pulse)
 *
 * A stored byte is (high nibble << 4) | low nibble; the write pointer is
 * sequential, exactly like the firmware's [0x306] counter.
 */
static const uint8_t nvm_rec[16] = {
    0x7e, 0x06, 0x6d, 0x4f, 0x17, 0x5b, 0x7b, 0x0e,
    0x7f, 0x1f, 0x3f, 0x73, 0x78, 0x67, 0x79, 0x39
};

typedef struct {
    uint8_t  mem[32];
    uint8_t  ptr;
    uint8_t  sel, clk, wen;
    uint8_t  csr;   /* 最后写入 CSR1 的值，供回读 */
    uint8_t  bitcnt;
    uint8_t  frame[36];
    uint8_t  valid;
} vaxmate_nvm_t;

static vaxmate_nvm_t vaxmate_nvm;

static uint8_t
vaxmate_nvm_decode(const uint8_t *bits)
{
    uint8_t v = 0, i, n;

    for (i = 0; i < 7; i++)
        v = (uint8_t) ((v << 1) | bits[i]);

    for (n = 0; n < 16; n++) {
        if (nvm_rec[n] == v)
            return n;
    }

    return 0xff;
}

static void
vaxmate_nvm_endframe(void)
{
    uint8_t lo, hi;

    vaxmate_nvm.bitcnt = 0;

    if (vaxmate_nvm.frame[0] != 1)
        return;

    lo = vaxmate_nvm_decode(&vaxmate_nvm.frame[1]);
    hi = vaxmate_nvm_decode(&vaxmate_nvm.frame[9]);

    if ((lo == 0xff) || (hi == 0xff))
        return;

    vaxmate_nvm.mem[vaxmate_nvm.ptr++ & 0x1f] = (uint8_t) ((hi << 4) | lo);
}

static uint8_t
vaxmate_csr_in(uint16_t addr, void *priv)
{
    if (addr != 0x0c00)
        return 0xff;


    /* 其余位回读最后写入的值：ROM 在 F409A 就靠 bit2 回读来判断
       "system disk / boot device" 分支。bit3 是只读的 NVR valid。 */
    return (uint8_t) (vaxmate_nvm.csr | (vaxmate_nvm.valid ? 0x08 : 0x00));
}

static void
vaxmate_csr_out(uint16_t addr, uint8_t val, void *priv)
{
    uint8_t sel, clk, di, we;

    if (addr != 0x0c00)
        return;

    sel = (val >> 6) & 1;
    clk = (val >> 5) & 1;
    di  = (val >> 4) & 1;
    we  = (val >> 1) & 1;

    vaxmate_nvm.wen = we;
    vaxmate_nvm.csr = val;

    if (sel) {
        /* /CS released: the frame is over. */
        if (vaxmate_nvm.bitcnt >= 36)
            vaxmate_nvm_endframe();
        vaxmate_nvm.bitcnt = 0;
        vaxmate_nvm.sel    = sel;
        vaxmate_nvm.clk    = clk;
        return;
    }

    if (clk && !vaxmate_nvm.clk) {
        if (vaxmate_nvm.bitcnt < 36)
            vaxmate_nvm.frame[vaxmate_nvm.bitcnt] = di;
        vaxmate_nvm.bitcnt++;

        if (vaxmate_nvm.bitcnt == 36)
            vaxmate_nvm_endframe();
    }

    vaxmate_nvm.sel = sel;
    vaxmate_nvm.clk = clk;
}

/*
 * Ethernet ROM window (0x0C20-0x0C27): 6-byte station address followed by its
 * checksum, high byte first.  The POST verifies it and reports error 0x0D when
 * it does not match (ROM 0xF0561..0xF05C1).
 */
static mem_mapping_t vaxmate_priv_mapping;

static uint8_t vaxmate_ethrom[8] = { 0x08, 0x00, 0x2b, 0x12, 0x34, 0x56, 0x00, 0x00 };

static void
vaxmate_ethrom_build(void)
{
    uint32_t bx = 0;
    int      i;

    for (i = 0; i < 6; i += 2) {
        bx = ((bx << 1) | (bx >> 15)) & 0xffff;
        bx += (uint32_t) ((vaxmate_ethrom[i] << 8) | vaxmate_ethrom[i + 1]);
        if (bx > 0xffff)
            bx = (bx & 0xffff) + 1;
    }

    if (bx == 0xffff)
        bx = 0;
    else if (bx == 0)
        bx = 0xffff;

    vaxmate_ethrom[6] = (uint8_t) (bx >> 8);
    vaxmate_ethrom[7] = (uint8_t) (bx & 0xff);
}

static uint8_t
vaxmate_ethrom_in(uint16_t addr, void *priv)
{
    return vaxmate_ethrom[addr - 0x0c20];
}


const device_t vaxmate_device = {
    .name          = "DEC VAXmate",
    .internal_name = "vaxmate",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};


/* ---- 精简 8250：VAXmate 的 COM1(3F8/IRQ4) 与串行打印机口(0CA0/IRQ11) ----
   ROM 的 INT 14h 直接读 LSR(基址+5)；端口没映射时总线上返回 0xFF，
   DR 位恒为 1 → F000:FCB2 的 XON/XOFF 轮询永远"有数据"，读 0xFF 后死循环。
   这里只实现轮询需要的寄存器语义：LSR=THRE|TEMT，无数据时 DR=0。 */
typedef struct {
    uint8_t ier, lcr, mcr, scr, fcr;
    uint8_t rx;
} vax_uart_t;

static vax_uart_t vax_uart[2];

static uint8_t
vaxmate_uart_read(uint16_t addr, void *priv)
{
    vax_uart_t *u = (vax_uart_t *) priv;
    uint8_t base = (uint8_t) ((uintptr_t) priv ? (addr & 7) : (addr & 7));

    (void) base;
    switch (addr & 7) {
        case 0: {
            uint8_t v = u->rx;
            u->rx = 0x00;
            return v;
        }
        case 1:
            return u->ier;
        case 2:
            return 0x01;              /* IIR: 无挂起中断 */
        case 3:
            return u->lcr;
        case 4:
            return u->mcr;
        case 5:
            return (uint8_t) (0x60 | (u->rx ? 0x01 : 0x00));  /* THRE|TEMT[|DR] */
        case 6:
            if (u->mcr & 0x10) {      /* 回环模式：MSR 镜像 MCR 输出 */
                uint8_t msr = 0x00;
                if (u->mcr & 0x02) msr |= 0x10;   /* RTS -> CTS */
                if (u->mcr & 0x01) msr |= 0x20;   /* DTR -> DSR */
                if (u->mcr & 0x04) msr |= 0x40;   /* OUT1 -> RI */
                if (u->mcr & 0x08) msr |= 0x80;   /* OUT2 -> DCD */
                return msr;
            }
            return 0xB0;              /* MSR: CTS|DSR|DCD */
        default:
            return u->scr;
    }
}

static void
vaxmate_uart_write(uint16_t addr, uint8_t val, void *priv)
{
    vax_uart_t *u = (vax_uart_t *) priv;

    switch (addr & 7) {
        case 1:
            u->ier = val;
            break;
        case 2:
            u->fcr = val;
            break;
        case 3:
            u->lcr = val;
            break;
        case 4:
            u->mcr = val;
            break;
        case 7:
            u->scr = val;
            break;
        case 0:
            if (u->mcr & 0x10)        /* 回环：THR 直接回到 RBR */
                u->rx = val;
            break;
        default:
            break;
    }
}

int
machine_at_vaxmate_init(const machine_t *model)
{
    int ret;

    ret = bios_load_linear(VAXMATE_BIOS, 0x000f0000, 65536, 0);
    if (bios_only || !ret)
        return ret;

    machine_at_init(model);

    /* TRM 内存图：0D0000-0EFFFF = DIGITAL 私有 RAM。
       BIOS 在 EFC0:0000 建扩展数据块，AH=15h 靠它做 DS 乒乓。 */
    mem_set_mem_state_both(0x000d0000, 0x00020000, MEM_READ_INTERNAL | MEM_WRITE_INTERNAL);
    mem_mapping_add(&vaxmate_priv_mapping, 0x000d0000, 0x00020000,
                    mem_read_ram, mem_read_ramw, mem_read_raml,
                    mem_write_ram, mem_write_ramw, mem_write_raml,
                    NULL, MEM_MAPPING_INTERNAL, NULL);

    device_add(&fdc_at_device);

    /* COM1 与串行打印机口：见上面 vax_uart_t 注释 */
    io_sethandler(0x03f8, 0x0008,
                  vaxmate_uart_read, NULL, NULL,
                  vaxmate_uart_write, NULL, NULL,
                  &vax_uart[0]);
    io_sethandler(0x0ca0, 0x0008,
                  vaxmate_uart_read, NULL, NULL,
                  vaxmate_uart_write, NULL, NULL,
                  &vax_uart[1]);
    device_add(&vaxmate_video_device);

    /* 板载鼠标：DEC 三键鼠标挂在 0x0C40 的 SCN2661 上，中断走 IRQ12。 */
    device_add(&vaxmate_mouse_device);
    /* 板载网卡按 86Box 惯例走 "Internal" 条目：机型表里 .net_device 指向
       本设备，只有用户在"网络"设置里选了 Internal 才实例化。 */
    if ((net_cards_conf[0].device_num == NET_INTERNAL) && machine_get_net_device(machine))
        device_add(machine_get_net_device(machine));

    /* AT 兼容的 RTC/CMOS。BIOS 在 F000:4AC5 用 mov al,0x92 / in al,0x71
       把 CMOS 0x12 读到 [0x549]，再由 F000:4E19-4E31 逐半字节计数：
       半字节 != 0 就算一块盘（注意不是 IBM 的 "= 0Fh 即无盘"）。
       类型 2 对应 ROM 参数表（F000:E401 + (type-1)*16）里的
       615 柱面 / 4 磁头 / 17 扇区，即扩展箱里的 RD31。

       这条路只有在 F000:4AB2 的闸门通过时才会走：
           [0x36F] bit0 == 0  且  [0x30D] bit6 == 0
       而 [0x30D] bit6 是 F000:53CC 的 CMOS 校验结果（置位 = 配置无效），
       无效时 F000:510A 会用 RAM 0x40:0x310 的默认镜像整块重写
       CMOS 0x10-0x2D（其中 0x12 = FFh），结果必然是两块盘。
       所以这里除了写 0x12，还必须把 0x2E/0x2F 校验补齐：
           sum(CMOS 0x10..0x2D) == 0x2E + 0x2F  (8 位)
       否则我们自己写的 0x12 每次 POST 都会被默认镜像覆盖。 */
    nvr_t *nvr = device_add(&nvr_at_device);
    if (nvr != NULL) {
        /* 高半字节 = 0 号盘类型 2（615/4/17），低半字节 = 0 表示没有 1 号盘。 */
        nvr->regs[0x12] = 0x20;

        /* 按 F000:53CC 的算法补校验，使 POST 认这份 CMOS 有效，
           从而跳过 F000:510A 的默认镜像重写。 */
        uint16_t sum = 0;
        for (int i = 0x10; i <= 0x2d; i++)
            sum += nvr->regs[i];
        nvr->regs[0x2e] = 0x00;
        nvr->regs[0x2f] = (uint8_t) (sum & 0xff);
    }

    /* 扩展箱 RCD31-EA：WD2010 @ 01F0-01F7 / IRQ14，20 MB MFM（615/4/17）。
       控制器在扩展箱里，与主机板一起出现，故无条件上设备；盘片由
       [Hard disks] 段的 hdd_01_* 提供，没有镜像时驱动按“无盘”处理。 */
    device_add(&st506_at_wd2010_device);

    io_sethandler(0x0c00, 0x0020,
                  vaxmate_csr_in, NULL, NULL,
                  vaxmate_csr_out, NULL, NULL,
                  NULL);

    vaxmate_ethrom_build();

    /* NVM image: station address, its checksum, then the DIGITAL
       configuration word (video = 010b, LK250 present, two RX33 drives). */
    memcpy(&vaxmate_nvm.mem[0], vaxmate_ethrom, 6);
    vaxmate_nvm.mem[6]  = vaxmate_ethrom[6];
    vaxmate_nvm.mem[7]  = vaxmate_ethrom[7];
    vaxmate_nvm.mem[8]  = 0x5a;
    vaxmate_nvm.mem[9]  = 0x00;
    memset(&vaxmate_nvm.mem[10], 0xff, 22);
    vaxmate_nvm.valid = 1;
    vaxmate_nvm.wen   = 1; /* 上电时 bit1 为高：ROM 的 F0034 用它选启动路径 */
    vaxmate_nvm.csr   = 0x02;
    io_sethandler(0x0c20, 0x0008,
                  vaxmate_ethrom_in, NULL, NULL,
                  NULL, NULL, NULL,
                  NULL);


    return ret;
}
