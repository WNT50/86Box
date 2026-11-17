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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/machine.h>
#include <86box/mca.h>
#include <86box/mem.h>
#include <86box/random.h>
#include <86box/rom.h>
#include <86box/thread.h>
#include <86box/timer.h>
#include <86box/network.h>
#include <86box/pic.h>

/* =========================================================================
 * 区 1：公共定义
 * ========================================================================= */

#define SONIC_RAM_SIZE     0x8000 /* 8K x 32-bit shared SRAM */
#define SONIC_BOOTROM_SIZE 0x4000 /* 16 KB boot ROM */

#define SONIC_IO_REGS_SIZE 0x80 /* +00h .. +7Fh: internal registers */
#define SONIC_IO_BYTE_SIZE 0x20 /* +80h .. +9Fh: board and ID registers */

#define BOARD_CTRL_ROM_RAM 0x80 /* 0 = boot ROM, 1 = shared RAM */
#define BOARD_CTRL_IRQ_PND 0x40 /* interrupt pending (read only) */
#define BOARD_CTRL_ALWAYS  0x30 /* bits 5 and 4 always read as 1 */
#define BOARD_CTRL_RESET   0x08 /* hardware reset, write 1 then 0 */
#define BOARD_CTRL_SLOT16  0x04 /* 0 = 32-bit slot, 1 = 16-bit slot */
#define BOARD_CTRL_WINDOW  0x02 /* memory window enable */
#define BOARD_CTRL_IRQ_EN  0x01 /* interrupt enable */
#define BOARD_CTRL_WRITABLE (BOARD_CTRL_ROM_RAM | BOARD_CTRL_RESET | BOARD_CTRL_WINDOW | BOARD_CTRL_IRQ_EN)

#define ROM_PATH_LONG  "roms/network/ibmlana/60g3315.bin"
#define ROM_PATH_SHORT "roms/network/ibmlana/25h3518.bin"

typedef enum {
    IBM_LAN_A_LONG  = 0, /* POS ID FFE0, boot ROM 60G3315 */
    IBM_LAN_A_SHORT = 1  /* POS ID FEE6, boot ROM 25H3518 */
} ibm_lan_a_model_t;

typedef struct dp8393x_t {
    const char *name;
    int         variant;

    netcard_t *card;
    uint8_t    mac[6];

    /* 区 2：控制器核心状态 */
    uint16_t regs[64];
    uint16_t cam[48]; /* 16 entries x 3 words */
    uint16_t cam_enable;
    uint16_t cep;
    uint16_t cam_valid;
    uint8_t  rx_enabled;
    uint8_t  stopped;
    uint8_t  irq_level;
    void   (*irq_cb)(void *priv, int level);
    void    *irq_priv;
    uint8_t  ram[SONIC_RAM_SIZE];

    /* 区 3：MCA 板级状态 */
    uint8_t       pos_regs[8];
    uint8_t       board_control;
    uint8_t       reset_armed;
    uint8_t       adapter_enabled;
    uint8_t       link_up;
    uint8_t       io_set;
    uint8_t       bootrom_loaded;
    uint8_t       bootrom[SONIC_BOOTROM_SIZE];
    uint8_t       id_rom[16];
    uint16_t      io_base;
    uint8_t       irq;
    uint32_t      mem_base;
    uint32_t      mem_size;
    mem_mapping_t window;
} dp8393x_t;

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
 * 区 2：SONIC/SONIC-T 控制器核心
 *
 * P1 实现寄存器阵列、写掩码、复位默认值、CAM 装载与中断聚合。
 * 收发状态机在 P2/P3 接入，这里只保留命令位与状态位的落点。
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

#define SONIC_ISR_LCD  0x1000

static uint16_t
dp8393x_ram_read16(const dp8393x_t *dev, uint16_t off)
{
    uint32_t size = dev->mem_size ? dev->mem_size : SONIC_RAM_SIZE;
    uint32_t o    = (uint32_t) (off & (size - 1)) & (SONIC_RAM_SIZE - 1);

    return (uint16_t) (dev->ram[o] | (dev->ram[(o + 1) & (SONIC_RAM_SIZE - 1)] << 8));
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

static void
dp8393x_core_reset(dp8393x_t *dev)
{
    memset(dev->regs, 0x00, sizeof(dev->regs));
    memset(dev->cam, 0x00, sizeof(dev->cam));

    dev->cam_enable = 0x0000;
    dev->cep        = 0x0000;
    dev->cam_valid  = 0x00;
    dev->rx_enabled = 0;
    dev->stopped    = 1;

    /* 硬件复位：RST|STP|RXDIS = 0094h。 */
    dev->regs[SONIC_CR >> 1] = SONIC_CR_RST | SONIC_CR_STP | SONIC_CR_RXDIS;

    dp8393x_update_irq(dev);
}

/* LCAM: CDP 指向窗口内的 CAM 描述符区，CDC 为描述符个数。
 * 每个描述符 16 字节：+0 控制/入口号，+4/+8/+0C 为 6 字节地址。 */
static void
dp8393x_cam_load(dp8393x_t *dev)
{
    uint16_t cdp = dev->regs[SONIC_CDP >> 1];
    uint16_t cdc = dev->regs[SONIC_CDC >> 1] & 0x000F;
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

    /* CAM Enable 字段紧跟在最后一个描述符之后。 */
    dev->cam_enable = dp8393x_ram_read16(dev, (uint16_t) (cdp + (cdc << 4)));

    dev->regs[SONIC_CR >> 1] &= (uint16_t) ~SONIC_CR_LCAM;
    dp8393x_raise_isr(dev, SONIC_ISR_LCD);
}

static uint16_t
dp8393x_reg_read(dp8393x_t *dev, uint16_t off)
{
    switch (off) {
        case SONIC_SR:
            return 0x0000;

        case SONIC_CE: {
            uint16_t val = dev->cam[dev->cep & 0x3F];

            dev->cep = (uint16_t) ((dev->cep + 1) & 0x3F);
            dev->regs[SONIC_CEP >> 1] = dev->cep;
            return val;
        }

        default:
            break;
    }

    if (off >= 0x80)
        return 0xFFFF;

    return dev->regs[off >> 1];
}

static void
dp8393x_reg_write(dp8393x_t *dev, uint16_t off, uint16_t val)
{
    switch (off) {
        case SONIC_CR: {
            if (val & SONIC_CR_RST) {
                dp8393x_core_reset(dev);
                return;
            }

            if (val & SONIC_CR_RRRA) {
                /* RRRA 复位读指针，命令位自清。 */
                dev->regs[SONIC_RRP >> 1] = dev->regs[SONIC_RSA >> 1];
            }

            if (val & SONIC_CR_LCAM)
                dp8393x_cam_load(dev);

            if (val & SONIC_CR_RXEN)
                dev->rx_enabled = 1;
            if (val & SONIC_CR_RXDIS)
                dev->rx_enabled = 0;

            if (val & SONIC_CR_STP)
                dev->stopped = 1;
            if (val & SONIC_CR_ST)
                dev->stopped = 0;

            /* TXP/HTX/RRRA/LCAM 为命令位，不保持；STP/RXEN/RXDIS 为状态位。 */
            dev->regs[SONIC_CR >> 1] = (uint16_t) ((dev->stopped ? SONIC_CR_STP : 0x0000)
                                                 | (dev->rx_enabled ? SONIC_CR_RXEN : SONIC_CR_RXDIS));
            break;
        }

        case SONIC_DCR:
            /* bit9-8、bit6 固定为 1，bit7、bit15-10 固定为 0。 */
            dev->regs[SONIC_DCR >> 1] = (uint16_t) ((val & 0x003F) | 0x0360);
            break;

        case SONIC_RCR:
            dev->regs[SONIC_RCR >> 1] = val;
            break;

        case SONIC_TCR:
            dev->regs[SONIC_TCR >> 1] = val & 0x0FFF;
            break;

        case SONIC_IMR:
            dev->regs[SONIC_IMR >> 1] = val & 0x7FFF;
            dp8393x_update_irq(dev);
            break;

        case SONIC_ISR:
            dev->regs[SONIC_ISR >> 1] &= (uint16_t) ~(val & 0x7FFF);
            dp8393x_update_irq(dev);
            break;

        case SONIC_CDC:
            dev->regs[SONIC_CDC >> 1] = val & 0x000F;
            break;

        case SONIC_CEP:
            dev->cep = val & 0x003F;
            dev->regs[SONIC_CEP >> 1] = dev->cep;
            break;

        case SONIC_CE:
            dev->cam[dev->cep & 0x3F] = val;
            dev->cep = (uint16_t) ((dev->cep + 1) & 0x3F);
            dev->regs[SONIC_CEP >> 1] = dev->cep;
            break;

        case SONIC_SR:
        case SONIC_RSC:
        case SONIC_CRCT:
        case SONIC_FAET:
        case SONIC_MPT:
            /* 只读：状态/计数器在 P4/P5 接入。 */
            break;

        default:
            if (off < 0x80)
                dev->regs[off >> 1] = val;
            break;
    }
}

/* =========================================================================
 * 区 3：MCA 板级
 * ========================================================================= */

static uint16_t lan_a_reg_read16(uint16_t port, void *priv);
static void     lan_a_reg_write16(uint16_t port, uint16_t val, void *priv);
static uint8_t  lan_a_byte_read(uint16_t port, void *priv);
static void     lan_a_byte_write(uint16_t port, uint8_t val, void *priv);
static void     lan_a_apply_pos(dp8393x_t *dev);

static uint16_t
lan_a_pos_id(int variant)
{
    return (variant == IBM_LAN_A_SHORT) ? 0xFEE6 : 0xFFE0;
}

/* POS 103 bit7-5: shared memory window base address. */
static uint32_t
lan_a_mem_base(uint8_t cfg, uint32_t size)
{
    uint8_t sel = (cfg >> 5) & 0x07;

    if (size == 0x8000) {
        switch (sel & 0x06) {
            case 0x02:
                return 0x000C8000;
            case 0x04:
                return 0x000D0000;
            case 0x06:
                return 0x000D8000;
            default:
                return 0x00000000;
        }
    }

    switch (sel) {
        case 0x01:
            return 0x000C4000;
        case 0x02:
            return 0x000C8000;
        case 0x04:
            return 0x000D0000;
        case 0x05:
            return 0x000D4000;
        case 0x06:
            return 0x000D8000;
        default:
            return 0x00000000;
    }
}

static uint8_t
lan_a_mem_config_read(const dp8393x_t *dev)
{
    uint8_t val = dev->pos_regs[3] & (uint8_t) ~0x0C;

    val |= 0x04; /* bit 2 always reads back as 1 */
    if (!dev->link_up)
        val |= 0x08; /* GDLNK: 1 = bad link */

    return val;
}

static void
lan_a_update_window(dp8393x_t *dev)
{
    uint8_t  size_sel = dev->pos_regs[3] & 0x03;
    uint32_t size;
    uint32_t base;

    /* POS 103 bit1-0: 10b = 16 KB, 11b = 32 KB. */
    if (size_sel < 0x02) {
        mem_mapping_disable(&dev->window);
        return;
    }

    size = (size_sel == 0x03) ? 0x8000 : 0x4000;
    base = lan_a_mem_base(dev->pos_regs[3], size);

    if (!dev->adapter_enabled || (base == 0x00000000)) {
        mem_mapping_disable(&dev->window);
        return;
    }

    if ((base != dev->mem_base) || (size != dev->mem_size)) {
        dev->mem_base = base;
        dev->mem_size = size;
        mem_mapping_set_addr(&dev->window, base, size);
    }

    if (dev->board_control & BOARD_CTRL_WINDOW)
        mem_mapping_enable(&dev->window);
    else
        mem_mapping_disable(&dev->window);
}

static void
lan_a_ioremove(dp8393x_t *dev)
{
    if (!dev->io_set)
        return;

    io_removehandler(dev->io_base, SONIC_IO_REGS_SIZE,
                     NULL, lan_a_reg_read16, NULL, NULL, lan_a_reg_write16, NULL, dev);
    io_removehandler(dev->io_base + SONIC_IO_REGS_SIZE, SONIC_IO_BYTE_SIZE,
                     lan_a_byte_read, NULL, NULL, lan_a_byte_write, NULL, NULL, dev);
    dev->io_set = 0;
}

/* 内部控制寄存器：解码后交给区 2 的芯片核心。 */
static uint16_t
lan_a_reg_read16(uint16_t port, void *priv)
{
    dp8393x_t *dev = (dp8393x_t *) priv;

    return dp8393x_reg_read(dev, (uint16_t) (port - dev->io_base));
}

static void
lan_a_reg_write16(uint16_t port, uint16_t val, void *priv)
{
    dp8393x_t *dev = (dp8393x_t *) priv;

    dp8393x_reg_write(dev, (uint16_t) (port - dev->io_base), val);
}

static uint8_t
lan_a_board_control_read(const dp8393x_t *dev)
{
    uint8_t val = dev->board_control & BOARD_CTRL_WRITABLE;

    val |= BOARD_CTRL_ALWAYS;

    if (!machine_has_bus(machine, MACHINE_BUS_MCA32))
        val |= BOARD_CTRL_SLOT16;

    return val;
}

static void
lan_a_irq(void *priv, int level)
{
    dp8393x_t *dev = (dp8393x_t *) priv;

    if (dev->irq == 0)
        return;

    if (level)
        picint((uint16_t) (1 << dev->irq));
    else
        picintc((uint16_t) (1 << dev->irq));
}

static void
lan_a_board_reset(dp8393x_t *dev)
{
    dev->board_control = 0x00;
    dev->reset_armed   = 0;

    dp8393x_core_reset(dev);
    lan_a_update_window(dev);

    dp8393x_log("%s: 板级硬件复位\n", dev->name);
}

static void
lan_a_board_control_write(dp8393x_t *dev, uint8_t val)
{
    dev->board_control = (dev->board_control & (uint8_t) ~BOARD_CTRL_WRITABLE) | (val & BOARD_CTRL_WRITABLE);

    if (val & BOARD_CTRL_RESET) {
        dev->reset_armed = 1;
    } else if (dev->reset_armed) {
        lan_a_board_reset(dev);
        return;
    }

    lan_a_update_window(dev);
}

static uint8_t
lan_a_byte_read(uint16_t port, void *priv)
{
    const dp8393x_t *dev = (dp8393x_t *) priv;
    uint16_t         off = (uint16_t) (port - dev->io_base);

    switch (off) {
        case 0x80:
            return lan_a_board_control_read(dev);
        case 0x82:
            return dev->pos_regs[2];
        case 0x83:
            return lan_a_mem_config_read(dev);
        default:
            break;
    }

    if ((off >= 0x90) && (off <= 0x9F))
        return dev->id_rom[off - 0x90];

    return 0xFF;
}

static void
lan_a_byte_write(uint16_t port, uint8_t val, void *priv)
{
    dp8393x_t *dev = (dp8393x_t *) priv;
    uint16_t   off = (uint16_t) (port - dev->io_base);

    switch (off) {
        case 0x80:
            lan_a_board_control_write(dev, val);
            break;
        case 0x82:
            dev->pos_regs[2] = val;
            lan_a_apply_pos(dev);
            break;
        case 0x83:
            dev->pos_regs[3] = (val & (uint8_t) ~0x0C) | 0x04;
            lan_a_apply_pos(dev);
            break;
        default:
            break;
    }
}

static void
lan_a_ioset(dp8393x_t *dev)
{
    io_sethandler(dev->io_base, SONIC_IO_REGS_SIZE,
                  NULL, lan_a_reg_read16, NULL, NULL, lan_a_reg_write16, NULL, dev);
    io_sethandler(dev->io_base + SONIC_IO_REGS_SIZE, SONIC_IO_BYTE_SIZE,
                  lan_a_byte_read, NULL, NULL, lan_a_byte_write, NULL, NULL, dev);

    dev->io_set = 1;
}

static void
lan_a_apply_pos(dp8393x_t *dev)
{
    static const uint8_t irq_table[4] = { 5, 15, 10, 11 };

    lan_a_ioremove(dev);

    if ((dev->irq != 0) && dev->irq_level)
        picintc((uint16_t) (1 << dev->irq));

    dev->adapter_enabled = dev->pos_regs[2] & 0x01;
    dev->io_base         = (uint16_t) (((dev->pos_regs[2] >> 5) & 0x07) << 12);
    dev->irq             = irq_table[(dev->pos_regs[2] >> 1) & 0x03];

    dev->pos_regs[3] = (dev->pos_regs[3] & (uint8_t) ~0x08) | 0x04;

    if (dev->adapter_enabled && (dev->io_base != 0x0000))
        lan_a_ioset(dev);

    lan_a_update_window(dev);

    dp8393x_log("%s: POS 102=%02X 103=%02X, I/O=%04X, IRQ=%u, %s\n",
                dev->name, dev->pos_regs[2], dev->pos_regs[3],
                dev->io_base, dev->irq,
                dev->adapter_enabled ? "启用" : "禁用");
}

static uint8_t
lan_a_window_byte(const dp8393x_t *dev, uint32_t off)
{
    off &= (dev->mem_size - 1);

    if (!(dev->board_control & BOARD_CTRL_ROM_RAM)) {
        /* Boot ROM is visible in the lower 16 KB only. */
        if ((off < SONIC_BOOTROM_SIZE) && dev->bootrom_loaded)
            return dev->bootrom[off];

        return 0xFF;
    }

    return dev->ram[off & (SONIC_RAM_SIZE - 1)];
}

static uint8_t
lan_a_window_read_b(uint32_t addr, void *priv)
{
    const dp8393x_t *dev = (dp8393x_t *) priv;

    return lan_a_window_byte(dev, addr - dev->mem_base);
}

static uint16_t
lan_a_window_read_w(uint32_t addr, void *priv)
{
    const dp8393x_t *dev = (dp8393x_t *) priv;
    uint32_t         off = addr - dev->mem_base;

    return (uint16_t) (lan_a_window_byte(dev, off) | (lan_a_window_byte(dev, off + 1) << 8));
}

static uint32_t
lan_a_window_read_l(uint32_t addr, void *priv)
{
    const dp8393x_t *dev = (dp8393x_t *) priv;
    uint32_t         off = addr - dev->mem_base;

    return (uint32_t) lan_a_window_byte(dev, off)
         | ((uint32_t) lan_a_window_byte(dev, off + 1) << 8)
         | ((uint32_t) lan_a_window_byte(dev, off + 2) << 16)
         | ((uint32_t) lan_a_window_byte(dev, off + 3) << 24);
}

static void
lan_a_window_write_b(uint32_t addr, uint8_t val, void *priv)
{
    dp8393x_t *dev = (dp8393x_t *) priv;
    uint32_t   off;

    /* The boot ROM is read only; only shared RAM accepts writes. */
    if (!(dev->board_control & BOARD_CTRL_ROM_RAM))
        return;

    off = (addr - dev->mem_base) & (dev->mem_size - 1);
    dev->ram[off & (SONIC_RAM_SIZE - 1)] = val;
}

static void
lan_a_window_write_w(uint32_t addr, uint16_t val, void *priv)
{
    lan_a_window_write_b(addr, (uint8_t) val, priv);
    lan_a_window_write_b(addr + 1, (uint8_t) (val >> 8), priv);
}

static void
lan_a_window_write_l(uint32_t addr, uint32_t val, void *priv)
{
    lan_a_window_write_w(addr, (uint16_t) val, priv);
    lan_a_window_write_w(addr + 2, (uint16_t) (val >> 16), priv);
}

static uint8_t
lan_a_mca_read(uint16_t port, void *priv)
{
    const dp8393x_t *dev = (dp8393x_t *) priv;

    if ((port & 0x07) == 0x03)
        return lan_a_mem_config_read(dev);

    return dev->pos_regs[port & 0x07];
}

static void
lan_a_mca_write(uint16_t port, uint8_t val, void *priv)
{
    dp8393x_t *dev = (dp8393x_t *) priv;

    /* POS registers 100h and 101h are read only. */
    if (port < 0x0102)
        return;

    dev->pos_regs[port & 0x07] = val;
    lan_a_apply_pos(dev);
}

static uint8_t
lan_a_mca_feedb(void *priv)
{
    const dp8393x_t *dev = (dp8393x_t *) priv;

    return dev->pos_regs[2] & 0x01;
}

static void
lan_a_build_id_rom(dp8393x_t *dev)
{
    uint16_t pos_id = lan_a_pos_id(dev->variant);

    memset(dev->id_rom, 0x00, sizeof(dev->id_rom));

    dev->id_rom[0] = (uint8_t) (pos_id & 0xFF);
    dev->id_rom[1] = (uint8_t) (pos_id >> 8);
    memcpy(&dev->id_rom[2], dev->mac, 6);
}

static int
lan_a_rx(void *priv, uint8_t *buf, int len)
{
    (void) priv;
    (void) buf;
    (void) len;

    /* 接收在 P3 实现。 */
    return 0;
}

static int
lan_a_set_link_state(void *priv, uint32_t link_state)
{
    dp8393x_t *dev = (dp8393x_t *) priv;
    uint8_t    link_up;

    link_up = !(link_state & (NET_LINK_DOWN | NET_LINK_TEMP_DOWN));

    if (link_up != dev->link_up) {
        dev->link_up = link_up;
        dp8393x_log("%s: 链路%s\n", dev->name, link_up ? "正常" : "断开");
    }

    return 0;
}

/* =========================================================================
 * 区 4：设备注册与生命周期
 * ========================================================================= */

static void *
lan_a_init(const device_t *info)
{
    dp8393x_t *dev = (dp8393x_t *) calloc(1, sizeof(dp8393x_t));
    int        mac;
    uint16_t   pos_id;

    dev->name    = info->name;
    dev->variant = (int) info->local;

    /* IBM OUI 00:06:29, as stored in the adapter's ID ROM. */
    dev->mac[0] = 0x00;
    dev->mac[1] = 0x06;
    dev->mac[2] = 0x29;

    mac = device_get_config_mac("mac", -1);
    if (mac & 0xff000000) {
        dev->mac[3] = random_generate();
        dev->mac[4] = random_generate();
        dev->mac[5] = random_generate();
        mac         = (dev->mac[3] << 16) | (dev->mac[4] << 8) | dev->mac[5];
        device_set_config_mac("mac", mac);
    } else {
        dev->mac[3] = (mac >> 16) & 0xFF;
        dev->mac[4] = (mac >> 8) & 0xFF;
        dev->mac[5] = mac & 0xFF;
    }

    pos_id          = lan_a_pos_id(dev->variant);
    dev->pos_regs[0] = (uint8_t) (pos_id & 0xFF);
    dev->pos_regs[1] = (uint8_t) (pos_id >> 8);
    dev->pos_regs[2] = 0x20; /* I/O 1000h, 10BASE-T, IRQ 5, adapter disabled */
    dev->pos_regs[3] = 0x46; /* C800h, 16 KB, bit 2 reads as 1 */

    dev->link_up   = 1;
    dev->mem_base  = 0x000C8000;
    dev->mem_size  = 0x4000;
    dev->irq_cb    = lan_a_irq;
    dev->irq_priv  = dev;

    dp8393x_core_reset(dev);

    if (dev->variant == IBM_LAN_A_SHORT)
        dev->bootrom_loaded = (rom_load_linear(ROM_PATH_SHORT, 0, SONIC_BOOTROM_SIZE, 0, dev->bootrom) != 0);
    else
        dev->bootrom_loaded = (rom_load_linear(ROM_PATH_LONG, 0, SONIC_BOOTROM_SIZE, 0, dev->bootrom) != 0);

    if (!dev->bootrom_loaded)
        dp8393x_log("%s: Boot ROM 缺失，内存窗口低 16 KB 将返回 0xFF\n", dev->name);

    lan_a_build_id_rom(dev);

    mem_mapping_add(&dev->window, dev->mem_base, dev->mem_size,
                    lan_a_window_read_b, lan_a_window_read_w, lan_a_window_read_l,
                    lan_a_window_write_b, lan_a_window_write_w, lan_a_window_write_l,
                    NULL, MEM_MAPPING_EXTERNAL, dev);
    mem_mapping_disable(&dev->window);

    mca_add(lan_a_mca_read, lan_a_mca_write, lan_a_mca_feedb, NULL, dev);

    dev->card = network_attach(dev, dev->mac, lan_a_rx, lan_a_set_link_state);
    if ((dev->card != NULL) && (dev->card->link_state & NET_LINK_DOWN))
        dev->link_up = 0;

    lan_a_apply_pos(dev);

    dp8393x_log("%s: MAC %02X:%02X:%02X:%02X:%02X:%02X, POS ID %04X\n",
                dev->name, dev->mac[0], dev->mac[1], dev->mac[2],
                dev->mac[3], dev->mac[4], dev->mac[5], pos_id);

    return dev;
}

static void
lan_a_close(void *priv)
{
    dp8393x_t *dev = (dp8393x_t *) priv;

    if (dev == NULL)
        return;

    lan_a_ioremove(dev);
    mem_mapping_disable(&dev->window);

    if (dev->card != NULL)
        netcard_close(dev->card);

    free(dev);
}

static void
lan_a_reset(void *priv)
{
    lan_a_board_reset((dp8393x_t *) priv);
}

static const device_config_t ibm_lan_a_config[] = {
    // clang-format off
    {
        .name           = "mac",
        .description    = "MAC Address",
        .type           = CONFIG_MAC,
        .default_string = NULL,
        .default_int    = -1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
    // clang-format on
};

const device_t ibm_lan_a_device = {
    .name          = "IBM LAN Adapter/A for Ethernet",
    .internal_name = "ibm_lan_a",
    .flags         = DEVICE_MCA,
    .local         = IBM_LAN_A_LONG,
    .init          = lan_a_init,
    .close         = lan_a_close,
    .reset         = lan_a_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ibm_lan_a_config
};

const device_t ibm_lan_a_short_device = {
    .name          = "IBM LAN Adapter/A for Ethernet (Short)",
    .internal_name = "ibm_lan_a_short",
    .flags         = DEVICE_MCA,
    .local         = IBM_LAN_A_SHORT,
    .init          = lan_a_init,
    .close         = lan_a_close,
    .reset         = lan_a_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ibm_lan_a_config
};
