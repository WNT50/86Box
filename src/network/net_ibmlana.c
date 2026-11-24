/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          Emulation of the IBM LAN Adapter/A for Ethernet board logic,
 *          on top of the DP8393x SONIC controller core in net_dp8393x.c.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include "cpu.h"
#include <86box/device.h>
#include <86box/io.h>
#include <86box/machine.h>
#include <86box/mca.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/network.h>
#include <86box/random.h>
#include <86box/rom.h>
#include <86box/pic.h>
#include <86box/net_dp8393x.h>
#include <86box/net_ibmlana.h>

//#define ENABLE_IBMLANA_LOG 1

#ifdef ENABLE_IBMLANA_LOG
int ibm_lana_do_log = ENABLE_IBMLANA_LOG;

static void
ibm_lana_log(const char *fmt, ...)
{
    va_list ap;

    if (ibm_lana_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define ibm_lana_log(fmt, ...)
#endif

/* =========================================================================
 * Board definitions
 * ========================================================================= */

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

static uint16_t lan_a_reg_read16(uint16_t port, void *priv);
static void     lan_a_reg_write16(uint16_t port, uint16_t val, void *priv);
static uint8_t  lan_a_byte_read(uint16_t port, void *priv);
static void     lan_a_byte_write(uint16_t port, uint8_t val, void *priv);
static void     lan_a_apply_pos(ibm_lana_t *dev);

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
lan_a_mem_config_read(const ibm_lana_t *dev)
{
    uint8_t val = dev->pos_regs[3] & (uint8_t) ~0x0C;

    val |= 0x04; /* bit 2 always reads back as 1 */
    if (!dev->link_up)
        val |= 0x08; /* GDLNK: 1 = bad link */

    return val;
}

static void
lan_a_update_window(ibm_lana_t *dev)
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
        dp8393x_set_ram_size(dev->sonic, size);
        mem_mapping_set_addr(&dev->window, base, size);
    }

    if (dev->board_control & BOARD_CTRL_WINDOW)
        mem_mapping_enable(&dev->window);
    else
        mem_mapping_disable(&dev->window);
}

static void
lan_a_ioremove(ibm_lana_t *dev)
{
    if (!dev->io_set)
        return;

    io_removehandler(dev->io_base, SONIC_IO_REGS_SIZE,
                     NULL, lan_a_reg_read16, NULL, NULL, lan_a_reg_write16, NULL, dev);
    io_removehandler(dev->io_base + SONIC_IO_REGS_SIZE, SONIC_IO_BYTE_SIZE,
                     lan_a_byte_read, NULL, NULL, lan_a_byte_write, NULL, NULL, dev);
    dev->io_set = 0;
}

/* Internal control register: decoded here and passed to the chip core. */
static uint16_t
lan_a_reg_read16(uint16_t port, void *priv)
{
    ibm_lana_t *dev = (ibm_lana_t *) priv;

    return dp8393x_reg_read(dev->sonic, (uint16_t) (port - dev->io_base));
}

static void
lan_a_reg_write16(uint16_t port, uint16_t val, void *priv)
{
    ibm_lana_t *dev = (ibm_lana_t *) priv;

    dp8393x_reg_write(dev->sonic, (uint16_t) (port - dev->io_base), val);
}

static uint8_t
lan_a_board_control_read(const ibm_lana_t *dev)
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
    ibm_lana_t *dev = (ibm_lana_t *) priv;

    if (dev->irq == 0)
        return;

    if (level)
        picint((uint16_t) (1 << dev->irq));
    else
        picintc((uint16_t) (1 << dev->irq));
}

static void
lan_a_board_reset(ibm_lana_t *dev)
{
    dev->board_control = 0x00;
    dev->reset_armed   = 0;

    dp8393x_reset(dev->sonic);
    lan_a_update_window(dev);

    ibm_lana_log("%s: board hardware reset\n", dev->name);
}

static void
lan_a_board_control_write(ibm_lana_t *dev, uint8_t val)
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
    const ibm_lana_t *dev = (ibm_lana_t *) priv;
    uint16_t          off = (uint16_t) (port - dev->io_base);

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
    ibm_lana_t *dev = (ibm_lana_t *) priv;
    uint16_t    off = (uint16_t) (port - dev->io_base);

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
lan_a_ioset(ibm_lana_t *dev)
{
    io_sethandler(dev->io_base, SONIC_IO_REGS_SIZE,
                  NULL, lan_a_reg_read16, NULL, NULL, lan_a_reg_write16, NULL, dev);
    io_sethandler(dev->io_base + SONIC_IO_REGS_SIZE, SONIC_IO_BYTE_SIZE,
                  lan_a_byte_read, NULL, NULL, lan_a_byte_write, NULL, NULL, dev);

    dev->io_set = 1;
}

static void
lan_a_apply_pos(ibm_lana_t *dev)
{
    static const uint8_t irq_table[4] = { 5, 15, 10, 11 };

    lan_a_ioremove(dev);

    if ((dev->irq != 0) && dev->sonic->irq_level)
        picintc((uint16_t) (1 << dev->irq));

    dev->adapter_enabled = dev->pos_regs[2] & 0x01;
    dev->io_base         = (uint16_t) (((dev->pos_regs[2] >> 5) & 0x07) << 12);
    dev->irq             = irq_table[(dev->pos_regs[2] >> 1) & 0x03];

    dev->pos_regs[3] = (dev->pos_regs[3] & (uint8_t) ~0x08) | 0x04;

    if (dev->adapter_enabled && (dev->io_base != 0x0000))
        lan_a_ioset(dev);

    lan_a_update_window(dev);

    ibm_lana_log("%s: POS 102=%02X 103=%02X, I/O=%04X, IRQ=%u, %s\n",
                 dev->name, dev->pos_regs[2], dev->pos_regs[3],
                 dev->io_base, dev->irq,
                 dev->adapter_enabled ? "enabled" : "disabled");
}

static uint8_t
lan_a_window_byte(const ibm_lana_t *dev, uint32_t off)
{
    off &= (dev->mem_size - 1);

    if (!(dev->board_control & BOARD_CTRL_ROM_RAM)) {
        /* Boot ROM is visible in the lower 16 KB only. */
        if ((off < SONIC_BOOTROM_SIZE) && dev->bootrom_loaded)
            return dev->bootrom[off];

        return 0xFF;
    }

    return dev->sonic->ram[off & (SONIC_RAM_SIZE - 1)];
}

static uint8_t
lan_a_window_read_b(uint32_t addr, void *priv)
{
    const ibm_lana_t *dev = (ibm_lana_t *) priv;

    return lan_a_window_byte(dev, addr - dev->mem_base);
}

static uint16_t
lan_a_window_read_w(uint32_t addr, void *priv)
{
    const ibm_lana_t *dev = (ibm_lana_t *) priv;
    uint32_t          off = addr - dev->mem_base;

    return (uint16_t) (lan_a_window_byte(dev, off) | (lan_a_window_byte(dev, off + 1) << 8));
}

static uint32_t
lan_a_window_read_l(uint32_t addr, void *priv)
{
    const ibm_lana_t *dev = (ibm_lana_t *) priv;
    uint32_t          off = addr - dev->mem_base;

    return (uint32_t) lan_a_window_byte(dev, off)
         | ((uint32_t) lan_a_window_byte(dev, off + 1) << 8)
         | ((uint32_t) lan_a_window_byte(dev, off + 2) << 16)
         | ((uint32_t) lan_a_window_byte(dev, off + 3) << 24);
}

static void
lan_a_window_write_b(uint32_t addr, uint8_t val, void *priv)
{
    ibm_lana_t *dev = (ibm_lana_t *) priv;
    uint32_t    off;

    /* The boot ROM is read only; only shared RAM accepts writes. */
    if (!(dev->board_control & BOARD_CTRL_ROM_RAM))
        return;

    off = (addr - dev->mem_base) & (dev->mem_size - 1);
    dev->sonic->ram[off & (SONIC_RAM_SIZE - 1)] = val;
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
    const ibm_lana_t *dev = (ibm_lana_t *) priv;

    if ((port & 0x07) == 0x03)
        return lan_a_mem_config_read(dev);

    return dev->pos_regs[port & 0x07];
}

static void
lan_a_mca_write(uint16_t port, uint8_t val, void *priv)
{
    ibm_lana_t *dev = (ibm_lana_t *) priv;

    /* POS registers 100h and 101h are read only. */
    if (port < 0x0102)
        return;

    dev->pos_regs[port & 0x07] = val;
    lan_a_apply_pos(dev);
}

static uint8_t
lan_a_mca_feedb(void *priv)
{
    const ibm_lana_t *dev = (ibm_lana_t *) priv;

    return dev->pos_regs[2] & 0x01;
}

static void
lan_a_build_id_rom(ibm_lana_t *dev)
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
    ibm_lana_t *dev = (ibm_lana_t *) priv;

    if ((dev == NULL) || (buf == NULL) || (len < 14))
        return 1;

    /* Returning 1 means the frame has been consumed and the network queue may continue. */
    dp8393x_rx(dev->sonic, buf, len, 0);
    return 1;
}

static int
lan_a_set_link_state(void *priv, uint32_t link_state)
{
    ibm_lana_t *dev = (ibm_lana_t *) priv;
    uint8_t     link_up;

    link_up = !(link_state & (NET_LINK_DOWN | NET_LINK_TEMP_DOWN));

    if (link_up != dev->link_up) {
        dev->link_up = link_up;
        ibm_lana_log("%s: link %s\n", dev->name, link_up ? "up" : "down");
    }

    return 0;
}

/* =========================================================================
 * Device registration and life cycle
 * ========================================================================= */

static void *
lan_a_init(const device_t *info)
{
    ibm_lana_t *dev;
    int         mac;
    uint16_t    pos_id;

    dev = (ibm_lana_t *) calloc(1, sizeof(ibm_lana_t));

    dev->sonic_inst = dp8393x_inst;
    dev->sonic      = (dp8393x_t *) device_add_inst(&dp8393x_device, dp8393x_inst++);

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

    dev->link_up  = 1;
    dev->mem_base = 0x000C8000;
    dev->mem_size = 0x4000;

    dp8393x_set_ram_size(dev->sonic, dev->mem_size);
    dev->sonic->irq_cb   = lan_a_irq;
    dev->sonic->irq_priv = dev;

    if (dev->variant == IBM_LAN_A_SHORT)
        dev->bootrom_loaded = (rom_load_linear(ROM_PATH_SHORT, 0, SONIC_BOOTROM_SIZE, 0, dev->bootrom) != 0);
    else
        dev->bootrom_loaded = (rom_load_linear(ROM_PATH_LONG, 0, SONIC_BOOTROM_SIZE, 0, dev->bootrom) != 0);

    if (!dev->bootrom_loaded)
        ibm_lana_log("%s: Boot ROM missing, the low 16 KB of the memory window returns 0xFF\n", dev->name);

    lan_a_build_id_rom(dev);

    mem_mapping_add(&dev->window, dev->mem_base, dev->mem_size,
                    lan_a_window_read_b, lan_a_window_read_w, lan_a_window_read_l,
                    lan_a_window_write_b, lan_a_window_write_w, lan_a_window_write_l,
                    NULL, MEM_MAPPING_EXTERNAL, dev);
    mem_mapping_disable(&dev->window);

    mca_add(lan_a_mca_read, lan_a_mca_write, lan_a_mca_feedb, NULL, dev);

    dev->sonic->card = network_attach(dev, dev->mac, lan_a_rx, lan_a_set_link_state);
    if ((dev->sonic->card != NULL) && (dev->sonic->card->link_state & NET_LINK_DOWN))
        dev->link_up = 0;

    lan_a_apply_pos(dev);

    ibm_lana_log("%s: MAC %02X:%02X:%02X:%02X:%02X:%02X, POS ID %04X\n",
                 dev->name, dev->mac[0], dev->mac[1], dev->mac[2],
                 dev->mac[3], dev->mac[4], dev->mac[5], pos_id);

    return dev;
}

static void
lan_a_close(void *priv)
{
    ibm_lana_t *dev = (ibm_lana_t *) priv;

    if (dev == NULL)
        return;

    lan_a_ioremove(dev);
    mem_mapping_disable(&dev->window);

    if (dev->sonic != NULL) {
        timer_disable(&dev->sonic->wt_timer);

        if (dev->sonic->card != NULL)
            netcard_close(dev->sonic->card);
    }

    /* The chip device instance is released centrally by device_close_all, matching
     * the dp8390 life cycle in net_ne2000.c; here we only ensure the timer and the
     * backend are stopped. */
    free(dev);
}

static void
lan_a_reset(void *priv)
{
    lan_a_board_reset((ibm_lana_t *) priv);
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
