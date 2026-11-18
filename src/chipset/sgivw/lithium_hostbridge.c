/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Lithium PCI Host Bridge device emulation.
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 2024 Dmitry Borisov
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/pci.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/sgivw/lithium.h>

typedef struct li_bridge_t {
    uint8_t regs[256];
    mem_mapping_t pci_config_mapping;
    uint8_t bus_number;
} li_bridge_t;

#ifdef ENABLE_LITHIUM_PCI_LOG
int li_pci_do_log = ENABLE_LITHIUM_PCI_LOG;

int li_pci_log_write_size = 0;
int li_pci_log_read_size = 0;

static void
lithium_pci_log(const char *fmt, ...)
{
    va_list ap;

    if (li_pci_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define lithium_pci_log(fmt, ...)
#endif

static bool
lithium_pci_is_pci_access_enabled(li_bridge_t *dev)
{
    // TODO: This access is on by default, but could be disabled with some command.
    return true;
}

static uint32_t
lithium_pci_get_pci_data(li_bridge_t *dev)
{
    if (!lithium_pci_is_pci_access_enabled(dev))
        return 0xFFFFFFFF;

    /* Read PCI data from the PCI subsystem */
    return pci_readl(0xCFC, NULL);
}

static void
lithium_pci_generate_configuration_cycles(li_bridge_t *dev)
{
    uint32_t address;

    /* Read the CONFIG_ADDRESS register */
    address = dev->regs[0xF8];
    address |= dev->regs[0xF9] << 8;
    address |= dev->regs[0xFA] << 16;
    address |= dev->regs[0xFB] << 24;

    /* Apply the correct PCI bus number */
    address &= ~(0xFF << 16);
    address |= dev->bus_number << 16;

    /* Route the access to the PCI subsystem */
    pci_writel(0xCF8, address, NULL);
}

static void
lithium_pci_mmio_write8(uint32_t addr, uint8_t val, void* priv)
{
    li_bridge_t *dev = priv;
    uint8_t write_bits_mask;

    addr &= VW_LI_BRIDGE_IO_DECODE_MASK;

#ifdef ENABLE_LITHIUM_PCI_LOG
    if (!li_pci_log_write_size) {
        lithium_pci_log("LI: PCI #%u [R08] [%X] <-- %X\n", dev->bus_number, addr, val);
    }
#endif

    switch (addr) {
        case 0x04:
            write_bits_mask = 0x40;
            break;
        case 0x05:
        case 0x42:
            write_bits_mask = 0x01;
            break;
        case 0x40:
            write_bits_mask = 0x20;
            break;
        case 0x47:
            write_bits_mask = 0x7F;
            break;
        case 0x59:
            write_bits_mask = 0x0F;
            break;
        case 0x44:
        case 0x45:
        case 0x46:
        case 0x58:
        case 0x60:
        case 0x61:
        case 0x64:
        case 0x65:
        case 0x68:
        case 0x69:
        case 0x6C:
        case 0x6D:
        case 0x70:
        case 0x71:
        case 0xF9:
        case 0xFA:
        case 0xFC:
        case 0xFD:
        case 0xFE:
        case 0xFF:
            write_bits_mask = 0xFF;
            break;
        case 0xF8:
            write_bits_mask = 0xFC;
            break;
        case 0x41:
        case 0xFB:
            write_bits_mask = 0x80;
            break;

        default:
            write_bits_mask = 0;
            break;
    }

    val &= write_bits_mask;
    val |= dev->regs[addr] & ~write_bits_mask;

    dev->regs[addr] = val;

    /* Data bytes of the configuration window must reach the PCI subsystem. */
    if ((addr >= 0xFC) && (addr <= 0xFF) && lithium_pci_is_pci_access_enabled(dev)) {
        uint32_t shift = (addr - 0xFC) << 3;
        uint32_t data  = pci_readl(0xCFC, NULL);

        data = (data & ~(0xFFu << shift)) | ((uint32_t) val << shift);
        pci_writel(0xCFC, data, NULL);
    }
}

static void
lithium_pci_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    li_bridge_t *dev = priv;

#ifdef ENABLE_LITHIUM_PCI_LOG
    if (!li_pci_log_write_size) {
        li_pci_log_write_size = sizeof(val);
        lithium_pci_log("LI: PCI #%u [W16] [%X] <-- %X\n", dev->bus_number, addr, val);
    }
#endif

    lithium_pci_mmio_write8(addr, val & 0xFF, priv);
    lithium_pci_mmio_write8(addr + 1, val >> 8, priv);

#ifdef ENABLE_LITHIUM_PCI_LOG
    if (li_pci_log_write_size == sizeof(val))
        li_pci_log_write_size = 0;
#endif
}

static void
lithium_pci_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    li_bridge_t *dev = priv;

    lithium_pci_log("LI: PCI #%u [W32] [%X] <-- %X\n", dev->bus_number, addr, val);

#ifdef ENABLE_LITHIUM_PCI_LOG
    li_pci_log_write_size = sizeof(val);
#endif

    /* Update the MMIO registers first */
    lithium_pci_mmio_write16(addr, val & 0xFFFF, priv);
    lithium_pci_mmio_write16(addr + 2, val >> 16, priv);

    /* Check for the PCI configuration space access (always aligned to 32-bits) */
    if ((addr & VW_LI_BRIDGE_IO_DECODE_MASK) == 0xF8) {
      lithium_pci_generate_configuration_cycles(dev);
    }

#ifdef ENABLE_LITHIUM_PCI_LOG
    li_pci_log_write_size = 0;
#endif
}

static uint8_t
lithium_pci_mmio_read8(uint32_t addr, void* priv)
{
    li_bridge_t *dev = priv;
    uint8_t ret;

    addr &= VW_LI_BRIDGE_IO_DECODE_MASK;

    switch (addr) {
        /* Check for the PCI configuration space access */
        case 0xFC:
        case 0xFD:
        case 0xFE:
        case 0xFF: {
            uint32_t data = lithium_pci_get_pci_data(dev);

            /* Latch the latest data from the 86Box PCI subsystem to MMIO */
            data >>= (addr - 0xFC) * 8;
            dev->regs[addr] = data & 0xFF;
            break;
        }

        default:
            break;
    }

    ret = dev->regs[addr];

#ifdef ENABLE_LITHIUM_PCI_LOG
    if (!li_pci_log_read_size) {
        lithium_pci_log("LI: PCI #%u [R08] [%X] --> %X\n", dev->bus_number, addr, ret);
    }
#endif

    return ret;
}

static uint16_t
lithium_pci_mmio_read16(uint32_t addr, void* priv)
{
    li_bridge_t *dev = priv;
    uint16_t ret;

#ifdef ENABLE_LITHIUM_PCI_LOG
    if (!li_pci_log_read_size)
        li_pci_log_read_size = sizeof(ret);
#endif

    ret = lithium_pci_mmio_read8(addr, priv);
    ret |= ((uint16_t)lithium_pci_mmio_read8(addr + 1, priv)) << 8;

#ifdef ENABLE_LITHIUM_PCI_LOG
    if (li_pci_log_read_size == sizeof(ret)) {
        li_pci_log_read_size = 0;
        lithium_pci_log("LI: PCI #%u [W16] [%X] <-- %X\n", dev->bus_number, addr, ret);
    }
#endif

    return ret;
}

static uint32_t
lithium_pci_mmio_read32(uint32_t addr, void* priv)
{
    li_bridge_t *dev = priv;
    uint32_t ret;

#ifdef ENABLE_LITHIUM_PCI_LOG
    li_pci_log_read_size = sizeof(ret);
#endif

    ret = lithium_pci_mmio_read16(addr, priv);
    ret |= ((uint32_t)lithium_pci_mmio_read16(addr + 2, priv)) << 16;

    lithium_pci_log("LI: PCI #%u [R32] [%X] --> %X\n", dev->bus_number, addr, ret);

#ifdef ENABLE_LITHIUM_PCI_LOG
    li_pci_log_read_size = 0;
#endif

    return ret;
}

static void
lithium_pci_reset(void *priv)
{
    li_bridge_t *dev = priv;

    memset(dev->regs, 0, sizeof(dev->regs));

    dev->regs[0x00] = 0xA9; /* Vendor: SGI */
    dev->regs[0x01] = 0x10;
    dev->regs[0x02] = 0x02; /* Device: Lithium */
    dev->regs[0x03] = 0x10;

    dev->regs[0x04] = 0x06; /* Command */
    dev->regs[0x05] = 0x00;
    dev->regs[0x06] = 0x80; /* Status */
    dev->regs[0x07] = 0x02;

    dev->regs[0x08] = 0x01; /* RevID */
    dev->regs[0x09] = 0x00; /* Prog IF */
    dev->regs[0x0A] = 0x00; /* Subclass: Host Bridge */
    dev->regs[0x0B] = 0x06; /* Class code: Bridge device */

    dev->regs[0x0C] = 0x40; /* Cache Line Size */
    dev->regs[0x0D] = 0x10; /* Latency Timer */
    dev->regs[0x0E] = 0x00; /* Header type */
    dev->regs[0x0F] = 0x00; /* BIST */

    dev->regs[0x40] = 0x20;
    dev->regs[0x41] = 0x80;
    dev->regs[0x42] = 0x01;
    dev->regs[0x43] = 0x00;

    if (dev->bus_number == 0) {
        dev->regs[0x44] = 0x01; /* Primary Bus Number */
        dev->regs[0x45] = 0xFF; /* Subordinate Bus Number */
        dev->regs[0x46] = 0x00; /* Interrupt control */
        dev->regs[0x47] = 0x00;
    }

    dev->regs[0x48] = 0x80;
    dev->regs[0x49] = 0x02;
    dev->regs[0x4A] = 0x00;
    dev->regs[0x4B] = 0x00;

    dev->regs[0x58] = 0x00;
    dev->regs[0x59] = 0x0C;
    dev->regs[0x5A] = 0x00;
    dev->regs[0x5B] = 0x00;

    dev->regs[0x60] = 0x3F;
    dev->regs[0x61] = 0x1C;
    dev->regs[0x62] = 0x00;
    dev->regs[0x63] = 0x00;

    dev->regs[0x64] = 0x3F;
    dev->regs[0x65] = 0x1C;
    dev->regs[0x66] = 0x00;
    dev->regs[0x67] = 0x00;

    dev->regs[0x68] = 0x3F;
    dev->regs[0x69] = 0x1C;
    dev->regs[0x6A] = 0x00;
    dev->regs[0x6B] = 0x00;

    dev->regs[0x6C] = 0x3F;
    dev->regs[0x6D] = 0x1C;
    dev->regs[0x6E] = 0x00;
    dev->regs[0x6F] = 0x00;

    dev->regs[0x70] = 0x3F;
    dev->regs[0x71] = 0x1C;
    dev->regs[0x72] = 0x00;
    dev->regs[0x73] = 0x00;

    /*
     * The PCI I/O space goes through to the Lithium I/O ASIC.
     * For an example of how to use configuration space see the code below:
     *
     * [W32] [FD0000F8] <-- 80002000 // Bus 0, Dev 4, Function 0, Offset 0
     * [R32] [FD0000FC] --> 71108086 // The VenID and DevID pair of PIIX4E
     *
     * TODO: This access is on by default, but could be disabled with some command.
     * In that case CONFIG_DATA returns 0xFFFFFFFF.
     */

    /* CONFIG_ADDRESS register */
    dev->regs[0xF8] = 0xC8;
    dev->regs[0xF9] = 0x20;
    dev->regs[0xFA] = 0x00;
    dev->regs[0xFB] = 0x80;

    /* CONFIG_DATA register */
    dev->regs[0xFC] = 0xFF;
    dev->regs[0xFD] = 0xFF;
    dev->regs[0xFE] = 0xFF;
    dev->regs[0xFF] = 0xFF;
}

static void
lithium_pci_close(void *priv)
{
    li_bridge_t *dev = priv;

    free(dev);
}

static void *
lithium_pci_init(const device_t *devinfo)
{
    li_bridge_t *dev = calloc(1, sizeof(li_bridge_t));
    uint32_t mmio_base;
    uint8_t bus_number;

   /*
    * NOTE: We should not add this bridge with the pci_add_card() function,
    * because both the Lithium Host bridges
    * cannot be enumerated by PCI configuration space reads.
    */

    /* Check for the bus number */
    if (devinfo->local == 0) {
      /* The PCI bus 0 is named "B" */
      mmio_base = VW_LI_BRIDGE_B_IO_BASE;
    } else {
      mmio_base = VW_LI_BRIDGE_A_IO_BASE;

      /* Register the second PCI bus (bus numbers in 86Box start with 1) */
      bus_number = pci_register_bus();
      assert(bus_number == 1);
    }
    dev->bus_number = devinfo->local;

    mem_mapping_add(&dev->pci_config_mapping,
                    mmio_base,
                    VW_LI_BRIDGE_IO_SIZE,
                    lithium_pci_mmio_read8,
                    lithium_pci_mmio_read16,
                    lithium_pci_mmio_read32,
                    lithium_pci_mmio_write8,
                    lithium_pci_mmio_write16,
                    lithium_pci_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    lithium_pci_reset(dev);

    return dev;
}

const device_t lithium_bridge_b_device = {
    .name          = "Lithium PCI Host Bridge (Bus 0)",
    .internal_name = "li_pci_bus_0",
    .flags         = DEVICE_PCI,
    .local         = 0x00, /* Bus 0 */
    .init          = lithium_pci_init,
    .close         = lithium_pci_close,
    .reset         = lithium_pci_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t lithium_bridge_a_device = {
    .name          = "Lithium PCI Host Bridge (Bus 1)",
    .internal_name = "li_pci_bus_1",
    .flags         = DEVICE_PCI,
    .local         = 0x01, /* Bus 1 */
    .init          = lithium_pci_init,
    .close         = lithium_pci_close,
    .reset         = lithium_pci_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
