/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Arsenic Display ASIC device emulation.
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 2024 Dmitry Borisov
 */
#include <86box/sgivw/arsenic_priv.h>

#ifdef ENABLE_ARSENIC_LOG
int ars_do_log = ENABLE_ARSENIC_LOG;

static void
arsenic_log(const char *fmt, ...)
{
    va_list ap;

    if (ars_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define arsenic_log(fmt, ...)
#endif

static uint8_t
arsenic_mmio_read8(uint32_t addr, void* priv)
{
    uint8_t ret = 0;

    /*
     * The Linux dbe driver states that all I/O access for Arsenic must be 32-bit size.
     * However, the PROM attempts a byte access to 0x10003, 0x30008, 0x20004, and 0x40002.
     */
    arsenic_log("ARS: [R08] [%X] --> %X Unhandled\n", addr & VW_ARS_IO_DECODE_MASK, ret);

    return ret;
}

static uint16_t
arsenic_mmio_read16(uint32_t addr, void* priv)
{
    arsenic_log("ARS: [R16] [%X] --> %X Unhandled\n", addr, 0);
    assert(0);
    return 0;
}

static inline uint32_t
arsenic_mmio_decode_address(uint32_t addr)
{
    addr &= VW_ARS_IO_DECODE_MASK;
    addr /= sizeof(uint32_t);

    /*
     * TODO: This is not quite right.
     * For example, the chip decodes [0x80048, 0x80088, 0x800C8] as 0x80008.
     *
     * For now, we wrap around at the end of the dev->regs[] buffer.
     */
    addr = MIN(addr, VW_ARS_IO_DECODE_MAX);

    /*
     * During POST test, the PROM attempts to read from the following "undefined" registers:
     * 0x10050, 0x2000C, 0x30010, 0x40008, 0x80024.
     *
     * Some undefined registers are wired to the previous defined range:
     * 0x10050 --> 0x10000
     * 0x10054 --> 0x10004
     * 0x10058 --> 0x10008
     * ... (and so on)
     */

    /* 0x10050 - 0x1FFFC */
    if (addr > VW_ARS_REG_VC_START_POS && addr < VW_ARS_REG_OVERLAY_PARAMS)
        addr -= (VW_ARS_REG_VC_START_POS - VW_ARS_REG_RETRACE_POS) + 1;  /* Map to 0x10000 - 0x1004C */
    /* 0x48080 - 0x4FFFC */
    else if (addr > VW_ARS_REG_MODE_REGS_END && addr < VW_ARS_REG_COLOR_MAP_START)
        addr -= (VW_ARS_REG_MODE_REGS_END - VW_ARS_REG_MODE_REGS_START) + 1;
    /* 0x70014 - 0x77FFC */
    else if (addr > VW_ARS_REG_CURSOR_MAP_END && addr < VW_ARS_REG_CURSOR_DATA_START)
        addr -= (VW_ARS_REG_CURSOR_MAP_END - VW_ARS_REG_CURSOR_POS) + 1;
    /* 0x78100 - 0x7FFFC */
    else if (addr > VW_ARS_REG_CURSOR_DATA_END && addr < VW_ARS_REG_VIDEO_CAPTURE_CTRL_0)
        addr -= (VW_ARS_REG_CURSOR_DATA_END - VW_ARS_REG_CURSOR_DATA_START) + 1;
    /* 0x80024 */
    else if (addr == VW_ARS_REG_VIDEO_CAPTURE_CTRL_8 + 1)
        addr -= (VW_ARS_REG_VIDEO_CAPTURE_CTRL_8 - VW_ARS_REG_VIDEO_CAPTURE_CTRL_0) + 1;

    return addr;
}


static void
arsenic_mmio_write8(uint32_t addr, uint8_t val, void* priv)
{
    arsenic_log("ARS: [W08] [%X] <-- %X unhandled\n", addr, val);
    assert(0);
}

static void
arsenic_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    arsenic_log("ARS: [W16] [%X] <-- %X unhandled\n", addr, val);
    assert(0);
}

static uint32_t
arsenic_mmio_read32(uint32_t addr, void* priv)
{
    const ars_t *dev = priv;
    uint32_t ret;

    assert((addr & 0x3) == 0);

    addr = arsenic_mmio_decode_address(addr);

    ret = dev->regs[addr];

    switch (addr) {
        case VW_ARS_REG_CRT_I2C_CTRL: {
            ret = (VW_ARS_I2C_SDA_LOW | VW_ARS_I2C_SCL_LOW);

            if (i2c_gpio_get_scl(dev->crt_i2c))
                ret &= ~VW_ARS_I2C_SCL_LOW;

            if (i2c_gpio_get_sda(dev->crt_i2c))
                ret &= ~VW_ARS_I2C_SDA_LOW;
            break;
        }

        case VW_ARS_REG_LCD_I2C_CTRL: {
            ret = (VW_ARS_I2C_SDA_LOW | VW_ARS_I2C_SCL_LOW);

            if (i2c_gpio_get_scl(dev->lcd_i2c))
                ret &= ~VW_ARS_I2C_SCL_LOW;

            if (i2c_gpio_get_sda(dev->lcd_i2c))
                ret &= ~VW_ARS_I2C_SDA_LOW;
            break;
        }

        default:
            break;
    }

    arsenic_log("ARS: [R32] [%X] --> %X\n", addr * sizeof(uint32_t), ret);
    return ret;
}

static void
arsenic_mmio_write32(uint32_t addr, uint32_t val, void* priv)
{
    ars_t *dev = priv;
    uint32_t write_bits_mask;

    assert((addr & 0x3) == 0);

    addr = arsenic_mmio_decode_address(addr);

    arsenic_log("ARS: [W32] [%X] <-- %X\n", addr * sizeof(uint32_t), val);

    switch (addr) {
        /* 0x00000 - 0x0001C */
        case VW_ARS_REG_GENERAL_CTRL:
            write_bits_mask = 0x3FFFFFE0;
            break;
        case VW_ARS_REG_DOT_CLOCK:
            write_bits_mask = 0xFFFFFFFF;
            break;
        case VW_ARS_REG_CRT_I2C_CTRL:
        case VW_ARS_REG_LCD_I2C_CTRL:
            write_bits_mask = 0x00000003;
            break;
        case VW_ARS_REG_BIST:
        case VW_ARS_REG_SYSCLK_CTRL:
        case VW_ARS_REG_ID:
            write_bits_mask = 0;
            break;
        case VW_ARS_REG_POWER_CONFIG:
            write_bits_mask = 0x000000EF;
            break;

        /* 0x10000 - 0x1004C */
        case VW_ARS_REG_RETRACE_POS:
        case VW_ARS_REG_RETRACE_POS_MAX:
        case VW_ARS_REG_RETRACE_VSYNC:
        case VW_ARS_REG_RETRACE_HSYNC:
        case VW_ARS_REG_RETRACE_VBLANK:
        case VW_ARS_REG_RETRACE_HBLANK:
        case VW_ARS_REG_RETRACE_CTRL:
        case VW_ARS_REG_RETRACE_CLK:
        case VW_ARS_REG_RETRACE_INTR_CTRL_1:
        case VW_ARS_REG_RETRACE_INTR_CTRL_2:
        case VW_ARS_REG_LCD_HDRV:
        case VW_ARS_REG_LCD_VDRV:
        case VW_ARS_REG_LCD_DATA_ENABLE:
        case VW_ARS_REG_RETRACE_HPIX_ENABLE:
        case VW_ARS_REG_RETRACE_VPIX_ENABLE:
        case VW_ARS_REG_RETRACE_HCOLOR_MAP:
        case VW_ARS_REG_RETRACE_VCOLOR_MAP:
        case VW_ARS_REG_DID_START_POS:
        case VW_ARS_REG_CURSOR_START_POS:
        case VW_ARS_REG_VC_START_POS:
            // TODO: Should be dynamic:
            // 0x00FFFFFF, 0x0000007F, 0x00000000
            write_bits_mask = 0x00FFFFFF;
            break;

        /* 0x20000 - 0x20008 */
        case VW_ARS_REG_OVERLAY_PARAMS:
            write_bits_mask = 0x00003FFF;
            break;
        case VW_ARS_REG_OVERLAY_STATUS:
        case VW_ARS_REG_OVERLAY_CTRL:
            write_bits_mask = 0xFFFFFFFF;
            break;

        /* 0x30000 - 0x3000C */
        case VW_ARS_REG_FB_PARAMS:
            write_bits_mask = 0x0000FFFF;
            break;
        case VW_ARS_REG_FB_HEIGHT:
            write_bits_mask = 0xFFFF0000;
            break;
        case VW_ARS_REG_FB_STATUS:
        case VW_ARS_REG_FB_CTRL:
            write_bits_mask = 0xFFFFFFFF;
            break;

        /* 0x40000 - 0x40004 */
        case VW_ARS_REG_DID_STATUS:
        case VW_ARS_REG_DID_CTRL:
            write_bits_mask = 0xFFFFFFFF;
            break;

        /* 0x48000 */
        case VW_ARS_REG_COLOR_MAP_FIFO_STATUS:
            write_bits_mask = 0;
            break;

        /* 0x70000 - 0x70004 */
        case VW_ARS_REG_CURSOR_POS:
            write_bits_mask = 0xFFFFFFFF;
            break;
        case VW_ARS_REG_CURSOR_CTRL:
            write_bits_mask = 0x00000003;
            break;

        /* 0x80000 - 0x80020 */
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_0:
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_1:
            write_bits_mask = 0x00FFFFFF;
            break;
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_2:
            write_bits_mask = 0x0000001F;
            break;
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_3:
            write_bits_mask = 0x0000000D;
            break;
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_7:
            write_bits_mask = 0x0000FFFF;
            break;
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_4:
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_5:
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_6:
        case VW_ARS_REG_VIDEO_CAPTURE_CTRL_8:
            write_bits_mask = 0xFFFFFFFF;
            break;

        default: {
            /* 0x48000 - 0x4807C */
            if (addr >= VW_ARS_REG_MODE_REGS_START && addr <= VW_ARS_REG_MODE_REGS_END)
                write_bits_mask = 0x0000FFFF;
            /* 0x50000 - 0x55FFC */
            else if (addr >= VW_ARS_REG_COLOR_MAP_START && addr <= VW_ARS_REG_COLOR_MAP_END)
                write_bits_mask = 0;
            /* 0x60000 - 0x603FC */
            else if (addr >= VW_ARS_REG_GAMMA_MAP_START && addr <= VW_ARS_REG_GAMMA_MAP_END)
                write_bits_mask = 0xFFFFFF00;
            /* 0x68000 - 0x68FFC */
            else if (addr >= VW_ARS_REG_GAMMA_MAP_10_START && addr <= VW_ARS_REG_GAMMA_MAP_10_END)
                write_bits_mask = 0xFFFFFFFC;
            /* 0x70008 - 0x70010 */
            else if (addr >= VW_ARS_REG_CURSOR_MAP_START && addr <= VW_ARS_REG_CURSOR_MAP_END)
                write_bits_mask = 0xFFFFFF00;
            /* 0x78000 - 0x780FC */
            else if (addr >= VW_ARS_REG_CURSOR_DATA_START && addr <= VW_ARS_REG_CURSOR_DATA_END)
                write_bits_mask = 0xFFFFFFFF;
            else
                write_bits_mask = 0;
        }
    }

    val &= write_bits_mask;
    dev->regs[addr] = val | (dev->regs[addr] & ~write_bits_mask);

    switch (addr) {
        case VW_ARS_REG_CRT_I2C_CTRL:
            i2c_gpio_set(dev->crt_i2c, !(val & VW_ARS_I2C_SCL_LOW), !(val & VW_ARS_I2C_SDA_LOW));
            break;

        case VW_ARS_REG_LCD_I2C_CTRL:
            i2c_gpio_set(dev->lcd_i2c, !(val & VW_ARS_I2C_SCL_LOW), !(val & VW_ARS_I2C_SDA_LOW));
            break;

        default:
            break;
    }
}

static void
arsenic_close(void *priv)
{
    ars_t *dev = priv;

    ddc_close(dev->ddc);
    i2c_gpio_close(dev->crt_i2c);
    i2c_gpio_close(dev->lcd_i2c);

    free(dev);
}

static void *
arsenic_init(const device_t *info)
{
    ars_t *dev = calloc(1, sizeof(ars_t));

    /* SGI 1600SW monitor (display option connector). Not emulated yet */
    dev->lcd_i2c = i2c_gpio_init("i2c_lcd_arsenic");

    /* VGA interface (monitor connector) */
    dev->crt_i2c = i2c_gpio_init("ddc_crt_arsenic");
    dev->ddc = ddc_init(i2c_gpio_get_bus(dev->crt_i2c));

    mem_mapping_add(&dev->mmio_mapping,
                    VW_ARS_IO_BASE,
                    VW_ARS_IO_SIZE,
                    arsenic_mmio_read8,
                    arsenic_mmio_read16,
                    arsenic_mmio_read32,
                    arsenic_mmio_write8,
                    arsenic_mmio_write16,
                    arsenic_mmio_write32,
                    NULL,
                    MEM_MAPPING_EXTERNAL,
                    dev);

    arsenic_reset_hard(dev);

    return dev;
}

const device_t arsenic_device = {
    .name          = "Arsenic Display ASIC",
    .internal_name = "arsenic",
    .flags         = 0,
    .local         = 0,
    .init          = arsenic_init,
    .close         = arsenic_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
