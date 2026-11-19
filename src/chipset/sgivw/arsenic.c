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

static uint32_t arsenic_mmio_decode_address(uint32_t addr);

static uint8_t
arsenic_mmio_read8(uint32_t addr, void* priv)
{
    const ars_t *dev   = priv;
    uint32_t     off   = addr & VW_ARS_IO_DECODE_MASK;
    uint32_t     idx   = arsenic_mmio_decode_address(addr);
    uint8_t      ret   = (uint8_t) (dev->regs[idx] >> ((addr & 3) * 8));

    /*
     * The Linux dbe driver states that all I/O access for Arsenic must be 32-bit size.
     * However, the PROM attempts a byte access to 0x10003, 0x30008, 0x20004, and 0x40002.
     */
    arsenic_log("ARS: [R08] [%X] --> %X\n", off, ret);

    return ret;
}

static uint16_t
arsenic_mmio_read16(uint32_t addr, void* priv)
{
    const ars_t *dev = priv;
    uint32_t     idx = arsenic_mmio_decode_address(addr & ~1);
    uint16_t     ret = (uint16_t) (dev->regs[idx] >> ((addr & 2) * 8));

    arsenic_log("ARS: [R16] [%X] --> %X\n", addr, ret);
    return ret;
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
    ars_t   *dev   = priv;
    uint32_t idx   = arsenic_mmio_decode_address(addr & ~3);
    uint32_t shift = (addr & 3) * 8;

    dev->regs[idx] = (dev->regs[idx] & ~(0xFFu << shift)) | ((uint32_t) val << shift);
    arsenic_log("ARS: [W08] [%X] <-- %X\n", addr, val);
}

static void
arsenic_mmio_write16(uint32_t addr, uint16_t val, void* priv)
{
    ars_t   *dev   = priv;
    uint32_t idx   = arsenic_mmio_decode_address(addr & ~3);
    uint32_t shift = (addr & 2) * 8;

    dev->regs[idx] = (dev->regs[idx] & ~(0xFFFFu << shift)) | ((uint32_t) val << shift);
    arsenic_log("ARS: [W16] [%X] <-- %X\n", addr, val);
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


/* The display reads its pixels out of system memory; there is no VRAM of
   its own.  The PROM enables the linear frame buffer in FB_CTRL, puts the
   pixel height in FB_HEIGHT and then draws 1024x768 in 8 bpp, so this first
   cut of the output path copies that rectangle straight into 86Box's 32-bit
   frame buffer, one colour map entry per pixel. */
#define ARS_FB_WIDTH   1024
#define ARS_FB_HEIGHT  768

static void
arsenic_blit_frame(ars_t *dev)
{
    uint32_t  fb    = dev->regs[VW_ARS_REG_FB_CTRL];
    uint32_t  base;
    uint32_t  tile;
    int       tiles;
    int       depth;
    int       width  = ARS_FB_WIDTH;
    int       height;

    /* Nothing is on screen until the firmware has enabled the linear
       frame buffer and told us where it is. */
    if (!(fb & 0x00000003))
        return;

    /* Bit fields as defined by the Linux visws frame buffer driver
       (FRM_CONTROL/FRM_SIZE_TILE/FRM_SIZE_PIXEL): the tile pointer is held
       in bits 31..9, so masking it off gives the physical address back. */
    base = fb & 0xfffffe00;

    /* FRM_SIZE_TILE: width in 512-byte tiles, depth in bits 14..13. */
    tile   = dev->regs[VW_ARS_REG_FB_PARAMS];
    tiles  = (int) ((tile >> 5) & 0xff);
    depth  = (int) ((tile >> 13) & 0x03);

    /* Only the 8 bpp path is modelled for now. */
    if (depth != 0)
        return;

    width = tiles * 512;
    if ((width <= 0) || (width > 2048))
        width = ARS_FB_WIDTH;

    /* FB_HEIGHT carries the panel height in its upper half. */
    height = (int) ((dev->regs[VW_ARS_REG_FB_HEIGHT] >> 16) & 0xffff);
    if (height <= 0)
        height = ARS_FB_HEIGHT;

    /* Never write outside the frame buffer the renderer gave us. */
    if (width > buffer32->w)
        width = buffer32->w;
    if (height > buffer32->h)
        height = buffer32->h;

    /* Only the parts of the frame buffer that actually changed are converted.
       The frame buffer is plain system memory, so it has no write handlers of
       its own; a shadow copy plus a per-4K compare gives the same effect
       without converting a million pixels on every field. */
    {
        uint32_t total   = (uint32_t) height * ARS_FB_WIDTH;
        int      changed = 0;
        uint32_t off;

        for (off = 0; off < total; off += 4096) {
            uint32_t       len = (total - off > 4096) ? 4096 : (total - off);
            const uint8_t *p   = mem_phys_ptr(base + off, len);
            uint32_t       i;

            {
                static int n;
                if ((off == 0) && (n++ < 3))
                    pclog("ARS chunk0 ptr=%p len=%u base=%08X\n", (void *) p, len, base);
            }

            if ((p == NULL) || (memcmp(p, dev->fb_shadow + off, len) != 0)) {
                changed = 1;

                if (p != NULL)
                    memcpy(dev->fb_shadow + off, p, len);

                for (i = 0; i < len; i++) {
                    uint32_t pos = off + i;
                    int      py  = (int) (pos / ARS_FB_WIDTH);
                    int      px  = (int) (pos % ARS_FB_WIDTH);
                    uint8_t  idx;
                    uint32_t c;

                    if ((py >= height) || (px >= width))
                        continue;

                    idx = (p != NULL) ? p[i] : mem_readb_phys(base + pos);
                    c   = dev->regs[VW_ARS_REG_COLOR_MAP_START + idx];

                    buffer32->line[py][px] = 0xff000000 | (c >> 8);
                }

                continue;
            }

            if (!dev->fb_valid)
                changed = 1;
        }

        if (!changed) {
            /* Nothing new: keep the picture on screen with an occasional
               refresh instead of redrawing it on every field. */
            if (++dev->fb_idle < 8) {
                dev->fb_valid = 1;
                return;
            }
        }

        dev->fb_idle  = 0;
        dev->fb_valid = 1;
    }

    video_blit_memtoscreen(0, 0, width, height);
}
/* Field rates the DBE can be programmed for, from the dot clock PLL and the
   total raster size: f = refclk * M / (N * 2^P) / (htotal * vtotal).  The
   27 MHz reference is the one the Linux visws timings fit (800x600 at 75 Hz
   comes out at exactly 49.5 MHz). */
static double
arsenic_refresh_hz(const ars_t *dev)
{
    uint32_t clk    = dev->regs[VW_ARS_REG_DOT_CLOCK];
    uint32_t xymax  = dev->regs[VW_ARS_REG_RETRACE_POS_MAX];
    int      m      = (int) (clk & 0xff) + 1;
    int      n      = (int) ((clk >> 8) & 0x3f) + 1;
    int      p      = (int) ((clk >> 14) & 0x03);
    int      htotal = (int) (xymax & 0xfff);
    int      vtotal = (int) ((xymax >> 12) & 0xfff);
    double   dotclk;
    double   hz;

    if (!(clk & 0x00100000) || (htotal == 0) || (vtotal == 0))
        return 0.0;

    dotclk = 27000000.0 * m / (n * (1 << p));
    hz     = dotclk / ((double) htotal * (double) vtotal);

    /* Anything outside a sane raster rate means the registers are not
       programmed yet. */
    if ((hz < 30.0) || (hz > 200.0))
        return 0.0;

    return hz;
}

static void
arsenic_blit_tick(void *priv)
{
    ars_t *dev = priv;
    double hz  = arsenic_refresh_hz(dev);
    uint64_t period;

    arsenic_blit_frame(dev);

    /* Follow the programmed field rate; fall back to 20 Hz until the
       firmware has set the dot clock up. */
    period = (hz > 0.0) ? (uint64_t) (1000000.0 / hz) : 50000ULL;
    timer_advance_u64(&dev->blit_timer, period * TIMER_USEC);
}

static void
arsenic_retrace(void *priv)
{
    ars_t   *dev  = priv;
    uint32_t max  = dev->regs[VW_ARS_REG_RETRACE_POS_MAX];
    uint32_t maxx = max & 0x0fff;
    uint32_t maxy = (max >> 12) & 0x0fff;
    uint32_t pos  = dev->regs[VW_ARS_REG_RETRACE_POS];
    uint32_t y    = (pos >> 12) & 0x0fff;

    if (maxx == 0)
        maxx = 1279;
    if (maxy == 0)
        maxy = 1023;

    y++;
    if (y > maxy)
        y = 0;

    /* x is left at the end of the line; software only uses y for vblank. */
    dev->regs[VW_ARS_REG_RETRACE_POS] = (maxx & 0x0fff) | ((y & 0x0fff) << 12);

    timer_advance_u64(&dev->retrace_timer, 1000 * TIMER_USEC);
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
                write_bits_mask = 0xFFFFFFFF;
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

        /* The "in hardware" registers mirror the enable bits of the
           corresponding control registers (sgivw.h: OVR_INHWCTRL,
           FRM_INHWCTRL and DID_INHWCTRL). */
        case VW_ARS_REG_OVERLAY_CTRL:
            dev->regs[VW_ARS_REG_OVERLAY_STATUS] =
                (dev->regs[VW_ARS_REG_OVERLAY_STATUS] & ~0x00000001) | (val & 0x00000001);
            break;

        case VW_ARS_REG_FB_CTRL:
            dev->regs[VW_ARS_REG_FB_STATUS] =
                (dev->regs[VW_ARS_REG_FB_STATUS] & ~0x00000001) | (val & 0x00000001);
            break;

        case VW_ARS_REG_DID_CTRL:
            dev->regs[VW_ARS_REG_DID_STATUS] =
                (dev->regs[VW_ARS_REG_DID_STATUS] & ~0x00010000) | ((val & 0x00000001) << 16);
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

    free(dev->fb_shadow);
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

    /* The firmware addresses the ASIC through the window at BD000000h. */
    mem_mapping_add(&dev->mmio_alt_mapping,
                    0xBD000000,
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

    timer_add(&dev->retrace_timer, arsenic_retrace, dev, 1);
    timer_add(&dev->blit_timer, arsenic_blit_tick, dev, 1);

    dev->fb_shadow_size = 1024 * 1024;
    dev->fb_shadow      = calloc(1, dev->fb_shadow_size);

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
