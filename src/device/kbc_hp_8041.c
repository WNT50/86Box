/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the HP 8041 keyboard controller used by the
 *          HP Vectra 45945A (1985).
 *
 *          The 8041 replaces the PC/AT's 8042. It keeps the familiar
 *          0x60/0x64 pair for the "application path", but adds an HP path
 *          of its own:
 *
 *            0x68 (W)  HP control buffer (8041 port 1 shadow)
 *            0x69 (R)  service request status; a code of 0x06 means the
 *                      request that was started with 0x6A is done
 *            0x6A (W)  clear HPINT / start of a service request
 *
 *          The system ROM's keyboard initialisation at F000:1DC2 is the
 *          authority for the behaviour implemented here:
 *
 *            F1DC2  in al,0x60 / cmp al,0x55   -- BAT OK, else error 0x303
 *            F1DCC  out 0x6a,al / in al,0x69   -- waits for 0x69 to read 0x06
 *            F1DE0  out 0x64,0x60 / out 0x60,0x45 -- read 8041 port 1
 *            F1DFD  in al,0x69 / cmp al,0x6    -- same wait, second time
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
#include <86box/io.h>
#include <86box/device.h>
#include <86box/keyboard.h>

#ifdef ENABLE_KBC_HP_LOG
int kbc_hp_do_log = ENABLE_KBC_HP_LOG;

static void
kbc_hp_log(const char *fmt, ...)
{
    va_list ap;

    if (kbc_hp_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define kbc_hp_log(fmt, ...)
#endif

typedef struct kbc_hp_t {
    uint8_t out;         /* output buffer read at 0x60 */
    uint8_t obf;         /* output buffer full */
    uint8_t ibf;         /* input buffer full */
    uint8_t awaiting;    /* next 0x60 write is data for the pending command */
    uint8_t cmd;         /* last command written to 0x64 */
    uint8_t hp_ctrl;     /* 0x68 control buffer */
    uint8_t svc;         /* 0x69 service request status */
    uint8_t port1;       /* 8041 port 1 (HP path data) */
    uint8_t kbd_enabled;
} kbc_hp_t;

static kbc_hp_t *kbc_hp_dev;

static void kbc_hp_reset(void *priv);

static void
kbc_hp_out(uint8_t val)
{
    kbc_hp_t *dev = (kbc_hp_t *) kbc_hp_dev;

    dev->out   = val;
    dev->obf   = 1;
}

static void
kbc_hp_write(uint16_t port, uint8_t val, void *priv)
{
    kbc_hp_t *dev = (kbc_hp_t *) priv;

    kbc_hp_log("[%04X:%08X] KBC-HP out(%04X, %02X)\n", CS, cpu_state.pc, port, val);

    switch (port) {
        case 0x0060:
            dev->ibf = 0;
            if (dev->awaiting) {
                dev->awaiting = 0;
                switch (dev->cmd) {
                    case 0x60:
                        /* HP path: 0x45 asks the 8041 for its port 1. */
                        if (val == 0x45) {
                            kbc_hp_out(dev->port1);
                            dev->svc = 0x06;
                        } else {
                            dev->hp_ctrl = val;
                            dev->svc     = 0x06;
                        }
                        break;
                    default:
                        kbc_hp_out(0xfa);
                        break;
                }
            } else {
                /* Data for the keyboard itself; acknowledge it. */
                kbc_hp_out(0xfa);
            }
            break;

        case 0x0064:
            dev->ibf      = 1;
            dev->cmd      = val;
            dev->awaiting = 0;
            switch (val) {
                case 0x20: /* read command byte */
                case 0x60: /* HP path: data follows */
                    dev->awaiting = 1;
                    break;
                case 0xaa: /* self test */
                    kbc_hp_out(0x55);
                    break;
                case 0xab: /* keyboard interface test */
                case 0xae: /* enable keyboard interface */
                    dev->kbd_enabled = 1;
                    kbc_hp_out(0x00);
                    break;
                case 0xad: /* disable keyboard interface */
                    dev->kbd_enabled = 0;
                    kbc_hp_out(0x00);
                    break;
                default:
                    kbc_hp_out(0xfa);
                    break;
            }
            dev->ibf = 0;
            break;

        case 0x0068:
            dev->hp_ctrl = val;
            break;

        case 0x006a:
            /* Clear HPINT: the service request has been consumed. */
            dev->svc = 0x00;
            break;

        default:
            break;
    }
}

static uint8_t
kbc_hp_read(uint16_t port, void *priv)
{
    kbc_hp_t *dev = (kbc_hp_t *) priv;
    uint8_t   ret = 0xff;

    switch (port) {
        case 0x0060:
            ret      = dev->out;
            dev->obf = 0;
            break;

        case 0x0064:
            /* bit 0: output buffer full, bit 1: input buffer full,
               bit 2: system flag (POST passed), bit 3: last access was
               a command, bit 4: keyboard enabled. */
            ret = (dev->obf ? 0x01 : 0x00) | (dev->ibf ? 0x02 : 0x00) |
                  0x04 | (dev->kbd_enabled ? 0x10 : 0x00);
            break;

        case 0x0068:
            ret = dev->hp_ctrl;
            break;

        case 0x0069:
            ret = dev->svc;
            break;

        case 0x006a:
            ret = 0x00;
            break;

        default:
            break;
    }

    kbc_hp_log("[%04X:%08X] KBC-HP in(%04X) = %02X\n", CS, cpu_state.pc, port, ret);

    return ret;
}

static void *
kbc_hp_init(const device_t *info)
{
    kbc_hp_t *dev = (kbc_hp_t *) calloc(1, sizeof(kbc_hp_t));

    kbc_hp_dev = dev;

    dev->port1       = 0x00;
    dev->kbd_enabled = 0;

    kbc_hp_reset(dev);

    io_sethandler(0x0060, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);
    io_sethandler(0x0064, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);
    io_sethandler(0x0068, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);
    io_sethandler(0x0069, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);
    io_sethandler(0x006a, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);

    return dev;
}

static void
kbc_hp_close(void *priv)
{
    kbc_hp_t *dev = (kbc_hp_t *) priv;

    io_removehandler(0x0060, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);
    io_removehandler(0x0064, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);
    io_removehandler(0x0068, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);
    io_removehandler(0x0069, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);
    io_removehandler(0x006a, 1, kbc_hp_read, NULL, NULL, kbc_hp_write, NULL, NULL, dev);

    kbc_hp_dev = NULL;

    free(dev);
}

static void
kbc_hp_reset(void *priv)
{
    kbc_hp_t *dev = (kbc_hp_t *) priv;

    dev->obf      = 0;
    dev->ibf      = 0;
    dev->awaiting = 0;
    dev->cmd      = 0x00;
    dev->svc      = 0x00;

    /* The 8041 reports BAT OK (0x55) after a reset; POST compares it. */
    dev->out = 0x55;
    dev->obf = 1;
}

const device_t kbc_hp_8041_device = {
    .name          = "HP 8041 Keyboard Controller",
    .internal_name = "kbc_hp_8041",
    .flags         = DEVICE_KBC,
    .local         = 0,
    .init          = kbc_hp_init,
    .close         = kbc_hp_close,
    .reset         = kbc_hp_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
