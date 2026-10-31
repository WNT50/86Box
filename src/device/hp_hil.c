/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          HP-HIL host controller (U777) as used by the HP Vectra 45945A.
 *
 *          Unlike MAME's MLC model, the Vectra's U777 is a plain 8-bit
 *          "index + data + status" register pair. The system ROM drives it
 *          like this (addresses are F000 offsets):
 *
 *            F3830  mov al,bh / or al,8 / out 0x6d   -- index, bit 3 = command
 *                   mov al,0x10 / out 0x6c           -- 0x10 = read request
 *                   call 0x38b7                      -- wait for 0x6f bit 0
 *                   in al,0x6e                       -- error, must be 0
 *                   in al,0x6d / in al,0x6c          -- read back
 *            F371B  mov al,8 / out 0x6d
 *                   mov al,0xfe / out 0x6c           -- write/broadcast
 *            F3820  in al,0x6d / in al,0x6c           -- poll until 0x6d bit 3
 *            F3800  read-back check: 0x6d, then 0x6c must read 0x80
 *
 *          So: index in 0x6d (bit 3 = command), data and results in 0x6c,
 *          error in 0x6e, "transfer complete" in 0x6f bit 0.
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
#include <86box/keyboard.h>
#include <86box/device.h>
#include <86box/pic.h>
extern void kbc_at_ext_out(uint8_t val);

#define HP_HIL_REGS 16

#ifdef ENABLE_HP_HIL_LOG
int hp_hil_do_log = ENABLE_HP_HIL_LOG;

static void
hp_hil_log(const char *fmt, ...)
{
    va_list ap;

    if (hp_hil_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#    define hp_hil_log(fmt, ...)
#endif

typedef struct hp_hil_t {
    uint8_t index;   /* 0x6d latch */
    uint8_t data;    /* 0x6c latch (last write, or last read result) */
    uint8_t error;   /* 0x6e */
    uint8_t busy;    /* 0x6f bit 0 = 1 -> transfer complete */
    uint8_t command; /* last command byte written while bit 3 was set */
    uint8_t hp_ctrl; /* 0x68: 8041 extension data */
    uint8_t svc;     /* 0x69: service request status (0x06 = done) */
    uint8_t regs[HP_HIL_REGS];
} hp_hil_t;

static void
hp_hil_write(uint16_t port, uint8_t val, void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;

    hp_hil_log("[%04X:%08X] HIL out(%03X, %02X)\n", CS, cpu_state.pc, port, val);

    switch (port) {
        case 0x0068:
            dev->hp_ctrl = val;
            kbc_at_ext_out(val);    /* loop the byte back through 0x60 + OBF */
            break;

        case 0x006a:
            /* Acknowledge: the service status drops back to zero. */
            dev->svc = 0x00;
            break;
        case 0x006d:
            dev->index = val;
            break;

        case 0x006c:
            if (getenv("EPOCH_HIL_TRACE"))
                pclog("HIL: out 6C val=%02X idx=%02X pc=%04X\n", val, dev->index, cpu_state.pc);
            dev->data = val;
            dev->error = 0x00;
            if (dev->index & 0x08) {
                /* Command mode: 0x07 = write register, 0x10 = read register. */
                uint8_t reg = dev->index & 0x07;

                dev->command = val;
                switch (val) {
                    case 0x07:
                        dev->regs[reg] = dev->data;
                        break;
                    case 0x10:
                        dev->data = dev->regs[reg];
                        break;
                    default:
                        break;
                }
            }
            dev->busy = 1;
            if (dev->index & 0x08) {
                /* HP-HIL is IRQ 20 (third 8259, line 4). It is a latched
                   edge: the request stays in the IRR until the CPU takes it,
                   which is what lets POST serve it from its sti loop. */
                if (getenv("EPOCH_HIL_TRACE"))
                    pclog("HIL: IRQ20 raise pc=%04X\n", cpu_state.pc);
                pic3_int(4, 1);
            }
            break;

        case 0x006e:
            dev->error = val;
            break;

        default:
            break;
    }
}

static uint8_t
hp_hil_read(uint16_t port, void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;
    uint8_t   ret = 0xff;

    switch (port) {
        case 0x0068:
            return dev->hp_ctrl;

        case 0x0069:
            return dev->svc;

        case 0x006a:
            return 0x00;
        case 0x006c:
            ret      = dev->data;
            dev->busy = 0;
            break;

        case 0x006d:
            ret = dev->index;
            break;

        case 0x006e:
            ret = dev->error;
            break;

        case 0x006f:
            ret = dev->busy ? 0x01 : 0x00;
            break;

        default:
            break;
    }

    hp_hil_log("[%04X:%08X] HIL in(%03X) = %02X\n", CS, cpu_state.pc, port, ret);

    return ret;
}

/* The 0x45 data write goes to the KBC, so the KBC tells us about it. */
extern void (*kbc_at_ext_write)(uint16_t port, uint8_t val);

static hp_hil_t *hp_hil_dev;

static void
hp_hil_kbc_ext(uint16_t port, uint8_t val)
{
    if ((port == 0x0060) && (val == 0x45) && hp_hil_dev)
        hp_hil_dev->svc = 0x06;
}

static void *
hp_hil_init(const device_t *info)
{
    hp_hil_t *dev = (hp_hil_t *) calloc(1, sizeof(hp_hil_t));

    hp_hil_dev      = dev;
    kbc_at_ext_write = hp_hil_kbc_ext;

    io_sethandler(0x0068, 3, hp_hil_read, NULL, NULL, hp_hil_write, NULL, NULL, dev);
    io_sethandler(0x006c, 4, hp_hil_read, NULL, NULL, hp_hil_write, NULL, NULL, dev);

    return dev;
}

static void
hp_hil_close(void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;

    kbc_at_ext_write = NULL;
    hp_hil_dev      = NULL;

    io_removehandler(0x0068, 3, hp_hil_read, NULL, NULL, hp_hil_write, NULL, NULL, dev);
    io_removehandler(0x006c, 4, hp_hil_read, NULL, NULL, hp_hil_write, NULL, NULL, dev);

    free(dev);
}

static void
hp_hil_reset(void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;

    memset(dev->regs, 0x00, sizeof(dev->regs));
    dev->index   = 0x00;
    dev->data    = 0x00;
    dev->error   = 0x00;
    dev->busy    = 1;
    dev->command = 0x00;
    dev->hp_ctrl = 0x00;
    dev->svc     = 0x06;
}

const device_t hp_hil_device = {
    .name          = "HP-HIL Host Controller (U777)",
    .internal_name = "hp_hil",
    .flags         = 0,
    .local         = 0,
    .init          = hp_hil_init,
    .close         = hp_hil_close,
    .reset         = hp_hil_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
