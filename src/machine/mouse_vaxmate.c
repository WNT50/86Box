/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          DEC VAXmate mouse: three-button Digital mouse on the SCN2661
 *          USART at I/O port 0x0C40, interrupt on IRQ12.
 *
 *          Position report (TRM 2.11.1.3):
 *            byte 1  7 = sign of X (1 = positive), 6 = sign of Y,
 *                    5 = left, 4 = middle, 3 = right, 2-0 = 100b
 *            byte 2  bit 7 = 0, bits 6-0 = |dX|
 *            byte 3  bit 7 = 0, bits 6-0 = |dY|
 *
 *          Self-test report (TRM 2.11.2.2), four bytes:
 *            byte 1  101b frame sync, revision in bits 4-1
 *            byte 2  manufacturer in bits 7-5, device code 0010b,
 *                    which marks the data as coming from a mouse
 *            byte 3  error code, 0 = OK
 *            byte 4  button state, 0 = OK
 *
 *          The mouse is the USART's backend: the chip pulls a byte whenever
 *          its receive holding register frees up and hands whatever the
 *          guest transmits to the command parser here.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/mouse.h>
#include <86box/serial.h>
#include <86box/plat_unused.h>
#include <86box/mouse_vaxmate.h>

#define MOUSE_CMD_PROMPT      0x44 /* 'D' */
#define MOUSE_CMD_REQUEST     0x50 /* 'P' */
#define MOUSE_CMD_INCREMENTAL 0x52 /* 'R' */
#define MOUSE_CMD_SELFTEST    0x54 /* 'T' */
#define MOUSE_CMD_VENDOR      0x5a /* 'Z' */

#define MOUSE_MODE_PROMPT      0
#define MOUSE_MODE_INCREMENTAL 1

#define MOUSE_QUEUE 32

typedef struct vaxmate_mouse_t {
    serial_backend_t be;
    pc_timer_t       timer;

    uint8_t queue[MOUSE_QUEUE];
    int     q_head;
    int     q_tail;

    int mode;
    int vendor_arg; /* the 'Z' command takes one argument byte */
    int ticks;      /* 1 ms ticks, used to sample at 100 Hz */
    int buttons;    /* last button state, 86Box bit order */
} vaxmate_mouse_t;

static vaxmate_mouse_t *vaxmate_mouse_inst;

static void
vaxmate_mouse_queue(vaxmate_mouse_t *dev, uint8_t val)
{
    int next = (dev->q_tail + 1) % MOUSE_QUEUE;

    if (next == dev->q_head)
        return; /* the guest is not reading: drop, as the real mouse would */

    dev->queue[dev->q_tail] = val;
    dev->q_tail             = next;
}

/* Three-byte position report; the caller has already clamped the deltas. */
static void
vaxmate_mouse_report(vaxmate_mouse_t *dev, int dx, int dy)
{
    uint8_t b1 = 0x04; /* bits 2-0 always read 100b */

    if (dx >= 0)
        b1 |= 0x80;
    if (dy >= 0)
        b1 |= 0x40;
    if (dev->buttons & 0x01)
        b1 |= 0x20; /* left */
    if (dev->buttons & 0x04)
        b1 |= 0x10; /* middle */
    if (dev->buttons & 0x02)
        b1 |= 0x08; /* right */

    vaxmate_mouse_queue(dev, b1);
    vaxmate_mouse_queue(dev, (uint8_t) ((dx < 0) ? -dx : dx) & 0x7f);
    vaxmate_mouse_queue(dev, (uint8_t) ((dy < 0) ? -dy : dy) & 0x7f);
}

/* Four-byte power-up self test and identification report. */
static void
vaxmate_mouse_selftest(vaxmate_mouse_t *dev)
{
    vaxmate_mouse_queue(dev, 0xa0); /* 101b frame sync, revision 0 */
    vaxmate_mouse_queue(dev, 0x22); /* manufacturer 1, device code 0010b */
    vaxmate_mouse_queue(dev, 0x00); /* no error */
    vaxmate_mouse_queue(dev, 0x00); /* no button stuck */
}

/* A byte the guest wrote to the USART transmit register. */
static void
vaxmate_mouse_command(vaxmate_mouse_t *dev, uint8_t val)
{
    if (dev->vendor_arg) {
        dev->vendor_arg = 0;
        return;
    }

    switch (val) {
        case MOUSE_CMD_PROMPT:
            dev->mode = MOUSE_MODE_PROMPT;
            break;

        case MOUSE_CMD_INCREMENTAL:
            dev->mode = MOUSE_MODE_INCREMENTAL;
            break;

        case MOUSE_CMD_REQUEST:
            dev->mode    = MOUSE_MODE_PROMPT;
            dev->buttons = mouse_get_buttons_ex();
            vaxmate_mouse_report(dev, 0, 0);
            break;

        case MOUSE_CMD_SELFTEST:
            dev->mode = MOUSE_MODE_PROMPT;
            vaxmate_mouse_selftest(dev);
            break;

        case MOUSE_CMD_VENDOR:
            dev->vendor_arg = 1;
            break;

        default:
            break;
    }
}

/* ---- the backend the USART talks to ------------------------------------ */

/* One byte per call. The chip asks again as soon as the guest takes the
   previous character, so a four byte self test drains within one poll. */
static int
vaxmate_mouse_read_byte(void *priv)
{
    vaxmate_mouse_t *dev = (vaxmate_mouse_t *) priv;
    uint8_t          val;

    if (dev->q_head == dev->q_tail)
        return -1;

    val         = dev->queue[dev->q_head];
    dev->q_head = (dev->q_head + 1) % MOUSE_QUEUE;

    return (int) val;
}

static void
vaxmate_mouse_write_byte(void *priv, uint8_t val)
{
    vaxmate_mouse_command((vaxmate_mouse_t *) priv, val);
}

/* The protocol is fixed at 4800 8O1; nothing to reconfigure. */
static void
vaxmate_mouse_set_format(UNUSED(void *priv), UNUSED(int baud), UNUSED(int data_bits),
                         UNUSED(int parity), UNUSED(int stop_bits))
{
}

static void
vaxmate_mouse_poll(void *priv)
{
    vaxmate_mouse_t *dev = (vaxmate_mouse_t *) priv;
    int              dx  = 0;
    int              dy  = 0;
    int              buttons;

    /* Incremental stream mode reports at 100 Hz, and only when something
       actually changed (TRM 2.11.2). */
    if ((dev->mode == MOUSE_MODE_INCREMENTAL) && ((++dev->ticks % 10) == 0)) {
        buttons = mouse_get_buttons_ex();

        mouse_subtract_coords(&dx, &dy, NULL, NULL, -127, 127, 0, 0);

        if (dx != 0 || dy != 0 || buttons != dev->buttons) {
            dev->buttons = buttons;
            vaxmate_mouse_report(dev, dx, dy);
        }
    }

    timer_advance_u64(&dev->timer, 1000 * TIMER_USEC);
}

/* The machine binds this to the USART once both devices exist. */
const serial_backend_t *
vaxmate_mouse_backend(void)
{
    return (vaxmate_mouse_inst != NULL) ? &vaxmate_mouse_inst->be : NULL;
}

static void *
vaxmate_mouse_init(const device_t *info)
{
    vaxmate_mouse_t *dev = calloc(1, sizeof(vaxmate_mouse_t));

    dev->be.priv       = dev;
    dev->be.read_byte  = vaxmate_mouse_read_byte;
    dev->be.write_byte = vaxmate_mouse_write_byte;
    dev->be.set_format = vaxmate_mouse_set_format;

    dev->mode    = MOUSE_MODE_PROMPT;
    dev->buttons = 0;

    timer_add(&dev->timer, vaxmate_mouse_poll, dev, 1);

    vaxmate_mouse_inst = dev;

    return dev;
}

static void
vaxmate_mouse_close(void *priv)
{
    vaxmate_mouse_t *dev = (vaxmate_mouse_t *) priv;

    timer_disable(&dev->timer);

    if (vaxmate_mouse_inst == dev)
        vaxmate_mouse_inst = NULL;

    free(dev);
}

static void
vaxmate_mouse_reset(void *priv)
{
    vaxmate_mouse_t *dev = (vaxmate_mouse_t *) priv;

    dev->q_head     = dev->q_tail = 0;
    dev->mode       = MOUSE_MODE_PROMPT;
    dev->vendor_arg = 0;
    dev->ticks      = 0;
    dev->buttons    = 0;
}

const device_t vaxmate_mouse_device = {
    .name          = "DEC VAXmate Mouse",
    .internal_name = "vaxmate_mouse",
    .flags         = DEVICE_ONBOARD,
    .local         = 0,
    .init          = vaxmate_mouse_init,
    .close         = vaxmate_mouse_close,
    .reset         = vaxmate_mouse_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
