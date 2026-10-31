/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          HP-HIL host adapter (U777) of the HP Vectra 45945A, modelled as
 *          an HP-HIL Master Link Controller plus the devices on the link.
 *
 *          The System sees a four register window at 0x6C-0x6F, which is the
 *          MLC's W0/W1/R0/R1/R2/R3 set (HP-HIL Technical Reference Manual,
 *          HP 45918A, Appendix I):
 *
 *            0x6C  W0/R0   frame data byte        (write sends the frame)
 *            0x6D  W1/R1   command bit + address  (bit 3 = command bit)
 *            0x6E  R2      parity/frame/overflow errors
 *            0x6F  R3      INT / NMI / link error (bit 0 = frame available)
 *
 *          Every read of R3 acknowledges the interrupt, every R0 read pops a
 *          frame from the receive FIFO, and R1 peeks at the frame that is on
 *          top of it. The System ROM drives it exactly like that:
 *
 *            F3830  mov al,bh / or al,8 / out 0x6d   -- W1: command, address
 *                   mov al,0x10 / out 0x6c           -- W0: POL
 *                   call 0x38b7                      -- wait for 0x6f bit 0
 *                   in al,0x6e                       -- R2 must be 0
 *                   in al,0x6d / in al,0x6c          -- R1 / R0
 *            F371B  mov al,8 / out 0x6d
 *                   mov al,0xfe / out 0x6c           -- DHR, broadcast
 *
 *          The keyboard answers ACF (address assignment), IDD (identify and
 *          describe) and POL (poll) the way hlekbd.cpp in MAME does, which in
 *          turn follows Appendix C and E of the same manual.
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
#include <86box/mem.h>
#include <86box/timer.h>

/* HP-HIL opcodes (Table 4-1). */
#define HIL_IFC 0x00 /* Interface Clear */
#define HIL_EPT 0x01 /* Enter Pass-Thru Mode */
#define HIL_ELB 0x02 /* Enter Loop-Back Mode */
#define HIL_IDD 0x03 /* Identify and Describe */
#define HIL_DSR 0x04 /* Device Soft Reset */
#define HIL_PST 0x05 /* Perform Self Test */
#define HIL_RRG 0x06 /* Read Register */
#define HIL_WRG 0x07 /* Write Register */
#define HIL_ACF 0x08 /* Auto Configure, 08..0f */
#define HIL_POL 0x10 /* Poll, 10..1f */
#define HIL_RPL 0x20 /* RePoll, 20..2f */
#define HIL_DHR 0xfe /* Device Hard Reset */

/* MLC register bits (Appendix I). */
#define HIL_R2_PERR 0x01
#define HIL_R2_FERR 0x02
#define HIL_R2_FOF  0x04
#define HIL_R3_INT  0x01
#define HIL_R3_NMI  0x02
#define HIL_R3_LERR 0x04

#define HIL_FIFO_SIZE 16

/* HP-HIL IRQ 20 is the third 8259's line 4 (vector 6Ch). */
#define HIL_IRQ_LINE 4

/* The keyboard of this machine hangs off the 8041 (0x60/0x64 and the
   0x68-0x6A mailbox), so the only device on the HP-HIL link is the mouse.
   Relative positioners are 60..6Fh; 68h is the HP Mouse 46060A. */
#define HIL_MOUSE_ID    0x68
/* Leading byte of a poll record: keycode set 1, relative coordinates. */
/* Poll header bits 1-0: 00 none, 01 X, 10 X and Y, 11 X, Y and Z. */
#define HIL_POL_AXES_XY 0x02
#define HIL_POL_RECORD  0x40
/* Describe header: 8-bit relative X and Y, I/O descriptor byte follows. */
#define HIL_MOUSE_DESCR 0x12

typedef struct hp_hil_t {
    /* MLC state. */
    uint16_t fifo[HIL_FIFO_SIZE];
    int      fifo_head;
    int      fifo_count;
    uint8_t  w1;   /* command bit + address of the frame being built */
    uint8_t  w2;
    uint8_t  w3;
    uint8_t  r2;
    uint8_t  r3;
    uint8_t  loop; /* Loop-Back Mode: frames come straight back */

    /* Keyboard on the link. */
    uint8_t kbd_addr;     /* 0 = not configured yet */
    uint8_t kbd_id;
    uint8_t kbd_passthru;
    uint8_t  scan_reported;
    int8_t   dx;
    int8_t   dy;

} hp_hil_t;

static hp_hil_t *hp_hil_dev;

static void hp_hil_link_ready(void *priv);

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

static void
hp_hil_trace(const char *fmt, ...)
{
    va_list ap;

    if (getenv("EPOCH_HIL_TRACE") == NULL)
        return;

    va_start(ap, fmt);
    pclog_ex(fmt, ap);
    va_end(ap);
}

static void
hp_hil_irq(void)
{
    hp_hil_t *dev = hp_hil_dev;

    /* The MLC's INT output follows the FIFO, it is not a one-shot latch:
       it stays up until the System has taken every queued frame, and drops
       again as soon as the last one is read. */
    if (dev->fifo_count) {
        dev->r3 |= HIL_R3_INT;
        pic3_int(HIL_IRQ_LINE, 1);
    } else {
        dev->r3 &= (uint8_t) ~HIL_R3_INT;
        pic3_int(HIL_IRQ_LINE, 0);
    }
}

/* A frame the System has to read: data byte plus its command/address byte. */
static void
hp_hil_fifo_push(uint16_t frame)
{
    hp_hil_t *dev = hp_hil_dev;

    hp_hil_trace("HIL: push %03X\n", frame);
    if (dev->fifo_count >= HIL_FIFO_SIZE) {
        dev->r2 |= HIL_R2_FOF;
        hp_hil_irq();
        return;
    }

    dev->fifo[(dev->fifo_head + dev->fifo_count) % HIL_FIFO_SIZE] = frame;
    dev->fifo_count++;

    hp_hil_irq();
}

/* The HIL driver keeps its context in RAM and finds it through the int 6Fh
   record table: IVT 6Fh -> record 0x114 -> its data segment.  The 8041 owns
   the bring-up, so this is where the "link initialised" bit of that context
   is set; without it the driver parks in state 2 and POST waits forever. */
static uint32_t
hp_hil_driver_ctx(void)
{
    uint16_t ivt = (uint16_t) (mem_readb_phys(0x1be) | (mem_readb_phys(0x1bf) << 8));
    uint16_t dseg = mem_readw_phys((uint32_t) ivt * 16 + 0x114 + 4);

    if (dseg == 0x0000)
        return 0;

    return (uint32_t) dseg * 16;
}

int
hp_hil_link_up(void)
{
    return (hp_hil_dev != NULL) && hp_hil_dev->kbd_passthru;
}

void
hp_hil_link_scan(void)
{
    hp_hil_t *dev = hp_hil_dev;
    uint32_t  ctx;

    if (dev == NULL)
        return;

    /* A device answered the scan (the mouse is on the link), so the link is
       up.  The "link ready" event itself comes from hp_hil_link_ready(). */
    ctx = hp_hil_driver_ctx();
    if ((ctx != 0) && (dev->kbd_id != 0x00)) {
        /* The driver asks the 8041 for the link right before it starts
           waiting; that is when a real 8041 reports the link is up. */
        hp_hil_link_ready(dev);
        return;
    }

    /* Nothing on the link has been given an address (the System never issues
       ACF/IDD itself, that is the 8041's job), so the scan finds no device.
       Report it the way the MLC does: a frame error in R2 and LERR in R3,
       with INT raised so the System comes and looks. */
    /* Only report once per error: the status is cleared when the System reads
       R2, and the next reset command arms it again. */
    if (dev->scan_reported)
        return;
    dev->scan_reported = 1;

    dev->r2 |= HIL_R2_FERR;
    dev->r3 |= HIL_R3_LERR;
    pic3_int(HIL_IRQ_LINE, 1);
}

static uint16_t
hp_hil_fifo_peek(void)
{
    hp_hil_t *dev = hp_hil_dev;

    if (dev->fifo_count == 0)
        return 0;

    return dev->fifo[dev->fifo_head];
}

static uint16_t
hp_hil_fifo_pop(void)
{
    hp_hil_t *dev = hp_hil_dev;
    uint16_t  frame;

    if (dev->fifo_count == 0)
        return 0;

    frame     = dev->fifo[dev->fifo_head];
    dev->fifo_head = (dev->fifo_head + 1) % HIL_FIFO_SIZE;
    dev->fifo_count--;

    hp_hil_irq();

    return frame;
}

/* The frame a device answers with: its own address, no command bit. */
static uint16_t
hp_hil_reply(uint8_t addr, uint8_t data)
{
    return (uint16_t) (((uint16_t) (addr & 7) << 8) | data);
}

/*
 * The single device on the link: an HP Mouse 46060A.  It starts out in
 * loop-back so the System's link self test sees its own frames come back;
 * EPT takes it out of loop-back and from then on it answers as a device.
 */
static void
hp_hil_kbd_frame(uint16_t frame)
{
    hp_hil_t *dev = hp_hil_dev;
    uint8_t   addr;
    uint8_t   data;
    int       command;

    data    = (uint8_t) (frame & 0xff);
    addr    = (uint8_t) ((frame >> 8) & 0x07);
    command = (frame >> 11) & 1;

    /* Loop-Back Mode closes the end of the link: whatever arrives is handed
       straight back, which is what the System's self test looks for.  A
       Device Hard Reset is the exception - it is link-wide, every device acts
       on it, and handing it back would make the System read its own reset as
       a reply (and send another one, forever). */
    /* Anything outside the command set is a link test (the System sends A5h
       and 5Ah frames and expects to see them again, and 80h the same way).
       Those are answered no matter what mode the device is in or which
       address they carry. */
    if (command && (data != HIL_DHR) && (data > 0x46)) {
        hp_hil_fifo_push(frame);
        return;
    }

    /* A loop-back device hands link-wide frames back; a frame addressed to
       somebody else is only passed on, and one addressed to this device is
       answered instead of echoed. */
    if (dev->loop && command && (addr != 0) && (addr != dev->kbd_addr)) {
        hp_hil_fifo_push(frame);
        return;
    }

    if (!command) {
        if (dev->loop && (addr == 0))
            hp_hil_fifo_push(frame);
        return;
    }
    if ((addr != 0) && (addr != dev->kbd_addr))
        return;

    hp_hil_trace("HIL: device <= %03X (addr %d, op %02X)\n", frame, addr, data);

    switch (data) {
        case HIL_IFC:
            dev->kbd_addr = 0;
            break;

        case HIL_EPT:
            /* Enter Poll Transfer: the link is no longer a mirror, commands
               now reach this device and it is expected to answer them. */
            dev->kbd_passthru = 1;
            dev->loop         = 0;
            hp_hil_trace("HIL: link leaves loop-back (EPT)\n");

            break;

        case HIL_ELB:
            dev->kbd_passthru = 0;
            dev->loop         = 1;
            hp_hil_trace("HIL: link back in loop-back (ELB)\n");
            break;

        case HIL_DSR:
            dev->kbd_addr = 0;
            break;

        case HIL_PST:
            hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr, 0x00));
            break;

        case HIL_IDD:
            /* ID byte, then the Describe record (Appendix C): header, the
               16-bit resolution that follows a non-zero axis count, and the
               I/O descriptor byte announcing the buttons. */
            hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr, dev->kbd_id));
            hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr, HIL_MOUSE_DESCR));
            hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr, 0xb4)); /* 180 counts/m, low */
            hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr, 0x00)); /* high */
            hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr, 0x03)); /* two buttons */
            break;

        case HIL_ACF + 0:
        case HIL_ACF + 1:
        case HIL_ACF + 2:
        case HIL_ACF + 3:
        case HIL_ACF + 4:
        case HIL_ACF + 5:
        case HIL_ACF + 6:
        case HIL_ACF + 7:
            /* The low three bits of the opcode are the address handed out.
               The frame goes back with the next address in it, which is how
               the System learns where the chain ends (F000:3782 reads the
               byte, subtracts 8 and uses the result as the address for the
               next command). */
            dev->kbd_addr = data & 0x07;
            hp_hil_trace("HIL: mouse configured as address %d\n", dev->kbd_addr);
            if (dev->loop && (addr == 0))
                hp_hil_fifo_push((uint16_t) ((frame & 0xff00) |
                                             (((dev->kbd_addr + 1) & 0x07) | HIL_ACF)));
            return;

        case HIL_DHR:
            /* Link-wide: every device acts on it and none hands it back. */
            dev->kbd_addr      = 0;
            dev->kbd_passthru = 0;
            /* The 8041 reports the link to the driver shortly after the
               reset, and the driver waits for it at F000:92AB. */
            return;

        default:
            if ((data >= HIL_POL) && (data <= (HIL_POL + 0x0f))) {
                /* Poll record (TRM appendix E): the header says which axes
                   follow in bits 1-0, and a device with nothing to report
                   answers with no record at all - only the poll command
                   comes back. */
                if (dev->dx || dev->dy) {
                    hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr,
                                                  HIL_POL_AXES_XY));
                    hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr, (uint8_t) dev->dx));
                    hp_hil_fifo_push(hp_hil_reply(dev->kbd_addr, (uint8_t) dev->dy));
                    dev->dx = 0;
                    dev->dy = 0;
                }
            } else if ((data >= HIL_RPL) && (data <= (HIL_RPL + 0x0f))) {
                /* RePoll: an empty record is just the incremented command. */
            }
            break;
    }

    /* The command frame goes back down the link after the reply: that is how
       the System learns the device is there and what it answered
       (F000:3820 drains until it sees the command come back, F000:37F5 uses
       it as a link test). */
    if ((data >= HIL_POL) && (data <= (HIL_POL + 0x0f))) {
        /* The record ends with the poll command incremented, so the next
           device on the link sees its own address. */
        hp_hil_fifo_push((uint16_t) ((frame & ~0x0700) |
                                     (((addr + 1) & 0x07) << 8)));
    } else {
        hp_hil_fifo_push(frame);
    }

    /* The 8041 watches the link: while the driver is parked in its wait
       loop, the next frame is also what makes it report "link up". */
    hp_hil_link_ready(dev);
}

/* The System wrote W0: the frame is complete, put it on the link. */
static void
hp_hil_transmit(uint8_t data)
{
    hp_hil_t *dev = hp_hil_dev;
    uint16_t  frame;
    uint8_t   addr = dev->w1 & 0x07;
    int       command = (dev->w1 >> 3) & 1;

    frame = (uint16_t) (((uint16_t) (dev->w1 & 0x0f) << 8) | data);
    if (command)
        frame |= 0x0800;

    hp_hil_trace("HIL: send %03X (addr %d, %s %02X)%s\n", frame, addr,
                 command ? "op" : "data", data, dev->loop ? " [loop]" : "");

    /* The adapter is only the wire: a device in loop-back hands the frame
       back itself (HP-HIL TRM 1-10), and a device in pass-thru answers the
       commands addressed to it. */
    hp_hil_kbd_frame(frame);
}

static void
hp_hil_link_ready(void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;
    uint32_t  ctx = hp_hil_driver_ctx();
    uint16_t  flags;

    if (ctx == 0)
        return;

    flags = mem_readw_phys(ctx + 0x10);
    if (!(flags & 0x0001))
        return;                         /* driver is not waiting */

    mem_writew_phys(ctx + 0x10, (uint16_t) (flags | 0x1004));
    pic3_int(HIL_IRQ_LINE, 1);
}

static void
hp_hil_write(uint16_t port, uint8_t val, void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;

    switch (port) {
        case 0x006d: /* W1: command bit + address of the next frame */
            dev->w1 = val;
            break;

        case 0x006c: /* W0: frame data, writing it sends the frame */
            dev->r2 = 0x00;
            /* A new command starts a new transaction: drop stale replies. */
            if ((val == HIL_IFC) && ((dev->w1 & 0x08) != 0) && ((dev->w1 & 0x07) == 0)) {
                dev->fifo_count = 0;
                dev->fifo_head  = 0;
            }
            hp_hil_transmit(val);
            break;

        case 0x006e:
            /* R2 is read-only on the MLC; the System ROM writes 0xFE here to
               reset the adapter, so drop the queued frames and the error
               flags instead of storing the opcode. */
            dev->fifo_head  = 0;
            dev->fifo_count = 0;
            dev->r2         = 0x00;
            hp_hil_trace("HIL: mode 0x%02X -> loop=%d\n", val, dev->loop);
            hp_hil_irq();
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
        case 0x006c: /* R0: data byte of the frame on top, and pop it */
            ret = (uint8_t) (hp_hil_fifo_pop() & 0xff);
            break;

        case 0x006d: /* R1: command bit + address of the frame on top */
            if (dev->fifo_count)
                ret = (uint8_t) ((hp_hil_fifo_peek() >> 8) & 0x0f);
            else
                ret = 0x00;
            break;

        case 0x006e: /* R2: parity, frame and overflow errors */
            ret     = dev->r2;
            dev->r2 = 0x00;
            /* Reading the error status acknowledges it: the next reset
               command may raise INT again. */
            if (dev->scan_reported) {
                dev->scan_reported = 0;
                dev->r3 &= (uint8_t) ~HIL_R3_LERR;
            }
            break;

        case 0x006f: /* R3: INT/NMI/LERR; reading it does not drop INT */
            ret = dev->r3;
            break;

        default:
            break;
    }

    hp_hil_trace("HIL: in(%03X) = %02X [fifo %d] pc=%04X\n", port, ret, dev->fifo_count, cpu_state.pc);

    return ret;
}

static void *
hp_hil_init(const device_t *info)
{
    hp_hil_t *dev = (hp_hil_t *) calloc(1, sizeof(hp_hil_t));

    hp_hil_dev       = dev;

    dev->loop   = 1; /* the adapter powers up in loop-back */
    dev->kbd_id = HIL_MOUSE_ID;

    io_sethandler(0x006c, 4, hp_hil_read, NULL, NULL, hp_hil_write, NULL, NULL, dev);

    return dev;
}

static void
hp_hil_close(void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;

    hp_hil_dev       = NULL;

    io_removehandler(0x006c, 4, hp_hil_read, NULL, NULL, hp_hil_write, NULL, NULL, dev);

    free(dev);
}

static void
hp_hil_reset(void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;

    memset(dev, 0x00, sizeof(*dev));
    dev->loop   = 1; /* the adapter starts in loop-back, like the MLC */
    dev->kbd_id = HIL_MOUSE_ID;
}

const device_t hp_hil_device = {
    .name          = "HP-HIL Host Adapter (U777)",
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
