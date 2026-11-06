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
#include <stddef.h>
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
#include <86box/mouse.h>
#include <86box/pic.h>
#include <86box/mem.h>
#include <86box/timer.h>

#include "hp_hil.h"

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
#define HIL_RPL 0x20 /* RePoll, 20..2f */
#define HIL_DHR 0xfe /* Device Hard Reset */

/* MLC register bits (Appendix I). */
#define HIL_R2_PERR 0x01
#define HIL_R2_FERR 0x02
#define HIL_R2_FOF  0x04
#define HIL_R3_INT  0x01
#define HIL_R3_NMI  0x02
#define HIL_R3_LERR 0x04

#define HIL_W2_IPF 0x04 /* suppress the answer when the FIFO is empty */

/* AP is the adapter's 60 Hz strobe (Vectra TRM Vol 1 section 4); with APE set
   each rising edge makes the MLC issue a POL of its own. */
#define HIL_AP_HZ     60
#define HIL_AP_PERIOD (1000000 / HIL_AP_HZ)

#define HIL_FIFO_SIZE 16

/* HP-HIL IRQ 20 is the third 8259's line 4 (vector 6Ch). */
#define HIL_IRQ_LINE 4

static hp_hil_t *hp_hil_dev;

/* Ignore Poll Frame: the MLC still queues the empty poll, it only keeps the
   interrupt to itself (TRM 45918A 4-19). */
static int hp_hil_no_irq;

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

void
hp_hil_trace(const char *fmt, ...)
{
    va_list ap;

    if (getenv("EPOCH_HIL_TRACE") == NULL)
        return;

    va_start(ap, fmt);
    pclog_ex(fmt, ap);
    va_end(ap);
}

/* A frame the System has to read: data byte plus its command/address byte. */
void
hp_hil_fifo_push(uint16_t frame)
{
    hp_hil_t *dev = hp_hil_dev;

    hp_hil_trace("HIL: push %03X\n", frame);
    if (dev->fifo_count >= HIL_FIFO_SIZE) {
        dev->r2 |= HIL_R2_FOF;
        dev->r3 |= HIL_R3_INT;
        pic3_int(HIL_IRQ_LINE, 1);
        return;
    }

    dev->fifo[(dev->fifo_head + dev->fifo_count) % HIL_FIFO_SIZE] = frame;
    dev->fifo_count++;

    /* The MLC interrupts on the command frame, not on the record in front of
       it: MAME's hp_hil_mlc_device::hil_write() only touches R3 for frames
       with the command bit set, and the data frames queue silently. */
    if ((frame & 0x0800) && !hp_hil_no_irq) {
        dev->r3 |= HIL_R3_INT;
        /* "The AUTO POLL ENABLE (APE) bit is cleared by the MLC whenever it
           interrupts the processor" (TRM 45918A 1-16). */
        dev->w3 &= (uint8_t) ~HIL_W3_APE;
        pic3_int(HIL_IRQ_LINE, 1);
    }
}

/* The HIL driver keeps its context in RAM and finds it through the int 6Fh
   record table: IVT 6Fh -> record 0x114 -> its data segment.  The 8041 owns
   the bring-up, so this is where the "link initialised" bit of that context
   is set; without it the driver parks in state 2 and POST waits forever. */
uint32_t
hp_hil_driver_ctx(void)
{
    uint16_t ivt = (uint16_t) (mem_readb_phys(0x1be) | (mem_readb_phys(0x1bf) << 8));
    uint16_t dseg = mem_readw_phys((uint32_t) ivt * 16 + 0x114 + 4);

    if (dseg == 0x0000)
        return 0;

    return (uint32_t) dseg * 16;
}

/* The System asks the 8041 to poll the link (port 68h, 7Dh); the 8041 walks
   it and hands the report records back through the adapter. */
/* The link is interrupt driven: a device with something to report raises the
   line and the System reads the report out of the FIFO.  The System does not
   poll us for motion, so sample the host pointer here and raise the line when
   the mouse has moved. */
static void
hp_hil_pointer_tick(void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;

    /* The host pointer belongs to the device that reports it; the adapter only
       owns the 10 ms tick (AP is a 60 Hz line from the 8041, Vectra TRM Vol 1
       section 4). */
    if (dev->dev[HIL_PTR].ops->tick != NULL)
        dev->dev[HIL_PTR].ops->tick(dev, HIL_PTR);

    timer_advance_u64(&dev->pointer_timer, (10000ULL * TIMER_USEC));
}

/* AP: the adapter's 60 Hz strobe.  With APE set the MLC stops waiting for the
   System and issues a POL itself.  The record it gets back ends with the poll
   command, and that command frame is what raises the interrupt - which in
   turn clears APE (TRM 45918A 1-16 and 4-19).  Nothing is queued when no
   device has anything to report, so a quiet link stays quiet. */
static void
hp_hil_ap_tick(void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;
    int       i, any = 0;
    uint8_t   addr = 0;

    if ((dev->w3 & HIL_W3_APE) && !dev->loop) {
        for (i = 0; i < HIL_NDEV; i++) {
            if (!dev->dev[i].addr)
                continue;
            addr = dev->dev[i].addr;
            if (dev->dev[i].ops->poll(dev, i))
                any = 1;
        }

        if (any) {
            /* IPF keeps the interrupt to the adapter: the terminator is still
               queued, the System just is not woken for it. */
            if (dev->w2 & HIL_W2_IPF)
                hp_hil_no_irq = 1;

            /* fifo_push() clears APE when it raises the interrupt. */
            hp_hil_fifo_push((uint16_t) (0x0800 | ((uint16_t) addr << 8) | HIL_POL));
            hp_hil_no_irq = 0;
        }
    }

    timer_advance_u64(&dev->ap_timer, (HIL_AP_PERIOD * TIMER_USEC));
}

void
hp_hil_poll_link(void)
{
    hp_hil_t *dev = hp_hil_dev;

    if ((dev == NULL) || !dev->dev[HIL_PTR].pt)
        return;

    /* The record, and whether there is one at all, is the device's business. */
    dev->dev[HIL_PTR].ops->poll(dev, HIL_PTR);
}

int
hp_hil_link_up(void)
{
    return (hp_hil_dev != NULL) && hp_hil_dev->dev[HIL_PTR].pt;
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
    if ((ctx != 0) && (dev->dev[HIL_PTR].id != 0x00)) {
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
    hp_hil_trace("HIL: pop %03X at %04X:%04X\n", frame,
                 cpu_state.seg_cs.seg, cpu_state.pc);

    return frame;
}

/* The frame a device answers with: its own address, no command bit. */
uint16_t
hp_hil_reply(uint8_t addr, uint8_t data)
{
    return (uint16_t) (((uint16_t) (addr & 7) << 8) | data);
}

/*
 * Walk a command frame down the link.
 *
 * The chain order matters: a frame only reaches device i when every device
 * before it is in Pass-Thru Mode, and a device in Loop-Back mirrors whatever
 * it gets, which is what the System's link self test looks for.  This is the
 * whole of the configuration loop in TRM 45918A 4.3:
 *
 *   ACF 09  -> keyboard takes 1, mirrors it back as 0Ah (device 2 not reached)
 *   IDD @1  -> keyboard describes itself
 *   EPT @1  -> keyboard goes pass-thru, device 2 mirrors the frame back, so the
 *              System knows the chain continues
 *   IFC     -> device 2 clears Power-Up and returns the frame
 *   ACF 09  -> keyboard keeps 1 and passes 0Ah on, device 2 takes 2 and answers
 *              0Bh, which is how the System learns there are two devices
 *   IDD @2  -> pointer describes itself
 *   EPT @2  -> last device, the frame runs off the end and nothing comes back
 */
static void
hp_hil_link_frame(uint16_t frame)
{
    hp_hil_t *dev = hp_hil_dev;
    uint8_t   addr;
    uint8_t   data;
    int       i;
    int       command;

    data    = (uint8_t) (frame & 0xff);
    addr    = (uint8_t) ((frame >> 8) & 0x07);
    command = (frame >> 11) & 1;

    if (!command) {
        hp_hil_fifo_push(frame);
        return;
    }

    /* Anything outside the command set is a link test (the System sends A5h
       and 5Ah frames, and 80h, and expects to see them again).  Those are
       answered no matter which mode or address they carry. */
    if ((data != HIL_DHR) && (data > 0x46)) {
        hp_hil_fifo_push(frame);
        return;
    }

    hp_hil_trace("HIL: device <= %03X (addr %d, op %02X)\n", frame, addr, data);

    /* Link-wide, and never handed back: every device acts, and the last
       device (which goes pass-thru) sends the frame off the end, so nothing
       comes back.  A hard reset destroys the link configuration: the devices
       end up in Loop-Back with their match address cleared and Power-Up Mode
       set, and the System configures the link again (TRM 45918A 4.5). */
    if (data == HIL_DHR) {
        /* The manual says DHR destroys the link configuration (TRM 45918A
           4.5), but this ROM's POST issues a DHR and then goes straight on to
           IDD/poll against the addresses it already handed out: clearing them
           here leaves POST waiting for a device that no longer answers and the
           machine never boots.  MAME keeps the address across HPHIL_DHR as
           well, so the address survives here too. */
        for (i = 0; i < HIL_NDEV; i++)
            dev->dev[i].pt = 0;
        dev->loop      = 1;
        dev->ready_ctx = 0;
        return;
    }

    /* Device Soft Reset does clear the addresses, and it is not returned
       either: the last device sends it off the end (TRM 45918A 4.4). */
    if (data == HIL_DSR) {
        for (i = 0; i < HIL_NDEV; i++) {
            dev->dev[i].pt   = 0;
            dev->dev[i].addr = 0;
            dev->dev[i].pwr  = 1;
        }
        dev->loop = 1;
        return;
    }

    for (i = 0; i < HIL_NDEV; i++) {
        hil_dev_t *d = &dev->dev[i];

        /* A device in pass-thru hands on the frame it received, which is not
           necessarily the one the System wrote (ACF is incremented on the
           way down), so re-read the fields for every device. */
        data = (uint8_t) (frame & 0xff);
        addr = (uint8_t) ((frame >> 8) & 0x07);

        if (addr != 0) {
            /* Addressed frame: it travels until it meets its device. */
            if (addr != d->addr) {
                if (d->pt)
                    continue;      /* retransmitted to the next device */
                hp_hil_fifo_push(frame); /* loop-back mirrors it back */
                return;
            }

            switch (data) {
                case HIL_EPT:
                    d->pt    = 1;
                    d->pwr   = 0;
                    dev->loop = 0;
                    hp_hil_trace("HIL: device %d pass-thru (EPT)\n", i);
                    continue;      /* retransmitted down the chain */

                case HIL_ELB:
                    d->pt     = 0;
                    dev->loop = 1;
                    break;

                case HIL_IFC:
                    d->pwr = 0;
                    break;

                case HIL_IDD:
                    d->ops->idd(dev, i);
                    break;

                default:
                    if ((data >= HIL_POL) && (data <= (HIL_POL + 0x0f))) {
                        if (!d->ops->poll(dev, i) && (dev->w2 & HIL_W2_IPF))
                            hp_hil_no_irq = 1; /* IPF + Auto Poll: queue silently */
                    }
                    break;
            }
            hp_hil_fifo_push(frame);
            hp_hil_no_irq = 0;
            hp_hil_link_ready(dev);
            return;
        }

        /* Universal address. */
        if ((data >= HIL_ACF) && (data <= (HIL_ACF + 0x07))) {
            d->addr = (uint8_t) (data & 0x07);
            d->pwr  = 0;
            /* The device takes the address, increments it and passes the
               frame on; the System reads the incremented opcode that comes
               back from the end of the link. */
            frame = (uint16_t) ((frame & 0xff00) |
                                ((((data & 0x07) + 1) & 0x07) | HIL_ACF));
            hp_hil_trace("HIL: %s configured as address %d\n", d->ops->name, d->addr);
            if (d->pt && (i < (HIL_NDEV - 1)))
                continue;
            hp_hil_fifo_push(frame);
            hp_hil_link_ready(dev);
            return;
        }

        /* Everything else with the universal address is for every device. */
        switch (data) {
            case HIL_IFC:
                d->pwr = 0;
                break;

            case HIL_ELB:
                d->pt     = 0;
                dev->loop = 1;
                break;

            default:
                if ((data >= HIL_POL) && (data <= (HIL_POL + 0x0f))) {
                    if (!d->ops->poll(dev, i) && (dev->w2 & HIL_W2_IPF))
                        hp_hil_no_irq = 1; /* IPF + Auto Poll: queue silently */
                }
                break;
        }
        if (d->pt && (i < (HIL_NDEV - 1)))
            continue;
        hp_hil_fifo_push(frame);
        hp_hil_no_irq = 0;
        hp_hil_link_ready(dev);
        return;
    }

    /* The frame ran off the end of the link: nothing comes back. */
}

/* The System wrote W0: the frame is complete, put it on the link. */
void
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
    hp_hil_link_frame(frame);
}

static void
hp_hil_link_ready(void *priv)
{
    hp_hil_t *dev = (hp_hil_t *) priv;
    uint32_t  ctx = hp_hil_driver_ctx();
    uint16_t  flags;

    if ((ctx == 0) || (ctx == dev->ready_ctx))
        return;

    flags = mem_readw_phys(ctx + 0x10);
    if (!(flags & 0x0001))
        return;                         /* driver is not waiting */

    dev->ready_ctx = ctx;
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

        case 0x006c: /* W0: frame data; the frame goes out at the end of the
                        write cycle (TRM 45918A 1-16).  The chapters on W0 and
                        W2 give no FIFO side effect -- R2 is cleared by reading
                        R2 -- so the receive FIFO is left alone here. */
            hp_hil_transmit(val);
            break;

        case 0x006e: /* W2: IPF / TEST / APE source flags (TRM 45918A 1-16).
                        Writing this register has no documented effect on the
                        receive FIFO. */
            dev->w2 = val;
            hp_hil_trace("HIL: W2 = %02X\n", val);
            break;

        case 0x006f: /* W3: APE (Auto Poll Enable) lives in bit 1 */
            dev->w3 = val;
            hp_hil_trace("HIL: W3 = %02X\n", val);
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
            if (dev->fifo_count) {
                ret = (uint8_t) (hp_hil_fifo_pop() & 0xff);
            } else {
                /* Nobody answered: the MLC reports a frame error (0xFD)
                   rather than handing back a zero, which is what the
                   driver's wait loop (F000:7B86) looks for. */
                ret      = 0xfd;
                dev->r2 |= HIL_R2_FERR;
            }
            break;

        case 0x006d: /* R1: command bit + address of the frame on top */
            if (dev->fifo_count)
                ret = (uint8_t) ((hp_hil_fifo_peek() >> 8) & 0x0f);
            else
                /* Bit 3 is set when no frame is waiting: the reader
                   (F000:7BCA) stops on it, and handing back 0x00 instead
                   makes it believe another frame is coming and poll forever. */
                ret = 0x08;
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

        case 0x006f: /* R3: INT/NMI/LERR.  Reading R3 acknowledges INT and
                        drops the line (MAME hp_hil.cpp, read case 3). */
            ret      = dev->r3;
            dev->r3 &= (uint8_t) ~HIL_R3_INT;
            pic3_int(HIL_IRQ_LINE, 0);
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
    dev->dev[HIL_KBD].id  = HIL_KBD_ID;
    dev->dev[HIL_KBD].ops = &hp_hil_kbd_ops;
    dev->dev[HIL_PTR].id  = HIL_MOUSE_ID;
    dev->dev[HIL_PTR].ops = &hp_hil_mouse_ops;

    io_sethandler(0x006c, 4, hp_hil_read, NULL, NULL, hp_hil_write, NULL, NULL, dev);

    timer_add(&dev->pointer_timer, hp_hil_pointer_tick, dev, 1);
    timer_add(&dev->ap_timer, hp_hil_ap_tick, dev, 1);

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

    /* Never memset() the whole struct: pointer_timer is a live pc_timer_t
       linked into the timer queue, and copying it around corrupts the queue
       (the sampler then silently stops firing after a reset). */
    dev->fifo_head    = 0;
    dev->fifo_count   = 0;
    dev->w1           = 0;
    dev->w2           = 0;
    dev->w3           = 0;
    dev->r2           = 0;
    dev->r3           = 0;
    dev->loop         = 1; /* the adapter starts in loop-back, like the MLC */
    dev->dev[HIL_KBD].addr = 0;
    dev->dev[HIL_KBD].id   = HIL_KBD_ID;
    dev->dev[HIL_KBD].pt   = 0;
    dev->dev[HIL_KBD].pwr  = 1;
    dev->dev[HIL_KBD].ops  = &hp_hil_kbd_ops;
    dev->dev[HIL_PTR].addr = 0;
    dev->dev[HIL_PTR].id   = HIL_MOUSE_ID;
    dev->dev[HIL_PTR].pt   = 0;
    dev->dev[HIL_PTR].pwr  = 1;
    dev->dev[HIL_PTR].ops  = &hp_hil_mouse_ops;
    dev->scan_reported = 0;
    dev->ready_ctx    = 0;
    dev->dx           = 0;
    dev->dy           = 0;
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
