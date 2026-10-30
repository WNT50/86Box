/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-D / PC-X keyboard and terminal controller, modelled at
 *          the protocol level instead of running the 8031 and 8035 firmware.
 *
 *          The board pair is a channel, not a register file: the host writes
 *          the character stream it wants displayed to 0xf980, an 8031 on the
 *          video board parses it, and anything the terminal has to say back
 *          -- key strokes as well as answers to identity queries -- arrives on
 *          the same port, with 0xf981 carrying the handshake bits:
 *
 *            0xf981 bit 0   a byte is waiting to be read at 0xf980
 *            0xf981 bit 1   the controller has not taken the last byte yet
 *
 *          The POST proves the channel works by sending "ESC [ 2 y" (v3 BIOS
 *          0xfd8ab, printed through 0xfc00:0x174d) and insisting on the exact
 *          answer "ESC P 2 y 0 ESC \" -- seven bytes, the fifth of which it
 *          compares against 0x30 (v3 0xfcd02..0xfdcd02 range, 0xfd8d3).  The
 *          answer used to come from the firmware; here the command stream is
 *          decoded and the reply synthesised, which keeps the observable
 *          behaviour identical without an MCS-51 core.
 *
 *          Anything else the terminal could be asked is deliberately left
 *          unanswered: a status query that times out is what the real board
 *          does when the firmware does not implement it either.
 *
 *
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
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
#include <86box/timer.h>
#include <86box/log.h>
#include <86box/kbc_siemens.h>

#ifdef ENABLE_KBC_SIEMENS_LOG
int kbc_siemens_do_log = ENABLE_KBC_SIEMENS_LOG;

static void
kbc_siemens_log(void *priv, const char *fmt, ...)
{
    if (kbc_siemens_do_log) {
        va_list ap;
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define kbc_siemens_log(fmt, ...)
#endif

#define KBC_FIFO_SIZE 64

/* Parser states for the host-to-terminal command stream. */
#define KBC_ST_IDLE   0 /* ordinary character, or the start of an escape */
#define KBC_ST_ESC    1 /* ESC received */
#define KBC_ST_CSI    2 /* ESC [ ... collecting parameters */
#define KBC_ST_IGNORE 3 /* some other escape sequence, drop until it ends */

struct kbc_siemens_t {
    uint8_t fifo[KBC_FIFO_SIZE];
    int     fifo_head;
    int     fifo_tail;

    int state;
    int csi_len;
    char csi[16];

    /* Queued key strokes are host events, not command stream: they share the
       same channel and therefore the same FIFO. */
    void *log;
};

static int
kbc_fifo_empty(kbc_siemens_t *dev)
{
    return dev->fifo_head == dev->fifo_tail;
}

static void
kbc_fifo_put(kbc_siemens_t *dev, uint8_t val)
{
    int next = (dev->fifo_tail + 1) % KBC_FIFO_SIZE;

    if (next == dev->fifo_head)
        return; /* the host is not reading; drop, as the real link would */

    dev->fifo[dev->fifo_tail] = val;
    dev->fifo_tail             = next;
}

/* The terminal identity exchange: "ESC [ 2 y" is answered with the device
   control string "ESC P 2 y 0 ESC \".  The trailing 0 is the status the POST
   checks digit by digit (it subtracts 0x30 and requires a zero result). */
static void
kbc_answer_identity(kbc_siemens_t *dev)
{
    static const uint8_t reply[7] = { 0x1b, 'P', '2', 'y', '0', 0x1b, '\\' };
    int                  i;

    for (i = 0; i < 7; i++)
        kbc_fifo_put(dev, reply[i]);

    kbc_siemens_log(dev->log, "identity query answered\n");
}

void
kbc_siemens_rx(kbc_siemens_t *dev, uint8_t byte)
{
    if (dev == NULL)
        return;

    switch (dev->state) {
        case KBC_ST_IDLE:
            if (byte == 0x1b)
                dev->state = KBC_ST_ESC;
            break;

        case KBC_ST_ESC:
            if (byte == '[') {
                dev->state   = KBC_ST_CSI;
                dev->csi_len = 0;
                dev->csi[0]  = '\0';
            } else if (byte == 0x1b) {
                dev->state = KBC_ST_ESC; /* repeated ESC, stay here */
            } else {
                dev->state = KBC_ST_IGNORE;
            }
            break;

        case KBC_ST_CSI:
            if ((byte >= 0x40) && (byte <= 0x7e)) {
                /* Final byte: "y" with parameter "2" is the identity query. */
                if ((byte == 'y') && (dev->csi_len == 1) && (dev->csi[0] == '2'))
                    kbc_answer_identity(dev);

                dev->state = KBC_ST_IDLE;
            } else if (dev->csi_len < (int) (sizeof(dev->csi) - 1)) {
                dev->csi[dev->csi_len++] = (char) byte;
                dev->csi[dev->csi_len]   = '\0';
            } else {
                dev->state = KBC_ST_IGNORE;
            }
            break;

        case KBC_ST_IGNORE:
        default:
            /* ESC P ... ESC \ style strings end on their own terminator. */
            if ((byte >= 0x40) && (byte <= 0x7e))
                dev->state = KBC_ST_IDLE;
            break;
    }
}

uint8_t
kbc_siemens_read_data(kbc_siemens_t *dev)
{
    uint8_t ret = 0x00;

    if (dev == NULL)
        return ret;

    if (!kbc_fifo_empty(dev)) {
        ret        = dev->fifo[dev->fifo_head];
        dev->fifo_head = (dev->fifo_head + 1) % KBC_FIFO_SIZE;
    }

    kbc_siemens_log(dev->log, "read %02X\n", ret);
    return ret;
}

uint8_t
kbc_siemens_read_status(kbc_siemens_t *dev)
{
    uint8_t ret = 0x00;

    if (dev == NULL)
        return ret;

    /* Bit 0: a byte waits at 0xf980 (v3 0xfd9e6 tests exactly this bit).
       Bit 1: busy, i.e. the controller has not taken the last byte.  The
       high level model consumes the stream at once, so it stays clear. */
    if (!kbc_fifo_empty(dev))
        ret |= 0x01;

    return ret;
}

/* Inject a key stroke.  The board scans a matrix and sends make/break codes
   through the same channel; the codes themselves are still unknown, so this
   entry point takes whatever the caller has and hands it to the FIFO. */
void
kbc_siemens_key(kbc_siemens_t *dev, uint8_t byte)
{
    if (dev == NULL)
        return;

    kbc_fifo_put(dev, byte);
    kbc_siemens_log(dev->log, "key %02X\n", byte);
}

kbc_siemens_t *
kbc_siemens_init(void)
{
    kbc_siemens_t *dev = (kbc_siemens_t *) calloc(1, sizeof(kbc_siemens_t));

    if (dev == NULL)
        return NULL;

    dev->log = log_open("KBCSIEMENS");

    return dev;
}

void
kbc_siemens_close(kbc_siemens_t *dev)
{
    if (dev == NULL)
        return;

    if (dev->log != NULL) {
        log_close(dev->log);
        dev->log = NULL;
    }

    free(dev);
}
