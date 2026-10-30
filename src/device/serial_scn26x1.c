/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Signetics SCN2661/SCN2651 EPCI -- Enhanced Programmable
 *          Communications Interface (USART), plus the identical SCN68661
 *          CMOS part.  Register model from the 1986 Signetics data book:
 *
 *            port+0  read  receive holding register  write transmit holding
 *            port+1  read  status register          write SYN1/SYN2/DLE
 *            port+2  read  mode register 1 or 2     write mode register 1/2
 *            port+3  read  command register         write command register
 *
 *          The SYN and mode ports are sequenced: the first access hits the
 *          first register, the next the second, and a third access wraps.
 *          Reading the command register rewinds both sequencers.
 *
 *          What is not modelled: synchronous (BISYNC/SDLC) framing, the
 *          external clock pins (an external clock source is assumed to run
 *          at the BRG rate), the BREAK output, and the transmitter-empty
 *          interrupt source -- INTR is only raised for received data so a
 *          level line cannot sit asserted forever.
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
#include <86box/io.h>
#include <86box/timer.h>
#include <86box/log.h>
#include <86box/serial_scn26x1.h>

#ifdef ENABLE_SCN26X1_LOG
int scn26x1_do_log = ENABLE_SCN26X1_LOG;

static void
scn26x1_log(void *priv, const char *fmt, ...)
{
    if (scn26x1_do_log) {
        va_list ap;
        va_start(ap, fmt);
        log_out(priv, fmt, ap);
        va_end(ap);
    }
}
#else
#    define scn26x1_log(fmt, ...)
#endif

/* Register select, A1:A0 on top of the base port. */
#define SCN_RDR        0x00 /* read:  receive holding register */
#define SCN_TDR        0x00 /* write: transmit holding register */
#define SCN_SR         0x01 /* read:  status register */
#define SCN_SYN        0x01 /* write: SYN1, SYN2, DLE (sequenced) */
#define SCN_MR         0x02 /* both:  mode register 1, then 2 */
#define SCN_CR         0x03 /* both:  command register */

/* Status register, table 8. */
#define SCN_SR_TXRDY   0x01 /* transmit holding register empty */
#define SCN_SR_RXRDY   0x02 /* received character waiting */
#define SCN_SR_TXEMT   0x04 /* transmit shift register empty */
#define SCN_SR_PE      0x08 /* parity / DLE detect */
#define SCN_SR_OE      0x10 /* overrun */
#define SCN_SR_FE      0x20 /* framing / SYN detect */
#define SCN_SR_DCD     0x40 /* 1 = DCD input low */
#define SCN_SR_DSR     0x80 /* 1 = DSR input low */

/* Command register, table 7. */
#define SCN_CR_TXEN    0x01
#define SCN_CR_DTR     0x02 /* 1 = DTR output low */
#define SCN_CR_RXEN    0x04
#define SCN_CR_BREAK   0x08 /* async: force break, sync: send DLE */
#define SCN_CR_ERRS    0x10 /* reset PE/OE/FE */
#define SCN_CR_RTS     0x20 /* 1 = RTS output low */
#define SCN_CR_MODE(x) ((uint8_t) (((x) >> 6) & 0x03))

#define SCN_MODE_NORMAL   0
#define SCN_MODE_ECHO     1
#define SCN_MODE_LOCAL    2 /* local loopback */
#define SCN_MODE_REMOTE   3 /* remote loopback */

/* Mode register 1, table 5. */
#define SCN_MR1_LEN(x)  ((uint8_t) (((x) >> 2) & 0x03))
#define SCN_MR1_PAREN  0x10
#define SCN_MR1_CLOCK(x) ((uint8_t) ((x) & 0x03))
#define SCN_MR1_STOP(x) ((uint8_t) (((x) >> 6) & 0x03))

struct scn26x1_t {
    uint16_t   base;
    uint8_t    mr1, mr2, cr, sr;
    uint8_t    tdr;      /* transmit holding register */
    uint8_t    tsr;      /* transmit shift register */
    uint8_t    rdr;      /* receive holding register */
    uint8_t    syn[3];   /* SYN1, SYN2, DLE */
    int        mr_ptr;   /* 0 = next access is MR1, 1 = MR2 */
    int        syn_ptr;  /* position in the SYN1/SYN2/DLE sequence */
    int        thr_full; /* transmit holding register holds a character */
    int        tx_busy;  /* shift register is serializing */
    int        tx_emt;   /* shift register empty */
    int        rx_full;  /* a character waits in the holding register */
    int        rx_line;  /* RxD pin level, idle high */
    int        dsr;      /* modem inputs, idle high */
    int        dcd;
    int        irq_state;
    void     (*irq_handler)(void *priv, int level);
    void      *irq_priv;
    pc_timer_t timer;
    void      *log;
};

/* Table 1, SCN2661B (BRCLK 4.9152 MHz).  The PC-X carries the B version. */
static const double scn26x1_rates[16] = {
    45.5, 50.0, 75.0, 110.0, 134.5, 150.0, 300.0, 600.0,
    1200.0, 1800.0, 2000.0, 2400.0, 4800.0, 9600.0, 19200.0, 38400.0
};

/* Bit time of the programmed rate, in microseconds.  Mode register 2 bits 7-4
   pick internal or external clocks; the external pins are not modelled, so the
   bit rate generator value is used either way. */
static double
scn26x1_bit_time(scn26x1_t *dev)
{
    double baud = scn26x1_rates[dev->mr2 & 0x0f];

    if (baud <= 0.0)
        baud = 9600.0;

    return 1000000.0 / baud;
}

/* Start bit + data bits + optional parity + stop bits. */
static int
scn26x1_frame_bits(scn26x1_t *dev)
{
    int data  = SCN_MR1_LEN(dev->mr1) + 5;
    int bits  = 1 + data;
    int stop  = SCN_MR1_STOP(dev->mr1);

    if (dev->mr1 & SCN_MR1_PAREN)
        bits++;

    if (SCN_MR1_CLOCK(dev->mr1) == 0) /* synchronous: one character, no framing */
        return data;

    /* 00 is documented as invalid; treat it as a single stop bit. */
    if (stop == 2)
        bits += 2;
    else
        bits += 1;

    return bits;
}

static void
scn26x1_update_irq(scn26x1_t *dev)
{
    int level;

    /* Only received data drives the line; see the file header. */
    level = dev->rx_full ? 1 : 0;

    if (level == dev->irq_state)
        return;

    dev->irq_state = level;

    if (dev->irq_handler != NULL)
        dev->irq_handler(dev->irq_priv, level);
}

static void
scn26x1_receive(scn26x1_t *dev, uint8_t val)
{
    if (dev->rx_full) {
        dev->sr |= SCN_SR_OE;
        return;
    }

    dev->rdr     = val;
    dev->rx_full = 1;
    dev->sr     &= ~(uint8_t) (SCN_SR_PE | SCN_SR_FE);

    scn26x1_log(dev->log, "Rx %02X\n", val);
    scn26x1_update_irq(dev);
}

static void scn26x1_tx_start(scn26x1_t *dev);

/* Called when the shift register has clocked out a character. */
static void
scn26x1_tx_done(void *priv)
{
    scn26x1_t *dev = (scn26x1_t *) priv;
    uint8_t    mode = SCN_CR_MODE(dev->cr);

    dev->tx_busy = 0;
    dev->tx_emt  = 1;

    /* In every loopback/echo submode the transmitted character comes back on
       the receive side; a normal line would need a remote station. */
    if ((mode == SCN_MODE_LOCAL) || (mode == SCN_MODE_REMOTE))
        scn26x1_receive(dev, dev->tsr);

    /* A character may have been written while the shift register was busy. */
    scn26x1_tx_start(dev);
}

/* Move a waiting character into the shift register if it is free. */
static void
scn26x1_tx_start(scn26x1_t *dev)
{
    if (!dev->thr_full || dev->tx_busy || !(dev->cr & SCN_CR_TXEN))
        return;

    dev->tsr      = dev->tdr;
    dev->thr_full = 0;
    dev->tx_busy  = 1;
    dev->tx_emt   = 0;

    scn26x1_log(dev->log, "Tx %02X\n", dev->tsr);
    timer_on_auto(&dev->timer, (double) scn26x1_frame_bits(dev) * scn26x1_bit_time(dev));
}

static uint8_t
scn26x1_read(uint16_t port, void *priv)
{
    scn26x1_t *dev = (scn26x1_t *) priv;
    uint8_t    ret = 0xff;

    switch (port - dev->base) {
        case SCN_RDR:
            ret          = dev->rdr;
            dev->rx_full = 0;
            scn26x1_update_irq(dev);
            break;

        case SCN_SR:
            ret = dev->sr;

            if (dev->thr_full || !(dev->cr & SCN_CR_TXEN))
                ret &= ~(uint8_t) SCN_SR_TXRDY;
            else
                ret |= SCN_SR_TXRDY;

            if (dev->rx_full)
                ret |= SCN_SR_RXRDY;
            else
                ret &= ~(uint8_t) SCN_SR_RXRDY;

            if (dev->tx_emt)
                ret |= SCN_SR_TXEMT;
            else
                ret &= ~(uint8_t) SCN_SR_TXEMT;

            if (!dev->dcd)
                ret |= SCN_SR_DCD;
            else
                ret &= ~(uint8_t) SCN_SR_DCD;

            if (!dev->dsr)
                ret |= SCN_SR_DSR;
            else
                ret &= ~(uint8_t) SCN_SR_DSR;

            /* Reading the status register clears the change flag. */
            dev->sr &= ~(uint8_t) (SCN_SR_DCD | SCN_SR_DSR);
            break;

        case SCN_MR:
            ret = dev->mr_ptr ? dev->mr2 : dev->mr1;
            dev->mr_ptr ^= 1;
            break;

        case SCN_CR:
            ret = dev->cr;
            /* A command register read rewinds both sequencers. */
            dev->mr_ptr  = 0;
            dev->syn_ptr = 0;
            break;

        default:
            break;
    }

    scn26x1_log(dev->log, "[%04X] read  %02X -> %02X\n", port, port - dev->base, ret);
    return ret;
}

static void
scn26x1_write(uint16_t port, uint8_t val, void *priv)
{
    scn26x1_t *dev = (scn26x1_t *) priv;

    scn26x1_log(dev->log, "[%04X] write %02X <- %02X\n", port, port - dev->base, val);

    switch (port - dev->base) {
        case SCN_TDR:
            dev->tdr      = val;
            dev->thr_full = 1;
            dev->tx_emt   = 0;
            scn26x1_tx_start(dev);
            break;

        case SCN_SYN:
            dev->syn[dev->syn_ptr] = val;
            dev->syn_ptr           = (dev->syn_ptr + 1) % 3;
            break;

        case SCN_MR:
            if (dev->mr_ptr)
                dev->mr2 = val;
            else
                dev->mr1 = val;
            dev->mr_ptr ^= 1;
            break;

        case SCN_CR: {
            uint8_t old = dev->cr;

            dev->cr = val;

            if (val & SCN_CR_ERRS) {
                dev->sr &= ~(uint8_t) (SCN_SR_PE | SCN_SR_OE | SCN_SR_FE);
                dev->cr &= ~(uint8_t) SCN_CR_ERRS; /* one-shot */
            }

            if (!(old & SCN_CR_TXEN) && (val & SCN_CR_TXEN))
                scn26x1_tx_start(dev);
            break;
        }

        default:
            break;
    }
}

void
scn26x1_set_rx(scn26x1_t *dev, int state)
{
    dev->rx_line = state ? 1 : 0;
}

void
scn26x1_set_dsr(scn26x1_t *dev, int state)
{
    state = state ? 1 : 0;

    if (state != dev->dsr) {
        dev->dsr = state;
        dev->sr |= SCN_SR_DSR;
    }
}

void
scn26x1_set_dcd(scn26x1_t *dev, int state)
{
    state = state ? 1 : 0;

    if (state != dev->dcd) {
        dev->dcd = state;
        dev->sr |= SCN_SR_DCD;
    }
}

void
scn26x1_reset(scn26x1_t *dev)
{
    timer_disable(&dev->timer);

    dev->mr1 = dev->mr2 = dev->cr = dev->sr = 0;
    dev->tdr = dev->tsr = dev->rdr = 0;
    memset(dev->syn, 0x00, sizeof(dev->syn));
    dev->mr_ptr   = 0;
    dev->syn_ptr  = 0;
    dev->thr_full = 0;
    dev->tx_busy  = 0;
    dev->tx_emt   = 1;
    dev->rx_full  = 0;
    dev->rx_line  = 1;
    dev->dsr      = 1;
    dev->dcd      = 1;

    scn26x1_update_irq(dev);
}

scn26x1_t *
scn26x1_init(uint16_t base, void (*irq)(void *priv, int level), void *irq_priv)
{
    scn26x1_t *dev = (scn26x1_t *) calloc(1, sizeof(scn26x1_t));

    dev->base       = base;
    dev->irq_handler = irq;
    dev->irq_priv   = irq_priv;
    dev->log        = log_open("SCN26X1");

    timer_add(&dev->timer, scn26x1_tx_done, dev, 0);
    scn26x1_reset(dev);

    io_sethandler(base, 0x0004,
                  scn26x1_read, NULL, NULL, scn26x1_write, NULL, NULL, dev);

    return dev;
}

void
scn26x1_close(scn26x1_t *dev)
{
    if (dev == NULL)
        return;

    io_removehandler(dev->base, 0x0004,
                     scn26x1_read, NULL, NULL, scn26x1_write, NULL, NULL, dev);
    timer_disable(&dev->timer);

    if (dev->log != NULL) {
        log_close(dev->log);
        dev->log = NULL;
    }

    free(dev);
}

static void *
scn26x1_device_init(const device_t *info)
{
    uint16_t base = (uint16_t) device_get_config_hex16("base");

    return scn26x1_init(base, NULL, NULL);
}

static void
scn26x1_device_close(void *priv)
{
    scn26x1_close((scn26x1_t *) priv);
}

static void
scn26x1_device_reset(void *priv)
{
    scn26x1_reset((scn26x1_t *) priv);
}

static const device_config_t scn26x1_config[] = {
  // clang-format off
    {
        .name           = "base",
        .description    = "Base port",
        .type           = CONFIG_HEX16,
        .default_string = "",
        .default_int    = 0xf9c0,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
  // clang-format on
};

const device_t scn26x1_device = {
    .name          = "Signetics SCN2661 EPCI",
    .internal_name = "scn26x1",
    .flags         = 0,
    .local         = 0,
    .init          = scn26x1_device_init,
    .close         = scn26x1_device_close,
    .reset         = scn26x1_device_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = scn26x1_config
};
