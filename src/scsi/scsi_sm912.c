/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-X on-board SCSI adapter (SM912).
 *
 *          The adapter itself is a thin shell around the SCSI bus: two
 *          registers and three bus lines.  Port +0 carries a data byte, port
 *          +1 carries the handshake state:
 *
 *              bit 7  CD    command/data: 1 = command, status or message
 *              bit 5  REQ   the target wants a byte
 *              bit 4  BSY   a target is on the bus
 *
 *          and a write to +1 with bit 2 set pulses the bus reset; any other
 *          write to +1 while BSY is clear drives SEL and starts a selection.
 *          A selection is acknowledged by the target raising BSY and asking
 *          for the command descriptor block, after which the byte traffic runs
 *          through the data register one ACK at a time.  The interrupt line
 *          (IRQ 5 on the PC-X) follows IO && CD && REQ, so it asks for service
 *          whenever a status or message byte is waiting (270788-001 section 9
 *          covers the same layout on the 80186 side).
 *
 *          Wiring, from the v3 POST: it writes the control register to select,
 *          waits for BSY (0xfdb16), then pushes a six byte CDB through the data
 *          port and reads status and message.  Nothing here is a full SCSI
 *          implementation; it is the protocol level model the POST and any
 *          later driver need in order to see a bridge board that answers.
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

#include <86box/86box.h>
#include <86box/io.h>
#include <86box/scsi_sm912.h>
#include <86box/timer.h>

#ifdef ENABLE_SCSI_SM912_LOG
int scsi_sm912_do_log = ENABLE_SCSI_SM912_LOG;

static void
scsi_sm912_log(const char *fmt, ...)
{
    va_list ap;

    if (scsi_sm912_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#  define scsi_sm912_log(fmt, ...)
#endif

/* Status register bits (read). */
#define SM912_ST_CD  0x80
#define SM912_ST_REQ 0x20
#define SM912_ST_BSY 0x10

/* Control register bits (write). */
#define SM912_CT_RST 0x04

/* SCSI phases the adapter can be in. */
enum {
    SM912_IDLE = 0,
    SM912_CMD,      /* command out: the host feeds the CDB */
    SM912_STATUS,   /* status in:   the host reads one status byte */
    SM912_MSGIN     /* message in:  the host reads one message byte */
};

typedef struct scsi_sm912_t {
    uint16_t          base;

    uint8_t           phase;
    uint8_t           cdb[6];
    int               cdb_len;
    int               cdb_pos;

    uint8_t           status_byte;
    uint8_t           message_byte;

    int               target_present;
    int               irq_level;

    scsi_sm912_irq_t  irq;
    void             *irq_priv;
} scsi_sm912_t;

static void
scsi_sm912_update_irq(scsi_sm912_t *dev)
{
    int level = 0;

    /* IO && CD && REQ: a status or message byte is waiting for the host. */
    if ((dev->phase == SM912_STATUS) || (dev->phase == SM912_MSGIN))
        level = 1;

    if (level == dev->irq_level)
        return;

    dev->irq_level = level;

    if (dev->irq != NULL)
        dev->irq(dev->irq_priv, level);
}

static uint8_t
scsi_sm912_status(scsi_sm912_t *dev)
{
    uint8_t ret = 0;

    switch (dev->phase) {
        case SM912_CMD:
            ret = SM912_ST_BSY | SM912_ST_REQ | SM912_ST_CD;
            break;
        case SM912_STATUS:
            ret = SM912_ST_BSY | SM912_ST_REQ | SM912_ST_CD;
            break;
        case SM912_MSGIN:
            ret = SM912_ST_BSY | SM912_ST_REQ | SM912_ST_CD;
            break;
        default:
            break;
    }

    return ret;
}

static void
scsi_sm912_select(scsi_sm912_t *dev)
{
    if (!dev->target_present) {
        /* Nobody answers, so SEL stays asserted and BSY never comes; the host
           times out and reports an empty bus. */
        scsi_sm912_log("[SM912] select, no target\n");
        return;
    }

    scsi_sm912_log("[SM912] select -> target answers\n");

    dev->cdb_len = 6;
    dev->cdb_pos = 0;
    dev->phase   = SM912_CMD;

    scsi_sm912_update_irq(dev);
}

/* One command byte arrives: the target takes it and, once the whole CDB is in,
   moves to the status phase.  With no drive behind the bridge every command
   ends in CHECK CONDITION and a zero message. */
static void
scsi_sm912_command_byte(scsi_sm912_t *dev, uint8_t val)
{
    if (dev->cdb_pos < dev->cdb_len)
        dev->cdb[dev->cdb_pos] = val;

    dev->cdb_pos++;

    if (dev->cdb_pos < dev->cdb_len)
        return;

    scsi_sm912_log("[SM912] cdb %02X %02X %02X %02X %02X %02X\n",
                   dev->cdb[0], dev->cdb[1], dev->cdb[2], dev->cdb[3],
                   dev->cdb[4], dev->cdb[5]);

    /* No unit attached: CHECK CONDITION (0x02) followed by a good message. */
    dev->status_byte  = 0x02;
    dev->message_byte = 0x00;
    dev->phase        = SM912_STATUS;

    scsi_sm912_update_irq(dev);
}

static uint8_t
scsi_sm912_data_read(scsi_sm912_t *dev)
{
    uint8_t ret = 0x00;

    switch (dev->phase) {
        case SM912_STATUS:
            ret         = dev->status_byte;
            dev->phase  = SM912_MSGIN;
            break;
        case SM912_MSGIN:
            ret         = dev->message_byte;
            dev->phase  = SM912_IDLE;
            break;
        default:
            break;
    }

    scsi_sm912_log("[SM912] data read %02X, phase %d\n", ret, dev->phase);

    scsi_sm912_update_irq(dev);

    return ret;
}

static void
scsi_sm912_data_write(scsi_sm912_t *dev, uint8_t val)
{
    if (dev->phase == SM912_CMD)
        scsi_sm912_command_byte(dev, val);
}

static uint8_t
scsi_sm912_read(uint16_t port, void *priv)
{
    scsi_sm912_t *dev = (scsi_sm912_t *) priv;
    uint16_t      off = (uint16_t) (port - dev->base);


    if (off & 1)
        return scsi_sm912_status(dev);

    return scsi_sm912_data_read(dev);
}

static void
scsi_sm912_write(uint16_t port, uint8_t val, void *priv)
{
    scsi_sm912_t *dev = (scsi_sm912_t *) priv;
    uint16_t      off = (uint16_t) (port - dev->base);


    if (!(off & 1)) {
        scsi_sm912_data_write(dev, val);
        return;
    }

    if (val & SM912_CT_RST) {
        scsi_sm912_log("[SM912] bus reset\n");
        scsi_sm912_reset(dev);
        return;
    }

    if ((dev->phase != SM912_IDLE) || !(scsi_sm912_status(dev) & SM912_ST_BSY))
        scsi_sm912_select(dev);
}

void
scsi_sm912_reset(scsi_sm912_t *dev)
{
    dev->phase   = SM912_IDLE;
    dev->cdb_pos = 0;
    dev->cdb_len = 6;

    scsi_sm912_update_irq(dev);
}

void
scsi_sm912_set_target(scsi_sm912_t *dev, int present)
{
    dev->target_present = present;
}

void
scsi_sm912_close(scsi_sm912_t *dev)
{
    if (dev == NULL)
        return;

    io_removehandler(dev->base, 4, scsi_sm912_read, NULL, NULL,
                     scsi_sm912_write, NULL, NULL, dev);
    free(dev);
}

scsi_sm912_t *
scsi_sm912_init(uint16_t base, scsi_sm912_irq_t irq, void *irq_priv)
{
    scsi_sm912_t *dev = (scsi_sm912_t *) calloc(1, sizeof(scsi_sm912_t));

    dev->base           = base;
    dev->irq            = irq;
    dev->irq_priv       = irq_priv;
    dev->cdb_len        = 6;
    dev->target_present = 1;

    scsi_sm912_reset(dev);

    io_sethandler(base, 4, scsi_sm912_read, NULL, NULL,
                  scsi_sm912_write, NULL, NULL, dev);

    scsi_sm912_log("[SM912] adapter at %04X\n", base);

    return dev;
}
