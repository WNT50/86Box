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
 *          write to +1 while BSY is clear drives SEL and starts a selection,
 *          with the target id in the low three bits.  A selection is
 *          acknowledged by the target raising BSY and asking for the command
 *          descriptor block, after which the byte traffic runs through the
 *          data register one byte at a time.  The interrupt line (IRQ 5 on the
 *          PC-X) follows IO && CD && REQ, so it asks for service whenever a
 *          status or message byte is waiting (270788-001 section 9 covers the
 *          same layout on the 80186 side).
 *
 *          The transfer itself is not bit-banged: once the whole CDB has
 *          arrived the command is handed to the device layer, which fills or
 *          drains its transfer buffer, and this file only meters that buffer
 *          out through the data register.  That is the same split the other
 *          PIO-only adapters in this tree use.
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
#include "cpu.h"
#include <86box/io.h>
#include <86box/scsi.h>
#include <86box/scsi_device.h>
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
    SM912_DATAIN,   /* data in:     the host reads transfer bytes */
    SM912_DATAOUT,  /* data out:    the host writes transfer bytes */
    SM912_STATUS,   /* status in:   the host reads one status byte */
    SM912_MSGIN     /* message in:  the host reads one message byte */
};

typedef struct scsi_sm912_t {
    uint16_t          base;

    uint8_t           phase;
    uint8_t           cdb[16];
    int               cdb_len;
    int               cdb_pos;

    uint8_t           status_byte;
    uint8_t           message_byte;

    uint8_t           bus;
    uint8_t           target_id;
    scsi_device_t    *sd;
    uint32_t          data_pos;

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
    static int log_on = -1;
    static int log_n;
    uint8_t ret = 0;

    switch (dev->phase) {
        case SM912_CMD:
        case SM912_DATAIN:
        case SM912_DATAOUT:
        case SM912_STATUS:
        case SM912_MSGIN:
            ret = SM912_ST_BSY | SM912_ST_REQ;
            if ((dev->phase == SM912_CMD) || (dev->phase == SM912_STATUS) ||
                (dev->phase == SM912_MSGIN))
                ret |= SM912_ST_CD;
            break;
        default:
            break;
    }

    if (log_on < 0)
        log_on = (getenv("PCX_SCSILOG") != NULL);
    if (log_on && (log_n++ < 400))
        pclog("[SM912] status phase=%i -> %02X from %04X:%04X\n", dev->phase, ret,
              CS, cpu_state.pc);

    return ret;
}

/* Group code in the top three bits of the opcode gives the CDB length. */
static int
scsi_sm912_cdb_length(uint8_t op)
{
    switch (op >> 5) {
        case 0:
            return 6;
        case 1:
        case 2:
            return 10;
        case 5:
            return 12;
        case 4:
            return 16;
        default:
            return 6;
    }
}

/* Hand the finished CDB to the device layer and see what it wants to do. */
static void
scsi_sm912_execute(scsi_sm912_t *dev)
{
    scsi_common_t *sc = NULL;

    dev->status_byte  = 0x02;   /* CHECK CONDITION when nothing takes it */
    dev->message_byte = 0x00;
    dev->data_pos     = 0;

    if ((dev->sd != NULL) && scsi_device_present(dev->sd)) {
        sc = dev->sd->sc;

        scsi_device_command_phase0(dev->sd, dev->cdb);
        scsi_device_command_phase1(dev->sd);

        dev->status_byte = dev->sd->status;

        if ((dev->sd->buffer_length > 0) && (sc != NULL) && (sc->temp_buffer != NULL)) {
            if (dev->sd->phase == SCSI_PHASE_DATA_IN) {
                sc->buffer_pos = 0;
                dev->phase     = SM912_DATAIN;
                scsi_sm912_update_irq(dev);
                return;
            }

            if (dev->sd->phase == SCSI_PHASE_DATA_OUT) {
                sc->buffer_pos = 0;
                dev->phase     = SM912_DATAOUT;
                scsi_sm912_update_irq(dev);
                return;
            }
        }
    }

    dev->phase = SM912_STATUS;

    scsi_sm912_update_irq(dev);
}

static void
scsi_sm912_select(scsi_sm912_t *dev, uint8_t val)
{
    if (!dev->target_present) {
        /* Nobody answers, so SEL stays asserted and BSY never comes; the host
           times out and reports an empty bus. */
        scsi_sm912_log("[SM912] select, no bridge\n");
        return;
    }

    dev->target_id = (uint8_t) (val & 0x07);
    dev->sd        = &scsi_devices[dev->bus][dev->target_id];

    {
        static int sl = -1;
        if (sl < 0)
            sl = (getenv("PCX_SCSILOG") != NULL);
        if (sl)
            pclog("[SM912] select id %i bus %i present %i val %02X\n",
                  dev->target_id, dev->bus, scsi_device_present(dev->sd), val);
    }

    scsi_sm912_log("[SM912] select id %i present %i\n", dev->target_id,
                   scsi_device_present(dev->sd));

    if (!scsi_device_present(dev->sd)) {
        /* Nothing at that id: the selection is not acknowledged. */
        dev->sd = NULL;
        return;
    }

    dev->cdb_len = 6;
    dev->cdb_pos = 0;
    dev->phase   = SM912_CMD;

    scsi_sm912_update_irq(dev);
}

/* One command byte arrives; once the whole CDB is in the command runs. */
static void
scsi_sm912_command_byte(scsi_sm912_t *dev, uint8_t val)
{
    if (dev->cdb_pos < (int) sizeof(dev->cdb))
        dev->cdb[dev->cdb_pos] = val;

    dev->cdb_pos++;

    if (dev->cdb_pos == 1)
        dev->cdb_len = scsi_sm912_cdb_length(dev->cdb[0]);

    if (dev->cdb_pos < dev->cdb_len)
        return;

    scsi_sm912_log("[SM912] cdb %02X %02X %02X %02X %02X %02X\n",
                   dev->cdb[0], dev->cdb[1], dev->cdb[2], dev->cdb[3],
                   dev->cdb[4], dev->cdb[5]);

    scsi_sm912_execute(dev);
}

static uint8_t
scsi_sm912_data_read(scsi_sm912_t *dev)
{
    uint8_t        ret = 0xff;
    scsi_common_t *sc  = (dev->sd != NULL) ? dev->sd->sc : NULL;

    switch (dev->phase) {
        case SM912_DATAIN:
            if ((sc != NULL) && (sc->temp_buffer != NULL) &&
                (dev->data_pos < (uint32_t) dev->sd->buffer_length))
                ret = sc->temp_buffer[dev->data_pos++];
            if ((sc == NULL) || (dev->data_pos >= (uint32_t) dev->sd->buffer_length)) {
                dev->status_byte = (dev->sd != NULL) ? dev->sd->status : 0x02;
                dev->phase       = SM912_STATUS;
            }
            break;
        case SM912_STATUS:
            ret        = dev->status_byte;
            dev->phase = SM912_MSGIN;
            break;
        case SM912_MSGIN:
            ret        = dev->message_byte;
            dev->phase = SM912_IDLE;
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
    scsi_common_t *sc = (dev->sd != NULL) ? dev->sd->sc : NULL;

    switch (dev->phase) {
        case SM912_CMD:
            scsi_sm912_command_byte(dev, val);
            break;
        case SM912_DATAOUT:
            if ((sc != NULL) && (sc->temp_buffer != NULL) &&
                (dev->data_pos < (uint32_t) dev->sd->buffer_length))
                sc->temp_buffer[dev->data_pos++] = val;
            if ((sc != NULL) && (dev->data_pos >= (uint32_t) dev->sd->buffer_length)) {
                /* Everything the command asked for is in: let the device
                   commit it (a disk writes the sectors here). */
                scsi_device_command_phase1(dev->sd);
                dev->status_byte = (dev->sd != NULL) ? dev->sd->status : 0x02;
                dev->phase       = SM912_STATUS;
            }
            break;
        default:
            break;
    }

    scsi_sm912_update_irq(dev);
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
        scsi_sm912_select(dev, val);
}

void
scsi_sm912_reset(scsi_sm912_t *dev)
{
    dev->phase   = SM912_IDLE;
    dev->cdb_pos = 0;
    dev->cdb_len = 6;
    dev->sd      = NULL;
    dev->data_pos = 0;

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
    dev->bus            = scsi_get_bus();

    scsi_sm912_reset(dev);

    io_sethandler(base, 4, scsi_sm912_read, NULL, NULL,
                  scsi_sm912_write, NULL, NULL, dev);

    scsi_sm912_log("[SM912] adapter at %04X on bus %i\n", base, dev->bus);

    return dev;
}

/* Descriptor only: the adapter is brought up by the machine's own init, but
   the SCSI bus planner has to be told how many buses it takes, or it reports
   the machine's settings as having none. */
static uint32_t
sm912_scsi_buses(const device_t *dev)
{
    return 1;
}

const device_t scsi_sm912_device = {
    .name          = "Siemens SM912",
    .internal_name = "sm912",
    .flags         = 0,
    .local         = 0,
    .init          = NULL,
    .close         = NULL,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL,
    .scsi_buses    = sm912_scsi_buses
};
