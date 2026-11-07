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
#define ENABLE_SCSI_SM912_LOG 0

/* DRQ0 is wired to the adapter (m_pcx: DRQ0 SCSI, DRQ1 WD2791). */
#define SM912_DMA_CHANNEL 0
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <86box/86box.h>
#include "cpu.h"
#include <86box/io.h>
#include <86box/scsi.h>
#include <86box/dma.h>

extern int i8018x_pcb_dma_remaining(int channel);
extern uint32_t i8018x_pcb_dma_dst(int channel);
extern void i8018x_pcb_dma_eop(int channel);
extern int i8018x_pcb_dma_src_port(int channel);
#include <86box/plat.h>
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
        char buf[256];
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        pclog("%s", buf);
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
    int               pio_used;   /* the host moved bytes through the data port */
    int               req;        /* REQ line: a byte waits in the data register */

    scsi_sm912_irq_t  irq;
    void             *irq_priv;
} scsi_sm912_t;

static void
scsi_sm912_update_irq(scsi_sm912_t *dev)
{
    int level = 0;

    /* IO && CD && REQ: a status or message byte is waiting for the host. */
    if (((dev->phase == SM912_STATUS) || (dev->phase == SM912_MSGIN)) && dev->req)
        level = 1;

    if (level == dev->irq_level)
        return;

    dev->irq_level = level;

    if (dev->irq != NULL)
        dev->irq(dev->irq_priv, level);
}

static int sm912_dma_used;
static int sm912_in_pump;   /* the byte came from the DMA engine, not the host */

/* Data phases can be driven by the DMA engine instead of the CPU: on this
   board the adapter only raises REQ, and DRQ0 makes the 80186 move the byte
   between memory and the data register.  The driver polls the status port
   while it waits, so each poll hands over one byte. */
static void
scsi_sm912_dma_pump(scsi_sm912_t *dev)
{
    scsi_common_t *sc = (dev->sd != NULL) ? dev->sd->sc : NULL;

    if ((sc == NULL) || (sc->temp_buffer == NULL))
        return;
    /* Do not gate on dma_channel_readable(): the BIOS arms the channel in a
       way that reports "not readable" while still accepting deposits, and it
       then waits for C/D without ever touching the data port.  Try the write
       and back off on DMA_NODATA instead. */

    if (dev->pio_used)
        return;
    if (i8018x_pcb_dma_src_port(SM912_DMA_CHANNEL) != dev->base)
        return;   /* the channel is not pointed at our data register */

    if (dev->phase == SM912_DATAIN) {
        if (dev->data_pos >= (uint32_t) dev->sd->buffer_length)
            return;
        if (getenv("PCX_SCSILOG") != NULL) {
            static long pr_n;

            if (++pr_n <= 4)
                pclog("[SMPRE] pos=%u/%d remain=%d dst=%05X drq_armed=1\n", dev->data_pos,
                      (int) dev->sd->buffer_length,
                      i8018x_pcb_dma_remaining(SM912_DMA_CHANNEL),
                      (unsigned) i8018x_pcb_dma_dst(SM912_DMA_CHANNEL));
        }
        /* Drain the whole record while the channel accepts bytes.  The DMA
           engine on the 80186 moves memory at bus speed, so one guest poll is
           enough -- handing over a single byte per poll made a 512-byte READ
           outlast the driver's timeout (SINIX: "Controller already busy"). */
        while (dev->data_pos < (uint32_t) dev->sd->buffer_length) {
            int r;
            /* Mirror the hardware handshake: REQ raises DRQ0, the controller
               performs one byte, then DRQ drops.  Without this the 80186 DMA
               never sees the transfer end and the driver waits for TC. */
            dma_set_drq(SM912_DMA_CHANNEL, 1);
            sm912_in_pump = 1;
            r = dma_channel_write(SM912_DMA_CHANNEL, sc->temp_buffer[dev->data_pos]);
            sm912_in_pump = 0;
            dma_set_drq(SM912_DMA_CHANNEL, 0);
            if (getenv("PCX_SCSILOG") != NULL) {
                static long dp_n;

                if (++dp_n <= 10)
                    pclog("[SMDP] pos=%u r=%d\n", dev->data_pos, r);
            }
            if (r == DMA_NODATA)
                return;
            sm912_dma_used = 1;
            dev->data_pos++;
            if (r & DMA_OVER) {
                /* The channel counted out before the CDB length did: that is
                   the guest's idea of "transfer done", so stop here. */
                if (dev->sd != NULL)
                    scsi_device_command_phase1(dev->sd);
                dev->status_byte = (dev->sd != NULL) ? dev->sd->status : 0x02;
                dev->phase       = SM912_STATUS;
                scsi_sm912_log("[SM912] dma over at %u/%u\n", dev->data_pos,
                               (dev->sd != NULL) ? (unsigned) dev->sd->buffer_length : 0);
                return;
            }
        }
        if (dev->data_pos >= (uint32_t) dev->sd->buffer_length) {
            /* The record ended: assert EOP so the channel stops here even if
               the guest programmed a longer count.  Without it the 80186 keeps
               waiting for a terminal count that never comes and the driver
               aborts the command ("Controller already busy"). */
            i8018x_pcb_dma_eop(SM912_DMA_CHANNEL);
            scsi_device_command_phase1(dev->sd);
            dev->status_byte = dev->sd->status;
            dev->phase       = SM912_STATUS;
            dev->req         = 1;
            scsi_sm912_log("[SM912] dma data-in done %u\n", dev->data_pos);
        }
    } else if (dev->phase == SM912_DATAOUT) {
        int val;
        if (dev->data_pos >= (uint32_t) dev->sd->buffer_length)
            return;
        val = dma_channel_read(SM912_DMA_CHANNEL);
        if (val == DMA_NODATA)
            return;
        sc->temp_buffer[dev->data_pos++] = (uint8_t) val;
        if (dev->data_pos >= (uint32_t) dev->sd->buffer_length) {
            scsi_device_command_phase1(dev->sd);
            dev->status_byte = dev->sd->status;
            dev->phase       = SM912_STATUS;
        }
    }

    /* Entering STATUS/MSGIN has to raise the interrupt: the driver waits for
       it instead of polling, and a silent transition makes it reset the bus. */
    if (getenv("PCX_SCSILOG") != NULL) {
        static long me_n;

        if (++me_n <= 4)
            pclog("[SMEND] phase=%d pos=%u/%d dma_used=%d remain=%d\n", dev->phase,
                  dev->data_pos, (int) ((dev->sd != NULL) ? dev->sd->buffer_length : -1),
                  sm912_dma_used, i8018x_pcb_dma_remaining(SM912_DMA_CHANNEL));
    }
    scsi_sm912_update_irq(dev);
}

static uint8_t
scsi_sm912_status(scsi_sm912_t *dev)
{
    static int log_on = -1;

    /* The driver polls this register while it waits for the command phase; a
       DMA channel armed after the selection only gets its chance here. */
    if ((dev->phase == SM912_DATAIN) && (getenv("PCX_SCSILOG") != NULL)) {
        static long st_n;

        if ((++st_n <= 5) || ((st_n % 100000) == 0))
            pclog("[SMST] DATAIN status polls=%ld pos=%u/%d\n", st_n,
                  dev->data_pos, (int) ((dev->sd != NULL) ? dev->sd->buffer_length : -1));
    }

    scsi_sm912_dma_pump(dev);



    static int log_n;
    uint8_t ret = 0;

    switch (dev->phase) {
        case SM912_CMD:
        case SM912_DATAIN:
        case SM912_DATAOUT:
        case SM912_STATUS:
        case SM912_MSGIN:
            ret = SM912_ST_BSY | (dev->req ? SM912_ST_REQ : 0);
            if ((dev->phase == SM912_CMD) || (dev->phase == SM912_STATUS) ||
                (dev->phase == SM912_MSGIN))
                ret |= SM912_ST_CD;
            /* Bit 6 is NOT I/O on this adapter: setting it makes the BIOS
               INT 13h handler spin forever at FC00:1B32.  Leave it clear. */
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
    dev->pio_used     = (getenv("PCX_SCSI_PIO") != NULL) ? 1 : 0;

    if ((dev->sd != NULL) && scsi_device_present(dev->sd)) {
        sc = dev->sd->sc;

        dev->sd->buffer_length = -1;
        scsi_device_command_phase0(dev->sd, dev->cdb);

        dev->status_byte = dev->sd->status;

        /* PCX_SCSILOG=1: the command stream the guest actually sends. */
        if (getenv("PCX_SCSILOG") != NULL) {
            static long q_n;

            pclog("[SCSIEX] #%ld op=%02X cdb=%02X %02X %02X %02X %02X "
                  "[%02X %02X %02X %02X %02X %02X] "
                  "st=%02X phase=%d len=%d\n",
                  ++q_n, dev->cdb[0], dev->cdb[1], dev->cdb[2], dev->cdb[3],
                  dev->cdb[4], dev->cdb[5],
                  dev->cdb[6], dev->cdb[7], dev->cdb[8], dev->cdb[9],
                  dev->cdb[10], dev->cdb[11], dev->sd->status,
                  dev->sd->phase, (int) dev->sd->buffer_length);
        }

        scsi_sm912_log("[SM912] exec op %02X status %02X phase %d len %d buf %p\n",
                       dev->cdb[0], dev->sd->status, dev->sd->phase,
                       (int) dev->sd->buffer_length,
                       (sc != NULL) ? (void *) sc->temp_buffer : NULL);

        if ((dev->sd->buffer_length > 0) && (sc != NULL) && (sc->temp_buffer != NULL)) {
            if (dev->sd->phase == SCSI_PHASE_DATA_IN) {
                sc->buffer_pos = 0;
                dev->phase     = SM912_DATAIN;
                dev->req       = 1;
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

    /* Commands without a data phase finish here; data phases finish in the
       register handlers, which is when phase1() may run. */
    if (dev->sd != NULL)
        scsi_device_command_phase1(dev->sd);

    dev->status_byte = (dev->sd != NULL) ? dev->sd->status : 0x02;
    dev->phase       = SM912_STATUS;

    scsi_sm912_update_irq(dev);
}

static void scsi_sm912_command_byte(scsi_sm912_t *dev, uint8_t val);

/* Command phase.  On the real board the target raises REQ with C/D low, and
   that combination drives DRQ0: the DMA controller fetches the CDB out of
   memory and drops it into the data register, so the CPU never writes those
   bytes itself (MAME: write_scsi_req() -> drq0_w(1)).  The driver programs the
   channel before it selects; if it did not, the PIO path in the data-register
   handler still applies. */
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

    /* The DMA controller may already hold the whole CDB. */
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
            /* The host is reading the data port itself: this command is PIO,
               and letting the DMA pump advance the same position would steal
               bytes from the stream. */
            if (!sm912_in_pump)
                dev->pio_used = 1;
            if (getenv("PCX_SCSILOG") != NULL) {
                static long d_n;

                if ((++d_n <= 5) || ((d_n % 100000) == 0))
                    pclog("[SMDATA] DATAIN data reads=%ld pos=%u\n", d_n,
                          dev->data_pos);
            }
            /* When the driver armed DMA for the transfer, the pump in the
               status handler moves the bytes; a CPU read of the data port
               must not consume a second time. */
            if (sm912_dma_used)
                return 0x00;
            dev->req = 0;
            if ((sc != NULL) && (sc->temp_buffer != NULL) &&
                (dev->data_pos < (uint32_t) dev->sd->buffer_length))
                ret = sc->temp_buffer[dev->data_pos++];
            if (dev->data_pos < (uint32_t) dev->sd->buffer_length)
                dev->req = 1;
            if ((sc == NULL) || (dev->data_pos >= (uint32_t) dev->sd->buffer_length)) {
                if (dev->sd != NULL)
                    scsi_device_command_phase1(dev->sd);
                dev->status_byte = (dev->sd != NULL) ? dev->sd->status : 0x02;
                dev->phase       = SM912_STATUS;
                dev->req         = 1;
                scsi_sm912_log("[SM912] data-in %u/%u byte(s)\n", dev->data_pos,
                               (dev->sd != NULL) ? (unsigned) dev->sd->buffer_length : 0);
            }
            break;
        case SM912_STATUS:
            /* The adapter owns the message phase: the target raises REQ with
               C/D and MSG, and the board takes the message byte itself and
               drops BSY (MAME: write_scsi_req() reads the data register and
               raises ACK without the CPU).  Waiting for the host to read the
               message leaves BSY up and the driver resets the bus. */
            ret      = dev->status_byte;
            dev->req = 0;
            {
                static int autoack = -1;
                if (autoack < 0)
                    autoack = (getenv("PCX_NOAUTOACK") != NULL) ? 0 : 1;
                if (autoack) {
                    dev->phase = SM912_IDLE;
                } else {
                    dev->message_byte = 0x00;   /* COMMAND COMPLETE */
                    dev->phase    = SM912_MSGIN;
                    dev->req      = 1;
                }
                /* The line follows the phase: leaving it high after the status
                   byte made the driver treat the command as still in progress
                   and reset the bus. */
                scsi_sm912_update_irq(dev);
            }
            scsi_sm912_log("[SM912] status %02X, message %s\n", ret,
                           (dev->phase == SM912_IDLE) ? "auto-acked" : "left for host");
            if (getenv("PCX_SCSILOG") != NULL) {
                static long sr_n;

                if (++sr_n <= 6)
                    pclog("[SMST2] status read #%ld byte=%02X new_phase=%d irq=%d\n",
                          sr_n, ret, dev->phase, dev->irq_level);
            }
            break;
        case SM912_MSGIN:
            ret        = dev->message_byte;
            dev->req   = 0;
            dev->phase = SM912_IDLE;
            scsi_sm912_update_irq(dev);
            if (getenv("PCX_SCSILOG") != NULL)
                pclog("[SMMSG] message read %02X, phase -> IDLE\n", ret);
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
            scsi_sm912_log("[SM912] pio cdb %02X from %04X:%04X\n", val,
                           cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
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


    {
        uint8_t v;
        int     is_status = (off & 1);

        v = is_status ? scsi_sm912_status(dev) : scsi_sm912_data_read(dev);

        if (getenv("PCX_SM912V") != NULL) {
            static long rn;

            if (rn++ < 600)
                pclog("[SMR] %s=%02X phase=%d pos=%u @%04X:%04X\n",
                      is_status ? "STS" : "DAT", v, dev->phase, dev->data_pos,
                      cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
        }

        return v;
    }
}

static void
scsi_sm912_write(uint16_t port, uint8_t val, void *priv)
{
    scsi_sm912_t *dev = (scsi_sm912_t *) priv;
    uint16_t      off = (uint16_t) (port - dev->base);

    if (getenv("PCX_SM912V") != NULL) {
        static long wn;

        if (wn++ < 600)
            pclog("[SMW] %s=%02X phase=%d pos=%u @%04X:%04X\n",
                  (off & 1) ? "CTL" : "DAT", val, dev->phase, dev->data_pos,
                  cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc);
    }


    if (!(off & 1)) {
        scsi_sm912_data_write(dev, val);
        return;
    }

    if (val & SM912_CT_RST) {
        scsi_sm912_log("[SM912] bus reset from %04X:%04X phase %d\n",
                       cpu_state.seg_cs.seg, (uint16_t) cpu_state.pc,
                       dev->phase);
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

    /* Debug logging is a run-time decision: the build always carries the
       messages, PCX_SCSILOG decides whether they are printed. */
    scsi_sm912_do_log = (getenv("PCX_SCSILOG") != NULL) ? 1 : 0;

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
