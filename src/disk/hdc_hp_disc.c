/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          HP Vectra hard disc adapter (the 45816A/45817A generation of
 *          HP "intelligent" controllers).
 *
 *          The model here is taken from the OS/2 1.1 driver HP shipped
 *          for this adapter (DISKHP1.SYS), disassembled and documented in
 *          documents/86box/os2drv/REPORT.md:
 *
 *            base + 0   read  status   (0x80 busy, 0x40 done, 0x0C ok)
 *            base + 0   write command  (0x02, 0x03, 0x04, ...)
 *            base + 2   16-bit data port (command packet / sector data)
 *
 *          The driver finds the adapter by scanning the option ROM area
 *          (0xC8000-0xDFFFF, 8 KB steps) for a "WINSTON" signature at
 *          window+0x1848 and then reads the port base, the drive count and
 *          the drive parameter table out of the ROM.  This device supplies
 *          that ROM (see roms/machines/hpvectra/hp_disc_option_rom.bin and
 *          tools/mk_hp_disc_rom.py), so a driver-side port of the model can
 *          be validated against the real discovery path.
 *
 *          Stage 1: discovery, register handshake and packet capture. The
 *          command packet layout is reported, not interpreted, until the
 *          driver tells us what each word means.
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
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/timer.h>
#include <86box/pic.h>
#include <86box/hdc.h>
#include <86box/plat.h>

#define HP_DISC_ROM_PATH   "roms/machines/hpvectra/hp_disc_option_rom.bin"
#define HP_DISC_ROM_ADDR   0x000c8000
#define HP_DISC_ROM_SIZE   0x4000

/* Offsets inside the option ROM that the OS/2 driver reads. */
#define HP_DISC_ROM_SIG    0x1848  /* "WINSTON"          */
#define HP_DISC_ROM_PARAMS 0x3022  /* port base, ...     */

#define HP_DISC_PACKET_WORDS 10
#define HP_DISC_PACKET_BYTES (HP_DISC_PACKET_WORDS * 2)

typedef struct hp_disc_t {
    uint16_t base;
    int      irq;

    uint8_t status;
    uint8_t cmd;

    uint16_t packet[HP_DISC_PACKET_WORDS];
    uint8_t  packet_pos;
    uint8_t  data_lo;
    uint8_t  data_hi;

    int      tracing;
    uint64_t packets;

    pc_timer_t timer;
    rom_t      rom;
} hp_disc_t;

static void
hp_disc_complete(void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    /* busy clear, done set, "ok" pattern in bits 3:2 */
    dev->status = 0x40 | 0x0c;

    if (dev->tracing)
        pclog("hp_disc: command %02x completed, status %02x\n", dev->cmd, dev->status);

    picint(1 << dev->irq);
}

static uint8_t
hp_disc_in(uint16_t addr, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    switch (addr - dev->base) {
        case 0:
            return dev->status;
        case 2:
            return dev->data_lo;
        case 3:
            return dev->data_hi;
        default:
            break;
    }

    return 0xff;
}

static uint16_t
hp_disc_inw(uint16_t addr, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    if ((addr - dev->base) == 2)
        return (uint16_t) (dev->data_lo | (dev->data_hi << 8));

    return 0xffff;
}

static void
hp_disc_out(uint16_t addr, uint8_t val, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    switch (addr - dev->base) {
        case 0:
            dev->cmd    = val;
            dev->status = 0x80; /* busy */
            dev->packet_pos = 0;

            if (dev->tracing)
                pclog("hp_disc: command %02x\n", val);

            timer_set_delay_u64(&dev->timer, 20 * TIMER_USEC);
            break;

        case 2:
            if (dev->packet_pos < HP_DISC_PACKET_WORDS)
                dev->packet[dev->packet_pos] = (uint16_t) (dev->packet[dev->packet_pos] & 0xff00) | val;
            break;

        case 3:
            if (dev->packet_pos < HP_DISC_PACKET_WORDS) {
                dev->packet[dev->packet_pos] =
                    (uint16_t) (dev->packet[dev->packet_pos] & 0x00ff) | ((uint16_t) val << 8);
                dev->packet_pos++;
            }

            if (dev->packet_pos == HP_DISC_PACKET_WORDS) {
                dev->packets++;
                if (dev->tracing) {
                    pclog("hp_disc: packet %llu after cmd %02x:",
                          (unsigned long long) dev->packets, dev->cmd);
                    for (uint8_t i = 0; i < HP_DISC_PACKET_WORDS; i++)
                        pclog(" %04x", dev->packet[i]);
                    pclog("\n");
                }
                dev->packet_pos = 0;
            }
            break;

        default:
            break;
    }
}

static void
hp_disc_outw(uint16_t addr, uint16_t val, void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    if ((addr - dev->base) != 2)
        return;

    if (dev->packet_pos < HP_DISC_PACKET_WORDS) {
        dev->packet[dev->packet_pos++] = val;

        if (dev->packet_pos == HP_DISC_PACKET_WORDS) {
            dev->packets++;
            if (dev->tracing) {
                pclog("hp_disc: packet %llu after cmd %02x:",
                      (unsigned long long) dev->packets, dev->cmd);
                for (uint8_t i = 0; i < HP_DISC_PACKET_WORDS; i++)
                    pclog(" %04x", dev->packet[i]);
                pclog("\n");
            }
            dev->packet_pos = 0;
        }
    }
}

static void *
hp_disc_init(const device_t *info)
{
    hp_disc_t *dev = (hp_disc_t *) calloc(1, sizeof(hp_disc_t));

    dev->base    = (uint16_t) info->local;
    dev->irq     = 5;
    dev->status  = 0x00;
    dev->tracing = (getenv("HP_DISC_TRACE") != NULL);

    timer_add(&dev->timer, hp_disc_complete, dev, 0);

    io_sethandler(dev->base, 4,
                  hp_disc_in, hp_disc_inw, NULL,
                  hp_disc_out, hp_disc_outw, NULL,
                  dev);

    /* The option ROM carries the discovery structure the OS/2 driver looks
       for; it is mapped read-only where an ISA adapter would sit. */
    if (rom_init(&dev->rom, HP_DISC_ROM_PATH, HP_DISC_ROM_ADDR, HP_DISC_ROM_SIZE,
                 HP_DISC_ROM_SIZE - 1, 0, MEM_MAPPING_EXTERNAL) != 0)
        pclog("hp_disc: option ROM %s missing, the adapter will not be found\n",
              HP_DISC_ROM_PATH);

    pclog("hp_disc: HP hard disc adapter at %04Xh, IRQ %i\n", dev->base, dev->irq);

    return dev;
}

static void
hp_disc_close(void *priv)
{
    hp_disc_t *dev = (hp_disc_t *) priv;

    timer_disable(&dev->timer);
    io_removehandler(dev->base, 4,
                     hp_disc_in, hp_disc_inw, NULL,
                     hp_disc_out, hp_disc_outw, NULL,
                     dev);

    free(dev);
}

const device_t hp_disc_device = {
    .name          = "HP Hard Disc Adapter (45816A)",
    .internal_name = "hp_disc",
    .flags         = DEVICE_ISA,
    .local         = 0x0300,
    .init          = hp_disc_init,
    .close         = hp_disc_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
