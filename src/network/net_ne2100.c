/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          Novell NE2100: a 16-bit ISA bus-master Ethernet card built
 *          around the AMD Am7990 LANCE.  The card itself is only the
 *          slot side of the story - the chip is net_lance.c, and this
 *          module supplies the jumpers (I/O base, IRQ), the station
 *          address and the optional boot ROM socket.
 *
 *          I/O decode (all 16-bit):
 *             base + 0x0   RDP
 *             base + 0x2   RAP
 *
 *          Not modelled: the on-board EEPROM/link-integrity logic and
 *          the AUI/BNC media selection, neither of which the driver can
 *          observe on card.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/random.h>
#include <86box/rom.h>
#include <86box/net_lance.h>

#define NE2100_ROM_SIZE 0x4000

typedef struct ne2100_t {
    const device_t *lance;
    lance_board_t   board;

    uint32_t        bios_addr;
    rom_t           bios_rom;
} ne2100_t;

static void *
ne2100_init(const device_t *info)
{
    ne2100_t *dev;
    int       mac;
    char     *rom;

    (void) info;

    dev = (ne2100_t *) calloc(1, sizeof(ne2100_t));
    if (dev == NULL)
        return NULL;

    dev->board.base = device_get_config_hex16("base");
    dev->board.irq  = device_get_config_int("irq");
    dev->bios_addr  = device_get_config_hex20("bios_addr");

    /* Same Novell OUI as the other cards in the tree; the last three
       bytes identify this particular card. */
    mac = device_get_config_mac("mac", -1);
    dev->board.mac[0] = 0x00;
    dev->board.mac[1] = 0x00;
    dev->board.mac[2] = 0xd8;
    if (mac & 0xff000000) {
        dev->board.mac[3] = random_generate();
        dev->board.mac[4] = random_generate();
        dev->board.mac[5] = random_generate();
        mac               = (((int) dev->board.mac[3]) << 16);
        mac |= (((int) dev->board.mac[4]) << 8);
        mac |= ((int) dev->board.mac[5]);
        device_set_config_mac("mac", mac);
    } else {
        dev->board.mac[3] = (mac >> 16) & 0xff;
        dev->board.mac[4] = (mac >> 8) & 0xff;
        dev->board.mac[5] = (mac & 0xff);
    }

    dev->lance = device_add_linked(&lance_device, &dev->board);
    if (dev->lance == NULL) {
        free(dev);
        return NULL;
    }

    rom = device_get_config_bios("bios");
    if ((dev->bios_addr != 0) && (rom != NULL)) {
        rom_init(&dev->bios_rom, rom, dev->bios_addr, NE2100_ROM_SIZE,
                 NE2100_ROM_SIZE - 1, 0, MEM_MAPPING_EXTERNAL);
    }

    return dev;
}

static void
ne2100_close(void *priv)
{
    ne2100_t *dev = (ne2100_t *) priv;

    if (dev == NULL)
        return;

    if (dev->bios_addr != 0)
        mem_mapping_disable(&dev->bios_rom.mapping);

    device_close(&lance_device);

    free(dev);
}

static const device_config_t ne2100_config[] = {
    {
        .name           = "base",
        .description    = "Address",
        .type           = CONFIG_HEX16,
        .default_string = NULL,
        .default_int    = 0x300,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "0x300", .value = 0x300 },
            { .description = "0x320", .value = 0x320 },
            { .description = "0x340", .value = 0x340 },
            { .description = "0x360", .value = 0x360 },
            { .description = ""                      }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = NULL,
        .default_int    = 10,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "IRQ 3",  .value =  3 },
            { .description = "IRQ 5",  .value =  5 },
            { .description = "IRQ 9",  .value =  9 },
            { .description = "IRQ 10", .value = 10 },
            { .description = "IRQ 11", .value = 11 },
            { .description = "IRQ 15", .value = 15 },
            { .description = ""                     }
        },
        .bios           = { { 0 } }
    },
    {
        .name           = "bios_addr",
        .description    = "Boot ROM address",
        .type           = CONFIG_HEX20,
        .default_string = NULL,
        .default_int    = 0,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = 0x00000 },
            { .description = "C0000",    .value = 0xc0000 },
            { .description = "C4000",    .value = 0xc4000 },
            { .description = "C8000",    .value = 0xc8000 },
            { .description = "CC000",    .value = 0xcc000 },
            { .description = "D0000",    .value = 0xd0000 },
            { .description = ""                         }
        },
        .bios           = {
            {
                .bios_type  = BIOS_NORMAL,
                .files_no   = 1,
                .local      = 0,
                .size       = NE2100_ROM_SIZE,
                .files      = { "roms/machines/ne2100/ne2100_boot_rom.bin", "" }
            },
            { .files_no = 0 }
        }
    },
    {
        .name           = "mac",
        .description    = "MAC Address",
        .type           = CONFIG_MAC,
        .default_string = NULL,
        .default_int    = -1,
        .file_filter    = NULL,
        .spinner        = { 0 },
        .selection      = { { 0 } },
        .bios           = { { 0 } }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

const device_t ne2100_device = {
    .name          = "Novell NE2100",
    .internal_name = "ne2100",
    .flags         = DEVICE_ISA16,
    .local         = 0,
    .init          = ne2100_init,
    .close         = ne2100_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = ne2100_config
};
