/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          SGI Visual Workstation 320/540 (Cobalt chipset).
 *
 * Authors: Dmitry Borisov, <di.sean@protonmail.com>
 *
 *          Copyright 2024 Dmitry Borisov
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/machine.h>
#include <86box/rom.h>
#include <86box/device.h>
#include <86box/usb.h>
#include <86box/pci.h>
#include <86box/chipset.h>
#include <86box/sio.h>
#include <86box/network.h>
#include <86box/flash.h>
#include "cpu.h"

#define VW_PIIX_GPI_DEFAULT              0x0000607E
#define VW_PIIX_GPI_CPU_BUS_100MHZ       0x00010000
#define VW_PIIX_GPI_MODEL_320            0x00000000
#define VW_PIIX_GPI_MODEL_540            0x00040000

#define VW_SIO_GPIO_DEFAULT              0xFF000080
#define VW_SIO_GPIO_320_BOARD_REV_006F   0x11
#define VW_SIO_GPIO_540_BOARD_REV_001A   0x21

static void
machine_sgivw_gpio_init(int is_320)
{
    uint32_t piix_gpio = VW_PIIX_GPI_DEFAULT;

    piix_gpio |= is_320 ? VW_PIIX_GPI_MODEL_320 : VW_PIIX_GPI_MODEL_540;
    if (cpu_busspeed > 66666667)
        piix_gpio |= VW_PIIX_GPI_CPU_BUS_100MHZ;

    machine_set_gpio_acpi_default(piix_gpio);
    machine_set_gpio_default(VW_SIO_GPIO_DEFAULT |
                             (is_320 ? VW_SIO_GPIO_320_BOARD_REV_006F
                                     : VW_SIO_GPIO_540_BOARD_REV_001A));
}

static int
machine_at_sgivw_common_init(const machine_t *model, int is_320)
{
    int ret;

    ret = bios_load_linear("roms/machines/sgivw/prom1004.bin",
                           0x00080000, 512 * 1024, 0x200);
    if (bios_only || !ret)
        return ret;

    machine_sgivw_gpio_init(is_320);

    machine_at_common_init_ex(model, 2);

    pci_init(PCI_CONFIG_TYPE_1);

    /* Lithium B bus #0 */
    pci_register_bus_slot(0, 0x04, PCI_CARD_SOUTHBRIDGE, 0, 0, 0, 0); /* PIIX4E */

    /* Lithium A bus #1 */
    pci_register_bus_slot(1, 0x03, PCI_CARD_NETWORK, 3, 6, 5, 6); /* On-board 82557 */

    device_add(&piix4e_device);
    device_add(&pc87307_device);
    device_add(&cobalt_chipset_device);
    device_add(&st_flash_29f400t_device);
    {
        static const uint8_t sgivw_mac[6] = { 0x08, 0x00, 0x69, 0x12, 0x34, 0x56 };
        /* The on-board variant, not i82557_device with the flag ORed in:
           that one carries 0xff to say "take the model from the BIOS
           configuration", and 0xff | 0x0100 leaves the variant index at
           0xff, which reads back 8086:0000 instead of 8086:1229. */
        void *nic = device_add(&i82557b_onboard_device);

        i8255x_set_bia(nic, sgivw_mac);
    }

    return ret;
}

/* The slots' interrupt lines on the Cobalt APIC, as Linux's
   visws_map_irq() has them (arch/x86/pci/visws.c, v3.13): each slot's
   first line takes an input of its own, and the three others rotate over
   a set the bus shares -- five and six on Lithium A, thirteen to fifteen
   on Lithium B.  The kernel's Lithium B expression runs past those three
   inputs from the second slot on, so the rotation its comment describes
   is what is used here. */
static void
machine_at_sgivw_pci_apic_routes(int busb_slots)
{
    for (int slot = 0; slot < busb_slots; slot++) {
        pci_apic_set_intin_route(0, slot, 0, 8 + slot);

        for (int pin = 1; pin < 4; pin++)
            pci_apic_set_intin_route(0, slot, pin,
                                     13 + (((3 - slot) + (pin - 1)) % 3));
    }

    for (int slot = 0; slot < 2; slot++) {
        pci_apic_set_intin_route(1, slot, 0, slot);

        for (int pin = 1; pin < 4; pin++)
            pci_apic_set_intin_route(1, slot, pin,
                                     5 + ((slot + (pin - 1)) % 2));
    }

    /* The on-board 82557 sits at device 3, past the two slots, and takes
       the next input after them (CO_APIC_PCIA_BASE0 + 3). */
    pci_apic_set_intin_route(1, 3, 0, 3);
}

int
machine_at_sgivw320_init(const machine_t *model)
{
    int ret = machine_at_sgivw_common_init(model, 1);

    if (bios_only || !ret)
        return ret;

    /* Lithium B bus #0: one half-length 32-bit slot. */
    pci_register_bus_slot(0, 0x00, PCI_CARD_NORMAL, 1, 2, 3, 4);
    machine_at_sgivw_pci_apic_routes(1);

    /* Lithium A bus #1: two full-length 64-bit slots. */
    pci_register_bus_slot(1, 0x00, PCI_CARD_NORMAL, 4, 1, 2, 3);
    pci_register_bus_slot(1, 0x01, PCI_CARD_NORMAL, 3, 4, 1, 2);

    return ret;
}

int
machine_at_sgivw540_init(const machine_t *model)
{
    int ret = machine_at_sgivw_common_init(model, 0);

    if (bios_only || !ret)
        return ret;

    /* Lithium B bus #0: four 64-bit slots. */
    pci_register_bus_slot(0, 0x00, PCI_CARD_NORMAL, 1, 2, 3, 4);
    pci_register_bus_slot(0, 0x01, PCI_CARD_NORMAL, 4, 1, 2, 3);
    pci_register_bus_slot(0, 0x02, PCI_CARD_NORMAL, 3, 4, 1, 2);
    pci_register_bus_slot(0, 0x03, PCI_CARD_NORMAL, 2, 3, 4, 1);
    machine_at_sgivw_pci_apic_routes(4);

    /* Lithium A bus #1: two 64-bit slots.  The on-board Ultra2 SCSI the
       guide lists beside them (slot 6) has no controller here. */
    pci_register_bus_slot(1, 0x00, PCI_CARD_NORMAL, 4, 1, 2, 3);
    pci_register_bus_slot(1, 0x01, PCI_CARD_NORMAL, 3, 4, 1, 2);

    return ret;
}
