/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          Header of the emulation of the IBM LAN Adapter/A for Ethernet
 *          board logic on top of the DP8393x SONIC controller.
 */

#ifndef NET_IBMLANA_H
#define NET_IBMLANA_H

#include <stdint.h>
#include <86box/device.h>
#include <86box/mem.h>
#include <86box/net_dp8393x.h>

#define SONIC_BOOTROM_SIZE 0x4000 /* 16 KB boot ROM */
#define SONIC_IO_BYTE_SIZE 0x20   /* +80h .. +9Fh: board and ID registers */

typedef struct ibm_lana_t {
    dp8393x_t *sonic;
    int        sonic_inst;

    const char *name;
    int         variant;
    uint8_t     mac[6];

    uint8_t       pos_regs[8];
    uint8_t       board_control;
    uint8_t       reset_armed;
    uint8_t       adapter_enabled;
    uint8_t       link_up;
    uint8_t       io_set;
    uint8_t       bootrom_loaded;
    uint8_t       bootrom[SONIC_BOOTROM_SIZE];
    uint8_t       id_rom[16];
    uint16_t      io_base;
    uint8_t       irq;
    uint32_t      mem_base;
    uint32_t      mem_size;
    mem_mapping_t window;
} ibm_lana_t;

extern const device_t ibm_lan_a_device;
extern const device_t ibm_lan_a_short_device;

#endif /*NET_IBMLANA_H*/
