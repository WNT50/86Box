/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          Header of the emulation of the National Semiconductor DP8393x
 *          SONIC/SONIC-T Ethernet controller as used by the IBM LAN
 *          Adapter/A for Ethernet.
 */

#ifndef NET_DP8393X_H
#define NET_DP8393X_H

#include <86box/device.h>

/* IBM LAN Adapter/A for Ethernet: long and short board variants. */
extern const device_t ibm_lan_a_device;
extern const device_t ibm_lan_a_short_device;

#endif /*NET_DP8393X_H*/
