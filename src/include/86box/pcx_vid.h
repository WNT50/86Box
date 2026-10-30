/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-X video board (SCN2672 + 8031) definitions.
 */

#ifndef EMU_PCX_VID_H
#define EMU_PCX_VID_H

#include <stdint.h>
#include <86box/device.h>

extern const device_t pcx_video_device;

/* Future keyboard path: a byte from the board's 8031 back to the host, which
   the host reads at 0xf980.  Takes the device pointer device_add() returned. */
extern void pcx_video_set_key(void *dev, uint8_t val);

struct kbc_siemens_t;
extern void pcx_video_set_kbc(void *dev, struct kbc_siemens_t *kbc);

#endif /*EMU_PCX_VID_H*/
