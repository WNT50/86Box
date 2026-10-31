/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          HP-HIL host controller (U777) definitions.
 */
#ifndef EMU_HP_HIL_H
#define EMU_HP_HIL_H

#include <86box/device.h>

extern const device_t hp_hil_device;

/* The 8041 is the link master: when the System asks it to reset the link, it
   scans for devices.  Nothing on this link has been addressed yet, so the
   scan comes back empty and the adapter reports a link error. */
extern void hp_hil_link_scan(void);

#endif /*EMU_HP_HIL_H*/
