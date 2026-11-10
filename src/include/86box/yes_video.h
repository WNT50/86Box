/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Philips :YES video: declarations.
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_YES_VIDEO_H
#define EMU_YES_VIDEO_H

#include <86box/device.h>

extern const device_t yes_video_device;

/* The ROM places the frame buffer with MMCS, so the machine has to say where
   it is whenever the register is rewritten. */
extern void yes_video_set_base(void *priv, uint32_t base);

#endif /* EMU_YES_VIDEO_H */
