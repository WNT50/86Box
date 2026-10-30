/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Siemens PC-D / PC-X keyboard and terminal controller, modelled at
 *          the protocol level (no MCS-51 / MCS-48 core).
 *
 *
 *
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_KBC_SIEMENS_H
#define EMU_KBC_SIEMENS_H

typedef struct kbc_siemens_t kbc_siemens_t;

/* Create and destroy the controller. */
extern kbc_siemens_t *kbc_siemens_init(void);
extern void           kbc_siemens_close(kbc_siemens_t *dev);

/* Hand it every byte the host writes to 0xf980 (the character stream the
   video board would otherwise consume). */
extern void kbc_siemens_rx(kbc_siemens_t *dev, uint8_t byte);

/* Serve the host's reads of 0xf980 (data) and 0xf981 (status). */
extern uint8_t kbc_siemens_read_data(kbc_siemens_t *dev);
extern uint8_t kbc_siemens_read_status(kbc_siemens_t *dev);

/* Inject a byte from the keyboard, delivered on the next host read. */
extern void kbc_siemens_key(kbc_siemens_t *dev, uint8_t byte);

#endif /*EMU_KBC_SIEMENS_H*/
