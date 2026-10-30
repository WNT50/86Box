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
 * Authors: WNT50
 *
 *          Copyright 2026 WNT50.
 */
#ifndef EMU_SCSI_SM912_H
#define EMU_SCSI_SM912_H

typedef struct scsi_sm912_t scsi_sm912_t;

/* IRQ line callback: the adapter asks for one when the target has a status or
   message byte waiting, exactly like the on-board 8259 IR5 input. */
typedef void (*scsi_sm912_irq_t)(void *priv, int level);

extern scsi_sm912_t *scsi_sm912_init(uint16_t base, scsi_sm912_irq_t irq,
                                     void *irq_priv);
extern void          scsi_sm912_close(scsi_sm912_t *dev);
extern void          scsi_sm912_reset(scsi_sm912_t *dev);

/* Whether a target answers a selection.  The PC-X POST probes the bus and
   treats "nothing there" and "there but unhappy" differently, so the machine
   layer can say whether the bridge board is fitted. */
extern void          scsi_sm912_set_target(scsi_sm912_t *dev, int present);

#endif /*EMU_SCSI_SM912_H*/
