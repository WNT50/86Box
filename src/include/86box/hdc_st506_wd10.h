/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Western Digital WD1010/WD2010 Winchester disk controller.
 */

#ifndef HDC_ST506_WD10_H
#define HDC_ST506_WD10_H

#include <stdint.h>

/* Variants understood by mfm_init(). */
#define WD10_VARIANT_1010 0 /* 1024 cylinders, CRC only */
#define WD10_VARIANT_2010 1 /* 2048 cylinders, accepts ECC commands */

typedef struct mfm_t mfm_t;

/* Lifecycle. */
mfm_t *wd10_init(int variant);
void   wd10_close(mfm_t *mfm);

/* Attach an image (already loaded through hdd_image_load()). */
void wd10_attach(mfm_t *mfm, int unit, int hdd_num, int spt, int hpc, int tracks);

/* Task file access, in absolute port numbers (01F0-01F7, 03F6). */
uint8_t  wd10_read(uint16_t port, void *priv);
uint16_t wd10_readw(uint16_t port, void *priv);
void     wd10_write(uint16_t port, uint8_t val, void *priv);
void     wd10_writew(uint16_t port, uint16_t val, void *priv);

#endif /* HDC_ST506_WD10_H */
