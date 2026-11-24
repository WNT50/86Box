/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Declarations shared by the floppy disk controllers: the sector
 *          id a command reports and the seam a hosted controller uses to
 *          reach the DMA channel and the interrupt line.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *          Fred N. van Kempen, <decwiz@yahoo.com>
 *          Toni Riikonen, <riikonen.toni@gmail.com>
 *
 *          Copyright 2008-2020 Sarah Walker.
 *          Copyright 2016-2020 Miran Grca.
 *          Copyright 2018-2020 Fred N. van Kempen.
 *          Copyright 2025 Toni Riikonen.
 */
#ifndef EMU_FDC_H
#define EMU_FDC_H

typedef struct sector_id_fields_t {
    uint8_t c;
    uint8_t h;
    uint8_t r;
    uint8_t n;
} sector_id_fields_t;

typedef union sector_id_t {
    uint32_t           dword;
    uint8_t            byte_array[4];
    sector_id_fields_t id;
} sector_id_t;

/*
   The device an FDC with FDC_FLAG_HOSTED sits behind. It takes the place
   of the ISA DMA controller and the PIC: dma_read and dma_write move one
   byte and return DMA_OVER at terminal count (dma_read DMA_NODATA when
   it has none), as dma_channel_read and dma_channel_write do.
 */
typedef struct fdc_host_t {
    int  (*dma_read)(void *priv);
    int  (*dma_write)(void *priv, uint8_t val);
    void (*drq)(void *priv, int set);
    void (*irq)(void *priv, int set);
} fdc_host_t;

#include <86box/fdc_upd765.h>

#endif /*EMU_FDC_H*/
