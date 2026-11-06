/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          HP-HIL internal interface: the MLC adapter (hp_hil.c) and the
 *          devices on the link (hp_hil_kbd.c, hp_hil_mouse.c) share this
 *          header.  Nothing here is exported outside the HP-HIL driver.
 */

#ifndef HP_HIL_H
#define HP_HIL_H

#include <stdint.h>
#include <86box/timer.h>

/* Device ID bytes (TRM 45918A appendix B).
   00..1Fh keypads (Vectra 46030A), 60..6Fh relative positioners
   (68h = HP Mouse 46060A). */
#define HIL_KBD_ID   0x00
#define HIL_MOUSE_ID 0x68

/* Describe Record header bits (appendix C). */
#define HIL_DESCR_ABS  0x40
#define HIL_DESCR_16B  0x20
#define HIL_DESCR_IOD  0x10

/* Poll Record header bits (appendix E). */
#define HIL_POL_READY   0x04 /* device ready for data transfer */
#define HIL_POL_AXES_XY 0x02 /* bits 1-0: 00 none, 01 X, 10 X and Y */

/* Mouse 46060A describe record: 8-bit relative X and Y, I/O descriptor. */
#define HIL_MOUSE_DESCR 0x12
#define HIL_MOUSE_RES_LO 0xc2
#define HIL_MOUSE_RES_HI 0x1e

/* HP-HIL commands the devices answer (Table 4-1). */
#define HIL_POL 0x10 /* Poll, 10..1f */

/* MLC register bits the devices care about (Appendix I). */
#define HIL_W3_APE 0x02 /* Auto Poll Enable: an AP strobe issues a POL */

#define HIL_FIFO_SIZE 16

typedef struct hp_hil_t hp_hil_t;

/* What a device on the link has to answer.  The adapter walks the chain and
   hands every frame to the device it addresses; the device owns the bytes it
   replies with. */
typedef struct hp_hil_dev_ops {
    const char *name;
    void (*idd)(hp_hil_t *dev, int i);
    int (*poll)(hp_hil_t *dev, int i);  /* 1 = a record was queued */
    void (*tick)(hp_hil_t *dev, int i); /* 10 ms housekeeping, may be NULL */
} hp_hil_dev_ops_t;

/* One device on the link.  SLC = the Link Control state machine: a device in
   Loop-Back mirrors every frame back to the System, a device in Pass-Thru
   retransmits it to the next device (TRM 45918A 4.2/4.3). */
typedef struct {
    uint8_t                 addr; /* match address, 0 = not configured */
    uint8_t                 id;   /* device ID byte, appendix B */
    uint8_t                 pt;   /* Pass-Thru Mode */
    uint8_t                 pwr;  /* Power-Up Mode: cleared by IFC */
    const hp_hil_dev_ops_t *ops;
} hil_dev_t;

#define HIL_KBD  0 /* nearest the System */
#define HIL_PTR  1
#define HIL_NDEV 2

struct hp_hil_t {
    /* MLC state. */
    uint16_t fifo[HIL_FIFO_SIZE];
    int      fifo_head;
    int      fifo_count;
    uint8_t  w1; /* command bit + address of the frame being built */
    uint8_t  w2;
    uint8_t  w3;
    uint8_t  r2;
    uint8_t  r3;
    uint8_t  loop; /* Loop-Back Mode: frames come straight back */

    /* The devices on the link, in chain order. */
    hil_dev_t dev[HIL_NDEV];
    uint8_t   scan_reported;
    uint32_t  ready_ctx; /* context already told the link is up */
    int8_t    dx;        /* pending displacement, latched by the mouse */
    int8_t    dy;
    pc_timer_t pointer_timer;
    pc_timer_t ap_timer;      /* 60 Hz AP strobe, drives Auto Poll */
    uint32_t   reports;
};

/* Adapter services the device modules build on. */
extern void     hp_hil_fifo_push(uint16_t frame);
extern uint16_t hp_hil_reply(uint8_t addr, uint8_t data);
extern void     hp_hil_transmit(uint8_t data);
extern uint32_t hp_hil_driver_ctx(void);
extern void     hp_hil_trace(const char *fmt, ...);

extern const hp_hil_dev_ops_t hp_hil_kbd_ops;
extern const hp_hil_dev_ops_t hp_hil_mouse_ops;

#endif /* HP_HIL_H */
