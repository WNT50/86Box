/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          DEC VAXmate onboard Ethernet: an AMD Am7990 LANCE behind the
 *          DEC interface logic.  This module owns everything that is
 *          specific to the VAXmate and adds the generic chip:
 *
 *            0x0C60 (alias 0x0C64)  16-bit  RDP
 *            0x0C62 (alias 0x0C66)  16-bit  RAP
 *            0x0C68                 8-bit   NICSR (interrupt/LED gate)
 *            IRQ10, vector 72h
 *
 *          The station address comes from the LAN address PROM at 0x0C20,
 *          which the machine layer also mirrors into NVM.
 */
#include <stdint.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/net_lance.h>

#define VAXMATE_LANCE_BASE      0x0c60
#define VAXMATE_LANCE_IRQ       10
#define VAXMATE_LANCE_NICSR     0x0c68

/* NICSR bits (TRM page 13-17, write only, set/clear style). */
#define NICSR_INT_SET           0x01
#define NICSR_INT_CLEAR         0x04
#define NICSR_LED_CLEAR         0x08
#define NICSR_LED_SET           0x10

static const lance_board_t vaxmate_lance_board = {
    .base = VAXMATE_LANCE_BASE,
    .irq  = VAXMATE_LANCE_IRQ,
    .mac  = { 0x08, 0x00, 0x2b, 0x12, 0x34, 0x56 }
};

static void *vaxmate_lance;

static void
vaxmate_lance_nicsr_write(uint16_t addr, uint8_t val, void *priv)
{
    int gate = 0;

    (void) addr;
    (void) priv;

    /* TRM 13-17: a 1 in the set bit enables, a 1 in the clear bit disables,
       and setting both toggles the state. */
    if ((val & NICSR_INT_SET) && (val & NICSR_INT_CLEAR))
        gate = -1;
    else if (val & NICSR_INT_SET)
        gate = 1;
    else if (val & NICSR_INT_CLEAR)
        gate = 0;

    if (gate >= 0)
        lance_set_irq_gate(vaxmate_lance, gate);
    else
        lance_set_irq_gate(vaxmate_lance, !lance_get_irq_gate(vaxmate_lance));
}

static void *
vaxmate_lance_init(const device_t *info)
{
    (void) info;

    vaxmate_lance = device_add_linked(&lance_device, (void *) &vaxmate_lance_board);
    if (vaxmate_lance == NULL)
        return NULL;

    io_sethandler(VAXMATE_LANCE_NICSR, 0x0008,
                  NULL, NULL, NULL,
                  vaxmate_lance_nicsr_write, NULL, NULL, NULL);

    return vaxmate_lance;
}

static void
vaxmate_lance_close(void *priv)
{
    (void) priv;

    io_removehandler(VAXMATE_LANCE_NICSR, 0x0008,
                     NULL, NULL, NULL,
                     vaxmate_lance_nicsr_write, NULL, NULL, NULL);
    device_close(&lance_device);
    vaxmate_lance = NULL;
}

static void
vaxmate_lance_reset(void *priv)
{
    /* Power-on state: interface interrupt gate disabled. */
    lance_set_irq_gate(priv, 0);
}

const device_t vaxmate_lance_device = {
    .name          = "DEC VAXmate onboard Ethernet (Am7990)",
    .internal_name = "vaxmate_lance",
    .flags         = DEVICE_ISA16,
    .local         = 0,
    .init          = vaxmate_lance_init,
    .close         = vaxmate_lance_close,
    .reset         = vaxmate_lance_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
