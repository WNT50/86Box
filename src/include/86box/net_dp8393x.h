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

#include <stdint.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/network.h>

#define SONIC_RAM_SIZE     0x8000              /* 8K x 32-bit shared SRAM */
#define SONIC_TX_FRAME_MAX (NET_MAX_FRAME + 4) /* 1522 */

#define SONIC_IO_REGS_SIZE 0x80 /* +00h .. +7Fh: internal registers */

typedef struct dp8393x_t {
    /* Registers and CAM */
    uint16_t regs[64];
    uint16_t cam[48]; /* 16 entries x 3 words */
    uint16_t cam_enable;
    uint16_t cep;
    uint16_t cam_valid;

    /* TX/RX state */
    uint8_t  rx_enabled;
    uint8_t  stopped;
    uint8_t  irq_level;
    void   (*irq_cb)(void *priv, int level);
    void    *irq_priv;
    uint8_t  tx_active;

    /* On-chip resources */
    uint8_t  ram[SONIC_RAM_SIZE];
    uint8_t  tx_frame[SONIC_TX_FRAME_MAX];
    uint32_t ram_size; /* DMA address mask, set by the board window */

    /* Receive engine state (P3) */
    uint32_t rx_buf;      /* byte offset of the current RBA in shared RAM */
    uint32_t rx_words;    /* remaining words in the current RBA (16-bit words) */
    uint8_t  rx_rba_full; /* LPKT: a new RRA is required before the next packet */
    uint8_t  rx_rbe;      /* RRA exhausted; reception stops until RBE is cleared */
    uint8_t  rx_rde;      /* RDA exhausted; reception stops until RDE is cleared */
    uint8_t  rx_pkt_seq;  /* RSC low byte: packet sequence within the RBA */
    uint8_t  rx_rba_seq;  /* RSC high byte: RBA sequence number */

    /* Generic watchdog timer (WT0/WT1), used by the Boot ROM POST */
    pc_timer_t wt_timer;
    uint32_t   wt_count;
    uint8_t    wt_running;

    /* Network backend, filled in by the board after network_attach */
    netcard_t *card;
} dp8393x_t;

extern const device_t dp8393x_device;
extern int            dp8393x_inst;

void     dp8393x_init(dp8393x_t *dev);
void     dp8393x_reset(dp8393x_t *dev);
uint16_t dp8393x_reg_read(dp8393x_t *dev, uint16_t off);
void     dp8393x_reg_write(dp8393x_t *dev, uint16_t off, uint16_t val);
int      dp8393x_rx(dp8393x_t *dev, const uint8_t *buf, int len, int loopback);
uint8_t  dp8393x_ram_read8(const dp8393x_t *dev, uint16_t off);
void     dp8393x_ram_write8(dp8393x_t *dev, uint16_t off, uint8_t val);
void     dp8393x_set_ram_size(dp8393x_t *dev, uint32_t size);

#endif /*NET_DP8393X_H*/
