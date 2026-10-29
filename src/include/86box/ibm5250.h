/*
 * 86Box    5250 Work Station Adapter (IBM 3477-S / 5550 family).
 *
 * Standalone device model: ports 0x1B0-0x1B6 (control) and 0x1C0-0x1D7
 * (data / status / strobe), plus one interrupt line.
 *
 * The machine supplies the IRQ line callback; the device never calls into
 * the PIC directly so the same model can run on machines with different
 * interrupt controllers.
 */
#ifndef IBM5250_H
#define IBM5250_H

#include <stdint.h>

typedef struct ibm5250_t ibm5250_t;

/* Called when the adapter wants to raise its interrupt line. */
typedef void (*ibm5250_irq_t)(void *priv);

ibm5250_t *ibm5250_init(int irq, ibm5250_irq_t irq_raise, void *irq_priv,
                        ibm5250_irq_t irq_clear, void *irq_clear_priv);

/* 客人写 0x1C2=09（选中适配器共享 RAM 窗口）时清 0xE400:0000 起的 256 字节。 */
void       ibm5250_set_shared_clear(ibm5250_t *dev,
                                    void (*fn)(void *priv), void *priv);

/* 客人写 0x722（ISR 收尾）时调用：消掉“数据就绪”电平。 */
void       ibm5250_irq_ack(ibm5250_t *dev);
void       ibm5250_close(ibm5250_t *dev);
void       ibm5250_reset(ibm5250_t *dev);
int        ibm5250_in_ab0(ibm5250_t *dev);

#endif /* IBM5250_H */
