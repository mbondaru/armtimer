#ifndef IRQCTRL_H_
#define IRQCTRL_H_

#include "rpi.h"
#include "sys/mman.h"
#include "fcntl.h"
#include "stdlib.h"
#include "unistd.h"
#include "stdio.h"
#include "system_rpi.h"

#ifndef BLOCK_SIZE
#define BLOCK_SIZE (4*1024)
#endif

void irqctrl_init(volatile unsigned **uirqctrl_ptr);
void irqctrl_print(Interrupt_Controller_Type *irqctrl);
void irqctrl_disable_fiq(Interrupt_Controller_Type *irqctrl);
/* irqctrl_enable_fiq - enable FIQ (Fast Interrupt) */
/* for a single interrupt source (per ARM chip). */
/* Params: */
/* Interrupt_Controller_Type *armctrl - memory mapped IRQ registers */
/* uint32_t irq - Interrupt source number */
/* [0-63] - GPU interrupts */
/* 0 - do not use */
/* 1 - system timer match 1 */
/* 2 - do not use */
/* 3 - system timer match 3 */
/* 4-8 - do not use */
/* 9 - USB Controller */
/* 10-28 - do not use */
/* 29 - AUX interrupt */
/* 30-42 - do not use */
/* 43 - i2c_spi_slv_int */
/* 44 - do not use */
/* 45 - pwa0 */
/* 46 - pwa1 */
/* 47 - do not use */
/* 48 - smi */
/* 49 - gpio_int[0] */
/* 50 - gpio_int[1] */
/* 51 - gpio_int[2] */
/* 52 - gpio_int[3] */
/* 53 - i2c_int */
/* 54 - spi_int */
/* 55 - pcm_int */
/* 56 - do not use */
/* 57 - uart_int */
/* 58-63 - do not use */
/* [64-71] - ARM-specific interrupts */
/* 64 - ARM Timer Interrupt */
/* 65 - ARM Mailbox interrupt */
/* 66 - ARM Doorbell 0 interrupt */
/* 67 - ARM Doorbell 1 interrupt */
/* 68 - GPU0 Halted Interrupt */
/* 69 - GPU1 Halted Interrupt */
/* 70 - Illegal access type-1 interrupt */
/* 71 - Illegal access type-0 interrupt */
/* 72-127 - do not use */
void irqctrl_enable_fiq(Interrupt_Controller_Type *irqctrl, uint32_t irq);

#endif /* IRQCTRL_H_ */
