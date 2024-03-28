/*
 *  linux/drivers/clocksource/timer-sp.c
 *
 *  Copyright (C) 1999 - 2003 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/*
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_clk.h>
#include <linux/of_irq.h>
#include <linux/sched_clock.h>

#include <clocksource/timer-sp804.h>
*/
/*#include "linux/arch/arm/fiq.h"*/
/*#define __ASSEMBLY_*/
/*
#include <linux/err.h>
*/
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
/*
#include <linux/init.h>
*/
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/delay.h>
/* <linux/dma_mapping> not present */
#include <asm/fiq.h>
/*
#include <asm/io.h>
*/
#include <asm/pgtable.h>
/*#include <uapi/asm/ptrace.h>*/
#include <linux/ioctl.h>

/*#include "asm/io.h"*/
/*#include "timer-sp.h"*/

/* This driver: Generate a sine wave via 
GPIO-connected DAC by updating its inputs within the ARMTimer FIQ */
#include "rpi-pg-unaligned.h"

#define SINE_TABLE_SIZE (360)
#define DAC_14BIT_3_3V ((u16) 0x3FFF)
#define DAC_14BIT_1V ((u16) 0x1364)
#define DAC_14BIT_0_5V ((u16) 0x09B2)
#define PI (3.14159265358979323846)
uint32_t sine_comms[SINE_TABLE_SIZE] = {
   38, 39, 40, 40, 
   41, 42, 42, 43, 
   44, 44, 45, 46, 
   46, 47, 48, 48, 
   49, 50, 50, 51, 
   52, 52, 53, 53, 
   54, 55, 55, 56, 
   56, 57, 58, 58, 
   59, 59, 60, 61, 
   61, 62, 62, 63, 
   63, 64, 64, 65, 
   65, 66, 66, 67, 
   67, 68, 68, 68, 
   69, 69, 70, 70, 
   70, 71, 71, 72, 
   72, 72, 73, 73, 
   73, 73, 74, 74, 
   74, 74, 75, 75, 
   75, 75, 76, 76, 
   76, 76, 76, 76, 
   76, 77, 77, 77, 
   77, 77, 77, 77, 
   77, 77, 77, 77, 
   77, 77, 77, 77, 
   77, 77, 77, 77, 
   76, 76, 76, 76, 
   76, 76, 76, 75, 
   75, 75, 75, 74, 
   74, 74, 74, 73, 
   73, 73, 73, 72, 
   72, 72, 71, 71, 
   70, 70, 70, 69, 
   69, 68, 68, 68, 
   67, 67, 66, 66, 
   65, 65, 64, 64, 
   63, 63, 62, 62, 
   61, 61, 60, 59, 
   59, 58, 58, 57, 
   56, 56, 55, 55, 
   54, 53, 53, 52, 
   52, 51, 50, 50, 
   49, 48, 48, 47, 
   46, 46, 45, 44, 
   44, 43, 42, 42, 
   41, 40, 40, 39, 
   38, 38, 37, 36, 
   36, 35, 34, 34, 
   33, 32, 32, 31, 
   30, 30, 29, 28, 
   28, 27, 26, 26, 
   25, 24, 24, 23, 
   23, 22, 21, 21, 
   20, 19, 19, 18, 
   18, 17, 17, 16, 
   15, 15, 14, 14, 
   13, 13, 12, 12, 
   11, 11, 10, 10, 
   9, 9, 9, 8, 
   8, 7, 7, 7, 
   6, 6, 5, 5, 
   5, 4, 4, 4, 
   3, 3, 3, 3, 
   2, 2, 2, 2, 
   1, 1, 1, 1, 
   1, 0, 0, 0, 
   0, 0, 0, 0, 
   0, 0, 0, 0, 
   0, 0, 0, 0, 
   0, 0, 0, 0, 
   0, 0, 0, 0, 
   0, 0, 0, 0, 
   1, 1, 1, 1, 
   1, 2, 2, 2, 
   2, 3, 3, 3, 
   3, 4, 4, 4, 
   5, 5, 5, 6, 
   6, 7, 7, 7, 
   8, 8, 9, 9, 
   9, 10, 10, 11, 
   11, 12, 12, 13, 
   13, 14, 14, 15, 
   15, 16, 17, 17, 
   18, 18, 19, 19, 
   20, 21, 21, 22, 
   23, 23, 24, 24, 
   25, 26, 26, 27, 
   28, 28, 29, 30, 
   30, 31, 32, 32, 
   33, 34, 34, 35, 
   36, 36, 37, 38};
static unsigned int sine_index = 0;
/*struct device_node gpio;*/

/* May not need __iomem **/
/*
static void __iomem *timerbase;
static void __iomem *gpiobase;
static void __iomem *irqbase;
*/
static uint32_t *timerbase;
static uint32_t *gpiobase;
static uint32_t *irqbase;

#define P1 ((GPIO_Type *) gpiobase)
#define ARMTIMER ((ARMTimer_Type *) timerbase)
#define IRQCTRL ((Interrupt_Controller_Type *) irqbase)

/*static uint32_t armtimer_load_value = 8;*/
static unsigned int count = 0;

static struct fiq_handler sp804_fh = {
   .name = "armtimer",
};

/* Therefore &sp804_handler will give us a pointer to */
/* something defined externally that starts with a char */
/* (a char pointer) */

extern unsigned char sp804_handler, sp804_handler_end;
static uint8_t fiqstack[4096];

/*u32 fiqhandler_asm[2];*/
/*
void c_irq_handler(void)
{
   if(count % 2) 
   {
      P1->GPCLR0 = 0x040000FF;
   }
   else
   {
      P1->GPSET0 = sine_comms[sine_index];
      sine_index = (sine_index + 1) % SINE_TABLE_SIZE;
      P1->GPSET0 = 0x04000000;
   }
   count++;
   ARMTIMER->IRQ_CLR_ACK = 1;
}
*/
int init_sp804(void)
{
   int ret = -EINVAL;
   struct pt_regs regs;
   uint32_t *fiqstack_wordptr = (uint32_t *) &fiqstack[sizeof(fiqstack)];
   *(fiqstack_wordptr - 7) = 0;
   *(fiqstack_wordptr - 8) = 0;
   /* Gain access to peripheral registers */
   timerbase = ioremap(ARM_TIMER_BASE, sizeof(ARMTimer_Type));
   gpiobase = ioremap(GPIO_BASE, sizeof(GPIO_Type));
   irqbase = ioremap(IRQ_BASE, sizeof(Interrupt_Controller_Type));

   ARMTIMER->CTL = 0;
   ARMTIMER->IRQ_CLR_ACK = 1;
   IRQCTRL->FIQCNTL = IRQ_FIQ_ENABLE | IRQ_FIQ_SRC__ARM_TIMER;

   ret = claim_fiq(&sp804_fh);
   if(ret)
   {
      printk("sp804: claim_fiq failed\n");
      goto err;
   }
   regs.ARM_r8 = (long) gpiobase; 
   regs.ARM_r9 = (long) timerbase;
   regs.ARM_r10 = (long) sine_comms;
   regs.ARM_fp = (long) 0; /* r11=count */
   regs.ARM_ip = (long) 0; /* r12=sine_index */
   regs.ARM_sp = (long) &fiqstack[sizeof(fiqstack)];
   set_fiq_regs(&regs);
   set_fiq_handler(&sp804_handler, &sp804_handler_end - &sp804_handler);
   /* DAC D6 */ 
   P1->GPFSEL0 = (P1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL0_MASK) |
      (1 << GPIO_GPFSEL0_FSEL0_OFS);
   P1->GPCLR0 |= GPIO_GPCLR00;
   /* DAC D7 */
   P1->GPFSEL0 = (P1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL1_MASK) |
      (1 << GPIO_GPFSEL0_FSEL1_OFS);
   P1->GPCLR0 |= GPIO_GPCLR01;
   /* DAC D8 */
   P1->GPFSEL0 = (P1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL2_MASK) |
      (1 << GPIO_GPFSEL0_FSEL2_OFS);
   P1->GPCLR0 |= GPIO_GPCLR02;
   /* DAC D9 */
   P1->GPFSEL0 = (P1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL3_MASK) |
      (1 << GPIO_GPFSEL0_FSEL3_OFS);
   P1->GPCLR0 |= GPIO_GPCLR03;
   /* DAC D10 */
   P1->GPFSEL0 = (P1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL4_MASK) |
      (1 << GPIO_GPFSEL0_FSEL4_OFS);
   P1->GPCLR0 |= GPIO_GPCLR04;
   /* DAC D11 */
   P1->GPFSEL0 = (P1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL5_MASK) |
      (1 << GPIO_GPFSEL0_FSEL5_OFS);
   P1->GPCLR0 |= GPIO_GPCLR05;
   /* DAC D12 */
   P1->GPFSEL0 = (P1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL6_MASK) |
      (1 << GPIO_GPFSEL0_FSEL6_OFS);
   P1->GPCLR0 |= GPIO_GPCLR06;
   /* DAC D13 (MSB) */
   P1->GPFSEL0 = (P1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL7_MASK) |
      (1 << GPIO_GPFSEL0_FSEL7_OFS);
   P1->GPCLR0 |= GPIO_GPCLR07;

   P1->GPFSEL2 = (P1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL26_MASK) |
      (1 << GPIO_GPFSEL2_FSEL26_OFS);
   P1->GPCLR0 |= GPIO_GPCLR026;

   ARMTIMER->LD = 8;
   ARMTIMER->PD = 0;
   printk(KERN_INFO "LD=0x%X PD=0x%X CTL=0x%X", ARMTIMER->LD,
      ARMTIMER->PD, ARMTIMER->CTL);
   printk(KERN_INFO "VAL=0x%X RELOAD=0x%X IRQ_RAW=0x%X",
      ARMTIMER->VAL, ARMTIMER->RELOAD, ARMTIMER->IRQ_RAW);
   printk(KERN_INFO "IRQ_MASKED=0x%X FRC=0x%X",
      ARMTIMER->IRQ_MASKED, ARMTIMER->FRC);
   printk(KERN_INFO " ");
   ARMTIMER->CTL = ARMTIMER_CTL_TIMER_EN | ARMTIMER_CTL_TIMER_IRQ_EN;
   return 0;
err:
   iounmap(timerbase);
   iounmap(gpiobase);
   iounmap(irqbase);
   return ret;
}
void exit_sp804(void)
{
	/* Ensure timer and its interrupt are disabled */
	/*writel(0, ARMTIMER->CTL);*/
   ARMTIMER->CTL = 0;
   /* Ensure interrupt flag is cleared */
	/*writel(1, ARMTIMER->IRQ_CLEAR_ACK);*/
   ARMTIMER->IRQ_CLR_ACK = 1;
   /* Enable ARM Timer FIQ */
   /*writel(IRQ_FIQ_ENABLE | IRQ_FIQ_SRC__ARM_TIMER, ARMCTRL->FIQCNTL);*/
   IRQCTRL->FIQCNTL = 0;
   
   release_fiq(&sp804_fh);
   printk(KERN_INFO "Removed ARM Timer SP804 Module. \n"); 
   return;
}
module_init(init_sp804);
module_exit(exit_sp804);

MODULE_DESCRIPTION("29.41MHz 8bit DAC Inputs for 1Vpp sine wave using ARM Timer FIQs");
MODULE_AUTHOR("BOIBOI");
MODULE_LICENSE("GPL");
