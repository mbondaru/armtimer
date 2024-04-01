#include "irqctrl.h"

void irqctrl_init(volatile unsigned **uirqctrl_ptr)
{
   int fd;
   void *map;
   fd = open("/dev/mem", O_RDWR | O_SYNC);
   if(fd < 0)
   {
      perror("Opening /dev/mem");
      exit(1);
   }
   map = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
      MAP_SHARED, fd, IRQ_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }  
   close(fd);
   *uirqctrl_ptr = (volatile unsigned *) map;
}

void irqctrl_print(Interrupt_Controller_Type *irqctrl)
{
   printf("Printing all IRQ Registers contents:\n");
   printf("IRQBP = 0x%X\n", irqctrl->IRQBP);
   printf("IRQP1 = 0x%X\n", irqctrl->IRQP1);
   printf("IRQP2 = 0x%X\n", irqctrl->IRQP2);
   printf("FIQCNTL = 0x%X\n", irqctrl->FIQCNTL);
   printf("IRQEN1 = 0x%X\n", irqctrl->IRQEN1);
   printf("IRQEN2 = 0x%X\n", irqctrl->IRQEN2);
   printf("BASIC_IRQEN = 0x%X\n", irqctrl->BASIC_IRQEN);
   printf("DISABLE_IRQ1 = 0x%X\n", irqctrl->DISABLE_IRQ1);
   printf("DISABLE_IRQ2 = 0x%X\n", irqctrl->DISABLE_IRQ2);
   printf("DISABLE_BASIC_IRQ = 0x%X\n", irqctrl->DISABLE_BASIC_IRQ);
}
void irqctrl_enable_fiq(Interrupt_Controller_Type *irqctrl, uint32_t irq)
{
   irqctrl->FIQCNTL = IRQ_FIQ_ENABLE | irq;
}
void irqctrl_disable_fiq(Interrupt_Controller_Type *irqctrl)
{
   irqctrl->FIQCNTL &= ~IRQ_FIQ_ENABLE;
}
