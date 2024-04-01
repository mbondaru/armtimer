#include "armtimer.h"

void armtimer_init(volatile unsigned **uarmtimer_ptr)
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
      MAP_SHARED, fd, ARM_TIMER_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }  
   close(fd);
   *uarmtimer_ptr = (volatile unsigned *) map;
}

void armtimer_print(ARMTimer_Type *armtimer)
{
   printf("Printing all ARM Timer registers contents:\n");
   printf("LD = 0x%X\n", armtimer->LD);
   printf("VAL = 0x%X\n", armtimer->VAL);
   printf("CTL = 0x%X\n", armtimer->CTL);
   printf("IRQ_RAW = 0x%X\n", armtimer->IRQ_RAW);
   printf("IRQ_MASKED = 0x%X\n", armtimer->IRQ_MASKED);
   printf("RELOAD = 0x%X\n", armtimer->RELOAD);
   printf("PD = 0x%X\n", armtimer->PD);
   printf("FRC = 0x%X\n", armtimer->FRC);
}

void armtimer_enable(uint32_t ldval, ARMTimer_Type *armtimer)
{
   printf("Enabling ARM Timer...\n");
   armtimer->LD = ldval;
   armtimer->IRQ_CLR_ACK = CLEAR_IRQ;
   armtimer->CTL = ARMTIMER_CTL_TIMER_IRQ_EN |
      ARMTIMER_CTL_TIMER_EN;
      /*ARMTIMER_CTL_PRESCALE__16;*/
   /*armtimer_print(armtimer);*/
}
void armtimer_disable(ARMTimer_Type *armtimer)
{
   printf("Disabling ARM Timer...\n");
   armtimer->CTL &= ~(ARMTIMER_CTL_TIMER_IRQ_EN | 
      ARMTIMER_CTL_TIMER_EN);
   armtimer->IRQ_CLR_ACK = CLEAR_IRQ;
   /*armtimer_print(armtimer);*/
}
void armtimer_set_predivider(uint32_t pdiv, ARMTimer_Type *armtimer)
{
   armtimer->PD = (armtimer->PD & ~ARMTIMER_PRE_DIVIDER_MASK) |
      (pdiv & ARMTIMER_PRE_DIVIDER_MASK); 
   /*armtimer_print(armtimer);*/
}
void armtimer_set_16bit_width(ARMTimer_Type *armtimer)
{
   armtimer->CTL &= ~ARMTIMER_CTL_COUNTER_LEN; 
}
void armtimer_set_23bit_width(ARMTimer_Type *armtimer)
{
   armtimer->CTL |= ARMTIMER_CTL_COUNTER_LEN;
}
