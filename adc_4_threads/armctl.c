#include "armctl.h"
void armctl_init(volatile unsigned **uarmctl_ptr)
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
      MAP_SHARED, fd, LOCAL_PERIPH_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }  
   close(fd);
   *uarmctl_ptr = (volatile unsigned *) map;
}
void armctl_print(ARM_Control_Logic_Module_Type *armctl)
{
   printf("Printing all armctl registers contents:\n");
   printf("CTL = 0x%X\n", armctl->CTL);
   printf("CORETIMER_PRESCALER = 0x%X\n", armctl->CORETIMER_PRESCALER);
   printf("GPU_IRQ_ROUTING = 0x%X\n", armctl->GPU_IRQ_ROUTING);
   printf("CORETIMER_BITS_LO = 0x%X\n", armctl->CORETIMER_BITS_LO);
   printf("CORETIMER_BITS_HI = 0x%X\n", armctl->CORETIMER_BITS_HI);
   printf("LOCAL_IRQ_ROUTING = 0x%X\n", armctl->LOCAL_IRQ_ROUTING);
   printf("AXI_OUTSTANDING_COUNTERS = 0x%X\n",
      armctl->AXI_OUTSTANDING_COUNTERS);
   printf("AXI_OUTSTANDING_IRQ = 0x%X\n", armctl->AXI_OUTSTANDING_IRQ);
   printf("LOCAL TIMER CONTROL AND STATUS = 0x%X\n",
      armctl->LTCSR);
   printf("CORE0 TIMERS IRQ CONTROL = 0x%X\n",
      armctl->CORE0_TIMERS_IRQ_CTL);
   printf("CORE1 TIMERS IRQ CONTROL = 0x%X\n",
      armctl->CORE1_TIMERS_IRQ_CTL);
   printf("CORE2 TIMERS IRQ CONTROL = 0x%X\n",
      armctl->CORE2_TIMERS_IRQ_CTL);
   printf("CORE3 TIMERS IRQ CONTROL = 0x%X\n",
      armctl->CORE3_TIMERS_IRQ_CTL);
   printf("CORE0 MAILBOXES IRQ CONTROL = 0x%X\n",
      armctl->CORE0_MAILBOXES_IRQ_CTL);
   printf("CORE1 MAILBOXES IRQ CONTROL = 0x%X\n",
      armctl->CORE1_MAILBOXES_IRQ_CTL);
   printf("CORE2 MAILBOXES IRQ CONTROL = 0x%X\n",
      armctl->CORE2_MAILBOXES_IRQ_CTL);
   printf("CORE3 MAILBOXES IRQ CONTROL = 0x%X\n",
      armctl->CORE3_MAILBOXES_IRQ_CTL);
   printf("CORE0 IRQ SOURCE = 0x%X\n", armctl->CORE0_IRQ_SRC);
   printf("CORE1 IRQ SOURCE = 0x%X\n", armctl->CORE1_IRQ_SRC);
   printf("CORE2 IRQ SOURCE = 0x%X\n", armctl->CORE2_IRQ_SRC);
   printf("CORE3 IRQ SOURCE = 0x%X\n", armctl->CORE3_IRQ_SRC);
   printf("CORE0 FIQ SOURCE = 0x%X\n", armctl->CORE0_FIQ_SRC);
   printf("CORE1 FIQ SOURCE = 0x%X\n", armctl->CORE1_FIQ_SRC);
   printf("CORE2 FIQ SOURCE = 0x%X\n", armctl->CORE2_FIQ_SRC);
   printf("CORE3 FIQ SOURCE = 0x%X\n", armctl->CORE3_FIQ_SRC);
   printf("CORE0 MAILBOX0 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE0 MAILBOX1 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE0 MAILBOX2 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE0 MAILBOX3 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE1 MAILBOX0 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE1 MAILBOX1 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE1 MAILBOX2 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE1 MAILBOX3 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE2 MAILBOX0 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE2 MAILBOX1 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE2 MAILBOX2 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE2 MAILBOX3 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE3 MAILBOX0 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_0_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE3 MAILBOX1 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_1_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE3 MAILBOX2 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_2_READ_WRITE_HIGH_TO_CLEAR);
   printf("CORE3 MAILBOX3 READ WRITE HIGH TO CLEAR = 0x%X\n",
      armctl->CORE0_MAILBOX_3_READ_WRITE_HIGH_TO_CLEAR);
}

void set_coretimer_divider_maximum(ARM_Control_Logic_Module_Type *armctl)
{
   printf("CTL = 0x%X\n", armctl->CTL);
   printf("CORETIMER_PRESCALER = 0x%X\n", armctl->CORETIMER_PRESCALER);
   armctl->CORETIMER_PRESCALER = CORE_TIMER_MAX_DIVIDER;
   printf("CTL = 0x%X\n", armctl->CTL);
   printf("CORETIMER_PRESCALER = 0x%X\n", armctl->CORETIMER_PRESCALER);
}

void set_coretimer_src_crystal(ARM_Control_Logic_Module_Type *armctl)
{
   armctl->CTL &= ~ARMCTL_CTL_CORETIMER_CLK_SRC;
   printf("CTL = 0x%X\n", armctl->CTL);
}

void set_coretimer_src_apb_clock(ARM_Control_Logic_Module_Type *armctl)
{
   armctl->CTL |= ARMCTL_CTL_CORETIMER_CLK_SRC;
   printf("CTL = 0x%X\n", armctl->CTL);
}
