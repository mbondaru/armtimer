#include "systimer.h"

void systimer_init(volatile unsigned **usystimer_ptr)
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
      MAP_SHARED, fd, SYSTIMER_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }  
   close(fd);
   *usystimer_ptr = (volatile unsigned *) map;
}

void systimer_print(SysTimer_Type *systimer)
{
   printf("Printing all System Timer register contents...\n");
   printf("CS = 0x%X\n", systimer->CS);
   printf("CLO = 0x%X\n", systimer->CLO);
   printf("CHI = 0x%X\n", systimer->CHI);
   printf("C0 = 0x%X\n", systimer->C0);
   printf("C1 = 0x%X\n", systimer->C1);
   printf("C2 = 0x%X\n", systimer->C2);
   printf("C3 = 0x%X\n", systimer->C3);
}
