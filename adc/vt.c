#include "vt.h"

void vt_init(volatile unsigned **uvt_ptr)
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
      MAP_SHARED, fd, VT_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }  
   close(fd);
   *uvt_ptr = (volatile unsigned *) map;
}

void vt_print(Vector_Table_Type *vt)
{
   int i;
   printf("Printing Exception Vector Table Contents:\n");
   printf("RESET=0x%X\n", vt->RESET);
   printf("UNDEF=0x%X\n", vt->UNDEF);
   printf("SWI=0x%X\n", vt->SWI);
   printf("PABT=0x%X\n", vt->PABT);
   printf("DABT=0x%X\n", vt->DABT);
   printf("IRQ=0x%X\n", vt->IRQ);
   printf("FIQ=\n");
   for(i = 0; i < 50; i++)
   {
      printf("0x%X\n", (vt->FIQ)[i]);
   }
}
