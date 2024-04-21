#include "aux.h"

void aux_init(volatile unsigned **uaux_ptr)
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
      MAP_SHARED, fd, AUX_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }  
   close(fd);

   *uaux_ptr = (volatile unsigned *) map;
}

void aux_print(AUX_Type *aux)
{
   printf("Printing all AUX registers contents:\n");
}
