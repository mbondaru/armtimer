#include "cm.h"

void cm_init(volatile unsigned **ucm_ptr)
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
      MAP_SHARED, fd, CM_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }
   close(fd);
   *ucm_ptr = (volatile unsigned *) map;
}

void cm_print(CM_Type *cm)
{
   printf("Printing all cm registers contents\n");
   printf("GPCTL0 = 0x%X\n", cm->GPCTL0);
   printf("GPDIV0 = 0x%X\n", cm->GPDIV0);
   printf("GPCTL1 = 0x%X\n", cm->GPCTL1);
   printf("GPDIV1 = 0x%X\n", cm->GPDIV1);
   printf("GPCTL2 = 0x%X\n", cm->GPCTL2);
   printf("GPDIV2 = 0x%X\n", cm->GPDIV2);
}

void cm_gpclk0_29_5_MHz(CM_Type *cm)
{
   if(cm->GPCTL0 & CM_GPCTL0_BUSY)
   {
      cm_gpclk0_kill(cm);
   }
   else
   {
      /* Source=PLLD, MASH = 0, divi = 68 */
      cm->GPDIV0 = (0x5A << CM_GPDIV0_PASSWD_OFS) |
         (17 << CM_GPDIV0_DIVI_OFS);
      printf("GPDIV0 = 0x%X\n", cm->GPDIV0);
      cm->GPCTL0 = (0x5A << CM_GPCTL0_PASSWD_OFS) |
         CM_GPCTL0_MASH__INTEGER_DIVISION |
         CM_GPCTL0_SRC__PLLD |
         CM_GPCTL0_ENAB;
      printf("GPCTL0 = 0x%X\n", cm->GPCTL0);
   }
}
void cm_gpclk0_kill(CM_Type *cm)
{
   cm->GPCTL0 = (0x5A << CM_GPCTL0_PASSWD_OFS) | CM_GPCTL0_KILL;
}
void cm_gpclk0_passwd(CM_Type *cm)
{
   cm->GPCTL0 = (0x5A << CM_GPCTL0_PASSWD_OFS); 
   printf("GPCTL0 = 0x%X\n", cm->GPCTL0);
}
