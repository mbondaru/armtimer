#include "gpio.h"
void pads_init(volatile unsigned **upads_ptr)
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
      MAP_SHARED, fd, GPIO_PADS_CTL_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }  
   close(fd);
   *upads_ptr = (volatile unsigned *) map;
}

void gpio_init(volatile unsigned **ugpio_ptr)
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
      MAP_SHARED, fd, GPIO_BASE);
   if((long) map == -1L)
   {
      perror("mmap(/dev/mem)");
      exit(1);
   }  
   close(fd);

   *ugpio_ptr = (volatile unsigned *) map;
}

void gpio_print(GPIO_Type *p1)
{
   printf("Printing all GPIO Register contents\n");
   printf("GPFSEL0 = 0x%X\n", p1->GPFSEL0);
   printf("GPFSEL1 = 0x%X\n", p1->GPFSEL1);
   printf("GPFSEL2 = 0x%X\n", p1->GPFSEL2);
   printf("GPFSEL3 = 0x%X\n", p1->GPFSEL3);
   printf("GPFSEL4 = 0x%X\n", p1->GPFSEL4);
   printf("GPFSEL5 = 0x%X\n", p1->GPFSEL5);
   printf("GPSET0 = 0x%X\n", p1->GPSET0);
   printf("GPSET1 = 0x%X\n", p1->GPSET1);
   printf("GPCLR0 = 0x%X\n", p1->GPCLR0);
   printf("GPCLR1 = 0x%X\n", p1->GPCLR1);
   printf("GPLEV0 = 0x%X\n", p1->GPLEV0);
   printf("GPLEV1 = 0x%X\n", p1->GPLEV1);
   printf("GPEDS0 = 0x%X\n", p1->GPEDS0);
   printf("GPEDS1 = 0x%X\n", p1->GPEDS1);
   printf("GPREN0 = 0x%X\n", p1->GPREN0);
   printf("GPREN1 = 0x%X\n", p1->GPREN1);
   printf("GPFEN0 = 0x%X\n", p1->GPFEN0);
   printf("GPFEN1 = 0x%X\n", p1->GPFEN1);
   printf("GPHEN0 = 0x%X\n", p1->GPHEN0);
   printf("GPHEN1 = 0x%X\n", p1->GPHEN1);
   printf("GPLEN0 = 0x%X\n", p1->GPLEN0);
   printf("GPLEN1 = 0x%X\n", p1->GPLEN1);
   printf("GPAREN0 = 0x%X\n", p1->GPAREN0);
   printf("GPAREN1 = 0x%X\n", p1->GPAREN1);
   printf("GPAFEN0 = 0x%X\n", p1->GPAFEN0);
   printf("GPAFEN1 = 0x%X\n", p1->GPAFEN1);
   printf("GPPUD = 0x%X\n", p1->GPPUD);
   printf("GPPUDCLK0 = 0x%X\n", p1->GPPUDCLK0);
   printf("GPPUDCLK1 = 0x%X\n", p1->GPPUDCLK1);
}

void pads_print(GPIO_Pads_Control_Type *pads)
{
   printf("Printing all GPIO Pads control register contents:\n");   
   printf("PADS0_27 = 0x%X\n", pads->PADS0_27);
   printf("PADS28_45 = 0x%X\n", pads->PADS28_45);
   printf("PADS46_53 = 0x%X\n", pads->PADS46_53);
}

void set_pads_0_27_maximum_drive_strength(GPIO_Pads_Control_Type *pads)
{
   printf("PADS0_27 = 0x%X\n", pads->PADS0_27);
   pads->PADS0_27 = (0x5A << GPIO_PADS0_27_PASSWD_OFS) |
      GPIO_PADS0_27_HYSTERESIS |
      GPIO_PADS0_27_SLEW_RATE |
      GPIO_PADS0_27_DRIVE__16MILLIAMPS;
   printf("PADS0_27 = 0x%X\n", pads->PADS0_27);
}

void set_pads_0_27_moderate_drive_strength(GPIO_Pads_Control_Type *pads)
{
   printf("PADS0_27 = 0x%X\n", pads->PADS0_27);
   pads->PADS0_27 = (0x5A << GPIO_PADS0_27_PASSWD_OFS) |
      GPIO_PADS0_27_HYSTERESIS |
      GPIO_PADS0_27_SLEW_RATE | 
      GPIO_PADS0_27_DRIVE__16MILLIAMPS;
   printf("PADS0_27 = 0x%X\n", pads->PADS0_27);
}

void set_pads_28_45_moderate_drive_strength(GPIO_Pads_Control_Type *pads)
{
   printf("PADS28_45 = 0x%X\n", pads->PADS28_45);
   pads->PADS28_45 = (0x5A << GPIO_PADS28_45_PASSWD_OFS) |
      GPIO_PADS28_45_HYSTERESIS |
      GPIO_PADS28_45_SLEW_RATE |
      GPIO_PADS28_45_DRIVE__8MILLIAMPS;
   printf("PADS28_45 = 0x%X\n", pads->PADS28_45);
}

void gpio_set_func(uint32_t pin_number, uint32_t func_number,
   GPIO_Type *p1)
{
   if(func_number < 0 || func_number >= RPI_GPIO_PIN_FUNC_COUNT)
   {
      printf("Invalid pin function number %d\n", func_number);
      return;
   }
   else
   {
      printf("GPFSEL0 = 0x%X\n", p1->GPFSEL0);
      printf("GPFSEL1 = 0x%X\n", p1->GPFSEL1);
      printf("GPFSEL2 = 0x%X\n", p1->GPFSEL2);
      printf("GPFSEL3 = 0x%X\n", p1->GPFSEL3);
      printf("GPFSEL4 = 0x%X\n", p1->GPFSEL4);
      printf("GPFSEL5 = 0x%X\n", p1->GPFSEL5);
      switch(pin_number)
      {
         case GPIO_PIN0:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL0_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL0_OFS); 
            break;
         case GPIO_PIN1:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL1_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL1_OFS); 
            break;
         case GPIO_PIN2:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL2_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL2_OFS); 
            break;
         case GPIO_PIN3:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL3_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL3_OFS); 
            break;
         case GPIO_PIN4:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL4_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL4_OFS); 
            break;
         case GPIO_PIN5:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL5_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL5_OFS); 
            break;
         case GPIO_PIN6:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL6_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL6_OFS); 
            break;
         case GPIO_PIN7:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL7_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL7_OFS); 
            break;
         case GPIO_PIN8:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL8_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL8_OFS); 
            break;
         case GPIO_PIN9:
            p1->GPFSEL0 = (p1->GPFSEL0 & ~GPIO_GPFSEL0_FSEL9_MASK) |
               (func_number << GPIO_GPFSEL0_FSEL9_OFS); 
            break;
         case GPIO_PIN10:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL10_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL10_OFS);
            break;
         case GPIO_PIN11:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL11_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL11_OFS);
            break;
         case GPIO_PIN12:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL12_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL12_OFS);
            break;
         case GPIO_PIN13:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL13_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL13_OFS);
            break;
         case GPIO_PIN14:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL14_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL14_OFS);
            break;
         case GPIO_PIN15:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL15_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL15_OFS);
            break;
         case GPIO_PIN16:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL16_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL16_OFS);
            break;
         case GPIO_PIN17:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL17_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL17_OFS);
            break;
         case GPIO_PIN18:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL18_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL18_OFS);
            break;
         case GPIO_PIN19:
            p1->GPFSEL1 = (p1->GPFSEL1 & ~GPIO_GPFSEL1_FSEL19_MASK) |
               (func_number << GPIO_GPFSEL1_FSEL19_OFS);
            break;
         case GPIO_PIN20:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL20_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL20_OFS);
            break;
         case GPIO_PIN21:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL21_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL21_OFS);
            break;
         case GPIO_PIN22:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL22_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL22_OFS);
            break;
         case GPIO_PIN23:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL23_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL23_OFS);
            break;
         case GPIO_PIN24:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL24_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL24_OFS);
            break;
         case GPIO_PIN25:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL25_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL25_OFS);
            break;
         case GPIO_PIN26:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL26_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL26_OFS);
            break;
         case GPIO_PIN27:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL27_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL27_OFS);
            break;
         case GPIO_PIN28:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL28_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL28_OFS);
            break;
         case GPIO_PIN29:
            p1->GPFSEL2 = (p1->GPFSEL2 & ~GPIO_GPFSEL2_FSEL29_MASK) |
               (func_number << GPIO_GPFSEL2_FSEL29_OFS);
            break;
         case GPIO_PIN30:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL30_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL30_OFS);
            break;
         case GPIO_PIN31:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL31_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL31_OFS);
            break;
         case GPIO_PIN32:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL32_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL32_OFS);
            break;
         case GPIO_PIN33:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL33_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL33_OFS);
            break;
         case GPIO_PIN34:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL34_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL34_OFS);
            break;
         case GPIO_PIN35:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL35_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL35_OFS);
            break;
         case GPIO_PIN36:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL36_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL36_OFS);
            break;
         case GPIO_PIN37:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL37_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL37_OFS);
            break;
         case GPIO_PIN38:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL38_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL38_OFS);
            break;
         case GPIO_PIN39:
            p1->GPFSEL3 = (p1->GPFSEL3 & ~GPIO_GPFSEL3_FSEL39_MASK) |
               (func_number << GPIO_GPFSEL3_FSEL39_OFS);
            break;
         case GPIO_PIN40:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL40_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL40_OFS);
            break;
         case GPIO_PIN41:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL41_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL41_OFS);
            break;
         case GPIO_PIN42:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL42_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL42_OFS);
            break;
         case GPIO_PIN43:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL43_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL43_OFS);
            break;
         case GPIO_PIN44:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL44_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL44_OFS);
            break;
         case GPIO_PIN45:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL45_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL45_OFS);
            break;
         case GPIO_PIN46:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL46_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL46_OFS);
            break;
         case GPIO_PIN47:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL47_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL47_OFS);
            break;
         case GPIO_PIN48:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL48_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL48_OFS);
            break;
         case GPIO_PIN49:
            p1->GPFSEL4 = (p1->GPFSEL4 & ~GPIO_GPFSEL4_FSEL49_MASK) |
               (func_number << GPIO_GPFSEL4_FSEL49_OFS);
            break;
         case GPIO_PIN50:
            p1->GPFSEL5 = (p1->GPFSEL5 & ~GPIO_GPFSEL5_FSEL50_MASK) |
               (func_number << GPIO_GPFSEL5_FSEL50_OFS);
            break;
         case GPIO_PIN51:
            p1->GPFSEL5 = (p1->GPFSEL5 & ~GPIO_GPFSEL5_FSEL51_MASK) |
               (func_number << GPIO_GPFSEL5_FSEL51_OFS);
            break;
         case GPIO_PIN52:
            p1->GPFSEL5 = (p1->GPFSEL5 & ~GPIO_GPFSEL5_FSEL52_MASK) |
               (func_number << GPIO_GPFSEL5_FSEL52_OFS);
            break;
         case GPIO_PIN53:
            p1->GPFSEL5 = (p1->GPFSEL5 & ~GPIO_GPFSEL5_FSEL53_MASK) |
               (func_number << GPIO_GPFSEL5_FSEL53_OFS);
            break;
         default:
            printf("Invalid pin number %d\n", pin_number);
            break;
      }
      printf("GPFSEL0 = 0x%X\n", p1->GPFSEL0);
      printf("GPFSEL1 = 0x%X\n", p1->GPFSEL1);
      printf("GPFSEL2 = 0x%X\n", p1->GPFSEL2);
      printf("GPFSEL3 = 0x%X\n", p1->GPFSEL3);
      printf("GPFSEL4 = 0x%X\n", p1->GPFSEL4);
      printf("GPFSEL5 = 0x%X\n", p1->GPFSEL5);
   }
}
