#include "cm.h"
#include "gpio.h"
#include "armctl.h"
#include "armtimer.h"
#include "dac.h"
#include "isp.h"
#include "aux.h"
#include "unistd.h"

volatile unsigned int *ugpio = 0;
volatile unsigned int *upads = 0;
volatile unsigned int *ucm = 0;
volatile unsigned int *uarmctl = 0;
volatile unsigned int *uarmtimer = 0;
volatile unsigned int *uisp = 0;
volatile unsigned int *uaux = 0;

#define P1 ((GPIO_Type *) ugpio)
#define PADS ((GPIO_Pads_Control_Type *) upads)
#define CM ((CM_Type *) ucm)
#define ARMCTL ((ARM_Control_Logic_Module_Type *) uarmctl)
#define ARMTIMER ((ARMTimer_Type *) uarmtimer)
#define ISP ((ISP_Type *) uisp)
#define AUX ((AUX_Type *) uaux)

int main(int argc, char *argv[])
{
   int i;
   /* Gain access to GPIO registers */
   gpio_init(&ugpio);
   /* Gain access to GPIO drive control registers */
   pads_init(&upads);
   /* Gain access to Clock Manager registers */
   cm_init(&ucm);
   /* Gain access to ARM Control Logic Module registers */
   armctl_init(&uarmctl);
   /* Gain access to ARM Timer registers */
   armtimer_init(&uarmtimer);
   isp_init(&uisp);
   aux_init(&uaux);

   gpio_print(P1);
   pads_print(PADS);
   cm_print(CM);
   armctl_print(ARMCTL);
   armtimer_print(ARMTIMER);
   isp_print(ISP);
   aux_print(AUX);
   
   gpio_set_func(GPIO_PIN14, GPIO_FUNC_ALT5, P1);
   gpio_set_func(GPIO_PIN15, GPIO_FUNC_ALT5, P1);
   P1->GPPUD = 0;
   //for(i = 0; i < 1500000; i++);
   nanosleep(100000000);
   P1->GPPUDCLK0 = GPIO_GPPUDCLK014 | GPIO_GPPUDCLK015;
   //for(i = 0; i < 1500000; i++);
   nanosleep(100000000);
   P1->GPPUDCLK0 = 0;
   AUX->AUX_ENABLES = 1;
   AUX->AUX_MU_IER_REG = 0;
   AUX->AUX_MU_CNTL_REG = 0;
   AUX->AUX_MU_LCR_REG = 3;
   AUX->AUX_MU_MCR_REG = 0;
   AUX->AUX_MU_IIR_REG = 0xC6;
   AUX->AUX_MU_BAUD_REG = ((400000000/115200)/8)-1;
   AUX->AUX_MU_LCR_REG = 0x03;
   AUX->AUX_MU_CNTL_REG = 3;
   AUX->AUX_MU_IO_REG = 'I';
   while(1)
   {
       
   }


   /* Set GPIO4 to ALT0 (GPIOCLK0) */
   gpio_set_func(GPIO_PIN20, GPIO_FUNC_ALT5, P1);

   /* DAC D6 */ 
   gpio_set_func(GPIO_PIN0, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR00;
   /* DAC D7 */
   gpio_set_func(GPIO_PIN1, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR01;
   /* DAC D8 */
   gpio_set_func(GPIO_PIN2, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR02;
   /* DAC D9 */
   gpio_set_func(GPIO_PIN3, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR03;
   /* DAC D10 */
   gpio_set_func(GPIO_PIN4, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR04;
   /* DAC D11 */
   gpio_set_func(GPIO_PIN5, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR05;
   /* DAC D12 */
   gpio_set_func(GPIO_PIN6, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR06;
   /* DAC D13 (MSB) */
   gpio_set_func(GPIO_PIN7, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR07;
   gpio_set_func(GPIO_PIN17, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR017;
   gpio_set_func(GPIO_PIN27, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR027;
   gpio_set_func(GPIO_PIN22, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR022;
   gpio_set_func(GPIO_PIN10, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR010;
   gpio_set_func(GPIO_PIN9, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR09;
   gpio_set_func(GPIO_PIN11, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR011;
   gpio_set_func(GPIO_PIN13, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR013;
   gpio_set_func(GPIO_PIN19, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR019;
   gpio_set_func(GPIO_PIN21, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR021;
   gpio_set_func(GPIO_PIN20, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR020;
   gpio_set_func(GPIO_PIN16, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR016;
   gpio_set_func(GPIO_PIN12, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR012;
   gpio_set_func(GPIO_PIN25, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR025;
   gpio_set_func(GPIO_PIN24, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR024;
   gpio_set_func(GPIO_PIN23, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR023;
   gpio_set_func(GPIO_PIN18, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR018;
   gpio_set_func(GPIO_PIN15, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR015;
   gpio_set_func(GPIO_PIN14, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR014;

   /* Delay measurement probe */
   gpio_set_func(GPIO_PIN26, GPIO_FUNC_OUTPUT, P1);
   P1->GPCLR0 |= GPIO_GPCLR026;

   /*cm_gpclk0_29_5_MHz(CM);*/
   printf("GPLEV0 = 0x%X\n", P1->GPLEV0);

   armtimer_disable(ARMTIMER);
   /*armtimer_enable(armtimer_load_value, ARMTIMER);*/
   while(1)
   {
      if(!(ARMTIMER->IRQ_RAW))
      {
         continue;
      }
      else
      {
         P1->GPSET0 = (1 << GPIO_PIN26);
         P1->GPCLR0 = (1 << GPIO_PIN26);
      }
   }
   /*set_pads_0_27_moderate_drive_strength();*/
   return 0;
}
