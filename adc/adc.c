#include "cm.h"
#include "gpio.h"
#include "armctl.h"
#include "armtimer.h"
#include "dac.h"
#include "irqctrl.h"

volatile unsigned int *ugpio = 0;
volatile unsigned int *upads = 0;
volatile unsigned int *ucm = 0;
volatile unsigned int *uarmctl = 0;
volatile unsigned int *uarmtimer = 0;
volatile unsigned int *uirqctrl = 0;
volatile unsigned int *uvt = 0;

#define P1 ((GPIO_Type *) ugpio)
#define PADS ((GPIO_Pads_Control_Type *) upads)
#define CM ((CM_Type *) ucm)
#define ARMCTL ((ARM_Control_Logic_Module_Type *) uarmctl)
#define ARMTIMER ((ARMTimer_Type *) uarmtimer)
#define IRQCTRL ((Interrupt_Controller_Type *) uirqctrl)
#define VT ((Vector_Table_Type *) uvt)
/*
   @brief The undefined instruction interrupt handler

   If an undefined instruction is encountered, the CPU will start
   executing this function. Just trap here as a debug solution

*/
void __attribute((interrupt("UNDEF"))) undefined_instruction_vector(void)
{
   while(1)
   {
      /*do nothing*/
   }
}

void __attribute__((interrupt ("IRQ"))) hardware_interrupt_vector(void)
{

}
int main(int argc, char *argv[])
{
   /* APB (Core_clk) freq = 470560000 */
   uint32_t armtimer_load_value = 16;
   register int sine_index = 0;
   int i;
   uint32_t sine;
   uint8_t sine_comms[SINE_TABLE_SIZE];
   printf("Generating 360-entry sine table...\n");
   for(i = 0; i < SINE_TABLE_SIZE; i++)
   {
      sine = DAC_14BIT_0_5V +
         (float)(DAC_14BIT_0_5V*sin(PI*i/180));
      sine_comms[i] = (sine >> 6);
      printf("i: %d ", sine_comms[i]);
   }
   printf("\n");
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
   /* Gain access to IRQ register */
   irqctrl_init(&uirqctrl);
   vt_init(&uvt);

   gpio_print(P1);
   pads_print(PADS);
   cm_print(CM);
   armctl_print(ARMCTL);
   armtimer_print(ARMTIMER);
   irqctrl_print(IRQCTRL);
   vt_print(VT);

   /* Seemingly didn't do anything */
   /*set_coretimer_src_apb_clock(ARMCTL);*/
   /*Inspect the registers and stop */
   while(1);
   armtimer_set_predivider(0, ARMTIMER);

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

   /* Set GPIO20 to ALT5 (GPIOCLK0) */
   /*
   gpio_set_func(GPIO_PIN20, GPIO_FUNC_ALT5, P1);
   cm_gpclk0_29_5_MHz(CM);
   */
   printf("GPLEV0 = 0x%X\n", P1->GPLEV0);

   armtimer_disable(ARMTIMER);
   /*armtimer_enable(armtimer_load_value, ARMTIMER);*/
   while(1)
   {
      /*if(!(ARMTIMER->IRQ_RAW))
      {
         continue;
      }
      else
      {*/
         P1->GPSET0 = (1 << GPIO_PIN26);
         ARMTIMER->IRQ_CLR_ACK = CLEAR_IRQ;
         P1->GPCLR0 = 0xFF;
         P1->GPSET0 = sine_comms[sine_index];
         sine_index = (sine_index + 1) % SINE_TABLE_SIZE;
         P1->GPCLR0 = (1 << GPIO_PIN26);
   }
   /*set_pads_0_27_moderate_drive_strength();*/
   return 0;
}
