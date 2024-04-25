#include "msp.h"
#include "periph.h"

/**
 * main.c
 */

void TA2_N_IRQHandler(void)
{
    //asm("   cpsid if");

    TIMER_A2->CCR[1] += 24;
    TIMER_A2->CCR[2] += 24;
    TIMER_A2->CCR[3] += 24;
    TIMER_A2->CCR[4] += 12;

    while(TIMER_A2->R < 62);

    TIMER_A2->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A2->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A2->CCTL[3] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A2->CCTL[4] &= ~TIMER_A_CCTLN_CCIFG;

    TIMER_A2->CCR[1] = 21;//42;
    TIMER_A2->CCR[2] = 29;//58;
    TIMER_A2->CCR[3] = 37;//74;
    TIMER_A2->CCR[4] = 50;//99;

    //TIMER_A2->CCTL[1] &= ~TIMER_A_CCTLN_CCIE;
    //TIMER_A2->CCTL[2] &= ~TIMER_A_CCTLN_CCIE;
    //TIMER_A2->CCTL[3] &= ~TIMER_A_CCTLN_CCIE;
    //TIMER_A2->CCTL[4] &= ~TIMER_A_CCTLN_CCIE;
    //asm("   cpsie if");

}
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	while(PCM->CTL1 & PCM_CTL1_PMR_BUSY);
	PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR__AM_LDO_VCORE1;
    while(PCM->CTL1 & PCM_CTL1_PMR_BUSY);
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK))
                       | FLCTL_BANK0_RDCTL_WAIT_1;
               FLCTL->BANK1_RDCTL  = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK1_RDCTL_WAIT_MASK))
                       | FLCTL_BANK1_RDCTL_WAIT_1;

	configure_hfxtclk();
	initialize_timer2();
	init_pins();
	start_timer2();

	__enable_interrupt();
	while(1);
}
