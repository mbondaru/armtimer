/*
 * periph.c
 *
 *  Created on: 16 апр. 2024 г.
 *      Author: bonda_000
 */

#include "periph.h"

void configure_hfxtclk()
{
    uint32_t status = SYSCTL->NMI_CTLSTAT & SYSCTL_NMI_CTLSTAT_CS_SRC;

    PJ->SEL1 &= ~BIT3;
    PJ->SEL0 |= BIT3;
    PJ->DIR &= ~BIT2;
    PJ->SEL1 &= ~BIT2;
    PJ->SEL0 &= ~BIT2;

    CS->KEY = CS_KEY_VAL;

    SYSCTL->NMI_CTLSTAT &= ~(SYSCTL_NMI_CTLSTAT_CS_SRC);

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXTFREQ_OFS) = 1;

    CS->CTL2 = (CS->CTL2 & (~CS_CTL2_HFXTFREQ_MASK)) | CS_CTL2_HFXTFREQ_4;

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXTBYPASS_OFS) = 1;

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXT_EN_OFS) = 1;

    while (BITBAND_PERI(CS->IFG, CS_IFG_HFXTIFG_OFS))
    {
        BITBAND_PERI(CS->CLRIFG, CS_CLRIFG_CLR_HFXTIFG_OFS) = 1;
    }

    SYSCTL->NMI_CTLSTAT |= status;


    //CS->CLRIFG = CS_CLRIFG_CLR_HFXTIFG;
    //CS->CTL2 = CS_CTL2_HFXTBYPASS;

    CS->CTL1 = (CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_SELS_MASK)) | CS_CTL1_SELM__HFXTCLK | CS_CTL1_SELS__HFXTCLK | CS_CTL1_DIVS__2;
    P4->DIR |= BIT3;
    P4->SEL1 &= ~BIT3;
    P4->SEL0 |= BIT3;
}
void start_timer2()
{
    TIMER_A2->CTL |= TIMER_A_CTL_MC_1;
}
void stop_timer2()
{
    TIMER_A2->CTL &= ~TIMER_A_CTL_MC_MASK;
}
void reset_timer2()
{
    TIMER_A2->R = 0;
}
void initialize_timer2()
{
    TIMER_A2->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP;

    TIMER_A2->CCR[0] = 472;//472;
    TIMER_A2->CCR[1] = 21;//21;
    TIMER_A2->CCR[2] = 29;//29;
    TIMER_A2->CCR[3] = 37;//37;
    TIMER_A2->CCR[4] = 50;//50;
    //TIMER_A2->CCTL[0] = TIMER_A_CCTLN_CCIE;
    TIMER_A2->CCTL[1] = TIMER_A_CCTLN_CCIE | TIMER_A_CCTLN_OUTMOD_2;
    TIMER_A2->CCTL[2] = /*TIMER_A_CCTLN_CCIE |*/ TIMER_A_CCTLN_OUTMOD_6;
    TIMER_A2->CCTL[3] = /*TIMER_A_CCTLN_CCIE |*/ TIMER_A_CCTLN_OUTMOD_2;
    TIMER_A2->CCTL[4] = /*TIMER_A_CCTLN_CCIE |*/ TIMER_A_CCTLN_OUTMOD_2;
    NVIC->ISER[0] = //(1 << TA2_0_IRQn) |
            (1 << TA2_N_IRQn);
}
void init_pins()
{
    P5->DIR |= BIT6 | BIT7;
    P5->SEL1 &= ~(BIT6 | BIT7);
    P5->SEL0 |= BIT6 | BIT7;

    P6->DIR |= BIT6 | BIT7;
    P6->SEL1 &= ~(BIT6 | BIT7);
    P6->SEL0 |= BIT6 | BIT7;
}


