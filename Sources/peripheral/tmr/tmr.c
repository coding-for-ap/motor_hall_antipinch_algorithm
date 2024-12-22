#include "tmr.h"
#include "lpTmr1.h"

/*!
 * @brief: LPMR interrupt handler.
 *         When an interrupt occurs clear channel flag and toggle LED_GREEN
 */
void LPTMR_ISR(void)
{
    /* Clear compare flag */
    LPTMR_DRV_ClearCompareFlag(INST_LPTMR1);
}

void TMR_Initial(void)
{
    /* Init LPTMR as timer
     *  -   interrupt after 1 second
     *  -   SIRC as clock source
     *  -   counter disabled
     */
    LPTMR_DRV_Init(INST_LPTMR1, &lpTmr1_config0, false);

    /* Install IRQ handler for LPTMR interrupt */
    INT_SYS_InstallHandler(LPTMR0_IRQn, &LPTMR_ISR, (isr_t *)0);
    /* Enable IRQ for LPTMR */
    INT_SYS_EnableIRQ(LPTMR0_IRQn);

    /* Start LPTMR counter */
    LPTMR_DRV_StartCounter(INST_LPTMR1);
}

