#include "LPIT.h"
#include "lpit1.h"

/* LPIT channel used */
#define LPIT_CHANNEL        0UL
#define LPIT_Channel_IRQn   LPIT0_IRQn

void LPIT_ISR(void)
{
  if (LPIT_DRV_GetInterruptFlagTimerChannels(INST_LPIT1,(1 << LPIT_CHANNEL)))
  {
    /* Clear LPIT channel flag */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL));
  }
  if (LPIT_DRV_GetInterruptFlagTimerChannels(INST_LPIT1,(1 << (LPIT_CHANNEL+1))))
  {
    /* Clear LPIT channel flag */
    LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1 << (LPIT_CHANNEL+1)));
    //Lpit_Timer_50ms ++;
  }
}


void LPIT_Initial(void)
{
    /* Initialize LPIT instance 0
     *  -   Reset and enable peripheral
     */
    LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
    /* Initialize LPIT channel 0 and configure it as a periodic counter
     * which is used to generate an interrupt every second.
     */
    LPIT_DRV_InitChannel(INST_LPIT1, LPIT_CHANNEL, &lpit1_ChnConfig0);
	LPIT_DRV_InitChannel(INST_LPIT1, (LPIT_CHANNEL + 1), &lpit1_ChnConfig1);

    /* Install LPIT_ISR as LPIT interrupt handler */
    INT_SYS_InstallHandler(LPIT_Channel_IRQn, &LPIT_ISR, (isr_t *)0);

    /* Start LPIT0 channel 0 counter */
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL));
    LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << (LPIT_CHANNEL + 1)));
}
