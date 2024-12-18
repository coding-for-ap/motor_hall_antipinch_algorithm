#ifndef _LPIT_H_
#define _LPIT_H_

typedef void (*TIMER_CALLBACK_T) (void);
extern void Task_RegisterFunction(TIMER_CALLBACK_T p);

extern void LPIT_Initial(void);

#endif
