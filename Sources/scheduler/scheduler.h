/*
 *  scheduler.h
 *
 *  created on: 2022/04/23
 *  author: 
 */

#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_
#include "stdint.h"

struct SCH_t
{
  void (*pTask)(void);
  uint32_t delay;
  uint32_t period;
  uint8_t runMe;
};

extern void task_dispatch(void);
extern void task_add(void (*pTask_)(void), uint32_t delay_, uint32_t period_);
extern void task_delete(uint8_t taskId);
extern void task_update(void);
extern void task_initial(void);

#endif

/*
 * end of line
 */
