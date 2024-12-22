/*
 *  task.h
 *
 *  created on: 2022/04/23
 *  author:
 */

#ifndef _TASK_H_
#define _TASK_H_
#include "stdint.h"

#define TASK_PERIOD_2MS     (2u)
#define TASK_PERIOD_5MS     (5u)
#define TASK_PERIOD_10MS    (10u)
#define TASK_PERIOD_500MS   (500u)

extern void task_2ms(void);
extern void task_5ms(void);
extern void task_10ms(void);
extern void task_500ms(void);
extern void task_create(void);

#endif

/*
 * end of line
 */
