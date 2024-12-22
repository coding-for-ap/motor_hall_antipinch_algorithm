/*
 *  task.c
 *
 *  created on: 2022/04/23
 *  author:
 */

#include "task.h"
#include "scheduler.h"
#include "io.h"
#include "lpit.h"
#include "tmr.h"
#include "ftm.h"
#include "adc.h"
#include "iic.h"
#include "spi.h"
#include "can.h"

/*
*  task_create
*
*  created on: 2022/04/23
*  author:
*/
void task_create(void)
{
  task_initial();
  task_add(task_2ms, 0, TASK_PERIOD_2MS);
  task_add(task_5ms, 0, TASK_PERIOD_5MS);
  task_add(task_10ms, 0, TASK_PERIOD_10MS);
  task_add(task_500ms, 0, TASK_PERIOD_500MS);
}

/*
*  task_2ms
*
*  created on: 2022/04/23
*  author:
*/
void task_2ms(void)
{

}

/*
*  task_5ms
*
*  created on: 2022/04/23
*  author:
*/
void task_5ms(void)
{

}

/*
*  task_10ms
*
*  created on: 2022/04/23
*  author:
*/
void task_10ms(void)
{

}

/*
*  task_500ms
*
*  created on: 2022/04/23
*  author:
*/
void task_500ms(void)
{
  IO_ToggleLed();
}

/*
 * end of line
 */
