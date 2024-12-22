/*
 *  scheduler.c
 *
 *  created on: 2022/04/23
 *  author: 
 */

#include "scheduler.h"

#define TASKS_SCH_MAX    (4u)

struct SCH_t task_scheduler[TASKS_SCH_MAX];

/*
*  task_dispatch
*
*  created on: 2022/04/23
*  author:
*/
void task_dispatch(void)
{
  uint8_t index = 0;
  
  for(index = 0; index < TASKS_SCH_MAX; index++)
  {
    if(task_scheduler[index].runMe > 0)
    {
      (*(task_scheduler[index].pTask))();
      task_scheduler[index].runMe -= 1;
      
      if(task_scheduler[index].period == 0)
      {
        task_delete(index);
      }
    }
  }
}

/*
*  task_add
*
*  created on: 2022/04/23
*  author:
*/
void task_add(void (*pTask_)(void), uint32_t delay_, uint32_t period_)
{
  uint8_t index = 0;
  
  while((task_scheduler[index].pTask != 0) && index < TASKS_SCH_MAX)
  {
    index++;
  }
  
  if(index == TASKS_SCH_MAX)
  {
    //return TASKS_SCH_MAX;
  }
  
  task_scheduler[index].pTask = pTask_;
  task_scheduler[index].delay = delay_;
  task_scheduler[index].period = period_;
  task_scheduler[index].runMe = 0;
}

/*
*  task_delete
*
*  created on: 2022/04/23
*  author:
*/
void task_delete(uint8_t taskId)
{
  if(taskId < TASKS_SCH_MAX)
  {
    task_scheduler[taskId].pTask = 0;
    task_scheduler[taskId].delay = 0;
    task_scheduler[taskId].period = 0;
    task_scheduler[taskId].runMe = 0;
  }
}

/*
*  task_update
*
*  created on: 2022/04/23
*  author:
*/
void task_update(void)
{
  uint8_t index = 0;
  
  for(index = 0; index < TASKS_SCH_MAX; index++)
  {
    if(task_scheduler[index].pTask)
    {
      if(task_scheduler[index].delay == 0)
      {
        task_scheduler[index].runMe += 1;
        
        if(task_scheduler[index].period)
        {
          task_scheduler[index].delay = task_scheduler[index].period - 1;
        }
      }
      else
      {
        task_scheduler[index].delay -= 1;
      }
    }
  }
}

/*
*  task_initial
*
*  created on: 2022/04/23
*  author:
*/
void task_initial(void)
{
  uint8_t i = 0;
  
  for(i = 0; i < TASKS_SCH_MAX; i++)
  {
    task_scheduler[i].pTask = 0;
    task_scheduler[i].delay = 0;
    task_scheduler[i].period = 0;
    task_scheduler[i].runMe = 0;
  }
}

/*
 * end of line
 */
