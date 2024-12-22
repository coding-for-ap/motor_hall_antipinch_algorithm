/******************************************************************************/
/*
* file        SWMT.c
* brief       switch control motor simple demo
* Version     Author      Date            Desc   
* 1.0         xxx         2020/04/23      Initial version
*/
/******************************************************************************/

#include "SW.h"

#if 0
/* from adc module */
uint16_t sw_adc_raw_value;

/* define switch level */
#define SW_CMD_OFF                (0)
#define SW_CMD_UP                 (1)
#define SW_CMD_AUTO_UP            (2)
#define SW_CMD_DOWN               (3)
#define SW_CMD_AUTO_DOWN          (4)
#define SW_CMD_NUM                (5)

/* each switch level adc val  */
#define SW_ADC_VAL_OffSet         (100u)
#define SW_ADC_VAL_OFF            (4090u)
#define SW_ADC_VAL_DOWN           (1870u)
#define SW_ADC_VAL_AUTO_DOWN      (980u)
#define SW_ADC_VAL_UP             (460u)
#define SW_ADC_VAL_AUTO_UP        (165u)

/*
*  sw_adc_val_read
*
*  created on: 2022/04/23
*  author:
*/
uint8_t sw_adc_val_read(void)
{
  uint8_t ret = 0;
  uint16_t adc_val = 0;
  
  adc_val = sw_adc_raw_value;
  
  if((adc_val > (SW_ADC_VAL_OFF - SW_ADC_VAL_OffSet)) && (adc_val < (SW_ADC_VAL_OFF + SW_ADC_VAL_OffSet))) ret = SW_CMD_OFF;
  else if((adc_val > (SW_ADC_VAL_UP - SW_ADC_VAL_OffSet)) && (adc_val < (SW_ADC_VAL_UP + SW_ADC_VAL_OffSet))) ret = SW_CMD_UP;
  else if((adc_val > (SW_ADC_VAL_AUTO_UP - SW_ADC_VAL_OffSet)) && (adc_val < (SW_ADC_VAL_AUTO_UP + SW_ADC_VAL_OffSet))) ret = SW_CMD_AUTO_UP;
  else if((adc_val > (SW_ADC_VAL_DOWN - SW_ADC_VAL_OffSet)) && (adc_val < (SW_ADC_VAL_DOWN + SW_ADC_VAL_OffSet))) ret = SW_CMD_DOWN;
  else if((adc_val > (SW_ADC_VAL_AUTO_DOWN - SW_ADC_VAL_OffSet)) && (adc_val < (SW_ADC_VAL_AUTO_DOWN + SW_ADC_VAL_OffSet))) ret = SW_CMD_AUTO_DOWN;

  return ret;
}

/*
*  swmt_task_5ms
*
*  created on: 2022/04/23
*  author:
*/
void swmt_task_5ms(void)
{
  uint8_t switch_cmd = SW_CMD_OFF;
  static uint8_t pre_switch_cmd = SW_CMD_OFF;
  
  switch_cmd = sw_adc_val_read();
  
  if(switch_cmd != pre_switch_cmd)
  {
    if(switch_cmd == SW_CMD_UP || switch_cmd == SW_CMD_AUTO_UP)
    {
      /* place the motor hardware output here */

      SEGGER_RTT_printf(0, "sw cmd Up\r\n");
    }
    else if(switch_cmd == SW_CMD_DOWN || switch_cmd == SW_CMD_AUTO_DOWN)
    {
	  /* place the motor hardware output here */

      SEGGER_RTT_printf(0, "sw cmd Down\r\n");
    }
    else if(switch_cmd == SW_CMD_OFF)
    {
      /* place the motor hardware output here */
	
      SEGGER_RTT_printf(0, "sw cmd Stop\r\n");
    }
    else
    {
    
    }
  }
  pre_switch_cmd = switch_cmd;
}

/*
 * end of line
 */

#endif

