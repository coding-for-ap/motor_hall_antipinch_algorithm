#include "IO.h"
#include "pin_mux.h"
#include "Cpu.h"

#define PORT_A	        PORTA
#define PT_A	        PTA
#define CAN_STB		    10U
#define NCV_EN_FL		11U
#define LED1		    13U

#define PORT_C	        PORTC
#define PT_C	        PTC
#define Vol_CTR		    5U
#define VS_PULL		    6U

#define PORT_E	        PORTE
#define PT_E	        PTE
#define NCV_PWM_FL		9U

#define PORT_B	        PORTB
#define PT_B	        PTB
#define NCV_PWM_RL		13U

#define PORT_D	        PORTD
#define PT_D	        PTD
#define NCV_EN_RL		3U

#define HAL_MOTO_ID_FL    (0)
#define HAL_MOTO_ID_RL    (1)
#define HAL_MOTO_ID_FR    (2)
#define HAL_MOTO_ID_RR    (3)
#define HAL_MOTO_ID_MAX   (4)


void IO_Initial(void)
{
  /* Initialize pins
   *	-	See PinSettings component for more info
   */
  PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

  /* Output direction for LED0 & LED1 */
  PINS_DRV_SetPinsDirection(PT_A, (1 << LED1) | (1 << NCV_EN_FL) | (1 << CAN_STB));

  PINS_DRV_ClearPins(PT_A, 1 << LED1);
  PINS_DRV_ClearPins(PT_A, 1 << NCV_EN_FL);
  PINS_DRV_ClearPins(PT_A, 1 << CAN_STB);

  //PINS_DRV_ClearPins(PT_A, 1 << NCV_EN_FL);
  //PINS_DRV_SetPins(PT_A, 1 << NCV_EN_RL);

  PINS_DRV_SetPinsDirection(PT_C, (1 << Vol_CTR) | (1 << VS_PULL));

  PINS_DRV_SetPins(PT_C, 1 << Vol_CTR);
  PINS_DRV_SetPins(PT_C, 1 << VS_PULL);

  PINS_DRV_SetPinsDirection(PT_E, (1 << NCV_PWM_FL));
  PINS_DRV_SetPins(PT_E, 1 << NCV_PWM_FL);

  PINS_DRV_SetPinsDirection(PT_B, (1 << NCV_PWM_RL));
  PINS_DRV_SetPins(PT_B, 1 << NCV_PWM_RL);

  PINS_DRV_SetPinsDirection(PT_D, (1 << NCV_EN_RL));
  PINS_DRV_ClearPins(PT_D, 1 << NCV_EN_RL);
}

void IO_ToggleLed(void)
{
  PINS_DRV_TogglePins(PT_A, (1 << LED1));
}

