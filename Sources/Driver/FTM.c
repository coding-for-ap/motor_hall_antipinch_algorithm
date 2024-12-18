#include "FTM.h"
#include "flexTimer_ic1.h"
#include "flexTimer_ic2.h"

void FTM_Initial(void)
{
    ftm_state_t ftm1StateStruct;
    ftm_state_t ftm2StateStruct;

    /* Initialize FTM instances, PWM and Input capture
     *  -   See ftm component for more details
     */
    FTM_DRV_Init(INST_FLEXTIMER_IC1, &flexTimer_ic1_InitConfig, &ftm1StateStruct);
    FTM_DRV_Init(INST_FLEXTIMER_IC2, &flexTimer_ic2_InitConfig, &ftm2StateStruct);

    /* Setup input capture for FTM 1 channel 0  - PTA12 */
    FTM_DRV_InitInputCapture(INST_FLEXTIMER_IC1, (ftm_input_param_t *)&flexTimer_ic1_InputCaptureChannelConfig);
    FTM_DRV_InitInputCapture(INST_FLEXTIMER_IC2, (ftm_input_param_t *)&flexTimer_ic2_InputCaptureChannelConfig);
}

void FTM_UpdatePwmChannel(void)
{
#if 0
    uint16_t inputCaptureMeas_1_2 = 0;
    uint16_t inputCaptureMeas_1_3 = 0;
    uint16_t inputCaptureMeas_1_6 = 0;
    uint16_t inputCaptureMeas_2_1 = 0;

    /* Get values */
    inputCaptureMeas_1_2 = FTM_DRV_GetInputCaptureMeasurement(INST_FLEXTIMER_IC1, 2);
    inputCaptureMeas_1_3 = FTM_DRV_GetInputCaptureMeasurement(INST_FLEXTIMER_IC1, 3);
    inputCaptureMeas_1_6 = FTM_DRV_GetInputCaptureMeasurement(INST_FLEXTIMER_IC1, 6);
    inputCaptureMeas_2_1 = FTM_DRV_GetInputCaptureMeasurement(INST_FLEXTIMER_IC2, 1);
#endif
    //SEGGER_RTT_printf(0, "ftm ic value %d - %d - %d - %d\r\n", inputCaptureMeas_1_2, inputCaptureMeas_1_3, inputCaptureMeas_1_6, inputCaptureMeas_2_1);
}
