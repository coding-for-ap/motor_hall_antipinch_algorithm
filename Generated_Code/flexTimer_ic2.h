/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : flexTimer_ic2.h
**     Project     : HALL_Ap_Sys
**     Processor   : S32K118_48
**     Component   : ftm_ic
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2024-12-21, 23:49, # CodeGen: 27
**     Contents    :
**         FTM_DRV_Init                            - status_t FTM_DRV_Init(uint32_t instance,const ftm_user_config_t * info,...
**         FTM_DRV_Deinit                          - status_t FTM_DRV_Deinit(uint32_t instance);
**         FTM_DRV_MaskOutputChannels              - status_t FTM_DRV_MaskOutputChannels(uint32_t instance, uint32_t channelsMask,...
**         FTM_DRV_SetInitialCounterValue          - status_t FTM_DRV_SetInitialCounterValue(uint32_t instance, uint16_t...
**         FTM_DRV_SetHalfCycleReloadPoint         - status_t FTM_DRV_SetHalfCycleReloadPoint(uint32_t instance, uint16_t...
**         FTM_DRV_SetSoftOutChnValue              - status_t FTM_DRV_SetSoftOutChnValue(uint32_t instance, uint8_t...
**         FTM_DRV_SetSoftwareOutputChannelControl - status_t FTM_DRV_SetSoftwareOutputChannelControl(uint32_t instance, uint8_t...
**         FTM_DRV_SetInvertingControl             - status_t FTM_DRV_SetInvertingControl(uint32_t instance, uint8_t...
**         FTM_DRV_SetModuloCounterValue           - status_t FTM_DRV_SetModuloCounterValue(uint32_t instance, uint16_t...
**         FTM_DRV_SetSync                         - status_t FTM_DRV_SetSync(uint32_t instance, const ftm_pwm_sync_t *param);
**         FTM_DRV_InitInputCapture                - status_t FTM_DRV_InitInputCapture(uint32_t instance,const ftm_input_param_t *...
**         FTM_DRV_DeinitInputCapture              - status_t FTM_DRV_DeinitInputCapture(uint32_t instance,const ftm_input_param_t...
**         FTM_DRV_GetInputCaptureMeasurement      - uint16_t FTM_DRV_GetInputCaptureMeasurement(uint32_t instance,uint8_t channel);
**         FTM_DRV_StartNewSignalMeasurement       - status_t FTM_DRV_StartNewSignalMeasurement(uint32_t instance,uint8_t channel);
**         FTM_DRV_GetFrequency                    - uint32_t FTM_DRV_GetFrequency(uint32_t instance);
**         FTM_DRV_ConvertFreqToPeriodTicks        - uint16_t FTM_DRV_ConvertFreqToPeriodTicks(uint32_t instance,uint32_t...
**
**     Copyright 1997 - 2015 Freescale Semiconductor, Inc. 
**     Copyright 2016-2017 NXP 
**     All Rights Reserved.
**     
**     THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
**     IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
**     OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
**     IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
**     INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
**     SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
**     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
**     STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
**     IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
**     THE POSSIBILITY OF SUCH DAMAGE.
** ###################################################################*/
/*!
** @file flexTimer_ic2.h
** @version 01.00
*/         
/*!
**  @addtogroup flexTimer_ic2_module flexTimer_ic2 module documentation
**  @{
*/         
#ifndef flexTimer_ic2_H
#define flexTimer_ic2_H

/* MODULE flexTimer_ic2.
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * The global macro will be used in function call of the module.
 */

/* Include inherited beans */
#include "clockMan1.h"
#include "Cpu.h"
/*! @brief Device instance number */
#define INST_FLEXTIMER_IC2 1U

/* Channels configuration structure for flexTimer_ic2* input capture channels */
extern ftm_input_ch_param_t flexTimer_ic2_InputCaptureChannelConfig[1];
/* Channels configuration structure for flexTimer_ic2 input capture */
extern ftm_input_param_t flexTimer_ic2_InputCaptureConfig;

/* Global configuration of flexTimer_ic2 */
extern ftm_user_config_t  flexTimer_ic2_InitConfig;

#endif
/* ifndef flexTimer_ic2_H */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the Freescale S32K series of microcontrollers.
**
** ###################################################################
*/
