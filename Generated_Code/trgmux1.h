/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : trgmux1.h
**     Project     : HALL_Ap_Sys
**     Processor   : S32K118_48
**     Component   : trgmux
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2024-12-18, 22:10, # CodeGen: 25
**     Contents    :
**         TRGMUX_DRV_Init                         - status_t TRGMUX_DRV_Init(const uint32_t instance, const trgmux_user_config_t...
**         TRGMUX_DRV_Deinit                       - status_t TRGMUX_DRV_Deinit(const uint32_t instance);
**         TRGMUX_DRV_SetTrigSourceForTargetModule - status_t TRGMUX_DRV_SetTrigSourceForTargetModule(const uint32_t instance,...
**         TRGMUX_DRV_GetTrigSourceForTargetModule - trgmux_trigger_source_t TRGMUX_DRV_GetTrigSourceForTargetModule(const...
**         TRGMUX_DRV_SetLockForTargetModule       - void TRGMUX_DRV_SetLockForTargetModule(const uint32_t instance, const...
**         TRGMUX_DRV_GetLockForTargetModule       - bool TRGMUX_DRV_GetLockForTargetModule(const uint32_t instance, const...
**         TRGMUX_DRV_GenSWTrigger                 - void TRGMUX_DRV_GenSWTrigger(const uint32_t instance);
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
** @file trgmux1.h
** @version 01.00
*/         
/*!
**  @addtogroup trgmux1_module trgmux1 module documentation
**  @{
*/         

#ifndef trgmux1_H
#define trgmux1_H
/* MODULE trgmux1. */

/**
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, global macro not referenced
 * There are global macros defined to be used by the integrator and another one used as include guard.
 */

/* Include inherited beans */
#include "Cpu.h"


/*! @brief Device instance number */
#define INST_TRGMUX1 (0U)


/*! @brief Declaration of InOut Mapping configuration */
extern const trgmux_inout_mapping_config_t trgmux1_InOutMappingConfig0[1];

/*! @brief Configuration declaration */
extern const trgmux_user_config_t trgmux1_InitConfig0;


#endif /* trgmux1_H */
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

