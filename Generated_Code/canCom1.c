/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : canCom1.c
**     Project     : HALL_Ap_Sys
**     Processor   : S32K118_48
**     Component   : flexcan
**     Version     : Component SDK_S32K1xx_15, Driver 01.00, CPU db: 3.00.000
**     Repository  : SDK_S32K1xx_15
**     Compiler    : GNU C Compiler
**     Date/Time   : 2024-12-18, 22:10, # CodeGen: 25
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
** @file canCom1.c
** @version 01.00
*/         
/*!
**  @addtogroup canCom1_module canCom1 module documentation
**  @{
*/         

/* MODULE canCom1.
 *
 * @page misra_violations MISRA-C:2012 violations
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 8.7, External variable could be made static.
 * The external variables will be used in other source files that user initialize
 * to use this module.
 *
 * @section [global]
 * Violates MISRA 2012 Advisory Rule 2.5, Global macro not referenced.
 * This is required to enable the use of a macro needed by the application (even
 * if the macro is not used inside the driver code).
 */

#include "canCom1.h"

flexcan_state_t canCom1_State; 

const flexcan_user_config_t canCom1_InitConfig0 = {
    .fd_enable = false,
    .pe_clock = FLEXCAN_CLK_SOURCE_PERIPH,
    .max_num_mb = 16,
    .num_id_filters = FLEXCAN_RX_FIFO_ID_FILTERS_8,
    .is_rx_fifo_needed = false,
    .flexcanMode = FLEXCAN_NORMAL_MODE,
    .payload = FLEXCAN_PAYLOAD_SIZE_8,
    .bitrate = {
        .propSeg = 7,
        .phaseSeg1 = 4,
        .phaseSeg2 = 1,
        .preDivider = 5,
        .rJumpwidth = 1
    },
    .bitrate_cbt = {
        .propSeg = 7,
        .phaseSeg1 = 4,
        .phaseSeg2 = 1,
        .preDivider = 5,
        .rJumpwidth = 1
    },
    .transfer_type = FLEXCAN_RXFIFO_USING_INTERRUPTS,
    .rxFifoDMAChannel = 0U
};


/* END canCom1. */

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


