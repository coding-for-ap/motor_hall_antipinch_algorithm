/* ###################################################################
**     This component module is generated by Processor Expert. Do not modify it.
**     Filename    : Cpu.h
**     Project     : HALL_Ap_Sys
**     Processor   : S32K118_48
**     Component   : S32K118_48
**     Version     : Component 01.197, Driver 01.00, CPU db: 3.00.000
**     Datasheet   : S32K1XXRM Rev. 6, 12/2017
**     Compiler    : GNU C Compiler
**     Date/Time   : 2024-12-22, 12:44, # CodeGen: 29
**     Abstract    :
**
**     Settings    :
**
**     Contents    :
**         SystemInit            - void SystemInit(void);
**         SystemCoreClockUpdate - void SystemCoreClockUpdate(void);
**         SystemSoftwareReset   - void SystemSoftwareReset(void);
**
**     (c) Freescale Semiconductor, Inc.
**     2004 All Rights Reserved
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
** @file Cpu.h
** @version 01.00
** @brief
**
*/         
/*!
**  @addtogroup Cpu_module Cpu module documentation
**  @{
*/         

#ifndef Cpu_H
#define Cpu_H


/* MODULE Cpu. */


/*Include shared modules, which are used for whole project*/
#include "device_registers.h"

#include "interrupt_manager.h"
#include "clock.h"
#include "lpit_driver.h"
#include "edma_driver.h"
#include "osif.h"
#include "lpspi_master_driver.h"
#include "lpspi_slave_driver.h"
#include "lpspi_shared_function.h"
#include "adc_driver.h"
#include "pdb_driver.h"
#include "trgmux_driver.h"
#include "lpi2c_driver.h"
#include "flexcan_driver.h"
#include "cmp_driver.h"
#include "ftm_ic_driver.h"
#include "lptmr_driver.h"
#include "system_S32K118.h"

/* Including needed modules to compile this module/procedure */
#include "clockMan1.h"
#include "pin_mux.h"
#include "lpit1.h"
#include "lpspiCom1.h"
#include "dmaController1.h"
#include "adConv1.h"
#include "pdly1.h"
#include "trgmux1.h"
#include "lpi2c1.h"
#include "canCom1.h"
#include "comparator1.h"
#include "flexTimer_ic1.h"
#include "flexTimer_ic2.h"
#include "lpTmr1.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TBD Cpu configuration will be declared here. */




#ifdef __cplusplus
}
#endif

/* END Cpu. */

#endif
/* Cpu_H */

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
