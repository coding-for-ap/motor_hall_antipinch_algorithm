/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/

/*!
 * @file HBRI.h
 * @brief This file contains API for H-Bridge Driver 2 Click Driver.
 */

#ifndef HBRI_H
#define HBRI_H

#include "Cpu.h"
#include "SEGGER_RTT.h"


/*!
 * @addtogroup HBRI H-Bridge Driver 2 Click Driver
 * @brief API for configuring and manipulating H-Bridge Driver 2 Click driver.
 * @{
 */

/**
 * @defgroup HBRI_reg H-Bridge Driver 2 Registers List
 * @brief List of registers of H-Bridge Driver 2 Click driver.
 */

/**
 * @addtogroup HBRI_reg
 * @{
 */

/**
 * @brief H-Bridge Driver 2 description register.
 * @details Specified register for description of H-Bridge Driver 2 Click driver.
 */
#define HBRI_CMD_REG_CTL_0        0x00
#define HBRI_CMD_REG_STAT_0       0x10
#define HBRI_CMD_REG_CFG          0x3F
#define HBRI_ID_REG_ID_HDR        0x00
#define HBRI_ID_REG_VERSION       0x01
#define HBRI_ID_REG_P_CODE_1      0x02
#define HBRI_ID_REG_P_CODE_2      0x03
#define HBRI_ID_REG_SPI_F_ID      0x3E

/*! @} */ // HBRI_reg

/**
 * @defgroup HBRI_set H-Bridge Driver 2 Registers Settings
 * @brief Settings for registers of H-Bridge Driver 2 Click driver.
 */

/**
 * @addtogroup global_fault
 * @{
 */

/**
 * @brief H-Bridge Driver 2 description Global Fault.
 * @details Specified global status byte content for description of H-Bridge Driver 2 Click driver.
 */
#define HBRI_GLB_FLT              0x80
#define HBRI_GLB_TF               0x40
#define HBRI_GLB_RESB_NORMAL      0x20
#define HBRI_GLB_TSD              0x10
#define HBRI_GLB_UOV_OC           0x04
#define HBRI_GLB_NRDY             0x01

/*! @} */ // global_fault

/**
 * @addtogroup HBRI_set
 * @{
 */

/**
 * @brief H-Bridge Driver 2default PWM frequency.
 * @details Specified setting for setting default PWM frequency of H-Bridge Driver 2 Click driver.
 */
#define HBRI_DEF_FREQ            20000

/**
 * @brief H-Bridge Driver 2 description ID data.
 * @details Specified Chip ID Registers Addresses and Values for description of H-Bridge Driver 2 Click driver.
 */
#define HBRI_ID_VAL_ID_HDR      0x4300 
#define HBRI_ID_VAL_VERSION     0x0100 
#define HBRI_ID_VAL_P_CODE_1    0x7500 
#define HBRI_ID_VAL_P_CODE_2    0x3500 
#define HBRI_ID_VAL_SPI_F_ID    0x0200 

/**
 * @brief H-Bridge Driver 2 description control data.
 * @details Specified Control Register Bits for description of H-Bridge Driver 2 Click driver.
 */
#define HBRI_CTL_0_HS1          0x0200 
#define HBRI_CTL_0_LS1          0x0100 
#define HBRI_CTL_0_HS2          0x0080 
#define HBRI_CTL_0_LS2          0x0040 
#define HBRI_CTL_0_FWH          0x0010 
#define HBRI_CTL_0_FWA          0x0008 
#define HBRI_CTL_0_OVR          0x0004 
#define HBRI_CTL_0_UVR          0x0002 
#define HBRI_CTL_0_MODE         0x0001

/**
 * @brief H-Bridge Driver 2 description status data.
 * @details Specified Status Register Bits for description of H-Bridge Driver 2 Click driver.
 */
#define HBRI_STAT_0_OCHS1       0x0200 
#define HBRI_STAT_0_OCLS1       0x0100 
#define HBRI_STAT_0_OCHS2       0x0080 
#define HBRI_STAT_0_OCLS2       0x0040 
#define HBRI_STAT_0_VSUV        0x0008 
#define HBRI_STAT_0_VSOV        0x0004 
#define HBRI_STAT_0_OK          0x0000

/**
 * @brief H-Bridge Driver 2 description configuration data.
 * @details Specified Configuration Register Bits for description of H-Bridge Driver 2 Click driver.
 */
#define HBRI_CFG_NOCRLH_250_NS   0x0000
#define HBRI_CFG_NOCRLH_500_NS   0x1000
#define HBRI_CFG_NOCRLH_750_NS   0x2000
#define HBRI_CFG_NOCRLH_1_US     0x3000
#define HBRI_CFG_NOCRLH_1_25_US  0x4000
#define HBRI_CFG_NOCRLH_1_5_US   0x5000
#define HBRI_CFG_NOCRLH_1_75_US  0x6000
#define HBRI_CFG_NOCRLH_2_US     0x7000
#define HBRI_CFG_NOCRLH_2_25_US  0x8000
#define HBRI_CFG_NOCRLH_2_5_US   0x9000
#define HBRI_CFG_NOCRLH_2_75_US  0xA000
#define HBRI_CFG_NOCRLH_3_US     0xB000
#define HBRI_CFG_NOCRLH_3_25_US  0xC000
#define HBRI_CFG_NOCRLH_3_5_US   0xD000
#define HBRI_CFG_NOCRLH_3_75_US  0xE000
#define HBRI_CFG_NOCRLH_4_US     0xF000
#define HBRI_CFG_NOCRHL_250_NS   0x0000
#define HBRI_CFG_NOCRHL_500_NS   0x0100
#define HBRI_CFG_NOCRHL_750_NS   0x0200
#define HBRI_CFG_NOCRHL_1_US     0x0300
#define HBRI_CFG_NOCRHL_1_25_US  0x0400
#define HBRI_CFG_NOCRHL_1_5_US   0x0500
#define HBRI_CFG_NOCRHL_1_75_US  0x0600
#define HBRI_CFG_NOCRHL_2_US     0x0700
#define HBRI_CFG_NOCRHL_2_25_US  0x0800
#define HBRI_CFG_NOCRHL_2_5_US   0x0900
#define HBRI_CFG_NOCRHL_2_75_US  0x0A00
#define HBRI_CFG_NOCRHL_3_US     0x0B00
#define HBRI_CFG_NOCRHL_3_25_US  0x0C00
#define HBRI_CFG_NOCRHL_3_5_US   0x0D00
#define HBRI_CFG_NOCRHL_3_75_US  0x0E00
#define HBRI_CFG_NOCRHL_4_US     0x0F00
#define HBRI_CFG_OCTH_0_25_V     0x0000
#define HBRI_CFG_OCTH_0_5_V      0x0020
#define HBRI_CFG_OCTH_0_75_V     0x0040
#define HBRI_CFG_OCTH_1_V        0x0060
#define HBRI_CFG_OCTH_1_25_V     0x0080
#define HBRI_CFG_OCTH_1_5_V      0x0090
#define HBRI_CFG_OCTH_1_75_V     0x00C0
#define HBRI_CFG_OCTH_2_V        0x00E0
#define HBRI_CFG_FULL_H_BRIDGE   0x0000
#define HBRI_CFG_2_HALF_BRIDGES  0x0010
#define HBRI_CFG_SLOW_SLEW_RATE  0x0000
#define HBRI_CFG_FAST_SLEW_RATE  0x0008

/**
 * @brief H-Bridge Driver 2 description Bit set/clear data.
 * @details Specified Bit set/clear for description of H-Bridge Driver 2 Click driver.
 */
#define HBRI_CLEAR                0x00
#define HBRI_SET                  0x01

/**
 * @brief Data sample selection.
 * @details This macro sets data samples for SPI modules.
 * @note Available only on Microchip PIC family devices.
 * This macro will set data sampling for all SPI modules on MCU. 
 * Can be overwritten with @b HBRI_init which will set
 * @b SET_SPI_DATA_SAMPLE_MIDDLE by default on the mapped mikrobus.
 */

/*! @} */ // HBRI_set

/**
 * @defgroup HBRI_map H-Bridge Driver 2 MikroBUS Map
 * @brief MikroBUS pin mapping of H-Bridge Driver 2 Click driver.
 */

/**
 * @addtogroup HBRI_map
 * @{
 */

/**
 * @brief MikroBUS pin mapping.
 * @details Mapping pins of H-Bridge Driver 2 Click to the selected MikroBUS.
 */


/*! @} */ // HBRI_map
/*! @} */ // HBRI

/**
 * @brief H-Bridge Driver 2 Click context object.
 * @details Context object definition of H-Bridge Driver 2 Click driver.
 */
#if 0
typedef struct
{
    // Output pins
    digital_out_t  en;          /**< Enable pin. */

    // Modules
    spi_master_t  spi;          /**< SPI driver object. */

    pin_name_t  chip_select;    /**< Chip select pin descriptor (used for SPI driver). */

    pwm_t pwm;                  /**< PWM driver object. */

    uint32_t pwm_freq;          /**< PWM frequency value. */

} HBRI_t;
#endif
/**
 * @brief H-Bridge Driver 2 Click configuration object.
 * @details Configuration object definition of H-Bridge Driver 2 Click driver.
 */
#if 0
typedef struct
{
    // Communication gpio pins
    pin_name_t  miso;    /**< Master input - slave output pin descriptor for SPI driver. */
    pin_name_t  mosi;    /**< Master output - slave input pin descriptor for SPI driver. */
    pin_name_t  sck;     /**< Clock pin descriptor for SPI driver. */
    pin_name_t  cs;      /**< Chip select pin descriptor for SPI driver. */

    // Additional gpio pins
    pin_name_t  en;      /**< Enable pin. */

    // static variable
    uint32_t                           spi_speed;        /**< SPI serial speed. */
    spi_master_mode_t                  spi_mode;         /**< SPI master mode. */
    spi_master_chip_select_polarity_t  cs_polarity;      /**< Chip select pin polarity. */

    pin_name_t pwm;           /**< PWM pin. */

    uint32_t dev_pwm_freq;   /**< PWM frequency value. */

} HBRI_cfg_t;
#endif

/**
 * @brief H-Bridge Driver 2 Click device ID object.
 * @details Device ID object definition of H-Bridge Driver 2 Click driver.
 */
typedef struct
{   
    uint16_t id_header;         /**< ID header. */
    uint16_t version;           /**< Version. */
    uint16_t product_code_1;    /**< Product Code 1. */
    uint16_t product_code_2;    /**< ID header. */
    uint16_t spi_frame_id;     /**< SPI Frame ID. */

} HBRI_dev_id_t;

/**
 * @brief H-Bridge Driver 2 Click device ID object.
 * @details Device ID object definition of H-Bridge Driver 2 Click driver.
 */
typedef struct
{   
    uint8_t hs_1;  /* HS 1 Output Control */       
    uint8_t ls_1;  /* LS 1 Output Control */    
    uint8_t hs_2;  /* HS 2 Output Control */           
    uint8_t ls_2;  /* LS 2 Output Control */    
    
    uint8_t fw_h;  /* Freewheeling High side or low side */      
    uint8_t fw_a;  /* Freewheeling Active or passive */
    
    uint8_t ovr;  /* Over−voltage Recovery */      
    uint8_t uvr;  /* Under−voltage Recovery */ 
    
    uint8_t mode; /* Mode Control */

} HBRI_ctrl_t;

/**
 * @brief H-Bridge Driver 2 Click return value data.
 * @details Predefined enum values for driver return values.
 */
typedef enum
{
    HBRI_OK = 0,
    HBRI_ERROR = -1

} HBRI_return_value_t;

#if 1
/**
 * @brief H-Bridge Driver 2 initialization function.
 * @details This function initializes all necessary pins and peripherals used
 * for this click board.
 * @param[out] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] cfg : Click configuration structure.
 * See #HBRI_cfg_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
extern void HBRI_InitIial(void);

/**
 * @brief H-Bridge Driver 2 default configuration function.
 * @details This function executes a default configuration of H-Bridge Driver 2
 * click board.
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note This function can consist any necessary configuration or setting to put
 * device into operating mode.
 */
void HBRI_default_cfg ( void );

/**
 * @brief H-Bridge Driver 2 enable function.
 * @details This function sets enable signal to turn on the 
 * HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_enable ( void );

/**
 * @brief H-Bridge Driver 2 disable function.
 * @details This function sets enable signal to turn off the 
 * HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_disable ( void );

/**
 * @brief H-Bridge Driver 2 data writing function.
 * @details This function writes a 16-bit of data from
 * the selected register by using SPI serial interface.
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[out] global_fault : Global status byte content.
 * @param[in] data_in : Data to be written.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_write_reg ( uint8_t reg, uint8_t *global_fault, uint16_t data_in );

/**
 * @brief H-Bridge Driver 2 data reading function.
 * @details This function reads a 16-bit of data from
 * the selected register by using SPI serial interface.
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[out] global_fault : Global status byte content.
 * @param[out] data_out : Output read data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_read_reg ( uint8_t reg, uint8_t *global_fault, uint16_t *data_out );

/**
 * @brief H-Bridge Driver 2 data read and clear function.
 * @details This function read and clear 16-bit of data from
 * the status register by using SPI serial interface.
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[out] global_fault : Global status byte content.
 * @param[out] data_out : Output read data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_read_clear_reg ( uint8_t reg, uint8_t *global_fault, uint16_t *data_out );

/**
 * @brief H-Bridge Driver 2 data read ID function.
 * @details This function reads 16-bit of ID data from
 * the selected ID register by using SPI serial interface.
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] reg : Start register address.
 * @param[out] global_fault : Global status byte content.
 * @param[out] data_out : Output read data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_read_id ( uint8_t reg, uint8_t *global_fault, uint16_t *data_out );

/**
 * @brief H-Bridge Driver 2 set active mode function.
 * @details This function set active mode 
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_set_active_mode ( void );

/**
 * @brief H-Bridge Driver 2 get status function.
 * @details This function reads status data
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[out] status : Status data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_get_status ( uint16_t *status );

/**
 * @brief H-Bridge Driver 2 get control function.
 * @details This function reads control data
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[out] control : Control data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_get_control ( uint16_t *control );

/**
 * @brief H-Bridge Driver 2 set control function.
 * @details This function sets control data
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] global_fault : Global status byte content.
 * @param[in] control : Control data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_set_control (  uint8_t *global_fault, uint16_t control );

/**
 * @brief H-Bridge Driver 2 get config function.
 * @details This function reads configuration data
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[out] config : Configuration data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_get_config ( uint16_t *config );

/**
 * @brief H-Bridge Driver 2 set config function.
 * @details This function sets configuration data
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] global_fault : Global status byte content.
 * @param[in] config : Configuration data.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_set_config ( uint8_t *global_fault, uint16_t config );

/**
 * @brief H-Bridge Driver 2 get device ID function.
 * @details This function reads device ID data
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] global_fault : Global status byte content.
 * @param[in] dev_id : Device ID data structure.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_get_device_id ( uint8_t *global_fault, HBRI_dev_id_t *dev_id );

/**
 * @brief H-Bridge Driver 2 control function.
 * @details This function configuration control 
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] global_fault : Global status byte content.
 * @param[in] dev_ctrl : Device configuration control data structure.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_control ( uint8_t *global_fault, HBRI_ctrl_t dev_ctrl );

/**
 * @brief H-Bridge Driver 2 run backward function.
 * @details This function performs motor start run backward
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] global_fault : Global status byte content.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_run_backward ( uint8_t *global_fault );

/**
 * @brief H-Bridge Driver 2 run forward function.
 * @details This function performs motor start run forward
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] global_fault : Global status byte content.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_run_forward ( uint8_t *global_fault );

/**
 * @brief H-Bridge Driver 2 stop function.
 * @details This function performs motor stop
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] global_fault : Global status byte content.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_stop ( uint8_t *global_fault );

/**
 * @brief H-Bridge Driver 2 stop with brake function.
 * @details This function performs motor stop with brake
 * of the HBRI7535 SPI Controlled H-bridge and Dual-Half Bridge Pre-Driver
 * on the H-Bridge Driver 2 click board�?
 * @param[in] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] global_fault : Global status byte content.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_stop_with_brake ( uint8_t *global_fault );

/**
 * @brief H-Bridge Driver 2 sets PWM duty cycle.
 * @details This function sets the PWM duty cycle in percentages ( Range[ 0..1 ] ).
 * @param[out] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @param[in] duty_cycle : PWM duty cycle.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_set_duty_cycle ( float duty_cycle );

/**
 * @brief H-Bridge Driver 2 stop PWM module.
 * @details This function stops the PWM module output.
 * @param[out] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_pwm_stop ( void );

/**
 * @brief H-Bridge Driver 2 start PWM module.
 * @details This function starts the PWM module output.
 * @param[out] ctx : Click context object.
 * See #HBRI_t object definition for detailed explanation.
 * @return @li @c  0 - Success,
 *         @li @c -1 - Error.
 *
 * See #err_t definition for detailed explanation.
 * @note None.
 */
void HBRI_pwm_start ( void );


extern void HBRI_moto_Open ( uint8_t motorId );
extern void HBRI_moto_Close ( uint8_t motorId );
extern void HBRI_moto_Stop ( uint8_t motorId );
extern void HBRI_moto_Stop_with_brake ( uint8_t motorId );
extern void display_status ( void );

#ifdef __cplusplus
}
#endif
#endif
#endif // HBRI_H

/*! @} */ // HBRI

// ------------------------------------------------------------------------ END
