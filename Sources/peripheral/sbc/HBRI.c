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
 * @file HBRI.c
 * @brief H-Bridge Driver 2 Click Driver.
 */

#include "HBRI.h"
//#include "lpspiCom1.h"
#include "pin_mux.h"

/* Definition of the data transfer size */
#define BUFFER_SIZE (3u)
#define TIMEOUT (50u)

/**
 * @brief Dummy data.
 * @details Definition of dummy data.
 */
#define DUMMY                            0x00
#define CMD_REG_BIT_MASK                 0x3F

/**
 * @brief Operation codes data.
 * @details Definition of operation codes data.
 */
#define ACCESS_MODE_WRITE                0x00
#define ACCESS_MODE_READ                 0x40
#define ACCESS_MODEP_READ_ADN_CLEAR      0x80
#define ACCESS_MODE_READ_DEVICE_ID       0xC0

static unsigned char global_fault;
static HBRI_dev_id_t dev_id;

#if 1
void Delay_us(volatile int cycles)
{
   unsigned short i = 0;
    /* Delay function - do nothing for a number of cycles */
    while(cycles--)
    {
	  i = 10;
	  while(i--);
	}
}

void Delay_ms(volatile int cycles)
{
   unsigned short i = 0;
	/* Delay function - do nothing for a number of cycles */
	while(cycles--)
	{
	  i = 12000;
	  while(i--);
	}
}

/**
 * @brief Tsact delay.
 * @details Time delay from Standby ( CS rising edge MODE = 1 and EN = 1 ) 
 * into Active mode ( NRDY = 0 ).
 */
static void dev_tsact_delay ( void );

/**
 * @brief Tscsn hi EN hi delay.
 * @details Minimum time between CS high and EN high edge.
 */
static void dev_tcsn_hi_en_hi_delay ( void );

/**
 * @brief Repeat set-up delay.
 * @details Minimum time to repeat set-up delay.
 */
static void dev_repeat_delay ( void );


void HBRI_InitIial(void)
{
    /** 
     * Logger initialization.
     * Default baud rate: 115200
     * Default log level: LOG_LEVEL_DEBUG
     * @note If USB_UART_RX and USB_UART_TX 
     * are defined as HAL_PIN_NC, you will 
     * need to define them manually for log to work. 
     * See @b LOG_MAP_USB_UART macro definition for detailed explanation.
     */
	SEGGER_RTT_printf(0, "HBRI Init FL\r\n");

#if 1
	LPSPI_DRV_SetPcs(LPSPICOM1, LPSPI_PCS1, LPSPI_ACTIVE_LOW);
	PINS_DRV_SetPins(PTA, 1 << 11);

    HBRI_enable();
    Delay_ms( 10 );
	SEGGER_RTT_printf(0, "Default conf\r\n");

    HBRI_default_cfg ();
    Delay_ms( 10 );

	SEGGER_RTT_printf(0, "--------------------------------\r\n");
    Delay_ms( 10 );

    HBRI_get_device_id( &global_fault, &dev_id );
    Delay_ms( 10 );
    SEGGER_RTT_printf(0, " ID header      : 0x%.4X \r\n", dev_id.id_header );
    SEGGER_RTT_printf(0, " Version        : 0x%.4X \r\n", dev_id.version );
    SEGGER_RTT_printf(0, " Product Code 1 : 0x%.4X \r\n", dev_id.product_code_1 );
    SEGGER_RTT_printf(0, " Product Code 2 : 0x%.4X \r\n", dev_id.product_code_2 );
    SEGGER_RTT_printf(0, " SPI Frame ID   : 0x%.4X \r\n", dev_id.spi_frame_id );
    SEGGER_RTT_printf(0, "--------------------------------\r\n" );
    Delay_ms( 10 );

	SEGGER_RTT_printf(0, "HBRI Init RL\r\n");
    
	LPSPI_DRV_SetPcs(LPSPICOM1, LPSPI_PCS0, LPSPI_ACTIVE_LOW);
	PINS_DRV_SetPins(PTD, 1 << 3);

    HBRI_enable();
    Delay_ms( 10 );
	SEGGER_RTT_printf(0, "Default conf\r\n");
    
    HBRI_default_cfg ();
    Delay_ms( 10 );

	SEGGER_RTT_printf(0, "--------------------------------\r\n");
    Delay_ms( 10 );
    
    HBRI_get_device_id( &global_fault, &dev_id );
    Delay_ms( 10 );  
    SEGGER_RTT_printf(0, " ID header      : 0x%.4X \r\n", dev_id.id_header ); 
    SEGGER_RTT_printf(0, " Version        : 0x%.4X \r\n", dev_id.version );
    SEGGER_RTT_printf(0, " Product Code 1 : 0x%.4X \r\n", dev_id.product_code_1 );
    SEGGER_RTT_printf(0, " Product Code 2 : 0x%.4X \r\n", dev_id.product_code_2 );
    SEGGER_RTT_printf(0, " SPI Frame ID   : 0x%.4X \r\n", dev_id.spi_frame_id );
    SEGGER_RTT_printf(0, "--------------------------------\r\n" );
    Delay_ms( 10 );
#endif
    //HBRI_set_duty_cycle ( 0.5 );
    //HBRI_pwm_start();
    Delay_ms( 10 );
    
    SEGGER_RTT_printf(0, "\t>>> START <<<\r\n" );
    Delay_ms( 10 );
}

void display_status ( void )
{
    uint16_t status;

    SEGGER_RTT_printf(0, "- - - - - - - - -  - - - - - - -\r\n" );
    SEGGER_RTT_printf(0, " Status :" );
    HBRI_get_status( &status );

    if ( HBRI_STAT_0_OCHS1 == status )
    {
    	SEGGER_RTT_printf(0, " HS1 Over current detected\r\n" );
    }

    if ( HBRI_STAT_0_OCLS1 == status )
    {
    	SEGGER_RTT_printf(0, " LS1 Over current detected\r\n" );
    }

    if ( HBRI_STAT_0_OCHS2 == status )
    {
    	SEGGER_RTT_printf(0, " HS2 Over current detected\r\n" );
    }

    if ( HBRI_STAT_0_OCLS2 == status )
    {
    	SEGGER_RTT_printf(0, " LS2 Over current detected\r\n" );
    }

    if ( HBRI_STAT_0_VSUV == status )
    {
    	SEGGER_RTT_printf(0, " Under voltage detected\r\n" );
    }

    if ( HBRI_STAT_0_VSOV == status )
    {
    	SEGGER_RTT_printf(0, " Over voltage detected\r\n" );
    }

    if ( HBRI_STAT_0_OK == status )
    {
    	SEGGER_RTT_printf(0, " Normal Operation\r\n" );
    }

    SEGGER_RTT_printf(0, "--------------------------------\r\n" );
    Delay_ms( 10 );
}


void HBRI_task ( void )
{
    //static uint32_t timer = 0;
    static uint8_t HBRI_task_Sts = 0;
    static uint8_t preHBRI_task_Sts = 0;

    if(preHBRI_task_Sts != HBRI_task_Sts)
    {
    switch(HBRI_task_Sts)
    {
    case 0:
        SEGGER_RTT_printf(0, "\t>>> Run Forward\r\n" );
        HBRI_run_forward( &global_fault );
    	break;
    case 1:
        SEGGER_RTT_printf(0, "\t>>> Stop With Brake\r\n" );
        HBRI_stop_with_brake( &global_fault );
    	break;
    case 2:
        SEGGER_RTT_printf(0, "\t>>> Run Backward\r\n" );
        HBRI_run_backward( &global_fault );
    	break;
    case 3:
        SEGGER_RTT_printf(0, "\t>>> Stop\r\n" );
        HBRI_stop( &global_fault );
    	break;
    }
    }
    preHBRI_task_Sts = HBRI_task_Sts;

#if 0
    SEGGER_RTT_printf(0, "\t>>> Run Forward\r\n" );
    HBRI_run_forward( &global_fault );
    Delay_ms( 1000 );
    
    SEGGER_RTT_printf(0, "\t>>> Stop With Brake\r\n" );
    HBRI_stop_with_brake( &global_fault );
    Delay_ms( 1000 );
    
    SEGGER_RTT_printf(0, "\t>>> Run Backward\r\n" );
    HBRI_run_backward( &global_fault );
    Delay_ms( 1000 );
    
    SEGGER_RTT_printf(0, "\t>>> Stop\r\n" );
    HBRI_stop( &global_fault );
    Delay_ms( 1000 );

    display_status ();
#endif
}


void HBRI_moto_Open ( uint8_t motorId )
{
  HBRI_run_backward( &global_fault );
}
void HBRI_moto_Close ( uint8_t motorId )
{
  HBRI_run_forward( &global_fault );

}
void HBRI_moto_Stop_with_brake ( uint8_t motorId )
{
  HBRI_stop_with_brake( &global_fault );
}

void HBRI_moto_Stop ( uint8_t motorId )
{
  HBRI_stop( &global_fault );
}


void HBRI_default_cfg ( void )
{   
    //HBRI_disable( );
    HBRI_set_active_mode( );
}

void HBRI_enable ( void )
{
	//PINS_DRV_SetPins(PTA, 1 << 11);
    //digital_out_high( &ctx->en );
}

void HBRI_disable ( void )
{
	PINS_DRV_ClearPins(PTA, 1 << 11);
    //digital_out_low( &ctx->en );
}

void HBRI_write_reg ( unsigned char reg, unsigned char *global_fault, uint16_t data_in )
{
    uint8_t rx_buf_msb = 0;
    uint8_t rx_buf_lsb = 0;
    uint8_t sendBuf[BUFFER_SIZE];
    uint8_t recBuf[BUFFER_SIZE];

    reg &= CMD_REG_BIT_MASK;
    reg |= ACCESS_MODE_WRITE;

    sendBuf[0] = reg;
	sendBuf[1] = ( uint8_t ) ( data_in >> 8 );
	sendBuf[2] = ( uint8_t ) data_in;

	LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, sendBuf, recBuf, BUFFER_SIZE, TIMEOUT);
	
	while ( ( rx_buf_msb != sendBuf[ 1 ] ) || ( rx_buf_lsb != sendBuf[ 2 ] ) )
	{
		LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, sendBuf, recBuf, BUFFER_SIZE, TIMEOUT);
		//dev_repeat_delay( );
		rx_buf_msb = recBuf[1];
		rx_buf_lsb = recBuf[2];
	}
	//SEGGER_RTT_printf(0, " sendBuf - 0x%.2X - 0x%.2X - 0x%.2X\r\n", sendBuf[0], sendBuf[1], sendBuf[2] );
	//SEGGER_RTT_printf(0, " recBuf - 0x%.2X - 0x%.2X - 0x%.2X\r\n\r\n", recBuf[0], recBuf[1], recBuf[2] );

	*global_fault = recBuf[0];
	//rx_buf_msb = recBuf[1];
	//rx_buf_lsb = recBuf[2];

}

void HBRI_read_reg ( unsigned char reg, unsigned char *global_fault, uint16_t *data_out )
{
    uint8_t rx_buf[ 2 ];
    uint8_t sendBuf[BUFFER_SIZE];
    uint8_t recBuf[BUFFER_SIZE];

    reg &= CMD_REG_BIT_MASK;
    reg |= ACCESS_MODE_READ;

    sendBuf[0] = reg;
	sendBuf[1] = DUMMY;
	sendBuf[2] = DUMMY;


	LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, sendBuf, recBuf, BUFFER_SIZE, TIMEOUT);

	global_fault = &recBuf[0];
	rx_buf[0] = recBuf[1];
	rx_buf[1] = recBuf[2];
    
    *data_out = rx_buf[0];
    *data_out <<= 8;
    *data_out |= rx_buf[1];


}

void HBRI_read_clear_reg ( unsigned char reg, unsigned char *global_fault, uint16_t *data_out )
{
    unsigned char rx_buf[ 2 ];
    uint8_t sendBuf[BUFFER_SIZE];
    uint8_t recBuf[BUFFER_SIZE];

    reg &= CMD_REG_BIT_MASK;
    reg |= ACCESS_MODEP_READ_ADN_CLEAR;
    
    sendBuf[0] = reg;
	sendBuf[1] = DUMMY;
	sendBuf[2] = DUMMY;


	LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, sendBuf, recBuf, BUFFER_SIZE, TIMEOUT);

	global_fault = &recBuf[0];
	rx_buf[0] = recBuf[1];
	rx_buf[1] = recBuf[2];
    
    *data_out = rx_buf[ 0 ];
    *data_out <<= 8;
    *data_out |= rx_buf[ 1 ];
}

void HBRI_read_id ( unsigned char reg, unsigned char *global_fault, uint16_t *data_out )
{
    unsigned char rx_buf[ 2 ];
    uint8_t sendBuf[BUFFER_SIZE];
    uint8_t recBuf[BUFFER_SIZE];
    
    reg &= CMD_REG_BIT_MASK;
    reg |= ACCESS_MODE_READ_DEVICE_ID;

    sendBuf[0] = reg;
	sendBuf[1] = DUMMY;
	sendBuf[2] = DUMMY;

	LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, sendBuf, recBuf, BUFFER_SIZE, TIMEOUT);

	global_fault = &recBuf[0];
	rx_buf[0] = recBuf[1];
	rx_buf[1] = recBuf[2];
    
    *data_out = rx_buf[ 0 ];
    *data_out <<= 8;
    *data_out |= rx_buf[ 1 ];
}

void HBRI_set_active_mode ( void )
{
    unsigned char device_status_bits;    
   
    HBRI_enable( );
    dev_tcsn_hi_en_hi_delay( );
    
    HBRI_set_control( &device_status_bits, HBRI_CTL_0_MODE );
    
    dev_tsact_delay( );

    while ( device_status_bits != HBRI_GLB_RESB_NORMAL )
    {
    	HBRI_set_control( &device_status_bits, HBRI_CTL_0_MODE );
        dev_repeat_delay( );
    }

    HBRI_set_config( &device_status_bits, HBRI_CFG_NOCRLH_250_NS |
                                                         HBRI_CFG_NOCRHL_250_NS |
														 HBRI_CFG_OCTH_1_V   |
                                                         HBRI_CFG_FULL_H_BRIDGE |
                                                         HBRI_CFG_SLOW_SLEW_RATE );
}

void HBRI_get_status ( uint16_t *status )
{
    unsigned char rx_buf_0;
    unsigned char rx_buf_1;
    uint8_t sendBuf[BUFFER_SIZE];
    uint8_t recBuf[BUFFER_SIZE];

    sendBuf[0] = HBRI_CMD_REG_STAT_0 |ACCESS_MODEP_READ_ADN_CLEAR;
	sendBuf[1] = DUMMY;
	sendBuf[2] = DUMMY;

	LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, sendBuf, recBuf, BUFFER_SIZE, TIMEOUT);

	rx_buf_0 = recBuf[1];
	rx_buf_1 = recBuf[2];
    
    *status = rx_buf_0;
    *status <<= 8;
    *status |= rx_buf_1;
    
}

void HBRI_get_control ( uint16_t *control )
{
    //unsigned char device_status_bits;
    unsigned char rx_buf_0;
    unsigned char rx_buf_1;
    uint8_t sendBuf[BUFFER_SIZE];
    uint8_t recBuf[BUFFER_SIZE];

    sendBuf[0] = HBRI_CMD_REG_CTL_0 | ACCESS_MODE_READ;
	sendBuf[1] = DUMMY;
	sendBuf[2] = DUMMY;

	LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, sendBuf, recBuf, BUFFER_SIZE, TIMEOUT);

	//device_status_bits = recBuf[0];
	rx_buf_0 = recBuf[1];
	rx_buf_1 = recBuf[2];
    
    *control = rx_buf_0;
    *control <<= 8;
    *control |= rx_buf_1;
    
}

void HBRI_set_control ( unsigned char *write_status, uint16_t control )
{   
    HBRI_write_reg( HBRI_CMD_REG_CTL_0, write_status, control );
}

void HBRI_get_config ( uint16_t *config )
{
    //unsigned char device_status_bits;
    unsigned char rx_buf_0;
    unsigned char rx_buf_1;

    uint8_t sendBuf[BUFFER_SIZE];
    uint8_t recBuf[BUFFER_SIZE];

    sendBuf[0] = HBRI_CMD_REG_CFG | ACCESS_MODE_READ;
	sendBuf[1] = DUMMY;
	sendBuf[2] = DUMMY;

	LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, sendBuf, recBuf, BUFFER_SIZE, TIMEOUT);

	//device_status_bits = recBuf[0];
	rx_buf_0 = recBuf[1];
	rx_buf_1 = recBuf[2];
    
    *config = rx_buf_0;
    *config <<= 8;
    *config |= rx_buf_1;
}

void HBRI_set_config ( unsigned char *write_status, uint16_t config )
{   
    return HBRI_write_reg( HBRI_CMD_REG_CFG, write_status, config );
}

void HBRI_get_device_id ( unsigned char *read_status, HBRI_dev_id_t *dev_id )
{      
    HBRI_read_id( HBRI_ID_REG_ID_HDR, read_status, &dev_id->id_header );
    
    HBRI_read_id( HBRI_ID_REG_VERSION, read_status, &dev_id->version );
    
    HBRI_read_id( HBRI_ID_REG_P_CODE_1, read_status, &dev_id->product_code_1 );
    
    HBRI_read_id( HBRI_ID_REG_P_CODE_2, read_status, &dev_id->product_code_2 );
    
    HBRI_read_id( HBRI_ID_REG_SPI_F_ID, read_status, &dev_id->spi_frame_id );
}

void HBRI_control ( unsigned char *write_status, HBRI_ctrl_t dev_ctrl )
{
    uint16_t rx_buf;
   
    rx_buf = dev_ctrl.hs_1;
    rx_buf <<= 1;
    rx_buf |= dev_ctrl.ls_1;
    rx_buf <<= 1;
    rx_buf |= dev_ctrl.hs_2;
    rx_buf <<= 1;
    rx_buf |= dev_ctrl.ls_2;
    rx_buf <<= 2;
    rx_buf |= dev_ctrl.fw_h;
    rx_buf <<= 1;
    rx_buf |= dev_ctrl.fw_a;
    rx_buf <<= 1;
    rx_buf |= dev_ctrl.ovr;
    rx_buf <<= 1;
    rx_buf |= dev_ctrl.uvr;
    rx_buf <<= 1;
    rx_buf |= dev_ctrl.mode;

    HBRI_write_reg( HBRI_CMD_REG_CTL_0, write_status, rx_buf );
}

void HBRI_run_backward ( unsigned char *global_fault )
{
    HBRI_ctrl_t dev_ctrl;
    
    dev_ctrl.hs_1 = HBRI_SET;
    dev_ctrl.ls_1 = HBRI_CLEAR;
    dev_ctrl.hs_2 = HBRI_CLEAR;
    dev_ctrl.ls_2 = HBRI_SET;
    dev_ctrl.fw_h = HBRI_SET;
    dev_ctrl.fw_a = HBRI_SET;
    dev_ctrl.ovr  = HBRI_CLEAR;
    dev_ctrl.uvr  = HBRI_CLEAR;
    dev_ctrl.mode = HBRI_SET; 

    HBRI_control( global_fault, dev_ctrl );
}

void HBRI_run_forward ( unsigned char *global_fault )
{
    HBRI_ctrl_t dev_ctrl;
    
    dev_ctrl.hs_1 = HBRI_CLEAR;
    dev_ctrl.ls_1 = HBRI_SET;
    dev_ctrl.hs_2 = HBRI_SET;
    dev_ctrl.ls_2 = HBRI_CLEAR;
    dev_ctrl.fw_h = HBRI_SET;
    dev_ctrl.fw_a = HBRI_SET;
    dev_ctrl.ovr  = HBRI_CLEAR;
    dev_ctrl.uvr  = HBRI_CLEAR;
    dev_ctrl.mode = HBRI_SET; 

    HBRI_control( global_fault, dev_ctrl );
}

void HBRI_stop ( unsigned char *global_fault )
{
    HBRI_ctrl_t dev_ctrl;
    
    dev_ctrl.hs_1 = HBRI_CLEAR;
    dev_ctrl.ls_1 = HBRI_CLEAR;
    dev_ctrl.hs_2 = HBRI_CLEAR;
    dev_ctrl.ls_2 = HBRI_CLEAR;
    dev_ctrl.fw_h = HBRI_CLEAR;
    dev_ctrl.fw_a = HBRI_CLEAR;
    dev_ctrl.ovr  = HBRI_CLEAR;
    dev_ctrl.uvr  = HBRI_CLEAR;
    dev_ctrl.mode = HBRI_SET; 

    HBRI_control( global_fault, dev_ctrl );
}

void HBRI_stop_with_brake ( unsigned char *global_fault )
{
    HBRI_ctrl_t dev_ctrl;
    
    dev_ctrl.hs_1 = HBRI_CLEAR;
    dev_ctrl.ls_1 = HBRI_SET;
    dev_ctrl.hs_2 = HBRI_CLEAR;
    dev_ctrl.ls_2 = HBRI_SET;
    dev_ctrl.fw_h = HBRI_CLEAR;
    dev_ctrl.fw_a = HBRI_CLEAR;
    dev_ctrl.ovr  = HBRI_CLEAR;
    dev_ctrl.uvr  = HBRI_CLEAR;
    dev_ctrl.mode = HBRI_SET; 

    HBRI_control( global_fault, dev_ctrl );
}

void HBRI_set_duty_cycle ( float duty_cycle )
{
    //pwm_set_duty( &ctx->pwm, duty_cycle );
}

void HBRI_pwm_stop ( void )
{
    //pwm_stop( &ctx->pwm );
}

void HBRI_pwm_start ( void )
{
    //pwm_start( &ctx->pwm );
}

static void dev_tsact_delay ( void )
{
    //Delay_80us( );
    //Delay_80us( );
    //Delay_80us( );
    //Delay_80us( );
    //Delay_22us( );

	Delay_us(80);
	Delay_us(80);
	Delay_us(80);
	Delay_us(80);
	Delay_us(22);
}

static void dev_tcsn_hi_en_hi_delay ( void )
{
    //Delay_80us( );
    //Delay_22us( );

	Delay_us(80);
	Delay_us(22);

}

static void dev_repeat_delay ( void )
{
    //Delay_10ms( );
	Delay_ms(10);

}
#endif
// ------------------------------------------------------------------------- END
