#include "ADC.h"
#include "lpit1.h"
#include "dmaController1.h"
#include "trgmux1.h"
#include "adConv1.h"
#include "pdly1.h"

#define LPIT_CHANNEL1        1UL

/* Timeout for PDB in microseconds */
#define PDLY_TIMEOUT    16UL

/* Variable to store value from ADC conversion */
volatile uint16_t adcRawValue[8];

volatile uint64_t adcRawConversions = 0;

uint16_t *Adc_Dma_Hw_Buffer;
uint16_t Adc_Dma_Hw_value[32];

/* configuration structures for simple loop transfers */
/* configuration structures for simple loop transfers */
edma_loop_transfer_config_t loopConfig = {
    .majorLoopIterationCount = 0,
    .srcOffsetEnable = false,
    .dstOffsetEnable = false,
    .minorLoopOffset = 0,
    .minorLoopChnLinkEnable = false,
    .minorLoopChnLinkNumber = 0,
    .majorLoopChnLinkEnable = false,
    .majorLoopChnLinkNumber = 0
};

edma_transfer_config_t transferConfig = {
    .srcAddr = 0,
    .destAddr = 0,
    .srcTransferSize = EDMA_TRANSFER_SIZE_2B,
    .destTransferSize = EDMA_TRANSFER_SIZE_2B,
    .srcOffset= 4,
    .destOffset = 2,
    .srcLastAddrAdjust = 0,
    .destLastAddrAdjust = 0,
    .srcModulo = EDMA_MODULO_OFF,
    .destModulo = EDMA_MODULO_OFF,
    .minorByteTransferCount = 2,
    .scatterGatherEnable = false,
    .scatterGatherNextDescAddr = 0,
    .interruptEnable = true,
    .loopTransferConfig = &loopConfig
};

/* @brief: ADC Interrupt Service Routine.
 *        Read the conversion result, store it
 *        into a variable and set a specified flag.
 */
void ADC_IRQHandler(void)
{
	if(ADC_DRV_GetConvCompleteFlag(INST_ADCONV1, 8u))
		adcRawConversions++;
	/*读取ADC寄存器的值，清楚中断标志*/
	ADC_DRV_GetChanResult(INST_ADCONV1, 8u, (uint16_t *)adcRawValue);
	//ADC_DRV_GetChanResult(INST_ADCONV1, 1U, (uint16_t *)(adcRawValue + 1));
	//ADC_DRV_GetChanResult(INST_ADCONV1, 2U, (uint16_t *)(adcRawValue + 2));
	//ADC_DRV_GetChanResult(INST_ADCONV1, 3U, (uint16_t *)(adcRawValue + 3));
	//ADC_DRV_GetChanResult(INST_ADCONV1, 4U, (uint16_t *)(adcRawValue + 4));
	//SEGGER_RTT_printf(0, "adcRawConversions = %d, adcRawValue = %d\r\n", adcRawConversions, adcRawValue[0]);
}


void DMAcallback(void *parameter, edma_chn_status_t status)
{
	//cRawConversions ++;
}

bool calculateIntValue(const pdb_timer_config_t *pdbConfig, uint32_t uSec, uint16_t * intVal)
{
    /* Local variables used to store different parameters
     * such as frequency and prescalers
     */
    uint32_t    intVal_l            = 0;
    uint8_t     pdbPrescaler        = (1 << pdbConfig->clkPreDiv);
    uint8_t     pdbPrescalerMult    = 0;
    uint32_t    pdbFrequency;

    bool resultValid = false;

    /* Get the Prescaler Multiplier from the configuration structure */
    switch (pdbConfig->clkPreMultFactor)
    {
        case PDB_CLK_PREMULT_FACT_AS_1:
            pdbPrescalerMult    =   1U;
            break;
        case PDB_CLK_PREMULT_FACT_AS_10:
            pdbPrescalerMult    =   10U;
            break;
        case PDB_CLK_PREMULT_FACT_AS_20:
            pdbPrescalerMult    =   20U;
            break;
        case PDB_CLK_PREMULT_FACT_AS_40:
            pdbPrescalerMult    =   40U;
            break;
        default:
            /* Defaulting the multiplier to 1 to avoid dividing by 0*/
            pdbPrescalerMult    =   1U;
            break;
    }

    /* Get the frequency of the PDB clock source and scale it
     * so that the result will be in microseconds.
     */
    CLOCK_SYS_GetFreq(CORE_CLK, &pdbFrequency);
    pdbFrequency /= 1000000;

    /* Calculate the interrupt value for the prescaler, multiplier, frequency
     * configured and time needed.
     */
    intVal_l = (pdbFrequency * uSec) / (pdbPrescaler * pdbPrescalerMult);

    /* Check if the value belongs to the interval */
    if((intVal_l == 0) || (intVal_l >= (1 << 16)))
    {
        resultValid = false;
        (*intVal) = 0U;
    }
    else
    {
        resultValid = true;
        (*intVal) = (uint16_t)intVal_l;
    }

    return resultValid;
}

/* This function triggers a loop memory-to-memory transfer. */
void triggerLoopTransfer(uint8_t channel, uint8_t * srcBuff, uint8_t * dstBuff, uint32_t size)
{
	dma_request_source_t DmaReq;

    /* configure transfer source and destination addresses */
    transferConfig.srcAddr 				= (uint32_t)srcBuff;
    transferConfig.destAddr 			= (uint32_t)dstBuff;
    transferConfig.srcLastAddrAdjust 	= -(4*size);
    transferConfig.destLastAddrAdjust	= -(2*size);
    loopConfig.majorLoopIterationCount	= size;

    if(channel == 0)
    {
    	DmaReq = EDMA_REQ_ADC0;
    }
    else
    {
    	//DmaReq = EDMA_REQ_ADC1;
    }

    /* configure the eDMA channel for a loop transfer (via transfer configuration structure */
    EDMA_DRV_ConfigLoopTransfer(channel, &transferConfig);

    /* select hw request */
    EDMA_DRV_SetChannelRequestAndTrigger(channel, DmaReq, false);

    /* start the channel */
    EDMA_DRV_StartChannel(channel);
}

void ADC_Initial(void)
{
    /* Write your local variable definition here */
    uint16_t delayValue = 0;

    /* Variables in which we store data from ADC */
    //uint16_t adcMax;
    //float adcValue;

    /* Buffer used to store processed data for serial communication */
    //char msg[255] =
    //{ 0, };

    Adc_Dma_Hw_Buffer = &Adc_Dma_Hw_value[0];

    /* Get ADC max value from the resolution */
    //if (adConv1_ConvConfig0.resolution == ADC_RESOLUTION_8BIT)
        ///adcMax = (uint16_t) (1 << 8);
    //else if (adConv1_ConvConfig0.resolution == ADC_RESOLUTION_10BIT)
        //adcMax = (uint16_t) (1 << 10);
    //else
        //adcMax = (uint16_t) (1 << 12);

    /* Configure and calibrate the ADC converter
     *  -   See ADC component for the configuration details
     */
    ADC_DRV_ConfigConverter(INST_ADCONV1, &adConv1_ConvConfig0);
    ADC_DRV_AutoCalibration(INST_ADCONV1);
    ADC_DRV_ConfigChan(INST_ADCONV1, 0UL, &adConv1_ChnConfig0);
    ADC_DRV_ConfigChan(INST_ADCONV1, 1UL, &adConv1_ChnConfig1);
    ADC_DRV_ConfigChan(INST_ADCONV1, 2UL, &adConv1_ChnConfig2);
    ADC_DRV_ConfigChan(INST_ADCONV1, 3UL, &adConv1_ChnConfig3);
    ADC_DRV_ConfigChan(INST_ADCONV1, 4UL, &adConv1_ChnConfig4);
    ADC_DRV_ConfigChan(INST_ADCONV1, 5UL, &adConv1_ChnConfig5);
    ADC_DRV_ConfigChan(INST_ADCONV1, 6UL, &adConv1_ChnConfig6);
    ADC_DRV_ConfigChan(INST_ADCONV1, 7UL, &adConv1_ChnConfig7);
    ADC_DRV_ConfigChan(INST_ADCONV1, 8UL, &adConv1_ChnConfig8);

    //INT_SYS_InstallHandler(ADC0_IRQn, &ADC_IRQHandler, (isr_t*) 0);
    //INT_SYS_EnableIRQ(ADC0_IRQn);

     /* Calculate the value needed for PDB instance
     * to generate an interrupt at a specified timeout.
     * If the value can not be reached, stop the application flow
     */
    if (!calculateIntValue(&pdly1_InitConfig0, PDLY_TIMEOUT, &delayValue))
    {
        /* Stop the application flow */
        while(1);
    }

    //SEGGER_RTT_printf(0, "delayValue = %d\r\n", delayValue);
    //delayValue = 30;
    /* Setup PDB instance
     *  -   See PDB component for details
     *  Note: Pre multiplier and Prescaler values come from
     *        calculateIntValue function.
     */
	PDB_DRV_Init(INST_PDLY1, &pdly1_InitConfig0);
	PDB_DRV_Enable(INST_PDLY1);

	// config 8 pretriggers on ch0 based on component setting
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 0UL , &pdly1_AdcTrigInitConfig0);
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 0UL , &pdly1_AdcTrigInitConfig1);
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 0UL , &pdly1_AdcTrigInitConfig2);
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 0UL , &pdly1_AdcTrigInitConfig3);
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 0UL , &pdly1_AdcTrigInitConfig4);
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 0UL , &pdly1_AdcTrigInitConfig5);
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 0UL , &pdly1_AdcTrigInitConfig6);
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 0UL , &pdly1_AdcTrigInitConfig7);
	// config 1 pretrigger on ch1 based on component setting
	PDB_DRV_ConfigAdcPreTrigger(INST_PDLY1, 1UL , &pdly1_AdcTrigInitConfig0);

	// set PDB1 counter period to delayValue (~30us)
	PDB_DRV_SetTimerModulusValue(INST_PDLY1,(uint32_t) delayValue);
	// set ch0 trigger delay to happen immediate upon Trigger_In 0 (PIT0 ch0)
	PDB_DRV_SetAdcPreTriggerDelayValue(INST_PDLY1, 0UL , 0UL , (uint32_t) delayValue);
	// set ch1 trigger0 delay to happen 10us upon Trigger_In 0 (PIT0 ch0)
	PDB_DRV_SetAdcPreTriggerDelayValue(INST_PDLY1, 1UL , 0UL , (uint32_t) (10*delayValue/PDLY_TIMEOUT));

	PDB_DRV_LoadValuesCmd(INST_PDLY1);

    /* Initialize DMA */
    EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);

    /* Loop memory-to-memory transfer */
    //triggerLoopTransfer(EDMA_CHN0_NUMBER, (uint8_t *)&(ADC0->R[0]), Adc_Dma_Hw_Buffer, 1);

	// set DMA ch0 to read first 16 ADC0 Results registers and move it to buffer array starting from 1st element
	//triggerLoopTransfer(DMA_CHANNEL0, (uint8_t *)&(ADC0->R[0]),buffer, 16);
	// set DMA ch1 to read first 9 ADC1 Results registers and move it to buffer array starting from 16th element
	triggerLoopTransfer(EDMA_CHN0_NUMBER, (uint8_t *)&(ADC0->R[0]),(uint8_t *)Adc_Dma_Hw_Buffer, 16);

    /* Initialize TRGMUX peripheral */
    /* Source input: TRGMUX_IN10 */
    /* Target module: LPIT Channel 0 */
    TRGMUX_DRV_Init(INST_TRGMUX1, &trgmux1_InitConfig0);

    //LPIT_DRV_StartTimerChannels(INST_LPIT1, (1 << LPIT_CHANNEL1));
    /* Enable ADC0 interrupt */
    //INT_SYS_EnableIRQ(ADC0_IRQn);
    //INT_SYS_EnableIRQ(PDB0_IRQn);
}

