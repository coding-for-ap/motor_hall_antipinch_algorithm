#include "SPI.h"
#include "lpspiCom1.h"

#define BUFFER_SIZE (3u)


void SPI_Initial(void)
{
    /* Write your local variable definition here */
    /* Declaration of the LPSPI transfer buffers */
    //uint8_t masterDataSend[BUFFER_SIZE];
    //uint8_t masterDataReceive[BUFFER_SIZE];
    uint16_t i;

    /* Initialize DMA */
    //EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);

    /* SPI master configuration: clock speed: 500 kHz, 8 bits/frame, MSB first */
    //INT_SYS_SetPriority(LPSPI0_IRQn,5);

    LPSPI_DRV_MasterInit(LPSPICOM1, &lpspiCom1State, &lpspiCom1_MasterConfig0);

    /* Set delay for transmission */
    LPSPI_DRV_MasterSetDelay(LPSPICOM1, 1u, 1u, 1u);



    /* Initialize the data buffer */
    for (i = 0u; i < BUFFER_SIZE; i++)
    {
        //masterDataSend[i] = i;
    }

    //LPSPI_DRV_MasterTransferBlocking(LPSPICOM1, masterDataSend, masterDataReceive, BUFFER_SIZE, TIMEOUT);
}
