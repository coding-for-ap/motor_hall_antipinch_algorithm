#include "CAN.h"
#include "canCom1.h"
#include<string.h>

/* Define receive buffer */
flexcan_msgbuff_t recvBuff;

/*
 * @brief: Send data via CAN to the specified mailbox with the specified message id
 * @param mailbox   : Destination mailbox number
 * @param messageId : Message ID
 * @param data      : Pointer to the TX data
 * @param len       : Length of the TX data
 * @return          : None
 */
void SendCANData(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len)
{
    /* Set information about the data to be sent
     *  - 1 byte in length
     *  - Standard message ID
     *  - Bit rate switch enabled to use a different bitrate for the data segment
     *  - Flexible data rate enabled
     *  - Use zeros for FD padding
     */
    flexcan_data_info_t dataInfo =
    {
            .data_length = len,
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .enable_brs  = false,
            .fd_enable   = false,
            .fd_padding  = 0U
    };

    /* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
    FLEXCAN_DRV_ConfigTxMb(INST_CANCOM1, mailbox, &dataInfo, messageId);

    /* Execute send non-blocking */
    FLEXCAN_DRV_Send(INST_CANCOM1, mailbox, &dataInfo, messageId, data);
}

void FLEXCAN_Tx_pro(void)
{
    uint8_t ciphertext[8];
    static uint16_t t = 1260;


    ciphertext[0] = (t >> 8) & 0xFF;
    ciphertext[1] = t & 0xFF;
    ciphertext[2] = (t >> 8) & 0xFF;
    ciphertext[3] = t & 0xFF;
    ciphertext[4] = (t >> 8) & 0xFF;
    ciphertext[5] = t & 0xFF;
    ciphertext[6] = (t >> 8) & 0xFF;
    ciphertext[7] = t & 0xFF;

    /* Send the information via CAN */
    SendCANData(TX_MAILBOX, TX_MSG_ID, ciphertext, 8UL);
}

void flexcan0_Callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t buffIdx, flexcan_state_t *flexcanState)
{
  (void)flexcanState;
  (void)instance;

  switch(eventType)
  {
    case FLEXCAN_EVENT_RX_COMPLETE:
    if(buffIdx==RX_MAILBOX)
    {
      /* Start receiving data in RX_MAILBOX again. */
      FLEXCAN_DRV_Receive(INST_CANCOM1, RX_MAILBOX, &recvBuff);

      //SEGGER_RTT_printf(0, "CAN0 rx = 0x%x - 0x%x - 0x%x - 0x%x - 0x%x - 0x%x - 0x%x - 0x%x\r\n", recvBuff.data[0],recvBuff.data[1],recvBuff.data[2],recvBuff.data[3],recvBuff.data[4],recvBuff.data[5],recvBuff.data[6],recvBuff.data[7]);

      if(recvBuff.msgId == 0x01)
      {
        //CPRA_CANRx_proc(&recvBuff.data[0]);
      }
    }
    break;
    case FLEXCAN_EVENT_RXFIFO_COMPLETE:
    break;
    case FLEXCAN_EVENT_DMA_COMPLETE:
    break;
    case FLEXCAN_EVENT_TX_COMPLETE:
    break;
    default:
    break;
  }
}

void flexcan0_ErrorCallback(uint8_t instance, flexcan_event_type_t eventType,
		  	  	  	  	   flexcan_state_t *flexcanState)
{
  volatile uint32_t error;

  (void)flexcanState;
  (void)instance;

  switch(eventType)
  {
    case FLEXCAN_EVENT_ERROR:
    error = FLEXCAN_DRV_GetErrorStatus(INST_CANCOM1);

    if(error&0x4) // if BOFFINT was set
    {
    // abort TX MB, after bus off recovery message is not send
    FLEXCAN_DRV_AbortTransfer(INST_CANCOM1,TX_MAILBOX);
    //SEGGER_RTT_printf(0, "abort TX MB, after bus off recovery message is not send\r\n");
    }
    break;
    default:
    break;
  }
}


void CAN_Initial(void)
{
    /*
     * Initialize FlexCAN driver
     *  - 8 byte payload size
     *  - FD enabled
     *  - Bus clock as peripheral engine clock
     */
	FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
    FLEXCAN_DRV_InstallEventCallback(INST_CANCOM1, (flexcan_callback_t)flexcan0_Callback, (void*)NULL);
    FLEXCAN_DRV_InstallErrorCallback(INST_CANCOM1, (flexcan_error_callback_t)flexcan0_ErrorCallback, (void*)NULL);

    /* Set information about the data to be received
     *  - 1 byte in length
     *  - Standard message ID
     *  - Bit rate switch enabled to use a different bitrate for the data segment
     *  - Flexible data rate enabled
     *  - Use zeros for FD padding
     */
    flexcan_data_info_t dataInfo =
    {
            .data_length = 8U,
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .enable_brs  = false,
            .fd_enable   = false,
            .fd_padding  = 0U
    };

    /* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
    FLEXCAN_DRV_ConfigRxMb(INST_CANCOM1, RX_MAILBOX, &dataInfo, RX_MSG_ID);

	/* Start receiving data in RX_MAILBOX. */
	FLEXCAN_DRV_Receive(INST_CANCOM1, RX_MAILBOX, &recvBuff);
}


