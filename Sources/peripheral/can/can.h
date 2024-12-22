#ifndef _CAN_H_
#define _CAN_H_

/* Definition of the TX and RX message buffers depending on the bus role */
#define TX_MAILBOX  (1UL)
#define TX_MSG_ID   (1UL)
#define RX_MAILBOX  (0UL)
#define RX_MSG_ID   (1UL)

extern void CAN_Initial(void);
extern void CAN_Tx_pro(void);

#endif
