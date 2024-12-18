#include "IIC.h"
#include <string.h>

#if 1
#include "lpi2c1.h"

#define EE_DATA_LEN   (480)

/* Define receive buffer */
extern flexcan_msgbuff_t recvBuff;

void Eeprom_Write(uint8_t *data, uint16_t addr, uint16_t len);
void Eeprom_Read(uint8_t *data, uint16_t addr, uint16_t len);


void IIC_Initial(void)
{
	int i = 0;
    /* Allocate memory for the LPI2C driver state structure */
    lpi2c_master_state_t lpi2c1MasterState;

    //uint8_t txbuff[] = {0x00, 0x55, 0xaa, 0x55, 0xaa}; /* 0x00：块1的偏移地址为0 后两个才为写入数据 */
    //uint8_t rx_buff[4] = {0};

	//uint32_t CpuID[4];
	//uint32_t Lock_Code;

    LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1MasterState);

    for(i = 0; i < 1000; i ++){}

#if 0

    /* Send a packet of data to the bus slave */
    LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, txbuff, 5, true, OSIF_WAIT_FOREVER); /* 写入数据 */

    for(i = 0; i < 1000000; i ++){}

    //delay_ms(5); /* 等待完全写入数据 */

    LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, txbuff, 1, false, OSIF_WAIT_FOREVER); /* 写入待读取数据地址 */

    LPI2C_DRV_MasterReceiveDataBlocking(INST_LPI2C1, rx_buff, 4, true, OSIF_WAIT_FOREVER); /* 读取数据 */

    SEGGER_RTT_printf(0, "LPI2C1_Initial = 0x%x - 0x%x - 0x%x - 0x%x\r\n",rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3]);
#endif
    for(i = 0; i < 1000; i ++){}

    //txbuff[0] = 0x11;
    //txbuff[1] = 0x22;
    //txbuff[2] = 0x33;
    //txbuff[3] = 0x44;

    //Eeprom_Write(txbuff, 0, EE_DATA_LEN);

    //Eeprom_Read(rx_buff, 0, EE_DATA_LEN);

    //SEGGER_RTT_printf(0, "LPI2C1_Initial = 0x%x - 0x%x - 0x%x - 0x%x\r\n",rx_buff[0], rx_buff[1], rx_buff[2], rx_buff[3]);


}

#define EE_page				16

void Eeprom_Write(uint8_t *pbuff, uint16_t addr, uint16_t len)
{
  int i = 0;
  int p = 0;

  uint8_t h_byte = 0;
  uint8_t l_byte = 0;

  uint8_t b[EE_page + 1];

  uint8_t page_all = 0;
  uint8_t page_all_last = 0;

  //if(addr + len > (512 - 16)) return;

  if((addr + len) >= 255) h_byte = 0x51;
  else h_byte = 0x50;

  //h_byte = DEV_ADDR | (((uint8_t)((addr >> 8) & 0x07)) << 1);

  //LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C1, h_byte, false);

  l_byte = (uint8_t)((addr) & 0xFF);

  //SEGGER_RTT_printf(0, "h_byte = 0x%x l_byte = 0x%x\r\n",h_byte, l_byte);

  page_all = len/EE_page;
  page_all_last = len%EE_page;
  //SEGGER_RTT_printf(0, "page_all = 0x%x page_all_last = 0x%x\r\n",page_all, page_all_last);

  if(page_all != 0)
  {
    for(p = 0; p < page_all; p ++)
    {
      b[0] = l_byte + (p*EE_page);

	  //SEGGER_RTT_printf(0, "w1 addr = %d h_byte = %d b[0] = %d len = %d\r\n",addr, h_byte, b[0],EE_page);

      for(i = 0; i < EE_page; i ++)
      {
    	b[i + 1] = pbuff[p*EE_page +i];
      }


      LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C1, h_byte, false);

      //LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, &b[0], 1, false, 100);
      for(i = 0; i < 1000; i ++){}

	  /* Send a packet of data to the bus slave */
	  if( STATUS_SUCCESS == LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, &b[0], EE_page + 1, true, 100)) /* 写入数据 */
	  {
		  lpi2c_master_state_t lpi2c1MasterState;
		  for(i = 0; i < 10000; i ++){}
		  LPI2C_DRV_MasterAbortTransferData(INST_LPI2C1);
		  LPI2C_DRV_MasterDeinit(INST_LPI2C1);
		  LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1MasterState);
		  for(i = 0; i < 10000; i ++){}
		  for(i = 0; i < 10000; i ++){}
		  //SEGGER_RTT_printf(0, "page_all ee write STATUS_SUCCESS ok\r\n");
	      //SEGGER_RTT_printf(0, "page_all p = %d b[0] = %d p*EE_page + 1 = %d\r\n",p, b[0], (p*EE_page + 1));

	  }
	  else
	  {
		  //SEGGER_RTT_printf(0, "page_all ee write STATUS_SUCCESS not ok\r\n");

	  }
    }
  }

  if(page_all_last != 0)
  {
      b[0] = l_byte + (p*EE_page);

	  //SEGGER_RTT_printf(0, "w2 addr = %d h_byte = %d b[0] = %d len = %d\r\n",addr, h_byte, b[0], page_all_last);

      for(i = 0; i < page_all_last; i ++)
      {
    	b[i+1] = pbuff[p*EE_page +i];
      }

      LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C1, h_byte, false);
	  for(i = 0; i < 1000; i ++){}

	  /* Send a packet of data to the bus slave */
	  if( STATUS_SUCCESS == LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, &b[0], page_all_last + 1, true, 100)) /* 写入数据 */
	  {
		  lpi2c_master_state_t lpi2c1MasterState;
		  for(i = 0; i < 10000; i ++){}
		  LPI2C_DRV_MasterAbortTransferData(INST_LPI2C1);
		  LPI2C_DRV_MasterDeinit(INST_LPI2C1);
		  LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1MasterState);
		  for(i = 0; i < 10000; i ++){}
		  for(i = 0; i < 10000; i ++){}
		  //SEGGER_RTT_printf(0, "page_all_last p = %d b[0] = %d page_all_last + 1 = %d\r\n",p, b[0], (page_all_last + 1));
	  }
	  else
	  {
		  //SEGGER_RTT_printf(0, "page_all_last ee write STATUS_SUCCESS not ok\r\n");

	  }
  }

}
#define READ_CMD				1
#define WRITE_CMD				0

#define DEV_ADDR				0x50					//设备硬件地址


void Eeprom_Read(uint8_t *pbuff, uint16_t addr, uint16_t len)
{
  int i  = 0;
  int p  = 0;

  uint8_t h_byte = 0;
  uint8_t l_byte = 0;
  uint8_t b[EE_page + 1];

  uint8_t page_all = 0;
  uint8_t page_all_last = 0;

  //if(addr + len > (512 - 16)) return;

  if((addr + len) >= 255) h_byte = 0x51;
  else h_byte = 0x50;

  //h_byte = DEV_ADDR | (((uint8_t)((addr >> 8) & 0x07)) << 1);

  l_byte = (uint8_t)((addr) & 0xFF);

  //SEGGER_RTT_printf(0, "h_byte = 0x%x l_byte = 0x%x\r\n",h_byte, l_byte);

  page_all = len/EE_page;
  page_all_last = len%EE_page;
  //SEGGER_RTT_printf(0, "page_all = 0x%x page_all_last = 0x%x\r\n",page_all, page_all_last);

  if(page_all != 0)
  {
    for(p = 0; p < page_all; p ++)
    {
        b[0] = l_byte + (p*EE_page);

		//SEGGER_RTT_printf(0, "r1 addr = %d h_byte = %d b[0] = %d len = %d\r\n",addr, h_byte, b[0], EE_page);

        LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C1, h_byte, false);

        LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, &b[0], 1, false, 100); /* 写入待读取数据地址 */
        for(i = 0; i < 1000; i ++){}
		for(i = 0; i < 1000; i ++){}

	    if( STATUS_SUCCESS == LPI2C_DRV_MasterReceiveDataBlocking(INST_LPI2C1, &pbuff[p*EE_page], EE_page, true, 100)) /* 写入数据 */
	    {
		  lpi2c_master_state_t lpi2c1MasterState;
		  for(i = 0; i < 10000; i ++){}
		  LPI2C_DRV_MasterAbortTransferData(INST_LPI2C1);
		  LPI2C_DRV_MasterDeinit(INST_LPI2C1);
		  LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1MasterState);
		  for(i = 0; i < 10000; i ++){}
		  for(i = 0; i < 10000; i ++){}
	      //SEGGER_RTT_printf(0, "page_all ee read STATUS_SUCCESS ok\r\n");
	      //SEGGER_RTT_printf(0, "page_all p = %d b[0] = %d p*EE_page = %d\r\n",p, b[0], (p*EE_page));

	    }
	    else
	    {
	      //SEGGER_RTT_printf(0, "page_all ee read STATUS_SUCCESS not ok\r\n");

	    }
    }
  }

  if(page_all_last != 0)
  {
    b[0] = l_byte + (p*EE_page);
	
	//SEGGER_RTT_printf(0, "r2 addr = %d h_byte = %d b[0] = %d len = %d\r\n",addr, h_byte, b[0],page_all_last);

    LPI2C_DRV_MasterSetSlaveAddr(INST_LPI2C1, h_byte, false);

    LPI2C_DRV_MasterSendDataBlocking(INST_LPI2C1, &b[0], 1, false, 100); /* 写入待读取数据地址 */
    for(i = 0; i < 1000; i ++){}
	for(i = 0; i < 1000; i ++){}

	if( STATUS_SUCCESS == LPI2C_DRV_MasterReceiveDataBlocking(INST_LPI2C1, &pbuff[p*EE_page], page_all_last, true, 100)) /* 写入数据 */
	{
		  lpi2c_master_state_t lpi2c1MasterState;
		  for(i = 0; i < 10000; i ++){}
		  LPI2C_DRV_MasterAbortTransferData(INST_LPI2C1);
		  LPI2C_DRV_MasterDeinit(INST_LPI2C1);
		  LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1MasterState);
		  for(i = 0; i < 10000; i ++){}
		  for(i = 0; i < 10000; i ++){}
	      //SEGGER_RTT_printf(0, "page_all ee read STATUS_SUCCESS ok\r\n");
	      //SEGGER_RTT_printf(0, "page_all_last p = %d b[0] = %d page_all_last = %d\r\n",p, b[0], page_all_last);

	}
	else
	{
	  //SEGGER_RTT_printf(0, "page_all_last ee read STATUS_SUCCESS not ok\r\n");

	}
  }

}

static uint8_t buf_1[EE_DATA_LEN];

void Eeprom_Task(void)
{
	int i = 0;
	int j = 0;

	uint8_t cmd = 0;
	static uint8_t precmd = 0;

	cmd = recvBuff.data[2];

	if(cmd != precmd)
	{
		  lpi2c_master_state_t lpi2c1MasterState;
		  LPI2C_DRV_MasterAbortTransferData(INST_LPI2C1);
		  LPI2C_DRV_MasterDeinit(INST_LPI2C1);
		  LPI2C_DRV_MasterInit(INST_LPI2C1, &lpi2c1_MasterConfig0, &lpi2c1MasterState);
		  for(i = 0; i < 10000; i ++){}
		switch(cmd)
		{
		case 0x01:

			for(i = 0; i < EE_DATA_LEN; i ++)
			{
				buf_1[i]= i + 1;
			}

            Eeprom_Write(buf_1, 5, EE_DATA_LEN);

            for(i = 0; i < EE_DATA_LEN; i ++)
            {
              //SEGGER_RTT_printf(0, "%d 0x%x\r\n",i,buf_1[i]);
              for(j = 0; j < 200; j ++){};
            }

            break;
		case 0x02:

			memset(buf_1,0,EE_DATA_LEN);

            Eeprom_Read(buf_1, 5, EE_DATA_LEN);

            for(i = 0; i < EE_DATA_LEN; i ++)
            {
              //SEGGER_RTT_printf(0, "%d  0x%x\r\n",i,buf_1[i]);
              for(j = 0; j < 200; j ++){};
            }

			break;

		case 0x03:
			for(i = 0; i < EE_DATA_LEN; i ++)
			{
				buf_1[i]= i;
			}

            Eeprom_Write(buf_1, 0, EE_DATA_LEN);

            for(i = 0; i < EE_DATA_LEN; i ++)
            {
              //SEGGER_RTT_printf(0, "%d  0x%x\r\n",i,buf_1[i]);
              for(j = 0; j < 200; j ++){};
            }
            break;
		case 0x04:
			memset(buf_1,0,EE_DATA_LEN);

            Eeprom_Read(buf_1, 0, EE_DATA_LEN);

            for(i = 0; i < EE_DATA_LEN; i ++)
            {
              //SEGGER_RTT_printf(0, "%d  0x%x\r\n",i,buf_1[i]);
              for(j = 0; j < 200; j ++){};
            }

			break;

		}
	}
	precmd = cmd;
}

#endif

