#ifndef _IIC_H_
#define _IIC_H_

#include "cpu.h"

#if 1
extern void IIC_Initial(void);
extern void Eeprom_Write(uint8_t *data, uint16_t addr, uint16_t len);
extern void Eeprom_Read(uint8_t *data, uint16_t addr, uint16_t len);
extern void Eeprom_Task(void);
#endif

#endif
