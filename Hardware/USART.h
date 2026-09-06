#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendNumber(uint32_t Number, uint8_t Length);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif
