#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"

void UART_Init(void);
uint8_t UART_ReadLine(char *buffer, uint16_t buffer_size);
void UART_SendChar(char ch);
void UART_SendString(const char *text);
void UART_SendUInt32(uint32_t value);
void UART_SendInt32(int32_t value);
void UART_SendFixed100(int32_t value_x100);

#endif
