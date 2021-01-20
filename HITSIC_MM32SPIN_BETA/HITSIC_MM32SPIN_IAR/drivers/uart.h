#ifndef __UART_H_
#define __UART_H_

#include "board.h"

void UART_PutChar(UART_TypeDef* UARTx,char ch);
void UART_PutStr(UART_TypeDef* UARTx,char *str);
void UART_PutNum(UART_TypeDef* UARTx, int num);
void UART_PutFloat(UART_TypeDef* UARTx,float num);
char UART_GetChar(UART_TypeDef* UARTx);
#endif