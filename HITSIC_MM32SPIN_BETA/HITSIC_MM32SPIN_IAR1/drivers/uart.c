#include "uart.h"

void UART_PutChar(UART_TypeDef* UARTx,char ch)
{
  UART_SendData(UARTx, ch);
  while (UART_GetFlagStatus(UARTx, UART_FLAG_TXEPT) == RESET);
}

void UART_PutStr(UART_TypeDef* UARTx,char *str)
{
  while(*str)
  {
    UART_PutChar(UARTx,*str++);
  }
}
void UART_PutNum(UART_TypeDef* UARTx, int num)
{
  char str[7];
  sprintf(str,"%d",num);
  UART_PutStr(UARTx,str);
}
void UART_PutFloat(UART_TypeDef* UARTx,float num)
{
  char str[10];
  snprintf(str, 10, "%.4f", num);
  UART_PutStr(UARTx,str);
}

char UART_GetChar(UART_TypeDef* UARTx)
{
  char ch;
  
  //while (UART_GetFlagStatus(UARTx, UART_FLAG_RXAVL) == RESET);
  UART_ClearFlag(UART1, UART_FLAG_RXAVL);
  ch = UART_ReceiveData(UARTx);
  return ch;
}

//char* UART_GetStr(UART_TypeDef* UARTx, int len)
//{
//  char str[10];
//  while(len--)
//  {
//    *str++ = UART_GetChar(UARTx);
//  }
//  return str;
//}
//int UART_GetNum(UART_TypeDef* UARTx)
//{
//  int num;
//  char str[7];
//  int len = 7;
//  while(len--)
//  {
//    *str++ = UART_GetChar(UARTx);
//  }
//  sprintf(str,"%d",num);
//  UART_GetChar(UARTx,str);
//}
//void UART_GetFloat(UART_TypeDef* UARTx,float num)
//{
//  char str[10];
//  snprintf(str, 10, "%.4f", num);
//  UART_GetChar(UARTx,str);
//}