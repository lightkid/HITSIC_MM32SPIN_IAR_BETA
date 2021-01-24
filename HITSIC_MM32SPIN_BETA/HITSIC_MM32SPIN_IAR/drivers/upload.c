#include "upload.h"

#define NUM_VAR 6
#define UP_UART UART1//uart2用作双机通信

float Variable[NUM_VAR];//发送缓存数组

//上位机帧头
void Send_Begin(void)
{
  UART_PutChar(UP_UART, 0x55);
  UART_PutChar(UP_UART, 0xaa);
  UART_PutChar(UP_UART, 0x11);
}

//上传数据
void Send_Variable(void)//发送实时变量
{
  uint8_t i = 0,ch = 0;
  float temp = 0;
  Send_Begin();
  Variable[0] = icm1.rawdata.gx;			//此处将需要发送的变量赋值到Variable
  Variable[1] = icm1.rawdata.gy;
  Variable[2] = icm1.rawdata.gz;
  Variable[3] = icm1.rawdata.ax;
  Variable[4] = icm1.rawdata.ay;
  Variable[5] = icm1.rawdata.az;	
  UART_PutChar(UP_UART, 0x55);
  UART_PutChar(UP_UART, 0xaa);
  UART_PutChar(UP_UART, 0x11);
  UART_PutChar(UP_UART, 0x55);
  UART_PutChar(UP_UART, 0xaa);
  UART_PutChar(UP_UART, 0xff);
  UART_PutChar(UP_UART, 0x01);
  UART_PutChar(UP_UART, NUM_VAR);
  for(i = 0; i < NUM_VAR; i++)
  {
    temp = Variable[i];
    ch = BYTE0(temp);
    UART_PutChar(UP_UART, ch);
    ch = BYTE1(temp);
    UART_PutChar(UP_UART, ch);
    ch = BYTE2(temp);
    UART_PutChar(UP_UART, ch);
    ch = BYTE3(temp);
    UART_PutChar(UP_UART, ch);
  }
  UART_PutChar(UP_UART, 0x01);
}


//传图片