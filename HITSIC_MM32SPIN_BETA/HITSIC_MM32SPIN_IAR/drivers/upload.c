#include "upload.h"

#define NUM_VAR 6
#define UP_UART UART1//uart2����˫��ͨ��

float Variable[NUM_VAR];//���ͻ�������

//��λ��֡ͷ
void Send_Begin(void)
{
  UART_PutChar(UP_UART, 0x55);
  UART_PutChar(UP_UART, 0xaa);
  UART_PutChar(UP_UART, 0x11);
}

//�ϴ�����
void Send_Variable(void)//����ʵʱ����
{
  uint8_t i = 0,ch = 0;
  float temp = 0;
  Send_Begin();
  Variable[0] = icm1.rawdata.gx;			//�˴�����Ҫ���͵ı�����ֵ��Variable
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


//��ͼƬ