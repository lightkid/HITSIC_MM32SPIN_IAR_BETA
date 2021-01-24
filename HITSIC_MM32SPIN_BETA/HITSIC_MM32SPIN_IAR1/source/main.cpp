
#include "board.h"

//核心2摄像头专用


int main(void)
{
  //HSE 96MHz MAX
  HSE_SetSysClk(RCC_PLLMul_12);
  delay_init();
  delay_ms(100);
//  RCC_ClocksTypeDef SYS_Clock;
//  RCC_GetClocksFreq(&SYS_Clock);
  
//  const uint32_t buff[10]={};
  //Flash_Page_Read(FLASH_SECTION_15, FLASH_PAGE_0, buff, 10);
//  uint8_t as = Flash_Page_Write (FLASH_SECTION_15, FLASH_PAGE_0, buff, 10);
  
  //uart2用来双机通讯 a2t  a3r
  GPIO_InitTypeDef GPIO_InitStructure;//声明一个结构体变量，用来初始化GPIO
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
  
  UART_InitTypeDef UART_InitStructure;
  
  //UART_StructInit(&UART_InitStructure);
  UART_InitStructure.UART_BaudRate=115200;//波特率
  UART_InitStructure.UART_WordLength = UART_WordLength_8b;
  UART_InitStructure.UART_StopBits = UART_StopBits_1;
  UART_InitStructure.UART_Parity = UART_Parity_No;
  UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  UART_Init(UART2, &UART_InitStructure);
  
//  UART_ITConfig(UART2, UART_IT_TXIEN, ENABLE);
//  UART_ITConfig(UART2, UART_IT_RXIEN, ENABLE);//两种中断
//  NVIC_InitTypeDef NVIC_InitStructure;
//  NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
  UART_Cmd(UART2, ENABLE);
  
  while(true)
  {

    //GPIO
//      if(GPIO_ReadInputDataBit(KEY_PORT,KEY0)==0u)
//      {
//        GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
//        delay(500);
//        GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
//      }
      
      //delay(10000);
    UART_PutChar(UART2,'L');                     //发送 字节到UART口
    delay_ms(500);
    delay_ms(500);
    delay_ms(500);
    delay_ms(500);
    delay_ms(500);
    delay_ms(500);
  }
  return 0;
}




#ifdef __cplusplus
extern "C"{
#endif



//uint16_t uartRxbuff = 0;
//
//void UART2_IRQHandler(void)
//{
//  //判断中断类型
//  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
//    delay_ms(100);
//    GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
//  if(UART_GetITStatus(UART2, UART_IT_RXIEN))
//  {
//    UART_ClearITPendingBit(UART2, UART_IT_RXIEN);
//    //读取缓冲区
//    //uartRxbuff = UART_ReceiveData(UART2);
//    char a = UART_GetChar(UART2);
//    OLED_P6x8Str(0,0,&a);
//    GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
//    delay_ms(100);
//    GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
//  }
////  else if(UART_GetITStatus(UART2, UART_IT_TXIEN))
////  {
////    UART_ClearITPendingBit(UART2, UART_IT_TXIEN);
////  }
//  else
//  {
//    //error
//  }
//}


#ifdef __cplusplus
}
#endif
