
#include "board.h"

//核心1

#define BEEP_PORT       GPIOD
#define BEEP_PIN        GPIO_Pin_6
#define BEEP_PORT_RCC   RCC_AHBPeriph_GPIOD

#define KEY_PORT        GPIOC
#define KEY0            GPIO_Pin_3
#define KEY1            GPIO_Pin_2
#define KEY2            GPIO_Pin_1
#define KEY_PORT_RCC    RCC_AHBPeriph_GPIOC

typedef struct value
{
  int a;
  float b;
  uint32_t c;
  uint8_t d;
}value_t;

value_t val;

float ax;

//void my_i2c_read_byte()//(uint8_t* data,uint8_t reg)
//{
//  int16_t val;
//  //发起始位
//  I2C_GenerateSTART(I2C1,ENABLE);
//  //发地址位
////  I2C_Send7bitAddress(I2C1, 0xD0, I2C_Direction_Receiver);//陀螺仪地址/接收方向
//  //发寄存器
//  I2C_SendData(I2C1, 0x3B);//加速度
//  //等中断
//  while(!I2C_GetITStatus(I2C1, I2C_IT_TX_EMPTY))
//  //接数
//  //接加速度ax高8位
//  while(!I2C_GetITStatus(I2C1, I2C_IT_RX_FULL))//rx满了就接
//  val = I2C_ReceiveData(I2C1);
//  while(!I2C_GetITStatus(I2C1, I2C_IT_RX_FULL))
//  //接加速度ax低8位
//  val<<8;
//  val |= I2C_ReceiveData(I2C1);
//  //现有ack/最后一个要有nack
//  
//  //发停止
//  I2C_GenerateSTOP(I2C1,ENABLE);
//  ax= (4.0 * 9.8 / 32768.0)*val;
//}

int main(void)
{
  
  //HSE 96MHz MAX
  HSE_SetSysClk(RCC_PLLMul_12);
  delay_init();
  //读写gpio
  GPIO_InitTypeDef GPIO_InitStructure1;//声明一个结构体变量，用来初始化GPIO
  RCC_AHBPeriphClockCmd(BEEP_PORT_RCC, ENABLE);
  GPIO_InitStructure1.GPIO_Pin = BEEP_PIN;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BEEP_PORT, &GPIO_InitStructure1);
  delay_ms(100);
//  RCC_ClocksTypeDef SYS_Clock;
//  RCC_GetClocksFreq(&SYS_Clock);
  
  MPU6050_Init();
  OLED_Init();
  delay_ms(1000);
//  GPIO_InitTypeDef GPIO_InitStructure;//声明一个结构体变量，用来初始化GPIO
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
//  GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
//  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_AF_OD;
//  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);
//  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1);
//  
//  I2C_InitTypeDef I2C_InitStructure;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
//  I2C_StructInit(&I2C_InitStructure);
//  I2C_Init(I2C1 ,&I2C_InitStructure);
//  I2C_ITConfig(I2C1, I2C_IT_TX_EMPTY, ENABLE);
//  I2C_ITConfig(I2C1, I2C_IT_RX_FULL, ENABLE);
//  I2C_Send7bitAddress(I2C1, 0xD0, I2C_Direction_Receiver);//陀螺仪地址/接收方向
//  
//  NVIC_InitTypeDef NVIC_InitStructure1;
//  NVIC_InitStructure1.NVIC_IRQChannel = I2C1_IRQn;
//  NVIC_InitStructure1.NVIC_IRQChannelPriority = 3;
//  NVIC_InitStructure1.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure1);
//  
//  I2C_Cmd(I2C1, ENABLE);
  memset(&val,0,sizeof(value_t));
//  val.a=2;
//  val.b=3.14;
//  val.c=6;
//  val.d=7;
  uint32_t buff[10]={0};
//  OLED_Print_Num(0,0,val.a);
//  OLED_Print_Float(0,1,val.b);
//  OLED_Print_Num(0,2,(int)val.c);
//  OLED_Print_Num(0,3,(int)val.d);
//  float var = 8.8;
//  uint32_t val=0;
//  memcpy(&val, &var, sizeof(float));
//  uint32_t buff[10]={0};//=0x40490e56;
//  memcpy(buff,&val,sizeof(value_t));
//  uint8_t as = Flash_Page_Write (FLASH_SECTION_15, FLASH_PAGE_0, buff, 10);
//    memset(&val,0,sizeof(value_t));
//    memset(buff,0,10);
////  const uint32_t buff=(((uint32_t)BYTE3(var)<<24)|((uint32_t)BYTE2(var)<<16)|((uint32_t)BYTE1(var)<<8)|BYTE0(var));// = (uint32_t)var;
//  
////  const uint32_t buff[10]={};
//  uint32_t buf=0;
  Flash_Page_Read(FLASH_SECTION_15, FLASH_PAGE_0, buff, 10);
  
//  
//  float var1 = 0;
  memcpy(&val, buff, sizeof(value_t));
  OLED_Print_Num(55,0,val.a);
   
  OLED_Print_Float(55,1,val.b);

  OLED_Print_Num(55,2,val.c);

  OLED_Print_Num(55,3,val.d);
//  OLED_Print_Float(0,0,var1);
//  for(uint8_t i=0;i<7;i++)
//  {
//    OLED_Print_Num(0,i,buff[i]);
//  }
  
  
////  使用TIM14作为定时器，5ms读取一次
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//    
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM14, ENABLE);
//  
//  TIM_TimeBaseInitStructure.TIM_Prescaler = 96;//1MHz
//  TIM_TimeBaseInitStructure.TIM_Period = 5000;//5ms
//  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
//  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStructure);
//  
//  NVIC_InitTypeDef NVIC_InitStructure;
//  NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//  
//  TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);//相应中断，此处为计数器溢出更新引起中断
//  TIM_Cmd(TIM14, ENABLE);//计数器使能
//  delay_ms(200);
//  OLED_CLS();
  
  //uart2用来双机通讯 a2t  a3r
//  GPIO_InitTypeDef GPIO_InitStructure;//声明一个结构体变量，用来初始化GPIO
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
//  
//  UART_InitTypeDef UART_InitStructure;
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART2, ENABLE);
////  UART_StructInit(&UART_InitStructure);
//  UART_InitStructure.UART_BaudRate=115200;//波特率
//  UART_InitStructure.UART_WordLength = UART_WordLength_8b;
//  UART_InitStructure.UART_StopBits = UART_StopBits_1;
//  UART_InitStructure.UART_Parity = UART_Parity_No;
//  UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
//  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
//  UART_Init(UART2, &UART_InitStructure);
//  
////  UART_ITConfig(UART2, UART_IT_TXIEN, ENABLE);
//  UART_ITConfig(UART2, UART_IT_RXIEN, ENABLE);//两种中断
//  NVIC_InitTypeDef NVIC_InitStructure;
//  NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//  UART_Cmd(UART2, ENABLE);
  //OLED_Logo();
  
  
//  //上位机初始化  a9tx a10rx uart1
//  GPIO_InitTypeDef GPIO_InitStructure;//声明一个结构体变量，用来初始化GPIO
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
//  
//  UART_InitTypeDef UART_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
////  UART_StructInit(&UART_InitStructure);
//  UART_InitStructure.UART_BaudRate=115200;//波特率
//  UART_InitStructure.UART_WordLength = UART_WordLength_8b;
//  UART_InitStructure.UART_StopBits = UART_StopBits_1;
//  UART_InitStructure.UART_Parity = UART_Parity_No;
//  UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
//  UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
//  UART_Init(UART1, &UART_InitStructure);
//  
////  UART_ITConfig(UART2, UART_IT_TXIEN, ENABLE);
////  UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);//两种中断
////  NVIC_InitTypeDef NVIC_InitStructure;
////  NVIC_InitStructure.NVIC_IRQChannel = UART2_IRQn;
////  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
////  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////  NVIC_Init(&NVIC_InitStructure);
//  UART_Cmd(UART1, ENABLE);
//  TIM_Cmd(TIM14, ENABLE);//计数器使能
  
//  GPIO_InitTypeDef GPIO_InitStructure2;//声明一个结构体变量，用来初始化GPIO
//  RCC_AHBPeriphClockCmd(KEY_PORT_RCC, ENABLE);
//  GPIO_InitStructure2.GPIO_Pin = KEY0;
//  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(KEY_PORT, &GPIO_InitStructure2);
//  //初始化一个外部中断，C1(KEY2)
//  GPIO_InitTypeDef GPIO_InitStructure;
//  EXTI_InitTypeDef EXTI_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//开时钟
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
//  
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//详见手册要求
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//  
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);//选择C1作为外部中断线路
//  
//  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
//  
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;     //IRQn type里找
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;        //优先级
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	 //IRQ通道使能
//  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器
  
//  //定时器中断TIM1为例在APB2上
//  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//  TIM_TimeBaseInitStructure.TIM_Prescaler = 9600;//10000Hz
//  TIM_TimeBaseInitStructure.TIM_Period = 20000;//计多少个数
//  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//  TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
//  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
//  
//  NVIC_InitTypeDef NVIC_InitStructure;
//  NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_UP_TRG_COM_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority=3;
//  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//  
//  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);//相应中断，此处为计数器溢出更新引起中断
//  TIM_Cmd(TIM1, ENABLE);//计数器使能
//  
  //PWM中央对齐
//  char str[]= "hello";
//  OLED_P6x8Str(0,0,str);
//  OLED_P6x8Str(0,1,"1.hello world!");
//  OLED_P6x8Str(0,2,"2.hello world!");
//  OLED_P6x8Str(0,3,"3.hello world!");
//  OLED_P6x8Str(0,4,"4.hello world!");
//  OLED_P6x8Str(0,5,"5.hello world!");
//  OLED_P6x8Str(0,6,"6.hello world!");
//  delay_ms(500);
//  delay_ms(500);
//  delay_ms(500);
//  delay_ms(500);
//  delay_ms(500);
//  delay_ms(500);
//  delay_ms(500);
//  delay_ms(500);
//  delay_ms(500);
//  delay_ms(500);
//  OLED_CLS();
//  OLED_Print_Float(0,0,1.11);
//  OLED_Print_Num(0,1,12);
//  OLED_P6x8Str(0,6,"              ");
 // OLED_P6x8Str(0,5,"hello world!");
  while(true)
  {
    //MPU6050_Get();
//    a=MPU6050->ax;
//        b=MPU6050->ay;
//        c=MPU6050->az;
//        d=MPU6050->gx;
//        e=MPU6050->gy;
//        f=MPU6050->gz;
    //GPIO
//      if(GPIO_ReadInputDataBit(KEY_PORT,KEY0)==0u)
//      {
//        GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
//        delay();
//        GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
//      }
//      Send_Variable();
      //delay(10000);
//    UART_PutChar(UART1,'L');                     //发送 字节到UART口
//    delay_ms(50);
//    my_i2c_read_byte();
//    OLED_Print_Float(0,0,ax);
  }
  return 0;
}




#ifdef __cplusplus
extern "C"{
#endif
//void EXTI0_1_IRQHandler(void)//名字看startup
//{
////  if(count==0)
////  {
////    GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
////    count=1;
////  }
////  else
////  {
////    GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
////    count=0;
////  }
//  EXTI_ClearFlag(EXTI_Line1);
//  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
//  delay(1000);
//  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
//}
//
//void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
//{
//  //TIM_ClearFlag(TIM1, TIM_FLAG_Update);
//  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
//  delay_ms(100);
//  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
//}
//float a=0;
//int count_tim14=0;
//陀螺仪
void TIM14_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
  inv_mpu6050_poll_sensor_data_reg(&icm1);
//  a=icm1.rawdata.gx;
//  
//  if((count_tim14++) == 100)//让显示的频率低一点
//  {
//    OLED_P6x8Str(0,0,"ax:");
//    OLED_Print_Float(19,0,icm1.rawdata.ax);
//    OLED_P6x8Str(0,1,"ay:");
//    OLED_Print_Float(19,1,icm1.rawdata.ay);
//    OLED_P6x8Str(0,2,"az:");
//    OLED_Print_Float(19,2,icm1.rawdata.az);
//    OLED_P6x8Str(0,3,"gx:");
//    OLED_Print_Float(19,3,icm1.rawdata.gx);
//    OLED_P6x8Str(0,4,"gy:");
//    OLED_Print_Float(19,4,icm1.rawdata.gy);
//    OLED_P6x8Str(0,5,"gz:");
//    OLED_Print_Float(19,5,icm1.rawdata.gz);
//    count_tim14=0;
//  }
}


uint16_t uartRxbuff = 0;

void UART2_IRQHandler(void)
{
  //判断中断类型
  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
    delay_ms(100);
    GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
  if(UART_GetITStatus(UART2, UART_IT_RXIEN))
  {
    UART_ClearITPendingBit(UART2, UART_IT_RXIEN);
    //读取缓冲区
    //uartRxbuff = UART_ReceiveData(UART2);
    char a = UART_GetChar(UART2);
    OLED_P6x8Str(0,0,&a);
    GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
    delay_ms(100);
    GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
  }
//  else if(UART_GetITStatus(UART2, UART_IT_TXIEN))
//  {
//    UART_ClearITPendingBit(UART2, UART_IT_TXIEN);
//  }
  else
  {
    //error
  }
}


#ifdef __cplusplus
}
#endif
