
#include "board.h"

#define BEEP_PORT       GPIOD
#define BEEP_PIN        GPIO_Pin_6
#define BEEP_PORT_RCC   RCC_AHBPeriph_GPIOD

#define KEY_PORT        GPIOC
#define KEY0            GPIO_Pin_3
#define KEY1            GPIO_Pin_2
#define KEY2            GPIO_Pin_1
#define KEY_PORT_RCC    RCC_AHBPeriph_GPIOC


int main(void)
{
  //HSE 96MHz MAX
  HSE_SetSysClk(RCC_PLLMul_12);
  delay_init();
  //��дgpio
  GPIO_InitTypeDef GPIO_InitStructure1;//����һ���ṹ�������������ʼ��GPIO
  RCC_AHBPeriphClockCmd(BEEP_PORT_RCC, ENABLE);
  GPIO_InitStructure1.GPIO_Pin = BEEP_PIN;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BEEP_PORT, &GPIO_InitStructure1);
  delay_ms(100);
  RCC_ClocksTypeDef SYS_Clock;
  RCC_GetClocksFreq(&SYS_Clock);
  
  MPU6050_Init();
  
  //ʹ��TIM14��Ϊ��ʱ����5ms��ȡһ��
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM14, ENABLE);
  
  TIM_TimeBaseInitStructure.TIM_Prescaler = 96;//1MHz
  TIM_TimeBaseInitStructure.TIM_Period = 5000;//5ms
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStructure);
  
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);//��Ӧ�жϣ��˴�Ϊ������������������ж�
  TIM_Cmd(TIM14, ENABLE);//������ʹ��
  
  
  
//  GPIO_InitTypeDef GPIO_InitStructure2;//����һ���ṹ�������������ʼ��GPIO
//  RCC_AHBPeriphClockCmd(KEY_PORT_RCC, ENABLE);
//  GPIO_InitStructure2.GPIO_Pin = KEY0;
//  GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IPU;
//  GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(KEY_PORT, &GPIO_InitStructure2);
//  //��ʼ��һ���ⲿ�жϣ�C1(KEY2)
//  GPIO_InitTypeDef GPIO_InitStructure;
//  EXTI_InitTypeDef EXTI_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStructure;
//  
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//��ʱ��
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
//  
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//����ֲ�Ҫ��
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//  
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);//ѡ��C1��Ϊ�ⲿ�ж���·
//  
//  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
//  
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;     //IRQn type����
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;        //���ȼ�
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	 //IRQͨ��ʹ��
//  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
  
//  //��ʱ���ж�TIM1Ϊ����APB2��
//  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//  TIM_TimeBaseInitStructure.TIM_Prescaler = 9600;//10000Hz
//  TIM_TimeBaseInitStructure.TIM_Period = 20000;//�ƶ��ٸ���
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
//  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);//��Ӧ�жϣ��˴�Ϊ������������������ж�
//  TIM_Cmd(TIM1, ENABLE);//������ʹ��
//  
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
      
      //delay(10000);
  }
  return 0;
}




#ifdef __cplusplus
extern "C"{
#endif
//void EXTI0_1_IRQHandler(void)//���ֿ�startup
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

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  //TIM_ClearFlag(TIM1, TIM_FLAG_Update);
  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
  delay_ms(100);
  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
}
float a=0;

//������
void TIM14_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
  inv_mpu6050_poll_sensor_data_reg(&icm1);
  a=icm1.rawdata.ax;
}

#ifdef __cplusplus
}
#endif
