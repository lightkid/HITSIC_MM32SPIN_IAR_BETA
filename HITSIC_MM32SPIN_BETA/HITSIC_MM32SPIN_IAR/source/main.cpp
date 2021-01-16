
#include "MM32SPIN2xx_p.h"
#include "HAL_gpio.h"
#include "HAL_rcc.h"
#include "HAL_exti.h"
#include "HAL_misc.h"
#include "HAL_syscfg.h"
#include "HAL_tim.h"

#define BEEP_PORT       GPIOD
#define BEEP_PIN        GPIO_Pin_6
#define BEEP_PORT_RCC   RCC_AHBPeriph_GPIOD

#define KEY_PORT        GPIOC
#define KEY0            GPIO_Pin_3
#define KEY1            GPIO_Pin_2
#define KEY2            GPIO_Pin_1
#define KEY_PORT_RCC    RCC_AHBPeriph_GPIOC

void delay(int time)
{
  int i=0;
  int j=0;
  while(true)
  {
    i++;
    if(i>10000)
    {
      i=0;
      j++;
    }
    if(j>time)
    {
      break;
    }
  }
}

int count=0;

void HSE_SetSysClk(uint32_t RCC_PLLMul_x)
{
  ErrorStatus HSEStatus;
  RCC_DeInit();//RCCʱ�Ӹ�λ
  RCC_HSEConfig(RCC_HSE_ON);//�����ⲿ����
  HSEStatus = RCC_WaitForHSEStartUp();//�ȴ���ȡHSE����״̬
  //HSE�����ɹ�
  if(HSEStatus == SUCCESS)
  {
    //FLASH��ʱ
    //�������߷�Ƶ����
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB
    RCC_PCLK1Config(RCC_HCLK_Div2);//APB1
    RCC_PCLK2Config(RCC_HCLK_Div1);//APB2
    //PLL���໷������Դ�ͱ�Ƶ���ӣ�ʹ��PLL
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_x);
    RCC_PLLCmd(ENABLE);
    //�ȴ�PLL׼������
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    //ѡ��PLLCLK��Ϊϵͳʱ��
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    //�ȴ�ϵͳʱ���л��ɹ����ȶ�
    while (RCC_GetSYSCLKSource() != 0x08);//0x08����ϵͳʹ��PLLCLK

  }
  else
  {
    //����ʧ��
  }
}


int main(void)
{
  HSE_SetSysClk(RCC_PLLMul_12);//��ߵ�16
  //��дgpio
  GPIO_InitTypeDef GPIO_InitStructure1;//����һ���ṹ�������������ʼ��GPIO
  RCC_AHBPeriphClockCmd(BEEP_PORT_RCC, ENABLE);
  GPIO_InitStructure1.GPIO_Pin = BEEP_PIN;
  GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BEEP_PORT, &GPIO_InitStructure1);
  
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
  
  //��ʱ���ж�TIM1Ϊ����APB2��
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  TIM_TimeBaseInitStructure.TIM_Prescaler = 20000;//��ʱ������2s
  TIM_TimeBaseInitStructure.TIM_Period = 10000;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
  
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_UP_TRG_COM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority=3;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);//��Ӧ�жϣ��˴�Ϊ������������������ж�
  TIM_Cmd(TIM1, ENABLE);//������ʹ��
  
  while(true)
  {
    
    //GPIO
//      if(GPIO_ReadInputDataBit(KEY_PORT,KEY0)==0u)
//      {
//        GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_SET);
//        delay();
//        GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
//      }
      
      //delay(10000);
      
    
    //������ ICM20602/MPU6050
    //SCL/SPC		PD03
    //SDA/SDI		PD04
    //SAO/SDO		PD05//useless
    //CS		PD02//useless
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
  delay(100);
  GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);
}
#ifdef __cplusplus
}
#endif
